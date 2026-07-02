// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// Thin regbus wrapper for the MoE scheduler datapath.
//
// The SoC side is expected to use:
//   soc_narrow_xbar.out_moe_scheduler -> atomic filter -> axi_to_reg -> this module
//
// Register map, 64-bit word addressed:
//   0x00 CTRL      W: bit0 init, bit1 start
//   0x08 STATUS    R: bit0 busy, bit1 done_sticky, bit2 remove_valid,
//                     bit3 plan_valid, bit4 plan_queue_full,
//                     bit5 active_empty, bit6 refill_req,
//                     bits[9:8] remove_count, bits[17:16] plan_count,
//                     bits[25:24] plan_slot_valid,
//                     bits[31:28] refill_count, bits[35:32] reserve_count
//   0x10 CONFIG    R/W: bits[7:0] cache_eid_c2, bits[15:8] cache_eid_c3,
//                       bits[16 +: NR_W] active_count, bits[32 +: T_W] total_conc
//   0x68 ROUND_COMMIT W: bit0 plan_pop
//   0x80 PLAN_FIFO_STATUS R: bit0 empty, bit1 full, bits[5:2] queue_count,
//                            bits[9:8] head_plan_count,
//                            bits[13:12] head_remove_count,
//                            bits[17:16] head_slot_valid
//   0x90 PLAN_FIFO_DATA0  R: FIFO head plan slot 0, including RTL local_slot
//   0x98 PLAN_FIFO_DATA1  R: FIFO head plan slot 1, including RTL local_slot
//   0xa0 PLAN_FIFO_PATCH  R: two compact S4PF patch records for FIFO head
//   0xa8 HEAD_PAIR0       W: compact head16 pair for top4 slot 0/1
//   0xb0 HEAD_PAIR1       W: compact head16 pair for top4 slot 2/3
//   0xb8 HEAD_PUSH_PAIR   W: compact head16 pair appended to reserve[4].
//   0xc0 RESERVE_PAIR0    W: compact head16 pair for reserve slot 0/1
//   0xc8 RESERVE_PAIR1    W: compact head16 pair for reserve slot 2/3
//                            head16 layout: ntok, eid, valid.

typedef struct packed {
  logic [47:0] addr;
  logic        write;
  logic [63:0] wdata;
  logic [7:0]  wstrb;
  logic        valid;
} moe_scheduler_default_reg_req_t;

typedef struct packed {
  logic [63:0] rdata;
  logic        error;
  logic        ready;
} moe_scheduler_default_reg_rsp_t;

module moe_scheduler_reg_wrapper
  import sched_pkg::*;
#(
  parameter type reg_req_t = moe_scheduler_default_reg_req_t,
  parameter type reg_rsp_t = moe_scheduler_default_reg_rsp_t
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  input  reg_req_t reg_req_i,
  output reg_rsp_t reg_rsp_o
);

  localparam logic [4:0] REG_CTRL             = 5'h00;
  localparam logic [4:0] REG_STATUS           = 5'h01;
  localparam logic [4:0] REG_CONFIG           = 5'h02;
  localparam logic [4:0] REG_ROUND_COMMIT     = 5'h0d;
  localparam logic [4:0] REG_PLAN_FIFO_STATUS = 5'h10;
  localparam logic [4:0] REG_PLAN_FIFO_DATA0  = 5'h12;
  localparam logic [4:0] REG_PLAN_FIFO_DATA1  = 5'h13;
  localparam logic [4:0] REG_PLAN_FIFO_PATCH  = 5'h14;
  localparam logic [4:0] REG_HEAD_PAIR0       = 5'h15;
  localparam logic [4:0] REG_HEAD_PAIR1       = 5'h16;
  localparam logic [4:0] REG_HEAD_PUSH_PAIR   = 5'h17;
  localparam logic [4:0] REG_RESERVE_PAIR0    = 5'h18;
  localparam logic [4:0] REG_RESERVE_PAIR1    = 5'h19;

  localparam int unsigned TASK_BITS = $bits(task_desc_t);
  localparam int unsigned PLAN_ALLOW_S4PF_LSB = TASK_BITS;
  localparam int unsigned PLAN_LOCAL_SLOT_LSB = PLAN_ALLOW_S4PF_LSB + 1;
  localparam int unsigned PLAN_SKIP_S2_LSB    = PLAN_LOCAL_SLOT_LSB + SLOT_W;
  localparam int unsigned PLAN_SKIP_S4_LSB    = PLAN_SKIP_S2_LSB + 1;
  localparam int unsigned PLAN_DMA_S1_LSB     = PLAN_SKIP_S4_LSB + 1;
  localparam int unsigned PLAN_DMA_S3_LSB     = PLAN_DMA_S1_LSB + 2;
  localparam int unsigned PLAN_M_S2_LSB       = PLAN_DMA_S3_LSB + 2;
  localparam int unsigned PLAN_M_S4_LSB       = PLAN_M_S2_LSB + NTOK_W;
  localparam int unsigned S4PF_PATCH_VALID_LSB      = 0;
  localparam int unsigned S4PF_PATCH_NO_COPY_LSB    = 1;
  localparam int unsigned S4PF_PATCH_CLUSTER_LSB    = 2;
  localparam int unsigned S4PF_PATCH_LOCAL_SLOT_LSB = 3;
  localparam int unsigned S4PF_PATCH_TARGET_EID_LSB = S4PF_PATCH_LOCAL_SLOT_LSB + SLOT_W;
  localparam int unsigned HEAD16_NTOK_LSB  = 0;
  localparam int unsigned HEAD16_EID_LSB   = HEAD16_NTOK_LSB + NTOK_W;
  localparam int unsigned HEAD16_VALID_LSB = HEAD16_EID_LSB + EID_RAW_W;

  logic [4:0]  word_addr;
  logic        write_req;
  logic [63:0] wr_data;
  logic [63:0] rd_data;
  logic        addr_hit;

  logic init_pulse;
  logic start_pulse;
  logic remove_ready_pulse;
  logic plan_pop_pulse;
  logic auto_run_q;
  logic ctrl_init_pulse;
  logic ctrl_start_pulse;
  logic round_commit_req;
  logic head_push_req;
  logic commit_plan_pop;

  logic [7:0]      cache_eid_c2_q;
  logic [7:0]      cache_eid_c3_q;
  head_ctx_t [3:0] head_q;
  head_ctx_t [3:0] reserve_q;
  head_ctx_t [3:0] compact_head;
  head_ctx_t [3:0] commit_head_next;
  head_ctx_t [1:0] push_head;
  logic [NR_W-1:0] active_count_q;
  logic [T_W-1:0]  total_conc_q;
  logic [T_W+1:0]  removed_conc;
  logic [3:0]      keep_mask;
  logic [1:0]      keep_target_idx [3:0];
  logic [2:0]      keep_count;
  logic [2:0]      head_valid_count;
  logic [2:0]      reserve_count;
  logic [2:0]      reserve_pop_count;
  logic [2:0]      target_head_count;
  logic [2:0]      fill_need;
  logic [NR_W-1:0] active_after_remove;
  logic            head_complete_current;
  logic            head_complete_after_remove;
  logic            auto_remove_pulse;
  logic            auto_start_after_remove;
  logic            auto_resume_pulse;
  logic            refill_req;
  logic [3:0]      refill_count;
  logic [1:0]      s4pf_pending_valid_q;
  logic [1:0]      s4pf_pending_valid_d;
  logic [1:0]      s4pf_pending_skip_s1_q;
  logic [1:0]      s4pf_pending_skip_s1_d;
  slot_id_t [1:0]  s4pf_pending_slot_q;
  slot_id_t [1:0]  s4pf_pending_slot_d;
  logic [63:0]     plan_patch_word;

  logic                         remove_valid;
  logic [1:0]                   remove_count;
  logic [3:0]                   remove_slot_mask;
  logic                         plan_valid;
  logic [1:0]                   plan_slot_valid;
  task_desc_t [1:0]             plan_task_desc;
  logic [1:0]                   plan_allow_s4pf;
  slot_id_t [1:0]               plan_local_slot;
  logic [1:0]                   plan_count;
  logic [1:0]                   plan_remove_count;
  logic [3:0]                   plan_fifo_count;
  logic                         plan_queue_full;
  logic                         busy;
  logic                         done;
  logic                         done_seen_q;

  function automatic head_ctx_t unpack_head16(input logic [15:0] word);
    head_ctx_t ctx;
    begin
      ctx = '0;
      ctx.ntok  = word[HEAD16_NTOK_LSB +: NTOK_W];
      ctx.eid   = word[HEAD16_EID_LSB +: EID_RAW_W];
      ctx.valid = word[HEAD16_VALID_LSB];
      unpack_head16 = ctx;
    end
  endfunction

  /*
   * 64-bit plan word layout:
	 *   bits [31:0]  compact task_desc_t
	 *   bit  [32]    allow_s4pf
	 *   bits [38:33] local_slot assigned by RTL per physical cluster
	 *   bit  [39]    skip_s2, precomputed from task_desc_t
	 *   bit  [40]    skip_s4, precomputed from task_desc_t
	 *   bits [42:41] dma_s1_for_shape
	 *   bits [44:43] dma_s3_for_shape
	 *   bits [53:45] m_s2_exec
	 *   bits [62:54] m_s4_exec
	 *   bit  [63]    reserved
	 */
  function automatic logic [63:0] pack_task(
    input task_desc_t task_desc,
    input logic allow_s4pf,
    input slot_id_t local_slot
  );
    logic [63:0] word;
    task_lower_scalar_t scalar;
    begin
      word = '0;
      scalar = lower_scalar_from_task(task_desc);
      word[TASK_BITS-1:0]                      = task_desc;
      word[PLAN_ALLOW_S4PF_LSB]                = allow_s4pf;
      word[PLAN_LOCAL_SLOT_LSB +: SLOT_W]      = local_slot;
      word[PLAN_SKIP_S2_LSB]                   = scalar.skip_s2;
      word[PLAN_SKIP_S4_LSB]                   = scalar.skip_s4;
      word[PLAN_DMA_S1_LSB +: 2]               = scalar.dma_s1_for_shape;
      word[PLAN_DMA_S3_LSB +: 2]               = scalar.dma_s3_for_shape;
      word[PLAN_M_S2_LSB +: NTOK_W]            = scalar.m_s2_exec;
      word[PLAN_M_S4_LSB +: NTOK_W]            = scalar.m_s4_exec;
      pack_task = word;
    end
  endfunction

  function automatic logic [31:0] pack_s4pf_patch(
    input logic valid,
    input logic no_copy,
    input logic cluster,
    input slot_id_t local_slot,
    input logic [EID_RAW_W-1:0] target_eid
  );
    logic [31:0] word;
    begin
      word = '0;
      word[S4PF_PATCH_VALID_LSB] = valid;
      word[S4PF_PATCH_NO_COPY_LSB] = no_copy;
      word[S4PF_PATCH_CLUSTER_LSB] = cluster;
      word[S4PF_PATCH_LOCAL_SLOT_LSB +: SLOT_W] = local_slot;
      word[S4PF_PATCH_TARGET_EID_LSB +: EID_RAW_W] = target_eid;
      pack_s4pf_patch = word;
    end
  endfunction

  assign word_addr          = reg_req_i.addr[7:3];
  assign write_req          = reg_req_i.valid && reg_req_i.write;
  assign wr_data            = reg_req_i.wdata[63:0];
  assign ctrl_init_pulse    = write_req && (word_addr == REG_CTRL) && wr_data[0];
  assign ctrl_start_pulse   = write_req && (word_addr == REG_CTRL) && wr_data[1];
  assign round_commit_req   = write_req && (word_addr == REG_ROUND_COMMIT);
  assign head_push_req      = write_req && (word_addr == REG_HEAD_PUSH_PAIR);
  assign commit_plan_pop    = round_commit_req && wr_data[0];

  assign init_pulse         = ctrl_init_pulse;
  assign start_pulse        = ctrl_start_pulse || auto_start_after_remove ||
                              auto_resume_pulse;
  assign remove_ready_pulse = auto_remove_pulse;
  assign plan_pop_pulse     = commit_plan_pop;
  assign push_head[0]       = unpack_head16(wr_data[15:0]);
  assign push_head[1]       = unpack_head16(wr_data[31:16]);

  always_comb begin
    logic [1:0]     walk_valid;
    logic [1:0]     walk_skip_s1;
    slot_id_t [1:0] walk_slot;

    plan_patch_word = '0;
    walk_valid = s4pf_pending_valid_q;
    walk_skip_s1 = s4pf_pending_skip_s1_q;
    walk_slot = s4pf_pending_slot_q;

    for (int s = 0; s < 2; s++) begin
      if (plan_valid && (s < int'(plan_count)) && plan_slot_valid[s]) begin
        int ci;
        logic cache_hit;
        logic no_copy;

        ci = int'(plan_task_desc[s].cluster);
        cache_hit = (ci == 0) ?
                    (!cache_eid_c2_q[7] &&
                     (cache_eid_c2_q[EID_RAW_W-1:0] == plan_task_desc[s].eid)) :
                    (!cache_eid_c3_q[7] &&
                     (cache_eid_c3_q[EID_RAW_W-1:0] == plan_task_desc[s].eid));
        no_copy = walk_skip_s1[ci] && cache_hit;

        if (walk_valid[ci]) begin
          plan_patch_word[s * 32 +: 32] =
              pack_s4pf_patch(1'b1, no_copy, plan_task_desc[s].cluster,
                              walk_slot[ci], plan_task_desc[s].eid);
          walk_valid[ci] = 1'b0;
          walk_skip_s1[ci] = 1'b0;
          walk_slot[ci] = '0;
        end

        if (plan_allow_s4pf[s]) begin
          walk_valid[ci] = 1'b1;
          walk_skip_s1[ci] = plan_task_desc[s].skip_s1;
          walk_slot[ci] = plan_local_slot[s];
        end
      end
    end

    s4pf_pending_valid_d = s4pf_pending_valid_q;
    s4pf_pending_skip_s1_d = s4pf_pending_skip_s1_q;
    s4pf_pending_slot_d = s4pf_pending_slot_q;
    if (plan_pop_pulse && plan_valid) begin
      s4pf_pending_valid_d = walk_valid;
      s4pf_pending_skip_s1_d = walk_skip_s1;
      s4pf_pending_slot_d = walk_slot;
    end
    if (init_pulse || ((active_count_q == NR_W'(0)) && !plan_valid)) begin
      s4pf_pending_valid_d = '0;
      s4pf_pending_skip_s1_d = '0;
      s4pf_pending_slot_d = '{default: '0};
    end
  end

  // Prefix/rank compact for the 4-entry top window plus reserve refill.
  //
  // keep_mask[i]=1 means HEADi survives into the next round.  target_idx is
  // the exclusive prefix sum of keep_mask.  Consumed entries have keep_mask=0,
  // so they do not get a new position.  Auto-run applies compact first and then
  // pops reserve[0..] into the top window tail.
  always_comb begin
    compact_head = '{default: '0};
    commit_head_next = head_q;
    removed_conc = '0;
    keep_count = '0;
    head_valid_count = '0;
    reserve_count = '0;
    reserve_pop_count = '0;
    active_after_remove = active_count_q;
    target_head_count = '0;
    fill_need = '0;
    refill_req = 1'b0;
    refill_count = '0;
    head_complete_current = 1'b0;
    head_complete_after_remove = 1'b0;

    for (int i = 0; i < 4; i++) begin
      if (head_q[i].valid) begin
        head_valid_count = head_valid_count + 3'd1;
      end
      if (reserve_q[i].valid) begin
        reserve_count = reserve_count + 3'd1;
      end
      keep_mask[i] = head_q[i].valid && !remove_slot_mask[i];
      if (head_q[i].valid && remove_slot_mask[i]) begin
        removed_conc = removed_conc + {2'b0, best_conc_t(head_q[i].ntok)};
      end
    end

    keep_target_idx[0] = 2'd0;
    keep_target_idx[1] = {1'b0, keep_mask[0]};
    keep_target_idx[2] = {1'b0, keep_mask[0]} + {1'b0, keep_mask[1]};
    keep_target_idx[3] = {1'b0, keep_mask[0]} + {1'b0, keep_mask[1]} +
                         {1'b0, keep_mask[2]};
    for (int i = 0; i < 4; i++) begin
      if (keep_mask[i]) begin
        compact_head[keep_target_idx[i]] = head_q[i];
        keep_count = keep_count + 3'd1;
      end
    end

    commit_head_next = remove_ready_pulse ? compact_head : head_q;

    if (remove_ready_pulse) begin
      active_after_remove = active_count_q - NR_W'(remove_count);
      target_head_count = (active_after_remove > NR_W'(4)) ? 3'd4 :
                          active_after_remove[2:0];
      fill_need = (target_head_count > keep_count) ?
                  (target_head_count - keep_count) : 3'd0;
      reserve_pop_count = (fill_need > reserve_count) ? reserve_count : fill_need;
      for (int p = 0; p < 4; p++) begin
        if (p < int'(reserve_pop_count)) begin
          commit_head_next[keep_count + p[2:0]] = reserve_q[p];
        end
      end
    end else begin
      active_after_remove = active_count_q;
      target_head_count = (active_count_q > NR_W'(4)) ? 3'd4 : active_count_q[2:0];
    end

    head_complete_current = (head_valid_count >= target_head_count);
    head_complete_after_remove =
        (remove_ready_pulse ? ((keep_count + reserve_pop_count) >= target_head_count) :
                              head_complete_current);

    if (active_count_q > NR_W'(head_valid_count + reserve_count)) begin
      logic [2:0] free_slots;
      logic [NR_W-1:0] remaining_unloaded;
      free_slots = 3'd4 - reserve_count;
      remaining_unloaded = active_count_q - NR_W'(head_valid_count + reserve_count);
      if (free_slots != 3'd0) begin
        refill_req = 1'b1;
        refill_count = (remaining_unloaded > NR_W'(free_slots)) ?
                       {1'b0, free_slots} : {1'b0, remaining_unloaded[2:0]};
      end
    end
  end

  assign auto_remove_pulse = auto_run_q && done_seen_q && remove_valid && !head_push_req;
  assign auto_start_after_remove = auto_remove_pulse &&
                                   (active_after_remove != NR_W'(0)) &&
                                   head_complete_after_remove &&
                                   !plan_queue_full;
  assign auto_resume_pulse = auto_run_q && done_seen_q && !remove_valid && !head_push_req &&
                             (active_count_q != NR_W'(0)) &&
                             head_complete_current &&
                             !plan_queue_full;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
	      cache_eid_c2_q <= 8'hff;
	      cache_eid_c3_q <= 8'hff;
	      head_q         <= '{default: '0};
	      reserve_q      <= '{default: '0};
	      active_count_q <= '0;
	      total_conc_q   <= '0;
	      done_seen_q    <= 1'b0;
	      auto_run_q     <= 1'b0;
	    end else begin
	      if (write_req) begin
	        unique case (word_addr)
          REG_CONFIG: begin
            cache_eid_c2_q <= wr_data[7:0];
            cache_eid_c3_q <= wr_data[15:8];
            active_count_q <= wr_data[16 +: NR_W];
            total_conc_q   <= wr_data[32 +: T_W];
          end
          REG_HEAD_PAIR0: begin
            head_q[0] <= unpack_head16(wr_data[15:0]);
            head_q[1] <= unpack_head16(wr_data[31:16]);
          end
	          REG_HEAD_PAIR1: begin
	            head_q[2] <= unpack_head16(wr_data[15:0]);
	            head_q[3] <= unpack_head16(wr_data[31:16]);
	          end
	          REG_HEAD_PUSH_PAIR: begin
	            int wr_pos;
	            wr_pos = int'(reserve_count);
	            for (int p = 0; p < 2; p++) begin
	              if (push_head[p].valid && (wr_pos < 4)) begin
	                reserve_q[wr_pos] <= push_head[p];
	                wr_pos++;
	              end
	            end
	          end
	          REG_RESERVE_PAIR0: begin
	            reserve_q[0] <= unpack_head16(wr_data[15:0]);
	            reserve_q[1] <= unpack_head16(wr_data[31:16]);
	          end
	          REG_RESERVE_PAIR1: begin
	            reserve_q[2] <= unpack_head16(wr_data[15:0]);
	            reserve_q[3] <= unpack_head16(wr_data[31:16]);
	          end
	          default: ;
	        endcase
	      end

	      if (remove_ready_pulse) begin
	        head_q <= commit_head_next;
	        active_count_q <= active_count_q - NR_W'(remove_count);
	        total_conc_q <= total_conc_q - T_W'(removed_conc);
	        for (int r = 0; r < 4; r++) begin
	          if ((r + int'(reserve_pop_count)) < 4) begin
	            reserve_q[r] <= reserve_q[r + int'(reserve_pop_count)];
	          end else begin
	            reserve_q[r] <= '0;
	          end
	        end
	      end

	      if (init_pulse || start_pulse) begin
	        done_seen_q <= 1'b0;
	      end else if (done) begin
	        done_seen_q <= 1'b1;
	      end

	      if (init_pulse) begin
	        auto_run_q <= ctrl_start_pulse;
	      end else if (ctrl_start_pulse) begin
	        auto_run_q <= 1'b1;
	      end else if (remove_ready_pulse &&
	                   ((active_count_q - NR_W'(remove_count)) == NR_W'(0))) begin
	        auto_run_q <= 1'b0;
	      end
	    end
	  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      s4pf_pending_valid_q <= '0;
      s4pf_pending_skip_s1_q <= '0;
      s4pf_pending_slot_q <= '{default: '0};
    end else begin
      s4pf_pending_valid_q <= s4pf_pending_valid_d;
      s4pf_pending_skip_s1_q <= s4pf_pending_skip_s1_d;
      s4pf_pending_slot_q <= s4pf_pending_slot_d;
    end
  end

  always_comb begin
    rd_data  = '0;
    addr_hit = 1'b1;

    unique case (word_addr)
      REG_CTRL: begin
        rd_data = '0;
      end
      REG_STATUS: begin
        rd_data[0]      = busy;
        rd_data[1]      = done_seen_q;
        rd_data[2]      = remove_valid;
	        rd_data[3]      = plan_valid;
	        rd_data[4]      = plan_queue_full;
	        rd_data[5]      = (active_count_q == NR_W'(0));
	        rd_data[6]      = refill_req;
	        rd_data[9:8]    = remove_count;
	        rd_data[17:16]  = plan_count;
	        rd_data[25:24]  = plan_slot_valid;
	        rd_data[31:28]  = refill_count;
	        rd_data[35:32]  = {1'b0, reserve_count};
	      end
      REG_CONFIG: begin
        rd_data[7:0]           = cache_eid_c2_q;
        rd_data[15:8]          = cache_eid_c3_q;
        rd_data[16 +: NR_W]    = active_count_q;
        rd_data[32 +: T_W]     = total_conc_q;
      end
	      REG_PLAN_FIFO_STATUS: begin
	        rd_data[0]      = !plan_valid;
	        rd_data[1]      = plan_queue_full;
	        rd_data[5:2]    = plan_fifo_count;
	        rd_data[9:8]    = plan_count;
	        rd_data[13:12]  = plan_remove_count;
	        rd_data[17:16]  = plan_slot_valid;
	        rd_data[21:18]  = {1'b0, reserve_count};
	        rd_data[22]     = refill_req;
	        rd_data[27:24]  = refill_count;
	        rd_data[28]     = (active_count_q == NR_W'(0));
	      end
      REG_PLAN_FIFO_DATA0: begin
        rd_data = pack_task(plan_task_desc[0], plan_allow_s4pf[0],
                            plan_local_slot[0]);
      end
      REG_PLAN_FIFO_DATA1: begin
        rd_data = pack_task(plan_task_desc[1], plan_allow_s4pf[1],
                            plan_local_slot[1]);
      end
      REG_PLAN_FIFO_PATCH: begin
        rd_data = plan_patch_word;
      end
	      REG_ROUND_COMMIT, REG_HEAD_PAIR0, REG_HEAD_PAIR1, REG_HEAD_PUSH_PAIR,
	      REG_RESERVE_PAIR0, REG_RESERVE_PAIR1: begin
	        rd_data = '0;
	      end
      default: begin
        addr_hit = 1'b0;
      end
    endcase
  end

  assign reg_rsp_o.rdata = rd_data;
  assign reg_rsp_o.error = reg_req_i.valid && !addr_hit;
  assign reg_rsp_o.ready = 1'b1;

  sched_schedule_core i_sched_schedule_core (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .init_i             (init_pulse),
    .start_i            (start_pulse),
    .cache_eid_c2_i     (cache_eid_c2_q),
    .cache_eid_c3_i     (cache_eid_c3_q),
    .head_i             (head_q),
    .active_count_i     (active_count_q),
    .total_conc_i       (total_conc_q),
    .remove_ready_i     (remove_ready_pulse),
    .remove_valid_o     (remove_valid),
    .remove_count_o     (remove_count),
    .remove_slot_mask_o (remove_slot_mask),
    .plan_pop_i         (plan_pop_pulse),
    .plan_valid_o       (plan_valid),
    .plan_slot_valid_o  (plan_slot_valid),
    .plan_task_desc_o   (plan_task_desc),
    .plan_allow_s4pf_o  (plan_allow_s4pf),
    .plan_local_slot_o  (plan_local_slot),
    .plan_count_o       (plan_count),
	    .plan_remove_count_o(plan_remove_count),
	    .plan_queue_full_o  (plan_queue_full),
	    .plan_queue_count_o (plan_fifo_count),
	    .busy_o             (busy),
    .done_o             (done),
    .makespan_o         ()
  );

endmodule
