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
//                     bits[9:8] remove_count, bits[17:16] plan_count,
//                     bits[25:24] plan_slot_valid
//   0x10 CONFIG    R/W: bits[7:0] cache_eid_c2, bits[15:8] cache_eid_c3,
//                       bits[16 +: NR_W] active_count, bits[32 +: T_W] total_conc
//   0x68 ROUND_COMMIT W: bit0 plan_pop, bit1 remove_ready, bit2 start_next,
//                        bits[5:4] push_count for staged HEAD_PUSH_PAIR
//   0x80 PLAN_FIFO_STATUS R: bit0 empty, bit1 full, bits[5:2] queue_count,
//                            bits[9:8] head_plan_count,
//                            bits[13:12] head_remove_count,
//                            bits[17:16] head_slot_valid
//   0x90 PLAN_FIFO_DATA0  R: FIFO head plan slot 0
//   0x98 PLAN_FIFO_DATA1  R: FIFO head plan slot 1
//   0xa8 HEAD_PAIR0       W: compact head32 pair for top4 slot 0/1
//   0xb0 HEAD_PAIR1       W: compact head32 pair for top4 slot 2/3
//   0xb8 HEAD_PUSH_PAIR   W: compact head32 pair for staged push0/push1
//                            consumed by ROUND_COMMIT.
//                            head32 layout: best_conc, ntok, eid, valid.

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
  localparam logic [4:0] REG_HEAD_PAIR0       = 5'h15;
  localparam logic [4:0] REG_HEAD_PAIR1       = 5'h16;
  localparam logic [4:0] REG_HEAD_PUSH_PAIR   = 5'h17;

  localparam int unsigned TASK_BITS = $bits(task_desc_t);
  localparam int unsigned PLAN_ALLOW_S4PF_LSB = TASK_BITS;
  localparam int unsigned HEAD32_BEST_LSB  = 0;
  localparam int unsigned HEAD32_NTOK_LSB  = HEAD32_BEST_LSB + T_W;
  localparam int unsigned HEAD32_EID_LSB   = HEAD32_NTOK_LSB + NTOK_W;
  localparam int unsigned HEAD32_VALID_LSB = HEAD32_EID_LSB + EID_RAW_W;

  logic [4:0]  word_addr;
  logic        write_req;
  logic [63:0] wr_data;
  logic [63:0] rd_data;
  logic        addr_hit;

  logic init_pulse;
  logic start_pulse;
  logic remove_ready_pulse;
  logic plan_pop_pulse;
  logic ctrl_init_pulse;
  logic ctrl_start_pulse;
  logic round_commit_req;
  logic commit_plan_pop;
  logic commit_remove_ready;
  logic commit_start_next;
  logic [1:0] commit_push_count;

  logic [7:0]      cache_eid_c2_q;
  logic [7:0]      cache_eid_c3_q;
  head_ctx_t [3:0] head_q;
  head_ctx_t [3:0] compact_head;
  head_ctx_t [3:0] commit_head_next;
  head_ctx_t [1:0] staged_head_q;
  logic [NR_W-1:0] active_count_q;
  logic [T_W-1:0]  total_conc_q;
  logic [T_W+1:0]  removed_conc;
  logic [3:0]      keep_mask;
  logic [1:0]      keep_target_idx [3:0];

  logic                         remove_valid;
  logic [1:0]                   remove_count;
  logic [3:0]                   remove_slot_mask;
  logic                         plan_valid;
  logic [1:0]                   plan_slot_valid;
  task_desc_t [1:0]             plan_task_desc;
  logic [1:0]                   plan_allow_s4pf;
  logic [1:0]                   plan_count;
  logic [1:0]                   plan_remove_count;
  logic [1:0]                   plan_fifo_count;
  logic                         plan_queue_full;
  logic                         busy;
  logic                         done;
  logic                         done_seen_q;

  function automatic head_ctx_t unpack_head32(input logic [31:0] word);
    head_ctx_t ctx;
    begin
      ctx = '0;
      ctx.best_conc = word[HEAD32_BEST_LSB +: T_W];
      ctx.ntok      = word[HEAD32_NTOK_LSB +: NTOK_W];
      ctx.eid       = word[HEAD32_EID_LSB +: EID_RAW_W];
      ctx.valid     = word[HEAD32_VALID_LSB];
      unpack_head32 = ctx;
    end
  endfunction

  /*
   * 64-bit plan word layout:
   *   bits [31:0]  compact task_desc_t
   *   bit  [32]    allow_s4pf
   *   bits [63:33] reserved for the future patch-record protocol.
   */
  function automatic logic [63:0] pack_task(input task_desc_t task_desc, input logic allow_s4pf);
    logic [63:0] word;
    begin
      word = '0;
      word[TASK_BITS-1:0]       = task_desc;
      word[PLAN_ALLOW_S4PF_LSB] = allow_s4pf;
      pack_task = word;
    end
  endfunction

  assign word_addr          = reg_req_i.addr[7:3];
  assign write_req          = reg_req_i.valid && reg_req_i.write;
  assign wr_data            = reg_req_i.wdata[63:0];
  assign ctrl_init_pulse    = write_req && (word_addr == REG_CTRL) && wr_data[0];
  assign ctrl_start_pulse   = write_req && (word_addr == REG_CTRL) && wr_data[1];
  assign round_commit_req   = write_req && (word_addr == REG_ROUND_COMMIT);
  assign commit_plan_pop    = round_commit_req && wr_data[0];
  assign commit_remove_ready = round_commit_req && wr_data[1];
  assign commit_start_next  = round_commit_req && wr_data[2];
  assign commit_push_count  = round_commit_req ? wr_data[5:4] : 2'd0;

  assign init_pulse         = ctrl_init_pulse;
  assign start_pulse        = ctrl_start_pulse || commit_start_next;
  assign remove_ready_pulse = commit_remove_ready;
  assign plan_pop_pulse     = commit_plan_pop;
  assign plan_fifo_count    = plan_queue_full ? 2'd2 : (plan_valid ? 2'd1 : 2'd0);

  // Prefix/rank compact for the 4-entry top window.
  //
  // keep_mask[i]=1 means HEADi survives into the next round.  target_idx is
  // the exclusive prefix sum of keep_mask.  Consumed entries have keep_mask=0,
  // so they do not get a new position.  ROUND_COMMIT applies compact first and
  // then appends 0/1/2 staged heads at the tail.
  always_comb begin
    compact_head = '{default: '0};
    commit_head_next = head_q;
    removed_conc = '0;

    for (int i = 0; i < 4; i++) begin
      keep_mask[i] = head_q[i].valid && !remove_slot_mask[i];
      if (head_q[i].valid && remove_slot_mask[i]) begin
        removed_conc = removed_conc + {2'b0, head_q[i].best_conc};
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
      end
    end

    commit_head_next = remove_ready_pulse ? compact_head : head_q;
    for (int p = 0; p < 2; p++) begin
      if ((p < int'(commit_push_count)) && staged_head_q[p].valid) begin
        if (!commit_head_next[2].valid) begin
          commit_head_next[2] = staged_head_q[p];
        end else if (!commit_head_next[3].valid) begin
          commit_head_next[3] = staged_head_q[p];
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cache_eid_c2_q <= 8'hff;
      cache_eid_c3_q <= 8'hff;
      head_q         <= '{default: '0};
      active_count_q <= '0;
      total_conc_q   <= '0;
      done_seen_q    <= 1'b0;
      staged_head_q  <= '{default: '0};
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
            head_q[0] <= unpack_head32(wr_data[31:0]);
            head_q[1] <= unpack_head32(wr_data[63:32]);
          end
          REG_HEAD_PAIR1: begin
            head_q[2] <= unpack_head32(wr_data[31:0]);
            head_q[3] <= unpack_head32(wr_data[63:32]);
          end
          REG_HEAD_PUSH_PAIR: begin
            staged_head_q[0] <= unpack_head32(wr_data[31:0]);
            staged_head_q[1] <= unpack_head32(wr_data[63:32]);
          end
          default: ;
        endcase
      end

      if (remove_ready_pulse) begin
        head_q <= commit_head_next;
        active_count_q <= active_count_q - NR_W'(remove_count);
        total_conc_q <= total_conc_q - T_W'(removed_conc);
      end

      if (init_pulse || round_commit_req) begin
        staged_head_q <= '{default: '0};
      end

      if (init_pulse || start_pulse) begin
        done_seen_q <= 1'b0;
      end else if (done) begin
        done_seen_q <= 1'b1;
      end
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
        rd_data[9:8]    = remove_count;
        rd_data[17:16]  = plan_count;
        rd_data[25:24]  = plan_slot_valid;
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
        rd_data[5:2]    = {2'b0, plan_fifo_count};
        rd_data[9:8]    = plan_count;
        rd_data[13:12]  = plan_remove_count;
        rd_data[17:16]  = plan_slot_valid;
      end
      REG_PLAN_FIFO_DATA0: begin
        rd_data = pack_task(plan_task_desc[0], plan_allow_s4pf[0]);
      end
      REG_PLAN_FIFO_DATA1: begin
        rd_data = pack_task(plan_task_desc[1], plan_allow_s4pf[1]);
      end
      REG_ROUND_COMMIT, REG_HEAD_PAIR0, REG_HEAD_PAIR1, REG_HEAD_PUSH_PAIR: begin
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
    .plan_count_o       (plan_count),
    .plan_remove_count_o(plan_remove_count),
    .plan_queue_full_o  (plan_queue_full),
    .busy_o             (busy),
    .done_o             (done),
    .makespan_o         ()
  );

endmodule
