// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// tb_scheduler_reg_wrapper.sv
// ---------------------------------------------------------------------------
// Register-wrapper test for the MoE scheduler.
//
// This testbench reuses the same golden vectors as tb_schedule_core, but drives
// the scheduler only through moe_scheduler_reg_wrapper's regbus MMIO map.  It
// therefore checks address decoding, struct packing, control pulses, status
// reporting, and result readback in addition to the schedule_core datapath.

`timescale 1ns/1ps
import sched_pkg::*;

module tb_scheduler_reg_wrapper;

  localparam int unsigned MAX_N_LOCAL    = E_MAX;
  localparam int unsigned MAX_PLAN_LOCAL = 2 * E_MAX;
  localparam string       VEC_FILE       = "schedule_vectors.txt";
  localparam int unsigned ROUND_TIMEOUT  = 20000;

  localparam logic [47:0] ADDR_CTRL      = 48'h00;
  localparam logic [47:0] ADDR_STATUS    = 48'h08;
  localparam logic [47:0] ADDR_CONFIG    = 48'h10;
  localparam logic [47:0] ADDR_TASK_POP = 48'h68;
  localparam logic [47:0] ADDR_HEAD_QUAD       = 48'ha8;
  localparam logic [47:0] ADDR_RESERVE_QUAD    = 48'hb0;
  localparam logic [47:0] ADDR_HEAD_PUSH_QUAD  = 48'hb8;
  localparam logic [47:0] ADDR_TASK_DATA_BASE = 48'h100;
  localparam logic [47:0] ADDR_TASK_DATA_STRIDE = 48'h08;

  localparam int unsigned TASK_WORD_EID_LSB         = 0;
  localparam int unsigned TASK_WORD_TOKEN_START_LSB = TASK_WORD_EID_LSB + EID_RAW_W;
  localparam int unsigned TASK_WORD_NTOK_LSB        = TASK_WORD_TOKEN_START_LSB + NTOK_W;
  localparam int unsigned TASK_WORD_HAS_S2PF_LSB    = TASK_WORD_NTOK_LSB + NTOK_W;
  localparam int unsigned TASK_WORD_CTRL_LSB        = TASK_WORD_HAS_S2PF_LSB + 1;
  localparam int unsigned TASK_WORD_CTRL_W          = 13;
  localparam int unsigned TASK_WORD_M_S2_LSB        = TASK_WORD_CTRL_LSB + TASK_WORD_CTRL_W;
  localparam int unsigned TASK_WORD_M_S4_LSB        = TASK_WORD_M_S2_LSB + NTOK_W;
  localparam int unsigned TASK_WORD_S4PF_DESC_LSB = TASK_WORD_M_S4_LSB + NTOK_W;
  localparam int unsigned TASK_CTRL_SKIP_S1_LSB    = 0;
  localparam int unsigned TASK_CTRL_SKIP_S3_LSB    = 1;
  localparam int unsigned TASK_CTRL_SHAPE_S1_LSB   = 2;
  localparam int unsigned TASK_CTRL_SHAPE_S3_LSB   = 4;
  localparam int unsigned TASK_CTRL_CLUSTER_LSB    = 6;
  localparam int unsigned TASK_CTRL_LOCAL_SLOT_LSB = 7;
  localparam int unsigned S4PF_DESC_VALID_LSB      = 0;
  localparam int unsigned S4PF_DESC_NO_COPY_LSB    = 1;
  localparam int unsigned S4PF_DESC_TARGET_EID_LSB = 2;

  typedef struct packed {
    logic       valid;
    task_desc_t desc;
    logic       allow_s4pf;
  } plan_entry_t;

  typedef struct packed {
    ntok_t m_s2_exec;
    ntok_t m_s4_exec;
  } tb_plan_scalar_t;

  logic clk_i;
  logic rst_ni;

  typedef struct packed {
    logic [47:0] addr;
    logic        write;
    logic [63:0] wdata;
    logic [7:0]  wstrb;
    logic        valid;
  } tb_reg_req_t;

  typedef struct packed {
    logic [63:0] rdata;
    logic        error;
    logic        ready;
  } tb_reg_rsp_t;

  tb_reg_req_t reg_req;
  tb_reg_rsp_t reg_rsp;

  moe_scheduler_reg_wrapper #(
    .reg_req_t (tb_reg_req_t),
    .reg_rsp_t (tb_reg_rsp_t)
  ) dut (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .reg_req_i (reg_req),
    .reg_rsp_o (reg_rsp)
  );

  logic [EID_RAW_W-1:0] rem_eid [MAX_N_LOCAL];
  logic [NTOK_W-1:0]    rem_ntok [MAX_N_LOCAL];
  logic                 rem_active [MAX_N_LOCAL];

  typedef struct packed {
    logic                 valid;
    logic [NR_W-1:0]      rem_index;
    logic [EID_RAW_W-1:0] eid;
    logic [NTOK_W-1:0]    ntok;
    logic [NR_W-1:0]      input_order;
  } rem_head_t;

  rem_head_t            sw_head [3:0];
  rem_head_t            sw_reserve [3:0];
  plan_entry_t golden_plan [MAX_PLAN_LOCAL];
  logic task_seen [1:0][1 << SLOT_W];

  int total_tests;
  int total_pass;
  int total_fail;
  int total_rounds;
  int total_plan_entries;

  initial clk_i = 1'b0;
  always #5 clk_i = ~clk_i;

  function automatic logic [7:0] cache_to_rtl(input int cache_eid);
    if (cache_eid < 0) begin
      cache_to_rtl = 8'h80;
    end else begin
      cache_to_rtl = cache_eid[7:0];
    end
  endfunction

  function automatic int count_active(input int n_experts);
    int n;
    begin
      n = 0;
      for (int i = 0; i < MAX_N_LOCAL; i++) begin
        if ((i < n_experts) && rem_active[i]) begin
          n++;
        end
      end
      count_active = n;
    end
  endfunction

  function automatic logic [15:0] pack_head16(input rem_head_t head);
    logic [15:0] word;
    begin
      word = '0;
      word[0 +: NTOK_W] = head.ntok;
      word[NTOK_W +: EID_RAW_W] = head.eid;
      word[NTOK_W + EID_RAW_W] = head.valid;
      pack_head16 = word;
    end
  endfunction

  function automatic logic [63:0] pack_head_quad(
    input rem_head_t h0,
    input rem_head_t h1,
    input rem_head_t h2,
    input rem_head_t h3
  );
    pack_head_quad = {pack_head16(h3), pack_head16(h2),
                      pack_head16(h1), pack_head16(h0)};
  endfunction

  function automatic logic [63:0] pack_config_word(
    input int cache_c2,
    input int cache_c3,
    input int active_count,
    input int unsigned total_conc
  );
    logic [63:0] word;
    begin
      word = '0;
      word[7:0]        = cache_to_rtl(cache_c2);
      word[15:8]       = cache_to_rtl(cache_c3);
      word[16 +: NR_W] = NR_W'(active_count);
      word[32 +: T_W]  = T_W'(total_conc);
      pack_config_word = word;
    end
  endfunction

  function automatic logic [63:0] pack_task_pop(
    input int unsigned pop_count
  );
    logic [63:0] word;
    begin
      word = '0;
      word[3:0] = pop_count[3:0];
      pack_task_pop = word;
    end
  endfunction

  function automatic rem_head_t make_head_from_rem(input int idx);
    rem_head_t h;
    begin
      h = '0;
      if ((idx >= 0) && (idx < MAX_N_LOCAL)) begin
        h.valid       = 1'b1;
        h.rem_index   = NR_W'(idx);
        h.eid         = rem_eid[idx];
        h.ntok        = rem_ntok[idx];
        h.input_order = NR_W'(idx);
      end
      make_head_from_rem = h;
    end
  endfunction

  function automatic task_desc_t unpack_task(input logic [63:0] word);
    task_desc_t t;
    logic [TASK_WORD_CTRL_W-1:0] ctrl;
    begin
      ctrl = word[TASK_WORD_CTRL_LSB +: TASK_WORD_CTRL_W];
      t = '0;
      t.eid       = word[TASK_WORD_EID_LSB +: EID_RAW_W];
      t.tok_start = word[TASK_WORD_TOKEN_START_LSB +: NTOK_W];
      t.ntok      = word[TASK_WORD_NTOK_LSB +: NTOK_W];
      t.has_s2pf  = word[TASK_WORD_HAS_S2PF_LSB];
      t.skip_s1   = ctrl[TASK_CTRL_SKIP_S1_LSB];
      t.skip_s3   = ctrl[TASK_CTRL_SKIP_S3_LSB];
      t.s1        = ctrl[TASK_CTRL_SHAPE_S1_LSB +: 2];
      t.s3        = ctrl[TASK_CTRL_SHAPE_S3_LSB +: 2];
      t.cluster   = ctrl[TASK_CTRL_CLUSTER_LSB];
      unpack_task = t;
    end
  endfunction

  function automatic slot_id_t unpack_local_slot(input logic [63:0] word);
    logic [TASK_WORD_CTRL_W-1:0] ctrl;
    begin
      ctrl = word[TASK_WORD_CTRL_LSB +: TASK_WORD_CTRL_W];
      unpack_local_slot = slot_id_t'(ctrl[TASK_CTRL_LOCAL_SLOT_LSB +: SLOT_W]);
    end
  endfunction

  function automatic tb_plan_scalar_t unpack_plan_scalar(input logic [63:0] word);
    tb_plan_scalar_t s;
    begin
      s = '0;
      s.m_s2_exec = word[TASK_WORD_M_S2_LSB +: NTOK_W];
      s.m_s4_exec = word[TASK_WORD_M_S4_LSB +: NTOK_W];
      unpack_plan_scalar = s;
    end
  endfunction

  function automatic ntok_t tb_shape_mdim(input shape_t shape);
    begin
      unique case (shape)
        SHAPE_A: tb_shape_mdim = ntok_t'(8);
        SHAPE_B: tb_shape_mdim = ntok_t'(4);
        default: tb_shape_mdim = ntok_t'(2);
      endcase
    end
  endfunction

  function automatic tb_plan_scalar_t scalar_from_task(input task_desc_t t);
    tb_plan_scalar_t s;
    ntok_t tail_s2;
    ntok_t tail_s4;
    begin
      tail_s2 = t.skip_s1 ? t.ntok :
                ((t.ntok > tb_shape_mdim(t.s1)) ? (t.ntok - tb_shape_mdim(t.s1)) : '0);
      tail_s4 = t.skip_s3 ? t.ntok :
                ((t.ntok > tb_shape_mdim(t.s3)) ? (t.ntok - tb_shape_mdim(t.s3)) : '0);
      s.m_s2_exec = ceil_div2_ntok(tail_s2);
      s.m_s4_exec = ceil_div2_ntok(tail_s4);
      scalar_from_task = s;
    end
  endfunction

  function automatic logic [47:0] task_data_addr(input int unsigned task_idx);
    task_data_addr = ADDR_TASK_DATA_BASE + ADDR_TASK_DATA_STRIDE * task_idx;
  endfunction

  task automatic reg_write(input logic [47:0] addr, input logic [63:0] data);
    begin
      @(negedge clk_i);
      reg_req.addr  = addr;
      reg_req.write = 1'b1;
      reg_req.wdata = data;
      reg_req.wstrb = 8'hff;
      reg_req.valid = 1'b1;
      @(negedge clk_i);
      reg_req.valid = 1'b0;
      reg_req.write = 1'b0;
      reg_req.addr  = '0;
      reg_req.wdata = '0;
      reg_req.wstrb = '0;
    end
  endtask

  task automatic reg_read(input logic [47:0] addr, output logic [63:0] data);
    begin
      @(negedge clk_i);
      reg_req.addr  = addr;
      reg_req.write = 1'b0;
      reg_req.wdata = '0;
      reg_req.wstrb = '0;
      reg_req.valid = 1'b1;
      #1;
      data = reg_rsp.rdata;
      if (reg_rsp.ready !== 1'b1) begin
        $display("[FAIL] reg read not ready addr=0x%0h", addr);
        total_fail++;
      end
      if (reg_rsp.error !== 1'b0) begin
        $display("[FAIL] reg read error addr=0x%0h", addr);
        total_fail++;
      end
      @(negedge clk_i);
      reg_req.valid = 1'b0;
      reg_req.addr  = '0;
    end
  endtask

  task automatic drive_round_context(input int n_experts, input int cache_c2, input int cache_c3);
    rem_head_t [3:0] heads;
    int h;
    int active_cnt;
    int unsigned total_conc;
    begin
      heads = '{default: '0};
      h = 0;
      active_cnt = 0;
      total_conc = 0;

      for (int i = 0; i < MAX_N_LOCAL; i++) begin
        if ((i < n_experts) && rem_active[i]) begin
          if (h < 4) begin
            heads[h].valid       = 1'b1;
            heads[h].rem_index   = NR_W'(i);
            heads[h].eid         = rem_eid[i];
            heads[h].ntok        = rem_ntok[i];
            heads[h].input_order = NR_W'(i);
            h++;
          end
          active_cnt++;
          total_conc += int'(best_conc_ticks(rem_ntok[i]));
        end
      end

      reg_write(ADDR_CONFIG, pack_config_word(cache_c2, cache_c3, active_cnt, total_conc));
      reg_write(ADDR_HEAD_QUAD, pack_head_quad(heads[0], heads[1], heads[2], heads[3]));
    end
  endtask

  function automatic bit consumed_eid_match(
    input logic [EID_RAW_W-1:0] eid,
    input logic [EID_RAW_W-1:0] consumed_eids [1:0],
    input int consumed_count
  );
    begin
      consumed_eid_match = 1'b0;
      for (int i = 0; i < 2; i++) begin
        if ((i < consumed_count) && (consumed_eids[i] == eid)) begin
          consumed_eid_match = 1'b1;
        end
      end
    end
  endfunction

  task automatic note_consumed_eid(
    input logic [EID_RAW_W-1:0] eid,
    inout logic [EID_RAW_W-1:0] consumed_eids [1:0],
    inout int consumed_count,
    input int tid,
    input int round_idx
  );
    begin
      if (!consumed_eid_match(eid, consumed_eids, consumed_count)) begin
        if (consumed_count >= 2) begin
          $display("[FAIL] tid=%0d round=%0d too many consumed experts", tid, round_idx);
          total_fail++;
        end else begin
          consumed_eids[consumed_count] = eid;
          consumed_count++;
        end
      end
    end
  endtask

  task automatic compact_sw_head(
    input logic [EID_RAW_W-1:0] consumed_eids [1:0],
    input int consumed_count
  );
    rem_head_t compacted [3:0];
    int wr_pos;
    begin
      compacted = '{default: '0};
      wr_pos = 0;
      for (int i = 0; i < 4; i++) begin
        if (sw_head[i].valid &&
            !consumed_eid_match(sw_head[i].eid, consumed_eids, consumed_count)) begin
          compacted[wr_pos] = sw_head[i];
          wr_pos++;
        end
      end
      sw_head = compacted;
    end
  endtask

  task automatic append_sw_head(input rem_head_t head);
    begin
      if (!head.valid) begin
        return;
      end
      if (!sw_head[0].valid) begin
        sw_head[0] = head;
      end else if (!sw_head[1].valid) begin
        sw_head[1] = head;
      end else if (!sw_head[2].valid) begin
        sw_head[2] = head;
      end else if (!sw_head[3].valid) begin
        sw_head[3] = head;
      end else begin
        $display("[FAIL] software head window overflow on append");
        total_fail++;
      end
    end
  endtask

  task automatic append_sw_reserve(input rem_head_t head);
    begin
      if (!head.valid) begin
        return;
      end
      if (!sw_reserve[0].valid) begin
        sw_reserve[0] = head;
      end else if (!sw_reserve[1].valid) begin
        sw_reserve[1] = head;
      end else if (!sw_reserve[2].valid) begin
        sw_reserve[2] = head;
      end else if (!sw_reserve[3].valid) begin
        sw_reserve[3] = head;
      end else begin
        $display("[FAIL] software reserve overflow on append");
        total_fail++;
      end
    end
  endtask

  task automatic refill_sw_head_from_reserve(input int active_remaining);
    int target_count;
    int head_count;
    begin
      target_count = (active_remaining > 4) ? 4 : active_remaining;
      head_count = 0;
      for (int i = 0; i < 4; i++) begin
        if (sw_head[i].valid) begin
          head_count++;
        end
      end
      while ((head_count < target_count) && sw_reserve[0].valid) begin
        append_sw_head(sw_reserve[0]);
        for (int r = 0; r < 3; r++) begin
          sw_reserve[r] = sw_reserve[r + 1];
        end
        sw_reserve[3] = '0;
        head_count++;
      end
    end
  endtask

  task automatic fail_msg(input string msg, input int tid, input int round_idx);
    begin
      $display("[FAIL] tid=%0d round=%0d %s", tid, round_idx, msg);
      total_fail++;
    end
  endtask

	  task automatic compare_task(
	    input int tid,
	    input int round_idx,
	    input int plan_idx,
	    input task_desc_t got,
		    input slot_id_t got_slot,
		    input tb_plan_scalar_t got_scalar,
		    input slot_id_t exp_slot,
	    input plan_entry_t exp
	  );
	    tb_plan_scalar_t exp_scalar;
	    begin
	      exp_scalar = scalar_from_task(exp.desc);
	      if (!exp.valid) begin
	        $display("[FAIL] tid=%0d plan=%0d expected entry invalid", tid, plan_idx);
	        total_fail++;
      end

      if (got.cluster !== exp.desc.cluster ||
          got.eid !== exp.desc.eid ||
          got.ntok !== exp.desc.ntok ||
          got.tok_start !== exp.desc.tok_start ||
	          got.s1 !== exp.desc.s1 ||
	          got.s3 !== exp.desc.s3 ||
		          got.skip_s1 !== exp.desc.skip_s1 ||
		          got.skip_s3 !== exp.desc.skip_s3 ||
		          got.has_s2pf !== exp.desc.has_s2pf ||
		          got_slot !== exp_slot ||
	          got_scalar.m_s2_exec !== exp_scalar.m_s2_exec ||
	          got_scalar.m_s4_exec !== exp_scalar.m_s4_exec) begin
	        $display("[FAIL] tid=%0d round=%0d plan=%0d mismatch", tid, round_idx, plan_idx);
		        $display("       got: cluster=%0d slot=%0d eid=%0d tok_start=%0d ntok=%0d s1=%0d s3=%0d skip_s1=%0d skip_s3=%0d has_s2pf=%0d m_s2=%0d m_s4=%0d",
		                 got.cluster, got_slot, got.eid, got.tok_start, got.ntok, got.s1, got.s3,
		                 got.skip_s1, got.skip_s3, got.has_s2pf,
		                 got_scalar.m_s2_exec, got_scalar.m_s4_exec);
		        $display("       exp: cluster=%0d slot=%0d eid=%0d tok_start=%0d ntok=%0d s1=%0d s3=%0d skip_s1=%0d skip_s3=%0d has_s2pf=%0d allow_s4pf=%0d m_s2=%0d m_s4=%0d",
		                 exp.desc.cluster, exp_slot, exp.desc.eid, exp.desc.tok_start, exp.desc.ntok,
		                 exp.desc.s1, exp.desc.s3, exp.desc.skip_s1,
	                 exp.desc.skip_s3, exp.desc.has_s2pf, exp.allow_s4pf,
	                 exp_scalar.m_s2_exec, exp_scalar.m_s4_exec);
	        total_fail++;
	      end
	    end
	  endtask

	  task automatic compare_final_ctrl(
	    input int tid,
	    input int round_idx,
	    input int plan_idx,
	    input logic [63:0] word,
	    input task_desc_t exp_task,
	    input slot_id_t exp_slot
		  );
			    logic [TASK_WORD_CTRL_W-1:0] ctrl;
			    begin
			      ctrl = word[TASK_WORD_CTRL_LSB +: TASK_WORD_CTRL_W];

		      if (ctrl[TASK_CTRL_SKIP_S1_LSB] !== exp_task.skip_s1 ||
		          ctrl[TASK_CTRL_SKIP_S3_LSB] !== exp_task.skip_s3 ||
		          ctrl[TASK_CTRL_SHAPE_S1_LSB +: 2] !== exp_task.s1 ||
		          ctrl[TASK_CTRL_SHAPE_S3_LSB +: 2] !== exp_task.s3 ||
		          ctrl[TASK_CTRL_CLUSTER_LSB] !== exp_task.cluster ||
		          ctrl[TASK_CTRL_LOCAL_SLOT_LSB +: SLOT_W] !== exp_slot) begin
		        $display("[FAIL] tid=%0d round=%0d plan=%0d compact ctrl mismatch ctrl=0x%04h",
		                 tid, round_idx, plan_idx, ctrl);
		        $display("       exp skip_s1=%0d skip_s3=%0d s1=%0d s3=%0d cluster=%0d slot=%0d",
		                 exp_task.skip_s1, exp_task.skip_s3,
		                 exp_task.s1, exp_task.s3,
		                 exp_task.cluster, exp_slot);
		        total_fail++;
		      end
	    end
	  endtask

  task automatic compare_dense_task_word(
    input int tid,
    input int drain_idx,
    input logic [63:0] word,
    input int golden_n,
    input int cache_c2,
    input int cache_c3
  );
    task_desc_t got;
    slot_id_t got_slot;
    tb_plan_scalar_t got_scalar;
    int ci;
    int match_idx;
    int cluster_pos;
    int next_idx;
    logic cache_hit;
    logic no_copy;
    logic [7:0] exp_s4pf_desc;
    begin
      got = unpack_task(word);
      got_slot = unpack_local_slot(word);
      got_scalar = unpack_plan_scalar(word);
      ci = int'(got.cluster);
      match_idx = -1;
      cluster_pos = 0;
      for (int i = 0; i < golden_n; i++) begin
        if (int'(golden_plan[i].desc.cluster) == ci) begin
          if (cluster_pos == int'(got_slot)) begin
            match_idx = i;
          end
          cluster_pos++;
        end
      end

      if (match_idx < 0) begin
        $display("[FAIL] tid=%0d drain=%0d no golden task for C%0d slot=%0d",
                 tid, drain_idx, ci + 2, got_slot);
        total_fail++;
      end else begin
        if (task_seen[ci][got_slot]) begin
          $display("[FAIL] tid=%0d drain=%0d duplicate C%0d slot=%0d",
                   tid, drain_idx, ci + 2, got_slot);
          total_fail++;
        end
        task_seen[ci][got_slot] = 1'b1;
        compare_task(tid, drain_idx, match_idx, got, got_slot, got_scalar,
                     got_slot, golden_plan[match_idx]);
        compare_final_ctrl(tid, drain_idx, match_idx, word,
                           golden_plan[match_idx].desc, got_slot);

        exp_s4pf_desc = '0;
        if (golden_plan[match_idx].allow_s4pf) begin
          next_idx = -1;
          for (int j = match_idx + 1; j < golden_n; j++) begin
            if ((next_idx < 0) &&
                (golden_plan[j].desc.cluster == golden_plan[match_idx].desc.cluster)) begin
              next_idx = j;
            end
          end
          if (next_idx >= 0) begin
            cache_hit = (ci == 0) ?
                ((cache_c2 >= 0) &&
                 (EID_RAW_W'(cache_c2) == golden_plan[next_idx].desc.eid)) :
                ((cache_c3 >= 0) &&
                 (EID_RAW_W'(cache_c3) == golden_plan[next_idx].desc.eid));
            no_copy = golden_plan[match_idx].desc.skip_s1 && cache_hit;
            exp_s4pf_desc[S4PF_DESC_VALID_LSB] = 1'b1;
            exp_s4pf_desc[S4PF_DESC_NO_COPY_LSB] = no_copy;
            exp_s4pf_desc[S4PF_DESC_TARGET_EID_LSB +: EID_RAW_W] =
                golden_plan[next_idx].desc.eid;
          end
        end
        if (word[TASK_WORD_S4PF_DESC_LSB +: 8] !== exp_s4pf_desc) begin
          $display("[FAIL] tid=%0d drain=%0d C%0d slot=%0d S4PF desc mismatch got=0x%02h exp=0x%02h",
                   tid, drain_idx, ci + 2, got_slot,
                   word[TASK_WORD_S4PF_DESC_LSB +: 8], exp_s4pf_desc);
          total_fail++;
        end
      end
    end
  endtask

  task automatic remove_one(input int idx, input int tid, input int round_idx);
    begin
      if ((idx < 0) || (idx >= MAX_N_LOCAL)) begin
        $display("[FAIL] tid=%0d round=%0d remove idx out of range idx=%0d",
                 tid, round_idx, idx);
        total_fail++;
      end else if (!rem_active[idx]) begin
        $display("[FAIL] tid=%0d round=%0d remove inactive idx=%0d",
                 tid, round_idx, idx);
        total_fail++;
      end else begin
        rem_active[idx] = 1'b0;
      end
    end
  endtask

  task automatic run_one_test(
    input int tid,
    input int n_experts,
    input int cache_c2,
    input int cache_c3,
    input int golden_n
  );
    int drain_idx;
    int task_seen_count;
    int cycles;
    int fail_before;
    int next_rem_pos;
    int unsigned total_conc;
    int refill_count;
    int refill_sent_count;
    int drain_tasks;
    logic [63:0] status_word;
    logic [63:0] task_word;
    rem_head_t push_heads [3:0];
    begin
      fail_before = total_fail;
      next_rem_pos = 0;
      total_conc = 0;
      for (int i = 0; i < n_experts; i++) begin
        total_conc += int'(best_conc_ticks(rem_ntok[i]));
      end

      sw_head = '{default: '0};
      sw_reserve = '{default: '0};
      for (int s = 0; s < 4; s++) begin
        if (next_rem_pos < n_experts) begin
          sw_head[s] = make_head_from_rem(next_rem_pos);
          next_rem_pos++;
        end
      end
      for (int s = 0; s < 4; s++) begin
        if (next_rem_pos < n_experts) begin
          sw_reserve[s] = make_head_from_rem(next_rem_pos);
          next_rem_pos++;
        end
      end

      reg_write(ADDR_CONFIG,
                pack_config_word(cache_c2, cache_c3, n_experts, total_conc));
      reg_write(ADDR_HEAD_QUAD,
                pack_head_quad(sw_head[0], sw_head[1], sw_head[2], sw_head[3]));
      reg_write(ADDR_RESERVE_QUAD,
                pack_head_quad(sw_reserve[0], sw_reserve[1],
                               sw_reserve[2], sw_reserve[3]));
      reg_write(ADDR_CTRL, 64'h3);

      drain_idx = 0;
      task_seen_count = 0;
      for (int ci = 0; ci < 2; ci++) begin
        for (int slot = 0; slot < (1 << SLOT_W); slot++) begin
          task_seen[ci][slot] = 1'b0;
        end
      end

      while (task_seen_count < golden_n) begin
        cycles = 0;
        do begin
          reg_read(ADDR_STATUS, status_word);
          cycles++;
        end while ((status_word[3] !== 1'b1) &&
                   (status_word[6] !== 1'b1) &&
                   (status_word[5] !== 1'b1) &&
                   (cycles < ROUND_TIMEOUT));

        if ((status_word[3] !== 1'b1) &&
            (status_word[6] !== 1'b1) &&
            (status_word[5] !== 1'b1)) begin
          $display("[FAIL] tid=%0d drain=%0d timeout waiting scheduler event",
                   tid, drain_idx);
          total_fail++;
          return;
        end

        if (status_word[6] === 1'b1) begin
          refill_count = int'(status_word[31:28]);
          refill_sent_count = 0;
          push_heads = '{default: '0};
          while ((refill_sent_count < refill_count) &&
                 (next_rem_pos < n_experts)) begin
            push_heads[refill_sent_count] = make_head_from_rem(next_rem_pos);
            next_rem_pos++;
            refill_sent_count++;
          end
          if (refill_sent_count != refill_count) begin
            $display("[FAIL] tid=%0d drain=%0d refill=%0d but only %0d experts remain",
                     tid, drain_idx, refill_count, refill_sent_count);
            total_fail++;
            return;
          end
          reg_write(ADDR_HEAD_PUSH_QUAD,
                    pack_head_quad(push_heads[0], push_heads[1],
                                   push_heads[2], push_heads[3]));
        end

        if (status_word[3] === 1'b1) begin
          drain_tasks = int'(status_word[39:36]);
          if ((drain_tasks <= 0) || (drain_tasks > TASKQ_DEPTH)) begin
            $display("[FAIL] tid=%0d drain=%0d illegal task FIFO count=%0d status=0x%016h",
                     tid, drain_idx, drain_tasks, status_word);
            total_fail++;
            return;
          end
          if (task_seen_count + drain_tasks > golden_n) begin
            $display("[FAIL] tid=%0d drain=%0d too many tasks seen=%0d count=%0d golden=%0d",
                     tid, drain_idx, task_seen_count, drain_tasks, golden_n);
            total_fail++;
            return;
          end

          for (int ti = 0; ti < drain_tasks; ti++) begin
            reg_read(task_data_addr(ti), task_word);
            compare_dense_task_word(tid, drain_idx, task_word,
                                    golden_n, cache_c2, cache_c3);
          end
          reg_write(ADDR_TASK_POP, pack_task_pop(drain_tasks));
          task_seen_count += drain_tasks;
          drain_idx++;
          total_rounds++;
        end else if ((status_word[5] === 1'b1) &&
                     (task_seen_count < golden_n)) begin
          $display("[FAIL] tid=%0d scheduler empty after %0d/%0d tasks",
                   tid, task_seen_count, golden_n);
          total_fail++;
          return;
        end
      end

      total_plan_entries += task_seen_count;
      if (total_fail == fail_before) begin
        total_pass++;
      end
    end
  endtask

  initial begin : tb_main
    int fd;
    int n_tests;
    int tid;
    int n_experts;
    int cache_c2;
    int cache_c3;
    int golden_n;
    int tmp_eid;
    int tmp_ntok;
    int tmp_bc;
    int f_valid;
    int f_cluster;
    int f_eid;
    int f_tok_start;
    int f_ntok;
    int f_s1;
    int f_s3;
    int f_skip_s1;
    int f_skip_s3;
    int f_has_s2pf;
    int f_allow_s4pf;
    int before_fail;
    int start_tid;
    int stop_tid;

    total_tests = 0;
    total_pass = 0;
    total_fail = 0;
    total_rounds = 0;
    total_plan_entries = 0;

    reg_req = '0;

    rst_ni = 1'b0;
    repeat (5) @(posedge clk_i);
    rst_ni = 1'b1;
    repeat (2) @(posedge clk_i);

    fd = $fopen(VEC_FILE, "r");
    if (fd == 0) begin
      $fatal(1, "[FAIL] cannot open %s", VEC_FILE);
    end
    if ($fscanf(fd, "%d", n_tests) != 1) begin
      $fatal(1, "[FAIL] cannot read test count from %s", VEC_FILE);
    end
    start_tid = 0;
    stop_tid = n_tests;
    void'($value$plusargs("START_TID=%d", start_tid));
    void'($value$plusargs("STOP_TID=%d", stop_tid));

    for (int t = 0; t < n_tests; t++) begin
      before_fail = total_fail;
      if ($fscanf(fd, "%d %d %d %d %d",
                  tid, n_experts, cache_c2, cache_c3, golden_n) != 5) begin
        $fatal(1, "[FAIL] malformed test header t=%0d", t);
      end

      for (int i = 0; i < MAX_N_LOCAL; i++) begin
        rem_eid[i] = '0;
        rem_ntok[i] = '0;
        rem_active[i] = 1'b0;
      end
      for (int i = 0; i < MAX_PLAN_LOCAL; i++) begin
        golden_plan[i] = '0;
      end

      for (int i = 0; i < n_experts; i++) begin
        if ($fscanf(fd, "%d %d %d", tmp_eid, tmp_ntok, tmp_bc) != 3) begin
          $fatal(1, "[FAIL] malformed rem entry tid=%0d i=%0d", tid, i);
        end
        rem_eid[i] = EID_RAW_W'(tmp_eid);
        rem_ntok[i] = NTOK_W'(tmp_ntok);
        rem_active[i] = 1'b1;
      end

      for (int i = 0; i < golden_n; i++) begin
        if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d",
                    f_valid, f_cluster, f_eid, f_tok_start, f_ntok,
                    f_s1, f_s3, f_skip_s1, f_skip_s3, f_has_s2pf,
                    f_allow_s4pf) != 11) begin
          $fatal(1, "[FAIL] malformed golden entry tid=%0d i=%0d", tid, i);
        end
        golden_plan[i].valid          = f_valid[0];
        golden_plan[i].desc.cluster   = f_cluster[0];
        golden_plan[i].desc.eid       = EID_RAW_W'(f_eid);
        golden_plan[i].desc.tok_start = NTOK_W'(f_tok_start);
        golden_plan[i].desc.ntok      = NTOK_W'(f_ntok);
        golden_plan[i].desc.s1        = f_s1[1:0];
        golden_plan[i].desc.s3        = f_s3[1:0];
        golden_plan[i].desc.skip_s1   = f_skip_s1[0];
        golden_plan[i].desc.skip_s3   = f_skip_s3[0];
        golden_plan[i].desc.has_s2pf  = f_has_s2pf[0];
        golden_plan[i].allow_s4pf     = f_allow_s4pf[0];
      end

      if ((tid < start_tid) || (tid >= stop_tid)) begin
        continue;
      end

      run_one_test(tid, n_experts, cache_c2, cache_c3, golden_n);
      total_tests++;

      if (total_fail == before_fail) begin
        $display("[PASS] tid=%0d n=%0d golden_plan=%0d", tid, n_experts, golden_n);
      end
    end

    $fclose(fd);

    if (total_fail == 0) begin
      $display("[RESULT] PASS tests=%0d rounds=%0d plan_entries=%0d",
               total_tests, total_rounds, total_plan_entries);
    end else begin
      $display("[RESULT] FAIL tests=%0d pass=%0d fail_count=%0d rounds=%0d plan_entries=%0d",
               total_tests, total_pass, total_fail, total_rounds, total_plan_entries);
    end
    $finish(total_fail == 0 ? 0 : 1);
  end

endmodule
