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
  localparam logic [47:0] ADDR_ROUND_COMMIT = 48'h68;
  localparam logic [47:0] ADDR_PLAN_FIFO_STATUS = 48'h80;
  localparam logic [47:0] ADDR_PLAN_FIFO_DATA0  = 48'h90;
  localparam logic [47:0] ADDR_PLAN_FIFO_DATA1  = 48'h98;
  localparam logic [47:0] ADDR_HEAD_PAIR0       = 48'ha8;
  localparam logic [47:0] ADDR_HEAD_PAIR1       = 48'hb0;
  localparam logic [47:0] ADDR_HEAD_PUSH_PAIR   = 48'hb8;

  localparam int unsigned TASK_BITS = $bits(task_desc_t);

  typedef struct packed {
    logic       valid;
    task_desc_t desc;
    logic       allow_s4pf;
  } plan_entry_t;

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
  logic [T_W-1:0]       rem_best_conc [MAX_N_LOCAL];
  logic                 rem_active [MAX_N_LOCAL];
  rem_head_t            sw_head [3:0];

  plan_entry_t golden_plan [MAX_PLAN_LOCAL];

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

  function automatic logic [31:0] pack_head32(input rem_head_t head);
    logic [31:0] word;
    begin
      word = '0;
      word[0 +: T_W] = head.best_conc;
      word[T_W +: NTOK_W] = head.ntok;
      word[T_W + NTOK_W +: EID_RAW_W] = head.eid;
      word[T_W + NTOK_W + EID_RAW_W] = head.valid;
      pack_head32 = word;
    end
  endfunction

  function automatic logic [63:0] pack_head_pair(input rem_head_t low, input rem_head_t high);
    pack_head_pair = {pack_head32(high), pack_head32(low)};
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

  function automatic logic [63:0] pack_round_commit(
    input bit plan_pop,
    input bit remove_ready,
    input bit start_next,
    input int push_count
  );
    logic [63:0] word;
    begin
      word = '0;
      word[0]   = plan_pop;
      word[1]   = remove_ready;
      word[2]   = start_next;
      word[5:4] = push_count[1:0];
      pack_round_commit = word;
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
        h.best_conc   = rem_best_conc[idx];
      end
      make_head_from_rem = h;
    end
  endfunction

  function automatic task_desc_t unpack_task(input logic [63:0] word);
    unpack_task = task_desc_t'(word[TASK_BITS-1:0]);
  endfunction

  function automatic logic unpack_allow_s4pf(input logic [63:0] word);
    unpack_allow_s4pf = word[TASK_BITS];
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
            heads[h].best_conc   = rem_best_conc[i];
            h++;
          end
          active_cnt++;
          total_conc += int'(rem_best_conc[i]);
        end
      end

      reg_write(ADDR_CONFIG, pack_config_word(cache_c2, cache_c3, active_cnt, total_conc));
      reg_write(ADDR_HEAD_PAIR0, pack_head_pair(heads[0], heads[1]));
      reg_write(ADDR_HEAD_PAIR1, pack_head_pair(heads[2], heads[3]));
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
    input logic got_allow,
    input plan_entry_t exp
  );
    begin
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
          got_allow !== exp.allow_s4pf) begin
        $display("[FAIL] tid=%0d round=%0d plan=%0d mismatch", tid, round_idx, plan_idx);
        $display("       got: cluster=%0d eid=%0d tok_start=%0d ntok=%0d s1=%0d s3=%0d skip_s1=%0d skip_s3=%0d has_s2pf=%0d allow_s4pf=%0d",
                 got.cluster, got.eid, got.tok_start, got.ntok, got.s1, got.s3,
                 got.skip_s1, got.skip_s3, got.has_s2pf, got_allow);
        $display("       exp: cluster=%0d eid=%0d tok_start=%0d ntok=%0d s1=%0d s3=%0d skip_s1=%0d skip_s3=%0d has_s2pf=%0d allow_s4pf=%0d",
                 exp.desc.cluster, exp.desc.eid, exp.desc.tok_start, exp.desc.ntok,
                 exp.desc.s1, exp.desc.s3, exp.desc.skip_s1,
                 exp.desc.skip_s3, exp.desc.has_s2pf, exp.allow_s4pf);
        total_fail++;
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
    int round_idx;
    int plan_seen;
    int cycles;
    int active_before;
    int fail_before;
    logic [63:0] status_word;
    logic [63:0] fifo_status_word;
    logic [63:0] plan_word [2];
    logic [1:0] remove_count;
    logic [1:0] plan_count;
    logic [1:0] plan_slot_valid;
    logic [1:0] exp_slot_valid;
    task_desc_t got_task;
    logic got_allow;
    logic [EID_RAW_W-1:0] consumed_eids [1:0];
    int consumed_count;
    int removed_count_seen;
    int active_remaining;
    int next_rem_pos;
    int unsigned total_conc;
    int removed_conc;
    int push_count;
    rem_head_t push_head;
    rem_head_t push_heads [1:0];
    begin
      fail_before = total_fail;

      active_remaining = n_experts;
      next_rem_pos = 0;
      total_conc = 0;
      for (int i = 0; i < n_experts; i++) begin
        total_conc += int'(rem_best_conc[i]);
      end

      sw_head = '{default: '0};
      for (int s = 0; s < 4; s++) begin
        if (next_rem_pos < n_experts) begin
          sw_head[s] = make_head_from_rem(next_rem_pos);
          next_rem_pos++;
        end
      end

      reg_write(ADDR_CONFIG, pack_config_word(cache_c2, cache_c3, active_remaining, total_conc));
      reg_write(ADDR_HEAD_PAIR0, pack_head_pair(sw_head[0], sw_head[1]));
      reg_write(ADDR_HEAD_PAIR1, pack_head_pair(sw_head[2], sw_head[3]));
      reg_write(ADDR_CTRL, 64'h3);

      round_idx = 0;
      plan_seen = 0;

      while (active_remaining > 0) begin
        active_before = active_remaining;

        cycles = 0;
        do begin
          reg_read(ADDR_STATUS, status_word);
          cycles++;
        end while ((status_word[1] !== 1'b1) && (cycles < ROUND_TIMEOUT));

        if (status_word[1] !== 1'b1) begin
          $display("[FAIL] tid=%0d round=%0d timeout active=%0d",
                   tid, round_idx, active_before);
          total_fail++;
          return;
        end

        if (status_word[2] !== 1'b1) begin
          fail_msg("STATUS.remove_valid not asserted", tid, round_idx);
        end
        if (status_word[3] !== 1'b1) begin
          fail_msg("STATUS.plan_valid not asserted", tid, round_idx);
        end

        reg_read(ADDR_PLAN_FIFO_STATUS, fifo_status_word);
        remove_count = status_word[9:8];
        plan_count = fifo_status_word[9:8];
        plan_slot_valid = fifo_status_word[17:16];

        if (fifo_status_word[0] !== 1'b0) begin
          fail_msg("PLAN_FIFO_STATUS.empty asserted despite plan_valid", tid, round_idx);
        end
        if (status_word[17:16] !== plan_count) begin
          $display("[FAIL] tid=%0d round=%0d STATUS/FIFO plan_count mismatch status=%0d fifo=%0d",
                   tid, round_idx, status_word[17:16], plan_count);
          total_fail++;
        end
        if (fifo_status_word[13:12] !== remove_count) begin
          $display("[FAIL] tid=%0d round=%0d STATUS/FIFO remove_count mismatch fifo=%0d status=%0d",
                   tid, round_idx, fifo_status_word[13:12], remove_count);
          total_fail++;
        end
        if (status_word[25:24] !== plan_slot_valid) begin
          $display("[FAIL] tid=%0d round=%0d STATUS/FIFO slot_valid mismatch status=%b fifo=%b",
                   tid, round_idx, status_word[25:24], plan_slot_valid);
          total_fail++;
        end

        if ((plan_count == 2'd0) || (plan_count > 2'd2)) begin
          fail_msg("illegal plan_count", tid, round_idx);
        end

        exp_slot_valid = (plan_count == 2'd2) ? 2'b11 :
                         (plan_count == 2'd1) ? 2'b01 : 2'b00;
        if (plan_slot_valid !== exp_slot_valid) begin
          $display("[FAIL] tid=%0d round=%0d slot_valid got=%b exp=%b",
                   tid, round_idx, plan_slot_valid, exp_slot_valid);
          total_fail++;
        end

        consumed_eids = '{default: '0};
        consumed_count = 0;
        if (plan_seen + int'(plan_count) > golden_n) begin
          $display("[FAIL] tid=%0d round=%0d produced too many plan entries seen=%0d count=%0d golden=%0d",
                   tid, round_idx, plan_seen, plan_count, golden_n);
          total_fail++;
        end else begin
          reg_read(ADDR_PLAN_FIFO_DATA0, plan_word[0]);
          if (plan_count == 2'd2) begin
            reg_read(ADDR_PLAN_FIFO_DATA1, plan_word[1]);
          end
          for (int s = 0; s < 2; s++) begin
            if (s < int'(plan_count)) begin
              got_task = unpack_task(plan_word[s]);
              got_allow = unpack_allow_s4pf(plan_word[s]);
              compare_task(tid, round_idx, plan_seen + s, got_task, got_allow,
                           golden_plan[plan_seen + s]);
              note_consumed_eid(got_task.eid, consumed_eids, consumed_count,
                                tid, round_idx);
            end
          end
        end

        if ((remove_count == 2'd0) || (remove_count > 2'd2) ||
            (int'(remove_count) > active_remaining)) begin
          fail_msg("illegal remove_count", tid, round_idx);
        end else if (consumed_count != int'(remove_count)) begin
          $display("[FAIL] tid=%0d round=%0d remove_count/plan-eid mismatch remove=%0d unique_eid=%0d",
                   tid, round_idx, remove_count, consumed_count);
          total_fail++;
        end else begin
          removed_conc = 0;
          removed_count_seen = 0;
          for (int s = 0; s < 4; s++) begin
            if (sw_head[s].valid &&
                consumed_eid_match(sw_head[s].eid, consumed_eids, consumed_count)) begin
              removed_count_seen++;
              removed_conc += int'(sw_head[s].best_conc);
              remove_one(int'(sw_head[s].rem_index), tid, round_idx);
            end
          end
          if (removed_count_seen != int'(remove_count)) begin
            $display("[FAIL] tid=%0d round=%0d removed head count mismatch got=%0d exp=%0d",
                     tid, round_idx, removed_count_seen, remove_count);
            total_fail++;
          end
          for (int e = 0; e < int'(remove_count); e++) begin
            bit found;
            found = 1'b0;
            for (int s = 0; s < 4; s++) begin
              if (!sw_head[s].valid) begin
                continue;
              end
              if (sw_head[s].eid == consumed_eids[e]) begin
                found = 1'b1;
              end
            end
            if (!found) begin
              $display("[FAIL] tid=%0d round=%0d consumed eid=%0d not in top window",
                       tid, round_idx, consumed_eids[e]);
              total_fail++;
            end
          end
          active_remaining -= int'(remove_count);
          total_conc -= removed_conc;
        end

        plan_seen += int'(plan_count);
        round_idx++;
        total_rounds++;

        // ROUND_COMMIT pops the completed plan, compacts the hardware top
        // window, consumes staged head pushes, and optionally starts the next
        // round in one fixed-order RTL transaction.
        compact_sw_head(consumed_eids, consumed_count);

        push_count = 0;
        push_heads = '{default: '0};
        for (int p = 0; p < int'(remove_count); p++) begin
          if (next_rem_pos < n_experts) begin
            push_head = make_head_from_rem(next_rem_pos);
            append_sw_head(push_head);
            push_heads[push_count] = push_head;
            next_rem_pos++;
            push_count++;
          end
        end
        if (push_count != 0) begin
          reg_write(ADDR_HEAD_PUSH_PAIR, pack_head_pair(push_heads[0], push_heads[1]));
        end
        if (active_remaining > 0) begin
          reg_write(ADDR_ROUND_COMMIT,
                    pack_round_commit(1'b1, 1'b1, 1'b1, push_count));
        end else begin
          reg_write(ADDR_ROUND_COMMIT,
                    pack_round_commit(1'b1, 1'b1, 1'b0, push_count));
        end
      end

      if (plan_seen != golden_n) begin
        $display("[FAIL] tid=%0d final plan count got=%0d exp=%0d",
                 tid, plan_seen, golden_n);
        total_fail++;
      end

      total_plan_entries += plan_seen;
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

    for (int t = 0; t < n_tests; t++) begin
      before_fail = total_fail;
      if ($fscanf(fd, "%d %d %d %d %d",
                  tid, n_experts, cache_c2, cache_c3, golden_n) != 5) begin
        $fatal(1, "[FAIL] malformed test header t=%0d", t);
      end

      for (int i = 0; i < MAX_N_LOCAL; i++) begin
        rem_eid[i] = '0;
        rem_ntok[i] = '0;
        rem_best_conc[i] = '0;
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
        rem_best_conc[i] = T_W'(tmp_bc);
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
