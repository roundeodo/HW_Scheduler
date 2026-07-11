// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// tb_schedule_core.sv
// ---------------------------------------------------------------------------
// End-to-end datapath test for sched_schedule_core.
//
// This testbench models the CVA6-side software loop around the pure-slave
// scheduler core:
//   1. keep the full sorted rem list outside RTL,
//   2. provide top4 + aggregate context before each round,
//   3. consume remove metadata after each round,
//   4. pop compact plan output and compare it with moe_make_hw_plan().

`timescale 1ns/1ps
import sched_pkg::*;

module tb_schedule_core;

  localparam int unsigned MAX_N_LOCAL    = E_MAX;
  localparam int unsigned MAX_PLAN_LOCAL = 2 * E_MAX;
  localparam string       VEC_FILE       = "schedule_vectors.txt";
  localparam int unsigned ROUND_TIMEOUT  = 20000;

  typedef struct packed {
    logic       valid;
    task_desc_t desc;
    logic       allow_s4pf;
  } plan_entry_t;

  function automatic logic [7:0] tb_pack_s4pf_desc(
    input logic     valid,
    input logic     no_copy,
    input logic [EID_RAW_W-1:0] target_eid
  );
    logic [7:0] word;
    begin
      word = '0;
      word[S4PF_DESC_VALID_LSB] = valid;
      word[S4PF_DESC_NO_COPY_LSB] = no_copy;
      word[S4PF_DESC_TARGET_EID_LSB +: EID_RAW_W] = target_eid;
      tb_pack_s4pf_desc = word;
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

  function automatic logic [63:0] tb_pack_plan_entry_word(
    input task_desc_t t,
    input slot_id_t   local_slot,
    input logic [7:0] s4pf_desc
  );
    logic [63:0] word;
    logic [TASK_WORD_CTRL_W-1:0] ctrl;
    ntok_t tail_s2;
    ntok_t tail_s4;
    begin
      tail_s2 = t.skip_s1 ? t.ntok :
                ((t.ntok > tb_shape_mdim(t.s1)) ? (t.ntok - tb_shape_mdim(t.s1)) : '0);
      tail_s4 = t.skip_s3 ? t.ntok :
                ((t.ntok > tb_shape_mdim(t.s3)) ? (t.ntok - tb_shape_mdim(t.s3)) : '0);

      ctrl = '0;
      ctrl[0]    = t.skip_s1;
      ctrl[1]    = t.skip_s3;
      ctrl[3:2]  = t.s1;
      ctrl[5:4]  = t.s3;
      ctrl[6]    = t.cluster;
      ctrl[12:7] = local_slot;

      word = '0;
      word[TASK_WORD_EID_LSB +: EID_RAW_W]          = t.eid;
      word[TASK_WORD_TOKEN_START_LSB +: NTOK_W]     = t.tok_start;
      word[TASK_WORD_NTOK_LSB +: NTOK_W]            = t.ntok;
      word[TASK_WORD_HAS_S2PF_LSB]                  = t.has_s2pf;
      word[TASK_WORD_CTRL_LSB +: TASK_WORD_CTRL_W]       = ctrl;
      word[TASK_WORD_M_S2_LSB +: NTOK_W]            = ceil_div2_ntok(tail_s2);
      word[TASK_WORD_M_S4_LSB +: NTOK_W]            = ceil_div2_ntok(tail_s4);
      word[TASK_WORD_S4PF_DESC_LSB +: 8]            = s4pf_desc;
      tb_pack_plan_entry_word = word;
    end
  endfunction

  logic clk_i;
  logic rst_ni;

  logic init_i;
  logic start_i;
  logic [7:0] cache_eid_c2_i;
  logic [7:0] cache_eid_c3_i;

  head_ctx_t [3:0] head_i;
  logic [NR_W-1:0] head_rem_index [3:0];
  logic [NR_W-1:0] active_count_i;
  logic [T_W-1:0] total_conc_i;

  logic remove_ready_i;
  logic remove_valid_o;
  logic [1:0] remove_count_o;
  logic [3:0] remove_slot_mask_o;

  logic [3:0] task_pop_count_i;
  logic [$clog2(TASKQ_DEPTH)-1:0] task_rd_index_i;
  logic task_valid_o;
  logic [63:0] task_rd_data_o;
  logic task_queue_full_o;
  logic [3:0] task_queue_count_o;

  logic busy_o;
  logic done_o;

  sched_schedule_core dut (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .init_i             (init_i),
    .start_i            (start_i),
    .cache_eid_c2_i     (cache_eid_c2_i),
    .cache_eid_c3_i     (cache_eid_c3_i),
    .head_i             (head_i),
    .active_count_i     (active_count_i),
    .total_conc_i       (total_conc_i),
    .remove_ready_i     (remove_ready_i),
    .remove_valid_o     (remove_valid_o),
    .remove_count_o     (remove_count_o),
    .remove_slot_mask_o (remove_slot_mask_o),
    .task_pop_count_i   (task_pop_count_i),
    .task_rd_index_i    (task_rd_index_i),
    .task_valid_o       (task_valid_o),
    .task_rd_data_o     (task_rd_data_o),
    .task_queue_full_o  (task_queue_full_o),
    .task_queue_count_o (task_queue_count_o),
    .busy_o             (busy_o),
    .done_o             (done_o)
  );

  logic [EID_RAW_W-1:0] rem_eid [MAX_N_LOCAL];
  logic [NTOK_W-1:0]    rem_ntok [MAX_N_LOCAL];
  logic                 rem_active [MAX_N_LOCAL];

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

  task automatic drive_round_context(input int n_experts);
    int h;
    int active_cnt;
    int unsigned total_conc;
    begin
      head_i = '{default: '0};
      head_rem_index = '{default: '0};
      h = 0;
      active_cnt = 0;
      total_conc = 0;

      for (int i = 0; i < MAX_N_LOCAL; i++) begin
        if ((i < n_experts) && rem_active[i]) begin
          if (h < 4) begin
            head_i[h].valid       = 1'b1;
            head_i[h].eid         = rem_eid[i];
            head_i[h].ntok        = rem_ntok[i];
            head_rem_index[h]     = NR_W'(i);
            h++;
          end
          active_cnt++;
          total_conc += int'(best_conc_ticks(rem_ntok[i]));
        end
      end

      active_count_i = NR_W'(active_cnt);
      total_conc_i   = T_W'(total_conc);
    end
  endtask

  task automatic fail_msg(input string msg, input int tid, input int round_idx);
    begin
      $display("[FAIL] tid=%0d round=%0d %s", tid, round_idx, msg);
      total_fail++;
    end
  endtask

  task automatic compare_task_word(
    input int tid,
    input int round_idx,
    input logic [63:0] got_word,
    input int golden_n,
    input int cache_c2,
    input int cache_c3
  );
    int ci;
    int local_slot;
    int match_idx;
    int cluster_pos;
    int next_idx;
    logic cache_hit;
    logic no_copy;
    logic [7:0] s4pf_desc;
    logic [63:0] exp_word;
    begin
      ci = int'(got_word[TASK_WORD_CTRL_LSB + 6]);
      local_slot = int'(got_word[TASK_WORD_CTRL_LSB + 7 +: SLOT_W]);
      match_idx = -1;
      cluster_pos = 0;
      for (int i = 0; i < golden_n; i++) begin
        if (int'(golden_plan[i].desc.cluster) == ci) begin
          if (cluster_pos == local_slot) begin
            match_idx = i;
          end
          cluster_pos++;
        end
      end

      if (match_idx < 0) begin
        $display("[FAIL] tid=%0d round=%0d no golden task for C%0d slot=%0d",
                 tid, round_idx, ci + 2, local_slot);
        total_fail++;
      end else begin
        if (task_seen[ci][local_slot]) begin
          $display("[FAIL] tid=%0d round=%0d duplicate C%0d slot=%0d",
                   tid, round_idx, ci + 2, local_slot);
          total_fail++;
        end
        task_seen[ci][local_slot] = 1'b1;
        s4pf_desc = '0;
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
            s4pf_desc = tb_pack_s4pf_desc(
                1'b1, no_copy, golden_plan[next_idx].desc.eid);
          end
        end
        exp_word = tb_pack_plan_entry_word(
            golden_plan[match_idx].desc, slot_id_t'(local_slot), s4pf_desc);
        if (got_word !== exp_word) begin
          $display("[FAIL] tid=%0d round=%0d C%0d slot=%0d packed word mismatch",
                   tid, round_idx, ci + 2, local_slot);
        $display("       got=0x%016h exp=0x%016h", got_word, exp_word);
        $display("       exp desc: cluster=%0d eid=%0d tok_start=%0d ntok=%0d s1=%0d s3=%0d skip_s1=%0d skip_s3=%0d has_s2pf=%0d allow_s4pf=%0d",
                   golden_plan[match_idx].desc.cluster,
                   golden_plan[match_idx].desc.eid,
                   golden_plan[match_idx].desc.tok_start,
                   golden_plan[match_idx].desc.ntok,
                   golden_plan[match_idx].desc.s1,
                   golden_plan[match_idx].desc.s3,
                   golden_plan[match_idx].desc.skip_s1,
                   golden_plan[match_idx].desc.skip_s3,
                   golden_plan[match_idx].desc.has_s2pf,
                   golden_plan[match_idx].allow_s4pf);
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

  function automatic int popcount4(input logic [3:0] mask);
    popcount4 = int'(mask[0]) + int'(mask[1]) + int'(mask[2]) + int'(mask[3]);
  endfunction

  task automatic remove_slots(input logic [3:0] mask, input int tid, input int round_idx);
    begin
      for (int s = 0; s < 4; s++) begin
        if (mask[s]) begin
          if (!head_i[s].valid) begin
            $display("[FAIL] tid=%0d round=%0d remove invalid head slot=%0d mask=%b",
                     tid, round_idx, s, mask);
            total_fail++;
          end else begin
            remove_one(int'(head_rem_index[s]), tid, round_idx);
          end
        end
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
    int drain_tasks;
    begin
      fail_before = total_fail;
      cache_eid_c2_i = cache_to_rtl(cache_c2);
      cache_eid_c3_i = cache_to_rtl(cache_c3);

      @(negedge clk_i);
      init_i = 1'b1;
      start_i = 1'b0;
      remove_ready_i = 1'b0;
      task_pop_count_i = 4'd0;
      task_rd_index_i = '0;
      @(negedge clk_i);
      init_i = 1'b0;

      round_idx = 0;
      plan_seen = 0;
      for (int ci = 0; ci < 2; ci++) begin
        for (int slot = 0; slot < (1 << SLOT_W); slot++) begin
          task_seen[ci][slot] = 1'b0;
        end
      end

      while (count_active(n_experts) > 0) begin
        drive_round_context(n_experts);
        active_before = int'(active_count_i);

        @(negedge clk_i);
        start_i = 1'b1;
        @(negedge clk_i);
        start_i = 1'b0;

        cycles = 0;
        while (cycles < ROUND_TIMEOUT) begin
          @(posedge clk_i);
          #1;
          if (done_o === 1'b1) begin
            break;
          end
          cycles++;
        end
        if (done_o !== 1'b1) begin
          $display("[FAIL] tid=%0d round=%0d timeout active=%0d",
                   tid, round_idx, active_before);
          total_fail++;
          return;
        end

        if (remove_valid_o !== 1'b1) begin
          fail_msg("remove_valid_o not asserted", tid, round_idx);
        end
        drain_tasks = int'(task_queue_count_o);
        if (plan_seen + drain_tasks > golden_n) begin
          $display("[FAIL] tid=%0d round=%0d produced too many tasks seen=%0d count=%0d golden=%0d",
                   tid, round_idx, plan_seen, drain_tasks, golden_n);
          total_fail++;
        end else begin
          for (int ti = 0; ti < drain_tasks; ti++) begin
            task_rd_index_i = $clog2(TASKQ_DEPTH)'(ti);
            #1;
            compare_task_word(tid, round_idx, task_rd_data_o,
                              golden_n, cache_c2, cache_c3);
          end
        end

        if ((remove_count_o == 2'd0) || (remove_count_o > 2'd2) ||
            (popcount4(remove_slot_mask_o) != int'(remove_count_o))) begin
          fail_msg("illegal remove_count_o", tid, round_idx);
        end else begin
          remove_slots(remove_slot_mask_o, tid, round_idx);
        end

        plan_seen += drain_tasks;
        round_idx++;
        total_rounds++;

        @(negedge clk_i);
        remove_ready_i = 1'b1;
        task_pop_count_i = 4'(drain_tasks);
        @(negedge clk_i);
        remove_ready_i = 1'b0;
        task_pop_count_i = 4'd0;
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
    int start_tid;
    int stop_tid;

    total_tests = 0;
    total_pass = 0;
    total_fail = 0;
    total_rounds = 0;
    total_plan_entries = 0;

    init_i = 1'b0;
    start_i = 1'b0;
    cache_eid_c2_i = '0;
    cache_eid_c3_i = '0;
    head_i = '{default: '0};
    head_rem_index = '{default: '0};
    active_count_i = '0;
    total_conc_i = '0;
    remove_ready_i = 1'b0;
    task_pop_count_i = 4'd0;
    task_rd_index_i = '0;

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
        golden_plan[i].valid        = f_valid[0];
        golden_plan[i].desc.cluster = f_cluster[0];
        golden_plan[i].desc.eid     = EID_RAW_W'(f_eid);
        golden_plan[i].desc.tok_start = NTOK_W'(f_tok_start);
        golden_plan[i].desc.ntok    = NTOK_W'(f_ntok);
        golden_plan[i].desc.s1      = f_s1[1:0];
        golden_plan[i].desc.s3      = f_s3[1:0];
        golden_plan[i].desc.skip_s1 = f_skip_s1[0];
        golden_plan[i].desc.skip_s3 = f_skip_s3[0];
        golden_plan[i].desc.has_s2pf = f_has_s2pf[0];
        golden_plan[i].allow_s4pf   = f_allow_s4pf[0];
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
