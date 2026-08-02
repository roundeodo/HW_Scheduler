`timescale 1ns/1ps

module tb_distilled_round_engine;
  import sched_pkg::*;
  import sched_distilled_pkg::*;

  logic clk_i;
  logic rst_ni;
  logic clear_i;
  logic start_i;
  logic done_o;
  logic feasible_o;
  head_ctx_t [7:0] hot_i;
  head_ctx_t bottom_i;
  distilled_cluster_state_t base_c2_i;
  distilled_cluster_state_t base_c3_i;
  distilled_counters_t counters_i;
  distilled_cluster_state_t child_c2_o;
  distilled_cluster_state_t child_c3_o;
  distilled_counters_t child_counters_o;
  winner_plan_t plan_o;
  logic [1:0] remove_count_o;
  logic [EID_RAW_W-1:0] remove_eid_a_o;
  logic [EID_RAW_W-1:0] remove_eid_b_o;
  distilled_action_token_t selected_token_o;
  distilled_score_record_t selected_score_o;

  int fd;
  int vectors;
  int row;
  int failures;
  int c2 [0:15];
  int c3 [0:15];
  int counters [0:8];
  int descriptor [0:8][0:2];
  int selected [0:6];
  int cc2 [0:15];
  int cc3 [0:15];
  int child_counters [0:8];
  int plan [0:22];
  int remove [0:2];
  int score [0:13];
  string vector_file;

  sched_distilled_round_engine dut (.*);
  always #5 clk_i = ~clk_i;

  function automatic distilled_cluster_state_t make_state(input int v [0:15]);
    distilled_cluster_state_t state;
    begin
      state = '0;
      state.cur_valid = v[0];
      state.task_start = time_t'(v[2]);
      state.task_end = time_t'(v[3]);
      state.dma1_end = time_t'(v[4]);
      state.s2_end = time_t'(v[5]);
      state.dma3_end = time_t'(v[6]);
      state.dma_s1 = dma_binding_t'(v[8]);
      state.dma_s3 = dma_binding_t'(v[9]);
      state.s2pf_dma = dma_binding_t'(v[10]);
      state.cache_valid = v[12];
      state.cache_eid = EID_RAW_W'(v[13]);
      make_state = state;
    end
  endfunction

  task automatic load_counters(
    output distilled_counters_t target,
    input int v [0:8]
  );
    target = '0;
    target.count = NR_W'(v[0]);
    target.token_sum = DIST_TOKEN_SUM_W'(v[1]);
    target.odd_count = DIST_HIST_W'(v[2]);
    target.block_sum = DIST_BLOCK_SUM_W'(v[3]);
    for (int bucket = 0; bucket < 4; bucket++)
      target.small_hist[bucket] = DIST_HIST_W'(v[4+bucket]);
    target.parent_bound = distilled_bound_t'(v[8]);
  endtask

  task automatic check(input string name, input int got, input int exp);
    if (got !== exp) begin
      $display("[FAIL] row=%0d %s got=%0d expected=%0d", row, name, got, exp);
      failures++;
    end
  endtask

  task automatic check_state(
    input string name,
    input distilled_cluster_state_t got,
    input int exp [0:15]
  );
    check({name,".valid"}, got.cur_valid, exp[0]);
    check({name,".start"}, got.task_start, exp[2]);
    check({name,".end"}, got.task_end, exp[3]);
    check({name,".dma1"}, got.dma1_end, exp[4]);
    check({name,".s2"}, got.s2_end, exp[5]);
    check({name,".dma3"}, got.dma3_end, exp[6]);
    check({name,".compute"}, got.task_end, exp[7]);
    check({name,".s1bind"}, got.dma_s1, exp[8]);
    check({name,".s3bind"}, got.dma_s3, exp[9]);
    check({name,".s2pfbind"}, got.s2pf_dma, exp[10]);
    if (got.s2pf_dma != DMA_NONE)
      check({name,".s2pfend"},
            got.dma1_end + s3_dma_ticks(got.s2pf_dma), exp[11]);
    check({name,".cachev"}, got.cache_valid, exp[12]);
    check({name,".cacheeid"}, got.cache_eid, exp[13]);
    if (exp[12]) begin
      check({name,".cacheend-invariant"}, exp[14], 0);
      check({name,".cachefull-invariant"}, exp[15], 1);
    end
  endtask

  initial begin
    clk_i = 1'b0;
    rst_ni = 1'b0;
    clear_i = 1'b0;
    start_i = 1'b0;
    hot_i = '{default: '0};
    bottom_i = '0;
    base_c2_i = '0;
    base_c3_i = '0;
    counters_i = '0;
    failures = 0;
    repeat (4) @(negedge clk_i);
    rst_ni = 1'b1;

    vector_file = "distilled_round_vectors.txt";
    void'($value$plusargs("VECTOR_FILE=%s", vector_file));
    fd = $fopen(vector_file, "r");
    if (fd == 0) $fatal(1, "cannot open round vectors");
    if ($fscanf(fd, "%d\n", vectors) != 1) $fatal(1, "missing vector count");
    for (row = 0; row < vectors; row++) begin
      int wait_cycles;
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          c2[0], c2[1], c2[2], c2[3], c2[4], c2[5], c2[6], c2[7],
          c2[8], c2[9], c2[10], c2[11], c2[12], c2[13], c2[14], c2[15]) != 16)
        $fatal(1, "bad c2 row=%0d", row);
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          c3[0], c3[1], c3[2], c3[3], c3[4], c3[5], c3[6], c3[7],
          c3[8], c3[9], c3[10], c3[11], c3[12], c3[13], c3[14], c3[15]) != 16)
        $fatal(1, "bad c3 row=%0d", row);
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d\n",
          counters[0], counters[1], counters[2], counters[3], counters[4],
          counters[5], counters[6], counters[7], counters[8]) != 9)
        $fatal(1, "bad counters row=%0d", row);
      for (int slot = 0; slot < 9; slot++)
        if ($fscanf(fd, "%d %d %d\n", descriptor[slot][0],
                   descriptor[slot][1], descriptor[slot][2]) != 3)
          $fatal(1, "bad descriptor row=%0d", row);
      if ($fscanf(fd, "%d %d %d %d %d %d %d\n",
          selected[0], selected[1], selected[2], selected[3], selected[4],
          selected[5], selected[6]) != 7) $fatal(1, "bad selected");
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          cc2[0], cc2[1], cc2[2], cc2[3], cc2[4], cc2[5], cc2[6], cc2[7],
          cc2[8], cc2[9], cc2[10], cc2[11], cc2[12], cc2[13], cc2[14], cc2[15]) != 16)
        $fatal(1, "bad child c2");
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          cc3[0], cc3[1], cc3[2], cc3[3], cc3[4], cc3[5], cc3[6], cc3[7],
          cc3[8], cc3[9], cc3[10], cc3[11], cc3[12], cc3[13], cc3[14], cc3[15]) != 16)
        $fatal(1, "bad child c3");
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d\n",
          child_counters[0], child_counters[1], child_counters[2],
          child_counters[3], child_counters[4], child_counters[5],
          child_counters[6], child_counters[7], child_counters[8]) != 9)
        $fatal(1, "bad child counters");
      if ($fscanf(fd,
          "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          plan[0], plan[1], plan[2], plan[3], plan[4], plan[5], plan[6],
          plan[7], plan[8], plan[9], plan[10], plan[11], plan[12], plan[13],
          plan[14], plan[15], plan[16], plan[17], plan[18], plan[19], plan[20],
          plan[21], plan[22]) != 23) $fatal(1, "bad plan");
      if ($fscanf(fd, "%d %d %d\n", remove[0], remove[1], remove[2]) != 3)
        $fatal(1, "bad remove");
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          score[0], score[1], score[2], score[3], score[4], score[5],
          score[6], score[7], score[8], score[9], score[10], score[11],
          score[12], score[13]) != 14) $fatal(1, "bad score");

      base_c2_i = make_state(c2);
      base_c3_i = make_state(c3);
      load_counters(counters_i, counters);
      for (int slot = 0; slot < 8; slot++) begin
        hot_i[slot].valid = descriptor[slot][0];
        hot_i[slot].eid = EID_RAW_W'(descriptor[slot][1]);
        hot_i[slot].ntok = ntok_t'(descriptor[slot][2]);
      end
      bottom_i.valid = descriptor[8][0];
      bottom_i.eid = EID_RAW_W'(descriptor[8][1]);
      bottom_i.ntok = ntok_t'(descriptor[8][2]);

      @(negedge clk_i); start_i = 1'b1;
      @(negedge clk_i); start_i = 1'b0;
      wait_cycles = 0;
      while (!done_o && wait_cycles < 5000) begin
        @(negedge clk_i);
        wait_cycles++;
      end
      if (!done_o) begin
        $display("[FAIL] timeout row=%0d round_st=%0d profile=%0d iter_st=%0d source=%0d previous=%0d minimum=%0d min_valid=%0d eval_st=%0d bound_st=%0d compare_st=%0d",
                 row, dut.st_q, dut.profile_addr_q,
                 dut.i_start_iter.st_q, dut.i_start_iter.source_q,
                 dut.i_start_iter.previous_q, dut.i_start_iter.minimum_q,
                 dut.i_start_iter.minimum_valid_q, dut.i_transition_eval.st_q,
                 dut.i_bound_score.st_q, dut.i_pair_compare.st_q);
        $fatal(1, "round timeout row=%0d", row);
      end
      check("feasible", feasible_o, 1);
      check("profile", selected_token_o.profile_slot, selected[0]);
      check("mode_index", selected_token_o.mode_index, selected[1]);
      check("logical", selected_token_o.logical_id, selected[2]);
      check("swap", selected_token_o.assignment_swap, selected[3]);
      check("start", selected_token_o.start, selected[4]);
      check("s4pf_c2", selected_token_o.targeted_s4pf_c2, selected[5]);
      check("s4pf_c3", selected_token_o.targeted_s4pf_c3, selected[6]);
      check_state("c2", child_c2_o, cc2);
      check_state("c3", child_c3_o, cc3);
      check("remove_count", remove_count_o, remove[0]);
      check("remove_a", remove_eid_a_o, remove[1]);
      check("remove_b", remove_eid_b_o, remove[2]);
      check("counter.count", child_counters_o.count, child_counters[0]);
      check("counter.token", child_counters_o.token_sum, child_counters[1]);
      check("counter.odd", child_counters_o.odd_count, child_counters[2]);
      check("counter.blocks", child_counters_o.block_sum, child_counters[3]);
      for (int bucket = 0; bucket < 4; bucket++)
        check("counter.hist", child_counters_o.small_hist[bucket],
              child_counters[4+bucket]);
      check("counter.parent", child_counters_o.parent_bound, child_counters[8]);
      check("score.F", selected_score_o.f, score[0]);
      check("score.H", selected_score_o.h, score[1]);
      check("score.C", selected_score_o.compute_bound, score[2]);
      check("score.D", selected_score_o.dma_bound, score[3]);
      check("plan.valid", plan_o.task_valid, plan[0]);
      for (int task_index = 0; task_index < 2; task_index++) begin
        int base;
        base = 1 + task_index * 11;
        check("plan.eid", plan_o.token[task_index].eid, plan[base]);
        check("plan.ntok", plan_o.token[task_index].ntok, plan[base+1]);
        check("plan.tok_start", plan_o.token[task_index].tok_start, plan[base+2]);
        check("plan.cluster", plan_o.ctrl[task_index].cluster, plan[base+3]);
        check("plan.s1", plan_o.ctrl[task_index].shape_s1, plan[base+4]);
        check("plan.s3", plan_o.ctrl[task_index].shape_s3, plan[base+5]);
        check("plan.skip1", plan_o.ctrl[task_index].skip_s1, plan[base+6]);
        check("plan.skip3", plan_o.ctrl[task_index].skip_s3, plan[base+7]);
        check("plan.s2pf", plan_o.ctrl[task_index].has_s2pf, plan[base+8]);
        check("plan.s1both", plan_o.ctrl[task_index].dma_s1_both, plan[base+9]);
        check("plan.lateboth", plan_o.ctrl[task_index].dma_late_both,
              plan[base+10]);
      end
      @(negedge clk_i);
    end
    $fclose(fd);
    if (failures == 0)
      $display("[RESULT] PASS distilled_round_engine vectors=%0d", vectors);
    else
      $display("[RESULT] FAIL distilled_round_engine failures=%0d", failures);
    $finish;
  end
endmodule
