`timescale 1ns/1ps

module tb_distilled_transition_eval;
  import sched_pkg::*;
  import sched_distilled_pkg::*;

  logic clk_i;
  logic rst_ni;
  logic clear_i;
  logic start_i;
  logic done_o;
  logic feasible_o;
  distilled_mode_t mode_i;
  logic [4:0] mode_index_i;
  logic [4:0] profile_addr_i;
  distilled_profile_t profile_i;
  logic assignment_swap_i;
  time_t start_time_i;
  logic incremental_i;
  logic rebuild_only_i;
  logic gain_ok_i;
  logic force_s1_hit_c2_i;
  logic force_s1_hit_c3_i;
  head_ctx_t [4:0] top_i;
  head_ctx_t bottom_i;
  head_ctx_t selected_a_i;
  head_ctx_t selected_b_i;
  distilled_cluster_state_t base_c2_i;
  distilled_cluster_state_t base_c3_i;
  logic bw_start_o;
  snap_bw_view_t bw_c2_o;
  snap_bw_view_t bw_c3_o;
  logic bw_done_i;
  logic bw_ok_i;
  distilled_cluster_state_t child_c2_o;
  distilled_cluster_state_t child_c3_o;
  winner_plan_t plan_o;
  logic [1:0] remove_count_o;
  logic [EID_RAW_W-1:0] remove_eid_a_o;
  logic [EID_RAW_W-1:0] remove_eid_b_o;
  ntok_t selected_max_o;
  ntok_t selected_sum_o;
  logic [1:0] s2pf_count_o;
  time_t latest_start_o;

  int fd;
  int case_count;
  int failures;
  int current_case;
  int current_round;
  int header [0:12];
  int desc [0:5][0:2];
  int exp_c2 [0:15];
  int exp_c3 [0:15];
  int exp_plan [0:22];

  sched_distilled_profile_decode i_decode (
    .profile_addr_i, .profile_o(profile_i)
  );

  function automatic head_ctx_t resolve_selector(
    input distilled_selector_t selector,
    input head_ctx_t [4:0]     top,
    input head_ctx_t           bottom
  );
    unique case (selector)
      DIST_SEL_T0: resolve_selector = top[0];
      DIST_SEL_T1: resolve_selector = top[1];
      DIST_SEL_T2: resolve_selector = top[2];
      DIST_SEL_T3: resolve_selector = top[3];
      DIST_SEL_T4: resolve_selector = top[4];
      DIST_SEL_B0: resolve_selector = bottom;
      default:     resolve_selector = '0;
    endcase
  endfunction

  assign selected_a_i = resolve_selector(profile_i.selector_a, top_i, bottom_i);
  assign selected_b_i = resolve_selector(profile_i.selector_b, top_i, bottom_i);

  sched_distilled_transition_eval dut (.*);

  sched_bandwidth_check i_bandwidth_check (
    .clk_i,
    .rst_ni,
    .clear_i,
    .start_i  (bw_start_o),
    .done_o   (bw_done_i),
    .snap_a_i (bw_c2_o),
    .snap_b_i (bw_c3_o),
    .ok_o     (bw_ok_i)
  );

  always #5 clk_i = ~clk_i;

  task automatic check(input string name, input int got, input int expected);
    if (got !== expected) begin
      $display("[FAIL] case=%0d round=%0d %s got=%0d expected=%0d",
               current_case, current_round, name, got, expected);
      failures++;
    end
  endtask

  function automatic distilled_cluster_state_t expected_state(input int v [0:15]);
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
      expected_state = state;
    end
  endfunction

  task automatic check_state(
    input string name,
    input distilled_cluster_state_t got,
    input distilled_cluster_state_t expected
  );
    if (got !== expected) begin
      $display("[FAIL] case=%0d round=%0d %s child mismatch got=%h expected=%h",
               current_case, current_round, name, got, expected);
      failures++;
    end
  endtask

  task automatic check_plan;
    int base;
    begin
      check("plan.valid", plan_o.task_valid, exp_plan[0]);
      for (int slot = 0; slot < 2; slot++) begin
        base = 1 + slot * 11;
        if (exp_plan[0] & (1 << slot)) begin
          check($sformatf("plan%0d.eid", slot), plan_o.token[slot].eid, exp_plan[base]);
          check($sformatf("plan%0d.ntok", slot), plan_o.token[slot].ntok, exp_plan[base+1]);
          check($sformatf("plan%0d.start", slot), plan_o.token[slot].tok_start, exp_plan[base+2]);
          check($sformatf("plan%0d.cluster", slot), plan_o.ctrl[slot].cluster, exp_plan[base+3]);
          check($sformatf("plan%0d.s1", slot), plan_o.ctrl[slot].shape_s1, exp_plan[base+4]);
          check($sformatf("plan%0d.s3", slot), plan_o.ctrl[slot].shape_s3, exp_plan[base+5]);
          check($sformatf("plan%0d.skip1", slot), plan_o.ctrl[slot].skip_s1, exp_plan[base+6]);
          check($sformatf("plan%0d.skip3", slot), plan_o.ctrl[slot].skip_s3, exp_plan[base+7]);
          check($sformatf("plan%0d.s2pf", slot), plan_o.ctrl[slot].has_s2pf, exp_plan[base+8]);
          check($sformatf("plan%0d.s1both", slot), plan_o.ctrl[slot].dma_s1_both, exp_plan[base+9]);
          check($sformatf("plan%0d.lateboth", slot), plan_o.ctrl[slot].dma_late_both, exp_plan[base+10]);
        end
      end
    end
  endtask

  task automatic run_round;
    int wait_cycles;
    distilled_cluster_state_t expected_c2;
    distilled_cluster_state_t expected_c3;
    begin
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          header[0], header[1], header[2], header[3], header[4], header[5],
          header[6], header[7], header[8], header[9], header[10], header[11],
          header[12]) != 13) $fatal(1, "malformed round header");
      for (int slot = 0; slot < 6; slot++) begin
        if ($fscanf(fd, "%d %d %d\n", desc[slot][0], desc[slot][1], desc[slot][2]) != 3)
          $fatal(1, "malformed descriptor");
      end
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          exp_c2[0], exp_c2[1], exp_c2[2], exp_c2[3], exp_c2[4], exp_c2[5],
          exp_c2[6], exp_c2[7], exp_c2[8], exp_c2[9], exp_c2[10], exp_c2[11],
          exp_c2[12], exp_c2[13], exp_c2[14], exp_c2[15]) != 16) $fatal(1, "bad c2");
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          exp_c3[0], exp_c3[1], exp_c3[2], exp_c3[3], exp_c3[4], exp_c3[5],
          exp_c3[6], exp_c3[7], exp_c3[8], exp_c3[9], exp_c3[10], exp_c3[11],
          exp_c3[12], exp_c3[13], exp_c3[14], exp_c3[15]) != 16) $fatal(1, "bad c3");
      if ($fscanf(fd,
          "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          exp_plan[0], exp_plan[1], exp_plan[2], exp_plan[3], exp_plan[4],
          exp_plan[5], exp_plan[6], exp_plan[7], exp_plan[8], exp_plan[9],
          exp_plan[10], exp_plan[11], exp_plan[12], exp_plan[13], exp_plan[14],
          exp_plan[15], exp_plan[16], exp_plan[17], exp_plan[18], exp_plan[19],
          exp_plan[20], exp_plan[21], exp_plan[22]) != 23) $fatal(1, "bad plan");

      mode_i = distilled_mode_t'(header[0]);
      mode_index_i = 5'(header[1]);
      profile_addr_i = distilled_mode_profile_base(mode_i) + mode_index_i;
      assignment_swap_i = header[2];
      start_time_i = time_t'(header[3]);
      incremental_i = 1'b0;
      rebuild_only_i = 1'b0;
      gain_ok_i = 1'b1;
      force_s1_hit_c2_i = header[4];
      force_s1_hit_c3_i = header[5];
      for (int slot = 0; slot < 5; slot++) begin
        top_i[slot].valid = desc[slot][0];
        top_i[slot].eid = EID_RAW_W'(desc[slot][1]);
        top_i[slot].ntok = ntok_t'(desc[slot][2]);
      end
      bottom_i.valid = desc[5][0];
      bottom_i.eid = EID_RAW_W'(desc[5][1]);
      bottom_i.ntok = ntok_t'(desc[5][2]);

      @(negedge clk_i); start_i = 1'b1;
      @(negedge clk_i);
      start_i = 1'b0;
      // The accepted request must be independent of subsequent live inputs.
      profile_addr_i = (profile_addr_i == 5'd0) ? 5'd1 : 5'd0;
      assignment_swap_i = ~assignment_swap_i;
      start_time_i = '1;
      incremental_i = 1'b1;
      rebuild_only_i = 1'b1;
      force_s1_hit_c2_i = ~force_s1_hit_c2_i;
      force_s1_hit_c3_i = ~force_s1_hit_c3_i;
      top_i = '{default: '0};
      bottom_i = '0;
      wait_cycles = 0;
      while (!done_o && wait_cycles < 40) begin
        @(negedge clk_i);
        wait_cycles++;
      end
      if (!done_o) $fatal(1, "transition timeout");
      expected_c2 = expected_state(exp_c2);
      expected_c3 = expected_state(exp_c3);
      check("feasible", feasible_o, 1);
      check("remove_count", remove_count_o, header[6]);
      check("remove_eid_a", remove_eid_a_o, header[7]);
      check("remove_eid_b", remove_eid_b_o, header[8]);
      check("selected_max", selected_max_o, header[9]);
      check("selected_sum", selected_sum_o, header[10]);
      check("s2pf_count", s2pf_count_o, header[11]);
      check("latest_start", latest_start_o, header[12]);
      check_state("c2", child_c2_o, expected_c2);
      check_state("c3", child_c3_o, expected_c3);
      check_plan();
      base_c2_i = expected_c2;
      base_c3_i = expected_c3;
      @(negedge clk_i);
    end
  endtask

  initial begin
    clk_i = 0;
    rst_ni = 0;
    clear_i = 0;
    start_i = 0;
    mode_i = DIST_MODE_SYNC;
    mode_index_i = 0;
    profile_addr_i = 0;
    assignment_swap_i = 0;
    start_time_i = 0;
    incremental_i = 0;
    rebuild_only_i = 0;
    gain_ok_i = 1;
    force_s1_hit_c2_i = 0;
    force_s1_hit_c3_i = 0;
    top_i = '{default: '0};
    bottom_i = '0;
    base_c2_i = '0;
    base_c3_i = '0;
    failures = 0;
    repeat (4) @(negedge clk_i);
    rst_ni = 1;

    fd = $fopen("distilled_transition_vectors.txt", "r");
    if (fd == 0) $fatal(1, "cannot open transition vectors");
    if ($fscanf(fd, "%d\n", case_count) != 1) $fatal(1, "missing cases");
    for (current_case = 0; current_case < case_count; current_case++) begin
      int case_id;
      int rounds;
      int initial_c2;
      int initial_c3;
      if ($fscanf(fd, "%d %d %d %d\n", case_id, rounds, initial_c2, initial_c3) != 4)
        $fatal(1, "malformed case header");
      base_c2_i = '0;
      base_c3_i = '0;
      if (initial_c2 >= 0) begin
        base_c2_i.cache_valid = 1'b1;
        base_c2_i.cache_eid = EID_RAW_W'(initial_c2);
      end
      if (initial_c3 >= 0) begin
        base_c3_i.cache_valid = 1'b1;
        base_c3_i.cache_eid = EID_RAW_W'(initial_c3);
      end
      for (current_round = 0; current_round < rounds; current_round++)
        run_round();
    end
    $fclose(fd);
    if (failures == 0)
      $display("[RESULT] PASS distilled_transition_eval cases=%0d", case_count);
    else
      $display("[RESULT] FAIL distilled_transition_eval failures=%0d", failures);
    $finish;
  end
endmodule
