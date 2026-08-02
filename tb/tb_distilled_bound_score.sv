`timescale 1ns/1ps

module tb_distilled_bound_score;
  import sched_pkg::*;
  import sched_distilled_pkg::*;

  logic clk_i;
  logic rst_ni;
  logic clear_i;
  logic start_i;
  logic done_o;
  distilled_bound_cluster_t child_c2_i;
  distilled_bound_cluster_t child_c3_i;
  distilled_bound_counters_t child_counters_i;
  distilled_bound_head_t [4:0] child_head5_i;
  distilled_bound_t f_o;
  distilled_bound_t h_o;
  distilled_bound_t compute_bound_o;
  distilled_bound_t dma_bound_o;

  int fd;
  int vectors;
  int failures;
  int row;
  int c2 [0:15];
  int c3 [0:15];
  int counters [0:8];
  int head [0:4][0:2];
  int expected [0:3];

  sched_distilled_bound_score dut (.*);
  always #5 clk_i = ~clk_i;

  function automatic distilled_bound_cluster_t make_state(input int v [0:15]);
    distilled_bound_cluster_t state;
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

  task automatic check(input string name, input int got, input int exp);
    if (got !== exp) begin
      $display("[FAIL] vector=%0d %s got=%0d expected=%0d", row, name, got, exp);
      failures++;
    end
  endtask

  initial begin
    clk_i = 0;
    rst_ni = 0;
    clear_i = 0;
    start_i = 0;
    child_c2_i = '0;
    child_c3_i = '0;
    child_counters_i = '0;
    child_head5_i = '{default: '0};
    failures = 0;
    repeat (4) @(negedge clk_i);
    rst_ni = 1;

    fd = $fopen("distilled_bound_vectors.txt", "r");
    if (fd == 0) $fatal(1, "cannot open bound vectors");
    if ($fscanf(fd, "%d\n", vectors) != 1) $fatal(1, "missing count");
    for (row = 0; row < vectors; row++) begin
      int wait_cycles;
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          c2[0], c2[1], c2[2], c2[3], c2[4], c2[5], c2[6], c2[7],
          c2[8], c2[9], c2[10], c2[11], c2[12], c2[13], c2[14], c2[15]) != 16)
        $fatal(1, "bad c2");
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          c3[0], c3[1], c3[2], c3[3], c3[4], c3[5], c3[6], c3[7],
          c3[8], c3[9], c3[10], c3[11], c3[12], c3[13], c3[14], c3[15]) != 16)
        $fatal(1, "bad c3");
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d\n",
          counters[0], counters[1], counters[2], counters[3], counters[4],
          counters[5], counters[6], counters[7], counters[8]) != 9)
        $fatal(1, "bad counters");
      for (int slot = 0; slot < 5; slot++)
        if ($fscanf(fd, "%d %d %d\n", head[slot][0], head[slot][1], head[slot][2]) != 3)
          $fatal(1, "bad head");
      if ($fscanf(fd, "%d %d %d %d\n", expected[0], expected[1], expected[2], expected[3]) != 4)
        $fatal(1, "bad expected");

      child_c2_i = make_state(c2);
      child_c3_i = make_state(c3);
      child_counters_i.count = NR_W'(counters[0]);
      child_counters_i.block_sum = DIST_BLOCK_SUM_W'(counters[3]);
      for (int bucket = 0; bucket < 4; bucket++)
        child_counters_i.small_hist[bucket] = DIST_HIST_W'(counters[4+bucket]);
      child_counters_i.parent_bound = distilled_bound_t'(counters[8]);
      for (int slot = 0; slot < 5; slot++) begin
        child_head5_i[slot].valid = head[slot][0];
        child_head5_i[slot].ntok = ntok_t'(head[slot][2]);
      end

      @(negedge clk_i); start_i = 1;
      @(negedge clk_i); start_i = 0;
      wait_cycles = 0;
      while (!done_o && wait_cycles < 400) begin
        @(negedge clk_i);
        wait_cycles++;
      end
      if (!done_o) $fatal(1, "bound timeout vector=%0d", row);
      check("F", f_o, expected[0]);
      check("H", h_o, expected[1]);
      check("C", compute_bound_o, expected[2]);
      check("D", dma_bound_o, expected[3]);
      @(negedge clk_i);
    end
    $fclose(fd);
    if (failures == 0)
      $display("[RESULT] PASS distilled_bound_score vectors=%0d", vectors);
    else
      $display("[RESULT] FAIL distilled_bound_score failures=%0d", failures);
    $finish;
  end
endmodule
