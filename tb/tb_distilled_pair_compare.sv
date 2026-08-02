`timescale 1ns/1ps

module tb_distilled_pair_compare;
  import sched_pkg::*;
  import sched_distilled_pkg::*;

  logic clk_i;
  logic rst_ni;
  logic clear_i;
  logic start_i;
  logic done_o;
  distilled_mode_t mode_i;
  distilled_regime_t regime_i;
  logic [NR_W-1:0] before_count_i;
  ntok_t min_remaining_load_i;
  distilled_score_record_t lhs_i;
  distilled_score_record_t rhs_i;
  logic children_differ_i;
  logic rhs_wins_o;
  logic override_o;

  int fd;
  int vectors;
  int row;
  int failures;
  int header [0:10];
  int lhs [0:13];
  int rhs [0:13];

  sched_distilled_pair_compare dut (.*);
  always #5 clk_i = ~clk_i;

  function automatic distilled_score_record_t make_record(input int v [0:13]);
    distilled_score_record_t record;
    begin
      record = '0;
      record.f = distilled_bound_t'(v[0]);
      record.h = distilled_bound_t'(v[1]);
      record.compute_bound = distilled_bound_t'(v[2]);
      record.dma_bound = distilled_bound_t'(v[3]);
      record.early_end = time_t'(v[4]);
      record.late_end = time_t'(v[5]);
      record.selected_max = ntok_t'(v[7]);
      record.selected_sum = (NTOK_W+1)'(v[8]);
      record.s2pf_count = 2'(v[9]);
      record.remaining_count = NR_W'(v[10]);
      record.selected_min_rank = NR_W'(v[11]);
      record.selected_max_rank = NR_W'(v[12]);
      record.selects_t0 = v[13];
      make_record = record;
    end
  endfunction

  initial begin
    clk_i = 1'b0;
    rst_ni = 1'b0;
    clear_i = 1'b0;
    start_i = 1'b0;
    mode_i = DIST_MODE_TERMINAL;
    regime_i = '0;
    before_count_i = '0;
    min_remaining_load_i = '0;
    lhs_i = '0;
    rhs_i = '0;
    children_differ_i = 1'b0;
    failures = 0;
    repeat (4) @(negedge clk_i);
    rst_ni = 1'b1;

    fd = $fopen("distilled_compare_vectors.txt", "r");
    if (fd == 0) $fatal(1, "cannot open compare vectors");
    if ($fscanf(fd, "%d\n", vectors) != 1) $fatal(1, "missing vector count");
    for (row = 0; row < vectors; row++) begin
      int wait_cycles;
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d\n",
          header[0], header[1], header[2], header[3], header[4], header[5],
          header[6], header[7], header[8], header[9], header[10]) != 11)
        $fatal(1, "bad header row=%0d", row);
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          lhs[0], lhs[1], lhs[2], lhs[3], lhs[4], lhs[5], lhs[6],
          lhs[7], lhs[8], lhs[9], lhs[10], lhs[11], lhs[12], lhs[13]) != 14)
        $fatal(1, "bad lhs row=%0d", row);
      if ($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          rhs[0], rhs[1], rhs[2], rhs[3], rhs[4], rhs[5], rhs[6],
          rhs[7], rhs[8], rhs[9], rhs[10], rhs[11], rhs[12], rhs[13]) != 14)
        $fatal(1, "bad rhs row=%0d", row);

      mode_i = distilled_mode_t'(header[0]);
      regime_i.low_work_progress = header[1];
      regime_i.sparse_hot_sync = header[2];
      regime_i.mid_plateau = header[3];
      regime_i.short_tail_plateau = header[4];
      regime_i.large_slack_fill = header[5];
      before_count_i = NR_W'(header[6]);
      min_remaining_load_i = ntok_t'(header[7]);
      children_differ_i = header[8];
      lhs_i = make_record(lhs);
      rhs_i = make_record(rhs);

      @(negedge clk_i); start_i = 1'b1;
      @(negedge clk_i); start_i = 1'b0;
      wait_cycles = 0;
      while (!done_o && wait_cycles < 40) begin
        @(negedge clk_i);
        wait_cycles++;
      end
      if (!done_o) $fatal(1, "compare timeout row=%0d", row);
      if (rhs_wins_o !== header[9]) begin
        $display("[FAIL] row=%0d winner got=%0d expected=%0d",
                 row, rhs_wins_o, header[9]);
        failures++;
      end
      if (override_o !== header[10]) begin
        $display("[FAIL] row=%0d override got=%0d expected=%0d",
                 row, override_o, header[10]);
        failures++;
      end
      @(negedge clk_i);
    end
    $fclose(fd);
    if (failures == 0)
      $display("[RESULT] PASS distilled_pair_compare vectors=%0d", vectors);
    else
      $display("[RESULT] FAIL distilled_pair_compare failures=%0d", failures);
    $finish;
  end
endmodule
