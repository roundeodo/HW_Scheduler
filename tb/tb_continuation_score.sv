`timescale 1ns/1ps

module tb_continuation_score;
  import sched_pkg::*;

  logic            clk_i;
  logic            rst_ni;
  logic            clear_i;
  logic            start_i;
  logic            done_o;
  time_t           c2_task_end_i;
  time_t           c3_task_end_i;
  logic [NR_W-1:0] rem_len_i;
  ntok_t           rem0_ntok_i;
  ntok_t           rem1_ntok_i;
  time_t           total_parallel_work_i;
  time_t           largest_parallel_work_i;
  time_t           total_serial_work_i;
  remaining_work_t [3:0] remaining_work_i;
  time_t           cost_o;

  int vec_fd;
  int total_tests;
  int total_fail;

  sched_continuation_score dut (
    .clk_i,
    .rst_ni,
    .clear_i,
    .start_i,
    .done_o,
    .c2_task_end_i,
    .c3_task_end_i,
    .remaining_count_i      (rem_len_i),
    .first_remaining_ntok_i (rem0_ntok_i),
    .total_parallel_work_i,
    .largest_parallel_work_i,
    .total_serial_work_i,
    .remaining_work_i,
    .cost_o
  );

  always #5 clk_i = ~clk_i;

  task automatic run_vector;
    int tid;
    int c2_end;
    int c3_end;
    int rem_len;
    int rem0;
    int rem1;
    int total_conc;
    int max_conc;
    int total_task;
    int child_valid [3:0];
    int child_ntok [3:0];
    int expected;
    int matched;
    int timeout;
    begin
      matched = $fscanf(vec_fd,
          "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          tid, c2_end, c3_end, rem_len, rem0, rem1,
          total_conc, max_conc, total_task,
          child_valid[0], child_ntok[0], child_valid[1], child_ntok[1],
          child_valid[2], child_ntok[2], child_valid[3], child_ntok[3], expected);
      if (matched != 18) begin
        $fatal(1, "Malformed score vector at test %0d: matched=%0d", tid, matched);
      end

      @(negedge clk_i);
      c2_task_end_i = time_t'(c2_end);
      c3_task_end_i = time_t'(c3_end);
      rem_len_i     = NR_W'(rem_len);
      rem0_ntok_i   = ntok_t'(rem0);
      rem1_ntok_i   = ntok_t'(rem1);
      total_parallel_work_i  = time_t'(total_conc);
      largest_parallel_work_i = time_t'(max_conc);
      total_serial_work_i = time_t'(total_task);
      for (int slot = 0; slot < 4; slot++) begin
        remaining_work_i[slot]       = '0;
        remaining_work_i[slot].valid = logic'(child_valid[slot]);
        remaining_work_i[slot].ntok  = ntok_t'(child_ntok[slot]);
      end
      start_i = 1'b1;
      @(negedge clk_i);
      start_i = 1'b0;

      timeout = 0;
      while (!done_o && timeout < 16) begin
        @(negedge clk_i);
        timeout++;
      end
      if (!done_o) begin
        $display("[FAIL] score tid=%0d timeout rem=%0d", tid, rem_len);
        total_fail++;
      end else if (cost_o !== time_t'(expected)) begin
        $display("[FAIL] score tid=%0d rem=%0d got=%0d expected=%0d",
                 tid, rem_len, cost_o, expected);
        total_fail++;
      end

      @(negedge clk_i);
    end
  endtask

  initial begin
    clk_i = 1'b0;
    rst_ni = 1'b0;
    clear_i = 1'b0;
    start_i = 1'b0;
    c2_task_end_i = '0;
    c3_task_end_i = '0;
    rem_len_i = '0;
    rem0_ntok_i = '0;
    rem1_ntok_i = '0;
    total_parallel_work_i = '0;
    largest_parallel_work_i = '0;
    total_serial_work_i = '0;
    remaining_work_i = '{default:'0};
    total_fail = 0;

    repeat (4) @(negedge clk_i);
    rst_ni = 1'b1;

    vec_fd = $fopen("score_vectors.txt", "r");
    if (vec_fd == 0) $fatal(1, "Cannot open score_vectors.txt");
    if ($fscanf(vec_fd, "%d\n", total_tests) != 1) $fatal(1, "Missing vector count");

    for (int test_idx = 0; test_idx < total_tests; test_idx++) begin
      run_vector();
    end
    $fclose(vec_fd);

    if (total_fail == 0) begin
      $display("[RESULT] PASS continuation_score tests=%0d", total_tests);
    end else begin
      $display("[RESULT] FAIL continuation_score tests=%0d failures=%0d",
               total_tests, total_fail);
    end
    $finish;
  end

endmodule
