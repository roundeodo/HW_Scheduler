`timescale 1ns/1ps

module tb_scheduler_core;
  import sched_pkg::*;
  import sched_candidate_pkg::*;

  localparam int MAX_N = 64;

  logic clk_i;
  logic rst_ni;
  logic init_i;
  logic start_i;
  pf_eid_t initial_cache_eid_c2_i;
  pf_eid_t initial_cache_eid_c3_i;
  head_ctx_t [5:0] head_i;
  logic [NR_W-1:0] active_count_i;
  time_t total_parallel_work_i;
  time_t total_serial_work_i;
  logic remove_ready_i;
  logic remove_valid_o;
  logic [1:0] remove_count_o;
  logic [3:0] remove_slot_mask_o;
  time_t remove_parallel_work_o;
  time_t remove_serial_work_o;
  logic task_pop_i;
  logic task_valid_o;
  logic [63:0] task_rd_data_o;
  logic task_queue_full_o;
  logic [3:0] task_queue_count_o;
  logic busy_o;

  int vec_fd;
  int total_tests;
  int total_fail;
  int rem_count;
  int rem_eid [MAX_N];
  int rem_ntok [MAX_N];

  moe_scheduler_core dut (
    .clk_i,
    .rst_ni,
    .init_i,
    .start_i,
    .initial_cache_eid_c2_i,
    .initial_cache_eid_c3_i,
    .head_i,
    .active_count_i,
    .total_parallel_work_i,
    .total_serial_work_i,
    .remove_ready_i,
    .remove_valid_o,
    .remove_count_o,
    .remove_slot_mask_o,
    .remove_parallel_work_o,
    .remove_serial_work_o,
    .task_fifo_pop_i        (task_pop_i),
    .task_fifo_valid_o      (task_valid_o),
    .task_fifo_read_data_o  (task_rd_data_o),
    .task_fifo_full_o       (task_queue_full_o),
    .task_fifo_count_o      (task_queue_count_o),
    .busy_o
  );

  always #5 clk_i = ~clk_i;

  // Drain every published dense task immediately.  The winner-token check is
  // independent of CVA6 lowering latency and must never be blocked by FIFO fill.
  always_comb begin
    task_pop_i = task_valid_o;
  end

  task automatic drive_round_context;
    int total_conc;
    int total_task;
    begin
      head_i = '{default:'0};
      total_conc = 0;
      total_task = 0;
      for (int i = 0; i < rem_count; i++) begin
        if (i < 6) begin
          head_i[i].valid = 1'b1;
          head_i[i].eid   = EID_RAW_W'(rem_eid[i]);
          head_i[i].ntok  = ntok_t'(rem_ntok[i]);
        end
        total_conc += int'(parallel_work_ticks(ntok_t'(rem_ntok[i])));
        total_task += int'(serial_work_ticks(ntok_t'(rem_ntok[i])));
      end
      active_count_i = NR_W'(rem_count);
      total_parallel_work_i = time_t'(total_conc);
      total_serial_work_i = time_t'(total_task);
    end
  endtask

  task automatic apply_remove_mask(input logic [3:0] mask);
    int wr;
    begin
      wr = 0;
      for (int i = 0; i < rem_count; i++) begin
        if ((i >= 4) || !mask[i]) begin
          rem_eid[wr] = rem_eid[i];
          rem_ntok[wr] = rem_ntok[i];
          wr++;
        end
      end
      rem_count = wr;
    end
  endtask

  task automatic pulse_init;
    begin
      @(negedge clk_i);
      init_i = 1'b1;
      @(negedge clk_i);
      init_i = 1'b0;
    end
  endtask

  task automatic run_one_test;
    int tid;
    int n;
    int cache2;
    int cache3;
    int n_rounds;
    int expected_final;
    int expected_mode [MAX_N];
    int expected_id [MAX_N];
    int timeout;
    int matched;
    int final_got;
    cand_token_t expected_token;
    logic [3:0] expected_mask;
    begin
      matched = $fscanf(vec_fd, "%d %d %d %d %d %d\n",
                        tid, n, cache2, cache3, n_rounds, expected_final);
      if (matched != 6) $fatal(1, "Malformed core header at tid=%0d", tid);
      rem_count = n;
      for (int i = 0; i < n; i++) begin
        if ($fscanf(vec_fd, "%d %d\n", rem_eid[i], rem_ntok[i]) != 2)
          $fatal(1, "Malformed rem descriptor tid=%0d idx=%0d", tid, i);
      end
      for (int r = 0; r < n_rounds; r++) begin
        if ($fscanf(vec_fd, "%d %d\n", expected_mode[r], expected_id[r]) != 2)
          $fatal(1, "Malformed winner token tid=%0d round=%0d", tid, r);
      end

      initial_cache_eid_c2_i = (cache2 < 0) ? PF_EID_NONE :
                                  encode_eid(EID_RAW_W'(cache2));
      initial_cache_eid_c3_i = (cache3 < 0) ? PF_EID_NONE :
                                  encode_eid(EID_RAW_W'(cache3));
      drive_round_context();
      pulse_init();

      for (int round_idx = 0; round_idx < n_rounds; round_idx++) begin
        drive_round_context();
        @(negedge clk_i);
        start_i = 1'b1;
        @(negedge clk_i);
        start_i = 1'b0;

        timeout = 0;
        while (!remove_valid_o && timeout < 20_000) begin
          @(negedge clk_i);
          timeout++;
        end
        if (!remove_valid_o) begin
          $display("[FAIL] core tid=%0d round=%0d timeout", tid, round_idx);
          total_fail++;
          return;
        end

        expected_token = '0;
        expected_token.valid = 1'b1;
        expected_token.mode = cand_mode_t'(expected_mode[round_idx]);
        expected_token.id = CAND_ID_W'(expected_id[round_idx]);
        expected_mask = cand_remove_mask(expected_token);
        if ((dut.selected_candidate.mode !== expected_token.mode) ||
            (dut.selected_candidate.id !== expected_token.id)) begin
          $display("[FAIL] core tid=%0d round=%0d token got=%0d/%0d exp=%0d/%0d",
                   tid, round_idx, dut.selected_candidate.mode,
                   dut.selected_candidate.id,
                   expected_token.mode, expected_token.id);
          total_fail++;
        end
        if ((remove_slot_mask_o !== expected_mask) ||
            (remove_count_o !== cand_remove_count_from_mask(expected_mask))) begin
          $display("[FAIL] core tid=%0d round=%0d remove got=%b/%0d exp=%b/%0d",
                   tid, round_idx, remove_slot_mask_o, remove_count_o,
                   expected_mask, cand_remove_count_from_mask(expected_mask));
          total_fail++;
        end

        apply_remove_mask(expected_mask);
        @(negedge clk_i);
        remove_ready_i = 1'b1;
        @(negedge clk_i);
        remove_ready_i = 1'b0;
      end

      final_got = (dut.c2_timeline_q.task_end > dut.c3_timeline_q.task_end) ?
                  dut.c2_timeline_q.task_end : dut.c3_timeline_q.task_end;
      if (final_got != expected_final) begin
        $display("[FAIL] core tid=%0d final got=%0d exp=%0d",
                 tid, final_got, expected_final);
        total_fail++;
      end
    end
  endtask

  initial begin
    clk_i = 1'b0;
    rst_ni = 1'b0;
    init_i = 1'b0;
    start_i = 1'b0;
    initial_cache_eid_c2_i = PF_EID_NONE;
    initial_cache_eid_c3_i = PF_EID_NONE;
    head_i = '{default:'0};
    active_count_i = '0;
    total_parallel_work_i = '0;
    total_serial_work_i = '0;
    remove_ready_i = 1'b0;
    total_fail = 0;

    repeat (4) @(negedge clk_i);
    rst_ni = 1'b1;

    vec_fd = $fopen("core_vectors.txt", "r");
    if (vec_fd == 0) $fatal(1, "Cannot open core_vectors.txt");
    if ($fscanf(vec_fd, "%d\n", total_tests) != 1) $fatal(1, "Missing test count");
    for (int test_idx = 0; test_idx < total_tests; test_idx++) begin
      run_one_test();
    end
    $fclose(vec_fd);

    if (total_fail == 0)
      $display("[RESULT] PASS scheduler_core tests=%0d", total_tests);
    else
      $display("[RESULT] FAIL scheduler_core tests=%0d failures=%0d", total_tests, total_fail);
    $finish;
  end

endmodule
