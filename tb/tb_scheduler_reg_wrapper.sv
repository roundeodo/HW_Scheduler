`timescale 1ns/1ps

module tb_scheduler_reg_wrapper;
  import sched_pkg::*;
  import sched_candidate_pkg::*;

  localparam int MAX_N = 64;
  localparam logic [47:0] ADDR_CONFIG        = 48'h000;
  localparam logic [47:0] ADDR_WINDOW0       = 48'h008;
  localparam logic [47:0] ADDR_WINDOW1       = 48'h010;
  localparam logic [47:0] ADDR_WINDOW2_START = 48'h018;
  localparam logic [47:0] ADDR_REFILL_QUAD   = 48'h020;
  localparam logic [47:0] ADDR_EVENT_WAIT    = 48'h028;
  localparam logic [47:0] ADDR_TASK_STREAM   = 48'h030;

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

  int vec_fd;
  int total_tests;
  int total_fail;
  int rem_eid [MAX_N];
  int rem_ntok [MAX_N];
  int expected_mode [MAX_N];
  int expected_id [MAX_N];
  int expected_rounds;
  int round_seen;
  int current_tid;
  bit test_active;

  moe_scheduler_reg_wrapper #(
    .reg_req_t (tb_reg_req_t),
    .reg_rsp_t (tb_reg_rsp_t)
  ) dut (
    .clk_i,
    .rst_ni,
    .reg_req_i (reg_req),
    .reg_rsp_o (reg_rsp)
  );

  always #5 clk_i = ~clk_i;

  function automatic logic [15:0] pack_head16(input int eid, input int ntok);
    logic [15:0] word;
    begin
      word = '0;
      word[8:0] = ntok_t'(ntok);
      word[14:9] = EID_RAW_W'(eid);
      word[15] = 1'b1;
      pack_head16 = word;
    end
  endfunction

  function automatic logic [63:0] pack_entries(input int first, input int count);
    logic [63:0] word;
    begin
      word = '0;
      for (int slot = 0; slot < 4; slot++) begin
        if (slot < count)
          word[slot*16 +: 16] = pack_head16(rem_eid[first+slot], rem_ntok[first+slot]);
      end
      pack_entries = word;
    end
  endfunction

  task automatic reg_write(input logic [47:0] addr, input logic [63:0] data);
    begin
      @(negedge clk_i);
      reg_req.addr = addr;
      reg_req.write = 1'b1;
      reg_req.wdata = data;
      reg_req.wstrb = 8'hff;
      reg_req.valid = 1'b1;
      #1;
      if ((reg_rsp.ready !== 1'b1) || (reg_rsp.error !== 1'b0)) begin
        $display("[FAIL] wrapper tid=%0d write addr=0x%0h", current_tid, addr);
        total_fail++;
      end
      @(negedge clk_i);
      reg_req = '0;
    end
  endtask

  task automatic reg_read(input logic [47:0] addr, output logic [63:0] data);
    int wait_cycles;
    begin
      @(negedge clk_i);
      reg_req.addr = addr;
      reg_req.write = 1'b0;
      reg_req.valid = 1'b1;
      #1;
      wait_cycles = 0;
      while ((reg_rsp.ready !== 1'b1) && (wait_cycles < 200_000)) begin
        @(negedge clk_i);
        #1;
        wait_cycles++;
      end
      data = reg_rsp.rdata;
      if ((reg_rsp.ready !== 1'b1) || (reg_rsp.error !== 1'b0)) begin
        $display("[FAIL] wrapper tid=%0d read timeout/error addr=0x%0h",
                 current_tid, addr);
        total_fail++;
      end
      @(negedge clk_i);
      reg_req = '0;
    end
  endtask

  // remove_ready_pulse is sampled by the core on this rising edge.  Observing
  // the accepted handshake avoids a same-timeslot race with MMIO request
  // deassertion in reg_write().
  always @(posedge clk_i) begin
    cand_token_t expected_token;
    logic [3:0] expected_mask;
    if (rst_ni && test_active && dut.remove_ready_pulse) begin
      if (round_seen >= expected_rounds) begin
        $display("[FAIL] wrapper tid=%0d emitted extra round", current_tid);
        total_fail++;
      end else begin
        expected_token = '0;
        expected_token.valid = 1'b1;
        expected_token.mode = cand_mode_t'(expected_mode[round_seen]);
        expected_token.id = CAND_ID_W'(expected_id[round_seen]);
        expected_mask = cand_remove_mask(expected_token);
        if ((dut.i_scheduler_core.selected_candidate.mode !== expected_token.mode) ||
            (dut.i_scheduler_core.selected_candidate.id !== expected_token.id)) begin
          $display("[FAIL] wrapper tid=%0d round=%0d token got=%0d/%0d exp=%0d/%0d",
                   current_tid, round_seen,
                   dut.i_scheduler_core.selected_candidate.mode,
                   dut.i_scheduler_core.selected_candidate.id,
                   expected_token.mode, expected_token.id);
          total_fail++;
        end
        if ((dut.remove_slot_mask !== cand_remove_mask(expected_token)) ||
            (dut.remove_count !== cand_remove_count_from_mask(expected_mask))) begin
          $display("[FAIL] wrapper tid=%0d round=%0d remove mismatch",
                   current_tid, round_seen);
          total_fail++;
        end
        round_seen++;
      end
    end
  end

  task automatic run_one_test;
    int n;
    int cache2;
    int cache3;
    int expected_final;
    int next_rem;
    int total_conc;
    int total_task;
    int refill_count;
    int fifo_count;
    int polls;
    int final_got;
    int hold_wait;
    bit delayed_refill_done;
    logic [63:0] status;
    logic [63:0] task_word;
    logic [63:0] config_word;
    begin
      if ($fscanf(vec_fd, "%d %d %d %d %d %d\n",
                   current_tid, n, cache2, cache3,
                   expected_rounds, expected_final) != 6)
        $fatal(1, "Malformed wrapper header");
      for (int i = 0; i < n; i++) begin
        if ($fscanf(vec_fd, "%d %d\n", rem_eid[i], rem_ntok[i]) != 2)
          $fatal(1, "Malformed wrapper rem tid=%0d idx=%0d", current_tid, i);
      end
      for (int r = 0; r < expected_rounds; r++) begin
        if ($fscanf(vec_fd, "%d %d\n", expected_mode[r], expected_id[r]) != 2)
          $fatal(1, "Malformed wrapper token tid=%0d round=%0d", current_tid, r);
      end

      total_conc = 0;
      total_task = 0;
      for (int i = 0; i < n; i++) begin
        total_conc += int'(parallel_work_ticks(ntok_t'(rem_ntok[i])));
        total_task += int'(serial_work_ticks(ntok_t'(rem_ntok[i])));
      end

      next_rem = (n < 12) ? n : 12;
      delayed_refill_done = 1'b0;
      config_word = '0;
      config_word[7:0] = (cache2 < 0) ? 8'h80 : 8'(cache2);
      config_word[15:8] = (cache3 < 0) ? 8'h80 : 8'(cache3);
      config_word[16 +: NR_W] = NR_W'(n);
      config_word[32 +: T_W] = time_t'(total_conc);
      config_word[48 +: T_W] = time_t'(total_task);

      reg_write(ADDR_CONFIG, config_word);
      reg_write(ADDR_WINDOW0, pack_entries(0, (n < 4) ? n : 4));
      reg_write(ADDR_WINDOW1,
                (n > 4) ? pack_entries(4, ((n-4) < 4) ? (n-4) : 4) : 64'd0);

      round_seen = 0;
      test_active = 1'b1;
      reg_write(ADDR_WINDOW2_START,
                (n > 8) ? pack_entries(8, ((n-8) < 4) ? (n-8) : 4) : 64'd0);

      polls = 0;
      while (polls < 200_000) begin
        reg_read(ADDR_EVENT_WAIT, status);
        fifo_count = int'(status[8:5]);
        refill_count = int'(status[4:2]);

        if (status[1] && (refill_count != 0)) begin
          if ((refill_count > 4) || (next_rem + refill_count > n)) begin
            $display("[FAIL] wrapper tid=%0d illegal refill count=%0d next=%0d n=%0d",
                     current_tid, refill_count, next_rem, n);
            total_fail++;
            return;
          end
          if ((refill_count < 4) && ((next_rem + refill_count) != n)) begin
            $display("[FAIL] wrapper tid=%0d non-tail short refill=%0d next=%0d n=%0d",
                     current_tid, refill_count, next_rem, n);
            total_fail++;
            return;
          end
          // The largest batch deliberately withholds one refill.  Drain the
          // already reported words first, then require the wrapper to hold a
          // resource-starved remove until REFILL_QUAD arrives.
          if ((current_tid == 511) && !delayed_refill_done) begin
            for (int task_idx = 0; task_idx < fifo_count; task_idx++) begin
              reg_read(ADDR_TASK_STREAM, task_word);
              if (^task_word === 1'bx) begin
                $display("[FAIL] wrapper tid=%0d delayed-drain word contains X",
                         current_tid);
                total_fail++;
              end
            end
            fifo_count = 0;
            hold_wait = 0;
            while (!(dut.remove_valid && !dut.remove_resources_ready) &&
                   (hold_wait < 200_000)) begin
              @(negedge clk_i);
              hold_wait++;
            end
            if (hold_wait >= 200_000) begin
              $display("[FAIL] wrapper tid=%0d did not reach refill hold", current_tid);
              total_fail++;
              return;
            end
            if (dut.auto_remove_pulse !== 1'b0) begin
              $display("[FAIL] wrapper tid=%0d accepted resource-starved remove",
                       current_tid);
              total_fail++;
              return;
            end
            delayed_refill_done = 1'b1;
          end
          reg_write(ADDR_REFILL_QUAD, pack_entries(next_rem, refill_count));
          next_rem += refill_count;
        end

        if ((fifo_count >= 4) || status[0]) begin
          if ((fifo_count < 1) || (fifo_count > TASKQ_DEPTH)) begin
            if (!(status[0] && (fifo_count == 0))) begin
              $display("[FAIL] wrapper tid=%0d illegal fifo count=%0d",
                       current_tid, fifo_count);
              total_fail++;
              return;
            end
          end
          for (int task_idx = 0; task_idx < fifo_count; task_idx++) begin
            reg_read(ADDR_TASK_STREAM, task_word);
            if (^task_word === 1'bx) begin
              $display("[FAIL] wrapper tid=%0d task word contains X", current_tid);
              total_fail++;
            end
          end
        end

        if (status[0] && (round_seen == expected_rounds)) break;
        polls++;
      end
      test_active = 1'b0;

      if (polls >= 200_000) begin
        $display("[FAIL] wrapper tid=%0d timeout rounds=%0d/%0d",
                 current_tid, round_seen, expected_rounds);
        total_fail++;
      end
      if (next_rem != n) begin
        $display("[FAIL] wrapper tid=%0d loaded=%0d expected=%0d",
                 current_tid, next_rem, n);
        total_fail++;
      end
      final_got = (dut.i_scheduler_core.c2_timeline_q.task_end >
                   dut.i_scheduler_core.c3_timeline_q.task_end) ?
                   dut.i_scheduler_core.c2_timeline_q.task_end :
                   dut.i_scheduler_core.c3_timeline_q.task_end;
      if (final_got != expected_final) begin
        $display("[FAIL] wrapper tid=%0d final got=%0d exp=%0d",
                 current_tid, final_got, expected_final);
        total_fail++;
      end
    end
  endtask

  initial begin
    clk_i = 1'b0;
    rst_ni = 1'b0;
    reg_req = '0;
    total_fail = 0;
    test_active = 1'b0;
    round_seen = 0;

    repeat (5) @(negedge clk_i);
    rst_ni = 1'b1;

    vec_fd = $fopen("core_vectors.txt", "r");
    if (vec_fd == 0) $fatal(1, "Cannot open core_vectors.txt");
    if ($fscanf(vec_fd, "%d\n", total_tests) != 1) $fatal(1, "Missing test count");
    for (int test_idx = 0; test_idx < total_tests; test_idx++) begin
      run_one_test();
    end
    $fclose(vec_fd);

    if (total_fail == 0)
      $display("[RESULT] PASS scheduler_reg_wrapper tests=%0d", total_tests);
    else
      $display("[RESULT] FAIL scheduler_reg_wrapper tests=%0d failures=%0d",
               total_tests, total_fail);
    $finish;
  end

endmodule
