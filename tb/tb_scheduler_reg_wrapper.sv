`timescale 1ns/1ps

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

module tb_scheduler_reg_wrapper;
  import sched_pkg::*;

  logic clk_i;
  logic rst_ni;
  tb_reg_req_t reg_req_i;
  tb_reg_rsp_t reg_rsp_o;

  int fd;
  int cases;
  int failures;
  int multibeat_refills;
  int max_task_batch;
  int event_reads;
  int refill_events;
  int refill_write_beats;
  int task_reads;
  int early_task_events;
  int refill_top [0:255];
  int refill_bottom [0:255];
  int refill_beats [0:255];
  logic [63:0] refill_word [0:255][0:1];
  logic [63:0] expected_task [0:255];

  moe_scheduler_reg_wrapper #(
    .reg_req_t(tb_reg_req_t),
    .reg_rsp_t(tb_reg_rsp_t)
  ) dut (.*);
  always #5 clk_i = ~clk_i;

  task automatic write_word(input logic [6:0] address,
                            input logic [63:0] data);
    @(negedge clk_i);
    reg_req_i.addr = address;
    reg_req_i.write = 1'b1;
    reg_req_i.wdata = data;
    reg_req_i.wstrb = 8'hff;
    reg_req_i.valid = 1'b1;
    @(negedge clk_i);
    reg_req_i = '0;
  endtask

  task automatic read_word(input logic [6:0] address,
                           output logic [63:0] data);
    int wait_cycles;
    @(negedge clk_i);
    reg_req_i.addr = address;
    reg_req_i.write = 1'b0;
    reg_req_i.valid = 1'b1;
    wait_cycles = 0;
    #1;
    while (!reg_rsp_o.ready && wait_cycles < 200000) begin
      @(negedge clk_i);
      #1;
      wait_cycles++;
    end
    if (!reg_rsp_o.ready)
      $fatal(1, "MMIO read timeout address=0x%0h", address);
    data = reg_rsp_o.rdata;
    @(negedge clk_i);
    reg_req_i = '0;
  endtask

  initial begin
    logic [63:0] cfg;
    logic [63:0] aggregate;
    logic [63:0] window0;
    logic [63:0] window1;
    logic [63:0] window2;
    logic [63:0] window3;
    clk_i = 1'b0;
    rst_ni = 1'b0;
    reg_req_i = '0;
    failures = 0;
    multibeat_refills = 0;
    max_task_batch = 0;
    event_reads = 0;
    refill_events = 0;
    refill_write_beats = 0;
    task_reads = 0;
    early_task_events = 0;
    repeat (5) @(negedge clk_i);
    rst_ni = 1'b1;

    fd = $fopen("distilled_wrapper_vectors.txt", "r");
    if (fd == 0) $fatal(1, "cannot open wrapper vectors");
    if ($fscanf(fd, "%d\n", cases) != 1) $fatal(1, "missing case count");
    for (int case_id = 0; case_id < cases; case_id++) begin
      int refill_count;
      int task_count;
      int refill_index;
      int task_index;
      int active_count;
      logic done;
      if ($fscanf(fd, "%h %h %h %h %h %h %d %d\n", cfg, aggregate,
          window0, window1, window2, window3,
          refill_count, task_count) != 8)
        $fatal(1, "bad case header %0d", case_id);
      for (int refill = 0; refill < refill_count; refill++)
        if ($fscanf(fd, "%d %d %d %h %h\n", refill_top[refill],
                   refill_bottom[refill], refill_beats[refill],
                   refill_word[refill][0], refill_word[refill][1]) != 5)
          $fatal(1, "bad refill case=%0d", case_id);
      for (int task_id = 0; task_id < task_count; task_id++)
        if ($fscanf(fd, "%h\n", expected_task[task_id]) != 1)
          $fatal(1, "bad task case=%0d", case_id);

      write_word(7'h00, cfg);
      write_word(7'h38, aggregate);
      active_count = cfg[16 +: NR_W];
      if (active_count <= 4) begin
        write_word(7'h08, window0);
      end else if (active_count <= 8) begin
        write_word(7'h08, window0);
        write_word(7'h10, window1);
      end else if (active_count <= 12) begin
        write_word(7'h08, window0);
        write_word(7'h10, window1);
        write_word(7'h18, window2);
      end else begin
        write_word(7'h08, window0);
        write_word(7'h10, window1);
        write_word(7'h18, window2);
        write_word(7'h40, window3);
      end
      refill_index = 0;
      task_index = 0;
      done = 1'b0;
      while (!done) begin
        logic [63:0] event_word;
        read_word(7'h28, event_word);
        event_reads++;
        if (event_word[11:8] > max_task_batch)
          max_task_batch = event_word[11:8];
        if (!event_word[0] && event_word[11:8] != 0 && event_word[11:8] < 6)
          early_task_events++;
        if (event_word[11:8] > TASKQ_DEPTH)
          $fatal(1, "event task count exceeds FIFO depth");
        if (event_word[1]) begin
          int got_top;
          int got_bottom;
          got_top = event_word[4:2];
          got_bottom = event_word[7:5];
          if (refill_index >= refill_count)
            $fatal(1, "unexpected refill case=%0d", case_id);
          if (got_top != refill_top[refill_index] ||
              got_bottom != refill_bottom[refill_index]) begin
            $display("[FAIL] case=%0d refill=%0d got=%0d+%0d expected=%0d+%0d",
                     case_id, refill_index, got_top, got_bottom,
                     refill_top[refill_index], refill_bottom[refill_index]);
            failures++;
          end
          if (refill_beats[refill_index] !=
              ((got_top + got_bottom + 3) / 4)) begin
            $display("[FAIL] case=%0d refill=%0d bad beat count=%0d",
                     case_id, refill_index, refill_beats[refill_index]);
            failures++;
          end
          for (int beat = 0; beat < refill_beats[refill_index]; beat++)
            write_word(7'h20, refill_word[refill_index][beat]);
          refill_events++;
          refill_write_beats += refill_beats[refill_index];
          if (refill_beats[refill_index] == 2)
            multibeat_refills++;
          refill_index++;
        end
        for (int task_id = 0; task_id < event_word[11:8]; task_id++) begin
          logic [63:0] got_task;
          read_word(7'h30, got_task);
          task_reads++;
          if (task_index >= task_count)
            $fatal(1, "unexpected task case=%0d", case_id);
          if (got_task !== expected_task[task_index]) begin
            $display("[FAIL] case=%0d task=%0d got=%016h expected=%016h",
                     case_id, task_index, got_task, expected_task[task_index]);
            failures++;
          end
          task_index++;
        end
        done = event_word[0];
      end
      if (refill_index != refill_count || task_index != task_count) begin
        $display("[FAIL] case=%0d consumed refill=%0d/%0d task=%0d/%0d",
                 case_id, refill_index, refill_count, task_index, task_count);
        failures++;
      end
      @(negedge clk_i);
      rst_ni = 1'b0;
      repeat (3) @(negedge clk_i);
      rst_ni = 1'b1;
    end
    $fclose(fd);
    if (multibeat_refills == 0 || early_task_events == 0) begin
      $display("[FAIL] protocol coverage multibeat=%0d early_task_events=%0d",
               multibeat_refills, early_task_events);
      failures++;
    end
    if (failures == 0)
      $display("[RESULT] PASS distilled_wrapper cases=%0d multibeat=%0d early_task_events=%0d max_task_batch=%0d event_reads=%0d refill_events=%0d refill_beats=%0d task_reads=%0d",
               cases, multibeat_refills, early_task_events, max_task_batch, event_reads,
               refill_events, refill_write_beats, task_reads);
    else
      $display("[RESULT] FAIL distilled_wrapper failures=%0d", failures);
    $finish;
  end
endmodule
