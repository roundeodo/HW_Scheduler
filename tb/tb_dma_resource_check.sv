`timescale 1ns/1ps

module tb_dma_resource_check;
  import sched_pkg::*;

  logic clk;
  logic rst_n;
  logic clear;
  logic start;
  logic done;
  logic ok;
  snap_bw_view_t snap_a;
  snap_bw_view_t snap_b;
  int unsigned tests;

  sched_bandwidth_check dut (
    .clk_i    (clk),
    .rst_ni   (rst_n),
    .clear_i  (clear),
    .start_i  (start),
    .done_o   (done),
    .snap_a_i (snap_a),
    .snap_b_i (snap_b),
    .ok_o     (ok)
  );

  always #5 clk = ~clk;

  task automatic run_case(input logic expected_ok, input string name);
    begin
      @(negedge clk);
      start = 1'b1;
      @(negedge clk);
      start = 1'b0;
      wait (done);
      assert (ok === expected_ok)
        else $fatal(1, "%s: got ok=%b expected=%b", name, ok, expected_ok);
      tests++;
      @(negedge clk);
    end
  endtask

  task automatic set_s1_pair(
    input dma_binding_t dma_a,
    input dma_binding_t dma_b,
    input time_t        start_b
  );
    begin
      snap_a = '0;
      snap_b = '0;
      snap_a.valid      = 1'b1;
      snap_a.task_start = time_t'(0);
      snap_a.dma1_end   = time_t'(4);
      snap_a.dma_s1     = dma_a;
      snap_b.valid      = 1'b1;
      snap_b.task_start = start_b;
      snap_b.dma1_end   = start_b + time_t'(4);
      snap_b.dma_s1     = dma_b;
    end
  endtask

  initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    clear = 1'b0;
    start = 1'b0;
    snap_a = '0;
    snap_b = '0;
    tests = 0;

    repeat (3) @(negedge clk);
    rst_n = 1'b1;

    set_s1_pair(DMA_IDMA, DMA_IDMA, time_t'(0));
    run_case(1'b0, "overlap IDMA/IDMA");
    set_s1_pair(DMA_XDMA, DMA_XDMA, time_t'(0));
    run_case(1'b0, "overlap XDMA/XDMA");
    set_s1_pair(DMA_IDMA, DMA_XDMA, time_t'(0));
    run_case(1'b1, "overlap IDMA/XDMA");
    set_s1_pair(DMA_BOTH, DMA_IDMA, time_t'(0));
    run_case(1'b0, "overlap BOTH/IDMA");
    set_s1_pair(DMA_BOTH, DMA_XDMA, time_t'(0));
    run_case(1'b0, "overlap BOTH/XDMA");
    set_s1_pair(DMA_IDMA, DMA_IDMA, time_t'(4));
    run_case(1'b1, "non-overlap IDMA/IDMA");

    // S2PF is fixed to BOTH and occupies one tick.
    snap_a = '0;
    snap_b = '0;
    snap_a.valid       = 1'b1;
    snap_a.s2pf_valid  = 1'b1;
    snap_a.s2pf_start  = time_t'(0);
    snap_a.s2pf_end    = time_t'(1);
    snap_a.s2_end      = time_t'(1);
    snap_a.s2pf_dma    = DMA_BOTH;
    snap_b.valid       = 1'b1;
    snap_b.task_start  = time_t'(0);
    snap_b.dma1_end    = time_t'(2);
    snap_b.dma_s1      = DMA_XDMA;
    run_case(1'b0, "S2PF BOTH conflicts with S1 XDMA");

    // S4PF is fixed to BOTH (128 B/cc) and occupies a two-tick interval.
    snap_a = '0;
    snap_b = '0;
    snap_a.valid       = 1'b1;
    snap_a.s4pf_valid  = 1'b1;
    snap_a.dma3_end    = time_t'(0);
    snap_a.s4pf_dma    = DMA_BOTH;
    snap_b.valid       = 1'b1;
    snap_b.task_start  = time_t'(0);
    snap_b.dma1_end    = time_t'(4);
    snap_b.dma_s1      = DMA_IDMA;
    run_case(1'b0, "S4PF BOTH conflicts with S1 IDMA");

    $display("[RESULT] PASS dma_resource_check tests=%0d", tests);
    $finish;
  end
endmodule
