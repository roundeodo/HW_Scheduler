// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - sequential DMA-resource conflict checker
//
// This module keeps the ordered DMA-window semantics, but does not store the whole
// segment queue.  Callers must keep snap_a_i/snap_b_i stable while the checker is
// high.  The checker rebuilds the 3 reachable segment slots combinationally and
// stores only two small pointers plus busy/done/ok state.

import sched_pkg::*;

module sched_bandwidth_check (
  input  logic       clk_i,
  input  logic       rst_ni,
  input  logic       clear_i,
  input  logic       start_i,
  output logic       done_o,

  input  wire snap_bw_view_t snap_a_i,
  input  wire snap_bw_view_t snap_b_i,
  output logic       ok_o
);

  typedef struct packed {
    logic            valid;
    logic [T_W-1:0]  lo;
    logic [T_W-1:0]  hi;
    dma_binding_t    dma_mask;
  } dma_segment_t;

  localparam int unsigned SEGMENT_COUNT = 3;

  dma_segment_t segment_a [SEGMENT_COUNT];
  dma_segment_t segment_b [SEGMENT_COUNT];

  logic both_sides_have_segments;
  logic dma_resource_conflict;
  logic       sweep_active;
  logic       done_q;
  logic       ok_q;
  logic [2:0] ptr_a_q;
  logic [2:0] ptr_b_q;
  logic [2:0] ptr_a_next;
  logic [2:0] ptr_b_next;
  logic [2:0] ptr_a_first;
  logic [2:0] ptr_b_first;
  logic       advance_a;
  logic       advance_b;
  logic       sweep_done_next;

  dma_segment_t current_segment_a;
  dma_segment_t current_segment_b;

  function automatic dma_segment_t make_segment(
    input logic            valid,
    input logic [T_W-1:0]  lo,
    input logic [T_W-1:0]  hi,
    input dma_binding_t    dma_mask
  );
    dma_segment_t segment;
    begin
      segment.valid  = valid;
      segment.lo     = lo;
      segment.hi     = hi;
      segment.dma_mask = dma_mask;
      make_segment = segment;
    end
  endfunction

  task automatic build_side_segments(
    input  snap_bw_view_t sn,
    output dma_segment_t segment_o [SEGMENT_COUNT]
  );
    begin
      segment_o = '{default: '0};

      // Producer invariants make the three reachable DMA windows ordered:
      // S1, one mutually-exclusive stage-3 transfer (S2PF or S3), and S4PF.
      segment_o[0] = make_segment(sn.valid && (sn.dma_s1 != DMA_NONE),
                                  sn.task_start, sn.dma1_end, sn.dma_s1);
      segment_o[1] = sn.s2pf_valid ?
          make_segment(sn.valid, sn.s2pf_start, sn.s2pf_end, sn.s2pf_dma) :
          make_segment(sn.valid && (sn.dma_s3 != DMA_NONE),
                       sn.s2_end, sn.dma3_end, sn.dma_s3);
      segment_o[2] = make_segment(sn.valid && sn.s4pf_valid,
                                  sn.dma3_end,
                                  sn.dma3_end + S4PF_DMA_TICKS,
                                  sn.s4pf_dma);
    end
  endtask

  function automatic logic [2:0] first_valid_ptr(
    input dma_segment_t segment_i [SEGMENT_COUNT]
  );
    begin
      if (segment_i[0].valid) begin
        first_valid_ptr = 3'b001;
      end else if (segment_i[1].valid) begin
        first_valid_ptr = 3'b010;
      end else if (segment_i[2].valid) begin
        first_valid_ptr = 3'b100;
      end else begin
        first_valid_ptr = 3'b000;
      end
    end
  endfunction

  function automatic logic [2:0] next_valid_ptr(
    input logic [2:0] ptr,
    input dma_segment_t segment_i [SEGMENT_COUNT]
  );
    begin
      unique case (ptr)
        3'b001: begin
          if (segment_i[1].valid) begin
            next_valid_ptr = 3'b010;
          end else if (segment_i[2].valid) begin
            next_valid_ptr = 3'b100;
          end else begin
            next_valid_ptr = 3'b000;
          end
        end
        3'b010: begin
          if (segment_i[2].valid) begin
            next_valid_ptr = 3'b100;
          end else begin
            next_valid_ptr = 3'b000;
          end
        end
        3'b100: begin
          next_valid_ptr = 3'b000;
        end
        3'b000: next_valid_ptr = 3'b000;
        default: next_valid_ptr = 'x;
      endcase
    end
  endfunction

  function automatic dma_segment_t segment_at_ptr(
    input logic [2:0] ptr,
    input dma_segment_t segment_i [SEGMENT_COUNT]
  );
    begin
      unique case (ptr)
        3'b001: segment_at_ptr = segment_i[0];
        3'b010: segment_at_ptr = segment_i[1];
        3'b100: segment_at_ptr = segment_i[2];
        3'b000: segment_at_ptr = '0;
        default: segment_at_ptr = 'x;
      endcase
    end
  endfunction

  always_comb begin
    build_side_segments(snap_a_i, segment_a);
    build_side_segments(snap_b_i, segment_b);
  end

  assign ptr_a_first = first_valid_ptr(segment_a);
  assign ptr_b_first = first_valid_ptr(segment_b);
  assign both_sides_have_segments = (|ptr_a_first) && (|ptr_b_first);
  assign sweep_active = (|ptr_a_q) && (|ptr_b_q);
  assign current_segment_a = segment_at_ptr(ptr_a_q, segment_a);
  assign current_segment_b = segment_at_ptr(ptr_b_q, segment_b);

  // 每段直接保存实际 DMA lane mask。两个重叠区间只有在资源集合不相交时
  // 才能并行：iDMA+xDMA 合法；iDMA+iDMA、xDMA+xDMA，以及任何与 BOTH
  // 的重叠都非法。这里不再根据 64/128 抽象带宽推测资源。
  assign dma_resource_conflict =
      (|(current_segment_a.dma_mask & current_segment_b.dma_mask)) &&
      (current_segment_a.hi > current_segment_b.lo) &&
      (current_segment_b.hi > current_segment_a.lo);

  // Ordered interval sweep:
  //   - invalid side: advance that side
  //   - both valid: advance the interval ending first, independent of overlap
  // Equal end timestamps advance both sides.
  // While busy, both pointers either select a valid segment or are zero.  A
  // zero pointer terminates the sweep; the datapath only decodes the three
  // legal one-hot positions and does not implement an invalid-pointer path.
  assign advance_a = (current_segment_a.hi <= current_segment_b.hi);
  assign advance_b = (current_segment_b.hi <= current_segment_a.hi);
  assign ptr_a_next = advance_a ? next_valid_ptr(ptr_a_q, segment_a) : ptr_a_q;
  assign ptr_b_next = advance_b ? next_valid_ptr(ptr_b_q, segment_b) : ptr_b_q;
  assign sweep_done_next = !(|ptr_a_next) || !(|ptr_b_next);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      done_q   <= 1'b0;
      ok_q     <= 1'b0;
      ptr_a_q  <= '0;
      ptr_b_q  <= '0;
    end else if (clear_i) begin
      done_q   <= 1'b0;
      ok_q     <= 1'b0;
      ptr_a_q  <= '0;
      ptr_b_q  <= '0;
    end else begin
      done_q <= 1'b0;

      if (sweep_active) begin
        if (dma_resource_conflict || sweep_done_next) begin
          if (dma_resource_conflict) begin
            ok_q <= 1'b0;
          end
          done_q <= 1'b1;
          ptr_a_q <= '0;
          ptr_b_q <= '0;
        end else begin
          ptr_a_q <= ptr_a_next;
          ptr_b_q <= ptr_b_next;
        end
      end else if (start_i) begin
        ok_q          <= 1'b1;
        if (both_sides_have_segments) begin
          ptr_a_q <= ptr_a_first;
          ptr_b_q <= ptr_b_first;
        end else begin
          ptr_a_q <= '0;
          ptr_b_q <= '0;
          done_q  <= 1'b1;
        end
      end
    end
  end

  assign done_o = done_q;
  assign ok_o   = ok_q;

`ifndef SYNTHESIS
  task automatic assert_side_contract(input snap_bw_view_t sn);
    begin
      assert (!sn.s2pf_valid ||
              ((sn.s2pf_start >= sn.dma1_end) &&
               (sn.s2pf_end <= sn.s2_end) &&
               (sn.s2pf_dma == DMA_BOTH) &&
               (sn.dma_s3 == DMA_NONE)))
        else $error("invalid S2PF timeline contract");
      assert (!sn.s4pf_valid || (sn.s4pf_dma == DMA_BOTH))
        else $error("S4PF must use the BOTH-DMA binding");
    end
  endtask

  // Pointer-only sweep 不锁存完整 segment queue；busy 期间依赖调用者保持
  // compact BW view 稳定。若输入变化，ptr_q 会指向不同的组合 segment。
  always_ff @(posedge clk_i) begin
    if (rst_ni && start_i) begin
      assert_side_contract(snap_a_i);
      assert_side_contract(snap_b_i);
    end
    if (rst_ni && sweep_active && $past(sweep_active)) begin
      assert ($stable(snap_a_i))
        else $error("sched_bandwidth_check snap_a_i changed while busy");
      assert ($stable(snap_b_i))
        else $error("sched_bandwidth_check snap_b_i changed while busy");
    end
  end
`endif

endmodule
