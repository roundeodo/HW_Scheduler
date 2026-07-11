// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - sequential bw_ok checker
//
// This module keeps the BW segment semantics, but does not store the whole
// segment queue.  Callers must keep snap_a_i/snap_b_i stable while busy_o is
// high.  The checker rebuilds the 4 fixed segment slots combinationally and
// stores only two small pointers plus busy/done/ok state.

import sched_pkg::*;

module sched_bw_ok_seq (
  input  logic       clk_i,
  input  logic       rst_ni,
  input  logic       clear_i,
  input  logic       start_i,
  output logic       done_o,

  input  snap_bw_view_t snap_a_i,
  input  snap_bw_view_t snap_b_i,
  output logic       ok_o
);

  typedef struct packed {
    logic            valid;
    logic [T_W-1:0]  lo;
    logic [T_W-1:0]  hi;
    logic            is_128;
  } seg_t;

  localparam int unsigned N_SEG = 4;

  seg_t seg_a_comb [N_SEG];
  seg_t seg_b_comb [N_SEG];

  logic have_cross_comb;
  logic pair_bad;
  logic       busy_q;
  logic       done_q;
  logic       ok_q;
  logic [3:0] ptr_a_q;
  logic [3:0] ptr_b_q;
  logic [3:0] ptr_a_next;
  logic [3:0] ptr_b_next;
  logic [3:0] ptr_a_first;
  logic [3:0] ptr_b_first;
  logic       adv_a;
  logic       adv_b;
  logic       sweep_done_next;

  seg_t       cur_a;
  seg_t       cur_b;
  logic       both_valid;

  function automatic seg_t pack_seg(
    input logic            valid,
    input logic [T_W-1:0]  lo,
    input logic [T_W-1:0]  hi,
    input logic            is_128
  );
    seg_t ret;
    begin
      ret.valid = valid;
      ret.lo    = lo;
      ret.hi    = hi;
      ret.is_128 = is_128;
      pack_seg = ret;
    end
  endfunction

  task automatic build_side_segments(
    input  snap_bw_view_t sn,
    output seg_t           seg_o [N_SEG]
  );
    begin
      seg_o = '{default: '0};

      // Producer invariants make the four DMA windows ordered and disjoint:
      // S1 ends at dma1_end; S2PF starts no earlier; S2PF and S3 are mutually
      // exclusive; next-S1 prefetch starts at dma3_end.
      seg_o[0] = pack_seg(sn.valid && (|sn.bw_s1),
                           sn.task_start, sn.dma1_end, sn.bw_s1[1]);
      seg_o[1] = pack_seg(sn.valid && sn.s2pf_valid,
                           sn.s2pf_start, sn.s2pf_end, sn.s2pf_bw[1]);
      seg_o[2] = pack_seg(sn.valid && (|sn.bw_s3),
                           sn.s2_end, sn.dma3_end, sn.bw_s3[1]);
      seg_o[3] = pack_seg(sn.valid && sn.s4pf_valid, sn.s4pf_start,
                           sn.s4pf_start + GHOST_WINDOW_TICKS, 1'b0);
    end
  endtask

  function automatic logic [3:0] first_valid_ptr(input seg_t seg_i [N_SEG]);
    begin
      if (seg_i[0].valid) begin
        first_valid_ptr = 4'b0001;
      end else if (seg_i[1].valid) begin
        first_valid_ptr = 4'b0010;
      end else if (seg_i[2].valid) begin
        first_valid_ptr = 4'b0100;
      end else if (seg_i[3].valid) begin
        first_valid_ptr = 4'b1000;
      end else begin
        first_valid_ptr = 4'b0000;
      end
    end
  endfunction

  function automatic logic [3:0] next_valid_ptr(
    input logic [3:0] ptr,
    input seg_t       seg_i [N_SEG]
  );
    begin
      unique case (ptr)
        4'b0001: begin
          if (seg_i[1].valid) begin
            next_valid_ptr = 4'b0010;
          end else if (seg_i[2].valid) begin
            next_valid_ptr = 4'b0100;
          end else if (seg_i[3].valid) begin
            next_valid_ptr = 4'b1000;
          end else begin
            next_valid_ptr = 4'b0000;
          end
        end
        4'b0010: begin
          if (seg_i[2].valid) begin
            next_valid_ptr = 4'b0100;
          end else if (seg_i[3].valid) begin
            next_valid_ptr = 4'b1000;
          end else begin
            next_valid_ptr = 4'b0000;
          end
        end
        4'b0100: begin
          next_valid_ptr = seg_i[3].valid ? 4'b1000 : 4'b0000;
        end
        default: begin
          next_valid_ptr = 4'b0000;
        end
      endcase
    end
  endfunction

  function automatic seg_t seg_at_ptr(
    input logic [3:0] ptr,
    input seg_t       seg_i [N_SEG]
  );
    begin
      unique case (ptr)
        4'b0001: seg_at_ptr = seg_i[0];
        4'b0010: seg_at_ptr = seg_i[1];
        4'b0100: seg_at_ptr = seg_i[2];
        4'b1000: seg_at_ptr = seg_i[3];
        default: seg_at_ptr = '0;
      endcase
    end
  endfunction

  always_comb begin
    build_side_segments(snap_a_i, seg_a_comb);
    build_side_segments(snap_b_i, seg_b_comb);
  end

  assign ptr_a_first = first_valid_ptr(seg_a_comb);
  assign ptr_b_first = first_valid_ptr(seg_b_comb);
  assign have_cross_comb = (|ptr_a_first) && (|ptr_b_first);
  assign cur_a = seg_at_ptr(ptr_a_q, seg_a_comb);
  assign cur_b = seg_at_ptr(ptr_b_q, seg_b_comb);

  assign both_valid = cur_a.valid && cur_b.valid;
  // Segment records store only the information needed by the global BW check:
  // whether this interval consumes 128 B/cc.  64+64 is legal; any overlapping
  assign pair_bad = both_valid &&
                    (cur_a.is_128 || cur_b.is_128) &&
                    (cur_a.hi > cur_b.lo) &&
                    (cur_b.hi > cur_a.lo);

  // Ordered interval sweep:
  //   - invalid side: advance that side
  //   - both valid: advance the interval ending first, independent of overlap
  // Equal end timestamps advance both sides.
  assign adv_a = !cur_a.valid || (both_valid && (cur_a.hi <= cur_b.hi));
  assign adv_b = !cur_b.valid || (both_valid && (cur_b.hi <= cur_a.hi));
  assign ptr_a_next = adv_a ? next_valid_ptr(ptr_a_q, seg_a_comb) : ptr_a_q;
  assign ptr_b_next = adv_b ? next_valid_ptr(ptr_b_q, seg_b_comb) : ptr_b_q;
  assign sweep_done_next = !(|ptr_a_next) || !(|ptr_b_next);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      busy_q   <= 1'b0;
      done_q   <= 1'b0;
      ok_q     <= 1'b0;
      ptr_a_q  <= '0;
      ptr_b_q  <= '0;
    end else if (clear_i) begin
      busy_q   <= 1'b0;
      done_q   <= 1'b0;
      ok_q     <= 1'b0;
      ptr_a_q  <= '0;
      ptr_b_q  <= '0;
    end else begin
      done_q <= 1'b0;

      if (busy_q) begin
        if (pair_bad || sweep_done_next) begin
          if (pair_bad) begin
            ok_q <= 1'b0;
          end
          done_q <= 1'b1;
          busy_q <= 1'b0;
        end else begin
          ptr_a_q <= ptr_a_next;
          ptr_b_q <= ptr_b_next;
        end
      end else if (start_i) begin
        ok_q          <= 1'b1;
        if (have_cross_comb) begin
          ptr_a_q <= ptr_a_first;
          ptr_b_q <= ptr_b_first;
          busy_q  <= 1'b1;
        end else begin
          ptr_a_q <= '0;
          ptr_b_q <= '0;
          done_q  <= 1'b1;
          busy_q  <= 1'b0;
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
               (sn.s2pf_end <= sn.s2_end) && (sn.bw_s3 == BW_0)))
        else $error("invalid S2PF timeline contract");
      assert (!sn.s4pf_valid || (sn.s4pf_start == sn.dma3_end))
        else $error("invalid next-S1 prefetch start");
    end
  endtask

  // Pointer-only sweep 不锁存完整 segment queue；busy 期间依赖调用者保持
  // compact BW view 稳定。若输入变化，ptr_q 会指向不同的组合 segment。
  always_ff @(posedge clk_i) begin
    if (rst_ni && start_i) begin
      assert_side_contract(snap_a_i);
      assert_side_contract(snap_b_i);
    end
    if (rst_ni && busy_q && $past(busy_q)) begin
      assert ($stable(snap_a_i))
        else $error("sched_bw_ok_seq snap_a_i changed while busy");
      assert ($stable(snap_b_i))
        else $error("sched_bw_ok_seq snap_b_i changed while busy");
    end
  end
`endif

endmodule
