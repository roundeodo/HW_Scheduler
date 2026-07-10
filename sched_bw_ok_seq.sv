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

  logic ok_single_a;
  logic ok_single_b;
  logic ok_single_comb;
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
    output seg_t           seg_o [N_SEG],
    output logic           ok_single_o
  );
    logic has_s1;
    logic has_s2pf;
    logic has_s3;
    logic has_s4pf;
    logic s1_s2pf_overlap;
    logic [T_W-1:0] ovl_hi;
    logic s1_is_128;
    logic s2pf_is_128;
    logic s3_is_128;
    begin
      seg_o = '{default: '0};
      ok_single_o = 1'b1;

      // BW encoding is fixed to 00/01/10.  In this checker the exact 64 value
      // is not needed: nonzero means an active DMA interval, bit[1] means
      // 128 B/cc and therefore makes any cross-cluster overlap illegal.
      s1_is_128   = sn.bw_s1[1];
      s2pf_is_128 = sn.s2pf_bw[1];
      s3_is_128   = sn.bw_s3[1];

      // Producers already encode skipped DMA with BW_0 and valid S2PF with
      // s2pf_valid.  Do not repeat interval-length guards here; this module
      // only consumes the compact timeline contract.
      has_s1   = sn.valid && (|sn.bw_s1);
      has_s2pf = sn.valid && sn.s2pf_valid;
      has_s3   = sn.valid && (|sn.bw_s3);
      has_s4pf = sn.valid && sn.s4pf_valid;

      // Current S2PF policy only creates pf_start >= task_start.  Therefore
      // S1/S2PF can only be ordered as:
      //   task_start <= s2pf_start < dma1_end  : local overlap
      //   task_start <  dma1_end <= s2pf_start : no overlap, S1 then S2PF
      // Branches for S2PF-before-S1 are unreachable and intentionally removed.
      s1_s2pf_overlap = has_s1 && has_s2pf &&
                         (sn.s2pf_start < sn.dma1_end);

      if (s1_s2pf_overlap) begin
        ovl_hi = (sn.dma1_end < sn.s2pf_end) ? sn.dma1_end : sn.s2pf_end;
        // In the overlap region both S1 and S2PF are active and nonzero.
        // With 0/64/128 encoding, legal overlap can only be 64+64=128.
        // If either side is already 128, this single cluster is over limit.
        if (s1_is_128 || s2pf_is_128) begin
          ok_single_o = 1'b0;
        end

        seg_o[0] = pack_seg(sn.task_start < sn.s2pf_start,
                            sn.task_start, sn.s2pf_start,
                            s1_is_128);

        seg_o[1] = pack_seg(1'b1, sn.s2pf_start, ovl_hi, 1'b1);

        if (sn.s2pf_end <= sn.dma1_end) begin
          seg_o[2] = pack_seg(sn.s2pf_end < sn.dma1_end,
                              sn.s2pf_end, sn.dma1_end,
                              s1_is_128);
        end else begin
          seg_o[2] = pack_seg(1'b1, sn.dma1_end, sn.s2pf_end,
                              s2pf_is_128);
        end
      end else begin
        if (has_s1 && has_s2pf) begin
          seg_o[0] = pack_seg(1'b1, sn.task_start, sn.dma1_end,
                              s1_is_128);
          seg_o[2] = pack_seg(1'b1, sn.s2pf_start, sn.s2pf_end,
                              s2pf_is_128);
        end else if (has_s1) begin
          seg_o[0] = pack_seg(1'b1, sn.task_start, sn.dma1_end,
                              s1_is_128);
        end else if (has_s2pf) begin
          seg_o[0] = pack_seg(1'b1, sn.s2pf_start, sn.s2pf_end,
                              s2pf_is_128);
        end
      end

      // S2PF and S3 are mutually exclusive in the current RTL producer:
      // apply_s2pf() sets bw_s3=0 when it sets s2pf_valid.  Therefore the
      // "later work" slot can hold either S1/S2PF tail or S3 DMA.
      if (!has_s2pf && has_s3) begin
        seg_o[2] = pack_seg(1'b1, sn.s2_end, sn.dma3_end, s3_is_128);
      end

      seg_o[3] = pack_seg(has_s4pf, sn.s4pf_start,
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
    build_side_segments(snap_a_i, seg_a_comb, ok_single_a);
    build_side_segments(snap_b_i, seg_b_comb, ok_single_b);
  end

  assign ok_single_comb = ok_single_a && ok_single_b;
  assign ptr_a_first = first_valid_ptr(seg_a_comb);
  assign ptr_b_first = first_valid_ptr(seg_b_comb);
  assign have_cross_comb = ok_single_comb && (|ptr_a_first) && (|ptr_b_first);
  assign cur_a = seg_at_ptr(ptr_a_q, seg_a_comb);
  assign cur_b = seg_at_ptr(ptr_b_q, seg_b_comb);

  assign both_valid = cur_a.valid && cur_b.valid;
  // Segment records store only the information needed by the global BW check:
  // whether this interval consumes 128 B/cc.  64+64 is legal; any overlapping
  // interval containing 128 B/cc is illegal.  No adder is required here.
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
        ok_q          <= ok_single_comb;
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
  // Pointer-only sweep 不锁存完整 segment queue；busy 期间依赖调用者保持
  // compact BW view 稳定。若输入变化，ptr_q 会指向不同的组合 segment。
  always_ff @(posedge clk_i) begin
    if (rst_ni && busy_q && $past(busy_q)) begin
      assert ($stable(snap_a_i))
        else $error("sched_bw_ok_seq snap_a_i changed while busy");
      assert ($stable(snap_b_i))
        else $error("sched_bw_ok_seq snap_b_i changed while busy");
    end
  end
`endif

endmodule
