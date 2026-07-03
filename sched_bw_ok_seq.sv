// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - sequential bw_ok checker
//
// sched_bw_ok is still the semantic primitive, but using it directly from
// top-level scheduler state creates very long FPGA timing paths.  This module
// keeps the same segment semantics, but scans one A-side segment against all
// five B-side segments per cycle.  On start_i the module latches compact
// segment records derived from snap_a_i/snap_b_i; snap records are
// not copied and do not need to stay stable while busy_o is high.

import sched_pkg::*;

module sched_bw_ok_seq (
  input  logic       clk_i,
  input  logic       rst_ni,
  input  logic       start_i,
  output logic       busy_o,
  output logic       done_o,

  input  eval_snap_t snap_a_i,
  input  eval_snap_t snap_b_i,
  output logic       ok_o
);

  typedef struct packed {
    logic            valid;
    logic [T_W-1:0]  lo;
    logic [T_W-1:0]  hi;
    logic [BW_W-1:0] bw;
  } seg_t;

  seg_t seg_a_comb [5];
  seg_t seg_b_comb [5];
  (* max_fanout = 16 *) seg_t seg_a_q [5];
  (* max_fanout = 16 *) seg_t seg_b_q [5];

  logic ok_single_a;
  logic ok_single_b;
  logic ok_single_comb;
  logic row_cross_ok;
  logic ok_next;
  logic       busy_q;
  logic       done_q;
  logic       ok_q;
  logic [2:0] row_q;

  int unsigned idx_a;
  int unsigned idx_b;

  logic has1_a, has4_a, has3_a, has5_a;
  logic has1_b, has4_b, has3_b, has5_b;
  logic [T_W-1:0] ovl_lo_a, ovl_hi_a;
  logic [T_W-1:0] ovl_lo_b, ovl_hi_b;
  logic [BW_W:0] merged_a;
  logic [BW_W:0] merged_b;

  logic [T_W-1:0] cross_lo;
  logic [T_W-1:0] cross_hi;
  logic [BW_W:0]  cross_bw;

  always_comb begin
    for (int i = 0; i < 5; i++) begin
      seg_a_comb[i] = '0;
      seg_b_comb[i] = '0;
    end

    idx_a       = 0;
    idx_b       = 0;
    ok_single_a = 1'b1;
    ok_single_b = 1'b1;

    has1_a = snap_a_i.valid &&
             (snap_a_i.bw_s1 != BW_0) &&
             (snap_a_i.dma1_end > snap_a_i.task_start);
    has4_a = snap_a_i.valid &&
             snap_a_i.s2pf_valid &&
             (snap_a_i.s2pf_bw != BW_0) &&
             (snap_a_i.s2pf_end > snap_a_i.s2pf_start);
    has3_a = snap_a_i.valid &&
             (snap_a_i.bw_s3 != BW_0) &&
             (snap_a_i.dma3_end > snap_a_i.s2_end);
    has5_a = snap_a_i.valid && snap_a_i.s4pf_valid;

    if (has1_a && has4_a &&
        (snap_a_i.task_start < snap_a_i.s2pf_end) &&
        (snap_a_i.s2pf_start < snap_a_i.dma1_end)) begin
      ovl_lo_a = (snap_a_i.task_start > snap_a_i.s2pf_start) ?
                 snap_a_i.task_start : snap_a_i.s2pf_start;
      ovl_hi_a = (snap_a_i.dma1_end < snap_a_i.s2pf_end) ?
                 snap_a_i.dma1_end : snap_a_i.s2pf_end;
      merged_a = {1'b0, snap_a_i.bw_s1} + {1'b0, snap_a_i.s2pf_bw};
      if (merged_a > {1'b0, BW_128}) begin
        ok_single_a = 1'b0;
      end

      if (snap_a_i.task_start < snap_a_i.s2pf_start) begin
        seg_a_comb[idx_a].valid = 1'b1;
        seg_a_comb[idx_a].lo    = snap_a_i.task_start;
        seg_a_comb[idx_a].hi    = snap_a_i.s2pf_start;
        seg_a_comb[idx_a].bw    = snap_a_i.bw_s1;
        idx_a++;
      end else if (snap_a_i.s2pf_start < snap_a_i.task_start) begin
        seg_a_comb[idx_a].valid = 1'b1;
        seg_a_comb[idx_a].lo    = snap_a_i.s2pf_start;
        seg_a_comb[idx_a].hi    = snap_a_i.task_start;
        seg_a_comb[idx_a].bw    = snap_a_i.s2pf_bw;
        idx_a++;
      end

      if (ovl_hi_a > ovl_lo_a) begin
        seg_a_comb[idx_a].valid = 1'b1;
        seg_a_comb[idx_a].lo    = ovl_lo_a;
        seg_a_comb[idx_a].hi    = ovl_hi_a;
        seg_a_comb[idx_a].bw    = merged_a[BW_W-1:0];
        idx_a++;
      end

      if (snap_a_i.dma1_end > snap_a_i.s2pf_end) begin
          seg_a_comb[idx_a].valid = 1'b1;
          seg_a_comb[idx_a].lo    = snap_a_i.s2pf_end;
          seg_a_comb[idx_a].hi    = snap_a_i.dma1_end;
          seg_a_comb[idx_a].bw    = snap_a_i.bw_s1;
        idx_a++;
      end else if (snap_a_i.s2pf_end > snap_a_i.dma1_end) begin
          seg_a_comb[idx_a].valid = 1'b1;
          seg_a_comb[idx_a].lo    = snap_a_i.dma1_end;
          seg_a_comb[idx_a].hi    = snap_a_i.s2pf_end;
          seg_a_comb[idx_a].bw    = snap_a_i.s2pf_bw;
        idx_a++;
      end
    end else begin
      if (has1_a) begin
        seg_a_comb[idx_a].valid = 1'b1;
        seg_a_comb[idx_a].lo    = snap_a_i.task_start;
        seg_a_comb[idx_a].hi    = snap_a_i.dma1_end;
        seg_a_comb[idx_a].bw    = snap_a_i.bw_s1;
        idx_a++;
      end
      if (has4_a) begin
        seg_a_comb[idx_a].valid = 1'b1;
        seg_a_comb[idx_a].lo    = snap_a_i.s2pf_start;
        seg_a_comb[idx_a].hi    = snap_a_i.s2pf_end;
        seg_a_comb[idx_a].bw    = snap_a_i.s2pf_bw;
        idx_a++;
      end
    end

    if (has3_a) begin
      seg_a_comb[idx_a].valid = 1'b1;
      seg_a_comb[idx_a].lo    = snap_a_i.s2_end;
      seg_a_comb[idx_a].hi    = snap_a_i.dma3_end;
      seg_a_comb[idx_a].bw    = snap_a_i.bw_s3;
      idx_a++;
    end

    if (has5_a) begin
      seg_a_comb[idx_a].valid = 1'b1;
      seg_a_comb[idx_a].lo    = snap_a_i.s4pf_start;
      seg_a_comb[idx_a].hi    = snap_a_i.s4pf_start + GHOST_WINDOW_TICKS;
      seg_a_comb[idx_a].bw    = BW_64;
      idx_a++;
    end

    has1_b = snap_b_i.valid &&
             (snap_b_i.bw_s1 != BW_0) &&
             (snap_b_i.dma1_end > snap_b_i.task_start);
    has4_b = snap_b_i.valid &&
             snap_b_i.s2pf_valid &&
             (snap_b_i.s2pf_bw != BW_0) &&
             (snap_b_i.s2pf_end > snap_b_i.s2pf_start);
    has3_b = snap_b_i.valid &&
             (snap_b_i.bw_s3 != BW_0) &&
             (snap_b_i.dma3_end > snap_b_i.s2_end);
    has5_b = snap_b_i.valid && snap_b_i.s4pf_valid;

    if (has1_b && has4_b &&
        (snap_b_i.task_start < snap_b_i.s2pf_end) &&
        (snap_b_i.s2pf_start < snap_b_i.dma1_end)) begin
      ovl_lo_b = (snap_b_i.task_start > snap_b_i.s2pf_start) ?
                 snap_b_i.task_start : snap_b_i.s2pf_start;
      ovl_hi_b = (snap_b_i.dma1_end < snap_b_i.s2pf_end) ?
                 snap_b_i.dma1_end : snap_b_i.s2pf_end;
      merged_b = {1'b0, snap_b_i.bw_s1} + {1'b0, snap_b_i.s2pf_bw};
      if (merged_b > {1'b0, BW_128}) begin
        ok_single_b = 1'b0;
      end

      if (snap_b_i.task_start < snap_b_i.s2pf_start) begin
        seg_b_comb[idx_b].valid = 1'b1;
        seg_b_comb[idx_b].lo    = snap_b_i.task_start;
        seg_b_comb[idx_b].hi    = snap_b_i.s2pf_start;
        seg_b_comb[idx_b].bw    = snap_b_i.bw_s1;
        idx_b++;
      end else if (snap_b_i.s2pf_start < snap_b_i.task_start) begin
        seg_b_comb[idx_b].valid = 1'b1;
        seg_b_comb[idx_b].lo    = snap_b_i.s2pf_start;
        seg_b_comb[idx_b].hi    = snap_b_i.task_start;
        seg_b_comb[idx_b].bw    = snap_b_i.s2pf_bw;
        idx_b++;
      end

      if (ovl_hi_b > ovl_lo_b) begin
        seg_b_comb[idx_b].valid = 1'b1;
        seg_b_comb[idx_b].lo    = ovl_lo_b;
        seg_b_comb[idx_b].hi    = ovl_hi_b;
        seg_b_comb[idx_b].bw    = merged_b[BW_W-1:0];
        idx_b++;
      end

      if (snap_b_i.dma1_end > snap_b_i.s2pf_end) begin
        seg_b_comb[idx_b].valid = 1'b1;
        seg_b_comb[idx_b].lo    = snap_b_i.s2pf_end;
        seg_b_comb[idx_b].hi    = snap_b_i.dma1_end;
        seg_b_comb[idx_b].bw    = snap_b_i.bw_s1;
        idx_b++;
      end else if (snap_b_i.s2pf_end > snap_b_i.dma1_end) begin
        seg_b_comb[idx_b].valid = 1'b1;
        seg_b_comb[idx_b].lo    = snap_b_i.dma1_end;
        seg_b_comb[idx_b].hi    = snap_b_i.s2pf_end;
        seg_b_comb[idx_b].bw    = snap_b_i.s2pf_bw;
        idx_b++;
      end
    end else begin
      if (has1_b) begin
        seg_b_comb[idx_b].valid = 1'b1;
        seg_b_comb[idx_b].lo    = snap_b_i.task_start;
        seg_b_comb[idx_b].hi    = snap_b_i.dma1_end;
        seg_b_comb[idx_b].bw    = snap_b_i.bw_s1;
        idx_b++;
      end
      if (has4_b) begin
        seg_b_comb[idx_b].valid = 1'b1;
        seg_b_comb[idx_b].lo    = snap_b_i.s2pf_start;
        seg_b_comb[idx_b].hi    = snap_b_i.s2pf_end;
        seg_b_comb[idx_b].bw    = snap_b_i.s2pf_bw;
        idx_b++;
      end
    end

    if (has3_b) begin
      seg_b_comb[idx_b].valid = 1'b1;
      seg_b_comb[idx_b].lo    = snap_b_i.s2_end;
      seg_b_comb[idx_b].hi    = snap_b_i.dma3_end;
      seg_b_comb[idx_b].bw    = snap_b_i.bw_s3;
      idx_b++;
    end

    if (has5_b) begin
      seg_b_comb[idx_b].valid = 1'b1;
      seg_b_comb[idx_b].lo    = snap_b_i.s4pf_start;
      seg_b_comb[idx_b].hi    = snap_b_i.s4pf_start + GHOST_WINDOW_TICKS;
      seg_b_comb[idx_b].bw    = BW_64;
      idx_b++;
    end
  end

  assign ok_single_comb = ok_single_a && ok_single_b;

  always_comb begin
    row_cross_ok = 1'b1;
    for (int ib = 0; ib < 5; ib++) begin
      cross_lo = (seg_a_q[row_q].lo > seg_b_q[ib].lo) ?
                 seg_a_q[row_q].lo : seg_b_q[ib].lo;
      cross_hi = (seg_a_q[row_q].hi < seg_b_q[ib].hi) ?
                 seg_a_q[row_q].hi : seg_b_q[ib].hi;
      cross_bw = {1'b0, seg_a_q[row_q].bw} + {1'b0, seg_b_q[ib].bw};
      if (seg_a_q[row_q].valid && seg_b_q[ib].valid &&
          (cross_lo < cross_hi) &&
          (cross_bw > {1'b0, BW_128})) begin
        row_cross_ok = 1'b0;
      end
    end
  end

  assign ok_next = ok_q && row_cross_ok;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      busy_q   <= 1'b0;
      done_q   <= 1'b0;
      ok_q     <= 1'b0;
      row_q    <= '0;
      seg_a_q  <= '{default: '0};
      seg_b_q  <= '{default: '0};
    end else begin
      done_q <= 1'b0;

      if (busy_q) begin
        ok_q <= ok_next;
        if (row_q == 3'd4) begin
          done_q <= 1'b1;
          busy_q <= 1'b0;
          row_q  <= '0;
        end else begin
          row_q <= row_q + 3'd1;
        end
      end else if (start_i) begin
        seg_a_q <= seg_a_comb;
        seg_b_q <= seg_b_comb;
        ok_q   <= ok_single_comb;
        row_q  <= '0;
        busy_q <= 1'b1;
      end
    end
  end

  assign busy_o = busy_q;
  assign done_o = done_q;
  assign ok_o   = ok_q;

endmodule
