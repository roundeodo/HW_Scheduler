// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — bw_ok primitive（Tick 域版本）
//
// 等价于 moe_scheduler.c::snap_segs() + bw_ok() 的组合逻辑形式。
// BW 使用 sched_pkg 的 2-bit 档位：0/1/2 分别表示 0/64/128 B/cc，
// 因此任意重叠 segment 的 bw_sum 必须 <= BW_128。

import sched_pkg::*;

module sched_bw_ok (
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

  seg_t seg_a [5];
  seg_t seg_b [5];

  logic ok_single_a;
  logic ok_single_b;
  logic ok_cross;

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
      seg_a[i] = '0;
      seg_b[i] = '0;
    end

    idx_a       = 0;
    idx_b       = 0;
    ok_single_a = 1'b1;
    ok_single_b = 1'b1;
    ok_cross    = 1'b1;

    // ── A side: merge S1 DMA and S2PF DMA if they overlap ────────────────
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
    has5_a = snap_a_i.valid &&
             snap_a_i.s4pf_valid;

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

      // S1 DMA 和 S2PF DMA 已确认有 overlap。下面不是再次判断合法性，
      // 而是把两个 interval 切成 piecewise-constant BW segments，便于后续
      // 和另一侧 snap 做 cross-snap BW 检查。
      //
      // pre-overlap 段：谁先开始，谁就在 overlap 之前单独占用带宽。
      //   S1   : [task_start, dma1_end)
      //   S2PF : [s2pf_start, s2pf_end)
      // 若 task_start < s2pf_start，则 [task_start, s2pf_start) 只有 S1 DMA；
      // 若 s2pf_start < task_start，则 [s2pf_start, task_start) 只有 S2PF DMA。
      if (snap_a_i.task_start < snap_a_i.s2pf_start) begin
        seg_a[idx_a].valid = 1'b1;
        seg_a[idx_a].lo    = snap_a_i.task_start;
        seg_a[idx_a].hi    = snap_a_i.s2pf_start;
        seg_a[idx_a].bw    = snap_a_i.bw_s1;
        idx_a++;
      end else if (snap_a_i.s2pf_start < snap_a_i.task_start) begin
        seg_a[idx_a].valid = 1'b1;
        seg_a[idx_a].lo    = snap_a_i.s2pf_start;
        seg_a[idx_a].hi    = snap_a_i.task_start;
        seg_a[idx_a].bw    = snap_a_i.s2pf_bw;
        idx_a++;
      end

      // overlap 段：S1 DMA 与 S2PF DMA 同时活动，带宽使用量相加。
      // 这段已经在上面检查过 merged_a <= BW_128；仍然需要保留下来，
      // 因为另一侧 cluster 在同一时间若也占 DMA，会通过 cross check 被过滤。
      if (ovl_hi_a > ovl_lo_a) begin
        seg_a[idx_a].valid = 1'b1;
        seg_a[idx_a].lo    = ovl_lo_a;
        seg_a[idx_a].hi    = ovl_hi_a;
        seg_a[idx_a].bw    = merged_a[BW_W-1:0];
        idx_a++;
      end

      // post-overlap 段：谁更晚结束，谁就在 overlap 之后继续单独占用带宽。
      if (snap_a_i.dma1_end > snap_a_i.s2pf_end) begin
        seg_a[idx_a].valid = 1'b1;
        seg_a[idx_a].lo    = snap_a_i.s2pf_end;
        seg_a[idx_a].hi    = snap_a_i.dma1_end;
        seg_a[idx_a].bw    = snap_a_i.bw_s1;
        idx_a++;
      end else if (snap_a_i.s2pf_end > snap_a_i.dma1_end) begin
        seg_a[idx_a].valid = 1'b1;
        seg_a[idx_a].lo    = snap_a_i.dma1_end;
        seg_a[idx_a].hi    = snap_a_i.s2pf_end;
        seg_a[idx_a].bw    = snap_a_i.s2pf_bw;
        idx_a++;
      end
    end else begin
      if (has1_a) begin
        seg_a[idx_a].valid = 1'b1;
        seg_a[idx_a].lo    = snap_a_i.task_start;
        seg_a[idx_a].hi    = snap_a_i.dma1_end;
        seg_a[idx_a].bw    = snap_a_i.bw_s1;
        idx_a++;
      end
      if (has4_a) begin
        seg_a[idx_a].valid = 1'b1;
        seg_a[idx_a].lo    = snap_a_i.s2pf_start;
        seg_a[idx_a].hi    = snap_a_i.s2pf_end;
        seg_a[idx_a].bw    = snap_a_i.s2pf_bw;
        idx_a++;
      end
    end

    if (has3_a) begin
      seg_a[idx_a].valid = 1'b1;
      seg_a[idx_a].lo    = snap_a_i.s2_end;
      seg_a[idx_a].hi    = snap_a_i.dma3_end;
      seg_a[idx_a].bw    = snap_a_i.bw_s3;
      idx_a++;
    end

    if (has5_a) begin
      // S4PF: ShapeA gate/up prefetch during S4.  Duration and BW are fixed,
      // so eval_snap_t only stores valid+start.
      seg_a[idx_a].valid = 1'b1;
      seg_a[idx_a].lo    = snap_a_i.s4pf_start;
      seg_a[idx_a].hi    = snap_a_i.s4pf_start + GHOST_WINDOW_TICKS;
      seg_a[idx_a].bw    = BW_64;
      idx_a++;
    end

    // ── B side: same segment construction ────────────────────────────────
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
    has5_b = snap_b_i.valid &&
             snap_b_i.s4pf_valid;

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

      // B side 同 A side：把 S1/S2PF overlap 切成
      // pre-overlap / overlap / post-overlap 三类 segment。
      if (snap_b_i.task_start < snap_b_i.s2pf_start) begin
        seg_b[idx_b].valid = 1'b1;
        seg_b[idx_b].lo    = snap_b_i.task_start;
        seg_b[idx_b].hi    = snap_b_i.s2pf_start;
        seg_b[idx_b].bw    = snap_b_i.bw_s1;
        idx_b++;
      end else if (snap_b_i.s2pf_start < snap_b_i.task_start) begin
        seg_b[idx_b].valid = 1'b1;
        seg_b[idx_b].lo    = snap_b_i.s2pf_start;
        seg_b[idx_b].hi    = snap_b_i.task_start;
        seg_b[idx_b].bw    = snap_b_i.s2pf_bw;
        idx_b++;
      end

      // overlap 段：B 侧 S1 DMA 与 S2PF DMA 同时活动，带宽使用量相加。
      if (ovl_hi_b > ovl_lo_b) begin
        seg_b[idx_b].valid = 1'b1;
        seg_b[idx_b].lo    = ovl_lo_b;
        seg_b[idx_b].hi    = ovl_hi_b;
        seg_b[idx_b].bw    = merged_b[BW_W-1:0];
        idx_b++;
      end

      // post-overlap 段：B 侧谁更晚结束，谁继续单独占用带宽。
      if (snap_b_i.dma1_end > snap_b_i.s2pf_end) begin
        seg_b[idx_b].valid = 1'b1;
        seg_b[idx_b].lo    = snap_b_i.s2pf_end;
        seg_b[idx_b].hi    = snap_b_i.dma1_end;
        seg_b[idx_b].bw    = snap_b_i.bw_s1;
        idx_b++;
      end else if (snap_b_i.s2pf_end > snap_b_i.dma1_end) begin
        seg_b[idx_b].valid = 1'b1;
        seg_b[idx_b].lo    = snap_b_i.dma1_end;
        seg_b[idx_b].hi    = snap_b_i.s2pf_end;
        seg_b[idx_b].bw    = snap_b_i.s2pf_bw;
        idx_b++;
      end
    end else begin
      if (has1_b) begin
        seg_b[idx_b].valid = 1'b1;
        seg_b[idx_b].lo    = snap_b_i.task_start;
        seg_b[idx_b].hi    = snap_b_i.dma1_end;
        seg_b[idx_b].bw    = snap_b_i.bw_s1;
        idx_b++;
      end
      if (has4_b) begin
        seg_b[idx_b].valid = 1'b1;
        seg_b[idx_b].lo    = snap_b_i.s2pf_start;
        seg_b[idx_b].hi    = snap_b_i.s2pf_end;
        seg_b[idx_b].bw    = snap_b_i.s2pf_bw;
        idx_b++;
      end
    end

    if (has3_b) begin
      seg_b[idx_b].valid = 1'b1;
      seg_b[idx_b].lo    = snap_b_i.s2_end;
      seg_b[idx_b].hi    = snap_b_i.dma3_end;
      seg_b[idx_b].bw    = snap_b_i.bw_s3;
      idx_b++;
    end

    if (has5_b) begin
      // S4PF: ShapeA gate/up prefetch during S4.  Duration and BW are fixed.
      seg_b[idx_b].valid = 1'b1;
      seg_b[idx_b].lo    = snap_b_i.s4pf_start;
      seg_b[idx_b].hi    = snap_b_i.s4pf_start + GHOST_WINDOW_TICKS;
      seg_b[idx_b].bw    = BW_64;
      idx_b++;
    end

    // ── Cross-snap overlap check ──────────────────────────────────────────
    for (int ia = 0; ia < 5; ia++) begin
      for (int ib = 0; ib < 5; ib++) begin
        cross_lo = (seg_a[ia].lo > seg_b[ib].lo) ? seg_a[ia].lo : seg_b[ib].lo;
        cross_hi = (seg_a[ia].hi < seg_b[ib].hi) ? seg_a[ia].hi : seg_b[ib].hi;
        cross_bw = {1'b0, seg_a[ia].bw} + {1'b0, seg_b[ib].bw};
        if (seg_a[ia].valid && seg_b[ib].valid &&
            (cross_lo < cross_hi) &&
            (cross_bw > {1'b0, BW_128})) begin
          ok_cross = 1'b0;
        end
      end
    end

    ok_o = ok_single_a && ok_single_b && ok_cross;
  end

endmodule
