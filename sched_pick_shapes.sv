// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — pick_shapes primitive（Tick 域版本）
//
// 等价于 moe_scheduler.c::pick_shapes()，时间量已全部转换为 Tick 域
//（1 Tick = 11264 cc），t0_i 和比较阈值均以 ticks 表示。
//
// S3 形状决策阈值：|S2a_end - S2b_end| ≥ shape_td3[C] = 1 tick（= ShapeC S3 DMA 持续时长）
// delta ≥ 1：两侧 S3 DMA 时间错开，不重叠，各用 ShapeC（128 B/cc）总峰值仍为 128 B/cc
// delta = 0：两侧 S3 DMA 同时启动，带宽叠加为 256 B/cc，超限，改用 ShapeB（64+64=128 B/cc）

import sched_pkg::*;

module sched_pick_shapes (
  input  logic [NTOK_W-1:0] ntok_a_i,
  input  logic [NTOK_W-1:0] ntok_b_i,
  input  logic               sw_a_i,   // swiglu hit（S1 pf 命中）
  input  logic               dn_a_i,   // down hit（S1+S3 full hit）
  input  logic               sw_b_i,
  input  logic               dn_b_i,
  input  logic [T_W-1:0]    t0_i,     // 两 cluster 共同基准时刻（ticks）

  output logic [1:0]  s1a_o,
  output logic [1:0]  s3a_o,
  output logic [1:0]  s1b_o,
  output logic [1:0]  s3b_o
);

  // ── 中间信号 ──────────────────────────────────────────────────────────────
  logic [NTOK_W-1:0] tail_a, tail_b;
  logic [T_W-1:0]    s2a_off, s2b_off;
  logic [T_W-1:0]    delta;
  logic              unused_t0;

  assign unused_t0 = ^t0_i;

  always_comb begin
    tail_a = '0;
    tail_b = '0;

    // ── S1 形状选择 ────────────────────────────────────────────────────────
    // 任意一侧有 swiglu 命中 → 两侧均用 ShapeC（保持带宽均衡）
    if (sw_a_i || sw_b_i) begin
      s1a_o = SHAPE_C;
      s1b_o = SHAPE_C;
    end else begin
      s1a_o = SHAPE_B;
      s1b_o = SHAPE_B;
    end

    // ── 估算两侧 S2 完成偏移（用于 S3 形状决策）──────────────────────────
    // 两侧都从同一个 t0_i 出发，S3 选择只需要 |S2a-S2b|。公共项 t0_i
    // 会在差值中抵消，因此这里只计算 offset，避免两条 T_W 宽的
    // "t0 + offset" 加法再进入 abs-diff 比较路径。
    if (sw_a_i) begin
      // S1 跳过：S2 直接从 t0 开始，所有 ntok 进 S2。
      s2a_off = best_s2_ticks(ntok_a_i);
    end else begin
      tail_a = (ntok_a_i > shape_mdim(s1a_o)) ? (ntok_a_i - shape_mdim(s1a_o)) : '0;
      s2a_off = shape_ts1(s1a_o) + best_s2_ticks(tail_a);
    end

    if (sw_b_i) begin
      s2b_off = best_s2_ticks(ntok_b_i);
    end else begin
      tail_b = (ntok_b_i > shape_mdim(s1b_o)) ? (ntok_b_i - shape_mdim(s1b_o)) : '0;
      s2b_off = shape_ts1(s1b_o) + best_s2_ticks(tail_b);
    end

    // ── S3 形状选择 ────────────────────────────────────────────────────────
    // 阈值 = shape_td3[C] = 1 tick = ShapeC S3 DMA 持续时长。
    // delta >= 1：落后侧 S3 DMA 开始时，领先侧已结束，时间不重叠；
    //             两侧均用 ShapeC（128 B/cc），峰值带宽仍在 128 B/cc 上限内。
    // delta = 0 ：两侧 S3 DMA 同时启动，128+128=256 B/cc，超出上限；
    //             改用 ShapeB（64 B/cc × 2 = 128 B/cc）。
    // dn 命中侧 S3 DMA 会被 skip_s3 跳过，shape 无实际带宽意义，
    // 统一输出 ShapeC 仅为保持字段一致性。
    delta = (s2a_off >= s2b_off) ? (s2a_off - s2b_off) : (s2b_off - s2a_off);

    if (dn_a_i || dn_b_i) begin
      s3a_o = SHAPE_C;
      s3b_o = SHAPE_C;
    end else if (delta >= T_W'(1)) begin
      // delta >= shape_td3[C]：S3 DMA 时间错开，带宽不叠加
      s3a_o = SHAPE_C;
      s3b_o = SHAPE_C;
    end else begin
      // delta = 0：S3 DMA 同时启动，必须用 ShapeB 控制总带宽
      s3a_o = SHAPE_B;
      s3b_o = SHAPE_B;
    end
  end

endmodule
