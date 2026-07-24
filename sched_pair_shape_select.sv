// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — paired task shape selector
//
// 时间量全部使用 Tick 域；shape 选择只依赖两侧 S2 完成 offset 的差值
//（1 Tick = 11264 cc）。
//
// S3 形状决策阈值：|S2a_end - S2b_end| ≥ shape_td3[C] = 1 tick（= ShapeC S3 DMA 持续时长）
// delta ≥ 1：两侧 S3 DMA 时间错开，各自可用 ShapeC 独占 iDMA+xDMA。
// delta = 0：两侧同时启动，ShapeC 的 BOTH 绑定会冲突；改用 ShapeB，
//            C2 绑定 iDMA、C3 绑定 xDMA，两条物理 lane 可并行。

import sched_pkg::*;

module sched_pair_shape_select (
  input  logic [NTOK_W-1:0] ntok_a_i,
  input  logic [NTOK_W-1:0] ntok_b_i,
  input  logic               s1_cache_hit_a_i,
  input  logic               full_cache_hit_a_i,
  input  logic               s1_cache_hit_b_i,
  input  logic               full_cache_hit_b_i,

  output shape_t             shape_s1_a_o,
  output shape_t             shape_s3_a_o,
  output shape_t             shape_s1_b_o,
  output shape_t             shape_s3_b_o
);

  // ── 中间信号 ──────────────────────────────────────────────────────────────
  ntok_t       s1_tail_a, s1_tail_b;
  best_ticks_t s2_offset_a, s2_offset_b;
  logic        any_s1_cache_hit;
  shape_t      selected_s1_shape;
  ntok_t       s1_tile_tokens;
  best_ticks_t s1_compute_duration;
  logic        s2_offsets_differ;

  assign any_s1_cache_hit = s1_cache_hit_a_i | s1_cache_hit_b_i;
  assign selected_s1_shape = any_s1_cache_hit ? SHAPE_C : SHAPE_B;
  // Pair S1 只会在 ShapeB/ShapeC 间选择，不会产生 ShapeA。
  // 直接用 hit_s1_any 选择 mdim/ts1，避免调用通用 shape helper 展开三路 case。
  assign s1_tile_tokens = any_s1_cache_hit ? ntok_t'(2) : ntok_t'(4);
  assign s1_compute_duration =
      any_s1_cache_hit ? best_ticks_t'(2) : best_ticks_t'(4);
  // Tick values are integral, so a non-zero delta is exactly delta >= 1.
  assign s2_offsets_differ = (s2_offset_a != s2_offset_b);

  always_comb begin
    s1_tail_a = '0;
    s1_tail_b = '0;

    // ── S1 形状选择 ────────────────────────────────────────────────────────
    // 任意一侧有 swiglu 命中时，命中侧跳过 S1 DMA；未命中侧可独占
    // iDMA+xDMA 使用 ShapeC。两侧都未命中时使用 ShapeB，分别绑定不同 lane。
    shape_s1_a_o = selected_s1_shape;
    shape_s1_b_o = selected_s1_shape;

    // ── 估算两侧 S2 完成偏移（用于 S3 形状决策）──────────────────────────
    // 两侧公共 start time 会在差值中抵消，因此这里只计算 offset，避免
    // 两条 T_W 宽的 "t0 + offset" 加法再进入 abs-diff 比较路径。
    if (s1_cache_hit_a_i) begin
      // S1 跳过：S2 直接从 t0 开始，所有 ntok 进 S2。
      s2_offset_a = best_s2_ticks(ntok_a_i);
    end else begin
      s1_tail_a = (ntok_a_i > s1_tile_tokens) ?
                  (ntok_a_i - s1_tile_tokens) : '0;
      s2_offset_a = s1_compute_duration + best_s2_ticks(s1_tail_a);
    end

    if (s1_cache_hit_b_i) begin
      s2_offset_b = best_s2_ticks(ntok_b_i);
    end else begin
      s1_tail_b = (ntok_b_i > s1_tile_tokens) ?
                  (ntok_b_i - s1_tile_tokens) : '0;
      s2_offset_b = s1_compute_duration + best_s2_ticks(s1_tail_b);
    end

    // ── S3 形状选择 ────────────────────────────────────────────────────────
    // 阈值 = shape_td3[C] = 1 tick = ShapeC S3 DMA 持续时长。
    // delta >= 1：两侧 ShapeC/BOTH 区间不重叠，可分别独占两条 DMA lane。
    // delta = 0 ：两侧 ShapeC/BOTH 会争用同一组 lane；改用 ShapeB 后
    //             C2 使用 iDMA、C3 使用 xDMA，资源集合不相交。
    // dn 命中侧 S3 DMA 会被 skip_s3 跳过，shape 无实际资源意义，
    // 统一输出 ShapeC 仅为保持字段一致性。
    if (full_cache_hit_a_i || full_cache_hit_b_i || s2_offsets_differ) begin
      // delta >= shape_td3[C]：S3 DMA 时间错开，可依次独占 BOTH
      shape_s3_a_o = SHAPE_C;
      shape_s3_b_o = SHAPE_C;
    end else begin
      // delta = 0：S3 DMA 同时启动，必须用 ShapeB 分配到不同 lane
      shape_s3_a_o = SHAPE_B;
      shape_s3_b_o = SHAPE_B;
    end
  end

endmodule
