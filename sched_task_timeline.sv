// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — task timeline generator（Tick 域版本）
//
// 本模块等价于软件 snap timing builder，但所有时间量均以 Tick 为单位
// （1 Tick = 11264 clock cycles），使每个时间字段从 32 bit 压缩到 T_W bit。
//
// 时序常量（tick 域，原始值 ÷ 11264）：
//   shape_ts1: A=8, B=4, C=2    shape_td1: A=4, B=4, C=2
//   shape_ts3: A=4, B=2, C=1    shape_td3: A=2, B=2, C=1
//   M_dim: A=8, B=4, C=2
//
// best_s4(r) = ((r+1)/2) ticks      → 硬件实现：(r+1)>>1（加法+右移，无乘法）
// best_s2(r) = 2 × best_s4(r) ticks → 硬件实现：best_s4(r) << 1
//
// DMA 资源绑定由 evaluator 根据 cluster/shape/cache-hit 直接生成；本模块只负责
// 时间 endpoint，避免 shape decode 在 timeline 和资源检查路径重复物化。

import sched_pkg::*;

module sched_task_timeline (
  // ── 输入 ─────────────────────────────────────────────────────────────────
  input  logic [T_W-1:0]   start_t_i,   // cluster 当前可用时刻（ticks）
  input  logic [NTOK_W-1:0] ntok_i,     // 本 expert 的 token 数（≤511）
  input  logic [1:0]        shape_s1_i, // Shape A/B/C
  input  logic [1:0]        shape_s3_i,
  input  logic              skip_s1_i,  // 1=S1 GEMM+DMA 已命中缓存
  input  logic              skip_s3_i,  // 1=S3 GEMM+DMA 已命中缓存

  // ── 时间输出（ticks，T_W follows sched_pkg::E_MAX）─────────────────────
  output logic [T_W-1:0]   task_end_o,
  output logic [T_W-1:0]   dma1_end_o,
  output logic [T_W-1:0]   s2_end_o,
  output logic [T_W-1:0]   dma3_end_o
);

  typedef struct packed {
    ntok_t half_tile_tokens;
    time_t compute_duration;
    time_t dma_duration;
  } shape_timing_t;

  function automatic shape_timing_t decode_s1_shape(input shape_t shape);
    shape_timing_t timing;
    begin
      timing = '0;
      unique case (shape)
        SHAPE_A: begin
          timing.half_tile_tokens = ntok_t'(4);
          timing.compute_duration = time_t'(8);
          timing.dma_duration     = time_t'(4);
        end
        SHAPE_B: begin
          timing.half_tile_tokens = ntok_t'(2);
          timing.compute_duration = time_t'(4);
          timing.dma_duration     = time_t'(4);
        end
        SHAPE_C: begin
          timing.half_tile_tokens = ntok_t'(1);
          timing.compute_duration = time_t'(2);
          timing.dma_duration     = time_t'(2);
        end
        default: timing = 'x;
      endcase
      decode_s1_shape = timing;
    end
  endfunction

  function automatic shape_timing_t decode_s3_shape(input shape_t shape);
    shape_timing_t timing;
    begin
      timing = '0;
      unique case (shape)
        SHAPE_A: begin
          timing.half_tile_tokens = ntok_t'(4);
          timing.compute_duration = time_t'(4);
          timing.dma_duration     = time_t'(2);
        end
        SHAPE_B: begin
          timing.half_tile_tokens = ntok_t'(2);
          timing.compute_duration = time_t'(2);
          timing.dma_duration     = time_t'(2);
        end
        SHAPE_C: begin
          timing.half_tile_tokens = ntok_t'(1);
          timing.compute_duration = time_t'(1);
          timing.dma_duration     = time_t'(1);
        end
        default: timing = 'x;
      endcase
      decode_s3_shape = timing;
    end
  endfunction

  // ── 中间信号 ──────────────────────────────────────────────────────────────
  ntok_t half_tokens_ceil;
  ntok_t s1_tail_half_ceil;
  ntok_t s3_tail_half_ceil;
  time_t full_s2_duration;
  time_t full_s4_duration;
  time_t s1_tail_s2_duration;
  time_t s3_tail_s4_duration;
  shape_timing_t s1_timing;
  shape_timing_t s3_timing;
  time_t s1_compute_offset;
  time_t dma1_end_offset;
  time_t s2_duration;
  time_t s3_duration;
  time_t dma3_duration;
  time_t s4_duration;
  // s2_end/dma3_end/task_end share the same first three operands.  Preserve
  // that prefix as one carry-save pair instead of rebuilding an identical
  // 3:2 compressor independently for every endpoint.
  time_t common_sum;
  time_t common_carry;
  time_t dma3_sum;
  time_t dma3_carry;
  time_t task_sum_stage1;
  time_t task_carry_stage1;
  time_t task_sum_stage2;
  time_t task_carry_stage2;
  time_t s2_endpoint;
  time_t dma3_endpoint;
  time_t task_endpoint;

  // best_s4(x)=ceil(x/2)，best_s2(x)=ceil(x/2)<<1。
  // 这里显式共享 ceil_div2 结果，避免 best_s2_ticks/best_s4_ticks
  // 对同一个 token 数重复展开小加法器。
  assign half_tokens_ceil = ceil_div2_ntok(ntok_i);
  // M_dim is always even.  Therefore:
  //   ceil(max(ntok-M,0)/2) = max(ceil(ntok/2)-M/2,0)
  // Share one ceil unit and subtract small shape constants afterwards.
  assign s1_tail_half_ceil =
      (half_tokens_ceil > s1_timing.half_tile_tokens) ?
      (half_tokens_ceil - s1_timing.half_tile_tokens) : '0;
  assign s3_tail_half_ceil =
      (half_tokens_ceil > s3_timing.half_tile_tokens) ?
      (half_tokens_ceil - s3_timing.half_tile_tokens) : '0;
  assign full_s4_duration = time_t'(half_tokens_ceil);
  assign full_s2_duration = time_t'({half_tokens_ceil, 1'b0});
  assign s1_tail_s2_duration = time_t'({s1_tail_half_ceil, 1'b0});
  assign s3_tail_s4_duration = time_t'(s3_tail_half_ceil);

  // Shape 解码集中成 stage-local attribute。对同一个 shape 只做一次
  // case，直接产出 tile/compute/DMA timing，避免 shape_mdim/shape_ts*/shape_td*
  // 分散展开成多套等价 mux。
  assign s1_timing = decode_s1_shape(shape_s1_i);
  assign s3_timing = decode_s3_shape(shape_s3_i);

  // ── 时序计算（组合逻辑）─────────────────────────────────────────────────
  // 直接生成最终可见 endpoint，不物化 front/back 中间和。三个及以上
  // 操作数先经 3:2 compressor 规约，最终只做一次 carry-propagate addition。
  always_comb begin
    // ── S1 / S2 阶段 ───────────────────────────────────────────────────────
    if (skip_s1_i) begin
      dma1_end_offset = '0;
      s1_compute_offset = '0;
      s2_duration = full_s2_duration;
    end else begin
      dma1_end_offset = s1_timing.dma_duration;
      s1_compute_offset = s1_timing.compute_duration;
      s2_duration = s1_tail_s2_duration;
    end

    // ── S3 / S4 阶段 ───────────────────────────────────────────────────────
    if (skip_s3_i) begin
      dma3_duration = '0;
      s3_duration = '0;
      s4_duration = full_s4_duration;
    end else begin
      dma3_duration = s3_timing.dma_duration;
      s3_duration = s3_timing.compute_duration;
      s4_duration = s3_tail_s4_duration;
    end
  end

  // T_W is sized for the complete scheduling horizon, so every visible
  // endpoint is representable without guard bits.  Keeping compressor rows at
  // T_W avoids widening all endpoint adders for bits that are never observed.
  assign common_sum = start_t_i ^ s1_compute_offset ^ s2_duration;
  assign common_carry = ((start_t_i & s1_compute_offset) |
                         (start_t_i & s2_duration) |
                         (s1_compute_offset & s2_duration)) << 1;

  assign s2_endpoint = common_sum + common_carry;

  assign dma3_sum = common_sum ^ common_carry ^ dma3_duration;
  assign dma3_carry = ((common_sum & common_carry) |
                       (common_sum & dma3_duration) |
                       (common_carry & dma3_duration)) << 1;
  assign dma3_endpoint = dma3_sum + dma3_carry;

  assign task_sum_stage1 = common_sum ^ common_carry ^ s3_duration;
  assign task_carry_stage1 = ((common_sum & common_carry) |
                              (common_sum & s3_duration) |
                              (common_carry & s3_duration)) << 1;
  assign task_sum_stage2 = task_sum_stage1 ^ task_carry_stage1 ^ s4_duration;
  assign task_carry_stage2 = ((task_sum_stage1 & task_carry_stage1) |
                              (task_sum_stage1 & s4_duration) |
                              (task_carry_stage1 & s4_duration)) << 1;
  assign task_endpoint = task_sum_stage2 + task_carry_stage2;

  assign dma1_end_o = start_t_i + dma1_end_offset;
  assign s2_end_o   = s2_endpoint;
  assign dma3_end_o = dma3_endpoint;
  assign task_end_o = task_endpoint;

endmodule
