// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — generic mk_snap primitive（Tick 域版本）
//
// 本模块等价于 moe_scheduler.c::mk_snap()，但所有时间量均以 Tick 为单位
// （1 Tick = 11264 clock cycles），使每个时间字段从 32 bit 压缩到 T_W bit。
//
// 时序常量（tick 域，原始值 ÷ 11264）：
//   kTs1: A=8, B=4, C=2    kTd1: A=4, B=4, C=2
//   kTs3: A=4, B=2, C=1    kTd3: A=2, B=2, C=1
//   M_dim: A=8, B=4, C=2
//
// best_s4(r) = ((r+1)/2) ticks      → 硬件实现：(r+1)>>1（加法+右移，无乘法）
// best_s2(r) = 2 × best_s4(r) ticks → 硬件实现：best_s4(r) << 1
//
// 带宽输出改为 2-bit 档位（BW_W=2）：
//   0=无活动(0 B/cc), 1=单路 64 B/cc, 2=双路 128 B/cc

import sched_pkg::*;

module sched_mk_snap (
  // ── 输入 ─────────────────────────────────────────────────────────────────
  input  logic [T_W-1:0]   start_t_i,   // cluster 当前可用时刻（ticks）
  input  logic [NTOK_W-1:0] ntok_i,     // 本 expert 的 token 数（≤511）
  input  logic [1:0]        shape_s1_i, // Shape A/B/C
  input  logic [1:0]        shape_s3_i,
  input  logic              skip_s1_i,  // 1=S1 GEMM+DMA 已命中缓存
  input  logic              skip_s3_i,  // 1=S3 GEMM+DMA 已命中缓存

  // ── 时间输出（ticks，T_W follows sched_pkg::E_MAX）─────────────────────
  output logic [T_W-1:0]   task_start_o,
  output logic [T_W-1:0]   task_end_o,
  output logic [T_W-1:0]   dma1_end_o,
  output logic [T_W-1:0]   s1_end_o,
  output logic [T_W-1:0]   s2_end_o,
  output logic [T_W-1:0]   dma3_end_o,
  output logic [T_W-1:0]   s3_end_o,
  output logic [T_W-1:0]   s4_start_o,

  // ── 带宽档位输出（2-bit：0/1/2 → 0/64/128 B/cc）────────────────────────
  output logic [BW_W-1:0]  bw_s1_o,
  output logic [BW_W-1:0]  bw_s3_o,

  // ── 执行参数输出 ─────────────────────────────────────────────────────────
  output logic [NTOK_W-1:0] m_s2_exec_o,
  output logic [NTOK_W-1:0] m_s4_exec_o,
  output logic              skip_s2_o,
  output logic              skip_s4_o,
  output logic [1:0]        dma_s1_o,   // 0=NONE, 1=IDMA, 2=XDMA, 3=BOTH
  output logic [1:0]        dma_s3_o
);

  localparam logic [1:0] DMA_NONE = 2'd0;
  localparam logic [1:0] DMA_IDMA = 2'd1;
  localparam logic [1:0] DMA_XDMA = 2'd2;
  localparam logic [1:0] DMA_BOTH = 2'd3;

  // ── 中间信号 ──────────────────────────────────────────────────────────────
  logic [NTOK_W-1:0] s1_tail;
  logic [NTOK_W-1:0] s3_tail;
  logic [T_W-1:0]    bs2_ntok, bs4_ntok;  // full ntok 的 best_s2/best_s4
  logic [T_W-1:0]    bs2_s1t,  bs4_s3t;   // tail ntok 的 best_s2/best_s4

  // 统一复用 sched_pkg 的时序原语函数，避免与包内定义漂移。
  assign bs2_ntok = best_s2_t(ntok_i);
  assign bs4_ntok = best_s4_t(ntok_i);
  assign bs2_s1t  = best_s2_t(s1_tail);
  assign bs4_s3t  = best_s4_t(s3_tail);

  // ── tail 计算 ─────────────────────────────────────────────────────────────
  always_comb begin
    s1_tail = (ntok_i > mdim(shape_s1_i)) ? (ntok_i - mdim(shape_s1_i)) : '0;
    s3_tail = (ntok_i > mdim(shape_s3_i)) ? (ntok_i - mdim(shape_s3_i)) : '0;
  end

  // ── 时序计算（组合逻辑）─────────────────────────────────────────────────
  always_comb begin
    task_start_o = start_t_i;

    // ── S1 / S2 阶段 ───────────────────────────────────────────────────────
    if (skip_s1_i) begin
      dma1_end_o  = start_t_i;
      s1_end_o    = start_t_i;
      bw_s1_o     = BW_0;
      m_s2_exec_o = ntok_i;
      skip_s2_o   = 1'b0;
      s2_end_o    = start_t_i + bs2_ntok;
      dma_s1_o    = DMA_NONE;
    end else begin
      dma1_end_o  = start_t_i + kTd1(shape_s1_i);
      s1_end_o    = start_t_i + kTs1(shape_s1_i);
      bw_s1_o     = alloc_bw(shape_s1_i);
      m_s2_exec_o = s1_tail;
      skip_s2_o   = (s1_tail == '0);
      s2_end_o    = s1_end_o + bs2_s1t;
      dma_s1_o    = (alloc_bw(shape_s1_i) == BW_128) ? DMA_BOTH : DMA_IDMA;
    end

    // ── S3 / S4 阶段 ───────────────────────────────────────────────────────
    if (skip_s3_i) begin
      dma3_end_o  = s2_end_o;
      s3_end_o    = s2_end_o;
      s4_start_o  = s2_end_o;
      bw_s3_o     = BW_0;
      m_s4_exec_o = ntok_i;
      skip_s4_o   = 1'b0;
      task_end_o  = s2_end_o + bs4_ntok;
      dma_s3_o    = DMA_NONE;
    end else begin
      dma3_end_o  = s2_end_o + kTd3(shape_s3_i);
      s3_end_o    = s2_end_o + kTs3(shape_s3_i);
      s4_start_o  = s3_end_o;
      bw_s3_o     = alloc_bw(shape_s3_i);
      m_s4_exec_o = s3_tail;
      skip_s4_o   = (s3_tail == '0);
      task_end_o  = s3_end_o + bs4_s3t;
      dma_s3_o    = (alloc_bw(shape_s3_i) == BW_128) ? DMA_BOTH : DMA_XDMA;
    end
  end

endmodule
