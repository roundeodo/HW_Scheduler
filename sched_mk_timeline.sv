// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — timing-only mk_timeline primitive（Tick 域版本）
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
// 带宽输出改为 2-bit 档位（BW_W=2）：
//   0=无活动(0 B/cc), 1=单路 64 B/cc, 2=双路 128 B/cc

import sched_pkg::*;

module sched_mk_timeline (
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
  output logic [T_W-1:0]   s2_end_o,
  output logic [T_W-1:0]   dma3_end_o,
  output logic [T_W-1:0]   s4_start_o,

  // ── 带宽档位输出（2-bit：0/1/2 → 0/64/128 B/cc）────────────────────────
  output logic [BW_W-1:0]  bw_s1_o,
  output logic [BW_W-1:0]  bw_s3_o
);

  typedef struct packed {
    ntok_t half_mdim;
    time_t compute_dur;
    time_t dma_dur;
    bw_t   bw;
  } shape_attr_t;

  function automatic shape_attr_t s1_attr(input shape_t sh);
    shape_attr_t a;
    begin
      a = '0;
      unique case (sh)
        SHAPE_A: begin
          a.half_mdim   = ntok_t'(4);
          a.compute_dur = time_t'(8);
          a.dma_dur     = time_t'(4);
          a.bw          = BW_64;
        end
        SHAPE_B: begin
          a.half_mdim   = ntok_t'(2);
          a.compute_dur = time_t'(4);
          a.dma_dur     = time_t'(4);
          a.bw          = BW_64;
        end
        default: begin
          a.half_mdim   = ntok_t'(1);
          a.compute_dur = time_t'(2);
          a.dma_dur     = time_t'(2);
          a.bw          = BW_128;
        end
      endcase
      s1_attr = a;
    end
  endfunction

  function automatic shape_attr_t s3_attr(input shape_t sh);
    shape_attr_t a;
    begin
      a = '0;
      unique case (sh)
        SHAPE_A: begin
          a.half_mdim   = ntok_t'(4);
          a.compute_dur = time_t'(4);
          a.dma_dur     = time_t'(2);
          a.bw          = BW_64;
        end
        SHAPE_B: begin
          a.half_mdim   = ntok_t'(2);
          a.compute_dur = time_t'(2);
          a.dma_dur     = time_t'(2);
          a.bw          = BW_64;
        end
        default: begin
          a.half_mdim   = ntok_t'(1);
          a.compute_dur = time_t'(1);
          a.dma_dur     = time_t'(1);
          a.bw          = BW_128;
        end
      endcase
      s3_attr = a;
    end
  endfunction

  // ── 中间信号 ──────────────────────────────────────────────────────────────
  logic [NTOK_W-1:0] ceil_ntok;
  logic [NTOK_W-1:0] ceil_s1t;
  logic [NTOK_W-1:0] ceil_s3t;
  logic [T_W-1:0]    bs2_ntok, bs4_ntok;  // full ntok 的 best_s2/best_s4
  logic [T_W-1:0]    bs2_s1t,  bs4_s3t;   // tail ntok 的 best_s2/best_s4
  shape_attr_t        s1_attr_w;
  shape_attr_t        s3_attr_w;
  logic [T_W-1:0]    s1_compute_off;
  logic [T_W-1:0]    dma1_end_off;
  logic [T_W-1:0]    s2_dur;
  logic [T_W-1:0]    front_off;
  logic [T_W-1:0]    back_off;
  logic [T_W-1:0]    s3_dur;
  logic [T_W-1:0]    dma3_dur;
  logic [T_W-1:0]    s4_dur;
  function automatic time_t csa3_time(
    input time_t a,
    input time_t b,
    input time_t c
  );
    time_t        sum_bits;
    time_t        carry_bits;
    logic [T_W:0] final_sum;
    begin
      sum_bits   = a ^ b ^ c;
      carry_bits = (a & b) | (a & c) | (b & c);
      final_sum  = {1'b0, sum_bits} + {carry_bits, 1'b0};
      csa3_time  = final_sum[T_W-1:0];
    end
  endfunction

  // best_s4(x)=ceil(x/2)，best_s2(x)=ceil(x/2)<<1。
  // 这里显式共享 ceil_div2 结果，避免 best_s2_ticks/best_s4_ticks
  // 对同一个 token 数重复展开小加法器。
  assign ceil_ntok = ceil_div2_ntok(ntok_i);
  // M_dim is always even.  Therefore:
  //   ceil(max(ntok-M,0)/2) = max(ceil(ntok/2)-M/2,0)
  // Share one ceil unit and subtract small shape constants afterwards.
  assign ceil_s1t  = (ceil_ntok > s1_attr_w.half_mdim) ?
                     (ceil_ntok - s1_attr_w.half_mdim) : '0;
  assign ceil_s3t  = (ceil_ntok > s3_attr_w.half_mdim) ?
                     (ceil_ntok - s3_attr_w.half_mdim) : '0;
  assign bs4_ntok  = time_t'(ceil_ntok);
  assign bs2_ntok  = time_t'({ceil_ntok, 1'b0});
  assign bs2_s1t   = time_t'({ceil_s1t, 1'b0});
  assign bs4_s3t   = time_t'(ceil_s3t);

  // Shape 解码集中成 stage-local attribute。对同一个 shape 只做一次
  // case，直接产出 mdim/compute/DMA/BW，避免 shape_mdim/shape_ts*/shape_td*
  // 分散展开成多套等价 mux。
  assign s1_attr_w = s1_attr(shape_s1_i);
  assign s3_attr_w = s3_attr(shape_s3_i);

  // ── 时序计算（组合逻辑）─────────────────────────────────────────────────
  // 用 offset 形式直接生成后续真正使用的 timestamp，避免暴露并传播
  // S1/S3 中间结束时间。最终每个 timestamp 都是 start_t_i + offset。
  // 对 task_end 使用 front_off + back_off，其中 front_off=(S1+S2),
  // back_off=(S3+S4)，让四项和以两级加法收敛，避免可见中间 timestamp
  // 被串接成更深的 carry-propagate 链。
  always_comb begin
    task_start_o = start_t_i;

    // ── S1 / S2 阶段 ───────────────────────────────────────────────────────
    if (skip_s1_i) begin
      dma1_end_off = '0;
      s1_compute_off = '0;
      s2_dur       = bs2_ntok;
      bw_s1_o     = BW_0;
    end else begin
      dma1_end_off = s1_attr_w.dma_dur;
      s1_compute_off = s1_attr_w.compute_dur;
      s2_dur       = bs2_s1t;
      bw_s1_o     = s1_attr_w.bw;
    end

    // ── S3 / S4 阶段 ───────────────────────────────────────────────────────
    if (skip_s3_i) begin
      dma3_dur    = '0;
      s3_dur      = '0;
      s4_dur      = bs4_ntok;
      bw_s3_o     = BW_0;
    end else begin
      dma3_dur    = s3_attr_w.dma_dur;
      s3_dur      = s3_attr_w.compute_dur;
      s4_dur      = bs4_s3t;
      bw_s3_o     = s3_attr_w.bw;
    end

    front_off    = s1_compute_off + s2_dur;
    back_off     = s3_dur + s4_dur;
    dma1_end_o = start_t_i + dma1_end_off;
    s2_end_o   = start_t_i + front_off;
    // These three endpoints each consume exactly one final visible sum.
    // A 3:2 compressor removes one serial carry-propagate level.
    dma3_end_o = csa3_time(start_t_i, front_off, dma3_dur);
    s4_start_o = csa3_time(start_t_i, front_off, s3_dur);
    task_end_o = csa3_time(start_t_i, front_off, back_off);
  end

endmodule
