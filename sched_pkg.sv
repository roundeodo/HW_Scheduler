// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — Parameter Package (sched_pkg)
//
// ─────────────────────────────────────────────────────────────────────────────
// 时间域选择：Tick 域（÷11264 cc）
//
// 算法中所有时序常数均是 11264 cc 的整数倍：
//   kTs1[A]=90112, kTs1[B]=45056, kTs1[C]=22528, kTd1[A]=45056,
//   kTd3[C]=11264, best_s2(r)=((r+1)/2)×22528, ...
// 除以 11264 后全部成为小整数 {1,2,4,8}。
//
// task_end 累积上界（E≤16 expert，M_total≤256 token）：
//   单任务最大 ≈ 399 ticks（约 4.5 M cc）
//   单 cluster 串行 16 expert ≤ 16×399 = 6384 ticks
//   → ceil(log2(6384)) = 13 bits 足够
//
// E_MAX=64 时，task_end 和 total_conc 上界都会超过 13 bit。因此 T_W
// 跟随 E_MAX 自动放大；E_MAX=64 时使用 16 bit，仍远小于 32-bit raw-CC。
// ─────────────────────────────────────────────────────────────────────────────

package sched_pkg;

  // ── 基本位宽参数 ─────────────────────────────────────────────────────────
  localparam int unsigned E_MAX     = 64; // 最大 expert 数；小规模实验可改回 16
  localparam int unsigned T_W       = (E_MAX <= 16) ? 13 :
                                      (E_MAX <= 32) ? 15 :
                                      (E_MAX <= 64) ? 16 : 17;
  localparam int unsigned BW_W      = 2;  // 带宽档位宽度（0/1/2 → 0/64/128 B/cc）
  localparam int unsigned NTOK_W    = 9;  // token 数宽度（≤511）
  localparam int unsigned CAND_ID_W = 6;  // 单轮候选 ID（覆盖 ≤64 个候选）
  localparam int unsigned SLOT_W    = 6;  // dynamic args slot_id ABI: ctrl[19:14], 0..63
  localparam int unsigned PLANQ_DEPTH = 4; // wrapper-visible completed-round FIFO depth

  // ── 唯一容量配置点：只改 E_MAX，EID_W/NR_W/T_W 自动推导 ─────────────────
  localparam int unsigned EID_W     = $clog2(E_MAX + 2); // 0=NONE,1=GHOST,2..E_MAX+1=experts
  localparam int unsigned EID_RAW_W = $clog2(E_MAX);     // 原始 expert ID 位宽（0..E_MAX-1）
  localparam int unsigned NR_W      = $clog2(E_MAX + 1); // rem 计数域 0..E_MAX

  typedef logic [SLOT_W-1:0] slot_id_t;

  // ── EID 特殊编码 ─────────────────────────────────────────────────────────
  // pf_eid 含义：
  //   PF_EID_NONE  = 0 → 无预取/无驻留
  //   PF_EID_GHOST = 1 → Ghost 预取（对任意 EID 有效，pf_full=0）
  //   2 .. E_MAX+1     → Expert ID 0..E_MAX-1
  localparam logic [EID_W-1:0] PF_EID_NONE  = '0;
  localparam logic [EID_W-1:0] PF_EID_GHOST = EID_W'(1);
  localparam logic [EID_W-1:0] PF_EID_BASE  = EID_W'(2);  // eid_hw = eid_sw + PF_EID_BASE

  // ── 带宽档位编码（BW_W = 2 bit）────────────────────────────────────────
  localparam logic [BW_W-1:0] BW_0   = 2'd0;   // 0   B/cc（DMA 跳过或无活动）
  localparam logic [BW_W-1:0] BW_64  = 2'd1;   // 64  B/cc（IDMA 或 XDMA 单路）
  localparam logic [BW_W-1:0] BW_128 = 2'd2;   // 128 B/cc（IDMA+XDMA 合用）

  // ── Dynamic args ctrl 中的 DMA binding 编码，与 moe_scheduler.h 保持一致 ──
  localparam logic [1:0] DMA_NONE = 2'd0;
  localparam logic [1:0] DMA_IDMA = 2'd1;
  localparam logic [1:0] DMA_XDMA = 2'd2;
  localparam logic [1:0] DMA_BOTH = 2'd3;

  // ── Shape 编码 ───────────────────────────────────────────────────────────
  localparam logic [1:0] SHAPE_A = 2'd0;  // M_dim=8, 64 B/cc DMA
  localparam logic [1:0] SHAPE_B = 2'd1;  // M_dim=4, 64 B/cc DMA
  localparam logic [1:0] SHAPE_C = 2'd2;  // M_dim=2, 128 B/cc DMA

  // ── Shape 时序常量（tick 域，1 tick = 11264 cc）─────────────────────────
  //   kTs1[A]=8, kTs1[B]=4, kTs1[C]=2  （S1 GEMM 完成时长）
  //   kTd1[A]=4, kTd1[B]=4, kTd1[C]=2  （S1 DMA 完成时长）
  //   kTs3[A]=4, kTs3[B]=2, kTs3[C]=1  （S3 GEMM 完成时长）
  //   kTd3[A]=2, kTd3[B]=2, kTd3[C]=1  （S3 DMA 完成时长）
  //   M_dim[A]=8, M_dim[B]=4, M_dim[C]=2

  function automatic logic [T_W-1:0] kTs1(input logic [1:0] sh);
    unique case (sh)
      SHAPE_A: kTs1 = T_W'(8);
      SHAPE_B: kTs1 = T_W'(4);
      default: kTs1 = T_W'(2);
    endcase
  endfunction

  function automatic logic [T_W-1:0] kTd1(input logic [1:0] sh);
    unique case (sh)
      SHAPE_A: kTd1 = T_W'(4);
      SHAPE_B: kTd1 = T_W'(4);
      default: kTd1 = T_W'(2);
    endcase
  endfunction

  function automatic logic [T_W-1:0] kTs3(input logic [1:0] sh);
    unique case (sh)
      SHAPE_A: kTs3 = T_W'(4);
      SHAPE_B: kTs3 = T_W'(2);
      default: kTs3 = T_W'(1);
    endcase
  endfunction

  function automatic logic [T_W-1:0] kTd3(input logic [1:0] sh);
    unique case (sh)
      SHAPE_A: kTd3 = T_W'(2);
      SHAPE_B: kTd3 = T_W'(2);
      default: kTd3 = T_W'(1);
    endcase
  endfunction

  function automatic logic [NTOK_W-1:0] mdim(input logic [1:0] sh);
    unique case (sh)
      SHAPE_A: mdim = 9'd8;
      SHAPE_B: mdim = 9'd4;
      default: mdim = 9'd2;
    endcase
  endfunction

  function automatic logic [BW_W-1:0] alloc_bw(input logic [1:0] sh);
    alloc_bw = (sh == SHAPE_C) ? BW_128 : BW_64;
  endfunction

  // ── Ghost 注入阈值 ───────────────────────────────────────────────────────
  // ghost_ok 条件：task_end - s4_start ≥ kTd1[A] = 4 ticks
  localparam logic [T_W-1:0] GHOST_WINDOW_TICKS = T_W'(4);

  // ── best_s4 / best_s2（tick 域，纯加法，无乘法）──────────────────────
  // C 模型是 best_s4(r)=((r+1)/2)*11264，即 tick 域 (r+1)>>1。
  // 注意 r=0 时结果必须是 0，不能写成 floor(r/2)+1。
  //
  // best_s2(r) = best_s4(r) << 1。
  //
  // 两者均可由 9-bit ntok 计算，结果最多 10 bit（对 ntok≤511）。
  // 在当前 T_W 时间域内不会溢出：best_s4(511) = 256 ticks。

  function automatic logic [T_W-1:0] best_s4_t(input logic [NTOK_W-1:0] r);
    logic [NTOK_W:0] tmp;
    tmp = {1'b0, r} + {{NTOK_W{1'b0}}, 1'b1};
    best_s4_t = T_W'(tmp >> 1);
  endfunction

  function automatic logic [T_W-1:0] best_s2_t(input logic [NTOK_W-1:0] r);
    logic [T_W-1:0] s4;
    s4 = best_s4_t(r);
    best_s2_t = {s4[T_W-2:0], 1'b0};  // best_s4_t(r) × 2
  endfunction

  // ── best_task / best_conc（用于 greedy_h，tick 域）──────────────────
  // best_task(n) = ((n+1)/2) × 3 ticks = best_s4_t(n) × 3
  //   = best_s2_t(n) + best_s4_t(n)
  // best_conc(n) = ((n+3)/4) × 6 ticks = ceil(n/4) × 6

  function automatic logic [T_W-1:0] best_task_t(input logic [NTOK_W-1:0] n);
    best_task_t = best_s2_t(n) + best_s4_t(n);  // 3 × ((n+1)/2)
  endfunction

  function automatic logic [T_W-1:0] best_conc_t(input logic [NTOK_W-1:0] n);
    logic [NTOK_W+2:0] tmp;
    tmp = {3'b0, n} + {{NTOK_W{1'b0}}, 3'd3};
    // multiply by 6: ×4 + ×2
    best_conc_t = T_W'((tmp >> 2) << 2) + T_W'((tmp >> 2) << 1);
  endfunction

  // ── 候选分数键（比较顺序：cost > rem_len > snap_max > snap_min）────────
  // 总宽度：3*T_W + NR_W；E_MAX=64 时为 55 bit。
  typedef struct packed {
    logic [T_W-1:0]  cost;      // continuation_cost 返回值（ticks）
    logic [NR_W-1:0] rem_len;   // 本轮消化后剩余 expert 数
    logic [T_W-1:0]  snap_max;  // max(sa.task_end, sb.task_end)
    logic [T_W-1:0]  snap_min;  // min(sa.task_end, sb.task_end)
  } score_key_t;                // 55 bits when E_MAX=64

  // ── stable top4 selector 输出项 ─────────────────────────────────────────
  //
  // rem_index 指向 CVA6/L3 sorted rem stream 中的物理项；input_order 保留
  // C insertion sort 的稳定性：ntok 相等时，较小 input_order 胜出。
  typedef struct packed {
    logic                  valid;
    logic [NR_W-1:0]       rem_index;
    logic [EID_RAW_W-1:0]  eid;
    logic [NTOK_W-1:0]     ntok;
    logic [NR_W-1:0]       input_order;
  } rem_head_t;

  // RTL 内部 top4 工作项。MMIO 使用 compact head16 ABI，只把调度 datapath
  // 真正需要的字段落到 FF，避免保存 rem_index/input_order/best_conc。
  //
  // 位宽（E_MAX=64）：valid + eid + ntok = 1 + 6 + 9 = 16 bits。
  // best_conc 由 ntok 通过 best_conc_t() 组合推导，不驻留在 FF 或软件 rem 表中。
  typedef struct packed {
    logic                  valid;
    logic [EID_RAW_W-1:0]  eid;
    logic [NTOK_W-1:0]     ntok;
  } head_ctx_t;

  // ── Evaluator snap ─────────────────────────────────────────────────────
  //
  // 这是 RTL 内部唯一 snap 协议，只保存后续硬件真正读取的字段。
  // s1_end/s3_end 是 C 模型的中间时间点，不参与 BW、候选生成、score、
  // ghost 或 commit 决策，因此不进入结构体，也不作为跨阶段状态传播。
  typedef struct packed {
    logic                 valid;
    logic [T_W-1:0]       task_start;
    logic [T_W-1:0]       task_end;
    logic [T_W-1:0]       dma1_end;
    logic [T_W-1:0]       s2_end;
    logic [T_W-1:0]       dma3_end;
    logic [T_W-1:0]       s4_start;
    logic [BW_W-1:0]      bw_s1;
    logic [BW_W-1:0]      bw_s3;
    logic                 s2pf_valid;
    logic [T_W-1:0]       s2pf_start;
    logic [T_W-1:0]       s2pf_end;
    logic [BW_W-1:0]      s2pf_bw;
    logic                 s4pf_valid;
    logic [T_W-1:0]       s4pf_start;
    logic [NTOK_W-1:0]    ntok;
    logic [EID_W-1:0]     pf_eid;
    logic [T_W-1:0]       pf_end;
    logic                 pf_full;
  } eval_snap_t;

  // ── Candidate-generator view of a snap ─────────────────────────────────
  //
  // candidate_generator 不需要完整 eval_snap_t。它只关心：
  //   - task_end：判断 both_idle / idle cluster / candidate start time；
  //   - pf_*：为候选标注 swiglu/down cache hit；
  //   - release_valid/t：not-both-idle early-start 候选的少数 DMA release 点。
  //
  // 把这个 view 单独传入，可以避免完整 eval_snap_t 的 DMA/S4 等字段跨模块边界
  // 扇出到 candidate generation 逻辑。
  typedef struct packed {
    logic [T_W-1:0]    task_end;
    logic [EID_W-1:0]  pf_eid;
    logic [T_W-1:0]    pf_end;
    logic              pf_full;
    logic [3:0]        release_valid;
    logic [3:0][T_W-1:0] release_t;
  } cand_gen_snap_t;

  function automatic logic [EID_W-1:0] encode_eid(input logic [EID_RAW_W-1:0] eid);
    encode_eid = PF_EID_BASE + EID_W'(eid);
  endfunction

  function automatic logic swiglu_hit_t(
    input logic [EID_RAW_W-1:0] eid,
    input logic [EID_W-1:0]  pf_eid,
    input logic [T_W-1:0]    pf_end,
    input logic [T_W-1:0]    t
  );
    swiglu_hit_t = (pf_eid != PF_EID_NONE) &&
                   (pf_end <= t) &&
                   ((pf_eid == PF_EID_GHOST) || (pf_eid == encode_eid(eid)));
  endfunction

  function automatic logic down_hit_t(
    input logic [EID_RAW_W-1:0] eid,
    input logic [EID_W-1:0]  pf_eid,
    input logic [T_W-1:0]    pf_end,
    input logic              pf_full,
    input logic [T_W-1:0]    t
  );
    down_hit_t = swiglu_hit_t(eid, pf_eid, pf_end, t) && pf_full;
  endfunction

  // ── 获胜候选描述符（提交到 plan 和状态更新用）──────────────────────────
  //
  // 设计原则：只存"控制选择"，不存"时序输出"。
  //
  // ● 时序字段（est_start/est_end/est_s2_end/est_dma1_end/est_s4_start）
  //   均可在 commit 时以此结构体中的控制字段重跑 mk_snap 得到，无需存储。
  //   重跑开销：mk_snap 是纯组合逻辑，1 cycle re-evaluate 即可。
  //   节约：省去 10 × T_W bits 的寄存器。
  //
  // ● has_s2pf_a / has_s2pf_b：try_s2pf_pair() 的选择结果。
  //   S2PF 是把当前 task 自己的 S3/down DMA 提前到 S2 窗口，不是预取
  //   另一个 expert；PAIR/SPLIT 两侧可能同时命中，因此必须按侧存储。
  //   s2pf 的时序（dma_start / dma_end）由 evaluator 输出 snap 或 commit 重跑
  //   s2pf 选择得到，不需要保存额外 eid。
  //
  // 位宽汇总（E_MAX=64, EID_RAW_W=6）：
  //   基础控制字段：2+1+2×(6+9+9+2+2+1+1) = 63 bits
  //   s2pf 扩展：2 bits
  //   合计：65 bits
  typedef struct packed {
    logic [1:0]       plan_type;      // 2'b00=PAIR, 2'b01=SPLIT, 2'b10=SOLO
    logic             cluster_a;      // task A 所在 cluster（0=C2, 1=C3）
    // Task A
    logic [EID_RAW_W-1:0] eid_a;
    logic [NTOK_W-1:0]    ntok_a;
    logic [NTOK_W-1:0] tok_start_a;  // SPLIT 时 task A 的 token 起始偏移
    logic [1:0]       s1a;
    logic [1:0]       s3a;
    logic             skip_s1_a;
    logic             skip_s3_a;
    // Task B（SOLO 时无效，字段保留 0）
    logic [EID_RAW_W-1:0] eid_b;
    logic [NTOK_W-1:0]    ntok_b;
    logic [NTOK_W-1:0] tok_start_b;
    logic [1:0]       s1b;
    logic [1:0]       s3b;
    logic             skip_s1_b;
    logic             skip_s3_b;
    // S2 预取（try_s2pf_pair 选择结果）
    logic             has_s2pf_a;
    logic             has_s2pf_b;
  } plan_desc_t;                      // 65 bits when E_MAX=64

  // ── Commit 后的单 task descriptor ──────────────────────────────────────
  //
  // plan_desc_t 用于 evaluator/best/replay 阶段，能表达 PAIR/SPLIT/SOLO
  // candidate。commit 后进入 wrapper FIFO 的每个 entry 已经规范化成
  // 单个 cluster task，因此只需要保存 A 侧字段对应的信息。
  //
  // 位宽（E_MAX=64, EID_RAW_W=6）：
  //   cluster + eid + ntok + tok_start + s1 + s3 + skip_s1 + skip_s3 + has_s2pf
  // = 1 + 6 + 9 + 9 + 2 + 2 + 1 + 1 + 1 = 32 bits
  typedef struct packed {
    logic                 cluster;     // 0=C2, 1=C3
    logic [EID_RAW_W-1:0] eid;
    logic [NTOK_W-1:0]    ntok;
    logic [NTOK_W-1:0]    tok_start;
    logic [1:0]           s1;
    logic [1:0]           s3;
    logic                 skip_s1;
    logic                 skip_s3;
    logic                 has_s2pf;
  } task_desc_t;                       // 32 bits when E_MAX=64

  // ── Plan FIFO 的 lightweight lowering scalar ─────────────────────────────
  //
  // 这些字段都能由 task_desc_t 组合推导，不需要跨 round 保存 snap，也不需要
  // 输出完整 dynamic args。RTL 在 PLAN_ENTRY_DATA payload 中带出它们，
  // 让 CVA6 fast lowering 少做 shape/tail/dma binding 相关的小计算。
  typedef struct packed {
    logic                 skip_s2;
    logic                 skip_s4;
    logic [1:0]           dma_s1_for_shape;
    logic [1:0]           dma_s3_for_shape;
    logic [NTOK_W-1:0]    m_s2_exec;
    logic [NTOK_W-1:0]    m_s4_exec;
  } task_lower_scalar_t;                // 24 bits when NTOK_W=9

  // ── MMIO plan entry word layout ─────────────────────────────────────────
  //
  // PLAN_ENTRY_DATAx 是 64-bit fast-lowering payload。RTL 在 plan FIFO enqueue
  // 时完成打包，wrapper read path 只做 indexed mux，避免读路径重复推导 patch。
  localparam int unsigned PLAN_EID_LSB          = 0;
  localparam int unsigned PLAN_TOKEN_START_LSB  = PLAN_EID_LSB + EID_RAW_W;
  localparam int unsigned PLAN_NTOK_LSB         = PLAN_TOKEN_START_LSB + NTOK_W;
  localparam int unsigned PLAN_HAS_S2PF_LSB     = PLAN_NTOK_LSB + NTOK_W;
  localparam int unsigned PLAN_CTRL_LSB         = PLAN_HAS_S2PF_LSB + 1;
  localparam int unsigned PLAN_CTRL_W           = 13;
  localparam int unsigned PLAN_M_S2_LSB         = PLAN_CTRL_LSB + PLAN_CTRL_W;
  localparam int unsigned PLAN_M_S4_LSB         = PLAN_M_S2_LSB + NTOK_W;
  localparam int unsigned PLAN_INLINE_PATCH_LSB = PLAN_M_S4_LSB + NTOK_W;

  localparam int unsigned INLINE_PATCH_VALID_LSB      = 0;
  localparam int unsigned INLINE_PATCH_NO_COPY_LSB    = 1;
  localparam int unsigned INLINE_PATCH_LOCAL_SLOT_LSB = 2;

  function automatic logic [NTOK_W-1:0] shape_c_mtiles(input logic [NTOK_W-1:0] ntok);
    logic [NTOK_W:0] tmp;
    begin
      tmp = {1'b0, ntok} + {{NTOK_W{1'b0}}, 1'b1};
      shape_c_mtiles = tmp[NTOK_W:1];
    end
  endfunction

  function automatic logic [1:0] s1_dma_for_shape(input logic [1:0] sh);
    s1_dma_for_shape = (sh == SHAPE_C) ? DMA_BOTH : DMA_IDMA;
  endfunction

  function automatic logic [1:0] s3_dma_for_shape(input logic [1:0] sh);
    s3_dma_for_shape = (sh == SHAPE_C) ? DMA_BOTH : DMA_XDMA;
  endfunction

  function automatic task_lower_scalar_t lower_scalar_from_task(input task_desc_t t);
    task_lower_scalar_t r;
    logic [NTOK_W-1:0]  s1_m;
    logic [NTOK_W-1:0]  s3_m;
    logic [NTOK_W-1:0]  s2_tail;
    logic [NTOK_W-1:0]  s4_tail;
    begin
      r = '0;
      s1_m = mdim(t.s1);
      s3_m = mdim(t.s3);

      if (t.skip_s1) begin
        r.m_s2_exec = shape_c_mtiles(t.ntok);
        r.skip_s2   = 1'b0;
      end else begin
        s2_tail     = (t.ntok > s1_m) ? (t.ntok - s1_m) : '0;
        r.m_s2_exec = shape_c_mtiles(s2_tail);
        r.skip_s2   = (r.m_s2_exec == '0);
      end

      if (t.skip_s3) begin
        r.m_s4_exec = shape_c_mtiles(t.ntok);
        r.skip_s4   = 1'b0;
      end else begin
        s4_tail     = (t.ntok > s3_m) ? (t.ntok - s3_m) : '0;
        r.m_s4_exec = shape_c_mtiles(s4_tail);
        r.skip_s4   = (r.m_s4_exec == '0);
      end

      r.dma_s1_for_shape = s1_dma_for_shape(t.s1);
      r.dma_s3_for_shape = s3_dma_for_shape(t.s3);
      lower_scalar_from_task = r;
    end
  endfunction

  function automatic logic [PLAN_CTRL_W-1:0] pack_plan_ctrl_word(
    input task_desc_t t,
    input slot_id_t   local_slot
  );
    logic [PLAN_CTRL_W-1:0] ctrl;
    begin
      ctrl = '0;
      ctrl[0]    = t.skip_s1;
      ctrl[1]    = t.skip_s3;
      ctrl[3:2]  = t.s1;
      ctrl[5:4]  = t.s3;
      ctrl[6]    = t.cluster;
      ctrl[12:7] = local_slot;
      pack_plan_ctrl_word = ctrl;
    end
  endfunction

  function automatic logic [7:0] pack_inline_patch(
    input logic     valid,
    input logic     no_copy,
    input slot_id_t local_slot
  );
    logic [7:0] word;
    begin
      word = '0;
      word[INLINE_PATCH_VALID_LSB] = valid;
      word[INLINE_PATCH_NO_COPY_LSB] = no_copy;
      word[INLINE_PATCH_LOCAL_SLOT_LSB +: SLOT_W] = local_slot;
      pack_inline_patch = word;
    end
  endfunction

  function automatic logic [63:0] pack_plan_entry_word(
    input task_desc_t t,
    input slot_id_t   local_slot,
    input logic [7:0] inline_patch
  );
    logic [63:0] word;
    task_lower_scalar_t lower;
    begin
      word = '0;
      lower = lower_scalar_from_task(t);
      word[PLAN_EID_LSB +: EID_RAW_W]          = t.eid;
      word[PLAN_TOKEN_START_LSB +: NTOK_W]     = t.tok_start;
      word[PLAN_NTOK_LSB +: NTOK_W]            = t.ntok;
      word[PLAN_HAS_S2PF_LSB]                  = t.has_s2pf;
      word[PLAN_CTRL_LSB +: PLAN_CTRL_W]       = pack_plan_ctrl_word(t, local_slot);
      word[PLAN_M_S2_LSB +: NTOK_W]            = lower.m_s2_exec;
      word[PLAN_M_S4_LSB +: NTOK_W]            = lower.m_s4_exec;
      word[PLAN_INLINE_PATCH_LSB +: 8]         = inline_patch;
      pack_plan_entry_word = word;
    end
  endfunction

  // ── Lite S2PF policy ────────────────────────────────────────────────────
  //
  // S2PF 搜索不再保留完整 try_s2pf_pair() 的 25-way 通用枚举。候选生成时
  // 直接指定当前候选所属的硬件模板集合，降低 start mux、BW request fanout
  // 和无效中间结果。
  typedef enum logic [1:0] {
    S2PF_OFF           = 2'd0, // 不尝试 S2PF，只检查原始 BW
    S2PF_PAIR_LITE     = 2'd1, // PAIR: none + both@task_start/dma1_end/latest
    S2PF_SPLIT_LITE    = 2'd2, // SPLIT: none + both@task_start/dma1_end + b_only@latest
    S2PF_SINGLE_LATEST = 2'd3  // SINGLE/not_both: none + active side latest
  } s2pf_policy_t;

  // ── candidate_generator → candidate_eval_lane 的规范化候选 ─────────────
  //
  // side A/B 在 evaluator 内对应物理 C2/C3 snap。SOLO 放到 C3 时，
  // side_a_valid=0、side_b_valid=1，cluster_a=1，commit_unit 会从 B 侧
  // 生成单个 plan entry。
  typedef struct packed {
    logic                  valid;
    logic [CAND_ID_W-1:0]  candidate_id;
    logic [1:0]            plan_type;
    logic                  cluster_a;
    s2pf_policy_t          s2pf_policy;
    logic                  force_shape_a;
    logic                  force_shape_b;
    logic [1:0]            forced_s1a;
    logic [1:0]            forced_s3a;
    logic [1:0]            forced_s1b;
    logic [1:0]            forced_s3b;
    logic                  cost_only_tie;
    logic                  score_makespan_only;

    logic                  side_a_valid;
    logic                  side_b_valid;
    logic [T_W-1:0]        start_a;
    logic [T_W-1:0]        start_b;
    logic [EID_RAW_W-1:0]  eid_a;
    logic [EID_RAW_W-1:0]  eid_b;
    logic [NTOK_W-1:0]     ntok_a;
    logic [NTOK_W-1:0]     ntok_b;
    logic [NTOK_W-1:0]     tok_start_a;
    logic [NTOK_W-1:0]     tok_start_b;

    logic                  sw_a;
    logic                  dn_a;
    logic                  sw_b;
    logic                  dn_b;
    logic [T_W-1:0]        shape_t0;

    logic [NR_W-1:0]       rem_len_after;
    logic [EID_RAW_W-1:0]  rem0_eid;
    logic [NTOK_W-1:0]     rem0_ntok;
    logic [NTOK_W-1:0]     rem1_ntok;
    logic [T_W-1:0]        total_conc_after;
    logic [T_W-1:0]        max_conc_after;
    logic [3:0]            remove_slot_mask; // bit i=1 表示 commit 后移除当前 top4 slot i
  } cand_desc_t;

  // ── 1-lane 架构存储原则 ────────────────────────────────────────────────
  // E_MAX=64 时，完整 rem_eid/ntok/order 和完整 plan list 放在 L3/CVA6
  // 软件内存；scheduler 寄存器侧只保留本轮 top4、两个 cluster snap、
  // compact best_id/best_score、depth=4 round FIFO 和 FSM。
  // 时序字段仍使用 tick 域，不回退到 32-bit raw-CC。

endpackage
