// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — Parameter Package (sched_pkg)
//
// ─────────────────────────────────────────────────────────────────────────────
// 时间域选择：Tick 域（÷11264 cc）
//
// 算法中所有 stage 时序常数均是 11264 cc 的整数倍。除以 11264 后，
// 当前 RTL 直接在各使用点以小整数 tick 实现 stage duration / DMA duration，
// 不在 package 中保留通用 shape timing helper。
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
  localparam int unsigned CAND_ID_W = 4;  // 单轮候选 ID（当前最大 id=12）
  localparam int unsigned SLOT_W    = 6;  // dynamic args slot_id ABI: ctrl[19:14], 0..63
  // Dense task FIFO: each entry stores one independently consumable 64-bit
  // task word.  Eight entries provide 512 bits of payload capacity.
  localparam int unsigned TASKQ_DEPTH = 8;

  // ── 唯一容量配置点：只改 E_MAX，EID_W/NR_W/T_W 自动推导 ─────────────────
  localparam int unsigned EID_RAW_W = $clog2(E_MAX);     // 原始 expert ID 位宽（0..E_MAX-1）
  localparam int unsigned EID_W     = EID_RAW_W + 1;     // 0=NONE,1=GHOST,1xxxx=experts
  localparam int unsigned NR_W      = $clog2(E_MAX + 1); // rem 计数域 0..E_MAX

  typedef logic [SLOT_W-1:0] slot_id_t;
  typedef logic [T_W-1:0]    time_t;
  typedef logic [BW_W-1:0]   bw_t;
  typedef logic [NTOK_W-1:0] ntok_t;
  typedef logic [1:0]        shape_t;
  typedef logic [NTOK_W:0]   best_ticks_t;
  typedef logic [EID_W-1:0]  pf_eid_t;

  // ── EID 特殊编码 ─────────────────────────────────────────────────────────
  // pf_eid 含义：
  //   0_000000 = NONE
  //   0_000001 = GHOST
  //   1_xxxxxx = raw expert id（MSB=1，低 EID_RAW_W bit 为 CVA6 传入的 eid）
  //
  // MSB tag 使编码不需要加法器；hit 判断只检查 tag 和低位 eid。
  localparam pf_eid_t PF_EID_NONE  = '0;
  localparam pf_eid_t PF_EID_GHOST = pf_eid_t'(1);

  // ── 带宽档位编码（BW_W = 2 bit）────────────────────────────────────────
  localparam logic [BW_W-1:0] BW_0   = 2'd0;   // 0   B/cc（DMA 跳过或无活动）
  localparam logic [BW_W-1:0] BW_64  = 2'd1;   // 64  B/cc（IDMA 或 XDMA 单路）
  localparam logic [BW_W-1:0] BW_128 = 2'd2;   // 128 B/cc（IDMA+XDMA 合用）

  // ── Shape 编码 ───────────────────────────────────────────────────────────
  localparam logic [1:0] SHAPE_A = 2'd0;  // M_dim=8, 64 B/cc DMA
  localparam logic [1:0] SHAPE_B = 2'd1;  // M_dim=4, 64 B/cc DMA
  localparam logic [1:0] SHAPE_C = 2'd2;  // M_dim=2, 128 B/cc DMA

  // 当前综合路径只需要根据 shape 判断 DMA BW；shape 的时序/mdim 解码
  // 已下沉到使用点，避免 package helper 在多个模块中被重复展开。
  function automatic bw_t shape_bw(input shape_t sh);
    shape_bw = (sh == SHAPE_C) ? BW_128 : BW_64;
  endfunction

  // ── Ghost 注入阈值 ───────────────────────────────────────────────────────
  // ghost_ok 条件：task_end - dma3_end >= shape_td1[A] = 4 ticks
  localparam time_t GHOST_WINDOW_TICKS = time_t'(4);

  // ── ceil-div helpers / best_*（tick 域，纯移位+小修正）───────────────
  // C 模型是 best_s4(r)=((r+1)/2)*11264，即 tick 域 ceil(r/2)。
  // 注意 r=0 时结果必须是 0，不能写成 floor(r/2)+1。
  //
  // best_s2(r) = best_s4(r) << 1。
  //
  // 两者均可由 9-bit ntok 计算，结果最多 10 bit（对 ntok≤511）。
  // 在当前 T_W 时间域内不会溢出：best_s4(511) = 256 ticks。

  function automatic ntok_t ceil_div2_ntok(input ntok_t x);
    ntok_t hi;
    begin
      hi = {1'b0, x[NTOK_W-1:1]};
      ceil_div2_ntok = hi + NTOK_W'(x[0]);
    end
  endfunction

  function automatic ntok_t ceil_div4_ntok(input ntok_t x);
    ntok_t hi;
    begin
      hi = {{2{1'b0}}, x[NTOK_W-1:2]};
      ceil_div4_ntok = hi + NTOK_W'(|x[1:0]);
    end
  endfunction

  function automatic best_ticks_t best_s4_ticks(input ntok_t r);
    best_s4_ticks = best_ticks_t'(ceil_div2_ntok(r));
  endfunction

  function automatic best_ticks_t best_s2_ticks(input ntok_t r);
    ntok_t h;
    best_ticks_t h2;
    begin
      h = ceil_div2_ntok(r);
      h2 = {h, 1'b0};
      best_s2_ticks = h2;
    end
  endfunction

  // ── best_task / best_conc（用于 greedy_h，tick 域）──────────────────
  // best_task(n) = ((n+1)/2) × 3 ticks = best_s4_ticks(n) × 3
  //   = 3 × ceil(n/2)，实现中只计算一次 ceil(n/2)
  // best_conc(n) = ((n+3)/4) × 6 ticks = ceil(n/4) × 6

  function automatic best_ticks_t best_task_ticks(input ntok_t n);
    ntok_t h;
    logic [NTOK_W+1:0] h3;
    begin
      h  = ceil_div2_ntok(n);
      h3 = {1'b0, h, 1'b0} + {{2{1'b0}}, h};
      best_task_ticks = best_ticks_t'(h3);
    end
  endfunction

  function automatic best_ticks_t best_conc_ticks(input ntok_t n);
    ntok_t q;
    logic [NTOK_W+1:0] q6;
    begin
      q  = ceil_div4_ntok(n);
      q6 = {q, 2'b00} + {1'b0, q, 1'b0};
      best_conc_ticks = best_ticks_t'(q6);
    end
  endfunction

  // ── 候选分数键（比较顺序：cost > rem_len > snap_max > candidate_id）──────
  // 稳定性由 candidate_id 决定，score path 只保留会参与当前 policy 的字段。
  typedef struct packed {
    logic [T_W-1:0]  cost;      // continuation_cost 返回值（ticks）
    logic [NR_W-1:0] rem_len;   // 本轮消化后剩余 expert 数
    logic [T_W-1:0]  snap_max;  // max(sa.task_end, sb.task_end)
  } score_key_t;

  // RTL 内部 top4 工作项。MMIO 使用 compact head16 ABI，只把调度 datapath
  // 真正需要的字段落到 FF，避免保存 rem_index/input_order/best_conc。
  //
  // 位宽（E_MAX=64）：valid + eid + ntok = 1 + 6 + 9 = 16 bits。
  // best_conc 由 ntok 通过 best_conc_ticks() 组合推导，不驻留在 FF 或软件 rem 表中。
  typedef struct packed {
    logic                  valid;
    logic [EID_RAW_W-1:0]  eid;
    logic [NTOK_W-1:0]     ntok;
  } head_ctx_t;

  // ── Snap view types for staged datapaths ────────────────────────────────
  //
  // S2PF/BW placement only consumes timeline and BW fields.  Cache-prefetch
  // identity fields are kept in snap_cache_t and only forwarded to score/cache
  // hit logic.  There is intentionally no package-wide full snap record.
  typedef struct packed {
    logic                 valid;
    time_t                task_start;
    time_t                task_end;
    time_t                dma1_end;
    time_t                s2_end;
    time_t                dma3_end;
    time_t                s4_start;
    bw_t                  bw_s1;
    bw_t                  bw_s3;
    logic                 s2pf_valid;
    time_t                s2pf_start;
    time_t                s2pf_end;
    bw_t                  s2pf_bw;
    logic                 s4pf_valid;
    time_t                s4pf_start;
    ntok_t                ntok;
  } snap_timeline_t;

  typedef struct packed {
    logic                 valid;
    time_t                task_start;
    time_t                dma1_end;
    time_t                s2_end;
    time_t                dma3_end;
    bw_t                  bw_s1;
    bw_t                  bw_s3;
    logic                 s2pf_valid;
    time_t                s2pf_start;
    time_t                s2pf_end;
    bw_t                  s2pf_bw;
    logic                 s4pf_valid;
    time_t                s4pf_start;
  } snap_bw_view_t;

  // S2PF search output is a patch, not a full timeline.  The caller owns the
  // original snap_timeline_t and locally applies these few changed fields.
  typedef struct packed {
    logic                 ok;
    logic                 has_a;
    time_t                pf_start_a;
    logic                 has_b;
    time_t                pf_start_b;
  } s2pf_patch_t;

  function automatic snap_timeline_t apply_s2pf_patch_timeline(
    input snap_timeline_t sn,
    input logic           has_pf,
    input time_t          pf_start,
    input shape_t         shape_s3
  );
    snap_timeline_t ret;
    time_t          pf_dur;
    begin
      ret = sn;
      pf_dur = (shape_s3 == SHAPE_C) ? time_t'(1) : time_t'(2);
      if (has_pf) begin
        ret.s2pf_valid = 1'b1;
        ret.s2pf_start = pf_start;
        ret.s2pf_end   = pf_start + pf_dur;
        ret.s2pf_bw    = shape_bw(shape_s3);
        ret.dma3_end   = sn.s2_end;
        ret.s4_start   = sn.s2_end;
        ret.bw_s3      = BW_0;
        ret.task_end   = sn.s2_end + time_t'(best_s4_ticks(sn.ntok));
      end
      apply_s2pf_patch_timeline = ret;
    end
  endfunction

  function automatic snap_bw_view_t to_bw_view(input snap_timeline_t sn);
    snap_bw_view_t v;
    begin
      v.valid       = sn.valid;
      v.task_start  = sn.task_start;
      v.dma1_end    = sn.dma1_end;
      v.s2_end      = sn.s2_end;
      v.dma3_end    = sn.dma3_end;
      v.bw_s1       = sn.bw_s1;
      v.bw_s3       = sn.bw_s3;
      v.s2pf_valid  = sn.s2pf_valid;
      v.s2pf_start  = sn.s2pf_start;
      v.s2pf_end    = sn.s2pf_end;
      v.s2pf_bw     = sn.s2pf_bw;
      v.s4pf_valid  = sn.s4pf_valid;
      v.s4pf_start  = sn.s4pf_start;
      to_bw_view = v;
    end
  endfunction

  typedef struct packed {
    pf_eid_t              pf_eid;
    time_t                pf_end;
    logic                 pf_full;
  } snap_cache_t;

  typedef struct packed {
    logic [1:0] count;  // 0..2 extra release starts after idle_t
    time_t      t0;
    time_t      t1;
  } early_start_ctx_t;

  function automatic pf_eid_t encode_eid(input logic [EID_RAW_W-1:0] eid);
    encode_eid = {1'b1, eid};
  endfunction

  function automatic logic swiglu_hit_t(
    input logic [EID_RAW_W-1:0] eid,
    input pf_eid_t              pf_eid,
    input time_t                pf_end,
    input time_t                t
  );
    swiglu_hit_t = (pf_eid != PF_EID_NONE) &&
                   (pf_end <= t) &&
                   ((pf_eid == PF_EID_GHOST) ||
                    (pf_eid[EID_W-1] && (pf_eid[EID_RAW_W-1:0] == eid)));
  endfunction

  function automatic logic down_hit_t(
    input logic [EID_RAW_W-1:0] eid,
    input pf_eid_t              pf_eid,
    input time_t                pf_end,
    input logic              pf_full,
    input time_t                t
  );
    down_hit_t = swiglu_hit_t(eid, pf_eid, pf_end, t) && pf_full;
  endfunction

  // ── Normalized winning plan ────────────────────────────────────────────
  //
  // eval/replay 后直接输出 normalized physical cluster task。每个输出项是
  // 单个 task 的 token/control 对，core-local commit 子状态机生成 task_desc_t。
  //
  // 位宽（E_MAX=64）：
  //   winner_token_t  = eid6 + ntok9 + tok_start9 = 24 bits
  //   task_control_t  = cluster1 + s1/s3 4 + skip2 + has_s2pf1 = 8 bits
  //   winner_plan_t   = valid2 + 2*(24+8) = 66 bits
  typedef struct packed {
    logic [EID_RAW_W-1:0] eid;
    ntok_t                 ntok;
    ntok_t                 tok_start;
  } winner_token_t;

  typedef struct packed {
    logic                  cluster;     // 0=C2, 1=C3
    shape_t                s1;
    shape_t                s3;
    logic                  skip_s1;
    logic                  skip_s3;
    logic                  has_s2pf;
  } task_control_t;

  typedef struct packed {
    logic [1:0]            valid;
    winner_token_t [1:0]   token;
    task_control_t [1:0]   ctrl;
  } winner_plan_t;

  // ── Commit 后的单 task descriptor ──────────────────────────────────────
  //
  // winner_plan_t 用于 evaluator/replay 到 commit 阶段。commit 后进入
  // wrapper FIFO 的每个 entry 已经规范化成单个 cluster task。
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

  // ── MMIO task word layout ───────────────────────────────────────────────
  //
  // TASK_FIFO_DATA[i] 是 64-bit fast-lowering payload。每条 task 自带其 S4PF
  // target；consumer 不需要再用后一条 task 反向修改前一个 L3 slot。
  localparam int unsigned TASK_WORD_EID_LSB         = 0;
  localparam int unsigned TASK_WORD_TOKEN_START_LSB = TASK_WORD_EID_LSB + EID_RAW_W;
  localparam int unsigned TASK_WORD_NTOK_LSB        = TASK_WORD_TOKEN_START_LSB + NTOK_W;
  localparam int unsigned TASK_WORD_HAS_S2PF_LSB    = TASK_WORD_NTOK_LSB + NTOK_W;
  localparam int unsigned TASK_WORD_CTRL_LSB        = TASK_WORD_HAS_S2PF_LSB + 1;
  localparam int unsigned TASK_WORD_CTRL_W          = 13;
  localparam int unsigned TASK_WORD_M_S2_LSB        = TASK_WORD_CTRL_LSB + TASK_WORD_CTRL_W;
  localparam int unsigned TASK_WORD_M_S4_LSB        = TASK_WORD_M_S2_LSB + NTOK_W;
  localparam int unsigned TASK_WORD_S4PF_DESC_LSB   = TASK_WORD_M_S4_LSB + NTOK_W;

  // High-byte S4PF descriptor: valid/no-copy/target-eid.  target_eid belongs
  // to this task's future S4 prefetch, not to the current task itself.
  localparam int unsigned S4PF_DESC_VALID_LSB      = 0;
  localparam int unsigned S4PF_DESC_NO_COPY_LSB    = 1;
  localparam int unsigned S4PF_DESC_TARGET_EID_LSB = 2;

  // ── Lite S2PF policy ────────────────────────────────────────────────────
  //
  // S2PF 搜索不再保留完整 try_s2pf_pair() 的 25-way 通用枚举。候选生成时
  // 直接指定当前候选所属的硬件模板集合，降低 start mux、BW request fanout
  // 和无效中间结果。
  typedef enum logic [1:0] {
    S2PF_OFF           = 2'd0, // 不尝试 S2PF，只检查原始 BW
    S2PF_PAIR_LITE     = 2'd1, // PAIR: both@dma1_end, raw
    S2PF_SPLIT_LITE    = 2'd2, // SPLIT: both@dma1_end, b_only@dma1_end, raw
    S2PF_SINGLE_DMA1   = 2'd3  // SINGLE/not_both: active side@dma1_end, raw
  } s2pf_policy_t;

  // ── 1-lane 架构存储原则 ────────────────────────────────────────────────
  // E_MAX=64 时，完整 rem_eid/ntok/order 和完整 plan list 放在 L3/CVA6
  // 软件内存；scheduler 寄存器侧只保留本轮 top4、两个 cluster snap、
  // compact best_token/best_score、depth=8 dense task FIFO 和 FSM。
  // 时序字段仍使用 tick 域，不回退到 32-bit raw-CC。

endpackage
