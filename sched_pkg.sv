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
// E_MAX=64 时，task_end 和 total_parallel_work 上界都会超过 13 bit。因此 T_W
// 跟随 E_MAX 自动放大；E_MAX=64 时使用 16 bit，仍远小于 32-bit raw-CC。
// ─────────────────────────────────────────────────────────────────────────────

package sched_pkg;

  // ── 基本位宽参数 ─────────────────────────────────────────────────────────
  localparam int unsigned E_MAX     = 64; // 最大 expert 数；小规模实验可改回 16
  localparam int unsigned T_W       = (E_MAX <= 16) ? 13 :
                                      (E_MAX <= 32) ? 15 :
                                      (E_MAX <= 64) ? 16 : 17;
  localparam int unsigned DMA_BIND_W = 2; // {xDMA, iDMA} 物理资源占用掩码
  localparam int unsigned NTOK_W    = 9;  // token 数宽度（≤511）
  localparam int unsigned CAND_ID_W = 4;  // 单轮候选 ID（当前最大 id=12）
  localparam int unsigned SLOT_W    = 6;  // dynamic args slot_id ABI: ctrl[19:14], 0..63
  // Dense task FIFO: each entry stores one independently consumable 64-bit
  // task word.  Eight entries provide 512 bits of payload capacity.
  localparam int unsigned TASKQ_DEPTH = 8;

  // ── 唯一容量配置点：只改 E_MAX，EID_W/NR_W/T_W 自动推导 ─────────────────
  localparam int unsigned EID_RAW_W = $clog2(E_MAX);     // 原始 expert ID 位宽（0..E_MAX-1）
  localparam int unsigned EID_W     = EID_RAW_W + 1;     // 0=NONE,1=S4PF wildcard,1xxxx=experts
  localparam int unsigned NR_W      = $clog2(E_MAX + 1); // rem 计数域 0..E_MAX

  typedef logic [SLOT_W-1:0] slot_id_t;
  typedef logic [T_W-1:0]    time_t;
  typedef logic [DMA_BIND_W-1:0] dma_binding_t;
  typedef logic [NTOK_W-1:0] ntok_t;
  typedef logic [1:0]        shape_t;
  typedef logic [NTOK_W:0]   best_ticks_t;
  typedef logic [EID_W-1:0]  pf_eid_t;

  // ── EID 特殊编码 ─────────────────────────────────────────────────────────
  // pf_eid 含义：
  //   0_000000 = NONE
  //   0_000001 = S4PF_WILDCARD（target eid 尚未绑定）
  //   1_xxxxxx = raw expert id（MSB=1，低 EID_RAW_W bit 为 CVA6 传入的 eid）
  //
  // MSB tag 使编码不需要加法器；hit 判断只检查 tag 和低位 eid。
  localparam pf_eid_t PF_EID_NONE          = '0;
  localparam pf_eid_t PF_EID_S4PF_WILDCARD = pf_eid_t'(1);

  // ── DMA 物理资源绑定 ───────────────────────────────────────────────────
  // 两个 bit 不是抽象带宽档位，而是同一全局资源池中的实际 lane mask：
  //   01: iDMA，10: xDMA，11: iDMA+xDMA。
  // 两个时间区间能否并行由 mask 是否相交决定；同名 DMA 即使来自不同
  // cluster 也不能同时使用。
  localparam dma_binding_t DMA_NONE = 2'b00;
  localparam dma_binding_t DMA_IDMA = 2'b01;
  localparam dma_binding_t DMA_XDMA = 2'b10;
  localparam dma_binding_t DMA_BOTH = 2'b11;

  // ── Shape 编码 ───────────────────────────────────────────────────────────
  localparam logic [1:0] SHAPE_A = 2'd0;  // M_dim=8, 64 B/cc DMA
  localparam logic [1:0] SHAPE_B = 2'd1;  // M_dim=4, 64 B/cc DMA
  localparam logic [1:0] SHAPE_C = 2'd2;  // M_dim=2, 128 B/cc DMA

  // 单路任务采用固定的 cluster-to-lane 绑定。这样两个 cluster 的 64 B/cc
  // 搬运天然落到不同物理 lane；Shape C 明确占用两条 lane。绑定是
  // {cluster, shape} 的确定函数，因此 task word 无需重复携带两个 DMA bit。
  function automatic dma_binding_t single_dma_for_cluster(input logic cluster);
    single_dma_for_cluster = cluster ? DMA_XDMA : DMA_IDMA;
  endfunction

  function automatic dma_binding_t shape_dma_binding(
    input shape_t shape,
    input logic   cluster
  );
    unique case (shape)
      SHAPE_A, SHAPE_B: shape_dma_binding = single_dma_for_cluster(cluster);
      SHAPE_C:          shape_dma_binding = DMA_BOTH;
      default:          shape_dma_binding = 'x;
    endcase
  endfunction

  // ── S4 prefetch window ───────────────────────────────────────────────────
  // S2PF/S4PF 都固定使用 iDMA+xDMA（128 B/cc）。S2PF 搬运
  // 1 tick，S4PF 从 dma3_end 开始搬运 2 ticks。
  localparam time_t S2PF_DMA_TICKS = time_t'(1);
  localparam time_t S4PF_DMA_TICKS = time_t'(2);

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

  function automatic best_ticks_t best_s4_ticks(input ntok_t tokens);
    best_s4_ticks = best_ticks_t'(ceil_div2_ntok(tokens));
  endfunction

  function automatic best_ticks_t best_s2_ticks(input ntok_t tokens);
    ntok_t half_tokens;
    best_ticks_t doubled_half_tokens;
    begin
      half_tokens = ceil_div2_ntok(tokens);
      doubled_half_tokens = {half_tokens, 1'b0};
      best_s2_ticks = doubled_half_tokens;
    end
  endfunction

  // ── Per-expert serial/parallel work estimates（tick 域）──────────────
  // serial_work(n) = ((n+1)/2) × 3 ticks = best_s4_ticks(n) × 3
  //   = 3 × ceil(n/2)，实现中只计算一次 ceil(n/2)
  // parallel_work(n) = ((n+3)/4) × 6 ticks = ceil(n/4) × 6

  function automatic best_ticks_t serial_work_ticks(input ntok_t tokens);
    ntok_t half_tokens;
    logic [NTOK_W+1:0] triple_half_tokens;
    begin
      half_tokens = ceil_div2_ntok(tokens);
      triple_half_tokens = {1'b0, half_tokens, 1'b0} +
                           {{2{1'b0}}, half_tokens};
      serial_work_ticks = best_ticks_t'(triple_half_tokens);
    end
  endfunction

  function automatic best_ticks_t parallel_work_ticks(input ntok_t tokens);
    ntok_t quarter_tokens;
    logic [NTOK_W+1:0] six_times_quarter_tokens;
    begin
      quarter_tokens = ceil_div4_ntok(tokens);
      six_times_quarter_tokens = {quarter_tokens, 2'b00} +
                                 {1'b0, quarter_tokens, 1'b0};
      parallel_work_ticks = best_ticks_t'(six_times_quarter_tokens);
    end
  endfunction

  // ── 候选分数键（cost > remaining_count > current_makespan > candidate_id）─
  // 稳定性由 candidate_id 决定，score path 只保留会参与当前 policy 的字段。
  typedef struct packed {
    logic [T_W-1:0]  cost;              // continuation score（ticks）
    logic [NR_W-1:0] remaining_count;   // 本轮消化后剩余 expert 数
    logic [T_W-1:0]  current_makespan;  // max(C2.task_end, C3.task_end)
  } score_key_t;

  // RTL 内部 active-window 工作项。MMIO 使用 compact head16 ABI，只把调度 datapath
  // 真正需要的字段落到 FF，避免保存 rem_index/input_order/best_conc。
  //
  // 位宽（E_MAX=64）：valid + eid + ntok = 1 + 6 + 9 = 16 bits。
  // Work estimates are derived from ntok and are not stored in head/reserve FFs.
  typedef struct packed {
    logic                  valid;
    logic [EID_RAW_W-1:0]  eid;
    logic [NTOK_W-1:0]     ntok;
  } head_ctx_t;

  // Continuation scoring consumes work only; expert identity must not fan out
  // into the multi-cycle LPT engine.
  typedef struct packed {
    logic  valid;
    ntok_t ntok;
  } remaining_work_t;

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
    dma_binding_t         dma_s1;
    dma_binding_t         dma_s3;
    logic                 s2pf_valid;
    time_t                s2pf_start;
    time_t                s2pf_end;
    dma_binding_t         s2pf_dma;
    logic                 s4pf_valid;
    dma_binding_t         s4pf_dma;
  } snap_timeline_t;

  typedef struct packed {
    logic                 valid;
    time_t                task_start;
    time_t                dma1_end;
    time_t                s2_end;
    time_t                dma3_end;
    dma_binding_t         dma_s1;
    dma_binding_t         dma_s3;
    logic                 s2pf_valid;
    time_t                s2pf_start;
    time_t                s2pf_end;
    dma_binding_t         s2pf_dma;
    logic                 s4pf_valid;
    dma_binding_t         s4pf_dma;
  } snap_bw_view_t;

  // S2PF search output is a patch, not a full timeline.  The caller owns the
  // original snap_timeline_t and locally applies these few changed fields.
  typedef struct packed {
    logic                 valid;
    logic                 apply_a;
    logic                 apply_b;
  } s2pf_patch_t;

  function automatic snap_timeline_t apply_s2pf_patch_timeline(
    input snap_timeline_t timeline,
    input logic           apply_prefetch,
    input ntok_t          ntok
  );
    snap_timeline_t updated_timeline;
    begin
      updated_timeline = timeline;
      if (apply_prefetch) begin
        updated_timeline.s2pf_valid = 1'b1;
        updated_timeline.s2pf_start = timeline.dma1_end;
        updated_timeline.s2pf_end   = timeline.dma1_end + S2PF_DMA_TICKS;
        updated_timeline.s2pf_dma   = DMA_BOTH;
        updated_timeline.dma3_end   = timeline.s2_end;
        updated_timeline.dma_s3     = DMA_NONE;
        updated_timeline.task_end   = timeline.s2_end +
                                      time_t'(best_s4_ticks(ntok));
      end
      apply_s2pf_patch_timeline = updated_timeline;
    end
  endfunction

  function automatic snap_bw_view_t to_bw_view(input snap_timeline_t timeline);
    snap_bw_view_t bandwidth_view;
    begin
      bandwidth_view.valid       = timeline.valid;
      bandwidth_view.task_start  = timeline.task_start;
      bandwidth_view.dma1_end    = timeline.dma1_end;
      bandwidth_view.s2_end      = timeline.s2_end;
      bandwidth_view.dma3_end    = timeline.dma3_end;
      bandwidth_view.dma_s1      = timeline.dma_s1;
      bandwidth_view.dma_s3      = timeline.dma_s3;
      bandwidth_view.s2pf_valid  = timeline.s2pf_valid;
      bandwidth_view.s2pf_start  = timeline.s2pf_start;
      bandwidth_view.s2pf_end    = timeline.s2pf_end;
      bandwidth_view.s2pf_dma    = timeline.s2pf_dma;
      bandwidth_view.s4pf_valid  = timeline.s4pf_valid;
      bandwidth_view.s4pf_dma    = timeline.s4pf_dma;
      to_bw_view = bandwidth_view;
    end
  endfunction

  typedef struct packed {
    pf_eid_t              pf_eid;
    time_t                pf_end;
    logic                 pf_full;
  } snap_cache_t;

  typedef struct packed {
    logic [1:0] release_count;  // idle time 之后可用的额外 release endpoint 数
    time_t      first_release;
    time_t      second_release;
  } early_start_ctx_t;

  function automatic pf_eid_t encode_eid(input logic [EID_RAW_W-1:0] eid);
    encode_eid = {1'b1, eid};
  endfunction

  function automatic logic swiglu_hit_t(
    input logic [EID_RAW_W-1:0] eid,
    input pf_eid_t              pf_eid,
    input time_t                pf_end,
    input time_t                task_start
  );
    swiglu_hit_t = (pf_eid != PF_EID_NONE) &&
                   (pf_end <= task_start) &&
                   ((pf_eid == PF_EID_S4PF_WILDCARD) ||
                    (pf_eid[EID_W-1] && (pf_eid[EID_RAW_W-1:0] == eid)));
  endfunction

  // ── Normalized winning plan ────────────────────────────────────────────
  //
  // eval/replay 后直接输出 normalized physical cluster task。每个输出项是
  // 单个 task 的 token/control 对，core-local commit 子状态机生成 task_desc_t。
  //
  // 位宽（E_MAX=64）：
  //   winner_token_t  = eid6 + ntok9 + tok_start9 = 24 bits
  //   task_control_t  = cluster1 + s1/s3 4 + skip2 + has_s2pf1 = 8 bits
  //   winner_plan_t   = task_valid2 + 2*(24+8) = 66 bits
  typedef struct packed {
    logic [EID_RAW_W-1:0] eid;
    ntok_t                 ntok;
    ntok_t                 tok_start;
  } winner_token_t;

  typedef struct packed {
    logic                  cluster;     // 0=C2, 1=C3
    shape_t                shape_s1;
    shape_t                shape_s3;
    logic                  skip_s1;
    logic                  skip_s3;
    logic                  has_s2pf;
  } task_control_t;

  typedef struct packed {
    logic [1:0]            task_valid;
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
    logic [1:0]           shape_s1;
    logic [1:0]           shape_s3;
    logic                 skip_s1;
    logic                 skip_s3;
    logic                 has_s2pf;
  } task_desc_t;                       // 32 bits when E_MAX=64

  // ── MMIO task word layout ───────────────────────────────────────────────
  //
  // TASK_STREAM 返回 64-bit fast-lowering payload。每条 task 自带其 S4PF
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

  // ── Fixed S2PF policy ───────────────────────────────────────────────────
  //
  // S2PF 搜索不保留 25-way 通用枚举。候选生成时
  // 直接指定当前候选所属的硬件模板集合，降低 start mux、BW request fanout
  // 和无效中间结果。
  typedef enum logic [1:0] {
    S2PF_DISABLED           = 2'd0, // 不尝试 S2PF，只检查原始 BW
    S2PF_PAIR        = 2'd1, // PAIR: both@dma1_end, raw
    S2PF_SPLIT       = 2'd2, // SPLIT: both@dma1_end, B-only@dma1_end, raw
    S2PF_ACTIVE_SIDE = 2'd3  // one active side: active@dma1_end, raw
  } s2pf_policy_t;

  // ── 1-lane 架构存储原则 ────────────────────────────────────────────────
  // E_MAX=64 时，完整 rem_eid/ntok/order 和完整 plan list 放在 L3/CVA6
  // 软件内存；scheduler 寄存器侧只保留本轮 top6、reserve6、两个 cluster snap、
  // compact best_token/best_score、depth=8 dense task FIFO 和 FSM。
  // 时序字段仍使用 tick 域，不回退到 32-bit raw-CC。

endpackage
