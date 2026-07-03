// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — candidate generator
//
// Emits one normalized candidate descriptor per step for a 1-lane evaluator.
// The descriptor uses physical side A=C2 and side B=C3.
//
// Covered candidate families:
//   nr == 1      : 2 clusters x 9 solo shapes, split half cuts,
//                  optional early-start on idle cluster
//   both_idle    : compressed PAIR/SPLIT set tuned for hardware timing
//   not both idle: top0 to idle cluster at idle_t / busy DMA endpoints

import sched_pkg::*;

module sched_candidate_generator (
  input  logic                 clk_i,
  input  logic                 rst_ni,

  // ── 控制接口 ──────────────────────────────────────────────────────────────
  // start_i：schedule_core 在新一轮开始时拉高一拍，触发本模块进入 ST_EMIT
  // advance_i：eval_lane 处理完当前候选后拉高，本模块推进 idx_q → 下一个候选
  // busy_o：处于 ST_EMIT 状态时为高，表示仍有候选未发完
  // done_o：一轮所有候选发完后高一拍，通知 schedule_core 可以进入 commit
  input  logic                 start_i,
  input  logic                 advance_i,
  input  logic                 replay_i,
  input  logic [CAND_ID_W-1:0] replay_candidate_id_i,
  output logic                 busy_o,
  output logic                 done_o,

  // ── 当前调度状态（只读，每轮候选评估期间不变）────────────────────────────
  // c2/c3_snap_i：两个 cluster 的候选生成 view，只包含 task_end/cache/release endpoint
  // head_i[4]：CVA6/software 从 L3 rem list 中准备的 top4 缓存，对应 C rem[] 的前四项
  // active_count_i：当前仍 active 的 expert 数（用于选分支和估算 rem_len_after）
  // total_conc_i：所有 active expert 的 best_conc 之和（greedy_h 用）
  input  cand_gen_snap_t       c2_snap_i,
  input  cand_gen_snap_t       c3_snap_i,
  input  head_ctx_t [3:0]      head_i,
  input  logic [NR_W-1:0]      active_count_i,
  input  logic [T_W-1:0]       total_conc_i,

  // ── 候选描述符输出（直接送往 sched_candidate_eval_lane）──────────────────
  output cand_desc_t           cand_o,
  output logic                 cand_valid_o
);

  // 候选类型：PAIR=双 cluster 分配两个 expert，SPLIT=单 expert 拆成两份跨 cluster，SOLO=单 cluster 单 expert
  // ── 内部常量与类型定义 ─────────────────────────────────────────────────────
  localparam logic [1:0] PLAN_PAIR  = 2'b00;
  localparam logic [1:0] PLAN_SPLIT = 2'b01;
  localparam logic [1:0] PLAN_SOLO  = 2'b10;

  // 模式由 active_count / both_idle 在 ST_IDLE 时确定，整轮不变
  typedef enum logic [1:0] {
    MODE_NONE     = 2'd0,  // active_count==0，不应出现（FSM 直接跳 ST_DONE）
    MODE_SINGLE   = 2'd1,  // nr==1：solo×18 + split×2 + early-start×3
    MODE_BOTH     = 2'd2,  // both_idle：3 fixed PAIR + 2 SPLIT cuts
    MODE_NOT_BOTH = 2'd3   // not_both_idle：top0 放 idle cluster，最多3个起始时刻
  } mode_t;

  typedef enum logic [1:0] {ST_IDLE, ST_SEARCH, ST_EMIT, ST_DONE} state_t;

  // ── 内部信号声明 ──────────────────────────────────────────────────────────
  state_t st_q, st_d;
  mode_t  mode_q, mode_d;
  logic [CAND_ID_W-1:0] idx_q, idx_d;
  logic [CAND_ID_W-1:0] idx_eff;
  logic [CAND_ID_W-1:0] last_idx;

  logic [T_W-1:0] tnow;
  logic           both_idle;
  logic           idle_is_c3;
  logic [T_W-1:0] idle_t;
  cand_gen_snap_t busy_snap;
  logic [T_W-1:0] tpts [3];
  logic [1:0]     ntpts;
  logic [T_W-1:0] release_t [4];
  logic [3:0]     release_valid;

  // ── 时刻基准信号（纯组合，跟随输入实时更新）──────────────────────────────
  // tnow：当前最早可用时刻（两 cluster task_end 的最大值），作为 both_idle 的基准起始时间
  assign tnow      = (c2_snap_i.task_end > c3_snap_i.task_end) ?
                     c2_snap_i.task_end : c3_snap_i.task_end;
  // both_idle：两 cluster 同时空闲（task_end 相等），对应 C moe_plan() 的 both_idle 分支
  assign both_idle = (c2_snap_i.task_end == c3_snap_i.task_end);
  // idle_is_c3：C3 先空闲（task_end 更小）时为1，新任务放 C3；
  //             C2 先空闲时为0，新任务放 C2。
  assign idle_is_c3 = (c3_snap_i.task_end < c2_snap_i.task_end);
  assign idle_t     = idle_is_c3 ? c3_snap_i.task_end : c2_snap_i.task_end;
  // busy_snap：较忙（task_end 更大）的 cluster 的状态。
  //   idle_is_c3=1 → C3 空闲、C2 更忙 → busy_snap = C2
  //   idle_is_c3=0 → C2 空闲、C3 更忙 → busy_snap = C3
  // 目的：从忙的那个 cluster 的 DMA 时间线中，提取其 S1/S2PF/S3 DMA 结束时刻，
  // 作为新任务在 idle cluster 上的"早起步"候选起始点（此时部分 DMA 带宽已释放）。
  assign busy_snap  = idle_is_c3 ? c2_snap_i : c3_snap_i;

  // ── early-start 候选起始时刻（MODE_SINGLE idx20..22 和 MODE_NOT_BOTH 共用）──
  //
  // release endpoint 已由 schedule_core 从完整 eval_snap_t 预先压缩成
  // cand_gen_snap_t.release_valid/t。这里只负责从忙侧 cluster 的 release list
  // 里选出前两个晚于 idle_t 的非重复 endpoint，避免完整 snap 字段跨模块扇出。
  always_comb begin
    int unsigned tpt_pos;
    logic dup;

    for (int i = 0; i < 4; i++) begin
      release_t[i] = '0;
    end
    release_valid = '0;
    release_valid = busy_snap.release_valid;
    for (int i = 0; i < 4; i++) begin
      release_t[i] = busy_snap.release_t[i];
    end

    tpts[0] = idle_t;
    tpts[1] = '0;
    tpts[2] = '0;
    tpt_pos = 1;

    for (int i = 0; i < 4; i++) begin
      if (release_valid[i] && (release_t[i] > idle_t) && (tpt_pos < 3)) begin
        dup = 1'b0;
        for (int j = 0; j < 3; j++) begin
          if ((j < int'(tpt_pos)) && (tpts[j] == release_t[i])) begin
            dup = 1'b1;
          end
        end
        if (!dup) begin
          tpts[tpt_pos] = release_t[i];
          tpt_pos++;
        end
      end
    end

    ntpts = 2'(tpt_pos);
  end

  // ── 辅助函数（综合时展开为查找表/比较树）────────────────────────────────
  // MODE_SINGLE 只保留两个 split：ceil(ntok/2) 与 floor(ntok/2)。
  // 删除通用 cut 枚举后，这里不再需要动态 slot case mux 和重复 cut 比较链。
  function automatic logic [NTOK_W-1:0] single_split_cut(
    input logic [NTOK_W-1:0] ntok,
    input logic              id_is_floor
  );
    single_split_cut = id_is_floor ? (ntok >> 1) :
                                     ((ntok + NTOK_W'(1)) >> 1);
  endfunction

  // solo shape 枚举：S1 和 S3 各3种 shape（A/B/C），组合成 9 种配置
  // 2D 展开：行方向 = S1 shape（每3个一组），列方向 = S3 shape
  //   slot: 0(AA) 1(AB) 2(AC) | 3(BA) 4(BB) 5(BC) | 6(CA) 7(CB) 8(CC)
  function automatic logic [1:0] solo_s1_for_slot(input logic [3:0] slot);
    unique case (slot)
      4'd0, 4'd1, 4'd2: solo_s1_for_slot = SHAPE_A;
      4'd3, 4'd4, 4'd5: solo_s1_for_slot = SHAPE_B;
      default:          solo_s1_for_slot = SHAPE_C;
    endcase
  endfunction

  function automatic logic [1:0] solo_s3_for_slot(input logic [3:0] slot);
    unique case (slot)
      4'd0, 4'd3, 4'd6: solo_s3_for_slot = SHAPE_A;
      4'd1, 4'd4, 4'd7: solo_s3_for_slot = SHAPE_B;
      default:          solo_s3_for_slot = SHAPE_C;
    endcase
  endfunction

  // MODE_SINGLE split 有效性：
  //   - ceil 候选：只要求 0 < ceil < ntok
  //   - floor 候选：还要求 floor != ceil，避免偶数 ntok 重复评估同一 cut
  function automatic logic single_split_valid(
    input logic [NTOK_W-1:0] ntok,
    input logic              id_is_floor
  );
    logic [NTOK_W-1:0] c0;
    logic [NTOK_W-1:0] c1;
    begin
      c0 = (ntok + NTOK_W'(1)) >> 1;
      c1 = ntok >> 1;

      if (!id_is_floor) begin
        single_split_valid = (ntok >= NTOK_W'(2)) && (c0 > '0) && (c0 < ntok);
      end else begin
        single_split_valid = (ntok >= NTOK_W'(2)) && (c1 > '0) &&
                             (c1 < ntok) && (c1 != c0);
      end
    end
  endfunction

  // both_idle 使用压缩后的 split 集合：
  //   id 3: half_ceil = ceil(ntok/2)
  //   id 4: front_m2  = cut=2
  // 这里不复用通用 duplicate 检查链。front_m2 只有在 ntok>=5 时才
  // 不同于 half_ceil 且合法。
  function automatic logic [NTOK_W-1:0] both_split_cut_for_id(
    input logic [NTOK_W-1:0]      ntok,
    input logic [CAND_ID_W-1:0]   id
  );
    both_split_cut_for_id = (id == CAND_ID_W'(3)) ?
                            ((ntok + NTOK_W'(1)) >> 1) :
                            NTOK_W'(2);
  endfunction

  function automatic logic both_split_cut_valid(
    input logic [NTOK_W-1:0]      ntok,
    input logic [CAND_ID_W-1:0]   id
  );
    if (id == CAND_ID_W'(3)) begin
      both_split_cut_valid = (ntok >= NTOK_W'(2));
    end else begin
      both_split_cut_valid = (ntok >= NTOK_W'(5));
    end
  endfunction

  function automatic logic [CAND_ID_W-1:0] mode_last_idx(input mode_t mode);
    unique case (mode)
      MODE_SINGLE:   mode_last_idx = CAND_ID_W'(22);
      MODE_BOTH:     mode_last_idx = CAND_ID_W'(4);
      MODE_NOT_BOTH: mode_last_idx = CAND_ID_W'(2);
      default:       mode_last_idx = '0;
    endcase
  endfunction

  // candidate_id_possible：只判断“这个 candidate ID 在本轮有没有可能有效”。
  // MODE_BOTH 已经改成压缩 candidate 编码；best/replay 使用这个压缩 ID 做稳定
  // tie-break。FSM 仍然顺序跳过无效 ID，避免展开多路 find 逻辑。
  function automatic logic candidate_id_possible(
    input mode_t                 mode,
    input logic [CAND_ID_W-1:0]  id
  );
    logic single_split_is_floor;
    logic [NTOK_W-1:0] cut;
    begin
      candidate_id_possible = 1'b0;
      single_split_is_floor = 1'b0;
      cut = '0;

      unique case (mode)
        MODE_SINGLE: begin
          if (id < CAND_ID_W'(18)) begin
            candidate_id_possible = head_i[0].valid;
          end else if (id < CAND_ID_W'(20)) begin
            single_split_is_floor = (id == CAND_ID_W'(19));
            candidate_id_possible = head_i[0].valid &&
                                    single_split_valid(head_i[0].ntok,
                                                       single_split_is_floor);
          end else if (id <= CAND_ID_W'(22)) begin
            candidate_id_possible = head_i[0].valid && !both_idle &&
                                    (2'(id - CAND_ID_W'(20)) < ntpts);
          end
        end

        MODE_BOTH: begin
          unique case (id)
            CAND_ID_W'(0): begin
              candidate_id_possible = (active_count_i >= NR_W'(2)) &&
                                      head_i[0].valid && head_i[1].valid;
            end
            CAND_ID_W'(1): begin
              candidate_id_possible = (active_count_i >= NR_W'(3)) &&
                                      head_i[1].valid && head_i[2].valid;
            end
            CAND_ID_W'(2): begin
              candidate_id_possible = (active_count_i >= NR_W'(4)) &&
                                      head_i[2].valid && head_i[3].valid;
            end
            CAND_ID_W'(3), CAND_ID_W'(4): begin
              cut = both_split_cut_for_id(head_i[0].ntok, id);
              candidate_id_possible = head_i[0].valid &&
                                      (cut > '0) && (cut < head_i[0].ntok) &&
                                      both_split_cut_valid(head_i[0].ntok, id);
            end
            default: candidate_id_possible = 1'b0;
          endcase
        end

        MODE_NOT_BOTH: begin
          candidate_id_possible = head_i[0].valid && (id < CAND_ID_W'(ntpts));
        end

        default: candidate_id_possible = 1'b0;
      endcase
    end
  endfunction

  // ── 轮次 FSM 与全模式候选编码 ────────────────────────────────────────────────
  // 覆盖 MODE_SINGLE / MODE_BOTH / MODE_NOT_BOTH 三种模式，与上方 tpts/cand_* 无直属模式绑定。

  // 每种模式的最后一个候选 ID（到达后下一拍转 ST_DONE）
  //   MODE_SINGLE   : 0..22 → 23候选 (18 solo + 2 split + 3 early-start)
  //   MODE_BOTH     : 0..4  → 5候选 (3 fixed PAIR + half_ceil/front_m2 SPLIT)
  //   MODE_NOT_BOTH : 0..2  → 最多3候选 (ntpts 个有效起始时刻)
  always_comb begin
    last_idx = mode_last_idx(mode_q);
  end

  always_comb begin
    st_d   = st_q;
    mode_d = mode_q;
    idx_d  = idx_q;

    unique case (st_q)
      ST_IDLE: begin
        // start_i 高时：根据 active_count / both_idle 决定本轮枚举模式。
        // idx 从 0 开始，后续由 ST_SEARCH 顺序跳过无效 candidate。
        // 这里刻意不用 24 路组合 find，避免 head_i/active_count_i 被大扇出展开。
        if (start_i) begin
          mode_t next_mode;

          if (active_count_i == NR_W'(0)) begin
            next_mode = MODE_NONE;
          end else if (active_count_i == NR_W'(1)) begin
            next_mode = MODE_SINGLE;
          end else if (both_idle) begin
            next_mode = MODE_BOTH;
          end else begin
            next_mode = MODE_NOT_BOTH;
          end

          mode_d = next_mode;
          if (next_mode == MODE_NONE) begin
            st_d = ST_DONE;
          end else begin
            idx_d = '0;
            st_d  = ST_SEARCH;
          end
        end
      end

      ST_SEARCH: begin
        // 每拍只检查一个 candidate ID。这样 head_i 只进入一个
        // candidate_id_possible() 实例，而不是同时展开 20+ 个实例。
        if (idx_q > last_idx) begin
          st_d = ST_DONE;
        end else if (candidate_id_possible(mode_q, idx_q)) begin
          st_d = ST_EMIT;
        end else begin
          idx_d = idx_q + CAND_ID_W'(1);
        end
      end

      ST_EMIT: begin
        // advance_i 由外层 eval_lane 驱动，每评估完一个候选拉高一拍。
        // 这里进入 ST_SEARCH 顺序跳过后续无效 ID，用时间换 fanout/组合深度。
        if (advance_i) begin
          idx_d = idx_q + CAND_ID_W'(1);
          st_d  = ST_SEARCH;
        end
      end

      ST_DONE: begin
        // ST_DONE 只保持一拍，schedule_core 在这一拍提交 best candidate。
        st_d = ST_IDLE;
      end

      default: st_d = ST_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q   <= ST_IDLE;
      mode_q <= MODE_NONE;
      idx_q  <= '0;
    end else begin
      st_q   <= st_d;
      mode_q <= mode_d;
      idx_q  <= idx_d;
    end
  end

  assign idx_eff = replay_i ? replay_candidate_id_i : idx_q;

  // mask_one_slot：将当前 top4 window 的 slot 编码成 remove_slot_mask。
  //
  // remove_slot_mask[i]=1 表示 commit 后移除本轮 HEADi。这样 wrapper/CVA6
  // 可以对 4-entry top window 做 order-preserving compact，再从 tail push
  // 新 expert。算法仍保留 PAIR(topK,topJ) 等非 FIFO 消耗模式。
  function automatic logic [3:0] mask_one_slot(input logic [1:0] slot);
    logic [3:0] m;
    m = '0;
    m[slot] = 1'b1;
    mask_one_slot = m;
  endfunction

  function automatic logic [3:0] mask_two_slots(input logic [1:0] slot_a,
                                                input logic [1:0] slot_b);
    mask_two_slots = mask_one_slot(slot_a) | mask_one_slot(slot_b);
  endfunction

  // ── 后继 rem 状态投影（由 annotate_cand_hits_and_rem 调用）──────────────────────────
  // project_rem_if_picked：纯组合投影，假设当前候选被选中时，rem 的剩余汇总状态。
  // 结果填入 cand_o，eval_lane 用这些字段计算 continuation_cost，不做任何实际状态更新。
  //
  // 注意：RTL 只能访问本轮 top4 cache，无法逐一读取 top4 以外的 expert。
  //   - rem_len_after：由 active_count_i 减去 remove_slot_mask 中被移除的数量得到（精确）
  //   - total_conc_after：由 total_conc_i 减去 top4 中被移除项的 best_conc 之和（精确，
  //     因为 commit 候选必在 top4 内）
  //   - rem0/rem1_ntok、max_conc_after：只看 top4 中第一、二个 still-active 项（近似，
  //     commit 后 top4 会由 CVA6/software 刷新，这里是估计值）
  task automatic project_rem_if_picked(
    input  logic [3:0] remove_slot_mask,
    output logic [NR_W-1:0]  rem_len_after,
    output logic [EID_RAW_W-1:0]  rem0_eid,
    output logic [NTOK_W-1:0] rem0_ntok,
    output logic [NTOK_W-1:0] rem1_ntok,
    output logic [T_W-1:0]   total_conc_after,
    output logic [T_W-1:0]   max_conc_after
  );
    logic [1:0] found;
    logic [T_W-1:0] removed_conc;

    found = '0;
    removed_conc = '0;
    rem0_eid = '0;
    rem0_ntok = '0;
    rem1_ntok = '0;
    max_conc_after = '0;

    // 遍历 top4：累加被移除项的 conc，并找出 commit 后剩余的第一、二项
    for (int i = 0; i < 4; i++) begin
      if (head_i[i].valid && remove_slot_mask[i]) begin
        removed_conc = removed_conc + best_conc_t(head_i[i].ntok);
      end
      if (head_i[i].valid && !remove_slot_mask[i]) begin
        if (found == 2'd0) begin
          rem0_eid = head_i[i].eid;
          rem0_ntok = head_i[i].ntok;
          max_conc_after = best_conc_t(head_i[i].ntok);  // 新 top0 的 best_conc
        end else if (found == 2'd1) begin
          rem1_ntok = head_i[i].ntok;  // 新 top1 ntok（greedy_h nr==2 时用）
        end
        if (found != 2'd3) found = found + 2'd1;
      end
    end

    rem_len_after = active_count_i;
    for (int i = 0; i < 4; i++) begin
      if (remove_slot_mask[i] && (rem_len_after != '0)) begin
        rem_len_after = rem_len_after - NR_W'(1);
      end
    end
    total_conc_after = total_conc_i - removed_conc;
  endtask

  // annotate_cand_hits_and_rem：为 cand_o 补全两类辅助字段：
  //   1. cache 命中标志（sw_a/dn_a/sw_b/dn_b）：判断各侧 expert 的 SwiGLU/Down 权重
  //      是否已在 prefetch buffer 中，避免 eval_lane 重复计算
  //   2. 后继 rem 汇总（rem_len_after / rem0_ntok / total_conc_after 等）：
  //      调用 project_rem_if_picked 投影"如果选中此候选，rem 会剩什么"供 continuation_cost 使用
  task automatic annotate_cand_hits_and_rem(
    input logic [3:0] remove_slot_mask
  );
    project_rem_if_picked(remove_slot_mask,
                       cand_o.rem_len_after,
                       cand_o.rem0_eid,
                       cand_o.rem0_ntok,
                       cand_o.rem1_ntok,
                       cand_o.total_conc_after,
                       cand_o.max_conc_after);

    cand_o.sw_a = cand_o.side_a_valid &&
                  swiglu_hit_t(cand_o.eid_a, c2_snap_i.pf_eid,
                               c2_snap_i.pf_end, cand_o.start_a);
    cand_o.dn_a = cand_o.side_a_valid &&
                  down_hit_t(cand_o.eid_a, c2_snap_i.pf_eid,
                             c2_snap_i.pf_end, c2_snap_i.pf_full,
                             cand_o.start_a);
    cand_o.sw_b = cand_o.side_b_valid &&
                  swiglu_hit_t(cand_o.eid_b, c3_snap_i.pf_eid,
                               c3_snap_i.pf_end, cand_o.start_b);
    cand_o.dn_b = cand_o.side_b_valid &&
                  down_hit_t(cand_o.eid_b, c3_snap_i.pf_eid,
                             c3_snap_i.pf_end, c3_snap_i.pf_full,
                             cand_o.start_b);
  endtask

  // ── 候选描述符生成（主体组合逻辑，覆盖全部三种模式）───────────────────────
  always_comb begin
    cand_o = '0;
    cand_o.valid               = replay_i || (st_q == ST_EMIT);
    cand_o.candidate_id        = idx_eff;
    cand_o.s2pf_policy         = S2PF_PAIR_LITE;
    cand_o.shape_t0            = tnow;

    unique case (mode_q)
      // ── MODE_SINGLE：nr==1 ──────────────────────────────────────────────────
      // 对应 C moe_plan() 中 nr==1 分支。只有 head_i[0]（唯一 active expert）有效。
      // idx 0..17 : solo，依次尝试 top0 放 C2（9种 shape）和放 C3（9种 shape）
      // idx 18..19: split，将 top0 拆成两份分配给两个 cluster（ceil/floor 各一刀）
      // idx 20..22: early-start（仅 not_both_idle 时有效），top0 放 idle cluster，
      //             在 tpts[early_slot] 时刻启动，强制 ShapeC，不做 S2PF
      MODE_SINGLE: begin
        logic [3:0] solo_slot;
        logic       solo_to_c3;
        logic       single_split_is_floor;
        logic [1:0] early_slot;
        logic [NTOK_W-1:0] cut;

        solo_to_c3 = (idx_eff >= CAND_ID_W'(9)) && (idx_eff < CAND_ID_W'(18));
        solo_slot = solo_to_c3 ? 4'(idx_eff - CAND_ID_W'(9)) : 4'(idx_eff);
        single_split_is_floor = (idx_eff == CAND_ID_W'(19));
        early_slot = 2'(idx_eff - CAND_ID_W'(20));
        cut = '0;

        if (idx_eff < CAND_ID_W'(18)) begin
          cand_o.plan_type     = PLAN_SOLO;
          cand_o.cluster_a     = solo_to_c3;
          cand_o.s2pf_policy   = S2PF_OFF;
          cand_o.cost_only_tie = 1'b1;
          cand_o.remove_slot_mask = mask_one_slot(2'd0);

          if (!solo_to_c3) begin
            cand_o.side_a_valid = head_i[0].valid;
            cand_o.start_a      = c2_snap_i.task_end;
            cand_o.eid_a        = head_i[0].eid;
            cand_o.ntok_a       = head_i[0].ntok;
            cand_o.force_shape_a = 1'b1;
            cand_o.forced_s1a    = solo_s1_for_slot(solo_slot);
            cand_o.forced_s3a    = solo_s3_for_slot(solo_slot);
            cand_o.shape_t0      = c2_snap_i.task_end;
          end else begin
            cand_o.side_b_valid = head_i[0].valid;
            cand_o.start_b      = c3_snap_i.task_end;
            cand_o.eid_b        = head_i[0].eid;
            cand_o.ntok_b       = head_i[0].ntok;
            cand_o.force_shape_b = 1'b1;
            cand_o.forced_s1b    = solo_s1_for_slot(solo_slot);
            cand_o.forced_s3b    = solo_s3_for_slot(solo_slot);
            cand_o.shape_t0      = c3_snap_i.task_end;
          end
        end else if (idx_eff < CAND_ID_W'(20)) begin
            cut = single_split_cut(head_i[0].ntok, single_split_is_floor);
            cand_o.plan_type     = PLAN_SPLIT;
            cand_o.cluster_a     = 1'b0;
            cand_o.s2pf_policy   = S2PF_SPLIT_LITE;
            cand_o.cost_only_tie = 1'b1;
            cand_o.side_a_valid  = single_split_valid(head_i[0].ntok,
                                                      single_split_is_floor);
            cand_o.side_b_valid  = cand_o.side_a_valid;
            cand_o.start_a       = tnow;
            cand_o.start_b       = tnow;
            cand_o.eid_a         = head_i[0].eid;
            cand_o.eid_b         = head_i[0].eid;
            cand_o.ntok_a        = cut;
            cand_o.ntok_b        = head_i[0].ntok - cut;
            cand_o.tok_start_b   = cut;
            cand_o.remove_slot_mask = mask_one_slot(2'd0);
        end else begin
          cand_o.valid          = cand_o.valid && !both_idle && head_i[0].valid &&
                                  (early_slot < ntpts);
          cand_o.plan_type      = PLAN_SOLO;
          cand_o.cluster_a      = idle_is_c3;
          cand_o.s2pf_policy    = S2PF_OFF;
          cand_o.cost_only_tie  = 1'b1;
          cand_o.remove_slot_mask = mask_one_slot(2'd0);
          cand_o.shape_t0       = tpts[early_slot];

          if (!idle_is_c3) begin
            cand_o.side_a_valid  = cand_o.valid;
            cand_o.start_a       = tpts[early_slot];
            cand_o.eid_a         = head_i[0].eid;
            cand_o.ntok_a        = head_i[0].ntok;
            cand_o.force_shape_a = 1'b1;
            cand_o.forced_s1a    = SHAPE_C;
            cand_o.forced_s3a    = SHAPE_C;
          end else begin
            cand_o.side_b_valid  = cand_o.valid;
            cand_o.start_b       = tpts[early_slot];
            cand_o.eid_b         = head_i[0].eid;
            cand_o.ntok_b        = head_i[0].ntok;
            cand_o.force_shape_b = 1'b1;
            cand_o.forced_s1b    = SHAPE_C;
            cand_o.forced_s3b    = SHAPE_C;
          end
        end
      end

      // ── MODE_BOTH：both_idle ─────────────────────────────────────────────────
      // hardware-lite both_idle 分支。两 cluster 同时从 tnow 启动。
      // idx 0: PAIR(top0 -> C2, top1 -> C3)
      // idx 1: PAIR(top1 -> C2, top2 -> C3)
      // idx 2: PAIR(top2 -> C2, top3 -> C3)
      // idx 3: SPLIT(top0), half_ceil
      // idx 4: SPLIT(top0), front_m2 (cut=2)
      MODE_BOTH: begin
        head_ctx_t ha;
        head_ctx_t hb;
        logic [1:0] slot_a;
        logic [1:0] slot_b;
        logic       pair_enough;
        logic [NTOK_W-1:0] cut;

        ha = '0;
        hb = '0;
        slot_a = '0;
        slot_b = '0;
        pair_enough = 1'b0;
        cut = '0;
        cand_o.plan_type = PLAN_PAIR;
        cand_o.cluster_a = 1'b0;  // A 侧固定 C2，B 侧固定 C3
        cand_o.start_a   = tnow;
        cand_o.start_b   = tnow;

        if (idx_eff <= CAND_ID_W'(2)) begin
          unique case (idx_eff)
            CAND_ID_W'(0): begin
              slot_a = 2'd0;
              slot_b = 2'd1;
              pair_enough = (active_count_i >= NR_W'(2));
            end
            CAND_ID_W'(1): begin
              slot_a = 2'd1;
              slot_b = 2'd2;
              pair_enough = (active_count_i >= NR_W'(3));
            end
            default: begin
              slot_a = 2'd2;
              slot_b = 2'd3;
              pair_enough = (active_count_i >= NR_W'(4));
            end
          endcase
          ha = head_i[slot_a];
          hb = head_i[slot_b];
        end else begin
          cut = both_split_cut_for_id(head_i[0].ntok, idx_eff);
          cand_o.plan_type    = PLAN_SPLIT;
          cand_o.s2pf_policy  = S2PF_SPLIT_LITE;
          cand_o.side_a_valid = both_split_cut_valid(head_i[0].ntok, idx_eff);
          cand_o.side_b_valid = cand_o.side_a_valid;
          cand_o.eid_a        = head_i[0].eid;
          cand_o.eid_b        = head_i[0].eid;
          cand_o.ntok_a       = cut;
          cand_o.ntok_b       = head_i[0].ntok - cut;
          cand_o.tok_start_b  = cut;
          cand_o.remove_slot_mask = mask_one_slot(2'd0);
        end

        if (idx_eff <= CAND_ID_W'(2)) begin
          cand_o.valid = cand_o.valid && ha.valid && hb.valid && pair_enough;
          cand_o.side_a_valid = cand_o.valid;
          cand_o.side_b_valid = cand_o.valid;
          cand_o.eid_a        = ha.eid;
          cand_o.eid_b        = hb.eid;
          cand_o.ntok_a       = ha.ntok;
          cand_o.ntok_b       = hb.ntok;
          cand_o.remove_slot_mask = mask_two_slots(slot_a, slot_b);
        end
      end

      // ── MODE_NOT_BOTH：not_both_idle ─────────────────────────────────────────
      // 对应 C moe_plan() 中 not_both_idle 分支。
      // 将 top0 放到 idle cluster，依次尝试 tpts[0..ntpts-1] 这几个起始时刻。
      // 固定 ShapeC（force_shape=1, forced_s1/s3=SHAPE_C）使 bw_req=128，
      // S2PF 使用 SINGLE_LATEST policy：只尝试最晚一个合法起点。
      MODE_NOT_BOTH: begin
        // idx >= ntpts 的候选标为无效（tpts 可能只有1或2个有效时刻）
        cand_o.valid              = cand_o.valid && head_i[0].valid &&
                                    (idx_eff < CAND_ID_W'(ntpts));
        cand_o.plan_type          = PLAN_SOLO;
        cand_o.cluster_a          = idle_is_c3;  // 把 top0 放到 idle 的那个 cluster
        cand_o.s2pf_policy        = S2PF_SINGLE_LATEST;
        cand_o.cost_only_tie      = 1'b1;
        cand_o.score_makespan_only = 1'b1;
        cand_o.remove_slot_mask   = mask_one_slot(2'd0);
        cand_o.shape_t0           = tpts[idx_eff[1:0]];  // 使用对应的候选起始时刻

        if (!idle_is_c3) begin
          cand_o.side_a_valid = cand_o.valid;
          cand_o.start_a      = tpts[idx_eff[1:0]];
          cand_o.eid_a        = head_i[0].eid;
          cand_o.ntok_a       = head_i[0].ntok;
          cand_o.force_shape_a = 1'b1;
          cand_o.forced_s1a    = SHAPE_C;
          cand_o.forced_s3a    = SHAPE_C;
        end else begin
          cand_o.side_b_valid = cand_o.valid;
          cand_o.start_b      = tpts[idx_eff[1:0]];
          cand_o.eid_b        = head_i[0].eid;
          cand_o.ntok_b       = head_i[0].ntok;
          cand_o.force_shape_b = 1'b1;
          cand_o.forced_s1b    = SHAPE_C;
          cand_o.forced_s3b    = SHAPE_C;
        end
      end

      default: cand_o.valid = 1'b0;
    endcase

    cand_o.valid = cand_o.valid && (cand_o.side_a_valid || cand_o.side_b_valid);
    annotate_cand_hits_and_rem(cand_o.remove_slot_mask);
  end

  // ── 输出连接 ──────────────────────────────────────────────────────────────
  assign cand_valid_o = cand_o.valid;
  assign busy_o       = (st_q == ST_EMIT);
  assign done_o       = (st_q == ST_DONE);

endmodule
