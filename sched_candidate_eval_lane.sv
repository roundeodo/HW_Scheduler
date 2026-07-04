// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — 1-lane candidate evaluator（Tick 域版本）
//
// 一个 lane 每次评估一个已经规范化的候选：
//   eval_req_q -> shape_q -> shared mk_snap(A) -> raw_timeline_a_q/cache_a_q
//              -> shared mk_snap(B) -> raw_timeline_b_q/cache_b_q
//              -> lite S2PF policy -> bw_ok -> continuation_cost -> score_key/plan_desc
//
// 本模块是带 start/done 握手的单候选 evaluator。start_i 到来后先把
// candidate_generator 输出裁剪成 eval_req_q，再显式经过 EV_PICK/EV_MK_A/
// EV_MK_B/S2PF/SCORE。这样 candidate 输入不会直接扇出到 pick/mk/s2pf/
// score/plan_desc，且 A/B 复用一套 mk_snap 组合逻辑。
// 当 continuation_cost 进入 rem_len==1 的 sim1 FSM 时，本 lane 会多拍完成。

import sched_pkg::*;
import sched_candidate_pkg::*;

module sched_candidate_eval_lane (
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic               start_i,
  output logic               busy_o,
  output logic               done_o,

  input  logic               cand_valid_i,

  // ── 候选控制 ─────────────────────────────────────────────────────────────
  input  logic [1:0]         plan_type_i,          // 00=PAIR, 01=SPLIT, 10=SOLO
  input  logic               cluster_a_i,          // task A 所在 cluster（0=C2, 1=C3）
  input  s2pf_policy_t       s2pf_policy_i,
  input  logic               force_shape_a_i,
  input  logic               force_shape_b_i,
  input  logic [1:0]         forced_s1a_i,
  input  logic [1:0]         forced_s3a_i,
  input  logic [1:0]         forced_s1b_i,
  input  logic [1:0]         forced_s3b_i,
  input  logic               cost_only_tie_i,
  input  logic               score_makespan_only_i,

  // ── 当前 C2/C3 base snap；某侧没有新 task 时直接沿用 base snap ─────────
  input  snap_timeline_t     base_timeline_a_i,
  input  snap_timeline_t     base_timeline_b_i,
  input  snap_cache_t        base_cache_a_i,
  input  snap_cache_t        base_cache_b_i,

  // ── 候选 task A/B 描述 ──────────────────────────────────────────────────
  input  logic               side_a_valid_i,
  input  logic               side_b_valid_i,
  input  logic [T_W-1:0]     start_a_i,
  input  logic [T_W-1:0]     start_b_i,
  input  logic [EID_RAW_W-1:0]  eid_a_i,
  input  logic [EID_RAW_W-1:0]  eid_b_i,
  input  logic [NTOK_W-1:0]  ntok_a_i,
  input  logic [NTOK_W-1:0]  ntok_b_i,
  input  logic [NTOK_W-1:0]  tok_start_a_i,
  input  logic [NTOK_W-1:0]  tok_start_b_i,

  // ── 缓存命中标志；由 candidate_generator 按当前 base snap/t0 计算 ───────
  input  logic               sw_a_i,
  input  logic               dn_a_i,
  input  logic               sw_b_i,
  input  logic               dn_b_i,
  input  logic [T_W-1:0]     shape_t0_i,

  // ── continuation_cost 所需的 rem-after 摘要 ─────────────────────────────
  input  logic [NR_W-1:0]    rem_len_after_i,
  input  logic [EID_RAW_W-1:0]  rem0_eid_i,
  input  logic [NTOK_W-1:0]  rem0_ntok_i,
  input  logic [NTOK_W-1:0]  rem1_ntok_i,
  input  logic [T_W-1:0]     total_conc_after_i,
  input  logic [T_W-1:0]     max_conc_after_i,

  output logic               bw_start_o,
  output snap_timeline_t     bw_snap_a_o,
  output snap_timeline_t     bw_snap_b_o,
  input  logic               bw_done_i,
  input  logic               bw_ok_i,

  // ── 评估结果 ─────────────────────────────────────────────────────────────
  output logic               eval_valid_o,
  output logic               bw_ok_o,
  output logic [T_W-1:0]     makespan_o,
  output score_key_t         score_key_o,
  output plan_desc_t         plan_desc_o,
  output snap_timeline_t     snap_timeline_a_o,
  output snap_timeline_t     snap_timeline_b_o,
  output snap_cache_t        snap_cache_a_o,
  output snap_cache_t        snap_cache_b_o,

  // ── 调试标量输出 ────────────────────────────────────────────────────────
  output logic [1:0]         shape_s1a_o,
  output logic [1:0]         shape_s3a_o,
  output logic [1:0]         shape_s1b_o,
  output logic [1:0]         shape_s3b_o,
  output logic [T_W-1:0]     task_end_a_o,
  output logic [T_W-1:0]     task_end_b_o,
  output logic [T_W-1:0]     s2_end_a_o,
  output logic [T_W-1:0]     s2_end_b_o,
  output logic [T_W-1:0]     s4_start_a_o,
  output logic [T_W-1:0]     s4_start_b_o,
  output logic [BW_W-1:0]    bw_s1a_o,
  output logic [BW_W-1:0]    bw_s3a_o,
  output logic [BW_W-1:0]    bw_s1b_o,
  output logic [BW_W-1:0]    bw_s3b_o,
  output logic [NTOK_W-1:0]  m_s2_a_o,
  output logic [NTOK_W-1:0]  m_s4_a_o,
  output logic [NTOK_W-1:0]  m_s2_b_o,
  output logic [NTOK_W-1:0]  m_s4_b_o,
  output logic               skip_s2_a_o,
  output logic               skip_s4_a_o,
  output logic               skip_s2_b_o,
  output logic               skip_s4_b_o,
  output logic [1:0]         dma_s1_a_o,
  output logic [1:0]         dma_s3_a_o,
  output logic [1:0]         dma_s1_b_o,
  output logic [1:0]         dma_s3_b_o
);

  // ── shared mk_snap scalar wires ────────────────────────────────────────
  logic [T_W-1:0] mk_task_start, mk_task_end, mk_dma1_end;
  logic [T_W-1:0] mk_s2_end, mk_dma3_end, mk_s4_start;
  logic [BW_W-1:0] mk_bw_s1, mk_bw_s3;
  logic [NTOK_W-1:0] mk_m_s2, mk_m_s4;
  logic mk_skip_s2, mk_skip_s4;
  logic [1:0] mk_dma_s1, mk_dma_s3;

  logic [T_W-1:0] mk_start_t;
  logic [NTOK_W-1:0] mk_ntok;
  logic [1:0] mk_shape_s1, mk_shape_s3;
  logic mk_skip_s1, mk_skip_s3;
  logic mk_side_b;

  snap_timeline_t raw_timeline_a_q, raw_timeline_a_d;
  snap_timeline_t raw_timeline_b_q, raw_timeline_b_d;
  snap_cache_t    raw_cache_a_q, raw_cache_a_d;
  snap_cache_t    raw_cache_b_q, raw_cache_b_d;
  snap_timeline_t post_s2pf_a_timeline;
  snap_timeline_t post_s2pf_b_timeline;
  logic [T_W-1:0] cont_cost;
  logic s2pf_start;
  logic s2pf_busy;
  logic s2pf_done;
  logic s2pf_bw_start;
  snap_timeline_t s2pf_bw_timeline_a;
  snap_timeline_t s2pf_bw_timeline_b;
  logic score_start;
  logic score_busy;
  logic score_done;
  logic score_bw_start;
  snap_timeline_t score_bw_snap_a;
  snap_timeline_t score_bw_snap_b;
  logic eval_candidate_ok;
  logic [1:0] pick_s1a;
  logic [1:0] pick_s3a;
  logic [1:0] pick_s1b;
  logic [1:0] pick_s3b;
  logic [1:0] shape_s1a_q, shape_s1a_d;
  logic [1:0] shape_s3a_q, shape_s3a_d;
  logic [1:0] shape_s1b_q, shape_s1b_d;
  logic [1:0] shape_s3b_q, shape_s3b_d;

  logic [NTOK_W-1:0] m_s2_a_q, m_s2_a_d;
  logic [NTOK_W-1:0] m_s4_a_q, m_s4_a_d;
  logic [NTOK_W-1:0] m_s2_b_q, m_s2_b_d;
  logic [NTOK_W-1:0] m_s4_b_q, m_s4_b_d;
  logic skip_s2_a_q, skip_s2_a_d;
  logic skip_s4_a_q, skip_s4_a_d;
  logic skip_s2_b_q, skip_s2_b_d;
  logic skip_s4_b_q, skip_s4_b_d;
  logic [1:0] dma_s1_a_q, dma_s1_a_d;
  logic [1:0] dma_s3_a_q, dma_s3_a_d;
  logic [1:0] dma_s1_b_q, dma_s1_b_d;
  logic [1:0] dma_s3_b_q, dma_s3_b_d;
  logic result_valid_q, result_valid_d;

  typedef enum logic [3:0] {
    EV_IDLE,
    EV_LATCH,
    EV_PICK,
    EV_MK_A,
    EV_MK_B,
    EV_S2PF_START,
    EV_WAIT_S2PF,
    EV_WAIT_SCORE,
    EV_DONE
  } eval_state_t;

  eval_state_t ev_st_q, ev_st_d;

  // eval_req_q 是 candidate_generator → eval_lane 的第一级寄存器边界。
  // 只锁存本 lane 后续 pick/mk/s2pf/score/plan_desc 真正读取的字段；
  // 不锁存完整 cand_issue_t，也不锁存 base snap，避免无意义 FF 和结构体大扇出。
  typedef struct packed {
    logic                  valid;
    logic [1:0]            plan_type;
    logic                  cluster_a;
    s2pf_policy_t          s2pf_policy;
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
    logic                  force_shape_a;
    logic                  force_shape_b;
    logic [1:0]            forced_s1a;
    logic [1:0]            forced_s3a;
    logic [1:0]            forced_s1b;
    logic [1:0]            forced_s3b;

    logic [NR_W-1:0]       rem_len_after;
    logic [EID_RAW_W-1:0]  rem0_eid;
    logic [NTOK_W-1:0]     rem0_ntok;
    logic [NTOK_W-1:0]     rem1_ntok;
    logic [T_W-1:0]        total_conc_after;
    logic [T_W-1:0]        max_conc_after;
  } eval_req_t;

  eval_req_t req_q, req_d;

  // ── pick_shapes 实例 ─────────────────────────────────────────────────────
  sched_pick_shapes i_pick_shapes (
    .ntok_a_i (req_q.ntok_a),
    .ntok_b_i (req_q.ntok_b),
    .sw_a_i   (req_q.sw_a),
    .dn_a_i   (req_q.dn_a),
    .sw_b_i   (req_q.sw_b),
    .dn_b_i   (req_q.dn_b),
    .t0_i     (req_q.shape_t0),
    .s1a_o    (pick_s1a),
    .s3a_o    (pick_s3a),
    .s1b_o    (pick_s1b),
    .s3b_o    (pick_s3b)
  );

  assign shape_s1a_o = shape_s1a_q;
  assign shape_s3a_o = shape_s3a_q;
  assign shape_s1b_o = shape_s1b_q;
  assign shape_s3b_o = shape_s3b_q;

  // ── shared mk_snap：EV_MK_A/EV_MK_B 两拍复用同一套组合逻辑 ─────────────
  assign mk_side_b   = (ev_st_q == EV_MK_B);
  assign mk_start_t  = mk_side_b ? req_q.start_b : req_q.start_a;
  assign mk_ntok     = mk_side_b ? req_q.ntok_b  : req_q.ntok_a;
  assign mk_shape_s1 = mk_side_b ? shape_s1b_q   : shape_s1a_q;
  assign mk_shape_s3 = mk_side_b ? shape_s3b_q   : shape_s3a_q;
  assign mk_skip_s1  = mk_side_b ? req_q.sw_b    : req_q.sw_a;
  assign mk_skip_s3  = mk_side_b ? req_q.dn_b    : req_q.dn_a;

  sched_mk_snap i_mk_snap (
    .start_t_i     (mk_start_t),
    .ntok_i        (mk_ntok),
    .shape_s1_i    (mk_shape_s1),
    .shape_s3_i    (mk_shape_s3),
    .skip_s1_i     (mk_skip_s1),
    .skip_s3_i     (mk_skip_s3),
    .task_start_o  (mk_task_start),
    .task_end_o    (mk_task_end),
    .dma1_end_o    (mk_dma1_end),
    .s2_end_o      (mk_s2_end),
    .dma3_end_o    (mk_dma3_end),
    .s4_start_o    (mk_s4_start),
    .bw_s1_o       (mk_bw_s1),
    .bw_s3_o       (mk_bw_s3),
    .m_s2_exec_o   (mk_m_s2),
    .m_s4_exec_o   (mk_m_s4),
    .skip_s2_o     (mk_skip_s2),
    .skip_s4_o     (mk_skip_s4),
    .dma_s1_o      (mk_dma_s1),
    .dma_s3_o      (mk_dma_s3)
  );

  sched_s2pf_pair i_s2pf_pair (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .start_i              (s2pf_start),
    .busy_o               (s2pf_busy),
    .done_o               (s2pf_done),
    .policy_i             (req_q.s2pf_policy),
    .side_a_active_i      (req_q.side_a_valid),
    .side_b_active_i      (req_q.side_b_valid),
    .shape_s3_a_i         (shape_s3a_q),
    .shape_s3_b_i         (shape_s3b_q),
    .snap_a_i             (raw_timeline_a_q),
    .snap_b_i             (raw_timeline_b_q),
    .bw_start_o           (s2pf_bw_start),
    .bw_snap_a_o          (s2pf_bw_timeline_a),
    .bw_snap_b_o          (s2pf_bw_timeline_b),
    .bw_done_i            ((ev_st_q == EV_WAIT_S2PF) ? bw_done_i : 1'b0),
    .bw_ok_i              (bw_ok_i),
    .ok_o                 (bw_ok_o),
    .snap_a_o             (post_s2pf_a_timeline),
    .snap_b_o             (post_s2pf_b_timeline)
  );

  assign eval_candidate_ok = req_q.valid && bw_ok_o &&
                             (req_q.side_a_valid || req_q.side_b_valid);

  sched_score_unit i_score (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .start_i            (score_start),
    .busy_o             (score_busy),
    .done_o             (score_done),
    .c2_timeline_i      (post_s2pf_a_timeline),
    .c3_timeline_i      (post_s2pf_b_timeline),
    .c2_cache_i         (raw_cache_a_q),
    .c3_cache_i         (raw_cache_b_q),
    .rem_len_i          (req_q.rem_len_after),
    .rem0_eid_i         (req_q.rem0_eid),
    .rem0_ntok_i        (req_q.rem0_ntok),
    .rem1_ntok_i        (req_q.rem1_ntok),
    .total_conc_i       (req_q.total_conc_after),
    .max_conc_i         (req_q.max_conc_after),
    .bw_start_o         (score_bw_start),
    .bw_snap_a_o        (score_bw_snap_a),
    .bw_snap_b_o        (score_bw_snap_b),
    .bw_done_i          ((ev_st_q == EV_WAIT_SCORE) ? bw_done_i : 1'b0),
    .bw_ok_i            (bw_ok_i),
    .cost_o             (cont_cost)
  );

  assign bw_start_o = (ev_st_q == EV_WAIT_SCORE) ? score_bw_start : s2pf_bw_start;
  assign bw_snap_a_o = (ev_st_q == EV_WAIT_SCORE) ? score_bw_snap_a : s2pf_bw_timeline_a;
  assign bw_snap_b_o = (ev_st_q == EV_WAIT_SCORE) ? score_bw_snap_b : s2pf_bw_timeline_b;

  always_comb begin
    ev_st_d    = ev_st_q;
    req_d      = req_q;
    raw_timeline_a_d = raw_timeline_a_q;
    raw_timeline_b_d = raw_timeline_b_q;
    raw_cache_a_d = raw_cache_a_q;
    raw_cache_b_d = raw_cache_b_q;
    shape_s1a_d = shape_s1a_q;
    shape_s3a_d = shape_s3a_q;
    shape_s1b_d = shape_s1b_q;
    shape_s3b_d = shape_s3b_q;
    m_s2_a_d = m_s2_a_q;
    m_s4_a_d = m_s4_a_q;
    m_s2_b_d = m_s2_b_q;
    m_s4_b_d = m_s4_b_q;
    skip_s2_a_d = skip_s2_a_q;
    skip_s4_a_d = skip_s4_a_q;
    skip_s2_b_d = skip_s2_b_q;
    skip_s4_b_d = skip_s4_b_q;
    dma_s1_a_d = dma_s1_a_q;
    dma_s3_a_d = dma_s3_a_q;
    dma_s1_b_d = dma_s1_b_q;
    dma_s3_b_d = dma_s3_b_q;
    result_valid_d = result_valid_q;
    s2pf_start = 1'b0;
    score_start = 1'b0;

    unique case (ev_st_q)
      EV_IDLE: begin
        if (start_i) begin
          result_valid_d = 1'b0;
          ev_st_d = EV_LATCH;
        end
      end

      EV_LATCH: begin
          req_d.valid                = cand_valid_i && (side_a_valid_i || side_b_valid_i);
          req_d.plan_type            = plan_type_i;
          req_d.cluster_a            = cluster_a_i;
          req_d.s2pf_policy          = s2pf_policy_i;
          req_d.cost_only_tie        = cost_only_tie_i;
          req_d.score_makespan_only  = score_makespan_only_i;
          req_d.side_a_valid         = side_a_valid_i;
          req_d.side_b_valid         = side_b_valid_i;
          req_d.start_a              = start_a_i;
          req_d.start_b              = start_b_i;
          req_d.eid_a                = eid_a_i;
          req_d.eid_b                = eid_b_i;
          req_d.ntok_a               = ntok_a_i;
          req_d.ntok_b               = ntok_b_i;
          req_d.tok_start_a          = tok_start_a_i;
          req_d.tok_start_b          = tok_start_b_i;
          req_d.sw_a                 = sw_a_i;
          req_d.dn_a                 = dn_a_i;
          req_d.sw_b                 = sw_b_i;
          req_d.dn_b                 = dn_b_i;
          req_d.shape_t0             = shape_t0_i;
          req_d.force_shape_a        = force_shape_a_i;
          req_d.force_shape_b        = force_shape_b_i;
          req_d.forced_s1a           = forced_s1a_i;
          req_d.forced_s3a           = forced_s3a_i;
          req_d.forced_s1b           = forced_s1b_i;
          req_d.forced_s3b           = forced_s3b_i;
          req_d.rem_len_after        = rem_len_after_i;
          req_d.rem0_eid             = rem0_eid_i;
          req_d.rem0_ntok            = rem0_ntok_i;
          req_d.rem1_ntok            = rem1_ntok_i;
          req_d.total_conc_after     = total_conc_after_i;
          req_d.max_conc_after       = max_conc_after_i;

          if (cand_valid_i && (side_a_valid_i || side_b_valid_i)) begin
            ev_st_d = EV_PICK;
          end else begin
            result_valid_d = 1'b0;
            ev_st_d = EV_DONE;
          end
      end

      EV_PICK: begin
        shape_s1a_d = req_q.force_shape_a ? req_q.forced_s1a : pick_s1a;
        shape_s3a_d = req_q.force_shape_a ? req_q.forced_s3a : pick_s3a;
        shape_s1b_d = req_q.force_shape_b ? req_q.forced_s1b : pick_s1b;
        shape_s3b_d = req_q.force_shape_b ? req_q.forced_s3b : pick_s3b;
        ev_st_d = EV_MK_A;
      end

      EV_MK_A: begin
        raw_timeline_a_d = base_timeline_a_i;
        raw_cache_a_d    = base_cache_a_i;
        if (req_q.side_a_valid) begin
          raw_timeline_a_d              = '0;
          raw_timeline_a_d.valid        = 1'b1;
          raw_timeline_a_d.task_start   = mk_task_start;
          raw_timeline_a_d.task_end     = mk_task_end;
          raw_timeline_a_d.dma1_end     = mk_dma1_end;
          raw_timeline_a_d.s2_end       = mk_s2_end;
          raw_timeline_a_d.dma3_end     = mk_dma3_end;
          raw_timeline_a_d.s4_start     = mk_s4_start;
          raw_timeline_a_d.bw_s1        = mk_bw_s1;
          raw_timeline_a_d.bw_s3        = mk_bw_s3;
          raw_timeline_a_d.ntok         = req_q.ntok_a;
          raw_cache_a_d                 = '0;
          raw_cache_a_d.pf_eid          = PF_EID_NONE;
        end
        m_s2_a_d = mk_m_s2;
        m_s4_a_d = mk_m_s4;
        skip_s2_a_d = mk_skip_s2;
        skip_s4_a_d = mk_skip_s4;
        dma_s1_a_d = mk_dma_s1;
        dma_s3_a_d = mk_dma_s3;
        ev_st_d = EV_MK_B;
      end

      EV_MK_B: begin
        raw_timeline_b_d = base_timeline_b_i;
        raw_cache_b_d    = base_cache_b_i;
        if (req_q.side_b_valid) begin
          raw_timeline_b_d              = '0;
          raw_timeline_b_d.valid        = 1'b1;
          raw_timeline_b_d.task_start   = mk_task_start;
          raw_timeline_b_d.task_end     = mk_task_end;
          raw_timeline_b_d.dma1_end     = mk_dma1_end;
          raw_timeline_b_d.s2_end       = mk_s2_end;
          raw_timeline_b_d.dma3_end     = mk_dma3_end;
          raw_timeline_b_d.s4_start     = mk_s4_start;
          raw_timeline_b_d.bw_s1        = mk_bw_s1;
          raw_timeline_b_d.bw_s3        = mk_bw_s3;
          raw_timeline_b_d.ntok         = req_q.ntok_b;
          raw_cache_b_d                 = '0;
          raw_cache_b_d.pf_eid          = PF_EID_NONE;
        end
        m_s2_b_d = mk_m_s2;
        m_s4_b_d = mk_m_s4;
        skip_s2_b_d = mk_skip_s2;
        skip_s4_b_d = mk_skip_s4;
        dma_s1_b_d = mk_dma_s1;
        dma_s3_b_d = mk_dma_s3;
        ev_st_d = EV_S2PF_START;
      end

      EV_S2PF_START: begin
        s2pf_start = req_q.valid;
        ev_st_d    = EV_WAIT_S2PF;
      end

      EV_WAIT_S2PF: begin
        if (s2pf_done) begin
          if (eval_candidate_ok) begin
            score_start = 1'b1;
            ev_st_d     = EV_WAIT_SCORE;
          end else begin
            result_valid_d = 1'b0;
            ev_st_d = EV_DONE;
          end
        end
      end

      EV_WAIT_SCORE: begin
        if (score_done) begin
          result_valid_d = eval_candidate_ok;
          ev_st_d = EV_DONE;
        end
      end

      EV_DONE: begin
        ev_st_d = EV_IDLE;
      end

      default: ev_st_d = EV_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ev_st_q <= EV_IDLE;
      req_q   <= '0;
      raw_timeline_a_q <= '0;
      raw_timeline_b_q <= '0;
      raw_cache_a_q <= '0;
      raw_cache_b_q <= '0;
      shape_s1a_q <= '0;
      shape_s3a_q <= '0;
      shape_s1b_q <= '0;
      shape_s3b_q <= '0;
      m_s2_a_q <= '0;
      m_s4_a_q <= '0;
      m_s2_b_q <= '0;
      m_s4_b_q <= '0;
      skip_s2_a_q <= 1'b0;
      skip_s4_a_q <= 1'b0;
      skip_s2_b_q <= 1'b0;
      skip_s4_b_q <= 1'b0;
      dma_s1_a_q <= '0;
      dma_s3_a_q <= '0;
      dma_s1_b_q <= '0;
      dma_s3_b_q <= '0;
      result_valid_q <= 1'b0;
    end else begin
      ev_st_q <= ev_st_d;
      req_q   <= req_d;
      raw_timeline_a_q <= raw_timeline_a_d;
      raw_timeline_b_q <= raw_timeline_b_d;
      raw_cache_a_q <= raw_cache_a_d;
      raw_cache_b_q <= raw_cache_b_d;
      shape_s1a_q <= shape_s1a_d;
      shape_s3a_q <= shape_s3a_d;
      shape_s1b_q <= shape_s1b_d;
      shape_s3b_q <= shape_s3b_d;
      m_s2_a_q <= m_s2_a_d;
      m_s4_a_q <= m_s4_a_d;
      m_s2_b_q <= m_s2_b_d;
      m_s4_b_q <= m_s4_b_d;
      skip_s2_a_q <= skip_s2_a_d;
      skip_s4_a_q <= skip_s4_a_d;
      skip_s2_b_q <= skip_s2_b_d;
      skip_s4_b_q <= skip_s4_b_d;
      dma_s1_a_q <= dma_s1_a_d;
      dma_s3_a_q <= dma_s3_a_d;
      dma_s1_b_q <= dma_s1_b_d;
      dma_s3_b_q <= dma_s3_b_d;
      result_valid_q <= result_valid_d;
    end
  end

  assign busy_o = (ev_st_q != EV_IDLE) && (ev_st_q != EV_DONE);
  assign done_o = (ev_st_q == EV_DONE);

  always_comb begin
    snap_timeline_a_o = post_s2pf_a_timeline;
    snap_timeline_b_o = post_s2pf_b_timeline;
    snap_cache_a_o    = raw_cache_a_q;
    snap_cache_b_o    = raw_cache_b_q;

    task_end_a_o  = post_s2pf_a_timeline.task_end;
    task_end_b_o  = post_s2pf_b_timeline.task_end;
    s2_end_a_o    = post_s2pf_a_timeline.s2_end;
    s2_end_b_o    = post_s2pf_b_timeline.s2_end;
    s4_start_a_o  = post_s2pf_a_timeline.s4_start;
    s4_start_b_o  = post_s2pf_b_timeline.s4_start;

    makespan_o = (post_s2pf_a_timeline.task_end > post_s2pf_b_timeline.task_end) ?
                 post_s2pf_a_timeline.task_end : post_s2pf_b_timeline.task_end;

    eval_valid_o = (ev_st_q == EV_DONE) && result_valid_q;

    score_key_o.cost     = req_q.score_makespan_only ? makespan_o : cont_cost;
    score_key_o.rem_len  = req_q.rem_len_after;
    score_key_o.snap_max = makespan_o;
    score_key_o.snap_min = req_q.cost_only_tie ? '0 :
                           ((post_s2pf_a_timeline.task_end < post_s2pf_b_timeline.task_end) ?
                            post_s2pf_a_timeline.task_end : post_s2pf_b_timeline.task_end);

    plan_desc_o = '0;
    plan_desc_o.plan_type   = req_q.plan_type;
    plan_desc_o.cluster_a   = req_q.cluster_a;
    plan_desc_o.eid_a       = req_q.eid_a;
    plan_desc_o.ntok_a      = req_q.ntok_a;
    plan_desc_o.tok_start_a = req_q.tok_start_a;
    plan_desc_o.s1a         = shape_s1a_o;
    plan_desc_o.s3a         = shape_s3a_o;
    // Match C plan_from_snap(): skip bits come from the final post-S2PF snap.
    // When S2PF succeeds, bw_s3 is cleared, so skip_s3=1 and has_s2pf=1.
    plan_desc_o.skip_s1_a   = (post_s2pf_a_timeline.bw_s1 == BW_0);
    plan_desc_o.skip_s3_a   = (post_s2pf_a_timeline.bw_s3 == BW_0);
    plan_desc_o.eid_b       = req_q.eid_b;
    plan_desc_o.ntok_b      = req_q.ntok_b;
    plan_desc_o.tok_start_b = req_q.tok_start_b;
    plan_desc_o.s1b         = shape_s1b_o;
    plan_desc_o.s3b         = shape_s3b_o;
    plan_desc_o.skip_s1_b   = (post_s2pf_b_timeline.bw_s1 == BW_0);
    plan_desc_o.skip_s3_b   = (post_s2pf_b_timeline.bw_s3 == BW_0);
    plan_desc_o.has_s2pf_a  = post_s2pf_a_timeline.s2pf_valid;
    plan_desc_o.has_s2pf_b  = post_s2pf_b_timeline.s2pf_valid;
  end

  assign bw_s1a_o = raw_timeline_a_q.bw_s1;
  assign bw_s3a_o = raw_timeline_a_q.bw_s3;
  assign bw_s1b_o = raw_timeline_b_q.bw_s1;
  assign bw_s3b_o = raw_timeline_b_q.bw_s3;
  assign m_s2_a_o = m_s2_a_q;
  assign m_s4_a_o = m_s4_a_q;
  assign m_s2_b_o = m_s2_b_q;
  assign m_s4_b_o = m_s4_b_q;
  assign skip_s2_a_o = skip_s2_a_q;
  assign skip_s4_a_o = skip_s4_a_q;
  assign skip_s2_b_o = skip_s2_b_q;
  assign skip_s4_b_o = skip_s4_b_q;
  assign dma_s1_a_o = dma_s1_a_q;
  assign dma_s3_a_o = dma_s3_a_q;
  assign dma_s1_b_o = dma_s1_b_q;
  assign dma_s3_b_o = dma_s3_b_q;

endmodule
