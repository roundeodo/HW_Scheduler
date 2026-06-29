// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — 1-lane candidate evaluator（Tick 域版本）
//
// 一个 lane 每次评估一个已经规范化的候选：
//   pick_shapes/forced_shape -> mk_snap(A/B) -> try_s2pf_pair/latest -> bw_ok
//   -> continuation_cost -> score_key/plan_desc
//
// 本模块是带 start/done 握手的单候选 evaluator。外层
// candidate_generator/best_reduce/commit 决定候选 ID 顺序和寄存器化边界；
// 当 continuation_cost 进入 rem_len==1 的 sim1 FSM 时，本 lane 会多拍完成。

import sched_pkg::*;

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
  input  logic               enable_s2pf_i,
  input  logic               single_latest_s2pf_i, // not_both_idle/latest-only 模式
  input  logic               force_shape_a_i,
  input  logic               force_shape_b_i,
  input  logic [1:0]         forced_s1a_i,
  input  logic [1:0]         forced_s3a_i,
  input  logic [1:0]         forced_s1b_i,
  input  logic [1:0]         forced_s3b_i,
  input  logic               cost_only_tie_i,
  input  logic               score_makespan_only_i,

  // ── 当前 C2/C3 base snap；某侧没有新 task 时直接沿用 base snap ─────────
  input  eval_snap_t         base_snap_a_i,
  input  eval_snap_t         base_snap_b_i,

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

  // ── 评估结果 ─────────────────────────────────────────────────────────────
  output logic               eval_valid_o,
  output logic               bw_ok_o,
  output logic [T_W-1:0]     makespan_o,
  output score_key_t         score_key_o,
  output plan_desc_t         plan_desc_o,
  output eval_snap_t         snap_a_o,
  output eval_snap_t         snap_b_o,

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

  // ── mk_snap scalar wires ────────────────────────────────────────────────
  logic [T_W-1:0] raw_task_start_a, raw_task_end_a, raw_dma1_end_a;
  logic [T_W-1:0] raw_s1_end_a, raw_s2_end_a, raw_dma3_end_a;
  logic [T_W-1:0] raw_s3_end_a, raw_s4_start_a;
  logic [T_W-1:0] raw_task_start_b, raw_task_end_b, raw_dma1_end_b;
  logic [T_W-1:0] raw_s1_end_b, raw_s2_end_b, raw_dma3_end_b;
  logic [T_W-1:0] raw_s3_end_b, raw_s4_start_b;

  eval_snap_t raw_snap_a;
  eval_snap_t raw_snap_b;
  eval_snap_t post_s2pf_a;
  eval_snap_t post_s2pf_b;
  logic [T_W-1:0] cont_cost;
  logic s2pf_start;
  logic s2pf_busy;
  logic s2pf_done;
  logic score_start;
  logic score_busy;
  logic score_done;
  logic eval_candidate_ok;
  logic [1:0] pick_s1a;
  logic [1:0] pick_s3a;
  logic [1:0] pick_s1b;
  logic [1:0] pick_s3b;

  typedef enum logic [1:0] {
    EV_IDLE,
    EV_WAIT_S2PF,
    EV_WAIT_SCORE,
    EV_INVALID_DONE
  } eval_state_t;

  eval_state_t ev_st_q, ev_st_d;

  // ── pick_shapes 实例 ─────────────────────────────────────────────────────
  sched_pick_shapes i_pick_shapes (
    .ntok_a_i (ntok_a_i),
    .ntok_b_i (ntok_b_i),
    .sw_a_i   (sw_a_i),
    .dn_a_i   (dn_a_i),
    .sw_b_i   (sw_b_i),
    .dn_b_i   (dn_b_i),
    .t0_i     (shape_t0_i),
    .s1a_o    (pick_s1a),
    .s3a_o    (pick_s3a),
    .s1b_o    (pick_s1b),
    .s3b_o    (pick_s3b)
  );

  assign shape_s1a_o = force_shape_a_i ? forced_s1a_i : pick_s1a;
  assign shape_s3a_o = force_shape_a_i ? forced_s3a_i : pick_s3a;
  assign shape_s1b_o = force_shape_b_i ? forced_s1b_i : pick_s1b;
  assign shape_s3b_o = force_shape_b_i ? forced_s3b_i : pick_s3b;

  // ── mk_snap A/B ─────────────────────────────────────────────────────────
  sched_mk_snap i_mk_snap_a (
    .start_t_i     (start_a_i),
    .ntok_i        (ntok_a_i),
    .shape_s1_i    (shape_s1a_o),
    .shape_s3_i    (shape_s3a_o),
    .skip_s1_i     (sw_a_i),
    .skip_s3_i     (dn_a_i),
    .task_start_o  (raw_task_start_a),
    .task_end_o    (raw_task_end_a),
    .dma1_end_o    (raw_dma1_end_a),
    .s1_end_o      (raw_s1_end_a),
    .s2_end_o      (raw_s2_end_a),
    .dma3_end_o    (raw_dma3_end_a),
    .s3_end_o      (raw_s3_end_a),
    .s4_start_o    (raw_s4_start_a),
    .bw_s1_o       (bw_s1a_o),
    .bw_s3_o       (bw_s3a_o),
    .m_s2_exec_o   (m_s2_a_o),
    .m_s4_exec_o   (m_s4_a_o),
    .skip_s2_o     (skip_s2_a_o),
    .skip_s4_o     (skip_s4_a_o),
    .dma_s1_o      (dma_s1_a_o),
    .dma_s3_o      (dma_s3_a_o)
  );

  sched_mk_snap i_mk_snap_b (
    .start_t_i     (start_b_i),
    .ntok_i        (ntok_b_i),
    .shape_s1_i    (shape_s1b_o),
    .shape_s3_i    (shape_s3b_o),
    .skip_s1_i     (sw_b_i),
    .skip_s3_i     (dn_b_i),
    .task_start_o  (raw_task_start_b),
    .task_end_o    (raw_task_end_b),
    .dma1_end_o    (raw_dma1_end_b),
    .s1_end_o      (raw_s1_end_b),
    .s2_end_o      (raw_s2_end_b),
    .dma3_end_o    (raw_dma3_end_b),
    .s3_end_o      (raw_s3_end_b),
    .s4_start_o    (raw_s4_start_b),
    .bw_s1_o       (bw_s1b_o),
    .bw_s3_o       (bw_s3b_o),
    .m_s2_exec_o   (m_s2_b_o),
    .m_s4_exec_o   (m_s4_b_o),
    .skip_s2_o     (skip_s2_b_o),
    .skip_s4_o     (skip_s4_b_o),
    .dma_s1_o      (dma_s1_b_o),
    .dma_s3_o      (dma_s3_b_o)
  );

  always_comb begin
    raw_snap_a = base_snap_a_i;
    if (side_a_valid_i) begin
      raw_snap_a              = '0;
      raw_snap_a.valid        = 1'b1;
      raw_snap_a.task_start   = raw_task_start_a;
      raw_snap_a.task_end     = raw_task_end_a;
      raw_snap_a.dma1_end     = raw_dma1_end_a;
      raw_snap_a.s1_end       = raw_s1_end_a;
      raw_snap_a.s2_end       = raw_s2_end_a;
      raw_snap_a.dma3_end     = raw_dma3_end_a;
      raw_snap_a.s3_end       = raw_s3_end_a;
      raw_snap_a.s4_start     = raw_s4_start_a;
      raw_snap_a.bw_s1        = bw_s1a_o;
      raw_snap_a.bw_s3        = bw_s3a_o;
      raw_snap_a.ntok         = ntok_a_i;
      raw_snap_a.pf_eid       = PF_EID_NONE;
    end

    raw_snap_b = base_snap_b_i;
    if (side_b_valid_i) begin
      raw_snap_b              = '0;
      raw_snap_b.valid        = 1'b1;
      raw_snap_b.task_start   = raw_task_start_b;
      raw_snap_b.task_end     = raw_task_end_b;
      raw_snap_b.dma1_end     = raw_dma1_end_b;
      raw_snap_b.s1_end       = raw_s1_end_b;
      raw_snap_b.s2_end       = raw_s2_end_b;
      raw_snap_b.dma3_end     = raw_dma3_end_b;
      raw_snap_b.s3_end       = raw_s3_end_b;
      raw_snap_b.s4_start     = raw_s4_start_b;
      raw_snap_b.bw_s1        = bw_s1b_o;
      raw_snap_b.bw_s3        = bw_s3b_o;
      raw_snap_b.ntok         = ntok_b_i;
      raw_snap_b.pf_eid       = PF_EID_NONE;
    end
  end

  sched_s2pf_pair i_s2pf_pair (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .start_i              (s2pf_start),
    .busy_o               (s2pf_busy),
    .done_o               (s2pf_done),
    .enable_i             (enable_s2pf_i),
    .single_latest_only_i (single_latest_s2pf_i),
    .side_a_active_i      (side_a_valid_i),
    .side_b_active_i      (side_b_valid_i),
    .shape_s3_a_i         (shape_s3a_o),
    .shape_s3_b_i         (shape_s3b_o),
    .snap_a_i             (raw_snap_a),
    .snap_b_i             (raw_snap_b),
    .ok_o                 (bw_ok_o),
    .snap_a_o             (post_s2pf_a),
    .snap_b_o             (post_s2pf_b)
  );

  assign eval_candidate_ok = cand_valid_i && bw_ok_o && (side_a_valid_i || side_b_valid_i);

  sched_score_unit i_score (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .start_i            (score_start),
    .busy_o             (score_busy),
    .done_o             (score_done),
    .c2_snap_i          (post_s2pf_a),
    .c3_snap_i          (post_s2pf_b),
    .rem_len_i          (rem_len_after_i),
    .rem0_eid_i         (rem0_eid_i),
    .rem0_ntok_i        (rem0_ntok_i),
    .rem1_ntok_i        (rem1_ntok_i),
    .total_conc_i       (total_conc_after_i),
    .max_conc_i         (max_conc_after_i),
    .cost_o             (cont_cost)
  );

  always_comb begin
    ev_st_d    = ev_st_q;
    s2pf_start = 1'b0;
    score_start = 1'b0;

    unique case (ev_st_q)
      EV_IDLE: begin
        if (start_i) begin
          if (cand_valid_i && (side_a_valid_i || side_b_valid_i)) begin
            s2pf_start = 1'b1;
            ev_st_d    = EV_WAIT_S2PF;
          end else begin
            ev_st_d = EV_INVALID_DONE;
          end
        end
      end

      EV_WAIT_S2PF: begin
        if (s2pf_done) begin
          if (eval_candidate_ok) begin
            score_start = 1'b1;
            ev_st_d     = EV_WAIT_SCORE;
          end else begin
            ev_st_d = EV_INVALID_DONE;
          end
        end
      end

      EV_WAIT_SCORE: begin
        if (score_done) begin
          ev_st_d = EV_IDLE;
        end
      end

      EV_INVALID_DONE: begin
        ev_st_d = EV_IDLE;
      end

      default: ev_st_d = EV_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ev_st_q <= EV_IDLE;
    end else begin
      ev_st_q <= ev_st_d;
    end
  end

  assign busy_o = (ev_st_q == EV_WAIT_S2PF) || (ev_st_q == EV_WAIT_SCORE) ||
                  s2pf_busy || score_busy;
  assign done_o = ((ev_st_q == EV_WAIT_SCORE) && score_done) ||
                  (ev_st_q == EV_INVALID_DONE);

  always_comb begin
    snap_a_o = post_s2pf_a;
    snap_b_o = post_s2pf_b;

    task_end_a_o  = post_s2pf_a.task_end;
    task_end_b_o  = post_s2pf_b.task_end;
    s2_end_a_o    = post_s2pf_a.s2_end;
    s2_end_b_o    = post_s2pf_b.s2_end;
    s4_start_a_o  = post_s2pf_a.s4_start;
    s4_start_b_o  = post_s2pf_b.s4_start;

    makespan_o = (post_s2pf_a.task_end > post_s2pf_b.task_end) ?
                 post_s2pf_a.task_end : post_s2pf_b.task_end;

    eval_valid_o = (ev_st_q == EV_WAIT_SCORE) && score_done && eval_candidate_ok;

    score_key_o.cost     = score_makespan_only_i ? makespan_o : cont_cost;
    score_key_o.rem_len  = rem_len_after_i;
    score_key_o.snap_max = makespan_o;
    score_key_o.snap_min = cost_only_tie_i ? '0 :
                           ((post_s2pf_a.task_end < post_s2pf_b.task_end) ?
                            post_s2pf_a.task_end : post_s2pf_b.task_end);

    plan_desc_o = '0;
    plan_desc_o.plan_type   = plan_type_i;
    plan_desc_o.cluster_a   = cluster_a_i;
    plan_desc_o.eid_a       = eid_a_i;
    plan_desc_o.ntok_a      = ntok_a_i;
    plan_desc_o.tok_start_a = tok_start_a_i;
    plan_desc_o.s1a         = shape_s1a_o;
    plan_desc_o.s3a         = shape_s3a_o;
    // Match C plan_from_snap(): skip bits come from the final post-S2PF snap.
    // When S2PF succeeds, bw_s3 is cleared, so skip_s3=1 and has_s2pf=1.
    plan_desc_o.skip_s1_a   = (post_s2pf_a.bw_s1 == BW_0);
    plan_desc_o.skip_s3_a   = (post_s2pf_a.bw_s3 == BW_0);
    plan_desc_o.eid_b       = eid_b_i;
    plan_desc_o.ntok_b      = ntok_b_i;
    plan_desc_o.tok_start_b = tok_start_b_i;
    plan_desc_o.s1b         = shape_s1b_o;
    plan_desc_o.s3b         = shape_s3b_o;
    plan_desc_o.skip_s1_b   = (post_s2pf_b.bw_s1 == BW_0);
    plan_desc_o.skip_s3_b   = (post_s2pf_b.bw_s3 == BW_0);
    plan_desc_o.has_s2pf_a  = post_s2pf_a.s2pf_valid;
    plan_desc_o.has_s2pf_b  = post_s2pf_b.s2pf_valid;
  end

endmodule
