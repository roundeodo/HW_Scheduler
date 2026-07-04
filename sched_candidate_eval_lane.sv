// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — 1-lane candidate evaluator（Tick 域版本）
//
// 一个 lane 每次评估一个 candidate token：
//   eval_req_q -> shape_q -> shared mk_timeline(A) -> raw_task_a_q
//              -> shared mk_timeline(B) -> raw_task_b_q
//              -> lite S2PF policy -> bw_ok -> continuation_cost -> score_key/winner_plan
//
// 本模块是带 start/done 握手的单候选 evaluator。start_i 到来后先把
// candidate token + round context 解码并裁剪成 eval_req_q，再显式经过
// EV_PICK/EV_MK_A/EV_MK_B/S2PF/SCORE。generator 不再输出完整候选 payload。
// 当 continuation_cost 进入 rem_len==1 的 sim1 FSM 时，本 lane 会多拍完成。

import sched_pkg::*;
import sched_candidate_pkg::*;

module sched_candidate_eval_lane (
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic               start_i,
  output logic               busy_o,
  output logic               done_o,

  input  cand_token_t        cand_i,

  // ── 候选控制 ─────────────────────────────────────────────────────────────
  input  head_ctx_t [3:0]    head_i,
  input  logic [NR_W-1:0]    active_count_i,
  input  logic [T_W-1:0]     total_conc_i,
  input  early_start_ctx_t   early_i,

  // ── 当前 C2/C3 base snap；某侧没有新 task 时直接沿用 base snap ─────────
  input  snap_timeline_t     base_timeline_a_i,
  input  snap_timeline_t     base_timeline_b_i,
  input  snap_cache_t        base_cache_a_i,
  input  snap_cache_t        base_cache_b_i,

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
  output winner_plan_t       winner_plan_o,
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
  output logic [BW_W-1:0]    bw_s3b_o
);

  // ── shared mk_timeline scalar wires ────────────────────────────────────────
  logic [T_W-1:0] mk_task_start, mk_task_end, mk_dma1_end;
  logic [T_W-1:0] mk_s2_end, mk_dma3_end, mk_s4_start;
  logic [BW_W-1:0] mk_bw_s1, mk_bw_s3;

  logic [T_W-1:0] mk_start_t;
  logic [NTOK_W-1:0] mk_ntok;
  logic [1:0] mk_shape_s1, mk_shape_s3;
  logic mk_skip_s1, mk_skip_s3;
  logic mk_side_b;

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
    ntok_t                ntok;
  } task_timeline_t;

  task_timeline_t raw_task_a_q, raw_task_a_d;
  task_timeline_t raw_task_b_q, raw_task_b_d;
  snap_timeline_t raw_timeline_a;
  snap_timeline_t raw_timeline_b;
  snap_timeline_t post_s2pf_a_timeline;
  snap_timeline_t post_s2pf_b_timeline;
  snap_cache_t    score_cache_a;
  snap_cache_t    score_cache_b;
  logic [T_W-1:0] cont_cost;
  logic s2pf_start;
  logic s2pf_busy;
  logic s2pf_done;
  logic s2pf_bw_start;
  snap_timeline_t s2pf_bw_timeline_a;
  snap_timeline_t s2pf_bw_timeline_b;
  s2pf_patch_t s2pf_patch;
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
  logic       force_shape_a;
  logic       force_shape_b;
  logic [1:0] forced_s1a;
  logic [1:0] forced_s3a;
  logic [1:0] forced_s1b;
  logic [1:0] forced_s3b;
  logic [1:0] shape_s1a_q, shape_s1a_d;
  logic [1:0] shape_s3a_q, shape_s3a_d;
  logic [1:0] shape_s1b_q, shape_s1b_d;
  logic [1:0] shape_s3b_q, shape_s3b_d;

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
  // 只锁存本 lane 后续 pick/mk/s2pf/score/winner_plan 真正读取的字段；
  // 不锁存 generator 旧式完整 payload，也不锁存 base snap，避免无意义 FF 和结构体大扇出。
  typedef struct packed {
    logic                  valid;
    cand_token_t           token;

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
  } eval_req_t;

  eval_req_t req_q, req_d;

  time_t token_tnow;
  time_t token_idle_t;
  time_t token_tpts [3];
  logic token_both_idle;
  logic token_idle_is_c3;
  logic [1:0] token_ntpts;

  assign token_tnow = (base_timeline_a_i.task_end > base_timeline_b_i.task_end) ?
                      base_timeline_a_i.task_end : base_timeline_b_i.task_end;
  assign token_both_idle = (base_timeline_a_i.task_end == base_timeline_b_i.task_end);
  assign token_idle_is_c3 = (base_timeline_b_i.task_end < base_timeline_a_i.task_end);
  assign token_idle_t = token_idle_is_c3 ? base_timeline_b_i.task_end :
                                           base_timeline_a_i.task_end;
  assign token_tpts[0] = token_idle_t;
  assign token_tpts[1] = early_i.t0;
  assign token_tpts[2] = early_i.t1;
  assign token_ntpts = 2'(1 + early_i.count);

  function automatic snap_timeline_t task_to_snap(input task_timeline_t t);
    snap_timeline_t s;
    begin
      s = '0;
      s.valid      = t.valid;
      s.task_start = t.task_start;
      s.task_end   = t.task_end;
      s.dma1_end   = t.dma1_end;
      s.s2_end     = t.s2_end;
      s.dma3_end   = t.dma3_end;
      s.s4_start   = t.s4_start;
      s.bw_s1      = t.bw_s1;
      s.bw_s3      = t.bw_s3;
      s.ntok       = t.ntok;
      task_to_snap = s;
    end
  endfunction

  function automatic snap_cache_t empty_cache();
    snap_cache_t c;
    begin
      c = '0;
      c.pf_eid = PF_EID_NONE;
      empty_cache = c;
    end
  endfunction

  function automatic s2pf_policy_t token_s2pf_policy(input cand_token_t token);
    unique case (token.mode)
      CAND_MODE_SINGLE: begin
        token_s2pf_policy = (token.id == CAND_ID_W'(SINGLE_SPLIT_ID)) ?
                            S2PF_SPLIT_LITE : S2PF_OFF;
      end
      CAND_MODE_BOTH: begin
        token_s2pf_policy = (token.id <= CAND_ID_W'(2)) ?
                            S2PF_PAIR_LITE : S2PF_SPLIT_LITE;
      end
      CAND_MODE_NOT_BOTH: token_s2pf_policy = S2PF_SINGLE_LATEST;
      default:            token_s2pf_policy = S2PF_OFF;
    endcase
  endfunction

  function automatic logic token_score_makespan_only(input cand_token_t token);
    token_score_makespan_only = (token.mode == CAND_MODE_NOT_BOTH);
  endfunction

  task automatic project_rem_after(
    input  logic [3:0] remove_mask,
    output logic [NR_W-1:0]      rem_len_after,
    output logic [EID_RAW_W-1:0] rem0_eid,
    output ntok_t                rem0_ntok,
    output ntok_t                rem1_ntok,
    output time_t                total_conc_after,
    output time_t                max_conc_after
  );
    time_t bc0;
    time_t bc1;
    time_t bc2;
    time_t bc3;
    time_t removed_conc;
    begin
      bc0 = best_conc_ticks(head_i[0].ntok);
      bc1 = best_conc_ticks(head_i[1].ntok);
      bc2 = best_conc_ticks(head_i[2].ntok);
      bc3 = best_conc_ticks(head_i[3].ntok);

      rem0_eid = '0;
      rem0_ntok = '0;
      rem1_ntok = '0;
      max_conc_after = '0;
      removed_conc = '0;

      unique case (remove_mask)
        4'b0001: begin
          removed_conc = bc0;
          rem0_eid = head_i[1].eid;
          rem0_ntok = head_i[1].ntok;
          rem1_ntok = head_i[2].ntok;
          max_conc_after = bc1;
        end
        4'b0011: begin
          removed_conc = bc0 + bc1;
          rem0_eid = head_i[2].eid;
          rem0_ntok = head_i[2].ntok;
          rem1_ntok = head_i[3].ntok;
          max_conc_after = bc2;
        end
        4'b0110: begin
          removed_conc = bc1 + bc2;
          rem0_eid = head_i[0].eid;
          rem0_ntok = head_i[0].ntok;
          rem1_ntok = head_i[3].ntok;
          max_conc_after = bc0;
        end
        4'b1100: begin
          removed_conc = bc2 + bc3;
          rem0_eid = head_i[0].eid;
          rem0_ntok = head_i[0].ntok;
          rem1_ntok = head_i[1].ntok;
          max_conc_after = bc0;
        end
        default: begin
          removed_conc = '0;
        end
      endcase

      unique case (remove_mask)
        4'b0011, 4'b0110, 4'b1100: rem_len_after = active_count_i - NR_W'(2);
        4'b0001:                   rem_len_after = active_count_i - NR_W'(1);
        default:                   rem_len_after = active_count_i;
      endcase
      total_conc_after = total_conc_i - removed_conc;
    end
  endtask

  logic [3:0]          score_remove_mask;
  logic [NR_W-1:0]     score_rem_len_after;
  logic [EID_RAW_W-1:0] score_rem0_eid;
  ntok_t               score_rem0_ntok;
  ntok_t               score_rem1_ntok;
  time_t               score_total_conc_after;
  time_t               score_max_conc_after;

  always_comb begin
    score_remove_mask = cand_remove_mask(req_q.token);
    project_rem_after(score_remove_mask, score_rem_len_after, score_rem0_eid,
                      score_rem0_ntok, score_rem1_ntok,
                      score_total_conc_after, score_max_conc_after);
  end

  task automatic decode_token(
    input  cand_token_t token,
    output eval_req_t   req
  );
    logic [3:0] solo_slot;
    logic       solo_to_c3;
    logic [1:0] early_tpt_idx;
    ntok_t      cut;
    logic [1:0] slot_a;
    logic [1:0] slot_b;
    begin
      req = '0;
      req.valid = token.valid;
      req.token = token;

      unique case (token.mode)
        CAND_MODE_SINGLE: begin
          solo_to_c3 = (token.id >= CAND_ID_W'(SINGLE_SOLO_SHAPES)) &&
                       (token.id < CAND_ID_W'(SINGLE_SOLO_COUNT));
          solo_slot = solo_to_c3 ?
                      4'(token.id - CAND_ID_W'(SINGLE_SOLO_SHAPES)) :
                      4'(token.id);
          early_tpt_idx = 2'(token.id - CAND_ID_W'(SINGLE_EARLY0_ID)) + 2'd1;
          cut = '0;

          if (token.id < CAND_ID_W'(SINGLE_SOLO_COUNT)) begin
            if (!solo_to_c3) begin
              req.side_a_valid = head_i[0].valid;
              req.start_a = base_timeline_a_i.task_end;
              req.eid_a = head_i[0].eid;
              req.ntok_a = head_i[0].ntok;
            end else begin
              req.side_b_valid = head_i[0].valid;
              req.start_b = base_timeline_b_i.task_end;
              req.eid_b = head_i[0].eid;
              req.ntok_b = head_i[0].ntok;
            end
          end else if (token.id == CAND_ID_W'(SINGLE_SPLIT_ID)) begin
            cut = cand_single_split_cut(head_i[0].ntok);
            req.side_a_valid = cand_single_split_valid(head_i[0].ntok);
            req.side_b_valid = req.side_a_valid;
            req.start_a = token_tnow;
            req.start_b = token_tnow;
            req.eid_a = head_i[0].eid;
            req.eid_b = head_i[0].eid;
            req.ntok_a = cut;
            req.ntok_b = head_i[0].ntok - cut;
            req.tok_start_b = cut;
          end else begin
            req.valid = token.valid && !token_both_idle && head_i[0].valid &&
                        (early_tpt_idx < token_ntpts);
            if (!token_idle_is_c3) begin
              req.side_a_valid = req.valid;
              req.start_a = token_tpts[early_tpt_idx];
              req.eid_a = head_i[0].eid;
              req.ntok_a = head_i[0].ntok;
            end else begin
              req.side_b_valid = req.valid;
              req.start_b = token_tpts[early_tpt_idx];
              req.eid_b = head_i[0].eid;
              req.ntok_b = head_i[0].ntok;
            end
          end
        end

        CAND_MODE_BOTH: begin
          req.start_a = token_tnow;
          req.start_b = token_tnow;
          if (token.id <= CAND_ID_W'(2)) begin
            unique case (token.id)
              CAND_ID_W'(0): begin
                slot_a = 2'd0;
                slot_b = 2'd1;
              end
              CAND_ID_W'(1): begin
                slot_a = 2'd1;
                slot_b = 2'd2;
              end
              default: begin
                slot_a = 2'd2;
                slot_b = 2'd3;
              end
            endcase
            req.side_a_valid = token.valid && head_i[slot_a].valid && head_i[slot_b].valid;
            req.side_b_valid = req.side_a_valid;
            req.eid_a = head_i[slot_a].eid;
            req.eid_b = head_i[slot_b].eid;
            req.ntok_a = head_i[slot_a].ntok;
            req.ntok_b = head_i[slot_b].ntok;
          end else begin
            cut = cand_both_split_cut(head_i[0].ntok, token.id);
            req.side_a_valid = cand_both_split_valid(head_i[0].ntok, token.id) &&
                               (cut > '0) && (cut < head_i[0].ntok);
            req.side_b_valid = req.side_a_valid;
            req.eid_a = head_i[0].eid;
            req.eid_b = head_i[0].eid;
            req.ntok_a = cut;
            req.ntok_b = head_i[0].ntok - cut;
            req.tok_start_b = cut;
          end
        end

        CAND_MODE_NOT_BOTH: begin
          req.valid = token.valid && head_i[0].valid && (token.id < CAND_ID_W'(token_ntpts));
          if (!token_idle_is_c3) begin
            req.side_a_valid = req.valid;
            req.start_a = token_tpts[token.id[1:0]];
            req.eid_a = head_i[0].eid;
            req.ntok_a = head_i[0].ntok;
          end else begin
            req.side_b_valid = req.valid;
            req.start_b = token_tpts[token.id[1:0]];
            req.eid_b = head_i[0].eid;
            req.ntok_b = head_i[0].ntok;
          end
        end

        default: begin
          req.valid = 1'b0;
        end
      endcase

      req.valid = req.valid && (req.side_a_valid || req.side_b_valid);
      req.sw_a = req.side_a_valid &&
                 swiglu_hit_t(req.eid_a, base_cache_a_i.pf_eid,
                              base_cache_a_i.pf_end, req.start_a);
      req.dn_a = req.side_a_valid &&
                 down_hit_t(req.eid_a, base_cache_a_i.pf_eid,
                            base_cache_a_i.pf_end, base_cache_a_i.pf_full,
                            req.start_a);
      req.sw_b = req.side_b_valid &&
                 swiglu_hit_t(req.eid_b, base_cache_b_i.pf_eid,
                              base_cache_b_i.pf_end, req.start_b);
      req.dn_b = req.side_b_valid &&
                 down_hit_t(req.eid_b, base_cache_b_i.pf_eid,
                            base_cache_b_i.pf_end, base_cache_b_i.pf_full,
                            req.start_b);
    end
  endtask

  task automatic decode_shape_override(
    input  cand_token_t token,
    output logic        force_a,
    output shape_t      s1a,
    output shape_t      s3a,
    output logic        force_b,
    output shape_t      s1b,
    output shape_t      s3b
  );
    logic [3:0] solo_slot;
    logic       solo_to_c3;
    begin
      force_a = 1'b0;
      force_b = 1'b0;
      s1a = '0;
      s3a = '0;
      s1b = '0;
      s3b = '0;

      unique case (token.mode)
        CAND_MODE_SINGLE: begin
          solo_to_c3 = (token.id >= CAND_ID_W'(SINGLE_SOLO_SHAPES)) &&
                       (token.id < CAND_ID_W'(SINGLE_SOLO_COUNT));
          solo_slot = solo_to_c3 ?
                      4'(token.id - CAND_ID_W'(SINGLE_SOLO_SHAPES)) :
                      4'(token.id);
          if (token.id < CAND_ID_W'(SINGLE_SOLO_COUNT)) begin
            if (solo_to_c3) begin
              force_b = 1'b1;
              s1b = cand_single_solo_s1(solo_slot);
              s3b = cand_single_solo_s3(solo_slot);
            end else begin
              force_a = 1'b1;
              s1a = cand_single_solo_s1(solo_slot);
              s3a = cand_single_solo_s3(solo_slot);
            end
          end else if (token.id > CAND_ID_W'(SINGLE_SPLIT_ID)) begin
            if (token_idle_is_c3) begin
              force_b = 1'b1;
              s1b = SHAPE_C;
              s3b = SHAPE_C;
            end else begin
              force_a = 1'b1;
              s1a = SHAPE_C;
              s3a = SHAPE_C;
            end
          end
        end

        CAND_MODE_NOT_BOTH: begin
          if (token_idle_is_c3) begin
            force_b = 1'b1;
            s1b = SHAPE_C;
            s3b = SHAPE_C;
          end else begin
            force_a = 1'b1;
            s1a = SHAPE_C;
            s3a = SHAPE_C;
          end
        end

        default: begin
        end
      endcase
    end
  endtask

  task automatic put_plan_slot(
    inout  winner_plan_t          p,
    input  logic                  dst,
    input  logic                  cluster,
    input  logic [EID_RAW_W-1:0]  eid,
    input  ntok_t                 ntok,
    input  ntok_t                 tok_start,
    input  shape_t                s1,
    input  shape_t                s3,
    input  snap_timeline_t        sn
  );
    begin
      p.token[dst].eid       = eid;
      p.token[dst].ntok      = ntok;
      p.token[dst].tok_start = tok_start;
      p.ctrl[dst].cluster    = cluster;
      p.ctrl[dst].s1         = s1;
      p.ctrl[dst].s3         = s3;
      p.ctrl[dst].skip_s1    = (sn.bw_s1 == BW_0);
      p.ctrl[dst].skip_s3    = (sn.bw_s3 == BW_0);
      p.ctrl[dst].has_s2pf   = sn.s2pf_valid;
    end
  endtask

  // ── pick_shapes 实例 ─────────────────────────────────────────────────────
  sched_pick_shapes i_pick_shapes (
    .ntok_a_i (req_q.ntok_a),
    .ntok_b_i (req_q.ntok_b),
    .sw_a_i   (req_q.sw_a),
    .dn_a_i   (req_q.dn_a),
    .sw_b_i   (req_q.sw_b),
    .dn_b_i   (req_q.dn_b),
    .s1a_o    (pick_s1a),
    .s3a_o    (pick_s3a),
    .s1b_o    (pick_s1b),
    .s3b_o    (pick_s3b)
  );

  assign shape_s1a_o = shape_s1a_q;
  assign shape_s3a_o = shape_s3a_q;
  assign shape_s1b_o = shape_s1b_q;
  assign shape_s3b_o = shape_s3b_q;

  always_comb begin
    decode_shape_override(req_q.token, force_shape_a, forced_s1a, forced_s3a,
                          force_shape_b, forced_s1b, forced_s3b);
  end

  // ── shared mk_timeline：EV_MK_A/EV_MK_B 两拍复用同一套组合逻辑 ─────────────
  assign mk_side_b   = (ev_st_q == EV_MK_B);
  assign mk_start_t  = mk_side_b ? req_q.start_b : req_q.start_a;
  assign mk_ntok     = mk_side_b ? req_q.ntok_b  : req_q.ntok_a;
  assign mk_shape_s1 = mk_side_b ? shape_s1b_q   : shape_s1a_q;
  assign mk_shape_s3 = mk_side_b ? shape_s3b_q   : shape_s3a_q;
  assign mk_skip_s1  = mk_side_b ? req_q.sw_b    : req_q.sw_a;
  assign mk_skip_s3  = mk_side_b ? req_q.dn_b    : req_q.dn_a;

  sched_mk_timeline i_mk_timeline (
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
    .bw_s3_o       (mk_bw_s3)
  );

  sched_s2pf_pair i_s2pf_pair (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .start_i              (s2pf_start),
    .busy_o               (s2pf_busy),
    .done_o               (s2pf_done),
    .policy_i             (token_s2pf_policy(req_q.token)),
    .side_a_active_i      (req_q.side_a_valid),
    .side_b_active_i      (req_q.side_b_valid),
    .shape_s3_a_i         (shape_s3a_q),
    .shape_s3_b_i         (shape_s3b_q),
    .snap_a_i             (raw_timeline_a),
    .snap_b_i             (raw_timeline_b),
    .bw_start_o           (s2pf_bw_start),
    .bw_snap_a_o          (s2pf_bw_timeline_a),
    .bw_snap_b_o          (s2pf_bw_timeline_b),
    .bw_done_i            ((ev_st_q == EV_WAIT_S2PF) ? bw_done_i : 1'b0),
    .bw_ok_i              (bw_ok_i),
    .patch_o              (s2pf_patch)
  );

  assign bw_ok_o = s2pf_patch.ok;
  assign raw_timeline_a = req_q.side_a_valid ? task_to_snap(raw_task_a_q) :
                                               base_timeline_a_i;
  assign raw_timeline_b = req_q.side_b_valid ? task_to_snap(raw_task_b_q) :
                                               base_timeline_b_i;
  assign score_cache_a = req_q.side_a_valid ? empty_cache() : base_cache_a_i;
  assign score_cache_b = req_q.side_b_valid ? empty_cache() : base_cache_b_i;
  assign post_s2pf_a_timeline = apply_s2pf_patch_timeline(
      raw_timeline_a, s2pf_patch.has_a, s2pf_patch.pf_start_a,
      s2pf_patch.pf_end_a, s2pf_patch.task_end_a, shape_s3a_q);
  assign post_s2pf_b_timeline = apply_s2pf_patch_timeline(
      raw_timeline_b, s2pf_patch.has_b, s2pf_patch.pf_start_b,
      s2pf_patch.pf_end_b, s2pf_patch.task_end_b, shape_s3b_q);

  assign eval_candidate_ok = req_q.valid && s2pf_patch.ok &&
                             (req_q.side_a_valid || req_q.side_b_valid);

  sched_score_unit i_score (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .start_i            (score_start),
    .busy_o             (score_busy),
    .done_o             (score_done),
    .c2_task_end_i      (post_s2pf_a_timeline.task_end),
    .c3_task_end_i      (post_s2pf_b_timeline.task_end),
    .c2_cache_i         (score_cache_a),
    .c3_cache_i         (score_cache_b),
    .rem_len_i          (score_rem_len_after),
    .rem0_eid_i         (score_rem0_eid),
    .rem0_ntok_i        (score_rem0_ntok),
    .rem1_ntok_i        (score_rem1_ntok),
    .total_conc_i       (score_total_conc_after),
    .max_conc_i         (score_max_conc_after),
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
    raw_task_a_d = raw_task_a_q;
    raw_task_b_d = raw_task_b_q;
    shape_s1a_d = shape_s1a_q;
    shape_s3a_d = shape_s3a_q;
    shape_s1b_d = shape_s1b_q;
    shape_s3b_d = shape_s3b_q;
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
          decode_token(cand_i, req_d);

          if (req_d.valid) begin
            ev_st_d = EV_PICK;
          end else begin
            result_valid_d = 1'b0;
            ev_st_d = EV_DONE;
          end
      end

      EV_PICK: begin
        shape_s1a_d = force_shape_a ? forced_s1a : pick_s1a;
        shape_s3a_d = force_shape_a ? forced_s3a : pick_s3a;
        shape_s1b_d = force_shape_b ? forced_s1b : pick_s1b;
        shape_s3b_d = force_shape_b ? forced_s3b : pick_s3b;
        ev_st_d = EV_MK_A;
      end

      EV_MK_A: begin
        raw_task_a_d = '0;
        if (req_q.side_a_valid) begin
          raw_task_a_d.valid      = 1'b1;
          raw_task_a_d.task_start = mk_task_start;
          raw_task_a_d.task_end   = mk_task_end;
          raw_task_a_d.dma1_end   = mk_dma1_end;
          raw_task_a_d.s2_end     = mk_s2_end;
          raw_task_a_d.dma3_end   = mk_dma3_end;
          raw_task_a_d.s4_start   = mk_s4_start;
          raw_task_a_d.bw_s1      = mk_bw_s1;
          raw_task_a_d.bw_s3      = mk_bw_s3;
          raw_task_a_d.ntok       = req_q.ntok_a;
        end
        ev_st_d = EV_MK_B;
      end

      EV_MK_B: begin
        raw_task_b_d = '0;
        if (req_q.side_b_valid) begin
          raw_task_b_d.valid      = 1'b1;
          raw_task_b_d.task_start = mk_task_start;
          raw_task_b_d.task_end   = mk_task_end;
          raw_task_b_d.dma1_end   = mk_dma1_end;
          raw_task_b_d.s2_end     = mk_s2_end;
          raw_task_b_d.dma3_end   = mk_dma3_end;
          raw_task_b_d.s4_start   = mk_s4_start;
          raw_task_b_d.bw_s1      = mk_bw_s1;
          raw_task_b_d.bw_s3      = mk_bw_s3;
          raw_task_b_d.ntok       = req_q.ntok_b;
        end
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
      raw_task_a_q <= '0;
      raw_task_b_q <= '0;
      shape_s1a_q <= '0;
      shape_s3a_q <= '0;
      shape_s1b_q <= '0;
      shape_s3b_q <= '0;
      result_valid_q <= 1'b0;
    end else begin
      ev_st_q <= ev_st_d;
      req_q   <= req_d;
      raw_task_a_q <= raw_task_a_d;
      raw_task_b_q <= raw_task_b_d;
      shape_s1a_q <= shape_s1a_d;
      shape_s3a_q <= shape_s3a_d;
      shape_s1b_q <= shape_s1b_d;
      shape_s3b_q <= shape_s3b_d;
      result_valid_q <= result_valid_d;
    end
  end

  assign busy_o = (ev_st_q != EV_IDLE) && (ev_st_q != EV_DONE);
  assign done_o = (ev_st_q == EV_DONE);

  always_comb begin
    snap_timeline_a_o = post_s2pf_a_timeline;
    snap_timeline_b_o = post_s2pf_b_timeline;
    snap_cache_a_o    = score_cache_a;
    snap_cache_b_o    = score_cache_b;

    task_end_a_o  = post_s2pf_a_timeline.task_end;
    task_end_b_o  = post_s2pf_b_timeline.task_end;
    s2_end_a_o    = post_s2pf_a_timeline.s2_end;
    s2_end_b_o    = post_s2pf_b_timeline.s2_end;
    s4_start_a_o  = post_s2pf_a_timeline.s4_start;
    s4_start_b_o  = post_s2pf_b_timeline.s4_start;

    makespan_o = (post_s2pf_a_timeline.task_end > post_s2pf_b_timeline.task_end) ?
                 post_s2pf_a_timeline.task_end : post_s2pf_b_timeline.task_end;

    eval_valid_o = (ev_st_q == EV_DONE) && result_valid_q;

    score_key_o.cost     = token_score_makespan_only(req_q.token) ? makespan_o : cont_cost;
    score_key_o.rem_len  = score_rem_len_after;
    score_key_o.snap_max = makespan_o;

    winner_plan_o = '0;
    if (req_q.side_a_valid) begin
      put_plan_slot(winner_plan_o, 1'b0, 1'b0, req_q.eid_a, req_q.ntok_a,
                    req_q.tok_start_a, shape_s1a_q, shape_s3a_q,
                    post_s2pf_a_timeline);
    end
    if (req_q.side_b_valid) begin
      put_plan_slot(winner_plan_o, req_q.side_a_valid, 1'b1, req_q.eid_b,
                    req_q.ntok_b, req_q.tok_start_b, shape_s1b_q, shape_s3b_q,
                    post_s2pf_b_timeline);
    end
    winner_plan_o.valid = (req_q.side_a_valid && req_q.side_b_valid) ? 2'b11 :
                          (req_q.side_a_valid || req_q.side_b_valid) ? 2'b01 :
                          2'b00;
  end

  assign bw_s1a_o = raw_timeline_a.bw_s1;
  assign bw_s3a_o = raw_timeline_a.bw_s3;
  assign bw_s1b_o = raw_timeline_b.bw_s1;
  assign bw_s3b_o = raw_timeline_b.bw_s3;

endmodule
