// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51

import sched_pkg::*;
import sched_distilled_pkg::*;

module sched_distilled_round_engine (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  logic                         clear_i,
  input  logic                         start_i,
  output logic                         done_o,
  output logic                         feasible_o,
  input  wire head_ctx_t [7:0]         hot_i,
  input  wire head_ctx_t               bottom_i,
  input  wire distilled_cluster_state_t base_c2_i,
  input  wire distilled_cluster_state_t base_c3_i,
  input  wire distilled_counters_t      counters_i,
  output distilled_cluster_state_t      child_c2_o,
  output distilled_cluster_state_t      child_c3_o,
  output distilled_counters_t           child_counters_o,
  output winner_plan_t                  plan_o,
  output logic [1:0]                    remove_count_o,
  output logic [EID_RAW_W-1:0]          remove_eid_a_o,
  output logic [EID_RAW_W-1:0]          remove_eid_b_o,
  output distilled_action_token_t       selected_token_o,
  output distilled_score_record_t       selected_score_o
);

  typedef enum logic [4:0] {
    ST_IDLE,
    ST_PROFILE_SETUP,
    ST_START_WAIT,
    ST_EVAL_WAIT,
    ST_S4_WAIT,
    ST_TARGET_START_WAIT,
    ST_TARGET_EVAL_START,
    ST_TARGET_EVAL_WAIT,
    ST_NEXT_ACTION,
    ST_SCORE_EVAL_START,
    ST_SCORE_EVAL_WAIT,
    ST_SCORE_BYPASS_START,
    ST_BOUND_WAIT,
    ST_COMPARE_WAIT,
    ST_SCORE_NEXT,
    ST_COMMIT_REBUILD_START,
    ST_COMMIT_REBUILD_WAIT,
    ST_DONE
  } state_t;

  (* fsm_encoding = "one_hot" *) state_t st_q;
  state_t st_d;
  distilled_mode_t mode_q, mode_d;
  logic [4:0] profile_addr_q, profile_addr_d;
  logic assignment_swap_q, assignment_swap_d;
  time_t candidate_start_q, candidate_start_d;
  distilled_group_record_t group_local_q, group_local_d;
  distilled_group_record_t group_target_q, group_target_d;
  logic global_valid_q, global_valid_d;
  distilled_replay_token_t global_token_q, global_token_d;
  distilled_score_record_t global_score_q, global_score_d;

  typedef enum logic [1:0] {
    MAT_NONE,
    MAT_LOCAL,
    MAT_TARGET,
    MAT_SCORE
  } materialization_t;
  materialization_t materialization_q, materialization_d;
  logic materialization_global_q, materialization_global_d;

  logic [4:0] profile_limit;
  logic [4:0] decode_address;
  distilled_profile_t decoded_profile;
  distilled_replay_token_t active_token;
  distilled_replay_token_t target_token;
  logic replay_phase;
  logic score_phase;
  logic target_phase;
  logic token_phase;
  head_ctx_t selected_descriptor;
  head_ctx_t selected_descriptor_b;
  logic profile_structural_valid;
  distilled_cluster_state_t iter_own;
  distilled_cluster_state_t iter_peer;

  logic start_iter_begin;
  logic start_iter_valid;
  logic start_iter_done;
  time_t start_iter_value;

  logic eval_start;
  time_t eval_start_time;
  logic eval_assignment_swap;
  logic eval_done;
  logic eval_feasible;
  logic eval_strict_gain;
  logic transition_bw_start;
  snap_bw_view_t transition_bw_c2;
  snap_bw_view_t transition_bw_c3;
  logic target_bw_start;
  snap_bw_view_t target_bw_c2;
  snap_bw_view_t target_bw_c3;
  logic shared_bw_start;
  logic shared_bw_done;
  logic shared_bw_ok;
  logic bw_owner_target_q;
  logic bw_request_target;
  snap_bw_view_t shared_bw_c2;
  snap_bw_view_t shared_bw_c3;
  distilled_cluster_state_t eval_child_c2;
  distilled_cluster_state_t eval_child_c3;
  winner_plan_t eval_plan;
  logic [1:0] eval_remove_count;
  logic [EID_RAW_W-1:0] eval_remove_eid_a;
  logic [EID_RAW_W-1:0] eval_remove_eid_b;
  ntok_t eval_selected_max;
  ntok_t eval_selected_sum;
  logic [1:0] eval_s2pf_count;
  time_t eval_latest_start;

  logic group_candidate_better;
  logic target_compare_phase;
  distilled_group_record_t group_compare_record;
  logic [1:0] group_compare_s4_count;
  time_t eval_max_end;
  logic [T_W:0] eval_sum_end;

  distilled_counters_t eval_child_counters;
  distilled_bound_head_t [4:0] eval_bound_head5;
  head_ctx_t removed_desc_a;
  head_ctx_t removed_desc_b;
  ntok_t removed_blocks_a;
  ntok_t removed_blocks_b;

  logic bound_start;
  logic bound_done;
  distilled_bound_t bound_f;
  distilled_bound_t bound_h;
  distilled_bound_t bound_compute;
  distilled_bound_t bound_dma;
  distilled_score_record_t current_score;
  logic [NR_W-1:0] selector_rank_a;
  logic [NR_W-1:0] selector_rank_b;
  logic [3:0] removed_hot_first;
  logic [3:0] removed_hot_second;

  distilled_regime_t regime;
  logic compare_start;
  logic compare_done;
  logic compare_rhs_wins;
  logic compare_override;
  logic s4_start;
  logic s4_done;
  logic s4_candidate_possible;
  dma_binding_t s4_c2_binding;
  dma_binding_t s4_c3_binding;
  logic [1:0] s4_count;
  logic [1:0] group_target_s4_count;
  logic group_target_wins;
  logic score_materialized;

  function automatic head_ctx_t resolve_selector(
    input distilled_selector_t selector,
    input head_ctx_t [7:0]     hot,
    input head_ctx_t           bottom
  );
    unique case (selector)
      DIST_SEL_T0: resolve_selector = hot[0];
      DIST_SEL_T1: resolve_selector = hot[1];
      DIST_SEL_T2: resolve_selector = hot[2];
      DIST_SEL_T3: resolve_selector = hot[3];
      DIST_SEL_T4: resolve_selector = hot[4];
      DIST_SEL_B0: resolve_selector = bottom;
      default:     resolve_selector = '0;
    endcase
  endfunction

  function automatic logic [NR_W-1:0] selector_rank(
    input distilled_selector_t selector,
    input logic [NR_W-1:0]     count
  );
    unique case (selector)
      DIST_SEL_T0: selector_rank = NR_W'(0);
      DIST_SEL_T1: selector_rank = NR_W'(1);
      DIST_SEL_T2: selector_rank = NR_W'(2);
      DIST_SEL_T3: selector_rank = NR_W'(3);
      DIST_SEL_T4: selector_rank = NR_W'(4);
      DIST_SEL_B0: selector_rank = count - NR_W'(1);
      default:     selector_rank = '1;
    endcase
  endfunction

  assign profile_limit = distilled_mode_profile_limit(mode_q);
  assign score_phase = (st_q == ST_SCORE_EVAL_START) ||
                       (st_q == ST_SCORE_EVAL_WAIT) ||
                       (st_q == ST_SCORE_BYPASS_START) ||
                       (st_q == ST_BOUND_WAIT) ||
                       (st_q == ST_COMPARE_WAIT) ||
                       (st_q == ST_SCORE_NEXT);
  assign replay_phase = score_phase ||
                        (st_q == ST_COMMIT_REBUILD_START) ||
                        (st_q == ST_COMMIT_REBUILD_WAIT) ||
                        (st_q == ST_DONE);
  assign target_phase = (st_q == ST_TARGET_START_WAIT) ||
                        (st_q == ST_TARGET_EVAL_START) ||
                        (st_q == ST_TARGET_EVAL_WAIT);
  assign token_phase = replay_phase || target_phase;
  always_comb begin
    target_token = '0;
    target_token.profile_addr = profile_addr_q;
    target_token.assignment_swap = assignment_swap_q;
    target_token.start = candidate_start_q;
    target_token.targeted_s4pf_c2 = s4_c2_binding;
    target_token.targeted_s4pf_c3 = s4_c3_binding;
  end
  assign active_token = target_phase ? target_token :
      (score_phase ? group_local_q.token : global_token_q);
  assign decode_address = replay_phase ? active_token.profile_addr : profile_addr_q;

  sched_distilled_profile_decode i_profile_decode (
    .profile_addr_i (decode_address),
    .profile_o      (decoded_profile)
  );

  assign selected_descriptor = resolve_selector(
      decoded_profile.selector_a, hot_i, bottom_i);
  assign selected_descriptor_b = resolve_selector(
      decoded_profile.selector_b, hot_i, bottom_i);
  always_comb begin
    profile_structural_valid = 1'b0;
    unique case (decoded_profile.family)
      DIST_FAMILY_SINGLE: begin
        profile_structural_valid = selected_descriptor.valid &&
            (decoded_profile.c2_active ^ decoded_profile.c3_active);
        if (base_c2_i.task_end < base_c3_i.task_end)
          profile_structural_valid &= decoded_profile.c2_active;
        else if (base_c3_i.task_end < base_c2_i.task_end)
          profile_structural_valid &= decoded_profile.c3_active;
        else
          profile_structural_valid &= decoded_profile.c2_active ||
              (decoded_profile.c3_active && (base_c2_i != base_c3_i));
      end
      DIST_FAMILY_PAIR: begin
        profile_structural_valid = selected_descriptor.valid &&
            selected_descriptor_b.valid &&
            (selected_descriptor.eid != selected_descriptor_b.eid) &&
            decoded_profile.c2_active && decoded_profile.c3_active;
      end
      DIST_FAMILY_SPLIT: begin
        profile_structural_valid = selected_descriptor.valid &&
            decoded_profile.c2_active && decoded_profile.c3_active &&
            (selected_descriptor.ntok >= ntok_t'(2));
        if (!decoded_profile.split_balanced)
          profile_structural_valid &= !selected_descriptor.ntok[0];
      end
      default: begin
      end
    endcase
  end
  assign iter_own = decoded_profile.c3_active && !decoded_profile.c2_active ?
                    base_c3_i : base_c2_i;
  assign iter_peer = decoded_profile.c3_active && !decoded_profile.c2_active ?
                     base_c2_i : base_c3_i;

  sched_distilled_start_iter i_start_iter (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .clear_i    (clear_i),
    .begin_i    (start_iter_begin),
    .target_i   ((st_q == ST_S4_WAIT) && s4_done),
    .result_valid_i      (eval_done),
    .result_feasible_i   (eval_feasible),
    .result_strict_gain_i(eval_strict_gain),
    .profile_i  (decoded_profile),
    .eid_i      (selected_descriptor.eid),
    .ntok_i     (selected_descriptor.ntok),
    .force_s1_hit_i (target_phase || ((st_q == ST_S4_WAIT) && s4_done)),
    .own_i      (iter_own),
    .peer_i     (iter_peer),
    .valid_o    (start_iter_valid),
    .done_o     (start_iter_done),
    .start_o    (start_iter_value)
  );

  always_comb begin
    eval_start_time = token_phase ? active_token.start : candidate_start_q;
    eval_assignment_swap = token_phase ? active_token.assignment_swap :
                                         assignment_swap_q;
    if (!token_phase) begin
      if ((st_q == ST_PROFILE_SETUP) &&
          (decoded_profile.family != DIST_FAMILY_SINGLE)) begin
        eval_start_time = (base_c2_i.task_end >= base_c3_i.task_end) ?
                          base_c2_i.task_end : base_c3_i.task_end;
      end else if ((st_q == ST_START_WAIT) && start_iter_valid) begin
        eval_start_time = start_iter_value;
      end
      if ((st_q == ST_NEXT_ACTION) &&
          (decoded_profile.family == DIST_FAMILY_PAIR) &&
          !assignment_swap_q)
        eval_assignment_swap = 1'b1;
    end
  end

  sched_distilled_transition_eval i_transition_eval (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .clear_i              (clear_i),
    .start_i              (eval_start),
    .done_o               (eval_done),
    .feasible_o           (eval_feasible),
    .profile_i            (decoded_profile),
    .assignment_swap_i    (eval_assignment_swap),
    .start_time_i         (eval_start_time),
    .incremental_i        (target_phase),
    .rebuild_only_i       ((st_q == ST_COMMIT_REBUILD_START) ||
                           (st_q == ST_COMMIT_REBUILD_WAIT)),
    .gain_ok_i            (eval_strict_gain),
    .force_s1_hit_c2_i    (token_phase &&
                           (active_token.targeted_s4pf_c2 != DMA_NONE)),
    .force_s1_hit_c3_i    (token_phase &&
                           (active_token.targeted_s4pf_c3 != DMA_NONE)),
    .selected_a_i         (selected_descriptor),
    .selected_b_i         (selected_descriptor_b),
    .base_c2_i            (base_c2_i),
    .base_c3_i            (base_c3_i),
    .bw_start_o           (transition_bw_start),
    .bw_c2_o              (transition_bw_c2),
    .bw_c3_o              (transition_bw_c3),
    .bw_done_i            (shared_bw_done && !bw_owner_target_q),
    .bw_ok_i              (shared_bw_ok),
    .child_c2_o           (eval_child_c2),
    .child_c3_o           (eval_child_c3),
    .plan_o               (eval_plan),
    .remove_count_o       (eval_remove_count),
    .remove_eid_a_o       (eval_remove_eid_a),
    .remove_eid_b_o       (eval_remove_eid_b),
    .selected_max_o       (eval_selected_max),
    .selected_sum_o       (eval_selected_sum),
    .s2pf_count_o         (eval_s2pf_count),
    .latest_start_o       (eval_latest_start)
  );

  assign eval_max_end = (eval_child_c2.task_end >= eval_child_c3.task_end) ?
                        eval_child_c2.task_end : eval_child_c3.task_end;
  assign eval_sum_end = {1'b0, eval_child_c2.task_end} +
                        {1'b0, eval_child_c3.task_end};
  assign eval_strict_gain = group_local_q.valid &&
                            (eval_max_end < group_local_q.max_end);

  assign s4_count = {1'b0, s4_c2_binding != DMA_NONE} +
                    {1'b0, s4_c3_binding != DMA_NONE};
  assign group_target_s4_count =
      {1'b0, group_target_q.token.targeted_s4pf_c2 != DMA_NONE} +
      {1'b0, group_target_q.token.targeted_s4pf_c3 != DMA_NONE};
  assign group_target_wins = group_local_q.valid && group_target_q.valid &&
                             (group_target_q.max_end < group_local_q.max_end);
  assign score_materialized = group_target_wins ?
      (materialization_q == MAT_TARGET) :
      (materialization_q == MAT_LOCAL);
  sched_distilled_target_s4pf i_target_s4pf (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .clear_i           (clear_i),
    .start_i           (s4_start),
    .done_o            (s4_done),
    .candidate_possible_o (s4_candidate_possible),
    .remaining_count_i (counters_i.count),
    .consumer_i        (distilled_s4pf_consumer_view(eval_plan)),
    .base_c2_i         (distilled_s4pf_cluster_view(base_c2_i)),
    .base_c3_i         (distilled_s4pf_cluster_view(base_c3_i)),
    .bw_start_o        (target_bw_start),
    .bw_c2_o           (target_bw_c2),
    .bw_c3_o           (target_bw_c3),
    .bw_done_i         (shared_bw_done && bw_owner_target_q),
    .bw_ok_i           (shared_bw_ok),
    .c2_binding_o      (s4_c2_binding),
    .c3_binding_o      (s4_c3_binding)
  );

  assign shared_bw_start = transition_bw_start || target_bw_start;
  assign bw_request_target = target_bw_start ? 1'b1 :
                             (transition_bw_start ? 1'b0 :
                                                    bw_owner_target_q);
  assign shared_bw_c2 = bw_request_target ? target_bw_c2 : transition_bw_c2;
  assign shared_bw_c3 = bw_request_target ? target_bw_c3 : transition_bw_c3;

  sched_bandwidth_check i_bandwidth_check (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clear_i  (clear_i),
    .start_i  (shared_bw_start),
    .done_o   (shared_bw_done),
    .snap_a_i (shared_bw_c2),
    .snap_b_i (shared_bw_c3),
    .ok_o     (shared_bw_ok)
  );

  // The shared checker is a transaction resource.  Capture its requester at
  // launch so response routing does not depend on a high-fanout outer state bit.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      bw_owner_target_q <= 1'b0;
    else if (clear_i)
      bw_owner_target_q <= 1'b0;
    else if (shared_bw_start)
      bw_owner_target_q <= target_bw_start;
  end

  assign target_compare_phase = (st_q == ST_TARGET_EVAL_WAIT);
  assign group_compare_record = target_compare_phase ?
                                group_target_q : group_local_q;
  assign group_compare_s4_count = target_compare_phase ?
                                  group_target_s4_count : 2'd0;

  // Local and targeted evaluations are mutually exclusive FSM phases.  Select
  // their incumbent first so one physical lexicographic comparator serves both.
  always_comb begin
    group_candidate_better = !group_compare_record.valid;
    if (!group_candidate_better) begin
      if (eval_max_end != group_compare_record.max_end)
        group_candidate_better = eval_max_end < group_compare_record.max_end;
      else if (eval_sum_end != group_compare_record.sum_end)
        group_candidate_better = eval_sum_end < group_compare_record.sum_end;
      else if (eval_latest_start != group_compare_record.token.start)
        group_candidate_better =
            eval_latest_start < group_compare_record.token.start;
      else if (eval_s2pf_count != group_compare_record.s2pf_count)
        group_candidate_better =
            eval_s2pf_count > group_compare_record.s2pf_count;
      else if (target_compare_phase &&
               (s4_count != group_compare_s4_count))
        group_candidate_better = s4_count > group_compare_s4_count;
      else
        group_candidate_better =
            profile_addr_q < group_compare_record.token.profile_addr;
    end
  end

  always_comb begin
    removed_desc_a = selected_descriptor;
    removed_desc_b = selected_descriptor_b;
    removed_blocks_a = ceil_div2_ntok(removed_desc_a.ntok);
    removed_blocks_b = ceil_div2_ntok(removed_desc_b.ntok);

    eval_child_counters = counters_i;
    eval_child_counters.count = counters_i.count - NR_W'(eval_remove_count);
    eval_child_counters.token_sum = counters_i.token_sum -
        DIST_TOKEN_SUM_W'(removed_desc_a.ntok) -
        ((eval_remove_count == 2'd2) ?
         DIST_TOKEN_SUM_W'(removed_desc_b.ntok) : DIST_TOKEN_SUM_W'(0));
    eval_child_counters.odd_count = counters_i.odd_count -
        DIST_HIST_W'(removed_desc_a.ntok[0]) -
        ((eval_remove_count == 2'd2) ?
         DIST_HIST_W'(removed_desc_b.ntok[0]) : DIST_HIST_W'(0));
    eval_child_counters.block_sum = counters_i.block_sum -
        DIST_BLOCK_SUM_W'(removed_blocks_a) -
        ((eval_remove_count == 2'd2) ?
         DIST_BLOCK_SUM_W'(removed_blocks_b) : DIST_BLOCK_SUM_W'(0));
    if (removed_blocks_a >= ntok_t'(1) && removed_blocks_a <= ntok_t'(4))
      eval_child_counters.small_hist[removed_blocks_a-1'b1] =
          eval_child_counters.small_hist[removed_blocks_a-1'b1] - 1'b1;
    if ((eval_remove_count == 2'd2) &&
        (removed_blocks_b >= ntok_t'(1)) &&
        (removed_blocks_b <= ntok_t'(4)))
      eval_child_counters.small_hist[removed_blocks_b-1'b1] =
          eval_child_counters.small_hist[removed_blocks_b-1'b1] - 1'b1;

  end

  sched_distilled_bound_score i_bound_score (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .clear_i           (clear_i),
    .start_i           (bound_start),
    .done_o            (bound_done),
    .child_c2_i        (distilled_bound_cluster_view(eval_child_c2)),
    .child_c3_i        (distilled_bound_cluster_view(eval_child_c3)),
    .child_counters_i  (distilled_bound_counters_view(eval_child_counters)),
    .child_head5_i     (eval_bound_head5),
    .f_o               (bound_f),
    .h_o               (bound_h),
    .compute_bound_o   (bound_compute),
    .dma_bound_o       (bound_dma)
  );

  assign selector_rank_a = selector_rank(decoded_profile.selector_a,
                                           counters_i.count);
  assign selector_rank_b = selector_rank(decoded_profile.selector_b,
                                           counters_i.count);
  always_comb begin
    removed_hot_first = 4'hf;
    removed_hot_second = 4'hf;
    if (selector_rank_a < NR_W'(8))
      removed_hot_first = 4'(selector_rank_a);
    if ((eval_remove_count == 2'd2) &&
        (selector_rank_b < NR_W'(8)) &&
        (selector_rank_b != selector_rank_a)) begin
      if ((removed_hot_first == 4'hf) ||
          (4'(selector_rank_b) < removed_hot_first)) begin
        removed_hot_second = removed_hot_first;
        removed_hot_first = 4'(selector_rank_b);
      end else begin
        removed_hot_second = 4'(selector_rank_b);
      end
    end

    for (int slot = 0; slot < 5; slot++) begin
      eval_bound_head5[slot] = '0;
      if (removed_hot_first > 4'(slot)) begin
        eval_bound_head5[slot].valid = hot_i[slot].valid;
        eval_bound_head5[slot].ntok = hot_i[slot].ntok;
      end else if (removed_hot_second > 4'(slot+1)) begin
        eval_bound_head5[slot].valid = hot_i[slot+1].valid;
        eval_bound_head5[slot].ntok = hot_i[slot+1].ntok;
      end else begin
        eval_bound_head5[slot].valid = hot_i[slot+2].valid;
        eval_bound_head5[slot].ntok = hot_i[slot+2].ntok;
      end
    end
    current_score = '0;
    current_score.f = bound_f;
    current_score.h = bound_h;
    current_score.compute_bound = bound_compute;
    current_score.dma_bound = bound_dma;
    if (eval_child_c2.task_end <= eval_child_c3.task_end) begin
      current_score.early_end = eval_child_c2.task_end;
      current_score.late_end = eval_child_c3.task_end;
    end else begin
      current_score.early_end = eval_child_c3.task_end;
      current_score.late_end = eval_child_c2.task_end;
    end
    current_score.selected_max = eval_selected_max;
    current_score.selected_sum = (NTOK_W+1)'(eval_selected_sum);
    current_score.s2pf_count = eval_s2pf_count;
    current_score.remaining_count = eval_child_counters.count;
    current_score.selected_min_rank = selector_rank_a;
    current_score.selected_max_rank = selector_rank_a;
    current_score.selects_t0 = (selector_rank_a == NR_W'(0));
    if (decoded_profile.family == DIST_FAMILY_PAIR) begin
      current_score.selected_min_rank =
          (selector_rank_a <= selector_rank_b) ? selector_rank_a : selector_rank_b;
      current_score.selected_max_rank =
          (selector_rank_a >= selector_rank_b) ? selector_rank_a : selector_rank_b;
      current_score.selects_t0 |= (selector_rank_b == NR_W'(0));
    end
  end

  sched_distilled_regime_classify i_regime_classify (
    .mode_i            (mode_q),
    .count_i           (counters_i.count),
    .token_sum_i       (counters_i.token_sum),
    .odd_count_i       (counters_i.odd_count),
    .one_block_count_i (counters_i.small_hist[0]),
    .t0_ntok_i         (hot_i[0].ntok),
    .t1_valid_i        (hot_i[1].valid),
    .t1_ntok_i         (hot_i[1].ntok),
    .t4_valid_i        (hot_i[4].valid),
    .t4_ntok_i         (hot_i[4].ntok),
    .c2_end_i          (base_c2_i.task_end),
    .c3_end_i          (base_c3_i.task_end),
    .regime_o          (regime)
  );

  sched_distilled_pair_compare i_pair_compare (
    .clk_i                 (clk_i),
    .rst_ni                (rst_ni),
    .clear_i               (clear_i),
    .start_i               (compare_start),
    .done_o                (compare_done),
    .mode_i                (mode_q),
    .regime_i              (regime),
    .before_count_i        (counters_i.count),
    .min_remaining_load_i  (bottom_i.ntok),
    .lhs_i                 (global_score_q),
    .rhs_i                 (current_score),
    .children_differ_i     (1'b1),
    .rhs_wins_o            (compare_rhs_wins),
    .override_o            (compare_override)
  );

  always_comb begin
    st_d = st_q;
    mode_d = mode_q;
    profile_addr_d = profile_addr_q;
    assignment_swap_d = assignment_swap_q;
    candidate_start_d = candidate_start_q;
    group_local_d = group_local_q;
    group_target_d = group_target_q;
    global_valid_d = global_valid_q;
    global_token_d = global_token_q;
    global_score_d = global_score_q;
    materialization_d = materialization_q;
    materialization_global_d = materialization_global_q;

    start_iter_begin = 1'b0;
    eval_start = 1'b0;
    bound_start = 1'b0;
    compare_start = 1'b0;
    s4_start = 1'b0;

    unique case (st_q)
      ST_IDLE, ST_DONE: begin
        if (start_i) begin
          mode_d = (counters_i.count == NR_W'(1)) ? DIST_MODE_TERMINAL :
                   ((base_c2_i.task_end == base_c3_i.task_end) ?
                    DIST_MODE_SYNC : DIST_MODE_ONE_IDLE);
          profile_addr_d = distilled_mode_profile_base(mode_d);
          assignment_swap_d = 1'b0;
          group_local_d = '0;
          group_target_d = '0;
          global_valid_d = 1'b0;
          materialization_d = MAT_NONE;
          materialization_global_d = 1'b0;
          st_d = ST_PROFILE_SETUP;
        end
      end

      ST_PROFILE_SETUP: begin
        assignment_swap_d = 1'b0;
        if (!profile_structural_valid) begin
          // PAIR profiles normally evaluate both assignments. Mark the swap as
          // consumed so ST_NEXT_ACTION advances or closes the logical group.
          assignment_swap_d = decoded_profile.family == DIST_FAMILY_PAIR;
          st_d = ST_NEXT_ACTION;
        end else if (decoded_profile.family == DIST_FAMILY_SINGLE) begin
          start_iter_begin = 1'b1;
          st_d = ST_START_WAIT;
        end else begin
          candidate_start_d = (base_c2_i.task_end >= base_c3_i.task_end) ?
                              base_c2_i.task_end : base_c3_i.task_end;
          eval_start = 1'b1;
          st_d = ST_EVAL_WAIT;
        end
      end

      ST_START_WAIT: begin
        if (start_iter_valid) begin
          candidate_start_d = start_iter_value;
          eval_start = 1'b1;
          st_d = ST_EVAL_WAIT;
        end else if (start_iter_done) begin
          st_d = ST_NEXT_ACTION;
        end
      end

      ST_EVAL_WAIT: begin
        if (eval_done) begin
          materialization_d = MAT_NONE;
          if (eval_feasible && group_candidate_better) begin
            group_local_d.valid = 1'b1;
            group_local_d.token.profile_addr = profile_addr_q;
            group_local_d.token.assignment_swap =
                assignment_swap_q;
            group_local_d.token.start =
                candidate_start_q;
            group_local_d.token.targeted_s4pf_c2 =
                DMA_NONE;
            group_local_d.token.targeted_s4pf_c3 =
                DMA_NONE;
            group_local_d.max_end = eval_max_end;
            group_local_d.sum_end = eval_sum_end;
            group_local_d.s2pf_count =
                eval_s2pf_count;
            materialization_d = MAT_LOCAL;
          end
          if (decoded_profile.family == DIST_FAMILY_SINGLE) begin
            if (eval_feasible) begin
              if (s4_candidate_possible) begin
                s4_start = 1'b1;
                st_d = ST_S4_WAIT;
              end else begin
                st_d = ST_NEXT_ACTION;
              end
            end else begin
              st_d = ST_START_WAIT;
            end
          end else begin
            if (eval_feasible && s4_candidate_possible) begin
              s4_start = 1'b1;
              st_d = ST_S4_WAIT;
            end else begin
              st_d = ST_NEXT_ACTION;
            end
          end
        end
      end

      ST_S4_WAIT: begin
        if (s4_done) begin
          if ((s4_c2_binding == DMA_NONE) && (s4_c3_binding == DMA_NONE)) begin
            st_d = ST_NEXT_ACTION;
          end else if (decoded_profile.family == DIST_FAMILY_SINGLE) begin
            start_iter_begin = 1'b1;
            st_d = ST_TARGET_START_WAIT;
          end else begin
            st_d = ST_TARGET_EVAL_START;
          end
        end
      end

      ST_TARGET_START_WAIT: begin
        if (start_iter_valid) begin
          candidate_start_d = start_iter_value;
          st_d = ST_TARGET_EVAL_START;
        end else if (start_iter_done) begin
          st_d = ST_NEXT_ACTION;
        end
      end

      ST_TARGET_EVAL_START: begin
        eval_start = 1'b1;
        st_d = ST_TARGET_EVAL_WAIT;
      end

      ST_TARGET_EVAL_WAIT: begin
        if (eval_done) begin
          materialization_d = MAT_NONE;
          if (eval_feasible && group_candidate_better) begin
            group_target_d.valid = 1'b1;
            group_target_d.token = target_token;
            group_target_d.max_end = eval_max_end;
            group_target_d.sum_end = eval_sum_end;
            group_target_d.s2pf_count = eval_s2pf_count;
            materialization_d = MAT_TARGET;
          end
          if (decoded_profile.family == DIST_FAMILY_SINGLE) begin
            if (!eval_strict_gain) begin
              st_d = ST_NEXT_ACTION;
            end else if (eval_feasible) begin
              st_d = ST_NEXT_ACTION;
            end else begin
              st_d = ST_TARGET_START_WAIT;
            end
          end else begin
            st_d = ST_NEXT_ACTION;
          end
        end
      end

      ST_NEXT_ACTION: begin
        if ((decoded_profile.family == DIST_FAMILY_PAIR) &&
            !assignment_swap_q) begin
          assignment_swap_d = 1'b1;
          eval_start = 1'b1;
          st_d = ST_EVAL_WAIT;
        end else begin
          if (decoded_profile.logical_last) begin
            if (group_target_wins)
              group_local_d.token = group_target_q.token;
            group_target_d = '0;
            if (group_local_q.valid) begin
              st_d = score_materialized ? ST_SCORE_BYPASS_START :
                                          ST_SCORE_EVAL_START;
            end else if ((profile_addr_q + 1'b1) < profile_limit) begin
              group_local_d = '0;
              profile_addr_d = profile_addr_q + 1'b1;
              assignment_swap_d = 1'b0;
              st_d = ST_PROFILE_SETUP;
            end else begin
              group_local_d = '0;
              st_d = global_valid_q ?
                  (materialization_global_q ? ST_DONE :
                                              ST_COMMIT_REBUILD_START) :
                  ST_DONE;
            end
          end else if ((profile_addr_q + 1'b1) < profile_limit) begin
            profile_addr_d = profile_addr_q + 1'b1;
            assignment_swap_d = 1'b0;
            st_d = ST_PROFILE_SETUP;
          end else begin
            st_d = global_valid_q ?
                (materialization_global_q ? ST_DONE :
                                            ST_COMMIT_REBUILD_START) :
                ST_DONE;
          end
        end
      end

      ST_SCORE_EVAL_START: begin
        eval_start = 1'b1;
        st_d = ST_SCORE_EVAL_WAIT;
      end

      ST_SCORE_EVAL_WAIT: begin
        if (eval_done) begin
          if (!eval_feasible) begin
            materialization_d = MAT_NONE;
            st_d = ST_SCORE_NEXT;
          end else begin
            materialization_d = MAT_SCORE;
            bound_start = 1'b1;
            st_d = ST_BOUND_WAIT;
          end
        end
      end

      ST_SCORE_BYPASS_START: begin
        bound_start = 1'b1;
        st_d = ST_BOUND_WAIT;
      end

      ST_BOUND_WAIT: begin
        if (bound_done) begin
          if (!global_valid_q) begin
            global_valid_d = 1'b1;
            global_token_d = active_token;
            global_score_d = current_score;
            materialization_global_d = 1'b1;
            st_d = ST_SCORE_NEXT;
          end else begin
            compare_start = 1'b1;
            st_d = ST_COMPARE_WAIT;
          end
        end
      end

      ST_COMPARE_WAIT: begin
        if (compare_done) begin
          if (compare_rhs_wins) begin
            global_token_d = active_token;
            global_score_d = current_score;
            materialization_global_d = 1'b1;
          end else begin
            materialization_global_d = 1'b0;
          end
          st_d = ST_SCORE_NEXT;
        end
      end

      ST_SCORE_NEXT: begin
        group_local_d = '0;
        group_target_d = '0;
        if ((profile_addr_q + 1'b1) < profile_limit) begin
          profile_addr_d = profile_addr_q + 1'b1;
          assignment_swap_d = 1'b0;
          st_d = ST_PROFILE_SETUP;
        end else begin
          st_d = global_valid_q ?
              (materialization_global_q ? ST_DONE : ST_COMMIT_REBUILD_START) :
              ST_DONE;
        end
      end

      ST_COMMIT_REBUILD_START: begin
        eval_start = 1'b1;
        st_d = ST_COMMIT_REBUILD_WAIT;
      end

      ST_COMMIT_REBUILD_WAIT: begin
        if (eval_done)
          st_d = ST_DONE;
      end

      default: st_d = ST_IDLE;
    endcase

    if (eval_start) begin
      materialization_d = MAT_NONE;
      materialization_global_d = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q <= ST_IDLE;
      mode_q <= DIST_MODE_TERMINAL;
      profile_addr_q <= '0;
      assignment_swap_q <= 1'b0;
      candidate_start_q <= '0;
      group_local_q <= '0;
      group_target_q <= '0;
      global_valid_q <= 1'b0;
      global_token_q <= '0;
      global_score_q <= '0;
      materialization_q <= MAT_NONE;
      materialization_global_q <= 1'b0;
    end else if (clear_i) begin
      st_q <= ST_IDLE;
      mode_q <= DIST_MODE_TERMINAL;
      profile_addr_q <= '0;
      assignment_swap_q <= 1'b0;
      candidate_start_q <= '0;
      group_local_q <= '0;
      group_target_q <= '0;
      global_valid_q <= 1'b0;
      global_token_q <= '0;
      global_score_q <= '0;
      materialization_q <= MAT_NONE;
      materialization_global_q <= 1'b0;
    end else begin
      st_q <= st_d;
      mode_q <= mode_d;
      profile_addr_q <= profile_addr_d;
      assignment_swap_q <= assignment_swap_d;
      candidate_start_q <= candidate_start_d;
      group_local_q <= group_local_d;
      group_target_q <= group_target_d;
      global_valid_q <= global_valid_d;
      global_token_q <= global_token_d;
      global_score_q <= global_score_d;
      materialization_q <= materialization_d;
      materialization_global_q <= materialization_global_d;
    end
  end

`ifndef SYNTHESIS
  longint unsigned trace_cycle_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      trace_cycle_q <= 0;
    else if (clear_i)
      trace_cycle_q <= 0;
    else
      trace_cycle_q <= trace_cycle_q + 1;
  end

  always_ff @(posedge clk_i) begin
    if (rst_ni) begin
      assert (profile_addr_q < 5'd28);
      assert (!(transition_bw_start && target_bw_start));
      if (s4_start)
        assert (s4_candidate_possible);
      if (st_q == ST_SCORE_BYPASS_START)
        assert (materialization_q inside {MAT_LOCAL, MAT_TARGET});

      if ($test$plusargs("MOE_SCHED_RTL_TRACE")) begin
        if (start_i)
          $display("[MOE_SCHED_RTL_TRACE] time_ns=%0.3f cycle=%0d scope=round event=START remaining=%0d",
                   $realtime, trace_cycle_q, counters_i.count);
        if (eval_start)
          $display("[MOE_SCHED_RTL_TRACE] time_ns=%0.3f cycle=%0d scope=round event=EVAL_START state=%0d profile=%0d swap=%0d",
                   $realtime, trace_cycle_q, st_q, decode_address,
                   eval_assignment_swap);
        if (eval_done)
          $display("[MOE_SCHED_RTL_TRACE] time_ns=%0.3f cycle=%0d scope=round event=EVAL_DONE state=%0d profile=%0d feasible=%0d",
                   $realtime, trace_cycle_q, st_q, decode_address,
                   eval_feasible);
        if (s4_start)
          $display("[MOE_SCHED_RTL_TRACE] time_ns=%0.3f cycle=%0d scope=round event=S4_START profile=%0d",
                   $realtime, trace_cycle_q, decode_address);
        if (s4_done)
          $display("[MOE_SCHED_RTL_TRACE] time_ns=%0.3f cycle=%0d scope=round event=S4_DONE profile=%0d count=%0d",
                   $realtime, trace_cycle_q, decode_address, s4_count);
        if (bound_start)
          $display("[MOE_SCHED_RTL_TRACE] time_ns=%0.3f cycle=%0d scope=round event=BOUND_START profile=%0d",
                   $realtime, trace_cycle_q, decode_address);
        if (bound_done)
          $display("[MOE_SCHED_RTL_TRACE] time_ns=%0.3f cycle=%0d scope=round event=BOUND_DONE profile=%0d",
                   $realtime, trace_cycle_q, decode_address);
        if (compare_start)
          $display("[MOE_SCHED_RTL_TRACE] time_ns=%0.3f cycle=%0d scope=round event=COMPARE_START profile=%0d",
                   $realtime, trace_cycle_q, decode_address);
        if (compare_done)
          $display("[MOE_SCHED_RTL_TRACE] time_ns=%0.3f cycle=%0d scope=round event=COMPARE_DONE profile=%0d rhs_wins=%0d",
                   $realtime, trace_cycle_q, decode_address, compare_rhs_wins);
        if ((st_q != ST_DONE) && (st_d == ST_DONE))
          $display("[MOE_SCHED_RTL_TRACE] time_ns=%0.3f cycle=%0d scope=round event=DONE feasible=%0d",
                   $realtime, trace_cycle_q, global_valid_d);
      end
    end
  end
`endif

  assign done_o = (st_q == ST_DONE);
  assign feasible_o = global_valid_q;
  always_comb begin
    child_c2_o = eval_child_c2;
    child_c3_o = eval_child_c3;
    child_counters_o = eval_child_counters;
    child_counters_o.parent_bound = global_score_q.f;
    selected_token_o = '0;
    selected_token_o.valid = global_valid_q;
    selected_token_o.profile_slot = decoded_profile.profile_slot;
    selected_token_o.mode_index = global_token_q.profile_addr -
                                  distilled_mode_profile_base(mode_q);
    selected_token_o.logical_id = decoded_profile.logical_id;
    selected_token_o.assignment_swap = global_token_q.assignment_swap;
    selected_token_o.start = global_token_q.start;
    selected_token_o.targeted_s4pf_c2 = global_token_q.targeted_s4pf_c2;
    selected_token_o.targeted_s4pf_c3 = global_token_q.targeted_s4pf_c3;
  end
  assign plan_o = eval_plan;
  assign remove_count_o = eval_remove_count;
  assign remove_eid_a_o = eval_remove_eid_a;
  assign remove_eid_b_o = eval_remove_eid_b;
  assign selected_score_o = global_score_q;

endmodule
