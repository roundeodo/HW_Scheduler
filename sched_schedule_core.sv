// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — one-round datapath core
//
// This is the datapath-only core under the MMIO wrapper:
//   wrapper-maintained top4/reserve state -> top4 round context -> candidate_generator
//   -> candidate_eval_lane -> compact best_reduce -> replay(best_id)
//   -> commit_unit -> remove metadata + depth-4 round FIFO
//
// There is intentionally no internal rem_sram, plan_sram, AXI master, or DMA
// writer in this module.  The full rem stream and final lowered args live
// outside this core; the wrapper only exposes register/FIFO state to CVA6.

import sched_pkg::*;

module sched_schedule_core (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  // init_i resets the persistent cluster timeline/cache state at the start of
  // a full scheduling run.  start_i computes exactly one scheduling round.
  input  logic                         init_i,
  input  logic                         start_i,
  input  logic [7:0]                   cache_eid_c2_i,
  input  logic [7:0]                   cache_eid_c3_i,

  // Per-round context supplied by CVA6/software from the L3 rem list.
  input  head_ctx_t [3:0]              head_i,
  input  logic [NR_W-1:0]              active_count_i,
  input  logic [T_W-1:0]               total_conc_i,

  // Core-level remove handshake.  In the 10K wrapper fast path this is driven
  // internally by auto-run after a round completes.
  input  logic                         remove_ready_i,
  output logic                         remove_valid_o,
  output logic [1:0]                   remove_count_o,
  output logic [3:0]                   remove_slot_mask_o,

  // FIFO drain path: CVA6 pops completed round packets after reading them.
  // In the current direct-lowering path this is not an L3 plan SRAM writeback.
  input  logic [2:0]                   plan_pop_count_i,
  output logic                         plan_valid_o,
  output logic [PLANQ_DEPTH-1:0][1:0]  plan_slot_valid_o,
  output logic [PLANQ_DEPTH-1:0][1:0][63:0] plan_data_o,
  output logic [PLANQ_DEPTH-1:0][1:0]  plan_count_o,
  output logic [PLANQ_DEPTH-1:0][1:0]  plan_remove_count_o,
  output logic                         plan_queue_full_o,
  output logic [3:0]                   plan_queue_count_o,

  output logic                         busy_o,
  output logic                         done_o,
  output logic [T_W-1:0]               makespan_o
);

  typedef enum logic [3:0] {
    ST_IDLE,
    ST_ROUND_START,
    ST_GHOST_C2_START,
    ST_GHOST_C2_WAIT,
    ST_GHOST_C3_START,
    ST_GHOST_C3_WAIT,
    ST_EVAL_START,
    ST_EVAL,
    ST_REPLAY_START,
    ST_REPLAY_WAIT,
    ST_FALLBACK,
    ST_COMMIT_WAIT,
    ST_DONE_EMPTY
  } state_t;

  state_t st_q, st_d;

  // These registers feed several local sub-blocks.  Give Vivado permission to
  // replicate drivers instead of forcing one high-fanout net across the whole
  // scheduler placement region.
  (* max_fanout = 64 *) eval_snap_t c2_snap_q;
  eval_snap_t c2_snap_d;
  (* max_fanout = 64 *) eval_snap_t c3_snap_q;
  eval_snap_t c3_snap_d;
  (* max_fanout = 64 *) head_ctx_t [3:0] round_head_q;
  head_ctx_t [3:0] round_head_d;
  logic [NR_W-1:0] round_active_count_q, round_active_count_d;
  logic [T_W-1:0]  round_total_conc_q, round_total_conc_d;
  slot_id_t   c2_slot_q, c2_slot_d;
  slot_id_t   c3_slot_q, c3_slot_d;

  logic eval_inflight_q, eval_inflight_d;
  logic done_q, done_d;

  logic              remove_valid_q, remove_valid_d;
  logic [1:0]        remove_count_q, remove_count_d;
  logic [3:0]        remove_slot_mask_q, remove_slot_mask_d;

  localparam int unsigned PLANQ_COUNT_W = $clog2(PLANQ_DEPTH + 1);

  logic [PLANQ_COUNT_W-1:0] planq_count_q, planq_count_d;
  logic [PLANQ_COUNT_W-1:0] planq_pop_count_eff;
  logic [PLANQ_COUNT_W-1:0] planq_count_after_pop;
  logic [PLANQ_DEPTH-1:0][1:0][63:0] planq_data_q, planq_data_d;
  logic [1:0]               planq_plan_count_q [PLANQ_DEPTH], planq_plan_count_d [PLANQ_DEPTH];
  logic [1:0]               planq_remove_count_q [PLANQ_DEPTH], planq_remove_count_d [PLANQ_DEPTH];
  logic              planq_has_space_after_pop;
  logic              remove_free_after_ready;
  logic [1:0]        tail_pending_valid_q, tail_pending_valid_d;
  logic [1:0]        tail_pending_skip_s1_q, tail_pending_skip_s1_d;
  slot_id_t [1:0]    tail_pending_slot_q, tail_pending_slot_d;
  logic [1:0][7:0]   commit_inline_patch;
  logic [1:0][63:0]  commit_plan_data;

  function automatic eval_snap_t make_initial_snap(input logic [7:0] cache_eid);
    eval_snap_t s;
    s = '0;
    s.pf_eid = PF_EID_NONE;
    if (!cache_eid[7]) begin
      s.pf_eid  = encode_eid(cache_eid[EID_RAW_W-1:0]);
      s.pf_end  = '0;
      s.pf_full = 1'b1;
    end
    make_initial_snap = s;
  endfunction

  function automatic eval_snap_t apply_accepted_ghost(
    input eval_snap_t base,
    input logic       accept
  );
    eval_snap_t s;
    begin
      s = base;
      if (accept) begin
        s.pf_eid      = PF_EID_GHOST;
        s.pf_end      = base.task_end;
        s.pf_full     = 1'b0;
        s.s4pf_valid  = 1'b1;
        s.s4pf_start  = base.s4_start;
      end
      apply_accepted_ghost = s;
    end
  endfunction

  function automatic cand_gen_snap_t make_cand_gen_snap(input eval_snap_t s);
    cand_gen_snap_t v;
    begin
      v = '0;
      v.task_end = s.task_end;
      v.pf_eid   = s.pf_eid;
      v.pf_end   = s.pf_end;
      v.pf_full  = s.pf_full;

      v.release_valid[0] = s.valid && (s.bw_s1 != BW_0) &&
                           (s.dma1_end > s.task_start);
      v.release_t[0]     = s.dma1_end;

      v.release_valid[1] = s.valid && s.s2pf_valid &&
                           (s.s2pf_bw != BW_0) &&
                           (s.s2pf_end > s.s2pf_start);
      v.release_t[1]     = s.s2pf_end;

      v.release_valid[2] = s.valid && (s.bw_s3 != BW_0) &&
                           (s.dma3_end > s.s2_end);
      v.release_t[2]     = s.dma3_end;

      v.release_valid[3] = s.valid && s.s4pf_valid;
      v.release_t[3]     = s.s4pf_start + GHOST_WINDOW_TICKS;

      make_cand_gen_snap = v;
    end
  endfunction

  // ── Ghost injection ───────────────────────────────────────────────────
  eval_snap_t c2_ghost_try;
  eval_snap_t c3_ghost_try;
  logic       ghost_bw_start;
  eval_snap_t ghost_bw_snap_a;
  eval_snap_t ghost_bw_snap_b;
  logic       c2_ghost_bw_ok_q, c2_ghost_bw_ok_d;
  logic       c2_new_ghost;
  logic       c3_new_ghost;

  logic       shared_bw_start;
  eval_snap_t shared_bw_snap_a;
  eval_snap_t shared_bw_snap_b;
  logic       shared_bw_done;
  logic       shared_bw_ok;

  logic       eval_bw_start;
  eval_snap_t eval_bw_snap_a;
  eval_snap_t eval_bw_snap_b;

  logic       commit_bw_start;
  eval_snap_t commit_bw_snap_a;
  eval_snap_t commit_bw_snap_b;

  sched_ghost_inject_unit i_ghost_c2 (
    .snap_i (c2_snap_q),
    .snap_o (c2_ghost_try)
  );

  sched_ghost_inject_unit i_ghost_c3 (
    .snap_i (c3_snap_q),
    .snap_o (c3_ghost_try)
  );

  sched_bw_ok_seq i_shared_bw_ok (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .start_i  (shared_bw_start),
    .busy_o   (),
    .done_o   (shared_bw_done),
    .snap_a_i (shared_bw_snap_a),
    .snap_b_i (shared_bw_snap_b),
    .ok_o     (shared_bw_ok)
  );

  // One physical BW checker is shared by the whole scheduler datapath.  The
  // core FSM guarantees these request sources are mutually exclusive, so this
  // is a stage mux rather than a general arbiter.
  always_comb begin
    shared_bw_start  = 1'b0;
    shared_bw_snap_a = '0;
    shared_bw_snap_b = '0;

    unique case (st_q)
      ST_GHOST_C2_START,
      ST_GHOST_C3_START: begin
        shared_bw_start  = ghost_bw_start;
        shared_bw_snap_a = ghost_bw_snap_a;
        shared_bw_snap_b = ghost_bw_snap_b;
      end

      ST_EVAL,
      ST_REPLAY_WAIT: begin
        shared_bw_start  = eval_bw_start;
        shared_bw_snap_a = eval_bw_snap_a;
        shared_bw_snap_b = eval_bw_snap_b;
      end

      ST_COMMIT_WAIT: begin
        shared_bw_start  = commit_bw_start;
        shared_bw_snap_a = commit_bw_snap_a;
        shared_bw_snap_b = commit_bw_snap_b;
      end

      default: begin
      end
    endcase
  end

  assign c2_new_ghost = (c2_snap_q.pf_eid == PF_EID_NONE) &&
                        (c2_ghost_try.pf_eid == PF_EID_GHOST);
  assign c3_new_ghost = (c3_snap_q.pf_eid == PF_EID_NONE) &&
                        (c3_ghost_try.pf_eid == PF_EID_GHOST);

  // ── Candidate generation ──────────────────────────────────────────────
  logic       gen_start;
  logic       gen_advance;
  logic       gen_replay;
  logic       gen_busy;
  logic       gen_done;
  cand_desc_t cand;
  logic       cand_valid;
  cand_gen_snap_t c2_gen_snap;
  cand_gen_snap_t c3_gen_snap;

  assign c2_gen_snap = make_cand_gen_snap(c2_snap_q);
  assign c3_gen_snap = make_cand_gen_snap(c3_snap_q);

  logic best_valid;
  logic [CAND_ID_W-1:0] best_id;
  logic [1:0] best_remove_count;
  logic [3:0] best_remove_slot_mask;

  sched_candidate_generator i_candidate_generator (
    .clk_i                 (clk_i),
    .rst_ni                (rst_ni),
    .start_i               (gen_start),
    .advance_i             (gen_advance),
    .replay_i              (gen_replay),
    .replay_candidate_id_i (best_id),
    .busy_o                (gen_busy),
    .done_o                (gen_done),
    .c2_snap_i             (c2_gen_snap),
    .c3_snap_i             (c3_gen_snap),
    .head_i                (round_head_q),
    .active_count_i        (round_active_count_q),
    .total_conc_i          (round_total_conc_q),
    .cand_o                (cand),
    .cand_valid_o          (cand_valid)
  );

  // ── Candidate evaluation lane ─────────────────────────────────────────
  logic       eval_valid;
  logic       eval_start;
  logic       eval_busy;
  logic       eval_done;
  logic       eval_bw_ok;
  logic [T_W-1:0] unused_makespan;
  score_key_t eval_score;
  plan_desc_t eval_plan;
  eval_snap_t eval_snap_a;
  eval_snap_t eval_snap_b;

  sched_candidate_eval_lane i_eval_lane (
    .clk_i                 (clk_i),
    .rst_ni                (rst_ni),
    .start_i               (eval_start),
    .busy_o                (eval_busy),
    .done_o                (eval_done),
    .cand_valid_i          (cand_valid),
    .plan_type_i           (cand.plan_type),
    .cluster_a_i           (cand.cluster_a),
    .s2pf_policy_i         (cand.s2pf_policy),
    .force_shape_a_i       (cand.force_shape_a),
    .force_shape_b_i       (cand.force_shape_b),
    .forced_s1a_i          (cand.forced_s1a),
    .forced_s3a_i          (cand.forced_s3a),
    .forced_s1b_i          (cand.forced_s1b),
    .forced_s3b_i          (cand.forced_s3b),
    .cost_only_tie_i       (cand.cost_only_tie),
    .score_makespan_only_i (cand.score_makespan_only),
    .base_snap_a_i         (c2_snap_q),
    .base_snap_b_i         (c3_snap_q),
    .side_a_valid_i        (cand.side_a_valid),
    .side_b_valid_i        (cand.side_b_valid),
    .start_a_i             (cand.start_a),
    .start_b_i             (cand.start_b),
    .eid_a_i               (cand.eid_a),
    .eid_b_i               (cand.eid_b),
    .ntok_a_i              (cand.ntok_a),
    .ntok_b_i              (cand.ntok_b),
    .tok_start_a_i         (cand.tok_start_a),
    .tok_start_b_i         (cand.tok_start_b),
    .sw_a_i                (cand.sw_a),
    .dn_a_i                (cand.dn_a),
    .sw_b_i                (cand.sw_b),
    .dn_b_i                (cand.dn_b),
    .shape_t0_i            (cand.shape_t0),
    .rem_len_after_i       (cand.rem_len_after),
    .rem0_eid_i            (cand.rem0_eid),
    .rem0_ntok_i           (cand.rem0_ntok),
    .rem1_ntok_i           (cand.rem1_ntok),
    .total_conc_after_i    (cand.total_conc_after),
    .max_conc_after_i      (cand.max_conc_after),
    .bw_start_o            (eval_bw_start),
    .bw_snap_a_o           (eval_bw_snap_a),
    .bw_snap_b_o           (eval_bw_snap_b),
    .bw_done_i             (((st_q == ST_EVAL) || (st_q == ST_REPLAY_WAIT)) ? shared_bw_done : 1'b0),
    .bw_ok_i               (shared_bw_ok),
    .eval_valid_o          (eval_valid),
    .bw_ok_o               (eval_bw_ok),
    .makespan_o            (unused_makespan),
    .score_key_o           (eval_score),
    .plan_desc_o           (eval_plan),
    .snap_a_o              (eval_snap_a),
    .snap_b_o              (eval_snap_b),
    .shape_s1a_o           (),
    .shape_s3a_o           (),
    .shape_s1b_o           (),
    .shape_s3b_o           (),
    .task_end_a_o          (),
    .task_end_b_o          (),
    .s2_end_a_o            (),
    .s2_end_b_o            (),
    .s4_start_a_o          (),
    .s4_start_b_o          (),
    .bw_s1a_o              (),
    .bw_s3a_o              (),
    .bw_s1b_o              (),
    .bw_s3b_o              (),
    .m_s2_a_o              (),
    .m_s4_a_o              (),
    .m_s2_b_o              (),
    .m_s4_b_o              (),
    .skip_s2_a_o           (),
    .skip_s4_a_o           (),
    .skip_s2_b_o           (),
    .skip_s4_b_o           (),
    .dma_s1_a_o            (),
    .dma_s3_a_o            (),
    .dma_s1_b_o            (),
    .dma_s3_b_o            ()
  );

  // ── Best reducer: compact winner only ─────────────────────────────────
  logic best_clear;

  sched_best_reduce i_best_reduce (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .clear_i              (best_clear),
    .cand_valid_i         ((st_q == ST_EVAL) && eval_valid),
    .cand_id_i            (cand.candidate_id),
    .cand_score_i         (eval_score),
    .cand_remove_slot_mask_i (cand.remove_slot_mask),
    .best_valid_o         (best_valid),
    .best_id_o            (best_id),
    .best_score_o         (),
    .best_remove_count_o  (best_remove_count),
    .best_remove_slot_mask_o (best_remove_slot_mask),
    .accepted_o           ()
  );

  // ── Fallback path matching scheduler.c semantics ──────────────────────
  logic fallback_valid;
  logic fallback_both_idle;
  logic fallback_idle_is_c3;
  logic fallback_cluster;
  logic [T_W-1:0] fallback_tnow;
  logic [T_W-1:0] fallback_start;
  logic fallback_sw;
  logic fallback_dn;
  logic [T_W-1:0] fallback_task_start;
  logic [T_W-1:0] fallback_task_end;
  logic [T_W-1:0] fallback_dma1_end;
  logic [T_W-1:0] fallback_s2_end;
  logic [T_W-1:0] fallback_dma3_end;
  logic [T_W-1:0] fallback_s4_start;
  logic [BW_W-1:0] fallback_bw_s1;
  logic [BW_W-1:0] fallback_bw_s3;
  logic [NTOK_W-1:0] fallback_m_s2;
  logic [NTOK_W-1:0] fallback_m_s4;
  logic fallback_skip_s2;
  logic fallback_skip_s4;
  logic [1:0] fallback_dma_s1;
  logic [1:0] fallback_dma_s3;
  eval_snap_t fallback_snap;
  plan_desc_t fallback_plan;

  sched_mk_snap i_fallback_mk_snap (
    .start_t_i    (fallback_start),
    .ntok_i       (round_head_q[0].ntok),
    .shape_s1_i   (SHAPE_C),
    .shape_s3_i   (SHAPE_C),
    .skip_s1_i    (fallback_sw),
    .skip_s3_i    (fallback_dn),
    .task_start_o (fallback_task_start),
    .task_end_o   (fallback_task_end),
    .dma1_end_o   (fallback_dma1_end),
    .s2_end_o     (fallback_s2_end),
    .dma3_end_o   (fallback_dma3_end),
    .s4_start_o   (fallback_s4_start),
    .bw_s1_o      (fallback_bw_s1),
    .bw_s3_o      (fallback_bw_s3),
    .m_s2_exec_o  (fallback_m_s2),
    .m_s4_exec_o  (fallback_m_s4),
    .skip_s2_o    (fallback_skip_s2),
    .skip_s4_o    (fallback_skip_s4),
    .dma_s1_o     (fallback_dma_s1),
    .dma_s3_o     (fallback_dma_s3)
  );

  always_comb begin
    fallback_tnow       = (c2_snap_q.task_end > c3_snap_q.task_end) ?
                          c2_snap_q.task_end : c3_snap_q.task_end;
    fallback_both_idle  = (c2_snap_q.task_end == c3_snap_q.task_end);
    fallback_idle_is_c3 = (c3_snap_q.task_end < c2_snap_q.task_end);
    fallback_cluster    = fallback_both_idle ? 1'b0 : fallback_idle_is_c3;
    fallback_start      = fallback_both_idle ? fallback_tnow :
                          (fallback_idle_is_c3 ? c3_snap_q.task_end : c2_snap_q.task_end);

    // Match C: not_both fallback reuses c2c0/c3c0 computed at tnow, not idle_t.
    fallback_sw = fallback_cluster ?
                  swiglu_hit_t(round_head_q[0].eid, c3_snap_q.pf_eid, c3_snap_q.pf_end,
                               fallback_tnow) :
                  swiglu_hit_t(round_head_q[0].eid, c2_snap_q.pf_eid, c2_snap_q.pf_end,
                               fallback_tnow);
    fallback_dn = fallback_cluster ?
                  down_hit_t(round_head_q[0].eid, c3_snap_q.pf_eid, c3_snap_q.pf_end,
                             c3_snap_q.pf_full, fallback_tnow) :
                  down_hit_t(round_head_q[0].eid, c2_snap_q.pf_eid, c2_snap_q.pf_end,
                             c2_snap_q.pf_full, fallback_tnow);

    fallback_snap = '0;
    fallback_snap.valid      = round_head_q[0].valid;
    fallback_snap.task_start = fallback_task_start;
    fallback_snap.task_end   = fallback_task_end;
    fallback_snap.dma1_end   = fallback_dma1_end;
    fallback_snap.s2_end     = fallback_s2_end;
    fallback_snap.dma3_end   = fallback_dma3_end;
    fallback_snap.s4_start   = fallback_s4_start;
    fallback_snap.bw_s1      = fallback_bw_s1;
    fallback_snap.bw_s3      = fallback_bw_s3;
    fallback_snap.ntok       = round_head_q[0].ntok;
    fallback_snap.pf_eid     = PF_EID_NONE;

    fallback_plan = '0;
    fallback_plan.plan_type   = 2'b10; // SOLO
    fallback_plan.cluster_a   = fallback_cluster;
    fallback_plan.eid_a       = round_head_q[0].eid;
    fallback_plan.ntok_a      = round_head_q[0].ntok;
    fallback_plan.tok_start_a = '0;
    fallback_plan.s1a         = SHAPE_C;
    fallback_plan.s3a         = SHAPE_C;
    fallback_plan.skip_s1_a   = (fallback_bw_s1 == BW_0);
    fallback_plan.skip_s3_a   = (fallback_bw_s3 == BW_0);

    fallback_valid = !best_valid && round_head_q[0].valid &&
                     (round_active_count_q != NR_W'(0));
  end

  // ── Commit mux and output-buffer producer ─────────────────────────────
  logic commit_from_replay;
  logic commit_from_fallback;
  logic commit_fire;
  plan_desc_t commit_plan;
  eval_snap_t commit_snap_a;
  eval_snap_t commit_snap_b;
  logic [1:0] commit_remove_count;
  logic [3:0] commit_remove_slot_mask;

  assign commit_from_replay   = (st_q == ST_REPLAY_WAIT) && eval_done && eval_valid;
  assign commit_from_fallback = (st_q == ST_FALLBACK) && fallback_valid;
  assign commit_fire          = commit_from_replay || commit_from_fallback;

  always_comb begin
    commit_plan          = commit_from_replay ? eval_plan   : fallback_plan;
    commit_remove_count  = commit_from_replay ? best_remove_count : 2'd1;
    commit_remove_slot_mask = commit_from_replay ? best_remove_slot_mask : 4'b0001;

    if (commit_from_replay) begin
      commit_snap_a = eval_snap_a;
      commit_snap_b = eval_snap_b;
    end else if (fallback_cluster) begin
      commit_snap_a = c2_snap_q;
      commit_snap_b = fallback_snap;
    end else begin
      commit_snap_a = fallback_snap;
      commit_snap_b = c3_snap_q;
    end
  end

  logic              commit_valid;
  eval_snap_t        commit_c2_snap;
  eval_snap_t        commit_c3_snap;
  task_desc_t [1:0] commit_task_desc;
  logic [1:0]        commit_plan_allow_s4pf;
  logic [1:0]        commit_plan_count;
  slot_id_t [1:0]    commit_local_slot;
  slot_id_t          c2_slot_after_commit;
  slot_id_t          c3_slot_after_commit;
  logic [1:0]        commit_remove_count_o;
  logic [3:0]        commit_remove_slot_mask_o;
  logic              commit_busy;
  logic              commit_done;

  sched_commit_unit i_commit (
    .clk_i               (clk_i),
    .rst_ni              (rst_ni),
    .commit_i            (commit_fire),
    .best_valid_i        (commit_fire),
    .best_plan_i         (commit_plan),
    .best_snap_a_i       (commit_snap_a),
    .best_snap_b_i       (commit_snap_b),
    .best_remove_count_i (commit_remove_count),
    .best_remove_slot_mask_i (commit_remove_slot_mask),
    .busy_o              (commit_busy),
    .done_o              (commit_done),
    .commit_valid_o      (commit_valid),
    .next_c2_snap_o      (commit_c2_snap),
    .next_c3_snap_o      (commit_c3_snap),
    .plan_valid_o        (),
    .task_desc_o         (commit_task_desc),
    .plan_allow_s4pf_o   (commit_plan_allow_s4pf),
    .plan_count_o        (commit_plan_count),
    .remove_count_o      (commit_remove_count_o),
    .remove_slot_mask_o  (commit_remove_slot_mask_o),
    .bw_start_o          (commit_bw_start),
    .bw_snap_a_o         (commit_bw_snap_a),
    .bw_snap_b_o         (commit_bw_snap_b),
    .bw_done_i           ((st_q == ST_COMMIT_WAIT) ? shared_bw_done : 1'b0),
    .bw_ok_i             (shared_bw_ok)
  );

  always_comb begin
    commit_local_slot = '{default: '0};
    c2_slot_after_commit = c2_slot_q;
    c3_slot_after_commit = c3_slot_q;

    // Assign local slots exactly when a committed task enters the output FIFO.
    // The task descriptor already carries the physical cluster, so this is just
    // two tiny per-cluster counters and no software-visible scheduling decision.
    for (int s = 0; s < 2; s++) begin
      if (s < int'(commit_plan_count)) begin
        if (commit_task_desc[s].cluster) begin
          commit_local_slot[s] = c3_slot_after_commit;
          c3_slot_after_commit = c3_slot_after_commit + slot_id_t'(1);
        end else begin
          commit_local_slot[s] = c2_slot_after_commit;
          c2_slot_after_commit = c2_slot_after_commit + slot_id_t'(1);
        end
      end
    end
  end

  // Compute S4PF inline patch at FIFO enqueue time, not on MMIO read.
  // This keeps the wrapper read path to a small indexed mux.  The only
  // sequential S4PF state is the tail pending record: for each cluster, whether
  // the latest enqueued task still waits for the next same-cluster task.
  always_comb begin
    logic [1:0]     pend_valid;
    logic [1:0]     pend_skip_s1;
    slot_id_t [1:0] pend_slot;

    commit_inline_patch = '{default: '0};
    commit_plan_data    = '{default: '0};

    pend_valid   = tail_pending_valid_q;
    pend_skip_s1 = tail_pending_skip_s1_q;
    pend_slot    = tail_pending_slot_q;

    for (int s = 0; s < 2; s++) begin
      if (s < int'(commit_plan_count)) begin
        int ci;
        logic cache_hit;
        logic no_copy;

        ci = int'(commit_task_desc[s].cluster);
        cache_hit = (ci == 0) ?
                    (!cache_eid_c2_i[7] &&
                     (cache_eid_c2_i[EID_RAW_W-1:0] == commit_task_desc[s].eid)) :
                    (!cache_eid_c3_i[7] &&
                     (cache_eid_c3_i[EID_RAW_W-1:0] == commit_task_desc[s].eid));
        no_copy = pend_skip_s1[ci] && cache_hit;

        if (pend_valid[ci]) begin
          commit_inline_patch[s] = pack_inline_patch(1'b1, no_copy, pend_slot[ci]);
          pend_valid[ci]   = 1'b0;
          pend_skip_s1[ci] = 1'b0;
          pend_slot[ci]    = '0;
        end

        if (commit_plan_allow_s4pf[s]) begin
          pend_valid[ci]   = 1'b1;
          pend_skip_s1[ci] = commit_task_desc[s].skip_s1;
          pend_slot[ci]    = commit_local_slot[s];
        end

        commit_plan_data[s] = pack_plan_entry_word(commit_task_desc[s],
                                                   commit_local_slot[s],
                                                   commit_inline_patch[s]);
      end
    end

    tail_pending_valid_d   = tail_pending_valid_q;
    tail_pending_skip_s1_d = tail_pending_skip_s1_q;
    tail_pending_slot_d    = tail_pending_slot_q;

    if (init_i) begin
      tail_pending_valid_d   = '0;
      tail_pending_skip_s1_d = '0;
      tail_pending_slot_d    = '{default: '0};
    end else if (commit_done && commit_valid &&
                 (planq_count_d < PLANQ_COUNT_W'(PLANQ_DEPTH))) begin
      tail_pending_valid_d   = pend_valid;
      tail_pending_skip_s1_d = pend_skip_s1;
      tail_pending_slot_d    = pend_slot;
    end
  end

  // ── Control FSM ───────────────────────────────────────────────────────
  always_comb begin
    st_d                 = st_q;
    c2_snap_d            = c2_snap_q;
    c3_snap_d            = c3_snap_q;
    round_head_d         = round_head_q;
    round_active_count_d = round_active_count_q;
    round_total_conc_d   = round_total_conc_q;
    c2_slot_d            = c2_slot_q;
    c3_slot_d            = c3_slot_q;
    eval_inflight_d      = eval_inflight_q;
    done_d               = 1'b0;
    c2_ghost_bw_ok_d     = c2_ghost_bw_ok_q;

    remove_valid_d        = remove_valid_q;
    remove_count_d        = remove_count_q;
    remove_slot_mask_d    = remove_slot_mask_q;

    planq_count_d         = planq_count_q;
    for (int q = 0; q < PLANQ_DEPTH; q++) begin
      planq_data_d[q]       = planq_data_q[q];
      planq_plan_count_d[q] = planq_plan_count_q[q];
      planq_remove_count_d[q] = planq_remove_count_q[q];
    end

    gen_start   = 1'b0;
    gen_advance = 1'b0;
    gen_replay  = 1'b0;
    eval_start  = 1'b0;
    best_clear  = 1'b0;
    ghost_bw_start = 1'b0;
    ghost_bw_snap_a = c2_snap_q;
    ghost_bw_snap_b = c3_snap_q;

    remove_free_after_ready = !remove_valid_q || remove_ready_i;
    planq_pop_count_eff = (PLANQ_COUNT_W'(plan_pop_count_i) > planq_count_q) ?
                           planq_count_q : PLANQ_COUNT_W'(plan_pop_count_i);
    planq_count_after_pop = planq_count_q - planq_pop_count_eff;
    planq_has_space_after_pop =
        (planq_count_after_pop < PLANQ_COUNT_W'(PLANQ_DEPTH));

    if (remove_ready_i) begin
      remove_valid_d = 1'b0;
      remove_count_d = '0;
      remove_slot_mask_d = '0;
    end

    // CVA6 pops the oldest queued plan entry after reading it.  Pop is handled
    // before an internal commit push, so pop+push in the same cycle is safe.
    if (planq_pop_count_eff != PLANQ_COUNT_W'(0)) begin
      for (int q = 0; q < PLANQ_DEPTH; q++) begin
        int src_idx;
        src_idx = q + int'(planq_pop_count_eff);
        if (src_idx < PLANQ_DEPTH) begin
          planq_data_d[q]       = planq_data_q[src_idx];
          planq_plan_count_d[q] = planq_plan_count_q[src_idx];
          planq_remove_count_d[q] = planq_remove_count_q[src_idx];
        end else begin
          planq_data_d[q]       = '{default: '0};
          planq_plan_count_d[q] = '0;
          planq_remove_count_d[q] = '0;
        end
      end
      planq_count_d = planq_count_after_pop;
    end

    if (init_i) begin
      c2_snap_d            = make_initial_snap(cache_eid_c2_i);
      c3_snap_d            = make_initial_snap(cache_eid_c3_i);
      round_head_d         = '{default: '0};
      round_active_count_d = '0;
      round_total_conc_d   = '0;
      c2_slot_d            = '0;
      c3_slot_d            = '0;
      eval_inflight_d      = 1'b0;
      c2_ghost_bw_ok_d     = 1'b0;
      remove_valid_d       = 1'b0;
      remove_count_d       = '0;
      remove_slot_mask_d   = '0;
      planq_count_d        = '0;
      for (int q = 0; q < PLANQ_DEPTH; q++) begin
        planq_data_d[q]       = '{default: '0};
        planq_plan_count_d[q] = '0;
        planq_remove_count_d[q] = '0;
      end
      // Batch initialization and first-round start may arrive in the same
      // control write.  Init still clears all persistent round state first;
      // start_i then arms the freshly initialized core for the first round.
      st_d                 = start_i ? ST_ROUND_START : ST_IDLE;
    end else begin
      unique case (st_q)
        ST_IDLE: begin
          if (start_i && remove_free_after_ready && planq_has_space_after_pop) begin
            eval_inflight_d = 1'b0;
            st_d            = ST_ROUND_START;
          end
        end

        ST_ROUND_START: begin
          // Latch the per-round context here rather than in ST_IDLE.  In the
          // auto-run path the wrapper can compact/refill head_q on the same
          // edge that raises start_i; one cycle later the updated head_i is
          // stable and can safely drive this round.
          round_head_d         = head_i;
          round_active_count_d = active_count_i;
          round_total_conc_d   = total_conc_i;
          if (active_count_i == NR_W'(0)) begin
            done_d = 1'b1;
            st_d   = ST_DONE_EMPTY;
          end else begin
            st_d = ST_GHOST_C2_START;
          end
        end

        ST_GHOST_C2_START: begin
          ghost_bw_start = 1'b1;
          ghost_bw_snap_a = c2_ghost_try;
          ghost_bw_snap_b = c3_snap_q;
          st_d           = ST_GHOST_C2_WAIT;
        end

        ST_GHOST_C2_WAIT: begin
          if (shared_bw_done) begin
            c2_ghost_bw_ok_d = shared_bw_ok;
            st_d             = ST_GHOST_C3_START;
          end
        end

        ST_GHOST_C3_START: begin
          ghost_bw_start = 1'b1;
          ghost_bw_snap_a = c2_snap_q;
          ghost_bw_snap_b = c3_ghost_try;
          st_d           = ST_GHOST_C3_WAIT;
        end

        ST_GHOST_C3_WAIT: begin
          if (shared_bw_done) begin
            c2_snap_d = apply_accepted_ghost(c2_snap_q,
                                             c2_new_ghost && c2_ghost_bw_ok_q);
            c3_snap_d = apply_accepted_ghost(c3_snap_q,
                                             c3_new_ghost && shared_bw_ok);
            c2_ghost_bw_ok_d = 1'b0;
            st_d      = ST_EVAL_START;
          end
        end

        ST_EVAL_START: begin
          best_clear      = 1'b1;
          gen_start       = 1'b1;
          eval_inflight_d = 1'b0;
          st_d            = ST_EVAL;
        end

        ST_EVAL: begin
          if (gen_busy && !eval_inflight_q) begin
            eval_start       = 1'b1;
            eval_inflight_d  = 1'b1;
          end else if (eval_inflight_q && eval_done) begin
            gen_advance      = 1'b1;
            eval_inflight_d  = 1'b0;
          end

          if (gen_done && !eval_inflight_q) begin
            st_d = best_valid ? ST_REPLAY_START : ST_FALLBACK;
          end
        end

        ST_REPLAY_START: begin
          // Re-emit only best_id through candidate_generator and evaluate it
          // once more.  This recovers full plan/snap without storing those
          // large structs during the whole candidate sweep.
          gen_replay       = 1'b1;
          eval_start       = 1'b1;
          eval_inflight_d  = 1'b1;
          st_d             = ST_REPLAY_WAIT;
        end

        ST_REPLAY_WAIT: begin
          gen_replay = 1'b1;
          if (eval_inflight_q && eval_done) begin
            eval_inflight_d = 1'b0;
            st_d             = ST_COMMIT_WAIT;
          end
        end

        ST_FALLBACK: begin
          st_d = ST_COMMIT_WAIT;
        end

        ST_COMMIT_WAIT: begin
          if (commit_done) begin
            if (commit_valid) begin
            c2_snap_d             = commit_c2_snap;
            c3_snap_d             = commit_c3_snap;
            c2_slot_d             = c2_slot_after_commit;
            c3_slot_d             = c3_slot_after_commit;
            remove_valid_d        = 1'b1;
            remove_count_d        = commit_remove_count_o;
            remove_slot_mask_d    = commit_remove_slot_mask_o;
            if (planq_count_d < PLANQ_COUNT_W'(PLANQ_DEPTH)) begin
              planq_data_d[planq_count_d] = commit_plan_data;
              planq_plan_count_d[planq_count_d] = commit_plan_count;
              planq_remove_count_d[planq_count_d] = commit_remove_count_o;
              planq_count_d = planq_count_d + PLANQ_COUNT_W'(1);
            end
          end
            done_d = 1'b1;
            st_d   = ST_IDLE;
          end
        end

        ST_DONE_EMPTY: begin
          st_d = ST_IDLE;
        end

        default: st_d = ST_IDLE;
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q                  <= ST_IDLE;
      c2_snap_q             <= '0;
      c3_snap_q             <= '0;
      round_head_q          <= '{default: '0};
      round_active_count_q  <= '0;
      round_total_conc_q    <= '0;
      c2_slot_q             <= '0;
      c3_slot_q             <= '0;
      eval_inflight_q       <= 1'b0;
      c2_ghost_bw_ok_q      <= 1'b0;
      done_q                <= 1'b0;
      remove_valid_q        <= 1'b0;
      remove_count_q        <= '0;
      remove_slot_mask_q    <= '0;
      planq_count_q         <= '0;
      tail_pending_valid_q   <= '0;
      tail_pending_skip_s1_q <= '0;
      tail_pending_slot_q    <= '{default: '0};
      for (int q = 0; q < PLANQ_DEPTH; q++) begin
        planq_data_q[q]       <= '{default: '0};
        planq_plan_count_q[q] <= '0;
        planq_remove_count_q[q] <= '0;
      end
    end else begin
      st_q                  <= st_d;
      c2_snap_q             <= c2_snap_d;
      c3_snap_q             <= c3_snap_d;
      round_head_q          <= round_head_d;
      round_active_count_q  <= round_active_count_d;
      round_total_conc_q    <= round_total_conc_d;
      c2_slot_q             <= c2_slot_d;
      c3_slot_q             <= c3_slot_d;
      eval_inflight_q       <= eval_inflight_d;
      c2_ghost_bw_ok_q      <= c2_ghost_bw_ok_d;
      done_q                <= done_d;
      remove_valid_q        <= remove_valid_d;
      remove_count_q        <= remove_count_d;
      remove_slot_mask_q    <= remove_slot_mask_d;
      planq_count_q         <= planq_count_d;
      tail_pending_valid_q   <= tail_pending_valid_d;
      tail_pending_skip_s1_q <= tail_pending_skip_s1_d;
      tail_pending_slot_q    <= tail_pending_slot_d;
      for (int q = 0; q < PLANQ_DEPTH; q++) begin
        planq_data_q[q]       <= planq_data_d[q];
        planq_plan_count_q[q] <= planq_plan_count_d[q];
        planq_remove_count_q[q] <= planq_remove_count_d[q];
      end
    end
  end

  assign remove_valid_o     = remove_valid_q;
  assign remove_count_o     = remove_count_q;
  assign remove_slot_mask_o = remove_slot_mask_q;

  assign plan_valid_o       = (planq_count_q != 2'd0);
  for (genvar q = 0; q < PLANQ_DEPTH; q++) begin : gen_planq_out
    assign plan_slot_valid_o[q] = (planq_plan_count_q[q] == 2'd0) ? 2'b00 :
                                  (planq_plan_count_q[q] == 2'd1) ? 2'b01 : 2'b11;
    assign plan_data_o[q] = planq_data_q[q];
    assign plan_count_o[q] = planq_plan_count_q[q];
    assign plan_remove_count_o[q] = planq_remove_count_q[q];
  end
  assign plan_queue_full_o  = (planq_count_q == PLANQ_COUNT_W'(PLANQ_DEPTH));
  assign plan_queue_count_o = {{(4-PLANQ_COUNT_W){1'b0}}, planq_count_q};

  assign busy_o = (st_q != ST_IDLE) ||
                  (remove_valid_q && !remove_ready_i) ||
                  ((planq_count_q == PLANQ_COUNT_W'(PLANQ_DEPTH)) &&
                   (planq_pop_count_eff == PLANQ_COUNT_W'(0)));
  assign done_o = done_q;
  assign makespan_o = (c2_snap_q.task_end > c3_snap_q.task_end) ?
                      c2_snap_q.task_end : c3_snap_q.task_end;

endmodule
