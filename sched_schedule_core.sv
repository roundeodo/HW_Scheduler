// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — one-round datapath core
//
// This is the datapath-only core under the MMIO wrapper:
//   wrapper-maintained top4/reserve state -> token candidate_generator
//   -> token-decode candidate_eval_lane -> compact best_reduce -> replay(best_token)
//   -> core-local commit/S4PF check -> remove metadata + depth-4 round FIFO
//
// There is intentionally no internal rem_sram, plan_sram, AXI master, or DMA
// writer in this module.  The full rem stream and final lowered args live
// outside this core; the wrapper only exposes register/FIFO state to CVA6.

import sched_pkg::*;
import sched_candidate_pkg::*;

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
  input  logic [$clog2(PLANQ_DEPTH)-1:0] plan_rd_entry_i,
  input  logic                         plan_rd_slot_i,
  output logic                         plan_valid_o,
  output logic [63:0]                  plan_rd_data_o,
  output logic [PLANQ_DEPTH-1:0][1:0]  plan_count_o,
  output logic                         plan_queue_full_o,
  output logic [3:0]                   plan_queue_count_o,

  output logic                         busy_o,
  output logic                         done_o
);

  localparam int unsigned PLANQ_COUNT_W = $clog2(PLANQ_DEPTH + 1);
  localparam int unsigned PLANQ_PTR_W   = $clog2(PLANQ_DEPTH);

  typedef enum logic [3:0] {
    ST_IDLE,
    ST_ROUND_START,
    ST_EVAL_START,
    ST_EVAL_ISSUE,
    ST_EVAL_WAIT,
    ST_REPLAY_START,
    ST_REPLAY_WAIT,
    ST_COMMIT_S4PF_A_START,
    ST_COMMIT_S4PF_A_WAIT,
    ST_COMMIT_S4PF_B_START,
    ST_COMMIT_S4PF_B_WAIT,
    ST_COMMIT_APPLY,
    ST_DONE_EMPTY
  } state_t;

  state_t st_q, st_d;

  // Persistent per-cluster state is split by consumer class.  Timeline feeds
  // BW/S2PF/commit timing, while cache feeds hit logic.  Do not recreate a
  // full snap struct here; that was the old high-fanout boundary.
  typedef struct packed {
    logic  valid;
    time_t task_start;
    time_t task_end;
    time_t dma1_end;
    time_t s2_end;
    time_t dma3_end;
    time_t s4_start;
    bw_t   bw_s1;
    bw_t   bw_s3;
    logic  s2pf_valid;
    time_t s2pf_start;
    time_t s2pf_end;
    bw_t   s2pf_bw;
    logic  s4pf_valid;
  } cluster_timeline_state_t;

  cluster_timeline_state_t c2_timeline_q, c2_timeline_d;
  cluster_timeline_state_t c3_timeline_q, c3_timeline_d;
  snap_timeline_t c2_timeline_view;
  snap_timeline_t c3_timeline_view;
  snap_cache_t    c2_cache_q, c2_cache_d;
  snap_cache_t    c3_cache_q, c3_cache_d;
  slot_id_t   c2_slot_q, c2_slot_d;
  slot_id_t   c3_slot_q, c3_slot_d;

  logic done_q, done_d;
  early_start_ctx_t early_ctx_q, early_ctx_d;

  logic              remove_valid_q, remove_valid_d;

  logic [PLANQ_COUNT_W-1:0] planq_count_q, planq_count_d;
  logic [PLANQ_PTR_W-1:0]   planq_head_q, planq_head_d;
  logic [PLANQ_COUNT_W-1:0] planq_count_after_pop;
  logic [PLANQ_DEPTH-1:0][1:0][63:0] planq_data_q;
  logic [PLANQ_DEPTH-1:0]   planq_is_two_q;
  logic [PLANQ_PTR_W-1:0]   planq_rd_idx;
  logic [PLANQ_PTR_W-1:0]   planq_tail_idx;
  logic                     planq_push;
  logic              planq_has_space_after_pop;
  logic              remove_free_after_ready;
  logic [1:0]        tail_pending_valid_q, tail_pending_valid_d;
  logic [1:0]        tail_pending_skip_s1_q, tail_pending_skip_s1_d;
  slot_id_t [1:0]    tail_pending_slot_q, tail_pending_slot_d;
  logic [1:0][7:0]   commit_inline_patch;
  logic [1:0][63:0]  commit_plan_word;

  function automatic logic [7:0] pack_inline_patch_byte(
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
      pack_inline_patch_byte = word;
    end
  endfunction

  function automatic snap_cache_t make_initial_cache(input logic [7:0] cache_eid);
    snap_cache_t s;
    s = '0;
    s.pf_eid = PF_EID_NONE;
    if (!cache_eid[7]) begin
      s.pf_eid  = encode_eid(cache_eid[EID_RAW_W-1:0]);
      s.pf_end  = '0;
      s.pf_full = 1'b1;
    end
    make_initial_cache = s;
  endfunction

  function automatic cluster_timeline_state_t timeline_to_state(
    input snap_timeline_t t
  );
    cluster_timeline_state_t s;
    begin
      s.valid       = t.valid;
      s.task_start  = t.task_start;
      s.task_end    = t.task_end;
      s.dma1_end    = t.dma1_end;
      s.s2_end      = t.s2_end;
      s.dma3_end    = t.dma3_end;
      s.s4_start    = t.s4_start;
      s.bw_s1       = t.bw_s1;
      s.bw_s3       = t.bw_s3;
      s.s2pf_valid  = t.s2pf_valid;
      s.s2pf_start  = t.s2pf_start;
      s.s2pf_end    = t.s2pf_end;
      s.s2pf_bw     = t.s2pf_bw;
      s.s4pf_valid  = t.s4pf_valid;
      timeline_to_state = s;
    end
  endfunction

  function automatic snap_timeline_t state_to_timeline(
    input cluster_timeline_state_t s
  );
    snap_timeline_t t;
    begin
      t = '0;
      t.valid       = s.valid;
      t.task_start  = s.task_start;
      t.task_end    = s.task_end;
      t.dma1_end    = s.dma1_end;
      t.s2_end      = s.s2_end;
      t.dma3_end    = s.dma3_end;
      t.s4_start    = s.s4_start;
      t.bw_s1       = s.bw_s1;
      t.bw_s3       = s.bw_s3;
      t.s2pf_valid  = s.s2pf_valid;
      t.s2pf_start  = s.s2pf_start;
      t.s2pf_end    = s.s2pf_end;
      t.s2pf_bw     = s.s2pf_bw;
      t.s4pf_valid  = s.s4pf_valid;
      t.s4pf_start  = s.s4_start;
      state_to_timeline = t;
    end
  endfunction

  assign c2_timeline_view = state_to_timeline(c2_timeline_q);
  assign c3_timeline_view = state_to_timeline(c3_timeline_q);

  function automatic early_start_ctx_t make_early_start_ctx(
    input snap_timeline_t busy_timeline,
    input time_t          idle_t
  );
    early_start_ctx_t e;
    logic [3:0] rel_valid;
    time_t rel_t [4];
    begin
      e = '0;
      rel_valid[0] = busy_timeline.valid && (busy_timeline.bw_s1 != BW_0);
      rel_t[0]     = busy_timeline.dma1_end;
      rel_valid[1] = busy_timeline.valid && busy_timeline.s2pf_valid;
      rel_t[1]     = busy_timeline.s2pf_end;
      rel_valid[2] = busy_timeline.valid && (busy_timeline.bw_s3 != BW_0);
      rel_t[2]     = busy_timeline.dma3_end;
      rel_valid[3] = busy_timeline.valid && busy_timeline.s4pf_valid;
      rel_t[3]     = busy_timeline.s4pf_start + GHOST_WINDOW_TICKS;

      rel_valid[0] &= (rel_t[0] > idle_t);
      rel_valid[1] &= (rel_t[1] > idle_t);
      rel_valid[2] &= (rel_t[2] > idle_t);
      rel_valid[3] &= (rel_t[3] > idle_t);

      // Fixed priority extraction preserves the old endpoint order without a
      // count-dependent append chain.  Only the first two distinct endpoints
      // are consumed by the hardware-lite early-start policy.
      if (rel_valid[0]) begin
        e.count = 2'd1;
        e.t0 = rel_t[0];
        if (rel_valid[1] && (rel_t[1] != rel_t[0])) begin
          e.count = 2'd2;
          e.t1 = rel_t[1];
        end else if (rel_valid[2] && (rel_t[2] != rel_t[0])) begin
          e.count = 2'd2;
          e.t1 = rel_t[2];
        end else if (rel_valid[3] && (rel_t[3] != rel_t[0])) begin
          e.count = 2'd2;
          e.t1 = rel_t[3];
        end
      end else if (rel_valid[1]) begin
        e.count = 2'd1;
        e.t0 = rel_t[1];
        if (rel_valid[2] && (rel_t[2] != rel_t[1])) begin
          e.count = 2'd2;
          e.t1 = rel_t[2];
        end else if (rel_valid[3] && (rel_t[3] != rel_t[1])) begin
          e.count = 2'd2;
          e.t1 = rel_t[3];
        end
      end else if (rel_valid[2]) begin
        e.count = 2'd1;
        e.t0 = rel_t[2];
        if (rel_valid[3] && (rel_t[3] != rel_t[2])) begin
          e.count = 2'd2;
          e.t1 = rel_t[3];
        end
      end else if (rel_valid[3]) begin
        e.count = 2'd1;
        e.t0 = rel_t[3];
      end

      make_early_start_ctx = e;
    end
  endfunction

  logic       eval_bw_start;
  snap_bw_view_t eval_bw_snap_a;
  snap_bw_view_t eval_bw_snap_b;
  logic       eval_bw_done;
  logic       eval_bw_checker_ok;

  logic       commit_bw_start;
  snap_timeline_t commit_bw_snap_a;
  snap_timeline_t commit_bw_snap_b;
  logic       commit_bw_done;
  logic       commit_bw_ok;

  // Two pointer-only BW checkers keep the input-stability contract local:
  // eval_lane holds its BW request stable while waiting, and the commit FSM
  // drives the same commit request through START/WAIT.  This removes the old
  // shared checker segment queues and avoids a wide shared snap mux.
  sched_bw_ok_seq i_eval_bw_ok (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clear_i  (init_i),
    .start_i  (eval_bw_start),
    .done_o   (eval_bw_done),
    .snap_a_i (eval_bw_snap_a),
    .snap_b_i (eval_bw_snap_b),
    .ok_o     (eval_bw_checker_ok)
  );

  sched_bw_ok_seq i_commit_bw_ok (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clear_i  (init_i),
    .start_i  (commit_bw_start),
    .done_o   (commit_bw_done),
    .snap_a_i (to_bw_view(commit_bw_snap_a)),
    .snap_b_i (to_bw_view(commit_bw_snap_b)),
    .ok_o     (commit_bw_ok)
  );

  // ── Candidate generation ──────────────────────────────────────────────
  logic       gen_start;
  logic       gen_advance;
  logic       gen_busy;
  logic       gen_done;
  cand_token_t cand_token;
  cand_token_t eval_token;
  cand_token_t best_token;
  logic [3:0] head_valid;
  early_start_ctx_t early_ctx_comb;
  time_t early_idle_t;
  snap_timeline_t early_busy_timeline;

  assign early_idle_t = (c3_timeline_q.task_end < c2_timeline_q.task_end) ?
                        c3_timeline_q.task_end : c2_timeline_q.task_end;
  assign early_busy_timeline = (c3_timeline_q.task_end < c2_timeline_q.task_end) ?
                               c2_timeline_view : c3_timeline_view;
  assign early_ctx_comb = make_early_start_ctx(early_busy_timeline, early_idle_t);
  assign head_valid = {head_i[3].valid, head_i[2].valid,
                       head_i[1].valid, head_i[0].valid};

  logic [1:0] best_remove_count;
  logic [3:0] best_remove_slot_mask;
  assign eval_token = (st_q == ST_REPLAY_START) ? best_token : cand_token;

  sched_candidate_generator i_candidate_generator (
    .clk_i                 (clk_i),
    .rst_ni                (rst_ni),
    .clear_i               (init_i),
    .start_i               (gen_start),
    .advance_i             (gen_advance),
    .busy_o                (gen_busy),
    .done_o                (gen_done),
    .both_idle_i           (c2_timeline_q.task_end == c3_timeline_q.task_end),
    .early_count_i         (early_ctx_q.count),
    .head_valid_i          (head_valid),
    .head0_ntok_i          (head_i[0].ntok),
    .active_count_i        (active_count_i),
    .cand_o                (cand_token)
  );

  // ── Candidate evaluation lane ─────────────────────────────────────────
  logic       eval_valid;
  logic       eval_start;
  logic       eval_done;
  score_key_t eval_score;
  winner_plan_t eval_plan;
  snap_timeline_t eval_timeline_a;
  snap_timeline_t eval_timeline_b;
  snap_cache_t    eval_cache_a;
  snap_cache_t    eval_cache_b;

  sched_candidate_eval_lane i_eval_lane (
    .clk_i                 (clk_i),
    .rst_ni                (rst_ni),
    .clear_i               (init_i),
    .start_i               (eval_start),
    .done_o                (eval_done),
    .cand_i                (eval_token),
    .head_i                (head_i),
    .active_count_i        (active_count_i),
    .total_conc_i          (total_conc_i),
    .early_i               (early_ctx_q),
    .base_timeline_a_i     (c2_timeline_view),
    .base_timeline_b_i     (c3_timeline_view),
    .base_cache_a_i        (c2_cache_q),
    .base_cache_b_i        (c3_cache_q),
    .bw_start_o            (eval_bw_start),
    .bw_snap_a_o           (eval_bw_snap_a),
    .bw_snap_b_o           (eval_bw_snap_b),
    .bw_done_i             (((st_q == ST_EVAL_WAIT) || (st_q == ST_REPLAY_WAIT)) ? eval_bw_done : 1'b0),
    .bw_ok_i               (eval_bw_checker_ok),
    .eval_valid_o          (eval_valid),
    .score_key_o           (eval_score),
    .winner_plan_o         (eval_plan),
    .snap_timeline_a_o     (eval_timeline_a),
    .snap_timeline_b_o     (eval_timeline_b),
    .snap_cache_a_o        (eval_cache_a),
    .snap_cache_b_o        (eval_cache_b)
  );

  // ── Best reducer: compact winner only ─────────────────────────────────
  logic best_clear;

  sched_best_reduce i_best_reduce (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .clear_i              (best_clear || init_i),
    .cand_valid_i         ((st_q == ST_EVAL_WAIT) && eval_valid),
    .cand_token_i         (cand_token),
    .cand_score_i         (eval_score),
    .best_token_o         (best_token),
    .best_remove_count_o  (best_remove_count),
    .best_remove_slot_mask_o (best_remove_slot_mask)
  );

  // ── Replay commit and output-buffer producer ──────────────────────────
  logic allow_s4pf_a_q, allow_s4pf_a_d;
  logic allow_s4pf_b_q, allow_s4pf_b_d;
  winner_plan_t commit_plan;
  snap_timeline_t commit_timeline_a;
  snap_timeline_t commit_timeline_b;
  snap_cache_t    commit_cache_a;
  snap_cache_t    commit_cache_b;
  snap_timeline_t commit_s4pf_timeline_a;
  snap_timeline_t commit_s4pf_timeline_b;
  snap_timeline_t commit_timeline_a_after_s4pf;
  logic commit_s4pf_candidate_a;
  logic commit_s4pf_candidate_b;

  function automatic task_desc_t make_task_desc(
    input winner_token_t token,
    input task_control_t ctrl
  );
    task_desc_t d;
    begin
      d = '0;
      d.cluster   = ctrl.cluster;
      d.eid       = token.eid;
      d.ntok      = token.ntok;
      d.tok_start = token.tok_start;
      d.s1        = ctrl.s1;
      d.s3        = ctrl.s3;
      d.skip_s1   = ctrl.skip_s1;
      d.skip_s3   = ctrl.skip_s3;
      d.has_s2pf  = ctrl.has_s2pf;
      make_task_desc = d;
    end
  endfunction

  assign commit_plan       = eval_plan;
  assign commit_timeline_a = eval_timeline_a;
  assign commit_timeline_b = eval_timeline_b;
  assign commit_cache_a    = eval_cache_a;
  assign commit_cache_b    = eval_cache_b;

  task_desc_t [1:0] commit_task_desc;
  logic [1:0]        commit_plan_allow_s4pf;
  logic [1:0]        commit_plan_count;
  slot_id_t [1:0]    commit_local_slot;
  slot_id_t          c2_slot_after_commit;
  slot_id_t          c3_slot_after_commit;

  for (genvar pp = 0; pp < 2; pp++) begin : gen_plan_pack
    sched_plan_pack i_plan_pack (
      .task_i         (commit_task_desc[pp]),
      .local_slot_i   (commit_local_slot[pp]),
      .inline_patch_i (commit_inline_patch[pp]),
      .word_o         (commit_plan_word[pp])
    );
  end

  assign commit_s4pf_candidate_a =
      commit_timeline_a.valid &&
      (commit_cache_a.pf_eid == PF_EID_NONE) &&
      (commit_timeline_a.dma1_end <= commit_timeline_a.s4_start) &&
      ((commit_timeline_a.s4_start + GHOST_WINDOW_TICKS) <=
       commit_timeline_a.task_end);

  assign commit_s4pf_candidate_b =
      commit_timeline_b.valid &&
      (commit_cache_b.pf_eid == PF_EID_NONE) &&
      (commit_timeline_b.dma1_end <= commit_timeline_b.s4_start) &&
      ((commit_timeline_b.s4_start + GHOST_WINDOW_TICKS) <=
       commit_timeline_b.task_end);

  always_comb begin
    commit_s4pf_timeline_a = commit_timeline_a;
    commit_s4pf_timeline_b = commit_timeline_b;
    if (commit_s4pf_candidate_a) begin
      commit_s4pf_timeline_a.s4pf_valid = 1'b1;
      commit_s4pf_timeline_a.s4pf_start = commit_timeline_a.s4_start;
    end
    if (commit_s4pf_candidate_b) begin
      commit_s4pf_timeline_b.s4pf_valid = 1'b1;
      commit_s4pf_timeline_b.s4pf_start = commit_timeline_b.s4_start;
    end
  end

  assign commit_timeline_a_after_s4pf =
      allow_s4pf_a_q ? commit_s4pf_timeline_a : commit_timeline_a;

  always_comb begin
    commit_task_desc       = '{default: '0};
    commit_plan_allow_s4pf = '0;
    commit_plan_count = {1'b0, commit_plan.valid[0]} +
                        {1'b0, commit_plan.valid[1]};
    for (int i = 0; i < 2; i++) begin
      if (commit_plan.valid[i]) begin
        commit_task_desc[i] = make_task_desc(commit_plan.token[i], commit_plan.ctrl[i]);
        commit_plan_allow_s4pf[i] =
            commit_plan.ctrl[i].cluster ? allow_s4pf_b_q : allow_s4pf_a_q;
      end
    end
  end

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

    pend_valid   = tail_pending_valid_q;
    pend_skip_s1 = tail_pending_skip_s1_q;
    pend_slot    = tail_pending_slot_q;

    for (int s = 0; s < 2; s++) begin
      int ci;
      logic cache_hit;
      logic no_copy;
      ci        = 0;
      cache_hit = 1'b0;
      no_copy   = 1'b0;
      if (s < int'(commit_plan_count)) begin
        ci = int'(commit_task_desc[s].cluster);
        cache_hit = (ci == 0) ?
                    (!cache_eid_c2_i[7] &&
                     (cache_eid_c2_i[EID_RAW_W-1:0] == commit_task_desc[s].eid)) :
                    (!cache_eid_c3_i[7] &&
                     (cache_eid_c3_i[EID_RAW_W-1:0] == commit_task_desc[s].eid));
        no_copy = pend_skip_s1[ci] && cache_hit;

        if (pend_valid[ci]) begin
          commit_inline_patch[s] = pack_inline_patch_byte(1'b1, no_copy, pend_slot[ci]);
          pend_valid[ci]   = 1'b0;
          pend_skip_s1[ci] = 1'b0;
          pend_slot[ci]    = '0;
        end

        if (commit_plan_allow_s4pf[s]) begin
          pend_valid[ci]   = 1'b1;
          pend_skip_s1[ci] = commit_task_desc[s].skip_s1;
          pend_slot[ci]    = commit_local_slot[s];
        end
      end
    end

    tail_pending_valid_d   = tail_pending_valid_q;
    tail_pending_skip_s1_d = tail_pending_skip_s1_q;
    tail_pending_slot_d    = tail_pending_slot_q;

    if (init_i) begin
      tail_pending_valid_d   = '0;
      tail_pending_skip_s1_d = '0;
      tail_pending_slot_d    = '{default: '0};
    end else if (planq_push) begin
      tail_pending_valid_d   = pend_valid;
      tail_pending_skip_s1_d = pend_skip_s1;
      tail_pending_slot_d    = pend_slot;
    end
  end

  // ── Control FSM ───────────────────────────────────────────────────────
  always_comb begin
    st_d                 = st_q;
    c2_timeline_d        = c2_timeline_q;
    c3_timeline_d        = c3_timeline_q;
    c2_cache_d           = c2_cache_q;
    c3_cache_d           = c3_cache_q;
    c2_slot_d            = c2_slot_q;
    c3_slot_d            = c3_slot_q;
    early_ctx_d          = early_ctx_q;
    done_d               = 1'b0;
    allow_s4pf_a_d       = allow_s4pf_a_q;
    allow_s4pf_b_d       = allow_s4pf_b_q;

    remove_valid_d        = remove_valid_q;
    planq_count_d         = planq_count_q;
    planq_head_d          = planq_head_q;
    planq_push            = 1'b0;

    gen_start   = 1'b0;
    gen_advance = 1'b0;
    eval_start  = 1'b0;
    best_clear  = 1'b0;
    commit_bw_start = 1'b0;
    commit_bw_snap_a = commit_timeline_a;
    commit_bw_snap_b = commit_timeline_b;

    remove_free_after_ready = !remove_valid_q || remove_ready_i;
    planq_count_after_pop = planq_count_q - PLANQ_COUNT_W'(plan_pop_count_i);
    planq_has_space_after_pop =
        (planq_count_after_pop < PLANQ_COUNT_W'(PLANQ_DEPTH));

    if (remove_ready_i) begin
      remove_valid_d = 1'b0;
    end

    // CVA6 pops the oldest queued plan entries after reading them.  The FIFO is
    // circular: pop only advances the head pointer and never shifts wide data.
    // Pop is handled before an internal commit push, so pop+push in the same
    // cycle is safe.
    if (plan_pop_count_i != 3'd0) begin
      planq_head_d  = planq_head_q + plan_pop_count_i[PLANQ_PTR_W-1:0];
      planq_count_d = planq_count_after_pop;
    end

    if (init_i) begin
      c2_timeline_d        = '0;
      c3_timeline_d        = '0;
      c2_cache_d           = make_initial_cache(cache_eid_c2_i);
      c3_cache_d           = make_initial_cache(cache_eid_c3_i);
      c2_slot_d            = '0;
      c3_slot_d            = '0;
      early_ctx_d          = '0;
      allow_s4pf_a_d       = 1'b0;
      allow_s4pf_b_d       = 1'b0;
      remove_valid_d       = 1'b0;
      planq_count_d        = '0;
      planq_head_d         = '0;
      // Batch initialization and first-round start may arrive in the same
      // control write.  Init still clears all persistent round state first;
      // start_i then arms the freshly initialized core for the first round.
      st_d                 = start_i ? ST_ROUND_START : ST_IDLE;
    end else begin
      unique case (st_q)
        ST_IDLE: begin
          if (start_i && remove_free_after_ready && planq_has_space_after_pop) begin
            st_d            = ST_ROUND_START;
          end
        end

        ST_ROUND_START: begin
          // ST_ROUND_START is the round-level predecode boundary after wrapper
          // compact/refill.  Only early_ctx is latched because it is derived
          // from C2/C3 timeline and consumed by both generator and eval lane.
          early_ctx_d = early_ctx_comb;
          if (active_count_i == NR_W'(0)) begin
            done_d = 1'b1;
            st_d   = ST_DONE_EMPTY;
          end else begin
            st_d = ST_EVAL_START;
          end
        end

        ST_EVAL_START: begin
          best_clear      = 1'b1;
          gen_start       = 1'b1;
          st_d            = ST_EVAL_ISSUE;
        end

        ST_EVAL_ISSUE: begin
          if (gen_busy) begin
            eval_start = 1'b1;
            st_d       = ST_EVAL_WAIT;
          end else if (gen_done) begin
            st_d = ST_REPLAY_START;
          end
        end

        ST_EVAL_WAIT: begin
          if (eval_done) begin
            gen_advance = 1'b1;
            st_d        = ST_EVAL_ISSUE;
          end
        end

        ST_REPLAY_START: begin
          // Replay the compact best token directly through eval_lane.  The
          // generator is not involved in replay, so no wide candidate payload
          // or duplicate generator path is kept.
          eval_start       = 1'b1;
          st_d             = ST_REPLAY_WAIT;
        end

        ST_REPLAY_WAIT: begin
          if (eval_done) begin
            allow_s4pf_a_d   = 1'b0;
            allow_s4pf_b_d   = 1'b0;
            st_d             = ST_COMMIT_S4PF_A_START;
          end
        end

        ST_COMMIT_S4PF_A_START: begin
          if (commit_s4pf_candidate_a) begin
            commit_bw_start = 1'b1;
            commit_bw_snap_a = commit_s4pf_timeline_a;
            commit_bw_snap_b = commit_timeline_b;
            st_d = ST_COMMIT_S4PF_A_WAIT;
          end else begin
            allow_s4pf_a_d = 1'b0;
            st_d = ST_COMMIT_S4PF_B_START;
          end
        end

        ST_COMMIT_S4PF_A_WAIT: begin
          commit_bw_snap_a = commit_s4pf_timeline_a;
          commit_bw_snap_b = commit_timeline_b;
          if (commit_bw_done) begin
            allow_s4pf_a_d = commit_bw_ok;
            st_d = ST_COMMIT_S4PF_B_START;
          end
        end

        ST_COMMIT_S4PF_B_START: begin
          if (commit_s4pf_candidate_b) begin
            commit_bw_start = 1'b1;
            // Preserve sequential C2-then-C3 S4PF semantics: C3 is checked
            // against C2 after the accepted C2 ghost window.
            commit_bw_snap_a = commit_timeline_a_after_s4pf;
            commit_bw_snap_b = commit_s4pf_timeline_b;
            st_d = ST_COMMIT_S4PF_B_WAIT;
          end else begin
            allow_s4pf_b_d = 1'b0;
            st_d = ST_COMMIT_APPLY;
          end
        end

        ST_COMMIT_S4PF_B_WAIT: begin
          commit_bw_snap_a = commit_timeline_a_after_s4pf;
          commit_bw_snap_b = commit_s4pf_timeline_b;
          if (commit_bw_done) begin
            allow_s4pf_b_d = commit_bw_ok;
            st_d = ST_COMMIT_APPLY;
          end
        end

        ST_COMMIT_APPLY: begin
          c2_timeline_d = timeline_to_state(
              allow_s4pf_a_q ? commit_s4pf_timeline_a : commit_timeline_a);
          c3_timeline_d = timeline_to_state(
              allow_s4pf_b_q ? commit_s4pf_timeline_b : commit_timeline_b);
          c2_cache_d    = commit_cache_a;
          c3_cache_d    = commit_cache_b;
          if (allow_s4pf_a_q) begin
            c2_cache_d.pf_eid  = PF_EID_GHOST;
            c2_cache_d.pf_end  = commit_timeline_a.task_end;
            c2_cache_d.pf_full = 1'b0;
          end
          if (allow_s4pf_b_q) begin
            c3_cache_d.pf_eid  = PF_EID_GHOST;
            c3_cache_d.pf_end  = commit_timeline_b.task_end;
            c3_cache_d.pf_full = 1'b0;
          end

          c2_slot_d       = c2_slot_after_commit;
          c3_slot_d       = c3_slot_after_commit;
          remove_valid_d  = 1'b1;
          planq_push      = 1'b1;
          planq_count_d   = planq_count_d + PLANQ_COUNT_W'(1);
          allow_s4pf_a_d  = 1'b0;
          allow_s4pf_b_d  = 1'b0;
          done_d = 1'b1;
          st_d   = ST_IDLE;
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
      c2_timeline_q         <= '0;
      c3_timeline_q         <= '0;
      c2_cache_q            <= '0;
      c3_cache_q            <= '0;
      c2_slot_q             <= '0;
      c3_slot_q             <= '0;
      early_ctx_q           <= '0;
      allow_s4pf_a_q        <= 1'b0;
      allow_s4pf_b_q        <= 1'b0;
      done_q                <= 1'b0;
      remove_valid_q        <= 1'b0;
      planq_count_q         <= '0;
      planq_head_q          <= '0;
      tail_pending_valid_q   <= '0;
      tail_pending_skip_s1_q <= '0;
      tail_pending_slot_q    <= '{default: '0};
    end else begin
      st_q                  <= st_d;
      c2_timeline_q         <= c2_timeline_d;
      c3_timeline_q         <= c3_timeline_d;
      c2_cache_q            <= c2_cache_d;
      c3_cache_q            <= c3_cache_d;
      c2_slot_q             <= c2_slot_d;
      c3_slot_q             <= c3_slot_d;
      early_ctx_q           <= early_ctx_d;
      allow_s4pf_a_q        <= allow_s4pf_a_d;
      allow_s4pf_b_q        <= allow_s4pf_b_d;
      done_q                <= done_d;
      remove_valid_q        <= remove_valid_d;
      planq_count_q         <= planq_count_d;
      planq_head_q          <= planq_head_d;
      tail_pending_valid_q   <= tail_pending_valid_d;
      tail_pending_skip_s1_q <= tail_pending_skip_s1_d;
      tail_pending_slot_q    <= tail_pending_slot_d;
      if (planq_push) begin
        planq_data_q[planq_tail_idx] <= commit_plan_word;
        planq_is_two_q[planq_tail_idx] <= commit_plan_count[1];
      end
    end
  end

  assign remove_valid_o     = remove_valid_q;
  // best_reduce is held until the next ST_EVAL_START.  The wrapper consumes
  // these derived fields on the immediate remove_valid/ready handshake, so no
  // second metadata register bank is required.
  assign remove_count_o     = remove_valid_q ? best_remove_count : 2'd0;
  assign remove_slot_mask_o = remove_valid_q ? best_remove_slot_mask : 4'b0000;

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni && remove_valid_q) begin
      assert (best_token.valid && cand_remove_mask_legal(best_remove_slot_mask))
        else $error("sched_schedule_core produced illegal remove_slot_mask=%b",
                    best_remove_slot_mask);
    end
  end
`endif

  assign plan_valid_o = (planq_count_q != PLANQ_COUNT_W'(0));
  // For a circular FIFO tail == head + occupancy (mod depth); storing a
  // second pointer would duplicate this invariant.
  assign planq_tail_idx = planq_head_q + planq_count_q[PLANQ_PTR_W-1:0];
  for (genvar q = 0; q < PLANQ_DEPTH; q++) begin : gen_planq_out
    localparam logic [PLANQ_COUNT_W-1:0] Q_COUNT = PLANQ_COUNT_W'(q);
    localparam logic [PLANQ_PTR_W-1:0]   Q_PTR   = PLANQ_PTR_W'(q);
    logic [PLANQ_PTR_W-1:0] count_rd_idx;

    assign count_rd_idx = planq_head_q + Q_PTR;
    assign plan_count_o[q] = (Q_COUNT >= planq_count_q) ? 2'd0 :
                             planq_is_two_q[count_rd_idx] ? 2'd2 : 2'd1;
  end
  assign planq_rd_idx = planq_head_q + plan_rd_entry_i;
  assign plan_rd_data_o =
      (PLANQ_COUNT_W'(plan_rd_entry_i) >= planq_count_q) ? 64'd0 :
      (plan_rd_slot_i && !planq_is_two_q[planq_rd_idx]) ? 64'd0 :
      planq_data_q[planq_rd_idx][plan_rd_slot_i];
  assign plan_queue_full_o  = (planq_count_q == PLANQ_COUNT_W'(PLANQ_DEPTH));
  assign plan_queue_count_o = {{(4-PLANQ_COUNT_W){1'b0}}, planq_count_q};

  assign busy_o = (st_q != ST_IDLE) ||
                  (remove_valid_q && !remove_ready_i) ||
                  ((planq_count_q == PLANQ_COUNT_W'(PLANQ_DEPTH)) &&
                   (plan_pop_count_i == 3'd0));
  assign done_o = done_q;

endmodule
