// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — one-round datapath core
//
// This is the datapath-only core under the MMIO wrapper:
//   wrapper-maintained top4/reserve state -> token candidate_generator
//   -> token-decode candidate_eval_lane -> compact best_reduce -> replay(best_token)
//   -> core-local commit/S4PF check -> remove metadata + depth-8 task FIFO
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

  // Core-level remove handshake.  The wrapper accepts this event, compacts the
  // top window, refills it from reserve, and then starts the next round.
  input  logic                         remove_ready_i,
  output logic                         remove_valid_o,
  output logic [1:0]                   remove_count_o,
  output logic [3:0]                   remove_slot_mask_o,

  // Dense task FIFO drain path.  CVA6 reads contiguous 64-bit task words and
  // acknowledges exactly that many words with task_pop_count_i.
  input  logic [3:0]                   task_pop_count_i,
  input  logic [$clog2(TASKQ_DEPTH)-1:0] task_rd_index_i,
  output logic                         task_valid_o,
  output logic [63:0]                  task_rd_data_o,
  output logic                         task_queue_full_o,
  output logic [3:0]                   task_queue_count_o,

  output logic                         busy_o,
  output logic                         done_o
);

  localparam int unsigned TASKQ_COUNT_W = $clog2(TASKQ_DEPTH + 1);
  localparam int unsigned TASKQ_PTR_W   = $clog2(TASKQ_DEPTH);

  // Round control is intentionally split into four phases:
  //   enumerate/evaluate -> replay winner -> commit S4PF -> serialize tasks.
  // Replay avoids storing a wide winner snapshot; serialization keeps the
  // FIFO at one physical write port.
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
    ST_COMMIT_EMIT_PREV,
    ST_COMMIT_EMIT_CUR,
    ST_COMMIT_FINISH,
    ST_FLUSH_PENDING
  } state_t;

  state_t st_q, st_d;

  // Persistent per-cluster state is split by consumer class.  Timeline feeds
  // BW/S2PF/commit timing, while cache identity feeds hit logic.  Each consumer
  // receives only the view it needs, limiting wide cross-module fanout.
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

  logic [TASKQ_COUNT_W-1:0] taskq_count_q, taskq_count_d;
  logic [TASKQ_PTR_W-1:0]   taskq_head_q, taskq_head_d;
  logic [TASKQ_COUNT_W-1:0] taskq_count_after_pop;
  logic [TASKQ_DEPTH-1:0][63:0] taskq_data_q;
  logic [TASKQ_PTR_W-1:0]   taskq_rd_idx;
  logic [TASKQ_PTR_W-1:0]   taskq_tail_idx;
  logic                     taskq_push;
  logic [63:0]              taskq_push_data;
  logic                     taskq_has_space_after_pop;
  logic              remove_free_after_ready;

  // S4PF target eid belongs to the next task on the same cluster.  A pending
  // record owns exactly the information that must survive until that task is
  // known; it is not a second FIFO or a copy of cluster timeline state.
  typedef struct packed {
    task_desc_t desc;
    slot_id_t   local_slot;
  } pending_task_t;
  logic [1:0]          pending_valid_q, pending_valid_d;
  pending_task_t [1:0] pending_task_q, pending_task_d;
  logic                emit_idx_q, emit_idx_d;
  logic                emit_current_valid;
  task_desc_t          emit_current_task;
  slot_id_t            emit_current_slot;
  logic                emit_current_allow_s4pf;
  logic                emit_current_cluster;
  logic                emit_target_cache_hit;
  logic                emit_pending_no_copy;
  logic                round_finishes_batch;
  task_desc_t          pack_task;
  slot_id_t            pack_local_slot;
  logic [7:0]          pack_s4pf_desc;
  logic [63:0]         pack_word;

  function automatic logic [7:0] pack_s4pf_desc_byte(
    input logic     valid,
    input logic     no_copy,
    input logic [EID_RAW_W-1:0] target_eid
  );
    logic [7:0] word;
    begin
      word = '0;
      word[S4PF_DESC_VALID_LSB] = valid;
      word[S4PF_DESC_NO_COPY_LSB] = no_copy;
      word[S4PF_DESC_TARGET_EID_LSB +: EID_RAW_W] = target_eid;
      pack_s4pf_desc_byte = word;
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
      // S4PF always starts when the current down-weight DMA releases its lane.
      // Derive it instead of storing another persistent timestamp.
      t.s4pf_start  = s.dma3_end;
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

      // Fixed priority extraction follows the physical stage order.  Only the
      // first two distinct release endpoints are candidates in the lite policy.
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

  // Two pointer-only BW checkers keep the input-stability contract local.
  // Eval and commit are independent clients, so neither path needs a wide
  // arbitration mux or duplicated segment storage.
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
  logic [1:0]        commit_task_allow_s4pf;
  logic [1:0]        commit_task_count;
  slot_id_t [1:0]    commit_local_slot;
  slot_id_t          c2_slot_after_commit;
  slot_id_t          c3_slot_after_commit;

  // One packer feeds the single-write-port task FIFO.  Commit emission is
  // deliberately micro-sequenced instead of building a four-word write crossbar.
  sched_task_word_pack i_task_word_pack (
    .task_i       (pack_task),
    .local_slot_i (pack_local_slot),
    .s4pf_desc_i  (pack_s4pf_desc),
    .word_o       (pack_word)
  );

  assign commit_s4pf_candidate_a =
      commit_timeline_a.valid &&
      (commit_cache_a.pf_eid == PF_EID_NONE) &&
      ((commit_timeline_a.dma3_end + GHOST_WINDOW_TICKS) <=
       commit_timeline_a.task_end);

  assign commit_s4pf_candidate_b =
      commit_timeline_b.valid &&
      (commit_cache_b.pf_eid == PF_EID_NONE) &&
      ((commit_timeline_b.dma3_end + GHOST_WINDOW_TICKS) <=
       commit_timeline_b.task_end);

  always_comb begin
    commit_s4pf_timeline_a = commit_timeline_a;
    commit_s4pf_timeline_b = commit_timeline_b;
    if (commit_s4pf_candidate_a) begin
      commit_s4pf_timeline_a.s4pf_valid = 1'b1;
      commit_s4pf_timeline_a.s4pf_start = commit_timeline_a.dma3_end;
    end
    if (commit_s4pf_candidate_b) begin
      commit_s4pf_timeline_b.s4pf_valid = 1'b1;
      commit_s4pf_timeline_b.s4pf_start = commit_timeline_b.dma3_end;
    end
  end

  assign commit_timeline_a_after_s4pf =
      allow_s4pf_a_q ? commit_s4pf_timeline_a : commit_timeline_a;

  always_comb begin
    commit_task_desc       = '{default: '0};
    commit_task_allow_s4pf = '0;
    commit_task_count = {1'b0, commit_plan.valid[0]} +
                        {1'b0, commit_plan.valid[1]};
    for (int i = 0; i < 2; i++) begin
      if (commit_plan.valid[i]) begin
        commit_task_desc[i] = make_task_desc(commit_plan.token[i], commit_plan.ctrl[i]);
        commit_task_allow_s4pf[i] =
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
      if (s < int'(commit_task_count)) begin
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

  // Dense task emission keeps at most one unresolved task per cluster.  A task
  // that owns an S4PF window is published only after the next same-cluster eid
  // is known; its high byte then carries that target directly.
  always_comb begin
    emit_current_valid = ({1'b0, emit_idx_q} < commit_task_count);
    emit_current_task = commit_task_desc[emit_idx_q];
    emit_current_slot = commit_local_slot[emit_idx_q];
    emit_current_allow_s4pf = commit_task_allow_s4pf[emit_idx_q];
    emit_current_cluster = emit_current_task.cluster;
    emit_target_cache_hit = emit_current_cluster ?
        (!cache_eid_c3_i[7] &&
         (cache_eid_c3_i[EID_RAW_W-1:0] == emit_current_task.eid)) :
        (!cache_eid_c2_i[7] &&
         (cache_eid_c2_i[EID_RAW_W-1:0] == emit_current_task.eid));
    emit_pending_no_copy =
        pending_task_q[emit_current_cluster].desc.skip_s1 && emit_target_cache_hit;
    round_finishes_batch =
        (active_count_i == NR_W'(best_remove_count));

    pack_task       = '0;
    pack_local_slot = '0;
    pack_s4pf_desc  = '0;
    unique case (st_q)
      ST_COMMIT_EMIT_PREV: begin
        pack_task = pending_task_q[emit_current_cluster].desc;
        pack_local_slot = pending_task_q[emit_current_cluster].local_slot;
        pack_s4pf_desc = pack_s4pf_desc_byte(
            1'b1, emit_pending_no_copy, emit_current_task.eid);
      end
      ST_COMMIT_EMIT_CUR: begin
        pack_task       = emit_current_task;
        pack_local_slot = emit_current_slot;
      end
      ST_FLUSH_PENDING: begin
        pack_task       = pending_task_q[emit_idx_q].desc;
        pack_local_slot = pending_task_q[emit_idx_q].local_slot;
      end
      default: begin
      end
    endcase
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
    taskq_count_d         = taskq_count_q;
    taskq_head_d          = taskq_head_q;
    taskq_push            = 1'b0;
    taskq_push_data       = pack_word;
    pending_valid_d       = pending_valid_q;
    pending_task_d        = pending_task_q;
    emit_idx_d            = emit_idx_q;

    gen_start   = 1'b0;
    gen_advance = 1'b0;
    eval_start  = 1'b0;
    best_clear  = 1'b0;
    commit_bw_start = 1'b0;
    commit_bw_snap_a = commit_timeline_a;
    commit_bw_snap_b = commit_timeline_b;

    remove_free_after_ready = !remove_valid_q || remove_ready_i;
    taskq_count_after_pop = taskq_count_q - TASKQ_COUNT_W'(task_pop_count_i);
    taskq_has_space_after_pop =
        (taskq_count_after_pop < TASKQ_COUNT_W'(TASKQ_DEPTH));

    if (remove_ready_i) begin
      remove_valid_d = 1'b0;
    end

    // Pop advances only the circular FIFO head.  The producer writes at the
    // pre-pop occupancy, which is also the first freed slot on simultaneous
    // pop/push.  Wrapper protocol guarantees pop_count <= current occupancy.
    if (task_pop_count_i != 4'd0) begin
      taskq_head_d  = taskq_head_q + task_pop_count_i[TASKQ_PTR_W-1:0];
      taskq_count_d = taskq_count_after_pop;
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
      taskq_count_d        = '0;
      taskq_head_d         = '0;
      pending_valid_d      = '0;
      pending_task_d       = '{default: '0};
      emit_idx_d           = 1'b0;
      // Batch initialization and first-round start may arrive in the same
      // control write.  Init still clears all persistent round state first;
      // start_i then arms the freshly initialized core for the first round.
      st_d                 = start_i ? ST_ROUND_START : ST_IDLE;
    end else begin
      unique case (st_q)
        ST_IDLE: begin
          if (start_i && remove_free_after_ready && taskq_has_space_after_pop) begin
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
            st_d   = ST_IDLE;
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
          // eval_lane reconstructs task identity from token + round context.
          // Keep the persistent timeline/cache unchanged until every task of
          // this round has crossed the single-write-port emit sequence.
          emit_idx_d = 1'b0;
          st_d = ST_COMMIT_EMIT_PREV;
        end

        ST_COMMIT_EMIT_PREV: begin
          if (!emit_current_valid) begin
            st_d = ST_COMMIT_FINISH;
          end else if (!pending_valid_q[emit_current_cluster]) begin
            st_d = ST_COMMIT_EMIT_CUR;
          end else if (taskq_has_space_after_pop) begin
            taskq_push = 1'b1;
            taskq_count_d = taskq_count_d + TASKQ_COUNT_W'(1);
            pending_valid_d[emit_current_cluster] = 1'b0;
            st_d = ST_COMMIT_EMIT_CUR;
          end
        end

        ST_COMMIT_EMIT_CUR: begin
          if (emit_current_allow_s4pf) begin
            pending_valid_d[emit_current_cluster] = 1'b1;
            pending_task_d[emit_current_cluster].desc = emit_current_task;
            pending_task_d[emit_current_cluster].local_slot = emit_current_slot;
            if ((emit_idx_q + 1'b1) < commit_task_count) begin
              emit_idx_d = emit_idx_q + 1'b1;
              st_d = ST_COMMIT_EMIT_PREV;
            end else begin
              st_d = ST_COMMIT_FINISH;
            end
          end else if (taskq_has_space_after_pop) begin
            taskq_push = 1'b1;
            taskq_count_d = taskq_count_d + TASKQ_COUNT_W'(1);
            if ((emit_idx_q + 1'b1) < commit_task_count) begin
              emit_idx_d = emit_idx_q + 1'b1;
              st_d = ST_COMMIT_EMIT_PREV;
            end else begin
              st_d = ST_COMMIT_FINISH;
            end
          end
        end

        ST_COMMIT_FINISH: begin
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
          c2_slot_d = c2_slot_after_commit;
          c3_slot_d = c3_slot_after_commit;
          if (round_finishes_batch) begin
            emit_idx_d = 1'b0;
            st_d = ST_FLUSH_PENDING;
          end else begin
            remove_valid_d = 1'b1;
            allow_s4pf_a_d = 1'b0;
            allow_s4pf_b_d = 1'b0;
            done_d = 1'b1;
            st_d = ST_IDLE;
          end
        end

        ST_FLUSH_PENDING: begin
          if (!pending_valid_q[emit_idx_q] || taskq_has_space_after_pop) begin
            if (pending_valid_q[emit_idx_q]) begin
              taskq_push = 1'b1;
              taskq_count_d = taskq_count_d + TASKQ_COUNT_W'(1);
              pending_valid_d[emit_idx_q] = 1'b0;
            end
            if (emit_idx_q == 1'b0) begin
              emit_idx_d = 1'b1;
            end else begin
              remove_valid_d = 1'b1;
              allow_s4pf_a_d = 1'b0;
              allow_s4pf_b_d = 1'b0;
              done_d = 1'b1;
              st_d = ST_IDLE;
            end
          end
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
      taskq_count_q         <= '0;
      taskq_head_q          <= '0;
      pending_valid_q       <= '0;
      pending_task_q        <= '{default: '0};
      emit_idx_q            <= 1'b0;
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
      taskq_count_q         <= taskq_count_d;
      taskq_head_q          <= taskq_head_d;
      pending_valid_q       <= pending_valid_d;
      pending_task_q        <= pending_task_d;
      emit_idx_q            <= emit_idx_d;
      if (taskq_push) begin
        taskq_data_q[taskq_tail_idx] <= taskq_push_data;
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

  assign task_valid_o = (taskq_count_q != TASKQ_COUNT_W'(0));
  // Tail is derived from head+occupancy; no duplicate tail register is kept.
  assign taskq_tail_idx = taskq_head_q + taskq_count_q[TASKQ_PTR_W-1:0];
  assign taskq_rd_idx = taskq_head_q + task_rd_index_i;
  assign task_rd_data_o =
      (TASKQ_COUNT_W'(task_rd_index_i) >= taskq_count_q) ? 64'd0 :
      taskq_data_q[taskq_rd_idx];
  assign task_queue_full_o =
      (taskq_count_q == TASKQ_COUNT_W'(TASKQ_DEPTH));
  assign task_queue_count_o = taskq_count_q[3:0];

  assign busy_o = (st_q != ST_IDLE) ||
                  (remove_valid_q && !remove_ready_i) ||
                  ((taskq_count_q == TASKQ_COUNT_W'(TASKQ_DEPTH)) &&
                   (task_pop_count_i == 4'd0));
  assign done_o = done_q;

endmodule
