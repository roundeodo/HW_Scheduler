// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — one-round datapath core
//
// This is the datapath-only core under the MMIO wrapper:
//   wrapper-maintained top6/reserve6 state -> token candidate_generator
//   -> candidate evaluator -> compact best selection -> replay(selected candidate)
//   -> core-local commit/S4PF check -> remove metadata + depth-8 task FIFO
//
// There is intentionally no internal rem_sram, plan_sram, AXI master, or DMA
// writer in this module.  The full rem stream and final lowered args live
// outside this core; the wrapper only exposes register/FIFO state to CVA6.

import sched_pkg::*;
import sched_candidate_pkg::*;

module moe_scheduler_core (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  // init_i resets the persistent cluster timeline/cache state at the start of
  // a full scheduling run.  start_i computes exactly one scheduling round.
  input  logic                         init_i,
  input  logic                         start_i,
  input  pf_eid_t                      initial_cache_eid_c2_i,
  input  pf_eid_t                      initial_cache_eid_c3_i,

  // Per-round context supplied by CVA6/software from the L3 rem list.
  input  wire head_ctx_t [5:0]         head_i,
  input  logic [NR_W-1:0]              active_count_i,
  input  logic [T_W-1:0]               total_parallel_work_i,
  input  logic [T_W-1:0]               total_serial_work_i,

  // Core-level remove handshake.  The wrapper accepts this event, compacts the
  // top window, refills it from reserve, and then starts the next round.
  input  logic                         remove_ready_i,
  output logic                         remove_valid_o,
  output logic [1:0]                   remove_count_o,
  output logic [3:0]                   remove_slot_mask_o,
  output time_t                        remove_parallel_work_o,
  output time_t                        remove_serial_work_o,

  // Dense task FIFO streaming path.  A successful MMIO TASK_STREAM read
  // consumes exactly the current head word, so the core only needs a 1-bit pop.
  input  logic                         task_fifo_pop_i,
  output logic                         task_fifo_valid_o,
  output logic [63:0]                  task_fifo_read_data_o,
  output logic                         task_fifo_full_o,
  output logic [3:0]                   task_fifo_count_o,

  output logic                         busy_o
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
    ST_COMMIT_S4PF_C2_START,
    ST_COMMIT_S4PF_C2_WAIT,
    ST_COMMIT_S4PF_C3_START,
    ST_COMMIT_S4PF_C3_WAIT,
    ST_COMMIT_APPLY,
    ST_COMMIT_EMIT_PREV,
    ST_COMMIT_EMIT_CUR,
    ST_COMMIT_FINISH,
    ST_FLUSH_PENDING
  } state_t;

  state_t st_q, st_d;

  // Timeline and cache identity are separate persistent facts: bandwidth paths
  // only receive to_bw_view(timeline), while hit logic reads the compact cache.
  snap_timeline_t c2_timeline_q, c2_timeline_d;
  snap_timeline_t c3_timeline_q, c3_timeline_d;
  snap_cache_t    c2_cache_q, c2_cache_d;
  snap_cache_t    c3_cache_q, c3_cache_d;
  slot_id_t   c2_slot_q, c2_slot_d;
  slot_id_t   c3_slot_q, c3_slot_d;

  early_start_ctx_t early_ctx_q, early_ctx_d;

  logic              remove_valid_q, remove_valid_d;

  logic [TASKQ_COUNT_W-1:0] task_fifo_count_q, task_fifo_count_d;
  logic [TASKQ_PTR_W-1:0]   task_fifo_head_q, task_fifo_head_d;
  logic [TASKQ_COUNT_W-1:0] task_fifo_count_after_pop;
  logic [TASKQ_DEPTH-1:0][63:0] task_fifo_data_q;
  logic [TASKQ_PTR_W-1:0]   task_fifo_write_address;
  logic                     task_fifo_push;
  logic [63:0]              task_fifo_push_data;
  logic                     task_fifo_space_after_pop;
  logic                     remove_channel_available;

  // S4PF target eid belongs to the next task on the same cluster.  A pending
  // record owns exactly the information that must survive until that task is
  // known; it is not a second FIFO or a copy of cluster timeline state.
  typedef struct packed {
    task_desc_t desc;
    slot_id_t   local_slot;
  } pending_task_t;
  logic [1:0]          pending_valid_q, pending_valid_d;
  pending_task_t [1:0] pending_task_q, pending_task_d;
  logic                emit_entry_index_q, emit_entry_index_d;
  logic                emit_current_valid;
  task_desc_t          emit_current_task;
  slot_id_t            emit_current_slot;
  logic                current_task_s4pf_available;
  logic                emit_current_cluster;
  logic                emit_target_cache_hit;
  logic                pending_s4pf_no_copy;
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
    logic [7:0] descriptor_byte;
    begin
      descriptor_byte = '0;
      descriptor_byte[S4PF_DESC_VALID_LSB] = valid;
      descriptor_byte[S4PF_DESC_NO_COPY_LSB] = no_copy;
      descriptor_byte[S4PF_DESC_TARGET_EID_LSB +: EID_RAW_W] = target_eid;
      pack_s4pf_desc_byte = descriptor_byte;
    end
  endfunction

  function automatic snap_cache_t make_initial_cache(input pf_eid_t cache_eid);
    snap_cache_t cache;
    cache = '0;
    cache.pf_eid = cache_eid;
    if (cache_eid != PF_EID_NONE) begin
      cache.pf_end  = '0;
      cache.pf_full = 1'b1;
    end
    make_initial_cache = cache;
  endfunction

  function automatic early_start_ctx_t make_early_start_ctx(
    input snap_timeline_t busy_timeline,
    input time_t          idle_t
  );
    early_start_ctx_t early_context;
    logic s1_release_valid;
    logic stage3_release_valid;
    logic s4pf_release_valid;
    time_t s1_release;
    time_t stage3_release;
    time_t s4pf_release;
    begin
      early_context = '0;
      s1_release = busy_timeline.dma1_end;
      stage3_release = busy_timeline.s2pf_valid ? busy_timeline.s2pf_end :
                                                 busy_timeline.dma3_end;
      s4pf_release = busy_timeline.dma3_end + S4PF_DMA_TICKS;

      s1_release_valid = busy_timeline.valid &&
                         (busy_timeline.dma_s1 != DMA_NONE) &&
                         (s1_release > idle_t);
      // S2PF replaces S3 DMA on one side, so these two endpoints are one
      // mutually-exclusive physical release source.
      stage3_release_valid = busy_timeline.valid &&
                             (busy_timeline.s2pf_valid ||
                              (busy_timeline.dma_s3 != DMA_NONE)) &&
                             (stage3_release > idle_t);
      s4pf_release_valid = busy_timeline.valid && busy_timeline.s4pf_valid &&
                           (s4pf_release > idle_t);

      // one-idle policy has two extra release slots.  Without S4PF, retain the
      // first two physical releases.  With BOTH-DMA S4PF, always retain its
      // final release as slot 2; otherwise all early candidates could overlap
      // the lane-exclusive prefetch and leave the round without a legal token.
      if (s1_release_valid) begin
        early_context.release_count = 2'd1;
        early_context.first_release = s1_release;
        if (s4pf_release_valid && (s4pf_release != s1_release)) begin
          early_context.release_count = 2'd2;
          early_context.second_release = s4pf_release;
        end else if (stage3_release_valid && (stage3_release != s1_release)) begin
          early_context.release_count = 2'd2;
          early_context.second_release = stage3_release;
        end
      end else if (stage3_release_valid) begin
        early_context.release_count = 2'd1;
        early_context.first_release = stage3_release;
        if (s4pf_release_valid && (s4pf_release != stage3_release)) begin
          early_context.release_count = 2'd2;
          early_context.second_release = s4pf_release;
        end
      end else if (s4pf_release_valid) begin
        early_context.release_count = 2'd1;
        early_context.first_release = s4pf_release;
      end

      make_early_start_ctx = early_context;
    end
  endfunction

  logic       eval_bw_start;
  snap_bw_view_t eval_bw_snap_a;
  snap_bw_view_t eval_bw_snap_b;
  logic       eval_bw_done;
  logic       eval_bw_ok;

  logic       commit_bw_start;
  snap_bw_view_t commit_bw_view_c2;
  snap_bw_view_t commit_bw_view_c3;
  logic       commit_bw_done;
  logic       commit_bw_ok;

  // Two pointer-only BW checkers keep the input-stability contract local.
  // Eval and commit are independent clients, so neither path needs a wide
  // arbitration mux or duplicated segment storage.
  sched_bandwidth_check i_eval_bandwidth_check (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clear_i  (init_i),
    .start_i  (eval_bw_start),
    .done_o   (eval_bw_done),
    .snap_a_i (eval_bw_snap_a),
    .snap_b_i (eval_bw_snap_b),
    .ok_o     (eval_bw_ok)
  );

  sched_bandwidth_check i_commit_bandwidth_check (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clear_i  (init_i),
    .start_i  (commit_bw_start),
    .done_o   (commit_bw_done),
    .snap_a_i (commit_bw_view_c2),
    .snap_b_i (commit_bw_view_c3),
    .ok_o     (commit_bw_ok)
  );

  // ── Candidate generation ──────────────────────────────────────────────
  logic       gen_start;
  logic       gen_advance;
  logic       gen_done;
  cand_token_t generated_candidate;
  cand_token_t candidate_under_evaluation;
  cand_token_t selected_candidate;
  logic [3:0] head_valid;
  early_start_ctx_t early_ctx_comb;
  time_t early_idle_t;
  snap_timeline_t early_busy_timeline;

  assign early_idle_t = (c3_timeline_q.task_end < c2_timeline_q.task_end) ?
                        c3_timeline_q.task_end : c2_timeline_q.task_end;
  assign early_busy_timeline = (c3_timeline_q.task_end < c2_timeline_q.task_end) ?
                               c2_timeline_q : c3_timeline_q;
  assign early_ctx_comb = make_early_start_ctx(early_busy_timeline, early_idle_t);
  assign head_valid = {head_i[3].valid, head_i[2].valid,
                       head_i[1].valid, head_i[0].valid};

  logic [1:0] best_remove_count;
  logic [3:0] best_remove_slot_mask;
  assign candidate_under_evaluation = (st_q == ST_REPLAY_START) ?
                                      selected_candidate : generated_candidate;

  sched_candidate_generator i_candidate_generator (
    .clk_i                 (clk_i),
    .rst_ni                (rst_ni),
    .clear_i               (init_i),
    .start_i               (gen_start),
    .advance_i             (gen_advance),
    .done_o                (gen_done),
    .both_idle_i           (c2_timeline_q.task_end == c3_timeline_q.task_end),
    .early_count_i         (early_ctx_q.release_count),
    .head_valid_i          (head_valid),
    .head0_ntok_i          (head_i[0].ntok),
    .active_count_i        (active_count_i),
    .cand_o                (generated_candidate)
  );

  // ── Candidate evaluation lane ─────────────────────────────────────────
  logic       eval_valid;
  logic       eval_start;
  logic       eval_done;
  score_key_t evaluated_score;
  time_t evaluated_removed_parallel_work;
  time_t evaluated_removed_serial_work;
  winner_plan_t evaluated_plan;
  snap_timeline_t evaluated_c2_timeline;
  snap_timeline_t evaluated_c3_timeline;

  sched_candidate_evaluator i_candidate_evaluator (
    .clk_i                 (clk_i),
    .rst_ni                (rst_ni),
    .clear_i               (init_i),
    .start_i               (eval_start),
    .done_o                (eval_done),
    .cand_i                (candidate_under_evaluation),
    .head_i                (head_i),
    .active_count_i        (active_count_i),
    .total_parallel_work_i (total_parallel_work_i),
    .total_serial_work_i   (total_serial_work_i),
    .early_i               (early_ctx_q),
    .base_timeline_a_i     (c2_timeline_q),
    .base_timeline_b_i     (c3_timeline_q),
    .base_cache_a_i        (c2_cache_q),
    .base_cache_b_i        (c3_cache_q),
    .bw_start_o            (eval_bw_start),
    .bw_snap_a_o           (eval_bw_snap_a),
    .bw_snap_b_o           (eval_bw_snap_b),
    .bw_done_i             (((st_q == ST_EVAL_WAIT) || (st_q == ST_REPLAY_WAIT)) ? eval_bw_done : 1'b0),
    .bw_ok_i               (eval_bw_ok),
    .eval_valid_o          (eval_valid),
    .score_key_o           (evaluated_score),
    .removed_parallel_work_o (evaluated_removed_parallel_work),
    .removed_serial_work_o   (evaluated_removed_serial_work),
    .winner_plan_o         (evaluated_plan),
    .snap_timeline_a_o     (evaluated_c2_timeline),
    .snap_timeline_b_o     (evaluated_c3_timeline)
  );

  // ── Best reducer: compact winner only ─────────────────────────────────
  logic best_clear;

  sched_best_candidate i_best_candidate (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .clear_i              (best_clear || init_i),
    .candidate_valid_i    ((st_q == ST_EVAL_WAIT) && eval_valid),
    .candidate_token_i    (generated_candidate),
    .candidate_score_i    (evaluated_score),
    .best_token_o         (selected_candidate),
    .best_remove_count_o  (best_remove_count),
    .best_remove_slot_mask_o (best_remove_slot_mask)
  );

  // ── Replay commit and output-buffer producer ──────────────────────────
  logic s4pf_accepted_c2_q, s4pf_accepted_c2_d;
  logic s4pf_accepted_c3_q, s4pf_accepted_c3_d;
  snap_timeline_t c2_s4pf_trial_timeline;
  snap_timeline_t c3_s4pf_trial_timeline;
  snap_timeline_t c2_timeline_after_s4pf_check;
  logic c2_s4pf_window_candidate;
  logic c3_s4pf_window_candidate;

  function automatic task_desc_t make_task_desc(
    input winner_token_t token,
    input task_control_t ctrl
  );
    task_desc_t task_descriptor;
    begin
      task_descriptor = '0;
      task_descriptor.cluster   = ctrl.cluster;
      task_descriptor.eid       = token.eid;
      task_descriptor.ntok      = token.ntok;
      task_descriptor.tok_start = token.tok_start;
      task_descriptor.shape_s1  = ctrl.shape_s1;
      task_descriptor.shape_s3  = ctrl.shape_s3;
      task_descriptor.skip_s1   = ctrl.skip_s1;
      task_descriptor.skip_s3   = ctrl.skip_s3;
      task_descriptor.has_s2pf  = ctrl.has_s2pf;
      make_task_desc = task_descriptor;
    end
  endfunction

  task_desc_t [1:0] commit_task_desc;
  logic [1:0]        commit_task_s4pf_available;
  logic [1:0]        commit_task_count;
  slot_id_t [1:0]    commit_local_slot;
  slot_id_t          c2_slot_after_commit;
  slot_id_t          c3_slot_after_commit;
  logic              commit_has_c2_task;
  logic              commit_has_c3_task;
  snap_cache_t       c2_cache_after_commit;
  snap_cache_t       c3_cache_after_commit;

  // One packer feeds the single-write-port task FIFO.  Commit emission is
  // deliberately micro-sequenced instead of building a four-word write crossbar.
  sched_task_word_pack i_task_word_pack (
    .task_i       (pack_task),
    .local_slot_i (pack_local_slot),
    .s4pf_desc_i  (pack_s4pf_desc),
    .word_o       (pack_word)
  );

  assign c2_s4pf_window_candidate =
      evaluated_c2_timeline.valid &&
      (c2_cache_after_commit.pf_eid == PF_EID_NONE) &&
      ((evaluated_c2_timeline.dma3_end + S4PF_DMA_TICKS) <=
       evaluated_c2_timeline.task_end);

  assign c3_s4pf_window_candidate =
      evaluated_c3_timeline.valid &&
      (c3_cache_after_commit.pf_eid == PF_EID_NONE) &&
      ((evaluated_c3_timeline.dma3_end + S4PF_DMA_TICKS) <=
       evaluated_c3_timeline.task_end);

  always_comb begin
    c2_s4pf_trial_timeline = evaluated_c2_timeline;
    c3_s4pf_trial_timeline = evaluated_c3_timeline;
    if (c2_s4pf_window_candidate) begin
      c2_s4pf_trial_timeline.s4pf_valid = 1'b1;
      c2_s4pf_trial_timeline.s4pf_dma   = DMA_BOTH;
    end
    if (c3_s4pf_window_candidate) begin
      c3_s4pf_trial_timeline.s4pf_valid = 1'b1;
      c3_s4pf_trial_timeline.s4pf_dma   = DMA_BOTH;
    end
  end

  assign c2_timeline_after_s4pf_check =
      s4pf_accepted_c2_q ? c2_s4pf_trial_timeline : evaluated_c2_timeline;

  always_comb begin
    commit_task_desc       = '{default: '0};
    commit_task_s4pf_available = '0;
    // Normalized plans are 00/01/11.  Decode count directly instead of
    // instantiating a two-bit adder for this three-value domain.
    commit_task_count = evaluated_plan.task_valid[1] ? 2'd2 :
                        {1'b0, evaluated_plan.task_valid[0]};
    if (evaluated_plan.task_valid[0]) begin
      commit_task_desc[0] = make_task_desc(evaluated_plan.token[0],
                                           evaluated_plan.ctrl[0]);
      commit_task_s4pf_available[0] = evaluated_plan.ctrl[0].cluster ?
          s4pf_accepted_c3_q : s4pf_accepted_c2_q;
    end
    if (evaluated_plan.task_valid[1]) begin
      commit_task_desc[1] = make_task_desc(evaluated_plan.token[1],
                                           evaluated_plan.ctrl[1]);
      commit_task_s4pf_available[1] = evaluated_plan.ctrl[1].cluster ?
          s4pf_accepted_c3_q : s4pf_accepted_c2_q;
    end
  end

  always_comb begin
    // A legal two-task winner contains one C2 task and one C3 task.  Decode the
    // two fixed entries directly instead of supporting an unreachable generic
    // same-cluster sequence with a loop-carried slot counter.
    commit_local_slot[0] = commit_task_desc[0].cluster ? c3_slot_q : c2_slot_q;
    commit_local_slot[1] = commit_task_desc[1].cluster ? c3_slot_q : c2_slot_q;
    commit_has_c2_task =
        (evaluated_plan.task_valid[0] && !commit_task_desc[0].cluster) ||
        (evaluated_plan.task_valid[1] && !commit_task_desc[1].cluster);
    commit_has_c3_task =
        (evaluated_plan.task_valid[0] && commit_task_desc[0].cluster) ||
        (evaluated_plan.task_valid[1] && commit_task_desc[1].cluster);
    c2_slot_after_commit = c2_slot_q + slot_id_t'(commit_has_c2_task);
    c3_slot_after_commit = c3_slot_q + slot_id_t'(commit_has_c3_task);
    c2_cache_after_commit = commit_has_c2_task ? snap_cache_t'('0) : c2_cache_q;
    c3_cache_after_commit = commit_has_c3_task ? snap_cache_t'('0) : c3_cache_q;
  end

  // Dense task emission keeps at most one unresolved task per cluster.  A task
  // that owns an S4PF window is published only after the next same-cluster eid
  // is known; its high byte then carries that target directly.
  always_comb begin
    emit_current_valid = ({1'b0, emit_entry_index_q} < commit_task_count);
    emit_current_task = commit_task_desc[emit_entry_index_q];
    emit_current_slot = commit_local_slot[emit_entry_index_q];
    current_task_s4pf_available =
        commit_task_s4pf_available[emit_entry_index_q];
    emit_current_cluster = emit_current_task.cluster;
    emit_target_cache_hit = emit_current_cluster ?
        (initial_cache_eid_c3_i == encode_eid(emit_current_task.eid)) :
        (initial_cache_eid_c2_i == encode_eid(emit_current_task.eid));
    pending_s4pf_no_copy =
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
            1'b1, pending_s4pf_no_copy, emit_current_task.eid);
      end
      ST_COMMIT_EMIT_CUR: begin
        pack_task       = emit_current_task;
        pack_local_slot = emit_current_slot;
      end
      ST_FLUSH_PENDING: begin
        pack_task       = pending_task_q[emit_entry_index_q].desc;
        pack_local_slot = pending_task_q[emit_entry_index_q].local_slot;
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
    s4pf_accepted_c2_d   = s4pf_accepted_c2_q;
    s4pf_accepted_c3_d   = s4pf_accepted_c3_q;

    remove_valid_d        = remove_valid_q;
    task_fifo_count_d     = task_fifo_count_q;
    task_fifo_head_d      = task_fifo_head_q;
    task_fifo_push        = 1'b0;
    task_fifo_push_data   = pack_word;
    pending_valid_d       = pending_valid_q;
    pending_task_d        = pending_task_q;
    emit_entry_index_d    = emit_entry_index_q;

    gen_start   = 1'b0;
    gen_advance = 1'b0;
    eval_start  = 1'b0;
    best_clear  = 1'b0;
    commit_bw_start = 1'b0;
    commit_bw_view_c2 = to_bw_view(evaluated_c2_timeline);
    commit_bw_view_c3 = to_bw_view(evaluated_c3_timeline);

    remove_channel_available = !remove_valid_q || remove_ready_i;
    task_fifo_count_after_pop = task_fifo_count_q -
                                TASKQ_COUNT_W'(task_fifo_pop_i);
    task_fifo_space_after_pop =
        (task_fifo_count_after_pop < TASKQ_COUNT_W'(TASKQ_DEPTH));

    if (remove_ready_i) begin
      remove_valid_d = 1'b0;
    end

    // A stream read advances one head entry.  The producer writes at the
    // post-pop tail, so simultaneous pop/push preserves occupancy and order.
    if (task_fifo_pop_i) begin
      task_fifo_head_d = task_fifo_head_q + TASKQ_PTR_W'(1);
      task_fifo_count_d = task_fifo_count_after_pop;
    end

    if (init_i) begin
      c2_timeline_d        = '0;
      c3_timeline_d        = '0;
      c2_cache_d           = make_initial_cache(initial_cache_eid_c2_i);
      c3_cache_d           = make_initial_cache(initial_cache_eid_c3_i);
      c2_slot_d            = '0;
      c3_slot_d            = '0;
      early_ctx_d          = '0;
      s4pf_accepted_c2_d   = 1'b0;
      s4pf_accepted_c3_d   = 1'b0;
      remove_valid_d       = 1'b0;
      task_fifo_count_d    = '0;
      task_fifo_head_d     = '0;
      pending_valid_d      = '0;
      pending_task_d       = '{default: '0};
      emit_entry_index_d   = 1'b0;
      // Batch initialization and first-round start may arrive in the same
      // control write.  Init still clears all persistent round state first;
      // start_i then arms the freshly initialized core for the first round.
      st_d                 = start_i ? ST_ROUND_START : ST_IDLE;
    end else begin
      unique case (st_q)
        ST_IDLE: begin
          if (start_i && remove_channel_available && task_fifo_space_after_pop) begin
            st_d            = ST_ROUND_START;
          end
        end

        ST_ROUND_START: begin
          // ST_ROUND_START is the round-level predecode boundary after wrapper
          // compact/refill.  Only early_ctx is latched because it is derived
          // from C2/C3 timeline and consumed by both generator and eval lane.
          early_ctx_d = early_ctx_comb;
          if (active_count_i == NR_W'(0)) begin
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
          if (generated_candidate.valid) begin
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
          // Replay the compact best token directly through the evaluator.  The
          // generator is not involved in replay, so no wide candidate payload
          // or duplicate generator path is kept.
          eval_start       = 1'b1;
          st_d             = ST_REPLAY_WAIT;
        end

        ST_REPLAY_WAIT: begin
          if (eval_done) begin
            s4pf_accepted_c2_d = 1'b0;
            s4pf_accepted_c3_d = 1'b0;
            st_d             = ST_COMMIT_S4PF_C2_START;
          end
        end

        ST_COMMIT_S4PF_C2_START: begin
          if (c2_s4pf_window_candidate) begin
            commit_bw_start = 1'b1;
            commit_bw_view_c2 = to_bw_view(c2_s4pf_trial_timeline);
            commit_bw_view_c3 = to_bw_view(evaluated_c3_timeline);
            st_d = ST_COMMIT_S4PF_C2_WAIT;
          end else begin
            s4pf_accepted_c2_d = 1'b0;
            st_d = ST_COMMIT_S4PF_C3_START;
          end
        end

        ST_COMMIT_S4PF_C2_WAIT: begin
          commit_bw_view_c2 = to_bw_view(c2_s4pf_trial_timeline);
          commit_bw_view_c3 = to_bw_view(evaluated_c3_timeline);
          if (commit_bw_done) begin
            s4pf_accepted_c2_d = commit_bw_ok;
            st_d = ST_COMMIT_S4PF_C3_START;
          end
        end

        ST_COMMIT_S4PF_C3_START: begin
          if (c3_s4pf_window_candidate) begin
            commit_bw_start = 1'b1;
            // Preserve sequential C2-then-C3 S4PF semantics: C3 is checked
            // against C2 after the accepted C2 S4PF window.
            commit_bw_view_c2 = to_bw_view(c2_timeline_after_s4pf_check);
            commit_bw_view_c3 = to_bw_view(c3_s4pf_trial_timeline);
            st_d = ST_COMMIT_S4PF_C3_WAIT;
          end else begin
            s4pf_accepted_c3_d = 1'b0;
            st_d = ST_COMMIT_APPLY;
          end
        end

        ST_COMMIT_S4PF_C3_WAIT: begin
          commit_bw_view_c2 = to_bw_view(c2_timeline_after_s4pf_check);
          commit_bw_view_c3 = to_bw_view(c3_s4pf_trial_timeline);
          if (commit_bw_done) begin
            s4pf_accepted_c3_d = commit_bw_ok;
            st_d = ST_COMMIT_APPLY;
          end
        end

        ST_COMMIT_APPLY: begin
          // The evaluator reconstructs task identity from token + round context.
          // Keep the persistent timeline/cache unchanged until every task of
          // this round has crossed the single-write-port emit sequence.
          emit_entry_index_d = 1'b0;
          st_d = ST_COMMIT_EMIT_PREV;
        end

        ST_COMMIT_EMIT_PREV: begin
          if (!emit_current_valid) begin
            st_d = ST_COMMIT_FINISH;
          end else if (!pending_valid_q[emit_current_cluster]) begin
            st_d = ST_COMMIT_EMIT_CUR;
          end else if (task_fifo_space_after_pop) begin
            task_fifo_push = 1'b1;
            task_fifo_count_d = task_fifo_count_d + TASKQ_COUNT_W'(1);
            pending_valid_d[emit_current_cluster] = 1'b0;
            st_d = ST_COMMIT_EMIT_CUR;
          end
        end

        ST_COMMIT_EMIT_CUR: begin
          if (current_task_s4pf_available) begin
            pending_valid_d[emit_current_cluster] = 1'b1;
            pending_task_d[emit_current_cluster].desc = emit_current_task;
            pending_task_d[emit_current_cluster].local_slot = emit_current_slot;
            if ((2'(emit_entry_index_q) + 2'd1) < commit_task_count) begin
              emit_entry_index_d = emit_entry_index_q + 1'b1;
              st_d = ST_COMMIT_EMIT_PREV;
            end else begin
              st_d = ST_COMMIT_FINISH;
            end
          end else if (task_fifo_space_after_pop) begin
            task_fifo_push = 1'b1;
            task_fifo_count_d = task_fifo_count_d + TASKQ_COUNT_W'(1);
            if ((2'(emit_entry_index_q) + 2'd1) < commit_task_count) begin
              emit_entry_index_d = emit_entry_index_q + 1'b1;
              st_d = ST_COMMIT_EMIT_PREV;
            end else begin
              st_d = ST_COMMIT_FINISH;
            end
          end
        end

        ST_COMMIT_FINISH: begin
          c2_timeline_d = s4pf_accepted_c2_q ?
                          c2_s4pf_trial_timeline : evaluated_c2_timeline;
          c3_timeline_d = s4pf_accepted_c3_q ?
                          c3_s4pf_trial_timeline : evaluated_c3_timeline;
          c2_cache_d = c2_cache_after_commit;
          c3_cache_d = c3_cache_after_commit;
          if (s4pf_accepted_c2_q) begin
            c2_cache_d.pf_eid  = PF_EID_S4PF_WILDCARD;
            c2_cache_d.pf_end  = evaluated_c2_timeline.task_end;
            c2_cache_d.pf_full = 1'b0;
          end
          if (s4pf_accepted_c3_q) begin
            c3_cache_d.pf_eid  = PF_EID_S4PF_WILDCARD;
            c3_cache_d.pf_end  = evaluated_c3_timeline.task_end;
            c3_cache_d.pf_full = 1'b0;
          end
          c2_slot_d = c2_slot_after_commit;
          c3_slot_d = c3_slot_after_commit;
          if (round_finishes_batch) begin
            emit_entry_index_d = 1'b0;
            st_d = ST_FLUSH_PENDING;
          end else begin
            remove_valid_d = 1'b1;
            s4pf_accepted_c2_d = 1'b0;
            s4pf_accepted_c3_d = 1'b0;
            st_d = ST_IDLE;
          end
        end

        ST_FLUSH_PENDING: begin
          if (!pending_valid_q[emit_entry_index_q] || task_fifo_space_after_pop) begin
            if (pending_valid_q[emit_entry_index_q]) begin
              task_fifo_push = 1'b1;
              task_fifo_count_d = task_fifo_count_d + TASKQ_COUNT_W'(1);
              pending_valid_d[emit_entry_index_q] = 1'b0;
            end
            if (emit_entry_index_q == 1'b0) begin
              emit_entry_index_d = 1'b1;
            end else begin
              remove_valid_d = 1'b1;
              s4pf_accepted_c2_d = 1'b0;
              s4pf_accepted_c3_d = 1'b0;
              st_d = ST_IDLE;
            end
          end
        end

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
      s4pf_accepted_c2_q    <= 1'b0;
      s4pf_accepted_c3_q    <= 1'b0;
      remove_valid_q        <= 1'b0;
      task_fifo_count_q     <= '0;
      task_fifo_head_q      <= '0;
      pending_valid_q       <= '0;
      pending_task_q        <= '{default: '0};
      emit_entry_index_q    <= 1'b0;
    end else begin
      st_q                  <= st_d;
      c2_timeline_q         <= c2_timeline_d;
      c3_timeline_q         <= c3_timeline_d;
      c2_cache_q            <= c2_cache_d;
      c3_cache_q            <= c3_cache_d;
      c2_slot_q             <= c2_slot_d;
      c3_slot_q             <= c3_slot_d;
      early_ctx_q           <= early_ctx_d;
      s4pf_accepted_c2_q    <= s4pf_accepted_c2_d;
      s4pf_accepted_c3_q    <= s4pf_accepted_c3_d;
      remove_valid_q        <= remove_valid_d;
      task_fifo_count_q     <= task_fifo_count_d;
      task_fifo_head_q      <= task_fifo_head_d;
      pending_valid_q       <= pending_valid_d;
      pending_task_q        <= pending_task_d;
      emit_entry_index_q    <= emit_entry_index_d;
      if (task_fifo_push) begin
        task_fifo_data_q[task_fifo_write_address] <= task_fifo_push_data;
      end
    end
  end

  assign remove_valid_o     = remove_valid_q;
  // best_reduce is held until the next ST_EVAL_START.  The wrapper consumes
  // these derived fields on the immediate remove_valid/ready handshake, so no
  // second metadata register bank is required.
  assign remove_count_o     = remove_valid_q ? best_remove_count : 2'd0;
  assign remove_slot_mask_o = remove_valid_q ? best_remove_slot_mask : 4'b0000;
  // Replay keeps the winning evaluator context stable until the wrapper accepts
  // remove_valid.  Reuse its score projection instead of recomputing the same
  // per-expert work functions in the wrapper.
  assign remove_parallel_work_o = remove_valid_q ?
                                   evaluated_removed_parallel_work : '0;
  assign remove_serial_work_o = remove_valid_q ?
                                 evaluated_removed_serial_work : '0;

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni && remove_valid_q) begin
      assert (selected_candidate.valid && cand_remove_mask_legal(best_remove_slot_mask))
        else $error("moe_scheduler_core produced illegal remove_slot_mask=%b",
                    best_remove_slot_mask);
    end
  end
`endif

  assign task_fifo_valid_o = (task_fifo_count_q != TASKQ_COUNT_W'(0));
  // Tail is derived from head+occupancy; no duplicate tail register is kept.
  assign task_fifo_write_address = task_fifo_head_q +
                                   task_fifo_count_q[TASKQ_PTR_W-1:0];
  assign task_fifo_read_data_o = task_fifo_data_q[task_fifo_head_q];
  assign task_fifo_full_o =
      (task_fifo_count_q == TASKQ_COUNT_W'(TASKQ_DEPTH));
  assign task_fifo_count_o = task_fifo_count_q[3:0];

  assign busy_o = (st_q != ST_IDLE) ||
                  (remove_valid_q && !remove_ready_i) ||
                  ((task_fifo_count_q == TASKQ_COUNT_W'(TASKQ_DEPTH)) &&
                   !task_fifo_pop_i);
endmodule
