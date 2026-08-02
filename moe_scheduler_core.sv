// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51

import sched_pkg::*;
import sched_distilled_pkg::*;

module moe_scheduler_core (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  logic                         init_i,
  input  logic                         start_i,
  input  pf_eid_t                      initial_cache_eid_c2_i,
  input  pf_eid_t                      initial_cache_eid_c3_i,
  input  wire head_ctx_t [7:0]         hot_i,
  input  wire head_ctx_t               bottom_i,
  input  wire distilled_counters_t      initial_counters_i,
  input  logic                         remove_ready_i,
  output logic                         remove_valid_o,
  output logic [1:0]                   remove_count_o,
  output logic [EID_RAW_W-1:0]         remove_eid_a_o,
  output logic [EID_RAW_W-1:0]         remove_eid_b_o,
  input  logic                         task_fifo_pop_i,
  output logic                         task_fifo_valid_o,
  output logic [63:0]                  task_fifo_read_data_o,
  output logic                         task_fifo_full_o,
  output logic [3:0]                   task_fifo_count_o,
  output logic                         busy_o
);

  localparam int unsigned TASKQ_COUNT_W = $clog2(TASKQ_DEPTH + 1);
  localparam int unsigned TASKQ_PTR_W = $clog2(TASKQ_DEPTH);

  typedef enum logic [3:0] {
    ST_IDLE,
    ST_ROUND_START,
    ST_ROUND_WAIT,
    ST_WAIT_REMOVE,
    ST_EMIT_PREVIOUS,
    ST_STORE_CURRENT,
    ST_COMMIT_STATE,
    ST_FLUSH_C2,
    ST_FLUSH_C3
  } state_t;

  typedef struct packed {
    task_desc_t desc;
    slot_id_t   local_slot;
  } pending_task_t;

  typedef struct packed {
    task_desc_t desc;
    slot_id_t   local_slot;
    logic [7:0] s4pf_desc;
  } task_queue_entry_t;

  state_t st_q, st_d;
  distilled_cluster_state_t c2_q, c2_d;
  distilled_cluster_state_t c3_q, c3_d;
  distilled_counters_t counters_q, counters_d;
  slot_id_t c2_slot_q, c2_slot_d;
  slot_id_t c3_slot_q, c3_slot_d;
  logic [1:0] pending_valid_q, pending_valid_d;
  pending_task_t pending_q [0:1];
  pending_task_t pending_d [0:1];
  logic task_index_q, task_index_d;

  logic round_start;
  logic round_done;
  logic round_feasible;
  distilled_cluster_state_t round_child_c2;
  distilled_cluster_state_t round_child_c3;
  distilled_counters_t round_child_counters;
  winner_plan_t round_plan;
  logic [1:0] round_remove_count;
  logic [EID_RAW_W-1:0] round_remove_eid_a;
  logic [EID_RAW_W-1:0] round_remove_eid_b;
  distilled_action_token_t round_token;
  distilled_score_record_t round_score;

  task_desc_t commit_task_q [0:1];
  logic [1:0] commit_task_count_q;
  dma_binding_t commit_target_c2_q;
  dma_binding_t commit_target_c3_q;

  logic [TASKQ_COUNT_W-1:0] task_count_q, task_count_d;
  (* ram_style = "registers", shreg_extract = "no" *)
  task_queue_entry_t task_mem_q [0:TASKQ_DEPTH-1];
  logic [TASKQ_PTR_W-1:0] task_head_q;
  logic [TASKQ_PTR_W-1:0] task_tail_q;
  logic task_pop;
  logic task_push;
  task_queue_entry_t task_push_entry;
  logic [TASKQ_COUNT_W-1:0] count_after_pop;
  logic fifo_space_after_pop;

  logic [1:0] round_task_count;
  logic current_valid;
  task_desc_t current_task;
  logic current_cluster;
  slot_id_t current_slot;
  dma_binding_t current_target_binding;
  task_desc_t pack_task;
  slot_id_t pack_slot;
  logic [7:0] pack_s4pf_desc;
  logic [63:0] packed_word;

  function automatic distilled_cluster_state_t initial_cluster_state(
    input pf_eid_t cache_eid
  );
    distilled_cluster_state_t state;
    begin
      state = '0;
      if (cache_eid != PF_EID_NONE) begin
        state.cache_valid = 1'b1;
        state.cache_eid = cache_eid[EID_RAW_W-1:0];
      end
      initial_cluster_state = state;
    end
  endfunction

  function automatic task_desc_t make_task_desc(
    input winner_token_t token,
    input task_control_t control
  );
    task_desc_t descriptor;
    begin
      descriptor = '0;
      descriptor.cluster = control.cluster;
      descriptor.eid = token.eid;
      descriptor.ntok = token.ntok;
      descriptor.tok_start = token.tok_start;
      descriptor.shape_s1 = control.shape_s1;
      descriptor.shape_s3 = control.shape_s3;
      descriptor.skip_s1 = control.skip_s1;
      descriptor.skip_s3 = control.skip_s3;
      descriptor.has_s2pf = control.has_s2pf;
      descriptor.dma_s1_both = control.dma_s1_both;
      descriptor.dma_late_both = control.dma_late_both;
      make_task_desc = descriptor;
    end
  endfunction

  function automatic logic [7:0] make_s4pf_desc(
    input dma_binding_t binding,
    input logic [EID_RAW_W-1:0] target_eid
  );
    logic [7:0] descriptor;
    begin
      descriptor = '0;
      if (binding != DMA_NONE) begin
        descriptor[S4PF_DESC_OP_LSB +: S4PF_DESC_OP_W] =
            (binding == DMA_BOTH) ? S4PF_DESC_OP_BOTH : S4PF_DESC_OP_SINGLE;
        descriptor[S4PF_DESC_TARGET_EID_LSB +: EID_RAW_W] = target_eid;
      end
      make_s4pf_desc = descriptor;
    end
  endfunction

  sched_distilled_round_engine i_round_engine (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .clear_i           (init_i),
    .start_i           (round_start),
    .done_o            (round_done),
    .feasible_o        (round_feasible),
    .hot_i             (hot_i),
    .bottom_i          (bottom_i),
    .base_c2_i         (c2_q),
    .base_c3_i         (c3_q),
    .counters_i        (counters_q),
    .child_c2_o        (round_child_c2),
    .child_c3_o        (round_child_c3),
    .child_counters_o  (round_child_counters),
    .plan_o            (round_plan),
    .remove_count_o    (round_remove_count),
    .remove_eid_a_o    (round_remove_eid_a),
    .remove_eid_b_o    (round_remove_eid_b),
    .selected_token_o  (round_token),
    .selected_score_o  (round_score)
  );

  assign round_task_count = commit_task_count_q;
  assign current_valid = ({1'b0, task_index_q} < commit_task_count_q);
  assign current_task = commit_task_q[task_index_q];
  assign current_cluster = current_task.cluster;
  assign current_slot = current_cluster ? c3_slot_q : c2_slot_q;
  assign current_target_binding = current_cluster ?
      commit_target_c3_q : commit_target_c2_q;

  always_comb begin
    pack_task = '0;
    pack_slot = '0;
    pack_s4pf_desc = '0;
    unique case (st_q)
      ST_EMIT_PREVIOUS: begin
        pack_task = pending_q[current_cluster].desc;
        pack_slot = pending_q[current_cluster].local_slot;
        pack_s4pf_desc = make_s4pf_desc(current_target_binding,
                                        current_task.eid);
      end
      ST_FLUSH_C2: begin
        pack_task = pending_q[0].desc;
        pack_slot = pending_q[0].local_slot;
      end
      ST_FLUSH_C3: begin
        pack_task = pending_q[1].desc;
        pack_slot = pending_q[1].local_slot;
      end
      default: begin
      end
    endcase
  end

  sched_task_word_pack i_task_word_pack (
    .task_i       (task_mem_q[task_head_q].desc),
    .local_slot_i (task_mem_q[task_head_q].local_slot),
    .s4pf_desc_i  (task_mem_q[task_head_q].s4pf_desc),
    .word_o       (packed_word)
  );

  assign task_push_entry.desc = pack_task;
  assign task_push_entry.local_slot = pack_slot;
  assign task_push_entry.s4pf_desc = pack_s4pf_desc;

  assign task_pop = task_fifo_pop_i && (task_count_q != '0);
  assign count_after_pop = task_count_q - TASKQ_COUNT_W'(task_pop);
  assign fifo_space_after_pop = count_after_pop < TASKQ_COUNT_W'(TASKQ_DEPTH);

  always_comb begin
    st_d = st_q;
    c2_d = c2_q;
    c3_d = c3_q;
    counters_d = counters_q;
    c2_slot_d = c2_slot_q;
    c3_slot_d = c3_slot_q;
    pending_valid_d = pending_valid_q;
    for (int cluster = 0; cluster < 2; cluster++)
      pending_d[cluster] = pending_q[cluster];
    task_index_d = task_index_q;
    task_count_d = count_after_pop;
    task_push = 1'b0;
    round_start = 1'b0;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i && (counters_q.count != NR_W'(0)))
          st_d = ST_ROUND_START;
      end

      ST_ROUND_START: begin
        round_start = 1'b1;
        st_d = ST_ROUND_WAIT;
      end

      ST_ROUND_WAIT: begin
        if (round_done)
          st_d = ST_WAIT_REMOVE;
      end

      ST_WAIT_REMOVE: begin
        if (remove_ready_i) begin
          c2_d = round_child_c2;
          c3_d = round_child_c3;
          counters_d = round_child_counters;
          task_index_d = 1'b0;
          st_d = ST_EMIT_PREVIOUS;
        end
      end

      ST_EMIT_PREVIOUS: begin
        if (!current_valid) begin
          st_d = ST_COMMIT_STATE;
        end else if (!pending_valid_q[current_cluster]) begin
          st_d = ST_STORE_CURRENT;
        end else if (fifo_space_after_pop) begin
          task_push = 1'b1;
          task_count_d = count_after_pop + TASKQ_COUNT_W'(1);
          pending_valid_d[current_cluster] = 1'b0;
          st_d = ST_STORE_CURRENT;
        end
      end

      ST_STORE_CURRENT: begin
        pending_valid_d[current_cluster] = 1'b1;
        pending_d[current_cluster].desc = current_task;
        pending_d[current_cluster].local_slot = current_slot;
        if (current_cluster)
          c3_slot_d = c3_slot_q + slot_id_t'(1);
        else
          c2_slot_d = c2_slot_q + slot_id_t'(1);
        if (({1'b0, task_index_q} + 2'd1) < round_task_count) begin
          task_index_d = task_index_q + 1'b1;
          st_d = ST_EMIT_PREVIOUS;
        end else begin
          st_d = ST_COMMIT_STATE;
        end
      end

      ST_COMMIT_STATE: begin
        if (counters_q.count == NR_W'(0))
          st_d = ST_FLUSH_C2;
        else
          st_d = ST_IDLE;
      end

      ST_FLUSH_C2: begin
        if (!pending_valid_q[0]) begin
          st_d = ST_FLUSH_C3;
        end else if (fifo_space_after_pop) begin
          task_push = 1'b1;
          task_count_d = count_after_pop + TASKQ_COUNT_W'(1);
          pending_valid_d[0] = 1'b0;
          st_d = ST_FLUSH_C3;
        end
      end

      ST_FLUSH_C3: begin
        if (!pending_valid_q[1]) begin
          st_d = ST_IDLE;
        end else if (fifo_space_after_pop) begin
          task_push = 1'b1;
          task_count_d = count_after_pop + TASKQ_COUNT_W'(1);
          pending_valid_d[1] = 1'b0;
          st_d = ST_IDLE;
        end
      end

      default: st_d = ST_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q <= ST_IDLE;
      c2_q <= '0;
      c3_q <= '0;
      counters_q <= '0;
      c2_slot_q <= '0;
      c3_slot_q <= '0;
      pending_valid_q <= '0;
      for (int cluster = 0; cluster < 2; cluster++)
        pending_q[cluster] <= '0;
      task_index_q <= '0;
      task_count_q <= '0;
    end else if (init_i) begin
      st_q <= ST_IDLE;
      c2_q <= initial_cluster_state(initial_cache_eid_c2_i);
      c3_q <= initial_cluster_state(initial_cache_eid_c3_i);
      counters_q <= initial_counters_i;
      counters_q.parent_bound <= '0;
      c2_slot_q <= '0;
      c3_slot_q <= '0;
      pending_valid_q <= '0;
      for (int cluster = 0; cluster < 2; cluster++)
        pending_q[cluster] <= '0;
      task_index_q <= '0;
      task_count_q <= '0;
    end else begin
      st_q <= st_d;
      c2_q <= c2_d;
      c3_q <= c3_d;
      counters_q <= counters_d;
      c2_slot_q <= c2_slot_d;
      c3_slot_q <= c3_slot_d;
      pending_valid_q <= pending_valid_d;
      for (int cluster = 0; cluster < 2; cluster++)
        pending_q[cluster] <= pending_d[cluster];
      task_index_q <= task_index_d;
      task_count_q <= task_count_d;
    end
  end

  always_ff @(posedge clk_i) begin
    if ((st_q == ST_ROUND_WAIT) && round_done) begin
      commit_task_q[0] <= make_task_desc(round_plan.token[0],
                                        round_plan.ctrl[0]);
      commit_task_q[1] <= make_task_desc(round_plan.token[1],
                                        round_plan.ctrl[1]);
      commit_task_count_q <= round_plan.task_valid[1] ? 2'd2 :
                             {1'b0, round_plan.task_valid[0]};
      commit_target_c2_q <= round_token.targeted_s4pf_c2;
      commit_target_c3_q <= round_token.targeted_s4pf_c3;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      task_head_q <= '0;
      task_tail_q <= '0;
    end else if (init_i) begin
      task_head_q <= '0;
      task_tail_q <= '0;
    end else begin
      if (task_pop)
        task_head_q <= task_head_q + TASKQ_PTR_W'(1);
      if (task_push) begin
        task_mem_q[task_tail_q] <= task_push_entry;
        task_tail_q <= task_tail_q + TASKQ_PTR_W'(1);
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni && !init_i) begin
      assert (task_count_q <= TASKQ_COUNT_W'(TASKQ_DEPTH));
      if (st_q == ST_ROUND_WAIT && round_done)
        assert (round_feasible);
      if (task_fifo_pop_i)
        assert (task_count_q != '0);
      if (task_push)
        assert (fifo_space_after_pop);
      if (task_push && task_pop)
        assert (task_count_d == task_count_q);
      assert ((1 << TASKQ_PTR_W) == TASKQ_DEPTH);
    end
  end
`endif

  assign remove_valid_o = (st_q == ST_WAIT_REMOVE);
  assign remove_count_o = round_remove_count;
  assign remove_eid_a_o = round_remove_eid_a;
  assign remove_eid_b_o = round_remove_eid_b;
  assign task_fifo_valid_o = (task_count_q != '0);
  assign task_fifo_read_data_o = packed_word;
  assign task_fifo_full_o = (task_count_q == TASKQ_COUNT_W'(TASKQ_DEPTH));
  assign task_fifo_count_o = 4'(task_count_q);
  assign busy_o = (st_q != ST_IDLE);

endmodule
