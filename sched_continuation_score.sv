// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - continuation scorer
//
// The scorer contains no child-candidate expansion.  It computes the exact
// small-tail greedy cases and, for rem>2, compares the aggregate greedy score
// with a four-step LPT projection over the candidate's child top4.
//
// One comparator/adder is reused for all four LPT placements.
// unassigned_serial_work_q starts with the complete child aggregate and is
// reduced as each visible work item is placed, so no parallel placement tree
// or child array FF is required.  The caller keeps remaining_work_i stable
// until done_o.

import sched_pkg::*;

module sched_continuation_score (
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic               clear_i,

  input  logic               start_i,
  output logic               done_o,

  input  time_t              c2_task_end_i,
  input  time_t              c3_task_end_i,
  input  logic [NR_W-1:0]    remaining_count_i,
  input  ntok_t              first_remaining_ntok_i,
  input  time_t              total_parallel_work_i,
  input  time_t              largest_parallel_work_i,
  input  time_t              total_serial_work_i,
  input  wire remaining_work_t [3:0] remaining_work_i,

  output time_t              cost_o
);

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_LPT0,
    ST_LPT1,
    ST_LPT2,
    ST_LPT3,
    ST_TAIL,
    ST_DONE
  } state_t;

  state_t st_q, st_d;
  time_t  assigned_load_c2_q, assigned_load_c2_d;
  time_t  assigned_load_c3_q, assigned_load_c3_d;
  time_t  unassigned_serial_work_q, unassigned_serial_work_d;
  time_t  best_cost_q, best_cost_d;

  function automatic time_t min_t(input time_t a, input time_t b);
    min_t = (a < b) ? a : b;
  endfunction

  function automatic time_t max_t(input time_t a, input time_t b);
    max_t = (a > b) ? a : b;
  endfunction

  function automatic time_t greedy_cost(
    input logic [NR_W-1:0] remaining_count,
    input time_t           earlier_end,
    input time_t           later_end,
    input ntok_t           first_remaining_ntok,
    input time_t           total_parallel_work,
    input time_t           largest_parallel_work,
    input time_t           total_serial_work
  );
    time_t parallel_cost;
    time_t serial_cost;
    time_t split_cost;
    time_t half_parallel_work_floor;
    begin
      half_parallel_work_floor = {1'b0, total_parallel_work[T_W-1:1]};

      unique case (remaining_count)
        NR_W'(0): greedy_cost = later_end;

        NR_W'(1): begin
          serial_cost = max_t(later_end, earlier_end + total_serial_work);
          split_cost = later_end +
                       time_t'(parallel_work_ticks(
                           ceil_div2_ntok(first_remaining_ntok)));
          greedy_cost = min_t(serial_cost, split_cost);
        end

        NR_W'(2): begin
          // remaining_work is sorted, so largest_parallel_work is the exact
          // maximum. total_serial_work already is the exact two-expert sum.
          parallel_cost = later_end + largest_parallel_work;
          serial_cost = max_t(later_end, earlier_end + total_serial_work);
          greedy_cost   = min_t(parallel_cost, serial_cost);
        end

        default: greedy_cost = later_end +
                               max_t(largest_parallel_work,
                                     half_parallel_work_floor);
      endcase
    end
  endfunction

  logic [1:0]     lpt_item_index;
  remaining_work_t lpt_item;
  time_t lpt_serial_work;
  time_t lower_assigned_load;
  time_t higher_assigned_load;
  time_t assigned_load_gap;
  time_t serial_work_beyond_gap;
  time_t half_excess_ceil;
  time_t lpt_projected_cost;
  time_t initial_earlier_end;
  time_t initial_later_end;
  time_t initial_greedy_cost;
  logic  c2_finishes_first;
  logic  c2_has_lower_assigned_load;

  always_comb begin
    unique case (st_q)
      ST_LPT1: lpt_item_index = 2'd1;
      ST_LPT2: lpt_item_index = 2'd2;
      ST_LPT3: lpt_item_index = 2'd3;
      default: lpt_item_index = 2'd0;
    endcase

    lpt_item = remaining_work_i[lpt_item_index];
    lpt_serial_work = lpt_item.valid ?
                      time_t'(serial_work_ticks(lpt_item.ntok)) : '0;

    // Share each ordering comparator across both min/max muxes.  The same
    // assigned-load ordering also controls the LPT placement update below.
    c2_finishes_first = (c2_task_end_i <= c3_task_end_i);
    initial_earlier_end = c2_finishes_first ? c2_task_end_i : c3_task_end_i;
    initial_later_end = c2_finishes_first ? c3_task_end_i : c2_task_end_i;
    initial_greedy_cost = greedy_cost(
        remaining_count_i, initial_earlier_end, initial_later_end,
        first_remaining_ntok_i, total_parallel_work_i,
        largest_parallel_work_i, total_serial_work_i);

    c2_has_lower_assigned_load = (assigned_load_c2_q <= assigned_load_c3_q);
    lower_assigned_load = c2_has_lower_assigned_load ?
                          assigned_load_c2_q : assigned_load_c3_q;
    higher_assigned_load = c2_has_lower_assigned_load ?
                           assigned_load_c3_q : assigned_load_c2_q;
    assigned_load_gap = higher_assigned_load - lower_assigned_load;
    serial_work_beyond_gap =
        (unassigned_serial_work_q > assigned_load_gap) ?
        (unassigned_serial_work_q - assigned_load_gap) : '0;
    half_excess_ceil = {1'b0, serial_work_beyond_gap[T_W-1:1]} +
                       T_W'(serial_work_beyond_gap[0]);
    lpt_projected_cost =
        (unassigned_serial_work_q <= assigned_load_gap) ?
        higher_assigned_load : (higher_assigned_load + half_excess_ceil);
  end

  always_comb begin
    st_d        = st_q;
    assigned_load_c2_d = assigned_load_c2_q;
    assigned_load_c3_d = assigned_load_c3_q;
    unassigned_serial_work_d = unassigned_serial_work_q;
    best_cost_d = best_cost_q;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          best_cost_d = initial_greedy_cost;
          if (remaining_count_i <= NR_W'(2)) begin
            st_d = ST_DONE;
          end else begin
            assigned_load_c2_d = c2_task_end_i;
            assigned_load_c3_d = c3_task_end_i;
            unassigned_serial_work_d = total_serial_work_i;
            st_d        = ST_LPT0;
          end
        end
      end

      ST_LPT0, ST_LPT1, ST_LPT2, ST_LPT3: begin
        if (lpt_item.valid) begin
          // Stable tie direction is part of the policy: equal loads choose C2.
          if (c2_has_lower_assigned_load) begin
            assigned_load_c2_d = assigned_load_c2_q + lpt_serial_work;
          end else begin
            assigned_load_c3_d = assigned_load_c3_q + lpt_serial_work;
          end
          unassigned_serial_work_d =
              unassigned_serial_work_q - lpt_serial_work;
        end

        unique case (st_q)
          ST_LPT0: st_d = ST_LPT1;
          ST_LPT1: st_d = ST_LPT2;
          ST_LPT2: st_d = ST_LPT3;
          default: st_d = ST_TAIL;
        endcase
      end

      ST_TAIL: begin
        best_cost_d = min_t(best_cost_q, lpt_projected_cost);
        st_d   = ST_DONE;
      end

      ST_DONE: begin
        if (!start_i) begin
          st_d = ST_IDLE;
        end
      end

      default: st_d = state_t'('x);
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q        <= ST_IDLE;
      assigned_load_c2_q <= '0;
      assigned_load_c3_q <= '0;
      unassigned_serial_work_q <= '0;
      best_cost_q <= '0;
    end else if (clear_i) begin
      st_q        <= ST_IDLE;
      assigned_load_c2_q <= '0;
      assigned_load_c3_q <= '0;
      unassigned_serial_work_q <= '0;
      best_cost_q <= '0;
    end else begin
      st_q        <= st_d;
      assigned_load_c2_q <= assigned_load_c2_d;
      assigned_load_c3_q <= assigned_load_c3_d;
      unassigned_serial_work_q <= unassigned_serial_work_d;
      best_cost_q <= best_cost_d;
    end
  end

  assign done_o = (st_q == ST_DONE);
  assign cost_o = best_cost_q;

endmodule
