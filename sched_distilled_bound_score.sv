// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51

import sched_pkg::*;
import sched_distilled_pkg::*;

module sched_distilled_bound_score (
  input  logic                       clk_i,
  input  logic                       rst_ni,
  input  logic                       clear_i,
  input  logic                       start_i,
  output logic                       done_o,
  input  wire distilled_bound_cluster_t child_c2_i,
  input  wire distilled_bound_cluster_t child_c3_i,
  input  wire distilled_bound_counters_t child_counters_i,
  input  wire distilled_bound_head_t [4:0] child_head5_i,
  output distilled_bound_t           f_o,
  output distilled_bound_t           h_o,
  output distilled_bound_t           compute_bound_o,
  output distilled_bound_t           dma_bound_o
);

  localparam int unsigned DIV_W = T_W + 2;
  localparam int unsigned DMA_WORK_W = NR_W + 4;

  typedef enum logic [3:0] {
    ST_IDLE,
    ST_C_DIV,
    ST_C_FINISH,
    ST_H_HEAD,
    ST_H_HIST,
    ST_H_OVERFLOW,
    ST_D_INIT,
    ST_D_SCAN,
    ST_D_APPLY,
    ST_FINISH,
    ST_DONE
  } state_t;

  typedef struct packed {
    logic         valid;
    distilled_bound_t lo;
    distilled_bound_t hi;
    dma_binding_t mask;
  } interval_t;

  state_t st_q, st_d;
  distilled_bound_t f_q, f_d;
  distilled_bound_t h_q, h_d;
  time_t compute_q, compute_d;
  distilled_bound_t dma_q, dma_d;

  localparam int unsigned SCRATCH_W = 2*DIV_W + 3 + $clog2(DIV_W);
  logic [SCRATCH_W-1:0] scratch_q, scratch_d;
  logic [DIV_W-1:0] dividend_q, dividend_d;
  logic [DIV_W-1:0] quotient_q, quotient_d;
  logic [2:0] div_remainder_q, div_remainder_d;
  logic [$clog2(DIV_W)-1:0] div_bit_q, div_bit_d;
  logic [DIST_BLOCK_SUM_W-1:0] crossing_k_floor;
  logic [2:0] crossing_remainder_adjust;

  time_t h_load_c2_q, h_load_c2_d;
  time_t h_load_c3_q, h_load_c3_d;
  logic [3:0][DIST_HIST_W-1:0] h_hist_q, h_hist_d;
  time_t h_tail_work_q, h_tail_work_d;
  logic [2:0] h_head_index_q, h_head_index_d;
  logic [1:0] h_bucket_q, h_bucket_d;
  logic [T_W+1:0] h_total_work;
  time_t h_balanced_end;

  distilled_bound_t d_time_q, d_time_d;
  logic [DMA_WORK_W-1:0] d_work_q, d_work_d;
  logic [1:0] d_interval_q, d_interval_d;
  dma_binding_t d_used_q, d_used_d;
  logic d_next_valid_q, d_next_valid_d;
  logic scratch_c_we, scratch_h_we, scratch_d_we;

  assign dividend_q = scratch_q[0 +: DIV_W];
  assign quotient_q = scratch_q[DIV_W +: DIV_W];
  assign div_remainder_q = scratch_q[2*DIV_W +: 3];
  assign div_bit_q = scratch_q[2*DIV_W + 3 +: $clog2(DIV_W)];
  assign h_hist_q = scratch_q[0 +: 4*DIST_HIST_W];
  assign h_head_index_q = scratch_q[4*DIST_HIST_W +: 3];
  assign h_bucket_q = scratch_q[4*DIST_HIST_W + 3 +: 2];
  assign d_time_q = scratch_q[0 +: DIST_BOUND_W];
  assign d_work_q = scratch_q[DIST_BOUND_W +: DMA_WORK_W];
  assign d_interval_q = scratch_q[DIST_BOUND_W + DMA_WORK_W +: 2];
  assign d_used_q = dma_binding_t'(
      scratch_q[DIST_BOUND_W + DMA_WORK_W + 2 +: $bits(dma_binding_t)]);
  assign d_next_valid_q =
      scratch_q[DIST_BOUND_W + DMA_WORK_W + 2 + $bits(dma_binding_t)];

  time_t earlier_end;
  time_t later_end;
  time_t three_blocks;
  time_t c_balanced_end;
  time_t release_chain;
  time_t critical_chain;
  ntok_t hottest_ntok;
  ntok_t hottest_blocks;
  ntok_t hottest_quarter;

  logic [1:0] cache_slots;
  logic [NR_W-1:0] missing_cache_count;
  logic caches_name_same_eid;
  logic [DMA_WORK_W-1:0] mandatory_dma_work;
  time_t earliest_dma_release;

  distilled_bound_head_t h_head_item;
  ntok_t h_item_blocks;
  time_t h_item_work;
  logic h_c2_is_lower;

  interval_t intervals [0:3];
  interval_t d_interval;
  logic d_interval_event_valid;
  distilled_bound_t d_interval_event_value;
  logic d_interval_active;
  logic [1:0] free_lanes;
  distilled_bound_t interval_span;
  logic [DMA_WORK_W+DIST_BOUND_W-1:0] interval_capacity;

  function automatic time_t max_t(input time_t a, input time_t b);
    max_t = (a > b) ? a : b;
  endfunction

  function automatic time_t min_t(input time_t a, input time_t b);
    min_t = (a < b) ? a : b;
  endfunction

  function automatic distilled_bound_t to_bound(input time_t value);
    to_bound = distilled_bound_t'(value) << 1;
  endfunction

  function automatic distilled_bound_t max_b(
    input distilled_bound_t a,
    input distilled_bound_t b
  );
    max_b = (a > b) ? a : b;
  endfunction

  function automatic interval_t make_interval(
    input logic valid,
    input time_t lo,
    input time_t hi,
    input dma_binding_t mask
  );
    interval_t interval;
    begin
      interval.valid = valid && (mask != DMA_NONE) && (lo < hi);
      interval.lo = to_bound(lo);
      interval.hi = to_bound(hi);
      interval.mask = mask;
      make_interval = interval;
    end
  endfunction

  always_comb begin
    if (child_c2_i.task_end <= child_c3_i.task_end) begin
      earlier_end = child_c2_i.task_end;
      later_end = child_c3_i.task_end;
    end else begin
      earlier_end = child_c3_i.task_end;
      later_end = child_c2_i.task_end;
    end
    hottest_ntok = child_head5_i[0].valid ? child_head5_i[0].ntok : '0;
    hottest_blocks = ceil_div2_ntok(hottest_ntok);
    hottest_quarter = ceil_div4_ntok(hottest_ntok);
    three_blocks = time_t'({1'b0, child_counters_i.block_sum, 1'b0}) +
                   time_t'(child_counters_i.block_sum);
    release_chain = child_counters_i.count == NR_W'(0) ? later_end : min_t(
        earlier_end + time_t'({1'b0, hottest_blocks, 1'b0}) +
                       time_t'(hottest_blocks),
        later_end + time_t'({1'b0, ceil_div2_ntok(hottest_blocks), 1'b0}) +
                     time_t'(ceil_div2_ntok(hottest_blocks)));
    critical_chain = child_counters_i.count == NR_W'(0) ? earlier_end :
        earlier_end + time_t'({1'b0, hottest_quarter, 1'b0}) +
                      time_t'(hottest_quarter);
  end

  always_comb begin
    crossing_k_floor = quotient_q[DIST_BLOCK_SUM_W-1:0];
    crossing_remainder_adjust = (div_remainder_q > 3'd3) ?
                                3'd3 : div_remainder_q;
    // For D = 6*q+r, floor ends at A+r and ceil at A+3, where
    // A = c2_end+3*q.  The best endpoint is A+min(r,3).
    c_balanced_end = child_c2_i.task_end +
        time_t'({1'b0, crossing_k_floor, 1'b0}) +
        time_t'(crossing_k_floor) +
        time_t'(crossing_remainder_adjust);
  end

  always_comb begin
    h_total_work = {2'b0, h_load_c2_q} +
                   {2'b0, h_load_c3_q} +
                   {{2{1'b0}}, h_tail_work_q};
    h_balanced_end = time_t'(h_total_work[T_W+1:1]) +
                     time_t'(h_total_work[0]);
    h_balanced_end = max_t(max_t(h_load_c2_q, h_load_c3_q),
                           h_balanced_end);
  end

  always_comb begin
    caches_name_same_eid = child_c2_i.cache_valid && child_c3_i.cache_valid &&
                           (child_c2_i.cache_eid == child_c3_i.cache_eid);
    cache_slots = {1'b0, child_c2_i.cache_valid} +
                  {1'b0, child_c3_i.cache_valid} -
                  {1'b0, caches_name_same_eid};
    if (cache_slots > child_counters_i.count)
      cache_slots = child_counters_i.count[1:0];
    missing_cache_count = child_counters_i.count - NR_W'(cache_slots);
    // Each missing S1 costs four lane-ticks and each missing S3 costs two.
    // DMA sweep works in half-tick-lane units, hence the extra factor of two.
    mandatory_dma_work = (DMA_WORK_W'(missing_cache_count) << 3) +
                         (DMA_WORK_W'(missing_cache_count) << 2);
    earliest_dma_release = min_t(child_c2_i.task_end, child_c3_i.task_end);
    if (child_c2_i.cur_valid)
      earliest_dma_release = min_t(earliest_dma_release, child_c2_i.dma3_end);
    if (child_c3_i.cur_valid)
      earliest_dma_release = min_t(earliest_dma_release, child_c3_i.dma3_end);
  end

  always_comb begin
    intervals[0] = make_interval(child_c2_i.cur_valid,
                                 child_c2_i.task_start,
                                 child_c2_i.dma1_end,
                                 child_c2_i.dma_s1);
    intervals[1] = (child_c2_i.s2pf_dma != DMA_NONE) ?
        make_interval(child_c2_i.cur_valid,
                      child_c2_i.dma1_end,
                      child_c2_i.dma1_end +
                          s3_dma_ticks(child_c2_i.s2pf_dma),
                      child_c2_i.s2pf_dma) :
        make_interval(child_c2_i.cur_valid,
                      child_c2_i.s2_end,
                      child_c2_i.dma3_end,
                      child_c2_i.dma_s3);
    intervals[2] = make_interval(child_c3_i.cur_valid,
                                 child_c3_i.task_start,
                                 child_c3_i.dma1_end,
                                 child_c3_i.dma_s1);
    intervals[3] = (child_c3_i.s2pf_dma != DMA_NONE) ?
        make_interval(child_c3_i.cur_valid,
                      child_c3_i.dma1_end,
                      child_c3_i.dma1_end +
                          s3_dma_ticks(child_c3_i.s2pf_dma),
                      child_c3_i.s2pf_dma) :
        make_interval(child_c3_i.cur_valid,
                      child_c3_i.s2_end,
                      child_c3_i.dma3_end,
                      child_c3_i.dma_s3);
    d_interval = intervals[d_interval_q];
    d_interval_event_valid = d_interval.valid &&
                             (d_interval.hi > d_time_q);
    d_interval_event_value = (d_interval.lo > d_time_q) ?
                             d_interval.lo : d_interval.hi;
    d_interval_active = d_interval.valid &&
                        (d_interval.lo <= d_time_q) &&
                        (d_time_q < d_interval.hi);
    free_lanes = 2'd2 - {1'b0, |(d_used_q & DMA_IDMA)} -
                         {1'b0, |(d_used_q & DMA_XDMA)};
    interval_span = d_next_valid_q ? (dma_q - d_time_q) : '0;
    interval_capacity = free_lanes * interval_span;
  end

  assign h_head_item = child_head5_i[h_head_index_q];
  assign h_item_blocks = (st_q == ST_H_HEAD) ?
      ceil_div2_ntok(h_head_item.ntok) : ntok_t'(h_bucket_q + 2'd1);
  assign h_item_work = time_t'({1'b0, h_item_blocks, 1'b0}) +
                       time_t'(h_item_blocks);
  assign h_c2_is_lower = (h_load_c2_q <= h_load_c3_q);

  always_comb begin
    st_d = st_q;
    f_d = f_q;
    h_d = h_q;
    compute_d = compute_q;
    dma_d = dma_q;
    dividend_d = dividend_q;
    quotient_d = quotient_q;
    div_remainder_d = div_remainder_q;
    div_bit_d = div_bit_q;
    h_load_c2_d = h_load_c2_q;
    h_load_c3_d = h_load_c3_q;
    h_hist_d = h_hist_q;
    h_tail_work_d = h_tail_work_q;
    h_head_index_d = h_head_index_q;
    h_bucket_d = h_bucket_q;
    d_time_d = d_time_q;
    d_work_d = d_work_q;
    d_interval_d = d_interval_q;
    d_used_d = d_used_q;
    d_next_valid_d = d_next_valid_q;
    scratch_c_we = 1'b0;
    scratch_h_we = 1'b0;
    scratch_d_we = 1'b0;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          h_load_c2_d = child_c2_i.task_end;
          h_load_c3_d = child_c3_i.task_end;
          h_tail_work_d = three_blocks;
          if (child_counters_i.count == NR_W'(0)) begin
            h_hist_d = child_counters_i.small_hist;
            h_head_index_d = '0;
            h_bucket_d = 2'd3;
            scratch_h_we = 1'b1;
            compute_d = later_end;
            st_d = ST_H_OVERFLOW;
          end else if ((child_c3_i.task_end + three_blocks) <= child_c2_i.task_end) begin
            h_hist_d = child_counters_i.small_hist;
            h_head_index_d = '0;
            h_bucket_d = 2'd3;
            scratch_h_we = 1'b1;
            compute_d = child_c2_i.task_end;
            st_d = ST_H_HEAD;
          end else if ((child_c2_i.task_end + three_blocks) <= child_c3_i.task_end) begin
            h_hist_d = child_counters_i.small_hist;
            h_head_index_d = '0;
            h_bucket_d = 2'd3;
            scratch_h_we = 1'b1;
            compute_d = child_c3_i.task_end;
            st_d = ST_H_HEAD;
          end else begin
            dividend_d = DIV_W'(child_c3_i.task_end + three_blocks -
                                child_c2_i.task_end);
            quotient_d = '0;
            div_remainder_d = '0;
            div_bit_d = $clog2(DIV_W)'(DIV_W-1);
            scratch_c_we = 1'b1;
            st_d = ST_C_DIV;
          end
        end
      end

      ST_C_DIV: begin
        logic [3:0] shifted_remainder;
        shifted_remainder = {div_remainder_q, dividend_q[div_bit_q]};
        quotient_d[div_bit_q] = (shifted_remainder >= 4'd6);
        div_remainder_d = (shifted_remainder >= 4'd6) ?
                          shifted_remainder - 4'd6 : shifted_remainder[2:0];
        if (div_bit_q == '0)
          st_d = ST_C_FINISH;
        else
          div_bit_d = div_bit_q - 1'b1;
        scratch_c_we = 1'b1;
      end

      ST_C_FINISH: begin
        compute_d = c_balanced_end;
        h_hist_d = child_counters_i.small_hist;
        h_head_index_d = '0;
        h_bucket_d = 2'd3;
        scratch_h_we = 1'b1;
        st_d = ST_H_HEAD;
      end

      ST_H_HEAD: begin
        scratch_h_we = 1'b1;
        if (h_head_item.valid) begin
          if (h_c2_is_lower)
            h_load_c2_d = h_load_c2_q + h_item_work;
          else
            h_load_c3_d = h_load_c3_q + h_item_work;
          h_tail_work_d = h_tail_work_q - h_item_work;
          if (h_item_blocks <= ntok_t'(4))
            h_hist_d[h_item_blocks-1'b1] =
                h_hist_q[h_item_blocks-1'b1] - 1'b1;
        end
        if (h_head_index_q == 3'd4)
          st_d = ST_H_HIST;
        else if (!child_head5_i[h_head_index_q + 1'b1].valid)
          st_d = ST_H_HIST;
        else
          h_head_index_d = h_head_index_q + 1'b1;
      end

      ST_H_HIST: begin
        scratch_h_we = 1'b1;
        if (h_hist_q[h_bucket_q] != DIST_HIST_W'(0)) begin
          if (h_c2_is_lower)
            h_load_c2_d = h_load_c2_q + h_item_work;
          else
            h_load_c3_d = h_load_c3_q + h_item_work;
          h_tail_work_d = h_tail_work_q - h_item_work;
          h_hist_d[h_bucket_q] = h_hist_q[h_bucket_q] - 1'b1;
        end else if (h_bucket_q == 2'd0) begin
          st_d = ST_H_OVERFLOW;
        end else begin
          h_bucket_d = h_bucket_q - 1'b1;
        end
      end

      ST_H_OVERFLOW: begin
        h_load_c2_d = h_balanced_end;
        h_load_c3_d = h_balanced_end;
        st_d = ST_D_INIT;
      end

      ST_D_INIT: begin
        scratch_d_we = 1'b1;
        d_time_d = to_bound(earliest_dma_release);
        d_work_d = mandatory_dma_work;
        d_interval_d = '0;
        d_used_d = DMA_NONE;
        d_next_valid_d = 1'b0;
        if (mandatory_dma_work == DMA_WORK_W'(0)) begin
          dma_d = to_bound(later_end);
          st_d = ST_FINISH;
        end else begin
          dma_d = '0;
          st_d = ST_D_SCAN;
        end
      end

      ST_D_SCAN: begin
        scratch_d_we = 1'b1;
        if (d_interval_active)
          d_used_d = d_used_q | d_interval.mask;
        if (d_interval_event_valid &&
            (!d_next_valid_q || (d_interval_event_value < dma_q))) begin
          dma_d = d_interval_event_value;
          d_next_valid_d = 1'b1;
        end
        if (d_interval_q == 2'd3)
          st_d = ST_D_APPLY;
        else
          d_interval_d = d_interval_q + 1'b1;
      end

      ST_D_APPLY: begin
        scratch_d_we = 1'b1;
        if (!d_next_valid_q) begin
          dma_d = max_b(to_bound(later_end), d_time_q +
              distilled_bound_t'(d_work_q[DMA_WORK_W-1:1]) + d_work_q[0]);
          st_d = ST_FINISH;
        end else if ((free_lanes != 2'd0) &&
                     (DMA_WORK_W'(d_work_q) <= interval_capacity)) begin
          dma_d = max_b(to_bound(later_end), d_time_q +
              ((free_lanes == 2'd2) ?
               (distilled_bound_t'(d_work_q[DMA_WORK_W-1:1]) + d_work_q[0]) :
               distilled_bound_t'(d_work_q)));
          st_d = ST_FINISH;
        end else begin
          if (free_lanes != 2'd0)
            d_work_d = d_work_q - DMA_WORK_W'(interval_capacity);
          d_time_d = dma_q;
          d_interval_d = '0;
          d_used_d = DMA_NONE;
          d_next_valid_d = 1'b0;
          dma_d = '0;
          st_d = ST_D_SCAN;
        end
      end

      ST_FINISH: begin
        distilled_bound_t combined;
        combined = to_bound(later_end);
        combined = max_b(combined, to_bound(compute_q));
        combined = max_b(combined, to_bound(release_chain));
        combined = max_b(combined, to_bound(critical_chain));
        combined = max_b(combined, dma_q);
        f_d = max_b(child_counters_i.parent_bound, combined);
        h_d = max_b(to_bound(max_t(h_load_c2_q, h_load_c3_q)),
                    max_b(child_counters_i.parent_bound, combined));
        st_d = ST_DONE;
      end

      ST_DONE: begin
        if (!start_i)
          st_d = ST_IDLE;
      end

      default: st_d = ST_IDLE;
    endcase
  end

  always_comb begin
    scratch_d = scratch_q;
    if (scratch_c_we) begin
      scratch_d[0 +: DIV_W] = dividend_d;
      scratch_d[DIV_W +: DIV_W] = quotient_d;
      scratch_d[2*DIV_W +: 3] = div_remainder_d;
      scratch_d[2*DIV_W + 3 +: $clog2(DIV_W)] = div_bit_d;
    end else if (scratch_h_we) begin
      scratch_d[0 +: 4*DIST_HIST_W] = h_hist_d;
      scratch_d[4*DIST_HIST_W +: 3] = h_head_index_d;
      scratch_d[4*DIST_HIST_W + 3 +: 2] = h_bucket_d;
    end else if (scratch_d_we) begin
      scratch_d[0 +: DIST_BOUND_W] = d_time_d;
      scratch_d[DIST_BOUND_W +: DMA_WORK_W] = d_work_d;
      scratch_d[DIST_BOUND_W + DMA_WORK_W +: 2] = d_interval_d;
      scratch_d[DIST_BOUND_W + DMA_WORK_W + 2 +: $bits(dma_binding_t)] =
          d_used_d;
      scratch_d[DIST_BOUND_W + DMA_WORK_W + 2 + $bits(dma_binding_t)] =
          d_next_valid_d;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q <= ST_IDLE;
      f_q <= '0;
      h_q <= '0;
      compute_q <= '0;
      dma_q <= '0;
      scratch_q <= '0;
      h_load_c2_q <= '0;
      h_load_c3_q <= '0;
      h_tail_work_q <= '0;
    end else if (clear_i) begin
      st_q <= ST_IDLE;
      f_q <= '0;
      h_q <= '0;
      compute_q <= '0;
      dma_q <= '0;
      scratch_q <= '0;
      h_load_c2_q <= '0;
      h_load_c3_q <= '0;
      h_tail_work_q <= '0;
    end else begin
      st_q <= st_d;
      f_q <= f_d;
      h_q <= h_d;
      compute_q <= compute_d;
      dma_q <= dma_d;
      scratch_q <= scratch_d;
      h_load_c2_q <= h_load_c2_d;
      h_load_c3_q <= h_load_c3_d;
      h_tail_work_q <= h_tail_work_d;
    end
  end

  assign done_o = (st_q == ST_DONE);
  assign f_o = f_q;
  assign h_o = h_q;
  assign compute_bound_o = to_bound(compute_q);
  assign dma_bound_o = dma_q;

endmodule
