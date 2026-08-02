// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51

import sched_pkg::*;
import sched_distilled_pkg::*;

module sched_distilled_regime_classify (
  input  wire distilled_mode_t                  mode_i,
  input  wire logic [NR_W-1:0]                  count_i,
  input  wire logic [DIST_TOKEN_SUM_W-1:0]      token_sum_i,
  input  wire logic [DIST_HIST_W-1:0]           odd_count_i,
  input  wire logic [DIST_HIST_W-1:0]           one_block_count_i,
  input  wire ntok_t                             t0_ntok_i,
  input  wire logic                              t1_valid_i,
  input  wire ntok_t                             t1_ntok_i,
  input  wire logic                              t4_valid_i,
  input  wire ntok_t                             t4_ntok_i,
  input  wire time_t                             c2_end_i,
  input  wire time_t                             c3_end_i,
  output distilled_regime_t                     regime_o
);

  time_t imbalance;
  logic [DIST_HIST_W+4:0] sparse_lhs;
  logic [NR_W+3:0] sparse_rhs;
  ntok_t twice_t1;

  always_comb begin
    imbalance = (c2_end_i >= c3_end_i) ?
                (c2_end_i - c3_end_i) : (c3_end_i - c2_end_i);
    sparse_lhs = one_block_count_i << 5;
    sparse_rhs = (count_i << 3) + (count_i << 1) + count_i;
    twice_t1 = t1_ntok_i << 1;

    regime_o = '0;
    regime_o.low_work_progress =
        (mode_i == DIST_MODE_ONE_IDLE) &&
        (token_sum_i <= DIST_TOKEN_SUM_W'(84)) &&
        (odd_count_i <= DIST_HIST_W'(9)) &&
        (!t4_valid_i || (t4_ntok_i <= ntok_t'(4)));
    regime_o.sparse_hot_sync =
        (mode_i == DIST_MODE_SYNC) &&
        (count_i >= NR_W'(2)) && t1_valid_i &&
        (t0_ntok_i >= twice_t1) && (sparse_lhs > sparse_rhs);
    regime_o.mid_plateau =
        (mode_i == DIST_MODE_ONE_IDLE) &&
        (count_i >= NR_W'(8)) && t1_valid_i &&
        (t1_ntok_i >= ntok_t'(5)) &&
        (t0_ntok_i <= ntok_t'(6)) && (imbalance == time_t'(3));
    regime_o.short_tail_plateau =
        (mode_i == DIST_MODE_ONE_IDLE) &&
        (count_i >= NR_W'(2)) &&
        (count_i <= NR_W'(7)) && t1_valid_i &&
        (t1_ntok_i >= ntok_t'(5)) &&
        (t0_ntok_i <= ntok_t'(6)) && (imbalance == time_t'(6));
    regime_o.large_slack_fill =
        (mode_i == DIST_MODE_ONE_IDLE) &&
        (count_i >= NR_W'(8)) &&
        (count_i <= NR_W'(16)) && t1_valid_i &&
        (t1_ntok_i >= ntok_t'(8)) && (imbalance >= time_t'(9));
  end

endmodule
