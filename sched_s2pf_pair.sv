// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - lite S2PF policy primitive
//
// This block intentionally does not implement the old 25-way try_s2pf_pair()
// search.  The scheduler now uses a fixed, hardware-oriented policy selected
// by candidate_generator:
//   S2PF_OFF           : raw no-prefetch only
//   S2PF_PAIR_LITE     : raw + both@task_start + both@dma1_end + both@latest
//   S2PF_SPLIT_LITE    : raw + both@task_start + both@dma1_end + B-only@latest
//   S2PF_SINGLE_LATEST : raw + active-side@latest
//
// The search is sequential: one template is turned into one BW request, then
// compared against the best accepted template.  This avoids A/B independent
// cross-product starts and removes the s1_end placement entirely.

import sched_pkg::*;

module sched_s2pf_pair (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         start_i,
  output logic         busy_o,
  output logic         done_o,

  input  s2pf_policy_t policy_i,
  input  logic         side_a_active_i,
  input  logic         side_b_active_i,
  input  logic [1:0]   shape_s3_a_i,
  input  logic [1:0]   shape_s3_b_i,
  input  snap_timeline_t snap_a_i,
  input  snap_timeline_t snap_b_i,

  output logic         bw_start_o,
  output snap_bw_view_t  bw_snap_a_o,
  output snap_bw_view_t  bw_snap_b_o,
  input  logic         bw_done_i,
  input  logic         bw_ok_i,

  output s2pf_patch_t patch_o
);

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_RAW_START,
    ST_RAW_WAIT,
    ST_TRIAL_START,
    ST_TRIAL_WAIT,
    ST_DONE
  } state_t;

  state_t st_q, st_d;

  localparam int unsigned SIDE_A = 0;
  localparam int unsigned SIDE_B = 1;

  // 本模块不锁存完整 snap request。调用者必须在 busy_o 期间保持 snap_a/b
  // 输入稳定；本模块只锁存 trial 阶段真正需要的小标量，避免 shape/policy/
  // endpoint 解码逻辑在每个 S2PF trial 中反复展开。
  logic [1:0]   scan_idx_q, scan_idx_d;
  s2pf_policy_t policy_q, policy_d;
  logic [1:0]   side_q, side_d;

  logic         best_valid_q, best_valid_d;
  logic [1:0]   best_class_q, best_class_d;
  logic         best_a_pf_q, best_a_pf_d;
  logic         best_b_pf_q, best_b_pf_d;
  logic [T_W-1:0] best_a_start_q, best_a_start_d;
  logic [T_W-1:0] best_b_start_q, best_b_start_d;

  time_t [1:0]   hi_q, hi_d;
  time_t [1:0]   best_s4_q, best_s4_d;
  logic [1:0]    dur_is2_q, dur_is2_d;
  logic [1:0]    pf_bw_is128_q, pf_bw_is128_d;
  logic [1:0]    can_q, can_d;
  logic [1:0]    dma_start_valid_q, dma_start_valid_d;
  logic [1:0]     trial_count_q, trial_count_d;

  snap_timeline_t try_a;
  snap_timeline_t try_b;
  logic       try_valid;
  logic [1:0] try_class;
  logic       try_a_pf;
  logic       try_b_pf;
  logic [1:0] try_a_sel;
  logic [1:0] try_b_sel;
  logic [T_W-1:0] try_a_start;
  logic [T_W-1:0] try_b_start;
  logic       last_trial;

  time_t [1:0]   dur_start;
  time_t [1:0]   hi_start;
  logic [1:0]    dur_is2_start;
  logic [1:0]    pf_bw_is128_start;
  logic [1:0]    can_start;
  logic [1:0]    dma_start_valid_start;
  logic [1:0]    room_start;
  logic [1:0]     trial_count_start;
  time_t [1:0]   dur_trial;
  bw_t [1:0]     pf_bw_trial;

  localparam logic [1:0] SEL_TASK_START = 2'd0;
  localparam logic [1:0] SEL_DMA1_END   = 2'd1;
  localparam logic [1:0] SEL_LATEST     = 2'd2;

  function automatic snap_timeline_t apply_s2pf(
    input snap_timeline_t sn,
    input logic [T_W-1:0] dur,
    input logic [BW_W-1:0] pf_bw,
    input logic [T_W-1:0] best_s4,
    input logic [T_W-1:0] pf_start
  );
    snap_timeline_t ret;
    logic [T_W-1:0] pf_end;
    begin
      ret    = sn;
      pf_end = pf_start + dur;
      // Call sites only invoke this for a valid template.  Do not repeat the
      // can_apply comparison here; keeping apply as a pure field patch avoids
      // duplicated duration decode and endpoint checks.
      ret.s2pf_valid = 1'b1;
      ret.s2pf_start = pf_start;
      ret.s2pf_end   = pf_end;
      ret.s2pf_bw    = pf_bw;
      ret.dma3_end   = sn.s2_end;
      ret.s4_start   = sn.s2_end;
      ret.bw_s3      = BW_0;
      ret.task_end   = sn.s2_end + best_s4;
      apply_s2pf = ret;
    end
  endfunction

  function automatic logic shape_s3_dur_is2(input logic [1:0] sh);
    unique case (sh)
      SHAPE_A, SHAPE_B: shape_s3_dur_is2 = 1'b1;
      default:          shape_s3_dur_is2 = 1'b0;
    endcase
  endfunction

  function automatic logic [T_W-1:0] dur_from_is2(input logic is2);
    dur_from_is2 = is2 ? T_W'(2) : T_W'(1);
  endfunction

  function automatic logic [BW_W-1:0] pf_bw_from_is128(input logic is128);
    pf_bw_from_is128 = is128 ? BW_128 : BW_64;
  endfunction

  function automatic logic [T_W-1:0] start_from_sel(
    input int unsigned side,
    input logic [1:0]  sel
  );
    unique case (sel)
      SEL_TASK_START: start_from_sel = (side == SIDE_A) ?
                                       snap_a_i.task_start : snap_b_i.task_start;
      SEL_DMA1_END:   start_from_sel = (side == SIDE_A) ?
                                       snap_a_i.dma1_end : snap_b_i.dma1_end;
      default:        start_from_sel = hi_q[side];
    endcase
  endfunction

  function automatic logic [1:0] policy_trial_count(input s2pf_policy_t policy);
    unique case (policy)
      S2PF_PAIR_LITE:     policy_trial_count = 2'd3;
      S2PF_SPLIT_LITE:    policy_trial_count = 2'd3;
      S2PF_SINGLE_LATEST: policy_trial_count = 2'd1;
      default:            policy_trial_count = 2'd0;
    endcase
  endfunction

  always_comb begin
    dur_is2_start[SIDE_A]     = shape_s3_dur_is2(shape_s3_a_i);
    dur_is2_start[SIDE_B]     = shape_s3_dur_is2(shape_s3_b_i);
    pf_bw_is128_start[SIDE_A] = (shape_s3_a_i == SHAPE_C);
    pf_bw_is128_start[SIDE_B] = (shape_s3_b_i == SHAPE_C);
    dur_start[SIDE_A]         = dur_from_is2(dur_is2_start[SIDE_A]);
    dur_start[SIDE_B]         = dur_from_is2(dur_is2_start[SIDE_B]);

    // Use task_start <= s2_end - dur instead of task_start + dur <= s2_end.
    // This moves the adder out of the repeated trial path and avoids exposing
    // an unnecessary intermediate sum.
    room_start[SIDE_A] = (snap_a_i.s2_end >= dur_start[SIDE_A]);
    room_start[SIDE_B] = (snap_b_i.s2_end >= dur_start[SIDE_B]);
    hi_start[SIDE_A]   = room_start[SIDE_A] ?
                         (snap_a_i.s2_end - dur_start[SIDE_A]) : '0;
    hi_start[SIDE_B]   = room_start[SIDE_B] ?
                         (snap_b_i.s2_end - dur_start[SIDE_B]) : '0;
    can_start[SIDE_A] = (policy_i != S2PF_OFF) && side_a_active_i &&
                        snap_a_i.valid &&
                        (snap_a_i.bw_s3 != BW_0) &&
                        room_start[SIDE_A] &&
                        (snap_a_i.task_start <= hi_start[SIDE_A]);
    can_start[SIDE_B] = (policy_i != S2PF_OFF) && side_b_active_i &&
                        snap_b_i.valid &&
                        (snap_b_i.bw_s3 != BW_0) &&
                        room_start[SIDE_B] &&
                        (snap_b_i.task_start <= hi_start[SIDE_B]);

    dma_start_valid_start[SIDE_A] = can_start[SIDE_A] &&
                                    (snap_a_i.dma1_end >= snap_a_i.task_start) &&
                                    (snap_a_i.dma1_end <= hi_start[SIDE_A]) &&
                                    (snap_a_i.dma1_end != snap_a_i.task_start);
    dma_start_valid_start[SIDE_B] = can_start[SIDE_B] &&
                                    (snap_b_i.dma1_end >= snap_b_i.task_start) &&
                                    (snap_b_i.dma1_end <= hi_start[SIDE_B]) &&
                                    (snap_b_i.dma1_end != snap_b_i.task_start);

    trial_count_start = policy_trial_count(policy_i);
    dur_trial[SIDE_A] = dur_from_is2(dur_is2_q[SIDE_A]);
    dur_trial[SIDE_B] = dur_from_is2(dur_is2_q[SIDE_B]);
    pf_bw_trial[SIDE_A] = pf_bw_from_is128(pf_bw_is128_q[SIDE_A]);
    pf_bw_trial[SIDE_B] = pf_bw_from_is128(pf_bw_is128_q[SIDE_B]);
    last_trial        = ((scan_idx_q + 2'd1) >= trial_count_q);
  end

  always_comb begin
    try_a         = snap_a_i;
    try_b         = snap_b_i;
    try_valid     = 1'b0;
    try_class     = 2'd0;
    try_a_pf      = 1'b0;
    try_b_pf      = 1'b0;
    try_a_sel     = SEL_TASK_START;
    try_b_sel     = SEL_TASK_START;
    try_a_start   = '0;
    try_b_start   = '0;

    unique case (st_q)
      ST_RAW_WAIT: begin
        try_valid = snap_a_i.valid || snap_b_i.valid;
      end

      ST_TRIAL_START,
      ST_TRIAL_WAIT: begin
        unique case (policy_q)
          S2PF_PAIR_LITE: begin
            try_class = 2'd2;
            unique case (scan_idx_q)
              2'd0: begin
                try_a_sel = SEL_TASK_START;
                try_b_sel = SEL_TASK_START;
                try_valid = can_q[SIDE_A] && can_q[SIDE_B];
              end
              2'd1: begin
                try_a_sel = SEL_DMA1_END;
                try_b_sel = SEL_DMA1_END;
                try_valid = dma_start_valid_q[SIDE_A] && dma_start_valid_q[SIDE_B];
              end
              default: begin
                try_a_sel = SEL_LATEST;
                try_b_sel = SEL_LATEST;
                try_valid = can_q[SIDE_A] && can_q[SIDE_B];
              end
            endcase
            try_a_pf      = try_valid;
            try_b_pf      = try_valid;
          end

          S2PF_SPLIT_LITE: begin
            unique case (scan_idx_q)
              2'd0: begin
                try_class = 2'd2;
                try_a_sel = SEL_TASK_START;
                try_b_sel = SEL_TASK_START;
                try_valid = can_q[SIDE_A] && can_q[SIDE_B];
                try_a_pf  = try_valid;
                try_b_pf  = try_valid;
              end
              2'd1: begin
                try_class = 2'd2;
                try_a_sel = SEL_DMA1_END;
                try_b_sel = SEL_DMA1_END;
                try_valid = dma_start_valid_q[SIDE_A] && dma_start_valid_q[SIDE_B];
                try_a_pf  = try_valid;
                try_b_pf  = try_valid;
              end
              default: begin
                try_class = 2'd1;
                try_b_sel = SEL_LATEST;
                try_valid = can_q[SIDE_B];
                try_b_pf  = try_valid;
              end
            endcase
          end

          S2PF_SINGLE_LATEST: begin
            try_class = 2'd1;
            if (side_q[SIDE_A] && !side_q[SIDE_B]) begin
              try_a_sel = SEL_LATEST;
              try_valid = can_q[SIDE_A];
              try_a_pf  = try_valid;
            end else if (side_q[SIDE_B] && !side_q[SIDE_A]) begin
              try_b_sel = SEL_LATEST;
              try_valid = can_q[SIDE_B];
              try_b_pf  = try_valid;
            end
          end

          default: begin
          end
        endcase

        if (try_a_pf) begin
          try_a_start = start_from_sel(SIDE_A, try_a_sel);
        end
        if (try_b_pf) begin
          try_b_start = start_from_sel(SIDE_B, try_b_sel);
        end
        if (try_a_pf) begin
          try_a = apply_s2pf(snap_a_i, dur_trial[SIDE_A], pf_bw_trial[SIDE_A],
                             best_s4_q[SIDE_A], try_a_start);
        end
        if (try_b_pf) begin
          try_b = apply_s2pf(snap_b_i, dur_trial[SIDE_B], pf_bw_trial[SIDE_B],
                             best_s4_q[SIDE_B], try_b_start);
        end
      end

      default: begin
      end
    endcase
  end

  always_comb begin
    st_d             = st_q;
    scan_idx_d       = scan_idx_q;
    policy_d         = policy_q;
    side_d           = side_q;
    best_valid_d     = best_valid_q;
    best_class_d     = best_class_q;
    best_a_pf_d      = best_a_pf_q;
    best_b_pf_d      = best_b_pf_q;
    best_a_start_d   = best_a_start_q;
    best_b_start_d   = best_b_start_q;
    hi_d             = hi_q;
    best_s4_d        = best_s4_q;
    dur_is2_d        = dur_is2_q;
    pf_bw_is128_d    = pf_bw_is128_q;
    can_d            = can_q;
    dma_start_valid_d = dma_start_valid_q;
    trial_count_d    = trial_count_q;
    bw_start_o       = 1'b0;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          policy_d          = policy_i;
          side_d            = {side_b_active_i, side_a_active_i};
          scan_idx_d        = '0;
          best_valid_d      = 1'b0;
          best_class_d      = '0;
          best_a_pf_d       = 1'b0;
          best_b_pf_d       = 1'b0;
          best_a_start_d    = '0;
          best_b_start_d    = '0;
          hi_d              = hi_start;
          best_s4_d[SIDE_A] = best_s4_ticks(snap_a_i.ntok);
          best_s4_d[SIDE_B] = best_s4_ticks(snap_b_i.ntok);
          dur_is2_d         = dur_is2_start;
          pf_bw_is128_d     = pf_bw_is128_start;
          can_d             = can_start;
          dma_start_valid_d = dma_start_valid_start;
          trial_count_d     = trial_count_start;
          st_d              = ST_RAW_START;
        end
      end

      ST_RAW_START: begin
        bw_start_o = 1'b1;
        st_d       = ST_RAW_WAIT;
      end

      ST_RAW_WAIT: begin
        if (bw_done_i) begin
          best_valid_d     = try_valid && bw_ok_i;
          best_class_d     = 2'd0;
          best_a_pf_d      = 1'b0;
          best_b_pf_d      = 1'b0;
          best_a_start_d   = '0;
          best_b_start_d   = '0;
          scan_idx_d       = '0;
          st_d             = (trial_count_q == 2'd0) ? ST_DONE : ST_TRIAL_START;
        end
      end

      ST_TRIAL_START: begin
        if (!try_valid) begin
          if (last_trial) begin
            st_d = ST_DONE;
          end else begin
            scan_idx_d = scan_idx_q + 2'd1;
          end
        end else begin
          bw_start_o = 1'b1;
          st_d       = ST_TRIAL_WAIT;
        end
      end

      ST_TRIAL_WAIT: begin
        if (bw_done_i) begin
          if (try_valid && bw_ok_i) begin
            // Trial order is already sorted by policy priority:
            //   - class is the main priority: both-side S2PF > one-side > raw
            //   - within the same class, scan_idx order is earliest/smallest
            //     start placement first.  Keep the first accepted same-class
            //     trial instead of storing and comparing start_sum.
            if (!best_valid_d || (try_class > best_class_d)) begin
              best_valid_d     = 1'b1;
              best_class_d     = try_class;
              best_a_pf_d      = try_a_pf;
              best_b_pf_d      = try_b_pf;
              best_a_start_d   = try_a_start;
              best_b_start_d   = try_b_start;
            end
          end

          if (last_trial) begin
            st_d = ST_DONE;
          end else begin
            scan_idx_d = scan_idx_q + 2'd1;
            st_d       = ST_TRIAL_START;
          end
        end
      end

      ST_DONE: begin
        st_d = ST_IDLE;
      end

      default: st_d = ST_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q             <= ST_IDLE;
      scan_idx_q       <= '0;
      policy_q         <= S2PF_OFF;
      side_q           <= '0;
      best_valid_q     <= 1'b0;
      best_class_q     <= '0;
      best_a_pf_q      <= 1'b0;
      best_b_pf_q      <= 1'b0;
      best_a_start_q   <= '0;
      best_b_start_q   <= '0;
      hi_q             <= '{default: '0};
      best_s4_q        <= '{default: '0};
      dur_is2_q        <= '0;
      pf_bw_is128_q    <= '0;
      can_q            <= '0;
      dma_start_valid_q <= '0;
      trial_count_q    <= '0;
    end else begin
      st_q             <= st_d;
      scan_idx_q       <= scan_idx_d;
      policy_q         <= policy_d;
      side_q           <= side_d;
      best_valid_q     <= best_valid_d;
      best_class_q     <= best_class_d;
      best_a_pf_q      <= best_a_pf_d;
      best_b_pf_q      <= best_b_pf_d;
      best_a_start_q   <= best_a_start_d;
      best_b_start_q   <= best_b_start_d;
      hi_q             <= hi_d;
      best_s4_q        <= best_s4_d;
      dur_is2_q        <= dur_is2_d;
      pf_bw_is128_q    <= pf_bw_is128_d;
      can_q            <= can_d;
      dma_start_valid_q <= dma_start_valid_d;
      trial_count_q    <= trial_count_d;
    end
  end

  always_comb begin
    patch_o = '0;
    patch_o.ok = best_valid_q;
    patch_o.has_a = best_a_pf_q;
    patch_o.has_b = best_b_pf_q;
    if (best_a_pf_q) begin
      patch_o.pf_start_a = best_a_start_q;
    end
    if (best_b_pf_q) begin
      patch_o.pf_start_b = best_b_start_q;
    end
  end

  // BW checker只需要valid/time/bw字段，不能把完整timeline继续向外广播。
  assign bw_snap_a_o = to_bw_view(try_a);
  assign bw_snap_b_o = to_bw_view(try_b);
  assign busy_o = (st_q != ST_IDLE) && (st_q != ST_DONE);
  assign done_o = (st_q == ST_DONE);

`ifndef SYNTHESIS
  // 本模块只锁存 policy/side/hi/can/duration 等小标量，完整 snap 由调用者
  // 在 busy 期间保持稳定。若调用方破坏该协议，trial BW 请求和最终 patch
  // 会基于不同 snap，仿真必须直接报错。
  always_ff @(posedge clk_i) begin
    if (rst_ni && busy_o && $past(busy_o)) begin
      assert ($stable(snap_a_i))
        else $error("sched_s2pf_pair snap_a_i changed while busy");
      assert ($stable(snap_b_i))
        else $error("sched_s2pf_pair snap_b_i changed while busy");
    end
  end
`endif

endmodule
