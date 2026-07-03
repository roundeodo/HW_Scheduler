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
  input  eval_snap_t   snap_a_i,
  input  eval_snap_t   snap_b_i,

  output logic         bw_start_o,
  output eval_snap_t   bw_snap_a_o,
  output eval_snap_t   bw_snap_b_o,
  input  logic         bw_done_i,
  input  logic         bw_ok_i,

  output logic         ok_o,
  output eval_snap_t   snap_a_o,
  output eval_snap_t   snap_b_o
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

  eval_snap_t   snap_a_q, snap_a_d;
  eval_snap_t   snap_b_q, snap_b_d;
  s2pf_policy_t policy_q, policy_d;
  logic         side_a_q, side_a_d;
  logic         side_b_q, side_b_d;
  logic [1:0]   shape_a_q, shape_a_d;
  logic [1:0]   shape_b_q, shape_b_d;
  logic [1:0]   scan_idx_q, scan_idx_d;

  logic         best_valid_q, best_valid_d;
  logic [1:0]   best_class_q, best_class_d;
  logic [T_W:0] best_start_sum_q, best_start_sum_d;
  logic         best_a_pf_q, best_a_pf_d;
  logic         best_b_pf_q, best_b_pf_d;
  logic [1:0]   best_a_sel_q, best_a_sel_d;
  logic [1:0]   best_b_sel_q, best_b_sel_d;

  logic [T_W-1:0] hi_a;
  logic [T_W-1:0] hi_b;
  logic           can_a;
  logic           can_b;
  logic           dma_start_valid_a;
  logic           dma_start_valid_b;

  eval_snap_t try_a;
  eval_snap_t try_b;
  logic       try_valid;
  logic [1:0] try_class;
  logic [T_W:0] try_start_sum;
  logic       try_a_pf;
  logic       try_b_pf;
  logic [1:0] try_a_sel;
  logic [1:0] try_b_sel;
  logic [1:0] trial_count;
  logic       last_trial;

  localparam logic [1:0] SEL_TASK_START = 2'd0;
  localparam logic [1:0] SEL_DMA1_END   = 2'd1;
  localparam logic [1:0] SEL_LATEST     = 2'd2;

  function automatic logic can_apply_s2pf(
    input eval_snap_t sn,
    input logic [1:0] shape_s3,
    input logic [T_W-1:0] pf_start
  );
    logic [T_W-1:0] dur;
    begin
      dur = kTd3(shape_s3);
      can_apply_s2pf = sn.valid &&
                       (sn.bw_s3 != BW_0) &&
                       (pf_start >= sn.task_start) &&
                       (pf_start + dur <= sn.s2_end);
    end
  endfunction

  function automatic eval_snap_t apply_s2pf(
    input eval_snap_t sn,
    input logic [1:0] shape_s3,
    input logic [T_W-1:0] pf_start
  );
    eval_snap_t ret;
    logic [T_W-1:0] pf_end;
    begin
      ret    = sn;
      pf_end = pf_start + kTd3(shape_s3);
      if (can_apply_s2pf(sn, shape_s3, pf_start)) begin
        ret.s2pf_valid = 1'b1;
        ret.s2pf_start = pf_start;
        ret.s2pf_end   = pf_end;
        ret.s2pf_bw    = alloc_bw(shape_s3);
        ret.dma3_end   = sn.s2_end;
        ret.s4_start   = sn.s2_end;
        ret.bw_s3      = BW_0;
        ret.task_end   = sn.s2_end + best_s4_t(sn.ntok);
      end
      apply_s2pf = ret;
    end
  endfunction

  function automatic logic [T_W-1:0] start_a_from_sel(input logic [1:0] sel);
    unique case (sel)
      SEL_TASK_START: start_a_from_sel = snap_a_q.task_start;
      SEL_DMA1_END:   start_a_from_sel = snap_a_q.dma1_end;
      default:        start_a_from_sel = hi_a;
    endcase
  endfunction

  function automatic logic [T_W-1:0] start_b_from_sel(input logic [1:0] sel);
    unique case (sel)
      SEL_TASK_START: start_b_from_sel = snap_b_q.task_start;
      SEL_DMA1_END:   start_b_from_sel = snap_b_q.dma1_end;
      default:        start_b_from_sel = hi_b;
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
    can_a = (policy_q != S2PF_OFF) && side_a_q &&
            can_apply_s2pf(snap_a_q, shape_a_q, snap_a_q.task_start);
    can_b = (policy_q != S2PF_OFF) && side_b_q &&
            can_apply_s2pf(snap_b_q, shape_b_q, snap_b_q.task_start);

    hi_a = can_a ? (snap_a_q.s2_end - kTd3(shape_a_q)) : '0;
    hi_b = can_b ? (snap_b_q.s2_end - kTd3(shape_b_q)) : '0;

    dma_start_valid_a = can_a &&
                        (snap_a_q.dma1_end >= snap_a_q.task_start) &&
                        (snap_a_q.dma1_end <= hi_a) &&
                        (snap_a_q.dma1_end != snap_a_q.task_start);
    dma_start_valid_b = can_b &&
                        (snap_b_q.dma1_end >= snap_b_q.task_start) &&
                        (snap_b_q.dma1_end <= hi_b) &&
                        (snap_b_q.dma1_end != snap_b_q.task_start);

    trial_count = policy_trial_count(policy_q);
    last_trial  = ((scan_idx_q + 2'd1) >= trial_count);
  end

  always_comb begin
    try_a         = snap_a_q;
    try_b         = snap_b_q;
    try_valid     = 1'b0;
    try_class     = 2'd0;
    try_start_sum = '0;
    try_a_pf      = 1'b0;
    try_b_pf      = 1'b0;
    try_a_sel     = SEL_TASK_START;
    try_b_sel     = SEL_TASK_START;

    unique case (st_q)
      ST_RAW_WAIT: begin
        try_valid = snap_a_q.valid || snap_b_q.valid;
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
                try_valid = can_a && can_b;
              end
              2'd1: begin
                try_a_sel = SEL_DMA1_END;
                try_b_sel = SEL_DMA1_END;
                try_valid = dma_start_valid_a && dma_start_valid_b;
              end
              default: begin
                try_a_sel = SEL_LATEST;
                try_b_sel = SEL_LATEST;
                try_valid = can_a && can_b;
              end
            endcase
            try_a_pf      = try_valid;
            try_b_pf      = try_valid;
            try_start_sum = {1'b0, start_a_from_sel(try_a_sel)} +
                            {1'b0, start_b_from_sel(try_b_sel)};
          end

          S2PF_SPLIT_LITE: begin
            unique case (scan_idx_q)
              2'd0: begin
                try_class = 2'd2;
                try_a_sel = SEL_TASK_START;
                try_b_sel = SEL_TASK_START;
                try_valid = can_a && can_b;
                try_a_pf  = try_valid;
                try_b_pf  = try_valid;
                try_start_sum = {1'b0, start_a_from_sel(try_a_sel)} +
                                {1'b0, start_b_from_sel(try_b_sel)};
              end
              2'd1: begin
                try_class = 2'd2;
                try_a_sel = SEL_DMA1_END;
                try_b_sel = SEL_DMA1_END;
                try_valid = dma_start_valid_a && dma_start_valid_b;
                try_a_pf  = try_valid;
                try_b_pf  = try_valid;
                try_start_sum = {1'b0, start_a_from_sel(try_a_sel)} +
                                {1'b0, start_b_from_sel(try_b_sel)};
              end
              default: begin
                try_class = 2'd1;
                try_b_sel = SEL_LATEST;
                try_valid = can_b;
                try_b_pf  = try_valid;
                try_start_sum = {1'b0, start_b_from_sel(try_b_sel)};
              end
            endcase
          end

          S2PF_SINGLE_LATEST: begin
            try_class = 2'd1;
            if (side_a_q && !side_b_q) begin
              try_a_sel = SEL_LATEST;
              try_valid = can_a;
              try_a_pf  = try_valid;
              try_start_sum = {1'b0, start_a_from_sel(try_a_sel)};
            end else if (side_b_q && !side_a_q) begin
              try_b_sel = SEL_LATEST;
              try_valid = can_b;
              try_b_pf  = try_valid;
              try_start_sum = {1'b0, start_b_from_sel(try_b_sel)};
            end
          end

          default: begin
          end
        endcase

        if (try_a_pf) begin
          try_a = apply_s2pf(snap_a_q, shape_a_q, start_a_from_sel(try_a_sel));
        end
        if (try_b_pf) begin
          try_b = apply_s2pf(snap_b_q, shape_b_q, start_b_from_sel(try_b_sel));
        end
      end

      default: begin
      end
    endcase
  end

  always_comb begin
    st_d             = st_q;
    snap_a_d         = snap_a_q;
    snap_b_d         = snap_b_q;
    policy_d         = policy_q;
    side_a_d         = side_a_q;
    side_b_d         = side_b_q;
    shape_a_d        = shape_a_q;
    shape_b_d        = shape_b_q;
    scan_idx_d       = scan_idx_q;
    best_valid_d     = best_valid_q;
    best_class_d     = best_class_q;
    best_start_sum_d = best_start_sum_q;
    best_a_pf_d      = best_a_pf_q;
    best_b_pf_d      = best_b_pf_q;
    best_a_sel_d     = best_a_sel_q;
    best_b_sel_d     = best_b_sel_q;
    bw_start_o       = 1'b0;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          snap_a_d          = snap_a_i;
          snap_b_d          = snap_b_i;
          policy_d          = policy_i;
          side_a_d          = side_a_active_i;
          side_b_d          = side_b_active_i;
          shape_a_d         = shape_s3_a_i;
          shape_b_d         = shape_s3_b_i;
          scan_idx_d        = '0;
          best_valid_d      = 1'b0;
          best_class_d      = '0;
          best_start_sum_d  = '0;
          best_a_pf_d       = 1'b0;
          best_b_pf_d       = 1'b0;
          best_a_sel_d      = SEL_TASK_START;
          best_b_sel_d      = SEL_TASK_START;
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
          best_start_sum_d = '0;
          best_a_pf_d      = 1'b0;
          best_b_pf_d      = 1'b0;
          best_a_sel_d     = SEL_TASK_START;
          best_b_sel_d     = SEL_TASK_START;
          scan_idx_d       = '0;
          st_d             = (trial_count == 2'd0) ? ST_DONE : ST_TRIAL_START;
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
            if (!best_valid_d ||
                (try_class > best_class_d) ||
                ((try_class == best_class_d) &&
                 (try_start_sum < best_start_sum_d))) begin
              best_valid_d     = 1'b1;
              best_class_d     = try_class;
              best_start_sum_d = try_start_sum;
              best_a_pf_d      = try_a_pf;
              best_b_pf_d      = try_b_pf;
              best_a_sel_d     = try_a_sel;
              best_b_sel_d     = try_b_sel;
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
      snap_a_q         <= '0;
      snap_b_q         <= '0;
      policy_q         <= S2PF_OFF;
      side_a_q         <= 1'b0;
      side_b_q         <= 1'b0;
      shape_a_q        <= '0;
      shape_b_q        <= '0;
      scan_idx_q       <= '0;
      best_valid_q     <= 1'b0;
      best_class_q     <= '0;
      best_start_sum_q <= '0;
      best_a_pf_q      <= 1'b0;
      best_b_pf_q      <= 1'b0;
      best_a_sel_q     <= SEL_TASK_START;
      best_b_sel_q     <= SEL_TASK_START;
    end else begin
      st_q             <= st_d;
      snap_a_q         <= snap_a_d;
      snap_b_q         <= snap_b_d;
      policy_q         <= policy_d;
      side_a_q         <= side_a_d;
      side_b_q         <= side_b_d;
      shape_a_q        <= shape_a_d;
      shape_b_q        <= shape_b_d;
      scan_idx_q       <= scan_idx_d;
      best_valid_q     <= best_valid_d;
      best_class_q     <= best_class_d;
      best_start_sum_q <= best_start_sum_d;
      best_a_pf_q      <= best_a_pf_d;
      best_b_pf_q      <= best_b_pf_d;
      best_a_sel_q     <= best_a_sel_d;
      best_b_sel_q     <= best_b_sel_d;
    end
  end

  always_comb begin
    snap_a_o = best_a_pf_q ? apply_s2pf(snap_a_q, shape_a_q,
                                        start_a_from_sel(best_a_sel_q)) : snap_a_q;
    snap_b_o = best_b_pf_q ? apply_s2pf(snap_b_q, shape_b_q,
                                        start_b_from_sel(best_b_sel_q)) : snap_b_q;
  end

  assign ok_o   = best_valid_q;
  assign bw_snap_a_o = try_a;
  assign bw_snap_b_o = try_b;
  assign busy_o = (st_q != ST_IDLE) && (st_q != ST_DONE);
  assign done_o = (st_q == ST_DONE);

endmodule
