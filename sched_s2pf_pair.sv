// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - priority-first lite S2PF search
//
// Templates are visited in final selection priority order.  The first legal
// BW result is therefore the winner and the search stops immediately:
//
//   PAIR  : both@task_start, both@dma1_end, both@latest, raw
//   SPLIT : both@task_start, both@dma1_end, B-only@latest, raw
//   SINGLE: active-side@latest, raw
//   OFF   : raw
//
// This is equivalent to the previous class-first/earliest tie-break, but it
// removes the raw-first provisional winner and all best-class comparisons.
// Trial construction produces a compact BW view directly; task_end and cache
// fields are not part of BW validation and are never materialized here.

import sched_pkg::*;

module sched_s2pf_pair (
  input  logic           clk_i,
  input  logic           rst_ni,
  input  logic           clear_i,
  input  logic           start_i,
  output logic           done_o,

  input  s2pf_policy_t   policy_i,
  input  logic           side_a_active_i,
  input  logic           side_b_active_i,
  input  shape_t         shape_s3_a_i,
  input  shape_t         shape_s3_b_i,
  input  snap_timeline_t snap_a_i,
  input  snap_timeline_t snap_b_i,

  output logic           bw_start_o,
  output snap_bw_view_t  bw_snap_a_o,
  output snap_bw_view_t  bw_snap_b_o,
  input  logic           bw_done_i,
  input  logic           bw_ok_i,

  output s2pf_patch_t    patch_o
);

  typedef enum logic [1:0] {
    ST_IDLE,
    ST_TRIAL_START,
    ST_TRIAL_WAIT,
    ST_DONE
  } state_t;

  localparam int unsigned SIDE_A = 0;
  localparam int unsigned SIDE_B = 1;
  localparam logic [1:0] SEL_TASK_START = 2'd0;
  localparam logic [1:0] SEL_DMA1_END   = 2'd1;
  localparam logic [1:0] SEL_LATEST     = 2'd2;

  state_t      st_q, st_d;
  s2pf_policy_t policy_q, policy_d;
  logic [1:0]  scan_idx_q, scan_idx_d;

  // Start-time predecode.  Shape inputs remain stable while busy, so only
  // endpoint legality is registered; duration/BW decode stays at the local
  // trial patch point.
  time_t [1:0] hi_q, hi_d;
  logic [1:0]  can_q, can_d;
  logic [1:0]  dma_start_valid_q, dma_start_valid_d;

  // Accepted result only.  No provisional raw winner or class state.
  logic        result_ok_q, result_ok_d;
  logic [1:0]  result_has_pf_q, result_has_pf_d;
  time_t [1:0] result_start_q, result_start_d;

  time_t [1:0] dur_start;
  time_t [1:0] hi_start;
  logic [1:0]  room_start;
  logic [1:0]  can_start;
  logic [1:0]  dma_start_valid_start;

  logic [1:0]  try_has_pf;
  logic [1:0]  try_sel [2];
  time_t [1:0] try_start;
  logic        try_valid;
  logic        try_is_raw;
  logic        last_trial;
  snap_bw_view_t try_a_bw;
  snap_bw_view_t try_b_bw;

  function automatic logic [1:0] policy_last_idx(input s2pf_policy_t policy);
    unique case (policy)
      S2PF_PAIR_LITE,
      S2PF_SPLIT_LITE:    policy_last_idx = 2'd3;
      S2PF_SINGLE_LATEST: policy_last_idx = 2'd1;
      default:            policy_last_idx = 2'd0;
    endcase
  endfunction

  function automatic time_t select_start(
    input snap_timeline_t sn,
    input time_t          hi,
    input logic [1:0]     sel
  );
    unique case (sel)
      SEL_TASK_START: select_start = sn.task_start;
      SEL_DMA1_END:   select_start = sn.dma1_end;
      default:        select_start = hi;
    endcase
  endfunction

  function automatic snap_bw_view_t apply_s2pf_bw(
    input snap_timeline_t sn,
    input logic           shape_is_c,
    input time_t          pf_start
  );
    snap_bw_view_t v;
    time_t         pf_dur;
    begin
      v = to_bw_view(sn);
      pf_dur       = shape_is_c ? time_t'(1) : time_t'(2);
      v.s2pf_valid = 1'b1;
      v.s2pf_start = pf_start;
      v.s2pf_end   = pf_start + pf_dur;
      v.s2pf_bw    = shape_is_c ? BW_128 : BW_64;
      v.dma3_end   = sn.s2_end;
      v.bw_s3      = BW_0;
      apply_s2pf_bw = v;
    end
  endfunction

  always_comb begin
    dur_start[SIDE_A] = (shape_s3_a_i == SHAPE_C) ? time_t'(1) : time_t'(2);
    dur_start[SIDE_B] = (shape_s3_b_i == SHAPE_C) ? time_t'(1) : time_t'(2);

    room_start[SIDE_A] = (snap_a_i.s2_end >= dur_start[SIDE_A]);
    room_start[SIDE_B] = (snap_b_i.s2_end >= dur_start[SIDE_B]);
    hi_start[SIDE_A] = room_start[SIDE_A] ?
                       (snap_a_i.s2_end - dur_start[SIDE_A]) : '0;
    hi_start[SIDE_B] = room_start[SIDE_B] ?
                       (snap_b_i.s2_end - dur_start[SIDE_B]) : '0;

    can_start[SIDE_A] = (policy_i != S2PF_OFF) && side_a_active_i &&
                        snap_a_i.valid && (snap_a_i.bw_s3 != BW_0) &&
                        room_start[SIDE_A] &&
                        (snap_a_i.task_start <= hi_start[SIDE_A]);
    can_start[SIDE_B] = (policy_i != S2PF_OFF) && side_b_active_i &&
                        snap_b_i.valid && (snap_b_i.bw_s3 != BW_0) &&
                        room_start[SIDE_B] &&
                        (snap_b_i.task_start <= hi_start[SIDE_B]);

    dma_start_valid_start[SIDE_A] = can_start[SIDE_A] &&
                                    (snap_a_i.dma1_end > snap_a_i.task_start) &&
                                    (snap_a_i.dma1_end <= hi_start[SIDE_A]);
    dma_start_valid_start[SIDE_B] = can_start[SIDE_B] &&
                                    (snap_b_i.dma1_end > snap_b_i.task_start) &&
                                    (snap_b_i.dma1_end <= hi_start[SIDE_B]);
  end

  // Fixed microcode decoder.  It emits only the side mask and start selector;
  // full candidate objects and class metadata do not exist in this block.
  always_comb begin
    try_has_pf = '0;
    try_sel[SIDE_A] = SEL_TASK_START;
    try_sel[SIDE_B] = SEL_TASK_START;
    try_valid  = 1'b0;
    try_is_raw = 1'b0;

    unique case (policy_q)
      S2PF_PAIR_LITE: begin
        unique case (scan_idx_q)
          2'd0: begin
            try_has_pf = 2'b11;
            try_valid  = can_q[SIDE_A] && can_q[SIDE_B];
          end
          2'd1: begin
            try_has_pf = 2'b11;
            try_sel[SIDE_A] = SEL_DMA1_END;
            try_sel[SIDE_B] = SEL_DMA1_END;
            try_valid = dma_start_valid_q[SIDE_A] && dma_start_valid_q[SIDE_B];
          end
          2'd2: begin
            try_has_pf = 2'b11;
            try_sel[SIDE_A] = SEL_LATEST;
            try_sel[SIDE_B] = SEL_LATEST;
            try_valid = can_q[SIDE_A] && can_q[SIDE_B];
          end
          default: begin
            try_is_raw = 1'b1;
            try_valid  = snap_a_i.valid || snap_b_i.valid;
          end
        endcase
      end

      S2PF_SPLIT_LITE: begin
        unique case (scan_idx_q)
          2'd0: begin
            try_has_pf = 2'b11;
            try_valid  = can_q[SIDE_A] && can_q[SIDE_B];
          end
          2'd1: begin
            try_has_pf = 2'b11;
            try_sel[SIDE_A] = SEL_DMA1_END;
            try_sel[SIDE_B] = SEL_DMA1_END;
            try_valid = dma_start_valid_q[SIDE_A] && dma_start_valid_q[SIDE_B];
          end
          2'd2: begin
            try_has_pf[SIDE_B] = 1'b1;
            try_sel[SIDE_B] = SEL_LATEST;
            try_valid = can_q[SIDE_B];
          end
          default: begin
            try_is_raw = 1'b1;
            try_valid  = snap_a_i.valid || snap_b_i.valid;
          end
        endcase
      end

      S2PF_SINGLE_LATEST: begin
        if (scan_idx_q == 2'd0) begin
          if (can_q[SIDE_A]) begin
            try_has_pf[SIDE_A] = 1'b1;
            try_sel[SIDE_A] = SEL_LATEST;
            try_valid = 1'b1;
          end else if (can_q[SIDE_B]) begin
            try_has_pf[SIDE_B] = 1'b1;
            try_sel[SIDE_B] = SEL_LATEST;
            try_valid = 1'b1;
          end
        end else begin
          try_is_raw = 1'b1;
          try_valid  = snap_a_i.valid || snap_b_i.valid;
        end
      end

      default: begin
        try_is_raw = 1'b1;
        try_valid  = snap_a_i.valid || snap_b_i.valid;
      end
    endcase

    try_start[SIDE_A] = select_start(snap_a_i, hi_q[SIDE_A], try_sel[SIDE_A]);
    try_start[SIDE_B] = select_start(snap_b_i, hi_q[SIDE_B], try_sel[SIDE_B]);

    try_a_bw = to_bw_view(snap_a_i);
    try_b_bw = to_bw_view(snap_b_i);
    if (!try_is_raw && try_has_pf[SIDE_A]) begin
      try_a_bw = apply_s2pf_bw(snap_a_i, shape_s3_a_i == SHAPE_C,
                               try_start[SIDE_A]);
    end
    if (!try_is_raw && try_has_pf[SIDE_B]) begin
      try_b_bw = apply_s2pf_bw(snap_b_i, shape_s3_b_i == SHAPE_C,
                               try_start[SIDE_B]);
    end
  end

  assign last_trial = (scan_idx_q == policy_last_idx(policy_q));

  always_comb begin
    st_d                  = st_q;
    policy_d              = policy_q;
    scan_idx_d            = scan_idx_q;
    hi_d                  = hi_q;
    can_d                 = can_q;
    dma_start_valid_d     = dma_start_valid_q;
    result_ok_d           = result_ok_q;
    result_has_pf_d       = result_has_pf_q;
    result_start_d        = result_start_q;
    bw_start_o            = 1'b0;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          policy_d          = policy_i;
          scan_idx_d        = '0;
          hi_d              = hi_start;
          can_d             = can_start;
          dma_start_valid_d = dma_start_valid_start;
          result_ok_d       = 1'b0;
          result_has_pf_d   = '0;
          result_start_d    = '{default: '0};
          st_d              = ST_TRIAL_START;
        end
      end

      ST_TRIAL_START: begin
        if (try_valid) begin
          bw_start_o = 1'b1;
          st_d       = ST_TRIAL_WAIT;
        end else if (last_trial) begin
          st_d = ST_DONE;
        end else begin
          scan_idx_d = scan_idx_q + 2'd1;
        end
      end

      ST_TRIAL_WAIT: begin
        if (bw_done_i) begin
          if (bw_ok_i) begin
            result_ok_d     = 1'b1;
            result_has_pf_d = try_is_raw ? 2'b00 : try_has_pf;
            result_start_d  = try_start;
            st_d            = ST_DONE;
          end else if (last_trial) begin
            st_d = ST_DONE;
          end else begin
            scan_idx_d = scan_idx_q + 2'd1;
            st_d       = ST_TRIAL_START;
          end
        end
      end

      ST_DONE: st_d = ST_IDLE;
      default: st_d = ST_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q                  <= ST_IDLE;
      policy_q              <= S2PF_OFF;
      scan_idx_q            <= '0;
      hi_q                  <= '{default: '0};
      can_q                 <= '0;
      dma_start_valid_q     <= '0;
      result_ok_q           <= 1'b0;
      result_has_pf_q       <= '0;
      result_start_q        <= '{default: '0};
    end else if (clear_i) begin
      st_q                  <= ST_IDLE;
      policy_q              <= S2PF_OFF;
      scan_idx_q            <= '0;
      hi_q                  <= '{default: '0};
      can_q                 <= '0;
      dma_start_valid_q     <= '0;
      result_ok_q           <= 1'b0;
      result_has_pf_q       <= '0;
      result_start_q        <= '{default: '0};
    end else begin
      st_q                  <= st_d;
      policy_q              <= policy_d;
      scan_idx_q            <= scan_idx_d;
      hi_q                  <= hi_d;
      can_q                 <= can_d;
      dma_start_valid_q     <= dma_start_valid_d;
      result_ok_q           <= result_ok_d;
      result_has_pf_q       <= result_has_pf_d;
      result_start_q        <= result_start_d;
    end
  end

  always_comb begin
    patch_o = '0;
    patch_o.ok         = result_ok_q;
    patch_o.has_a      = result_has_pf_q[SIDE_A];
    patch_o.has_b      = result_has_pf_q[SIDE_B];
    patch_o.pf_start_a = result_start_q[SIDE_A];
    patch_o.pf_start_b = result_start_q[SIDE_B];
  end

  assign bw_snap_a_o = try_a_bw;
  assign bw_snap_b_o = try_b_bw;
  assign done_o      = (st_q == ST_DONE);

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni && (st_q != ST_IDLE) && (st_q != ST_DONE) &&
        $past((st_q != ST_IDLE) && (st_q != ST_DONE))) begin
      assert ($stable(snap_a_i) && $stable(snap_b_i));
      assert ($stable(shape_s3_a_i) && $stable(shape_s3_b_i));
      assert ($stable(policy_i));
    end
  end
`endif

endmodule
