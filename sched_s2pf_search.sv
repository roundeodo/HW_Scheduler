// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - priority-first fixed S2PF search
//
// Templates are visited in final selection priority order.  The first legal
// BW result is therefore the winner and the search stops immediately:
//
//   PAIR  : both@dma1_end, raw
//   SPLIT : both@dma1_end, B-only@dma1_end, raw
//   SINGLE: active-side@dma1_end, raw
//   OFF   : raw
//
// The template order directly implements class priority and earliest-start
// tie-breaking, so no provisional winner or best-class comparator is needed.
// Trial construction produces a compact DMA-resource view directly; task_end
// and cache fields are not part of resource validation and are never
// materialized here.

import sched_pkg::*;

module sched_s2pf_search (
  input  logic           clk_i,
  input  logic           rst_ni,
  input  logic           clear_i,
  input  logic           start_i,
  output logic           done_o,

  input  s2pf_policy_t   policy_i,
  input  logic           side_a_active_i,
  input  logic           side_b_active_i,
  input  wire snap_timeline_t snap_a_i,
  input  wire snap_timeline_t snap_b_i,

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
  state_t      st_q, st_d;
  logic [1:0]  trial_index_q, trial_index_d;

  // Start-time predecode. Policy/timeline inputs remain stable while busy, so
  // only endpoint legality is registered; DMA binding stays local to the trial.
  logic [1:0]  eligible_side_q, eligible_side_d;

  // Accepted result only.  No provisional raw winner or class state.
  logic        selected_valid_q, selected_valid_d;
  logic [1:0]  selected_side_mask_q, selected_side_mask_d;

  logic [1:0]  fits_prefetch_window;
  logic [1:0]  eligible_side_at_start;

  logic [1:0]  trial_side_mask;
  logic        trial_valid;
  logic        last_trial;
  snap_bw_view_t trial_bw_a;
  snap_bw_view_t trial_bw_b;

  function automatic snap_bw_view_t apply_s2pf_dma(
    input snap_timeline_t sn,
    input time_t          pf_start
  );
    snap_bw_view_t v;
    begin
      v = to_bw_view(sn);
      v.s2pf_valid = 1'b1;
      v.s2pf_start = pf_start;
      v.s2pf_end   = pf_start + S2PF_DMA_TICKS;
      v.s2pf_dma   = DMA_BOTH;
      v.dma3_end   = sn.s2_end;
      v.dma_s3     = DMA_NONE;
      apply_s2pf_dma = v;
    end
  endfunction

  always_comb begin
    // S2PF 固定占用 BOTH DMA 一个 tick，不再从 S3 shape 派生
    // 可变资源/时长。窗口不足或资源冲突时由现有 trial 直接拒绝。

    // dma1_end can lie after s2_end for a legal task shape.  Guard the
    // unsigned subtraction explicitly; otherwise the wrapped difference would
    // turn an empty S2PF window into a large, apparently legal window.
    fits_prefetch_window[SIDE_A] =
        (snap_a_i.s2_end >= snap_a_i.dma1_end) &&
        (S2PF_DMA_TICKS <= (snap_a_i.s2_end - snap_a_i.dma1_end));
    fits_prefetch_window[SIDE_B] =
        (snap_b_i.s2_end >= snap_b_i.dma1_end) &&
        (S2PF_DMA_TICKS <= (snap_b_i.s2_end - snap_b_i.dma1_end));

    eligible_side_at_start[SIDE_A] =
        side_a_active_i &&
        (snap_a_i.dma_s3 != DMA_NONE) && fits_prefetch_window[SIDE_A];
    eligible_side_at_start[SIDE_B] =
        side_b_active_i &&
        (snap_b_i.dma_s3 != DMA_NONE) && fits_prefetch_window[SIDE_B];
  end

  // Fixed microcode decoder.  It emits only the side mask;
  // full candidate objects and class metadata do not exist in this block.
  always_comb begin
    trial_side_mask = '0;
    trial_valid     = 1'b0;
    last_trial     = 1'b0;

    unique case (policy_i)
      S2PF_PAIR: begin
        last_trial = (trial_index_q == 2'd1);
        unique case (trial_index_q)
          2'd0: begin
            trial_side_mask = 2'b11;
            trial_valid = eligible_side_q[SIDE_A] && eligible_side_q[SIDE_B];
          end
          2'd1: begin
            trial_valid = 1'b1;
          end
          default: begin
            trial_valid = 1'bx;
            last_trial  = 1'bx;
          end
        endcase
      end

      S2PF_SPLIT: begin
        last_trial = (trial_index_q == 2'd2);
        unique case (trial_index_q)
          2'd0: begin
            trial_side_mask = 2'b11;
            trial_valid = eligible_side_q[SIDE_A] && eligible_side_q[SIDE_B];
          end
          2'd1: begin
            trial_side_mask[SIDE_B] = 1'b1;
            trial_valid = eligible_side_q[SIDE_B];
          end
          2'd2: begin
            trial_valid = 1'b1;
          end
          default: begin
            trial_valid = 1'bx;
            last_trial  = 1'bx;
          end
        endcase
      end

      S2PF_ACTIVE_SIDE: begin
        last_trial = (trial_index_q == 2'd1);
        if (trial_index_q == 2'd0) begin
          if (eligible_side_q[SIDE_A]) begin
            trial_side_mask[SIDE_A] = 1'b1;
            trial_valid = 1'b1;
          end else if (eligible_side_q[SIDE_B]) begin
            trial_side_mask[SIDE_B] = 1'b1;
            trial_valid = 1'b1;
          end
        end else if (trial_index_q == 2'd1) begin
          trial_valid = 1'b1;
        end else begin
          trial_valid = 1'bx;
          last_trial  = 1'bx;
        end
      end

      S2PF_DISABLED: begin
        last_trial = 1'b1;
        trial_valid = 1'b1;
      end
    endcase

    trial_bw_a = to_bw_view(snap_a_i);
    trial_bw_b = to_bw_view(snap_b_i);
    if (trial_side_mask[SIDE_A]) begin
      trial_bw_a = apply_s2pf_dma(snap_a_i, snap_a_i.dma1_end);
    end
    if (trial_side_mask[SIDE_B]) begin
      trial_bw_b = apply_s2pf_dma(snap_b_i, snap_b_i.dma1_end);
    end
  end

  always_comb begin
    st_d                  = st_q;
    trial_index_d         = trial_index_q;
    eligible_side_d       = eligible_side_q;
    selected_valid_d      = selected_valid_q;
    selected_side_mask_d  = selected_side_mask_q;
    bw_start_o            = 1'b0;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          trial_index_d     = '0;
          eligible_side_d      = eligible_side_at_start;
          selected_valid_d     = 1'b0;
          selected_side_mask_d = '0;
          st_d              = ST_TRIAL_START;
        end
      end

      ST_TRIAL_START: begin
        if (trial_valid) begin
          bw_start_o = 1'b1;
          st_d       = ST_TRIAL_WAIT;
        end else if (last_trial) begin
          st_d = ST_DONE;
        end else begin
          trial_index_d = trial_index_q + 2'd1;
        end
      end

      ST_TRIAL_WAIT: begin
        if (bw_done_i) begin
          if (bw_ok_i) begin
            selected_valid_d     = 1'b1;
            selected_side_mask_d = trial_side_mask;
            st_d            = ST_DONE;
          end else if (last_trial) begin
            st_d = ST_DONE;
          end else begin
            trial_index_d = trial_index_q + 2'd1;
            st_d       = ST_TRIAL_START;
          end
        end
      end

      ST_DONE: st_d = ST_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q                  <= ST_IDLE;
      trial_index_q         <= '0;
      eligible_side_q       <= '0;
      selected_valid_q      <= 1'b0;
      selected_side_mask_q  <= '0;
    end else if (clear_i) begin
      st_q                  <= ST_IDLE;
      trial_index_q         <= '0;
      eligible_side_q       <= '0;
      selected_valid_q      <= 1'b0;
      selected_side_mask_q  <= '0;
    end else begin
      st_q                  <= st_d;
      trial_index_q         <= trial_index_d;
      eligible_side_q       <= eligible_side_d;
      selected_valid_q      <= selected_valid_d;
      selected_side_mask_q  <= selected_side_mask_d;
    end
  end

  always_comb begin
    patch_o = '0;
    patch_o.valid      = selected_valid_q;
    patch_o.apply_a    = selected_side_mask_q[SIDE_A];
    patch_o.apply_b    = selected_side_mask_q[SIDE_B];
  end

  assign bw_snap_a_o = trial_bw_a;
  assign bw_snap_b_o = trial_bw_b;
  assign done_o      = (st_q == ST_DONE);

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni && start_i) begin
      assert (side_a_active_i || side_b_active_i);
      if (side_a_active_i) begin
        assert (snap_a_i.valid);
      end
      if (side_b_active_i) begin
        assert (snap_b_i.valid);
      end
    end
    if (rst_ni && (st_q != ST_IDLE) && (st_q != ST_DONE) &&
        $past((st_q != ST_IDLE) && (st_q != ST_DONE))) begin
      assert ($stable(snap_a_i) && $stable(snap_b_i));
      assert ($stable(policy_i));
    end
  end
`endif

endmodule
