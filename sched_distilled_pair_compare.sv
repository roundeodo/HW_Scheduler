// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51

import sched_pkg::*;
import sched_distilled_pkg::*;

module sched_distilled_pair_compare (
  input  logic                    clk_i,
  input  logic                    rst_ni,
  input  logic                    clear_i,
  input  logic                    start_i,
  output logic                    done_o,
  input  wire distilled_mode_t    mode_i,
  input  wire distilled_regime_t  regime_i,
  input  logic [NR_W-1:0]         before_count_i,
  input  ntok_t                   min_remaining_load_i,
  input  wire distilled_score_record_t lhs_i,
  input  wire distilled_score_record_t rhs_i,
  input  logic                    children_differ_i,
  output logic                    rhs_wins_o,
  output logic                    override_o
);

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_BASE,
    ST_PROGRESS,
    ST_HOTSPOT,
    ST_OVERRIDE,
    ST_DONE
  } state_t;

  typedef enum logic [1:0] {
    KEY_BASE,
    KEY_PROGRESS,
    KEY_HOTSPOT
  } key_t;

  state_t st_q, st_d;
  logic [3:0] field_q, field_d;
  logic base_rhs_q, base_rhs_d;
  logic progress_rhs_q, progress_rhs_d;
  logic hotspot_rhs_q, hotspot_rhs_d;
  logic rhs_wins_q, rhs_wins_d;
  logic override_q, override_d;

  logic key_different;
  logic key_rhs_better;
  key_t active_key;
  logic need_progress;
  logic need_hotspot;
  distilled_score_record_t base_record;
  distilled_score_record_t progress_record;
  distilled_score_record_t hotspot_record;
  logic [NR_W-1:0] cold_rank_floor;

  function automatic logic lower_rhs(
    input logic [DIST_BOUND_W-1:0] lhs,
    input logic [DIST_BOUND_W-1:0] rhs
  );
    lower_rhs = rhs < lhs;
  endfunction

  always_comb begin
    active_key = KEY_BASE;
    unique case (st_q)
      ST_PROGRESS: active_key = KEY_PROGRESS;
      ST_HOTSPOT:  active_key = KEY_HOTSPOT;
      default: begin
      end
    endcase
  end

  // The FSM exits a key as soon as the first unequal field is observed.
  always_comb begin
    key_different = 1'b0;
    key_rhs_better = 1'b0;
    unique case (active_key)
      KEY_BASE: begin
        unique case (field_q)
          4'd0: begin key_different = lhs_i.f != rhs_i.f;
                      key_rhs_better = lower_rhs(lhs_i.f, rhs_i.f); end
          4'd1: begin key_different = lhs_i.h != rhs_i.h;
                      key_rhs_better = lower_rhs(lhs_i.h, rhs_i.h); end
          4'd2: begin key_different = lhs_i.compute_bound != rhs_i.compute_bound;
                      key_rhs_better = lower_rhs(lhs_i.compute_bound,
                                                 rhs_i.compute_bound); end
          4'd3: begin key_different = lhs_i.dma_bound != rhs_i.dma_bound;
                      key_rhs_better = lower_rhs(lhs_i.dma_bound,
                                                 rhs_i.dma_bound); end
          4'd4: begin
            if (mode_i == DIST_MODE_SYNC) begin
              key_different = lhs_i.selected_max != rhs_i.selected_max;
              key_rhs_better = rhs_i.selected_max > lhs_i.selected_max;
            end else begin
              key_different = lhs_i.late_end != rhs_i.late_end;
              key_rhs_better = rhs_i.late_end < lhs_i.late_end;
            end
          end
          4'd5: begin
            if (mode_i == DIST_MODE_SYNC) begin
              key_different = lhs_i.selected_sum != rhs_i.selected_sum;
              key_rhs_better = rhs_i.selected_sum < lhs_i.selected_sum;
            end else begin
              key_different = lhs_i.early_end != rhs_i.early_end;
              key_rhs_better = rhs_i.early_end < lhs_i.early_end;
            end
          end
          4'd6: begin
            if (mode_i == DIST_MODE_SYNC) begin
              key_different = lhs_i.late_end != rhs_i.late_end;
              key_rhs_better = rhs_i.late_end < lhs_i.late_end;
            end else begin
              key_different = lhs_i.selected_sum != rhs_i.selected_sum;
              key_rhs_better = rhs_i.selected_sum < lhs_i.selected_sum;
            end
          end
          4'd7: begin
            key_different = lhs_i.s2pf_count != rhs_i.s2pf_count;
            key_rhs_better = rhs_i.s2pf_count > lhs_i.s2pf_count;
          end
          4'd8: begin
            if (mode_i != DIST_MODE_SYNC) begin
              key_different = lhs_i.remaining_count != rhs_i.remaining_count;
              key_rhs_better = rhs_i.remaining_count < lhs_i.remaining_count;
            end
          end
          default: begin
          end
        endcase
      end

      KEY_PROGRESS: begin
        unique case (field_q)
          4'd0: begin key_different = lhs_i.f != rhs_i.f;
                      key_rhs_better = rhs_i.f < lhs_i.f; end
          4'd1: begin key_different = lhs_i.h != rhs_i.h;
                      key_rhs_better = rhs_i.h < lhs_i.h; end
          4'd2: begin key_different = lhs_i.s2pf_count != rhs_i.s2pf_count;
                      key_rhs_better = rhs_i.s2pf_count > lhs_i.s2pf_count; end
          4'd3: begin key_different = lhs_i.selected_sum != rhs_i.selected_sum;
                      key_rhs_better = rhs_i.selected_sum > lhs_i.selected_sum; end
          4'd4: begin key_different = lhs_i.compute_bound != rhs_i.compute_bound;
                      key_rhs_better = rhs_i.compute_bound < lhs_i.compute_bound; end
          4'd5: begin key_different = lhs_i.dma_bound != rhs_i.dma_bound;
                      key_rhs_better = rhs_i.dma_bound < lhs_i.dma_bound; end
          4'd6: begin key_different = lhs_i.late_end != rhs_i.late_end;
                      key_rhs_better = rhs_i.late_end < lhs_i.late_end; end
          4'd7: begin key_different = lhs_i.early_end != rhs_i.early_end;
                      key_rhs_better = rhs_i.early_end < lhs_i.early_end; end
          4'd8: begin key_different = lhs_i.selected_max != rhs_i.selected_max;
                      key_rhs_better = rhs_i.selected_max > lhs_i.selected_max; end
          default: begin
          end
        endcase
      end

      KEY_HOTSPOT: begin
        unique case (field_q)
          4'd0: begin key_different = lhs_i.f != rhs_i.f;
                      key_rhs_better = rhs_i.f < lhs_i.f; end
          4'd1: begin key_different = lhs_i.h != rhs_i.h;
                      key_rhs_better = rhs_i.h < lhs_i.h; end
          4'd2: begin key_different = lhs_i.selected_max != rhs_i.selected_max;
                      key_rhs_better = rhs_i.selected_max > lhs_i.selected_max; end
          4'd3: begin key_different = lhs_i.selected_sum != rhs_i.selected_sum;
                      key_rhs_better = rhs_i.selected_sum < lhs_i.selected_sum; end
          4'd4: begin key_different = lhs_i.compute_bound != rhs_i.compute_bound;
                      key_rhs_better = rhs_i.compute_bound < lhs_i.compute_bound; end
          4'd5: begin key_different = lhs_i.dma_bound != rhs_i.dma_bound;
                      key_rhs_better = rhs_i.dma_bound < lhs_i.dma_bound; end
          4'd6: begin key_different = lhs_i.late_end != rhs_i.late_end;
                      key_rhs_better = rhs_i.late_end < lhs_i.late_end; end
          4'd7: begin key_different = lhs_i.s2pf_count != rhs_i.s2pf_count;
                      key_rhs_better = rhs_i.s2pf_count > lhs_i.s2pf_count; end
          default: begin
          end
        endcase
      end

      default: begin
      end
    endcase
  end

  assign need_progress = regime_i.low_work_progress || regime_i.mid_plateau ||
                         regime_i.short_tail_plateau ||
                         regime_i.large_slack_fill;
  assign need_hotspot = regime_i.sparse_hot_sync;
  assign base_record = base_rhs_q ? rhs_i : lhs_i;
  assign progress_record = progress_rhs_q ? rhs_i : lhs_i;
  assign hotspot_record = hotspot_rhs_q ? rhs_i : lhs_i;
  assign cold_rank_floor = before_count_i - NR_W'(2);

  always_comb begin
    st_d = st_q;
    field_d = field_q;
    base_rhs_d = base_rhs_q;
    progress_rhs_d = progress_rhs_q;
    hotspot_rhs_d = hotspot_rhs_q;
    rhs_wins_d = rhs_wins_q;
    override_d = override_q;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          field_d = '0;
          base_rhs_d = 1'b0;
          progress_rhs_d = 1'b0;
          hotspot_rhs_d = 1'b0;
          override_d = 1'b0;
          st_d = ST_BASE;
        end
      end

      ST_BASE: begin
        if (key_different ||
            ((mode_i == DIST_MODE_SYNC) && (field_q == 4'd7)) ||
            ((mode_i != DIST_MODE_SYNC) && (field_q == 4'd8))) begin
          base_rhs_d = key_different && key_rhs_better;
          field_d = '0;
          if (need_progress)
            st_d = ST_PROGRESS;
          else if (need_hotspot)
            st_d = ST_HOTSPOT;
          else
            st_d = ST_OVERRIDE;
        end else begin
          field_d = field_q + 1'b1;
        end
      end

      ST_PROGRESS: begin
        if (key_different || (field_q == 4'd8)) begin
          progress_rhs_d = key_different && key_rhs_better;
          field_d = '0;
          st_d = need_hotspot ? ST_HOTSPOT : ST_OVERRIDE;
        end else begin
          field_d = field_q + 1'b1;
        end
      end

      ST_HOTSPOT: begin
        if (key_different || (field_q == 4'd7)) begin
          hotspot_rhs_d = key_different && key_rhs_better;
          field_d = '0;
          st_d = ST_OVERRIDE;
        end else begin
          field_d = field_q + 1'b1;
        end
      end

      ST_OVERRIDE: begin
        logic selected_rhs;
        logic use_override;
        logic common_fill;
        selected_rhs = base_rhs_q;
        use_override = 1'b0;
        common_fill = 1'b0;

        if (children_differ_i && regime_i.low_work_progress &&
            (progress_rhs_q != base_rhs_q) &&
            (base_record.s2pf_count == 2'd0) &&
            (progress_record.selected_sum > base_record.selected_sum) &&
            (progress_record.early_end <= base_record.early_end + time_t'(1))) begin
          selected_rhs = progress_rhs_q;
          use_override = 1'b1;
        end

        if (children_differ_i && regime_i.mid_plateau &&
            (progress_rhs_q != base_rhs_q) &&
            (base_record.s2pf_count == 2'd0) &&
            (progress_record.s2pf_count != 2'd0) &&
            (progress_record.selected_min_rank <= NR_W'(3)) &&
            (base_record.selected_max_rank >= cold_rank_floor) &&
            (progress_record.early_end == base_record.early_end) &&
            (progress_record.late_end <= base_record.late_end + time_t'(6))) begin
          selected_rhs = progress_rhs_q;
          use_override = 1'b1;
        end

        common_fill = children_differ_i &&
            (progress_rhs_q != base_rhs_q) &&
            (progress_record.s2pf_count != 2'd0) &&
            (progress_record.selected_sum > base_record.selected_sum) &&
            (progress_record.early_end >= base_record.early_end) &&
            (progress_record.f == base_record.f) &&
            (progress_record.h == base_record.h) &&
            (progress_record.dma_bound == base_record.dma_bound);
        if (regime_i.short_tail_plateau && common_fill &&
            (base_record.s2pf_count != 2'd0) &&
            progress_record.selects_t0 && !base_record.selects_t0 &&
            (progress_record.late_end <= base_record.late_end + time_t'(3))) begin
          selected_rhs = progress_rhs_q;
          use_override = 1'b1;
        end
        if (regime_i.large_slack_fill && common_fill &&
            (base_record.s2pf_count == 2'd0) &&
            (progress_record.selected_min_rank <= NR_W'(1)) &&
            (base_record.selected_max_rank >= cold_rank_floor) &&
            (progress_record.late_end == base_record.late_end)) begin
          selected_rhs = progress_rhs_q;
          use_override = 1'b1;
        end

        if (children_differ_i && regime_i.sparse_hot_sync &&
            (hotspot_rhs_q != base_rhs_q) &&
            hotspot_record.selects_t0 && !base_record.selects_t0 &&
            (hotspot_record.selected_max > base_record.selected_max) &&
            (hotspot_record.f == base_record.f) &&
            (hotspot_record.h == base_record.h) &&
            (hotspot_record.compute_bound <=
             base_record.compute_bound + distilled_bound_t'(6)) &&
            (hotspot_record.dma_bound <=
             base_record.dma_bound + distilled_bound_t'(12)) &&
            ((base_record.dma_bound > distilled_bound_t'(230)) ||
             ((min_remaining_load_i >= ntok_t'(2)) &&
              (base_record.dma_bound > distilled_bound_t'(204))))) begin
          selected_rhs = hotspot_rhs_q;
          use_override = 1'b1;
        end

        rhs_wins_d = selected_rhs;
        override_d = use_override;
        st_d = ST_DONE;
      end

      ST_DONE: begin
        if (!start_i)
          st_d = ST_IDLE;
      end

      default: st_d = ST_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q <= ST_IDLE;
      field_q <= '0;
      base_rhs_q <= 1'b0;
      progress_rhs_q <= 1'b0;
      hotspot_rhs_q <= 1'b0;
      rhs_wins_q <= 1'b0;
      override_q <= 1'b0;
    end else if (clear_i) begin
      st_q <= ST_IDLE;
      field_q <= '0;
      base_rhs_q <= 1'b0;
      progress_rhs_q <= 1'b0;
      hotspot_rhs_q <= 1'b0;
      rhs_wins_q <= 1'b0;
      override_q <= 1'b0;
    end else begin
      st_q <= st_d;
      field_q <= field_d;
      base_rhs_q <= base_rhs_d;
      progress_rhs_q <= progress_rhs_d;
      hotspot_rhs_q <= hotspot_rhs_d;
      rhs_wins_q <= rhs_wins_d;
      override_q <= override_d;
    end
  end

  assign done_o = (st_q == ST_DONE);
  assign rhs_wins_o = rhs_wins_q;
  assign override_o = override_q;

endmodule
