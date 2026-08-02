// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// Shared four-stage endpoint datapath for the distilled scheduler.  Compute
// shape and physical DMA binding are independent inputs.

import sched_pkg::*;

module sched_distilled_timeline (
  input  time_t         start_i,
  input  ntok_t         ntok_i,
  input  shape_t        shape_s1_i,
  input  shape_t        shape_s3_i,
  input  logic          s1_cached_i,
  input  logic          s3_cached_i,
  input  dma_binding_t  dma_s1_i,
  input  dma_binding_t  dma_s3_i,
  input  dma_binding_t  s2pf_dma_i,
  output snap_timeline_t timeline_o,
  output time_t         compute_end_o
);

  ntok_t half_tokens;
  time_t s1_min_ticks;
  time_t s3_min_ticks;
  time_t dma1_ticks;
  time_t dma3_ticks;
  time_t s2_work_ticks;
  time_t s2_duration;
  time_t s4_duration;
  time_t s2pf_end;
  time_t s4_ready;

  always_comb begin
    unique case (shape_s1_i)
      SHAPE_A: begin
        s1_min_ticks = time_t'(8);
      end
      SHAPE_B: begin
        s1_min_ticks = time_t'(4);
      end
      SHAPE_C: begin
        s1_min_ticks = time_t'(2);
      end
      default: begin
        s1_min_ticks = 'x;
      end
    endcase
    unique case (shape_s3_i)
      SHAPE_A: begin
        s3_min_ticks = time_t'(4);
      end
      SHAPE_B: begin
        s3_min_ticks = time_t'(2);
      end
      SHAPE_C: begin
        s3_min_ticks = time_t'(1);
      end
      default: begin
        s3_min_ticks = 'x;
      end
    endcase
  end

  assign half_tokens = ceil_div2_ntok(ntok_i);
  assign s2_work_ticks = time_t'({half_tokens, 1'b0});
  assign dma1_ticks = s1_cached_i ? '0 : s1_dma_ticks(dma_s1_i);
  assign dma3_ticks = s3_cached_i || (s2pf_dma_i != DMA_NONE) ?
                      '0 : s3_dma_ticks(dma_s3_i);
  assign s2_duration = s1_cached_i || (s2_work_ticks >= s1_min_ticks) ?
                       s2_work_ticks : s1_min_ticks;
  assign s4_duration = s3_cached_i || (s2pf_dma_i != DMA_NONE) ||
                       (time_t'(half_tokens) >= s3_min_ticks) ?
                       time_t'(half_tokens) : s3_min_ticks;

  always_comb begin
    timeline_o = '0;
    timeline_o.valid      = 1'b1;
    timeline_o.task_start = start_i;
    timeline_o.dma_s1     = s1_cached_i ? DMA_NONE : dma_s1_i;
    timeline_o.dma_s3     = s3_cached_i || (s2pf_dma_i != DMA_NONE) ?
                            DMA_NONE : dma_s3_i;
    timeline_o.dma1_end   = start_i + dma1_ticks;
    timeline_o.s2_end     = start_i + s2_duration;

    timeline_o.s2pf_valid = (s2pf_dma_i != DMA_NONE) && !s3_cached_i;
    timeline_o.s2pf_start = timeline_o.s2pf_valid ? timeline_o.dma1_end : '0;
    timeline_o.s2pf_dma   = timeline_o.s2pf_valid ? s2pf_dma_i : DMA_NONE;
    s2pf_end = timeline_o.dma1_end + s3_dma_ticks(s2pf_dma_i);
    timeline_o.s2pf_end = timeline_o.s2pf_valid ? s2pf_end : '0;

    s4_ready = timeline_o.s2_end;
    if (timeline_o.s2pf_valid && (s2pf_end > s4_ready))
      s4_ready = s2pf_end;
    timeline_o.dma3_end = timeline_o.s2pf_valid ? s4_ready :
        (timeline_o.s2_end + dma3_ticks);
    compute_end_o = timeline_o.s2pf_valid ?
                    (s4_ready + time_t'(half_tokens)) :
                    (timeline_o.s2_end + s4_duration);
    timeline_o.task_end = compute_end_o;
  end

endmodule
