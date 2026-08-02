// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51

import sched_pkg::*;
import sched_distilled_pkg::*;

module sched_distilled_target_s4pf (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  logic                         clear_i,
  input  logic                         start_i,
  output logic                         done_o,
  output logic                         candidate_possible_o,
  input  logic [NR_W-1:0]              remaining_count_i,
  input  wire distilled_s4pf_consumer_t consumer_i,
  input  wire distilled_s4pf_cluster_t  base_c2_i,
  input  wire distilled_s4pf_cluster_t  base_c3_i,
  output logic                           bw_start_o,
  output snap_bw_view_t                  bw_c2_o,
  output snap_bw_view_t                  bw_c3_o,
  input  logic                           bw_done_i,
  input  logic                           bw_ok_i,
  output dma_binding_t                  c2_binding_o,
  output dma_binding_t                  c3_binding_o
);

  typedef enum logic [3:0] {
    ST_IDLE,
    ST_C2_LOCAL_START,
    ST_C2_LOCAL_WAIT,
    ST_C2_BOTH_START,
    ST_C2_BOTH_WAIT,
    ST_C3_LOCAL_START,
    ST_C3_LOCAL_WAIT,
    ST_C3_BOTH_START,
    ST_C3_BOTH_WAIT,
    ST_DONE
  } state_t;

  state_t st_q, st_d;
  dma_binding_t c2_binding_q, c2_binding_d;
  dma_binding_t c3_binding_q, c3_binding_d;
  logic consumer_c2_valid;
  logic consumer_c3_valid;
  logic consumer_c2_s1_cached;
  logic consumer_c3_s1_cached;
  logic consumer_c2_s2pf;
  logic consumer_c3_s2pf;
  logic c2_eligible;
  logic c3_eligible;
  logic c2_local_possible;
  logic c2_both_possible;
  logic c3_local_possible;
  logic c3_both_possible;
  dma_binding_t trial_c2_binding;
  dma_binding_t trial_c3_binding;

  always_comb begin
    consumer_c2_valid = 1'b0;
    consumer_c3_valid = 1'b0;
    consumer_c2_s1_cached = 1'b0;
    consumer_c3_s1_cached = 1'b0;
    consumer_c2_s2pf = 1'b0;
    consumer_c3_s2pf = 1'b0;
    for (int task_index = 0; task_index < 2; task_index++) begin
      if (consumer_i.valid[task_index]) begin
        if (consumer_i.cluster[task_index]) begin
          consumer_c3_valid = 1'b1;
          consumer_c3_s1_cached = consumer_i.skip_s1[task_index];
          consumer_c3_s2pf = consumer_i.has_s2pf[task_index];
        end else begin
          consumer_c2_valid = 1'b1;
          consumer_c2_s1_cached = consumer_i.skip_s1[task_index];
          consumer_c2_s2pf = consumer_i.has_s2pf[task_index];
        end
      end
    end
    c2_eligible = (remaining_count_i >= NR_W'(DISTILLED_S4PF_MIN_REMAINING)) &&
                  consumer_c2_valid && !consumer_c2_s1_cached &&
                  !consumer_c2_s2pf &&
                  base_c2_i.cur_valid && !base_c2_i.cache_valid &&
                  (!base_c3_i.cur_valid ||
                   (base_c2_i.dma3_end >= base_c3_i.task_start));
    c3_eligible = (remaining_count_i >= NR_W'(DISTILLED_S4PF_MIN_REMAINING)) &&
                  consumer_c3_valid && !consumer_c3_s1_cached &&
                  !consumer_c3_s2pf &&
                  base_c3_i.cur_valid && !base_c3_i.cache_valid &&
                  (!base_c2_i.cur_valid ||
                   (base_c3_i.dma3_end >= base_c2_i.task_start));
    c2_local_possible = c2_eligible &&
        ((base_c2_i.dma3_end + s4pf_dma_ticks(DMA_IDMA)) <=
         base_c2_i.task_end);
    c2_both_possible = c2_eligible &&
        ((base_c2_i.dma3_end + s4pf_dma_ticks(DMA_BOTH)) <=
         base_c2_i.task_end);
    c3_local_possible = c3_eligible &&
        ((base_c3_i.dma3_end + s4pf_dma_ticks(DMA_XDMA)) <=
         base_c3_i.task_end);
    c3_both_possible = c3_eligible &&
        ((base_c3_i.dma3_end + s4pf_dma_ticks(DMA_BOTH)) <=
         base_c3_i.task_end);
  end

  assign candidate_possible_o = c2_local_possible || c2_both_possible ||
                                c3_local_possible || c3_both_possible;

  always_comb begin
    trial_c2_binding = c2_binding_q;
    trial_c3_binding = c3_binding_q;
    unique case (st_q)
      ST_C2_LOCAL_START, ST_C2_LOCAL_WAIT: trial_c2_binding = DMA_IDMA;
      ST_C2_BOTH_START, ST_C2_BOTH_WAIT:   trial_c2_binding = DMA_BOTH;
      ST_C3_LOCAL_START, ST_C3_LOCAL_WAIT: trial_c3_binding = DMA_XDMA;
      ST_C3_BOTH_START, ST_C3_BOTH_WAIT:   trial_c3_binding = DMA_BOTH;
      default: begin
      end
    endcase
    bw_c2_o = distilled_s4pf_bw_view(base_c2_i);
    bw_c3_o = distilled_s4pf_bw_view(base_c3_i);
    bw_c2_o.s4pf_valid = (trial_c2_binding != DMA_NONE);
    bw_c2_o.s4pf_dma = trial_c2_binding;
    bw_c3_o.s4pf_valid = (trial_c3_binding != DMA_NONE);
    bw_c3_o.s4pf_dma = trial_c3_binding;
  end

  always_comb begin
    st_d = st_q;
    c2_binding_d = c2_binding_q;
    c3_binding_d = c3_binding_q;
    bw_start_o = 1'b0;
    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          c2_binding_d = DMA_NONE;
          c3_binding_d = DMA_NONE;
          if (c2_local_possible)
            st_d = ST_C2_LOCAL_START;
          else if (c2_both_possible)
            st_d = ST_C2_BOTH_START;
          else if (c3_local_possible)
            st_d = ST_C3_LOCAL_START;
          else if (c3_both_possible)
            st_d = ST_C3_BOTH_START;
          else
            st_d = ST_DONE;
        end
      end

      ST_C2_LOCAL_START: begin bw_start_o = 1'b1; st_d = ST_C2_LOCAL_WAIT; end
      ST_C2_LOCAL_WAIT: begin
        if (bw_done_i) begin
          if (bw_ok_i) begin
            c2_binding_d = DMA_IDMA;
            if (c3_local_possible)
              st_d = ST_C3_LOCAL_START;
            else if (c3_both_possible)
              st_d = ST_C3_BOTH_START;
            else
              st_d = ST_DONE;
          end else if (c2_both_possible) begin
            st_d = ST_C2_BOTH_START;
          end else if (c3_local_possible) begin
            st_d = ST_C3_LOCAL_START;
          end else if (c3_both_possible) begin
            st_d = ST_C3_BOTH_START;
          end else begin
            st_d = ST_DONE;
          end
        end
      end

      ST_C2_BOTH_START: begin bw_start_o = 1'b1; st_d = ST_C2_BOTH_WAIT; end
      ST_C2_BOTH_WAIT: begin
        if (bw_done_i) begin
          if (bw_ok_i)
            c2_binding_d = DMA_BOTH;
          if (c3_local_possible)
            st_d = ST_C3_LOCAL_START;
          else if (c3_both_possible)
            st_d = ST_C3_BOTH_START;
          else
            st_d = ST_DONE;
        end
      end

      ST_C3_LOCAL_START: begin bw_start_o = 1'b1; st_d = ST_C3_LOCAL_WAIT; end
      ST_C3_LOCAL_WAIT: begin
        if (bw_done_i) begin
          if (bw_ok_i) begin
            c3_binding_d = DMA_XDMA;
            st_d = ST_DONE;
          end else if (c3_both_possible) begin
            st_d = ST_C3_BOTH_START;
          end else begin
            st_d = ST_DONE;
          end
        end
      end

      ST_C3_BOTH_START: begin bw_start_o = 1'b1; st_d = ST_C3_BOTH_WAIT; end
      ST_C3_BOTH_WAIT: begin
        if (bw_done_i) begin
          if (bw_ok_i)
            c3_binding_d = DMA_BOTH;
          st_d = ST_DONE;
        end
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
      c2_binding_q <= DMA_NONE;
      c3_binding_q <= DMA_NONE;
    end else if (clear_i) begin
      st_q <= ST_IDLE;
      c2_binding_q <= DMA_NONE;
      c3_binding_q <= DMA_NONE;
    end else begin
      st_q <= st_d;
      c2_binding_q <= c2_binding_d;
      c3_binding_q <= c3_binding_d;
    end
  end

  assign done_o = (st_q == ST_DONE);
  assign c2_binding_o = c2_binding_q;
  assign c3_binding_o = c3_binding_q;

endmodule
