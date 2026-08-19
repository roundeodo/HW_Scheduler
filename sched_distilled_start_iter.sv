// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51

import sched_pkg::*;
import sched_distilled_pkg::*;

module sched_distilled_start_iter (
  input  logic                       clk_i,
  input  logic                       rst_ni,
  input  logic                       clear_i,
  input  logic                       begin_i,
  input  logic                       target_i,
  input  logic                       result_valid_i,
  input  logic                       result_feasible_i,
  input  logic                       result_strict_gain_i,
  input  wire distilled_profile_t    profile_i,
  input  logic [EID_RAW_W-1:0]       eid_i,
  input  ntok_t                      ntok_i,
  input  logic                       force_s1_hit_i,
  input  wire distilled_cluster_state_t own_i,
  input  wire distilled_cluster_state_t peer_i,
  output logic                       valid_o,
  output logic                       done_o,
  output time_t                      start_o
);

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_DIRECT,
    ST_SCAN,
    ST_EMIT,
    ST_DONE
  } state_t;

  state_t st_q, st_d;
  logic [4:0] source_q, source_d;
  time_t previous_q, previous_d;
  time_t minimum_q, minimum_d;
  logic minimum_valid_q, minimum_valid_d;

  logic target_q;
  time_t decision_floor_q;
  time_t peer_task_end_q;
  logic [2:0] peer_release_valid_q;
  time_t peer_release_value_q [0:2];
  time_t dma1_offset_q;
  time_t s2_offset_q;
  logic s3_cached_q;
  logic offset0_valid_q;
  logic profile_s2pf_valid_q;
  dma_binding_t final_dma_s3_q;

  logic active_c3;
  shape_t shape_s1;
  dma_binding_t dma_s1;
  dma_binding_t final_dma_s3;
  dma_binding_t profile_s2pf;
  logic s1_cached;
  logic s3_cached;
  time_t decision_floor;
  time_t s1_compute;
  time_t dma1_offset;
  time_t s2_work;
  time_t s2_offset;
  logic [1:0] release_index;
  logic [2:0] offset_index;
  logic release_valid;
  time_t release_value;
  logic offset_valid;
  time_t offset_value;
  logic source_valid;
  time_t source_value;
  logic [2:0] peer_release_valid_input;
  logic scan_finish;
  logic [4:0] scan_next_source;

  assign active_c3 = profile_i.c3_active && !profile_i.c2_active;
  assign shape_s1 = active_c3 ? profile_i.c3_shape_s1 : profile_i.c2_shape_s1;
  assign dma_s1 = active_c3 ? profile_i.c3_dma_s1 : profile_i.c2_dma_s1;
  assign final_dma_s3 = active_c3 ? profile_i.c3_dma_s3 : profile_i.c2_dma_s3;
  assign profile_s2pf = active_c3 ? profile_i.c3_s2pf : profile_i.c2_s2pf;
  assign s1_cached = force_s1_hit_i ||
                     distilled_cache_s1_hit(own_i, eid_i);
  assign s3_cached = distilled_cache_s3_hit(own_i, eid_i);

  always_comb begin
    unique case (shape_s1)
      SHAPE_A: s1_compute = time_t'(8);
      SHAPE_B: s1_compute = time_t'(4);
      default: s1_compute = time_t'(2);
    endcase
    dma1_offset = s1_cached ? '0 : s1_dma_ticks(dma_s1);
    s2_work = time_t'({ceil_div2_ntok(ntok_i), 1'b0});
    s2_offset = s1_cached || (s2_work >= s1_compute) ?
                s2_work : s1_compute;
    decision_floor = own_i.task_end;
    if (peer_i.cur_valid && (peer_i.task_start > decision_floor))
      decision_floor = peer_i.task_start;
  end

  assign peer_release_valid_input[0] = peer_i.cur_valid &&
                                       (peer_i.dma_s1 != DMA_NONE);
  assign peer_release_valid_input[1] = peer_i.cur_valid &&
                                       (peer_i.s2pf_dma != DMA_NONE);
  assign peer_release_valid_input[2] = peer_i.cur_valid &&
                                       (peer_i.dma_s3 != DMA_NONE);

  always_comb begin
    scan_finish = 1'b0;
    scan_next_source = source_q + 1'b1;
    unique case (source_q)
      5'd0: begin
        if (peer_release_valid_q[0])
          scan_next_source = s3_cached_q ? 5'd3 : 5'd1;
        else if (peer_release_valid_q[1])
          scan_next_source = s3_cached_q ? 5'd10 : 5'd8;
        else if (peer_release_valid_q[2])
          scan_next_source = s3_cached_q ? 5'd17 : 5'd15;
        else
          scan_finish = 1'b1;
      end
      5'd3: begin
        if (s3_cached_q) begin
          if (peer_release_valid_q[1])
            scan_next_source = 5'd10;
          else if (peer_release_valid_q[2])
            scan_next_source = 5'd17;
          else
            scan_finish = 1'b1;
        end
      end
      5'd7: begin
        if (peer_release_valid_q[1])
          scan_next_source = 5'd8;
        else if (peer_release_valid_q[2])
          scan_next_source = 5'd15;
        else
          scan_finish = 1'b1;
      end
      5'd10: begin
        if (s3_cached_q) begin
          if (peer_release_valid_q[2])
            scan_next_source = 5'd17;
          else
            scan_finish = 1'b1;
        end
      end
      5'd14: begin
        if (peer_release_valid_q[2])
          scan_next_source = 5'd15;
        else
          scan_finish = 1'b1;
      end
      5'd17: begin
        if (s3_cached_q)
          scan_finish = 1'b1;
      end
      5'd21: scan_finish = 1'b1;
      default: begin
      end
    endcase
  end

  always_comb begin
    release_index = '0;
    offset_index = '0;
    if (source_q >= 5'd15) begin
      release_index = 2'd2;
      offset_index = source_q - 5'd15;
    end else if (source_q >= 5'd8) begin
      release_index = 2'd1;
      offset_index = source_q - 5'd8;
    end else if (source_q >= 5'd1) begin
      release_index = 2'd0;
      offset_index = source_q - 5'd1;
    end
    release_valid = 1'b0;
    release_value = '0;
    unique case (release_index)
      2'd0: begin
        release_valid = peer_release_valid_q[0];
        release_value = peer_release_value_q[0];
      end
      2'd1: begin
        release_valid = peer_release_valid_q[1];
        release_value = peer_release_value_q[1];
      end
      2'd2: begin
        release_valid = peer_release_valid_q[2];
        release_value = peer_release_value_q[2];
      end
      default: begin
      end
    endcase

    offset_valid = 1'b1;
    offset_value = '0;
    unique case (offset_index)
      3'd0: begin
        offset_valid = offset0_valid_q;
        offset_value = '0;
      end
      3'd1: begin
        offset_valid = !s3_cached_q;
        offset_value = dma1_offset_q;
      end
      3'd2: begin
        offset_valid = s3_cached_q ||
                       (!profile_s2pf_valid_q &&
                        (final_dma_s3_q != DMA_NONE));
        offset_value = s2_offset_q;
      end
      3'd3: begin
        offset_valid = !s3_cached_q &&
                       (profile_s2pf_valid_q ||
                        (final_dma_s3_q == DMA_IDMA) ||
                        (final_dma_s3_q == DMA_XDMA));
        offset_value = s2_offset_q + time_t'(1);
      end
      3'd4: begin
        offset_valid = !s3_cached_q &&
                       (profile_s2pf_valid_q ||
                        (final_dma_s3_q == DMA_BOTH));
        offset_value = s2_offset_q + time_t'(2);
      end
      3'd5: begin
        offset_valid = !s3_cached_q;
        offset_value = (s2_offset_q > (dma1_offset_q + time_t'(1))) ?
                       s2_offset_q : (dma1_offset_q + time_t'(1));
      end
      3'd6: begin
        offset_valid = !s3_cached_q;
        offset_value = (s2_offset_q > (dma1_offset_q + time_t'(2))) ?
                       s2_offset_q : (dma1_offset_q + time_t'(2));
      end
      default: begin
      end
    endcase

    source_valid = 1'b0;
    source_value = '0;
    if (source_q == 5'd0) begin
      source_valid = peer_task_end_q > previous_q;
      source_value = peer_task_end_q;
    end else if (release_valid && offset_valid &&
                 (release_value >= offset_value)) begin
      source_value = release_value - offset_value;
      source_valid = (source_value > previous_q) &&
                     (source_value >= decision_floor_q) &&
                     (source_value <= peer_task_end_q);
    end
  end

  always_comb begin
    st_d = st_q;
    source_d = source_q;
    previous_d = previous_q;
    minimum_d = minimum_q;
    minimum_valid_d = minimum_valid_q;
    unique case (st_q)
      ST_IDLE: begin
        if (begin_i) begin
          previous_d = decision_floor;
          st_d = ST_DIRECT;
        end
      end
      ST_DIRECT: begin
        if (result_valid_i) begin
          if (result_feasible_i ||
              (target_q && !result_strict_gain_i)) begin
            st_d = ST_DONE;
          end else begin
            source_d = '0;
            minimum_valid_d = 1'b0;
            st_d = ST_SCAN;
          end
        end
      end
      ST_SCAN: begin
        if (source_valid &&
            (!minimum_valid_q || (source_value < minimum_q))) begin
          minimum_d = source_value;
          minimum_valid_d = 1'b1;
        end
        if (scan_finish) begin
          if (minimum_valid_d) begin
            previous_d = minimum_d;
            st_d = ST_EMIT;
          end else begin
            st_d = ST_DONE;
          end
        end else begin
          source_d = scan_next_source;
        end
      end
      ST_EMIT: begin
        if (result_valid_i) begin
          if (result_feasible_i ||
              (target_q && !result_strict_gain_i)) begin
            st_d = ST_DONE;
          end else begin
            source_d = '0;
            minimum_valid_d = 1'b0;
            st_d = ST_SCAN;
          end
        end
      end
      ST_DONE: begin
        if (!begin_i)
          st_d = ST_IDLE;
      end
      default: st_d = ST_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q <= ST_IDLE;
      source_q <= '0;
      previous_q <= '0;
      minimum_q <= '0;
      minimum_valid_q <= 1'b0;
      target_q <= 1'b0;
      decision_floor_q <= '0;
      peer_task_end_q <= '0;
      peer_release_valid_q <= '0;
      peer_release_value_q <= '{default: '0};
      dma1_offset_q <= '0;
      s2_offset_q <= '0;
      s3_cached_q <= 1'b0;
      offset0_valid_q <= 1'b0;
      profile_s2pf_valid_q <= 1'b0;
      final_dma_s3_q <= DMA_NONE;
    end else if (clear_i) begin
      st_q <= ST_IDLE;
      source_q <= '0;
      previous_q <= '0;
      minimum_q <= '0;
      minimum_valid_q <= 1'b0;
      target_q <= 1'b0;
      decision_floor_q <= '0;
      peer_task_end_q <= '0;
      peer_release_valid_q <= '0;
      peer_release_value_q <= '{default: '0};
      dma1_offset_q <= '0;
      s2_offset_q <= '0;
      s3_cached_q <= 1'b0;
      offset0_valid_q <= 1'b0;
      profile_s2pf_valid_q <= 1'b0;
      final_dma_s3_q <= DMA_NONE;
    end else begin
      st_q <= st_d;
      source_q <= source_d;
      previous_q <= previous_d;
      minimum_q <= minimum_d;
      minimum_valid_q <= minimum_valid_d;
      if ((st_q == ST_IDLE) && begin_i) begin
        target_q <= target_i;
        decision_floor_q <= decision_floor;
        peer_task_end_q <= peer_i.task_end;
        peer_release_valid_q <= peer_release_valid_input;
        peer_release_value_q[0] <= peer_i.dma1_end;
        peer_release_value_q[1] <=
            peer_i.dma1_end + s3_dma_ticks(peer_i.s2pf_dma);
        peer_release_value_q[2] <= peer_i.dma3_end;
        dma1_offset_q <= dma1_offset;
        s2_offset_q <= s2_offset;
        s3_cached_q <= s3_cached;
        offset0_valid_q <= !s1_cached && (dma_s1 != DMA_NONE);
        profile_s2pf_valid_q <= profile_s2pf != DMA_NONE;
        final_dma_s3_q <= final_dma_s3;
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni && (st_q == ST_SCAN) && s3_cached_q) begin
      assert ((source_q == 5'd0) || (source_q == 5'd3) ||
              (source_q == 5'd10) || (source_q == 5'd17));
    end
  end
`endif

  assign valid_o = (st_q == ST_DIRECT) || (st_q == ST_EMIT);
  assign done_o = (st_q == ST_DONE);
  assign start_o = (st_q == ST_DIRECT) ? decision_floor_q : previous_q;

endmodule
