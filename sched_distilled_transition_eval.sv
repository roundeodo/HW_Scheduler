// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51

import sched_pkg::*;
import sched_distilled_pkg::*;

module sched_distilled_transition_eval (
  input  logic                       clk_i,
  input  logic                       rst_ni,
  input  logic                       clear_i,
  input  logic                       start_i,
  output logic                       done_o,
  output logic                       feasible_o,

  input  wire distilled_profile_t    profile_i,
  input  logic                       assignment_swap_i,
  input  time_t                      start_time_i,
  input  logic                       incremental_i,
  input  logic                       rebuild_only_i,
  input  logic                       gain_ok_i,
  input  logic                       force_s1_hit_c2_i,
  input  logic                       force_s1_hit_c3_i,
  input  wire head_ctx_t             selected_a_i,
  input  wire head_ctx_t             selected_b_i,
  input  wire distilled_cluster_state_t base_c2_i,
  input  wire distilled_cluster_state_t base_c3_i,

  output logic                       bw_start_o,
  output snap_bw_view_t              bw_c2_o,
  output snap_bw_view_t              bw_c3_o,
  input  logic                       bw_done_i,
  input  logic                       bw_ok_i,

  output distilled_cluster_state_t   child_c2_o,
  output distilled_cluster_state_t   child_c3_o,
  output winner_plan_t               plan_o,
  output logic [1:0]                 remove_count_o,
  output logic [EID_RAW_W-1:0]       remove_eid_a_o,
  output logic [EID_RAW_W-1:0]       remove_eid_b_o,
  output ntok_t                      selected_max_o,
  output ntok_t                      selected_sum_o,
  output logic [1:0]                 s2pf_count_o,
  output time_t                      latest_start_o
);

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_BUILD_C2,
    ST_BUILD_C3,
    ST_GAIN_CHECK,
    ST_BW_START,
    ST_BW_WAIT,
    ST_DONE
  } state_t;

  typedef struct packed {
    logic                    valid;
    logic [EID_RAW_W-1:0]    eid;
    ntok_t                   ntok;
    tok_start_t              tok_start;
    shape_t                  shape_s1;
    shape_t                  shape_s3;
    logic                    s1_cached;
    logic                    s3_cached;
    dma_binding_t            dma_s1;
    dma_binding_t            dma_s3;
    dma_binding_t            s2pf_dma;
  } side_request_t;

  typedef struct packed {
    time_t task_end;
    time_t dma1_end;
    time_t s2_end;
    time_t dma3_end;
  } endpoint_t;

  state_t st_q, st_d;
  // Volatile per-evaluation request; parent cluster state is round-stable.
  side_request_t c2_req_in, c3_req_in;
  side_request_t c2_req_q, c3_req_q;
  logic decode_valid;
  head_ctx_t selected_a, selected_b;
  time_t start_time_q;
  logic incremental_q;
  logic rebuild_only_q;
  logic assignment_swap_q;
  logic rebuild_c2_in, rebuild_c3_in;
  logic rebuild_c3_q;

  snap_timeline_t built_timeline;
  endpoint_t built_endpoint;
  endpoint_t c2_endpoint_q, c2_endpoint_d;
  endpoint_t c3_endpoint_q, c3_endpoint_d;
  logic feasible_q, feasible_d;
  logic timeline_side_c3;
  side_request_t timeline_req;
  function automatic distilled_cluster_state_t task_child(
    input side_request_t   request,
    input endpoint_t       endpoint,
    input time_t           start_time
  );
    distilled_cluster_state_t child;
    begin
      child = '0;
      child.cur_valid   = request.valid;
      child.task_start  = start_time;
      child.task_end    = endpoint.task_end;
      child.dma1_end    = endpoint.dma1_end;
      child.s2_end      = endpoint.s2_end;
      child.dma3_end    = endpoint.dma3_end;
      child.dma_s1      = request.dma_s1;
      child.dma_s3      = request.dma_s3;
      child.s2pf_dma    = request.s2pf_dma;
      task_child = child;
    end
  endfunction

  function automatic snap_bw_view_t endpoint_bw_view(
    input side_request_t request,
    input endpoint_t     endpoint,
    input time_t         start_time
  );
    snap_bw_view_t view;
    begin
      view = '0;
      view.valid = request.valid;
      view.task_start = start_time;
      view.dma1_end = endpoint.dma1_end;
      view.s2_end = endpoint.s2_end;
      view.dma3_end = endpoint.dma3_end;
      view.dma_s1 = request.dma_s1;
      view.dma_s3 = request.dma_s3;
      view.s2pf_valid = (request.s2pf_dma != DMA_NONE);
      view.s2pf_start = endpoint.dma1_end;
      view.s2pf_end = endpoint.dma1_end + s3_dma_ticks(request.s2pf_dma);
      view.s2pf_dma = request.s2pf_dma;
      endpoint_bw_view = view;
    end
  endfunction

  always_comb begin
    selected_a = selected_a_i;
    selected_b = selected_b_i;
    c2_req_in = '0;
    c3_req_in = '0;
    decode_valid = 1'b1;

    unique case (profile_i.family)
      DIST_FAMILY_SINGLE: begin
        decode_valid &= selected_a.valid &&
                        (profile_i.c2_active ^ profile_i.c3_active);
        if (profile_i.c2_active) begin
          c2_req_in.valid = 1'b1;
          c2_req_in.eid = selected_a.eid;
          c2_req_in.ntok = selected_a.ntok;
        end else begin
          c3_req_in.valid = 1'b1;
          c3_req_in.eid = selected_a.eid;
          c3_req_in.ntok = selected_a.ntok;
        end
        if (base_c2_i.task_end < base_c3_i.task_end)
          decode_valid &= profile_i.c2_active;
        else if (base_c3_i.task_end < base_c2_i.task_end)
          decode_valid &= profile_i.c3_active;
        else
          decode_valid &= profile_i.c2_active ||
                          (profile_i.c3_active &&
                           (base_c2_i != base_c3_i));
      end

      DIST_FAMILY_PAIR: begin
        decode_valid &= selected_a.valid && selected_b.valid &&
                        (selected_a.eid != selected_b.eid) &&
                        profile_i.c2_active && profile_i.c3_active;
        c2_req_in.valid = 1'b1;
        c3_req_in.valid = 1'b1;
        c2_req_in.eid = assignment_swap_i ? selected_b.eid : selected_a.eid;
        c2_req_in.ntok = assignment_swap_i ? selected_b.ntok : selected_a.ntok;
        c3_req_in.eid = assignment_swap_i ? selected_a.eid : selected_b.eid;
        c3_req_in.ntok = assignment_swap_i ? selected_a.ntok : selected_b.ntok;
      end

      DIST_FAMILY_SPLIT: begin
        decode_valid &= selected_a.valid && profile_i.c2_active &&
                        profile_i.c3_active && (selected_a.ntok >= ntok_t'(2));
        if (!profile_i.split_balanced)
          decode_valid &= !selected_a.ntok[0];
        c2_req_in.valid = 1'b1;
        c3_req_in.valid = 1'b1;
        c2_req_in.eid = selected_a.eid;
        c3_req_in.eid = selected_a.eid;
        c2_req_in.ntok = {1'b0, selected_a.ntok[NTOK_W-1:1]};
        c3_req_in.ntok = selected_a.ntok - c2_req_in.ntok;
        c3_req_in.tok_start = tok_start_t'(c2_req_in.ntok);
      end

      default: decode_valid = 1'b0;
    endcase

    c2_req_in.shape_s1 = profile_i.c2_shape_s1;
    c2_req_in.shape_s3 = profile_i.c2_shape_s3;
    c3_req_in.shape_s1 = profile_i.c3_shape_s1;
    c3_req_in.shape_s3 = profile_i.c3_shape_s3;

    if (c2_req_in.valid) begin
      c2_req_in.s1_cached = force_s1_hit_c2_i ||
          distilled_cache_s1_hit(base_c2_i, c2_req_in.eid);
      c2_req_in.s3_cached = distilled_cache_s3_hit(base_c2_i, c2_req_in.eid);
      decode_valid &= (profile_i.c2_dma_s1 != DMA_NONE) || c2_req_in.s1_cached;
      decode_valid &= (profile_i.c2_s2pf != DMA_NONE) ||
                      (profile_i.c2_dma_s3 != DMA_NONE) || c2_req_in.s3_cached;
      c2_req_in.dma_s1 = c2_req_in.s1_cached ? DMA_NONE : profile_i.c2_dma_s1;
      c2_req_in.s2pf_dma = c2_req_in.s3_cached ? DMA_NONE : profile_i.c2_s2pf;
      c2_req_in.dma_s3 = c2_req_in.s3_cached ||
                         (c2_req_in.s2pf_dma != DMA_NONE) ?
                         DMA_NONE : profile_i.c2_dma_s3;
    end
    if (c3_req_in.valid) begin
      c3_req_in.s1_cached = force_s1_hit_c3_i ||
          distilled_cache_s1_hit(base_c3_i, c3_req_in.eid);
      c3_req_in.s3_cached = distilled_cache_s3_hit(base_c3_i, c3_req_in.eid);
      decode_valid &= (profile_i.c3_dma_s1 != DMA_NONE) || c3_req_in.s1_cached;
      decode_valid &= (profile_i.c3_s2pf != DMA_NONE) ||
                      (profile_i.c3_dma_s3 != DMA_NONE) || c3_req_in.s3_cached;
      c3_req_in.dma_s1 = c3_req_in.s1_cached ? DMA_NONE : profile_i.c3_dma_s1;
      c3_req_in.s2pf_dma = c3_req_in.s3_cached ? DMA_NONE : profile_i.c3_s2pf;
      c3_req_in.dma_s3 = c3_req_in.s3_cached ||
                         (c3_req_in.s2pf_dma != DMA_NONE) ?
                         DMA_NONE : profile_i.c3_dma_s3;
    end
  end

  assign rebuild_c2_in = c2_req_in.valid &&
                         (!incremental_i || force_s1_hit_c2_i);
  assign rebuild_c3_in = c3_req_in.valid &&
                         (!incremental_i || force_s1_hit_c3_i);

  assign timeline_side_c3 = (st_q == ST_BUILD_C3);
  assign timeline_req = timeline_side_c3 ? c3_req_q : c2_req_q;

  sched_distilled_timeline i_timeline (
    .start_i       (start_time_q),
    .ntok_i        (timeline_req.ntok),
    .shape_s1_i    (timeline_req.shape_s1),
    .shape_s3_i    (timeline_req.shape_s3),
    .s1_cached_i   (timeline_req.s1_cached),
    .s3_cached_i   (timeline_req.s3_cached),
    .dma_s1_i      (timeline_req.dma_s1),
    .dma_s3_i      (timeline_req.dma_s3),
    .s2pf_dma_i    (timeline_req.s2pf_dma),
    .timeline_o    (built_timeline),
    .compute_end_o ()
  );

  assign built_endpoint.task_end = built_timeline.task_end;
  assign built_endpoint.dma1_end = built_timeline.dma1_end;
  assign built_endpoint.s2_end = built_timeline.s2_end;
  assign built_endpoint.dma3_end = built_timeline.dma3_end;

  always_comb begin
    bw_c2_o = c2_req_q.valid ? endpoint_bw_view(c2_req_q, c2_endpoint_q,
                                                start_time_q) :
                             distilled_bw_view(base_c2_i);
    bw_c3_o = c3_req_q.valid ? endpoint_bw_view(c3_req_q, c3_endpoint_q,
                                                start_time_q) :
                             distilled_bw_view(base_c3_i);
  end

  always_comb begin
    st_d = st_q;
    c2_endpoint_d = c2_endpoint_q;
    c3_endpoint_d = c3_endpoint_q;
    feasible_d = feasible_q;
    bw_start_o = 1'b0;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          feasible_d = decode_valid;
          if (!decode_valid) begin
            st_d = ST_DONE;
          end else if (rebuild_c2_in) begin
            st_d = ST_BUILD_C2;
          end else if (rebuild_c3_in) begin
            st_d = ST_BUILD_C3;
          end else begin
            st_d = rebuild_only_i ? ST_DONE :
                   (incremental_i ? ST_GAIN_CHECK : ST_BW_START);
          end
        end
      end

      ST_BUILD_C2: begin
        c2_endpoint_d = built_endpoint;
        if (rebuild_c3_q) begin
          st_d = ST_BUILD_C3;
        end else begin
          st_d = rebuild_only_q ? ST_DONE :
                 (incremental_q ? ST_GAIN_CHECK : ST_BW_START);
        end
      end

      ST_BUILD_C3: begin
        c3_endpoint_d = built_endpoint;
        st_d = rebuild_only_q ? ST_DONE :
               (incremental_q ? ST_GAIN_CHECK : ST_BW_START);
      end

      ST_GAIN_CHECK: begin
        // The round engine reuses its existing max-end comparator here.  This
        // avoids a second wide comparator inside the transition evaluator.
        if (gain_ok_i) begin
          st_d = ST_BW_START;
        end else begin
          feasible_d = 1'b0;
          st_d = ST_DONE;
        end
      end

      ST_BW_START: begin
        bw_start_o = 1'b1;
        st_d = ST_BW_WAIT;
      end

      ST_BW_WAIT: begin
        if (bw_done_i) begin
          feasible_d = bw_ok_i;
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
      c2_req_q <= '0;
      c3_req_q <= '0;
      start_time_q <= '0;
      incremental_q <= 1'b0;
      rebuild_only_q <= 1'b0;
      assignment_swap_q <= 1'b0;
      rebuild_c3_q <= 1'b0;
      c2_endpoint_q <= '0;
      c3_endpoint_q <= '0;
      feasible_q <= 1'b0;
    end else if (clear_i) begin
      st_q <= ST_IDLE;
      c2_req_q <= '0;
      c3_req_q <= '0;
      start_time_q <= '0;
      incremental_q <= 1'b0;
      rebuild_only_q <= 1'b0;
      assignment_swap_q <= 1'b0;
      rebuild_c3_q <= 1'b0;
      c2_endpoint_q <= '0;
      c3_endpoint_q <= '0;
      feasible_q <= 1'b0;
    end else begin
      st_q <= st_d;
      if ((st_q == ST_IDLE) && start_i) begin
        c2_req_q <= c2_req_in;
        c3_req_q <= c3_req_in;
        start_time_q <= start_time_i;
        incremental_q <= incremental_i;
        rebuild_only_q <= rebuild_only_i;
        assignment_swap_q <= assignment_swap_i;
        rebuild_c3_q <= rebuild_c3_in;
      end
      c2_endpoint_q <= c2_endpoint_d;
      c3_endpoint_q <= c3_endpoint_d;
      feasible_q <= feasible_d;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni) begin
      if (start_i && incremental_i)
        assert ((c2_req_in.valid && force_s1_hit_c2_i) ||
                (c3_req_in.valid && force_s1_hit_c3_i));
      if (st_q == ST_GAIN_CHECK)
        assert (incremental_q);
      if ((st_q != ST_IDLE) && (st_q != ST_DONE)) begin
        assert ($stable(base_c2_i));
        assert ($stable(base_c3_i));
      end
    end
  end
`endif

  always_comb begin
    child_c2_o = c2_req_q.valid ?
        task_child(c2_req_q, c2_endpoint_q, start_time_q) : base_c2_i;
    child_c3_o = c3_req_q.valid ?
        task_child(c3_req_q, c3_endpoint_q, start_time_q) : base_c3_i;
    plan_o = '0;
    if (c2_req_q.valid) begin
      plan_o.task_valid[0] = 1'b1;
      plan_o.token[0].eid = c2_req_q.eid;
      plan_o.token[0].ntok = c2_req_q.ntok;
      plan_o.token[0].tok_start = c2_req_q.tok_start;
      plan_o.ctrl[0].cluster = 1'b0;
      plan_o.ctrl[0].shape_s1 = c2_req_q.shape_s1;
      plan_o.ctrl[0].shape_s3 = c2_req_q.shape_s3;
      plan_o.ctrl[0].skip_s1 = c2_req_q.s1_cached;
      plan_o.ctrl[0].skip_s3 = c2_req_q.s3_cached ||
                               (c2_req_q.s2pf_dma != DMA_NONE);
      plan_o.ctrl[0].has_s2pf = (c2_req_q.s2pf_dma != DMA_NONE);
      plan_o.ctrl[0].dma_s1_both = (c2_req_q.dma_s1 == DMA_BOTH);
      plan_o.ctrl[0].dma_late_both = (c2_req_q.s2pf_dma != DMA_NONE) ?
          (c2_req_q.s2pf_dma == DMA_BOTH) : (c2_req_q.dma_s3 == DMA_BOTH);
    end
    if (c3_req_q.valid) begin
      plan_o.task_valid[c2_req_q.valid] = 1'b1;
      plan_o.token[c2_req_q.valid].eid = c3_req_q.eid;
      plan_o.token[c2_req_q.valid].ntok = c3_req_q.ntok;
      plan_o.token[c2_req_q.valid].tok_start = c3_req_q.tok_start;
      plan_o.ctrl[c2_req_q.valid].cluster = 1'b1;
      plan_o.ctrl[c2_req_q.valid].shape_s1 = c3_req_q.shape_s1;
      plan_o.ctrl[c2_req_q.valid].shape_s3 = c3_req_q.shape_s3;
      plan_o.ctrl[c2_req_q.valid].skip_s1 = c3_req_q.s1_cached;
      plan_o.ctrl[c2_req_q.valid].skip_s3 = c3_req_q.s3_cached ||
                                            (c3_req_q.s2pf_dma != DMA_NONE);
      plan_o.ctrl[c2_req_q.valid].has_s2pf = (c3_req_q.s2pf_dma != DMA_NONE);
      plan_o.ctrl[c2_req_q.valid].dma_s1_both = (c3_req_q.dma_s1 == DMA_BOTH);
      plan_o.ctrl[c2_req_q.valid].dma_late_both =
          (c3_req_q.s2pf_dma != DMA_NONE) ?
          (c3_req_q.s2pf_dma == DMA_BOTH) :
          (c3_req_q.dma_s3 == DMA_BOTH);
    end
  end

  assign done_o = (st_q == ST_DONE);
  assign feasible_o = feasible_q;
  assign remove_count_o = (c2_req_q.valid && c3_req_q.valid &&
                           (c2_req_q.eid != c3_req_q.eid)) ? 2'd2 : 2'd1;
  assign remove_eid_a_o = (assignment_swap_q && (remove_count_o == 2'd2)) ?
                          c3_req_q.eid :
                          (c2_req_q.valid ? c2_req_q.eid : c3_req_q.eid);
  assign remove_eid_b_o = (remove_count_o == 2'd2) ?
                          (assignment_swap_q ? c2_req_q.eid : c3_req_q.eid) :
                          remove_eid_a_o;
  assign selected_max_o = (c2_req_q.ntok > c3_req_q.ntok) ?
                          c2_req_q.ntok : c3_req_q.ntok;
  assign selected_sum_o = c2_req_q.ntok + c3_req_q.ntok;
  assign s2pf_count_o = {1'b0, c2_req_q.s2pf_dma != DMA_NONE} +
                        {1'b0, c3_req_q.s2pf_dma != DMA_NONE};
  assign latest_start_o = start_time_q;

endmodule
