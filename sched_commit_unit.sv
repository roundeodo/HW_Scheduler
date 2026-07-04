// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - winning candidate commit helper
//
// Registered timing version.  The S4PF allow decision needs a BW check, so it
// must not be a same-cycle combinational extension of replay/fallback.  This
// unit latches the winning plan/snap, runs two registered bw_ok checks, then
// emits compact task descriptors for one cycle when done_o is high.

import sched_pkg::*;

module sched_commit_unit (
  input  logic                    clk_i,
  input  logic                    rst_ni,
  input  logic                    commit_i,
  input  logic                    best_valid_i,
  input  plan_desc_t              best_plan_i,
  input  snap_timeline_t          best_timeline_a_i,   // physical C2
  input  snap_timeline_t          best_timeline_b_i,   // physical C3
  input  snap_cache_t             best_cache_a_i,
  input  snap_cache_t             best_cache_b_i,
  input  logic [1:0]              best_remove_count_i,
  input  logic [3:0]              best_remove_slot_mask_i,

  output logic                    busy_o,
  output logic                    done_o,
  output logic                    commit_valid_o,
  output snap_timeline_t          next_c2_timeline_o,
  output snap_timeline_t          next_c3_timeline_o,
  output snap_cache_t             next_c2_cache_o,
  output snap_cache_t             next_c3_cache_o,

  output logic [1:0]              plan_valid_o,
  output task_desc_t [1:0]        task_desc_o,
  output logic [1:0]              plan_allow_s4pf_o,
  output logic [1:0]              plan_count_o,

  output logic [1:0]              remove_count_o,
  output logic [3:0]              remove_slot_mask_o,

  output logic                    bw_start_o,
  output snap_timeline_t          bw_snap_a_o,
  output snap_timeline_t          bw_snap_b_o,
  input  logic                    bw_done_i,
  input  logic                    bw_ok_i
);

  localparam logic [1:0] PLAN_PAIR  = 2'b00;
  localparam logic [1:0] PLAN_SPLIT = 2'b01;
  localparam logic [1:0] PLAN_SOLO  = 2'b10;

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_BW_A_START,
    ST_BW_A_WAIT,
    ST_BW_B_START,
    ST_BW_B_WAIT,
    ST_DONE
  } state_t;

  state_t st_q, st_d;

  logic       valid_q, valid_d;
  plan_desc_t plan_q, plan_d;
  snap_timeline_t timeline_a_q, timeline_a_d;
  snap_timeline_t timeline_b_q, timeline_b_d;
  snap_cache_t cache_a_q, cache_a_d;
  snap_cache_t cache_b_q, cache_b_d;
  logic [1:0] remove_count_q, remove_count_d;
  logic [3:0] remove_mask_q, remove_mask_d;
  logic       allow_a_q, allow_a_d;
  logic       allow_b_q, allow_b_d;

  snap_timeline_t s4pf_timeline_a;
  snap_timeline_t s4pf_timeline_b;
  logic       s4pf_candidate_a;
  logic       s4pf_candidate_b;
  logic       allow_s4pf_a;
  logic       allow_s4pf_b;

  function automatic task_desc_t make_task_from_a(input plan_desc_t p, input logic cluster);
    task_desc_t d;
    d = '0;
    d.cluster   = cluster;
    d.eid       = p.eid_a;
    d.ntok      = p.ntok_a;
    d.tok_start = p.tok_start_a;
    d.s1        = p.s1a;
    d.s3        = p.s3a;
    d.skip_s1   = p.skip_s1_a;
    d.skip_s3   = p.skip_s3_a;
    d.has_s2pf  = p.has_s2pf_a;
    make_task_from_a = d;
  endfunction

  function automatic task_desc_t make_task_from_b(input plan_desc_t p, input logic cluster);
    task_desc_t d;
    d = '0;
    d.cluster   = cluster;
    d.eid       = p.eid_b;
    d.ntok      = p.ntok_b;
    d.tok_start = p.tok_start_b;
    d.s1        = p.s1b;
    d.s3        = p.s3b;
    d.skip_s1   = p.skip_s1_b;
    d.skip_s3   = p.skip_s3_b;
    d.has_s2pf  = p.has_s2pf_b;
    make_task_from_b = d;
  endfunction

  assign s4pf_candidate_a = timeline_a_q.valid &&
                            (cache_a_q.pf_eid == PF_EID_NONE) &&
                            (timeline_a_q.dma1_end <= timeline_a_q.s4_start) &&
                            ((timeline_a_q.s4_start + GHOST_WINDOW_TICKS) <= timeline_a_q.task_end);
  assign s4pf_candidate_b = timeline_b_q.valid &&
                            (cache_b_q.pf_eid == PF_EID_NONE) &&
                            (timeline_b_q.dma1_end <= timeline_b_q.s4_start) &&
                            ((timeline_b_q.s4_start + GHOST_WINDOW_TICKS) <= timeline_b_q.task_end);

  always_comb begin
    s4pf_timeline_a = timeline_a_q;
    s4pf_timeline_b = timeline_b_q;
    if (s4pf_candidate_a) begin
      s4pf_timeline_a.s4pf_valid = 1'b1;
      s4pf_timeline_a.s4pf_start = timeline_a_q.s4_start;
    end
    if (s4pf_candidate_b) begin
      s4pf_timeline_b.s4pf_valid = 1'b1;
      s4pf_timeline_b.s4pf_start = timeline_b_q.s4_start;
    end
  end

  assign allow_s4pf_a = allow_a_q;
  assign allow_s4pf_b = allow_b_q;

  always_comb begin
    st_d           = st_q;
    valid_d        = valid_q;
    plan_d         = plan_q;
    timeline_a_d   = timeline_a_q;
    timeline_b_d   = timeline_b_q;
    cache_a_d      = cache_a_q;
    cache_b_d      = cache_b_q;
    remove_count_d = remove_count_q;
    remove_mask_d  = remove_mask_q;
    allow_a_d      = allow_a_q;
    allow_b_d      = allow_b_q;
    bw_start_o     = 1'b0;
    bw_snap_a_o    = s4pf_timeline_a;
    bw_snap_b_o    = timeline_b_q;

    unique case (st_q)
      ST_IDLE: begin
        if (commit_i) begin
          valid_d        = best_valid_i;
          plan_d         = best_plan_i;
          timeline_a_d   = best_timeline_a_i;
          timeline_b_d   = best_timeline_b_i;
          cache_a_d      = best_cache_a_i;
          cache_b_d      = best_cache_b_i;
          remove_count_d = best_remove_count_i;
          remove_mask_d  = best_remove_slot_mask_i;
          allow_a_d      = 1'b0;
          allow_b_d      = 1'b0;
          st_d           = best_valid_i ? ST_BW_A_START : ST_DONE;
        end
      end

      ST_BW_A_START: begin
        if (s4pf_candidate_a) begin
          bw_start_o  = 1'b1;
          bw_snap_a_o = s4pf_timeline_a;
          bw_snap_b_o = timeline_b_q;
          st_d        = ST_BW_A_WAIT;
        end else begin
          allow_a_d = 1'b0;
          st_d      = ST_BW_B_START;
        end
      end

      ST_BW_A_WAIT: begin
        if (bw_done_i) begin
          allow_a_d = bw_ok_i;
          st_d      = ST_BW_B_START;
        end
      end

      ST_BW_B_START: begin
        if (s4pf_candidate_b) begin
          bw_start_o  = 1'b1;
          bw_snap_a_o = timeline_a_q;
          bw_snap_b_o = s4pf_timeline_b;
          st_d        = ST_BW_B_WAIT;
        end else begin
          allow_b_d = 1'b0;
          st_d      = ST_DONE;
        end
      end

      ST_BW_B_WAIT: begin
        if (bw_done_i) begin
          allow_b_d = bw_ok_i;
          st_d      = ST_DONE;
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
      st_q           <= ST_IDLE;
      valid_q        <= 1'b0;
      plan_q         <= '0;
      timeline_a_q   <= '0;
      timeline_b_q   <= '0;
      cache_a_q      <= '0;
      cache_b_q      <= '0;
      remove_count_q <= '0;
      remove_mask_q  <= '0;
      allow_a_q      <= 1'b0;
      allow_b_q      <= 1'b0;
    end else begin
      st_q           <= st_d;
      valid_q        <= valid_d;
      plan_q         <= plan_d;
      timeline_a_q   <= timeline_a_d;
      timeline_b_q   <= timeline_b_d;
      cache_a_q      <= cache_a_d;
      cache_b_q      <= cache_b_d;
      remove_count_q <= remove_count_d;
      remove_mask_q  <= remove_mask_d;
      allow_a_q      <= allow_a_d;
      allow_b_q      <= allow_b_d;
    end
  end

  always_comb begin
    commit_valid_o    = (st_q == ST_DONE) && valid_q;
    next_c2_timeline_o = timeline_a_q;
    next_c3_timeline_o = timeline_b_q;
    next_c2_cache_o    = cache_a_q;
    next_c3_cache_o    = cache_b_q;
    plan_valid_o      = '0;
    task_desc_o       = '{default: '0};
    plan_allow_s4pf_o = '0;
    plan_count_o      = '0;
    remove_count_o    = commit_valid_o ? remove_count_q : '0;
    remove_slot_mask_o = commit_valid_o ? remove_mask_q : '0;

    if (commit_valid_o) begin
      unique case (plan_q.plan_type)
        PLAN_PAIR, PLAN_SPLIT: begin
          plan_valid_o[0]       = 1'b1;
          task_desc_o[0]        = make_task_from_a(plan_q, 1'b0);
          plan_allow_s4pf_o[0]  = allow_s4pf_a;
          plan_valid_o[1]       = 1'b1;
          task_desc_o[1]        = make_task_from_b(plan_q, 1'b1);
          plan_allow_s4pf_o[1]  = allow_s4pf_b;
          plan_count_o          = 2'd2;
        end

        PLAN_SOLO: begin
          plan_valid_o[0] = 1'b1;
          if (plan_q.cluster_a == 1'b0) begin
            task_desc_o[0]       = make_task_from_a(plan_q, 1'b0);
            plan_allow_s4pf_o[0] = allow_s4pf_a;
          end else if (plan_q.ntok_a != '0) begin
            task_desc_o[0]       = make_task_from_a(plan_q, 1'b1);
            plan_allow_s4pf_o[0] = allow_s4pf_b;
          end else begin
            task_desc_o[0]       = make_task_from_b(plan_q, 1'b1);
            plan_allow_s4pf_o[0] = allow_s4pf_b;
          end
          plan_count_o = 2'd1;
        end

        default: begin
        end
      endcase
    end
  end

  assign busy_o = (st_q != ST_IDLE) && (st_q != ST_DONE);
  assign done_o = (st_q == ST_DONE);

endmodule
