// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — winning candidate commit helper
//
// Pure-slave datapath version.  This module does not write an internal
// plan_sram and does not update a rem_manager.  It converts the replayed
// winner into at most two compact single-task descriptors plus compact remove
// information.  It also emits an explicit allow_s4pf bit per output task so
// CVA6 does not need to save full timing snaps in L3 just to decide whether
// the task can prefetch the next same-cluster expert during S4.
// CVA6/software later reads these outputs, writes the real L3 plan buffer,
// and updates the L3/software rem list.

import sched_pkg::*;

module sched_commit_unit (
  input  logic                    commit_i,
  input  logic                    best_valid_i,
  input  plan_desc_t              best_plan_i,
  input  eval_snap_t              best_snap_a_i,       // physical C2
  input  eval_snap_t              best_snap_b_i,       // physical C3
  input  logic [1:0]              best_remove_count_i,
  input  logic [3:0]              best_remove_slot_mask_i,

  output logic                    commit_valid_o,
  output eval_snap_t              next_c2_snap_o,
  output eval_snap_t              next_c3_snap_o,

  output logic [1:0]              plan_valid_o,
  output task_desc_t [1:0]        task_desc_o,
  output logic [1:0]              plan_allow_s4pf_o,
  output logic [1:0]              plan_count_o,

  output logic [1:0]              remove_count_o,
  output logic [3:0]              remove_slot_mask_o
);

  localparam logic [1:0] PLAN_PAIR  = 2'b00;
  localparam logic [1:0] PLAN_SPLIT = 2'b01;
  localparam logic [1:0] PLAN_SOLO  = 2'b10;

  eval_snap_t s4pf_snap_a;
  eval_snap_t s4pf_snap_b;
  logic       allow_s4pf_a;
  logic       allow_s4pf_b;
  logic       s4pf_bw_ok_a;
  logic       s4pf_bw_ok_b;

  function automatic eval_snap_t with_s4pf(input eval_snap_t s);
    eval_snap_t r;
    begin
      r = s;
      if (s.valid &&
          (s.pf_eid == PF_EID_NONE) &&
          (s.dma1_end <= s.s4_start) &&
          ((s.s4_start + GHOST_WINDOW_TICKS) <= s.task_end)) begin
        r.s4pf_valid = 1'b1;
        r.s4pf_start = s.s4_start;
      end
      with_s4pf = r;
    end
  endfunction

  assign s4pf_snap_a = with_s4pf(best_snap_a_i);
  assign s4pf_snap_b = with_s4pf(best_snap_b_i);

  sched_bw_ok i_s4pf_bw_ok_a (
    .snap_a_i (s4pf_snap_a),
    .snap_b_i (best_snap_b_i),
    .ok_o     (s4pf_bw_ok_a)
  );

  sched_bw_ok i_s4pf_bw_ok_b (
    .snap_a_i (best_snap_a_i),
    .snap_b_i (s4pf_snap_b),
    .ok_o     (s4pf_bw_ok_b)
  );

  assign allow_s4pf_a = best_snap_a_i.valid &&
                        (best_snap_a_i.pf_eid == PF_EID_NONE) &&
                        (best_snap_a_i.dma1_end <= best_snap_a_i.s4_start) &&
                        ((best_snap_a_i.s4_start + GHOST_WINDOW_TICKS) <= best_snap_a_i.task_end) &&
                        s4pf_bw_ok_a;
  assign allow_s4pf_b = best_snap_b_i.valid &&
                        (best_snap_b_i.pf_eid == PF_EID_NONE) &&
                        (best_snap_b_i.dma1_end <= best_snap_b_i.s4_start) &&
                        ((best_snap_b_i.s4_start + GHOST_WINDOW_TICKS) <= best_snap_b_i.task_end) &&
                        s4pf_bw_ok_b;

  // plan 最终按“单个实际 task”写入 L3，而 best_plan_i 可能是 PAIR/SPLIT。
  // make_task_from_a/b 把 winner descriptor 的 A/B 两侧分别规范化成
  // task_desc_t，便于软件后续按单 task lowering/发射。
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

  always_comb begin
    commit_valid_o   = commit_i && best_valid_i;
    next_c2_snap_o   = best_snap_a_i;
    next_c3_snap_o   = best_snap_b_i;

    plan_valid_o     = '0;
    task_desc_o      = '{default: '0};
    plan_allow_s4pf_o = '0;
    plan_count_o     = '0;

    remove_count_o   = commit_valid_o ? best_remove_count_i : '0;
    remove_slot_mask_o = commit_valid_o ? best_remove_slot_mask_i : '0;

    if (commit_valid_o) begin
      unique case (best_plan_i.plan_type)
        PLAN_PAIR, PLAN_SPLIT: begin
          // PAIR/SPLIT 都产生两个实际 task：
          //   entry0 = A 侧任务，物理 C2
          //   entry1 = B 侧任务，物理 C3
          plan_valid_o[0] = 1'b1;
          task_desc_o[0]  = make_task_from_a(best_plan_i, 1'b0);
          plan_allow_s4pf_o[0] = allow_s4pf_a;

          plan_valid_o[1] = 1'b1;
          task_desc_o[1]  = make_task_from_b(best_plan_i, 1'b1);
          plan_allow_s4pf_o[1] = allow_s4pf_b;

          plan_count_o     = 2'd2;
        end

        PLAN_SOLO: begin
          // SOLO 只产生一条 task。若 cluster_a==1 且 A 字段有效，
          // candidate_generator 会用 A 字段承载“放到 C3 的 solo”。
          plan_valid_o[0] = 1'b1;

          if (best_plan_i.cluster_a == 1'b0) begin
            task_desc_o[0] = make_task_from_a(best_plan_i, 1'b0);
            plan_allow_s4pf_o[0] = allow_s4pf_a;
          end else if (best_plan_i.ntok_a != '0) begin
            task_desc_o[0] = make_task_from_a(best_plan_i, 1'b1);
            plan_allow_s4pf_o[0] = allow_s4pf_b;
          end else begin
            task_desc_o[0] = make_task_from_b(best_plan_i, 1'b1);
            plan_allow_s4pf_o[0] = allow_s4pf_b;
          end

          plan_count_o     = 2'd1;
        end

        default: begin
          plan_valid_o      = '0;
          plan_allow_s4pf_o = '0;
          plan_count_o      = '0;
        end
      endcase
    end
  end

endmodule
