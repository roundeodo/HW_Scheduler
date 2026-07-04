// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// Candidate issue package
//
// This package is intentionally narrow: it only defines the descriptor that
// crosses candidate_generator -> candidate_eval_lane -> best_reduce.  It is
// kept out of sched_pkg so the global package does not become a dumping ground
// for full internal datapath records.

package sched_candidate_pkg;

  import sched_pkg::*;

  typedef struct packed {
    logic                  valid;
    logic [CAND_ID_W-1:0]  candidate_id;
    logic [1:0]            plan_type;
    logic                  cluster_a;
    s2pf_policy_t          s2pf_policy;
    logic                  force_shape_a;
    logic                  force_shape_b;
    shape_t                forced_s1a;
    shape_t                forced_s3a;
    shape_t                forced_s1b;
    shape_t                forced_s3b;
    logic                  cost_only_tie;
    logic                  score_makespan_only;

    logic                  side_a_valid;
    logic                  side_b_valid;
    time_t                 start_a;
    time_t                 start_b;
    logic [EID_RAW_W-1:0]  eid_a;
    logic [EID_RAW_W-1:0]  eid_b;
    ntok_t                 ntok_a;
    ntok_t                 ntok_b;
    ntok_t                 tok_start_a;
    ntok_t                 tok_start_b;

    logic                  sw_a;
    logic                  dn_a;
    logic                  sw_b;
    logic                  dn_b;
    time_t                 shape_t0;

    logic [NR_W-1:0]       rem_len_after;
    logic [EID_RAW_W-1:0]  rem0_eid;
    ntok_t                 rem0_ntok;
    ntok_t                 rem1_ntok;
    time_t                 total_conc_after;
    time_t                 max_conc_after;
    logic [3:0]            remove_slot_mask;
  } cand_issue_t;

endpackage
