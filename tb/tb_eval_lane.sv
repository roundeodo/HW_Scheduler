// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// tb_eval_lane.sv — SystemVerilog testbench for sched_candidate_eval_lane
//
// Reads eval_vectors.txt (88 space-separated integers per line) produced by
// gen_eval_vectors.  Drives the DUT combinationally, checks all output fields,
// and reports PASS / FAIL with per-field detail on every mismatch.
//
// Compile + run via the accompanying Makefile.

`timescale 1ns/1ps
import sched_pkg::*;

module tb_eval_lane;

  // ── DUT port signals ──────────────────────────────────────────────────────
  logic               clk_i;
  logic               rst_ni;
  logic               start_i;
  logic               busy_o;
  logic               done_o;
  logic               cand_valid_i;
  logic [1:0]         plan_type_i;
  logic               cluster_a_i;
  s2pf_policy_t       s2pf_policy_i;
  logic               force_shape_a_i;
  logic               force_shape_b_i;
  logic [1:0]         forced_s1a_i;
  logic [1:0]         forced_s3a_i;
  logic [1:0]         forced_s1b_i;
  logic [1:0]         forced_s3b_i;
  logic               cost_only_tie_i;
  logic               score_makespan_only_i;

  eval_snap_t         base_snap_a_i;
  eval_snap_t         base_snap_b_i;

  logic               side_a_valid_i;
  logic               side_b_valid_i;
  logic [T_W-1:0]     start_a_i;
  logic [T_W-1:0]     start_b_i;
  logic [EID_RAW_W-1:0]  eid_a_i;
  logic [EID_RAW_W-1:0]  eid_b_i;
  logic [NTOK_W-1:0]  ntok_a_i;
  logic [NTOK_W-1:0]  ntok_b_i;
  logic [NTOK_W-1:0]  tok_start_a_i;
  logic [NTOK_W-1:0]  tok_start_b_i;
  logic               sw_a_i;
  logic               dn_a_i;
  logic               sw_b_i;
  logic               dn_b_i;
  logic [T_W-1:0]     shape_t0_i;

  logic [NR_W-1:0]       rem_len_after_i;
  logic [EID_RAW_W-1:0]  rem0_eid_i;
  logic [NTOK_W-1:0]     rem0_ntok_i;
  logic [NTOK_W-1:0]     rem1_ntok_i;
  logic [T_W-1:0]        total_conc_after_i;
  logic [T_W-1:0]        max_conc_after_i;

  // ── DUT outputs ───────────────────────────────────────────────────────────
  logic           eval_valid_o;
  logic           bw_ok_o;
  logic [T_W-1:0] makespan_o;
  score_key_t     score_key_o;
  plan_desc_t     plan_desc_o;
  eval_snap_t     snap_a_o;
  eval_snap_t     snap_b_o;
  logic [1:0]     shape_s1a_o, shape_s3a_o, shape_s1b_o, shape_s3b_o;
  logic [T_W-1:0] task_end_a_o, task_end_b_o;
  logic [T_W-1:0] s2_end_a_o,   s2_end_b_o;
  logic [T_W-1:0] s4_start_a_o, s4_start_b_o;
  logic [BW_W-1:0] bw_s1a_o, bw_s3a_o, bw_s1b_o, bw_s3b_o;
  logic [NTOK_W-1:0] m_s2_a_o, m_s4_a_o, m_s2_b_o, m_s4_b_o;
  logic              skip_s2_a_o, skip_s4_a_o, skip_s2_b_o, skip_s4_b_o;
  logic [1:0]        dma_s1_a_o, dma_s3_a_o, dma_s1_b_o, dma_s3_b_o;
  logic              bw_start;
  eval_snap_t        bw_snap_a;
  eval_snap_t        bw_snap_b;
  logic              bw_done;
  logic              bw_ok;

  // ── DUT instantiation ─────────────────────────────────────────────────────
  sched_candidate_eval_lane dut (
    .clk_i                 (clk_i),
    .rst_ni                (rst_ni),
    .start_i               (start_i),
    .busy_o                (busy_o),
    .done_o                (done_o),
    .cand_valid_i          (cand_valid_i),
    .plan_type_i           (plan_type_i),
    .cluster_a_i           (cluster_a_i),
    .s2pf_policy_i         (s2pf_policy_i),
    .force_shape_a_i       (force_shape_a_i),
    .force_shape_b_i       (force_shape_b_i),
    .forced_s1a_i          (forced_s1a_i),
    .forced_s3a_i          (forced_s3a_i),
    .forced_s1b_i          (forced_s1b_i),
    .forced_s3b_i          (forced_s3b_i),
    .cost_only_tie_i       (cost_only_tie_i),
    .score_makespan_only_i (score_makespan_only_i),
    .base_snap_a_i         (base_snap_a_i),
    .base_snap_b_i         (base_snap_b_i),
    .side_a_valid_i        (side_a_valid_i),
    .side_b_valid_i        (side_b_valid_i),
    .start_a_i             (start_a_i),
    .start_b_i             (start_b_i),
    .eid_a_i               (eid_a_i),
    .eid_b_i               (eid_b_i),
    .ntok_a_i              (ntok_a_i),
    .ntok_b_i              (ntok_b_i),
    .tok_start_a_i         (tok_start_a_i),
    .tok_start_b_i         (tok_start_b_i),
    .sw_a_i                (sw_a_i),
    .dn_a_i                (dn_a_i),
    .sw_b_i                (sw_b_i),
    .dn_b_i                (dn_b_i),
    .shape_t0_i            (shape_t0_i),
    .rem_len_after_i       (rem_len_after_i),
    .rem0_eid_i            (rem0_eid_i),
    .rem0_ntok_i           (rem0_ntok_i),
    .rem1_ntok_i           (rem1_ntok_i),
    .total_conc_after_i    (total_conc_after_i),
    .max_conc_after_i      (max_conc_after_i),
    .bw_start_o            (bw_start),
    .bw_snap_a_o           (bw_snap_a),
    .bw_snap_b_o           (bw_snap_b),
    .bw_done_i             (bw_done),
    .bw_ok_i               (bw_ok),
    .eval_valid_o          (eval_valid_o),
    .bw_ok_o               (bw_ok_o),
    .makespan_o            (makespan_o),
    .score_key_o           (score_key_o),
    .plan_desc_o           (plan_desc_o),
    .snap_a_o              (snap_a_o),
    .snap_b_o              (snap_b_o),
    .shape_s1a_o           (shape_s1a_o),
    .shape_s3a_o           (shape_s3a_o),
    .shape_s1b_o           (shape_s1b_o),
    .shape_s3b_o           (shape_s3b_o),
    .task_end_a_o          (task_end_a_o),
    .task_end_b_o          (task_end_b_o),
    .s2_end_a_o            (s2_end_a_o),
    .s2_end_b_o            (s2_end_b_o),
    .s4_start_a_o          (s4_start_a_o),
    .s4_start_b_o          (s4_start_b_o),
    .bw_s1a_o              (bw_s1a_o),
    .bw_s3a_o              (bw_s3a_o),
    .bw_s1b_o              (bw_s1b_o),
    .bw_s3b_o              (bw_s3b_o),
    .m_s2_a_o              (m_s2_a_o),
    .m_s4_a_o              (m_s4_a_o),
    .m_s2_b_o              (m_s2_b_o),
    .m_s4_b_o              (m_s4_b_o),
    .skip_s2_a_o           (skip_s2_a_o),
    .skip_s4_a_o           (skip_s4_a_o),
    .skip_s2_b_o           (skip_s2_b_o),
    .skip_s4_b_o           (skip_s4_b_o),
    .dma_s1_a_o            (dma_s1_a_o),
    .dma_s3_a_o            (dma_s3_a_o),
    .dma_s1_b_o            (dma_s1_b_o),
    .dma_s3_b_o            (dma_s3_b_o)
  );

  sched_bw_ok_seq i_eval_lane_bw (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .start_i  (bw_start),
    .busy_o   (),
    .done_o   (bw_done),
    .snap_a_i (bw_snap_a),
    .snap_b_i (bw_snap_b),
    .ok_o     (bw_ok)
  );

  // ── Simulation-time constants ─────────────────────────────────────────────
  localparam string VEC_FILE = "eval_vectors.txt";
  localparam int    SETTLE_NS = 1;  // combinational settle time

  initial clk_i = 1'b0;
  always #5 clk_i = ~clk_i;

  // ── Counters ──────────────────────────────────────────────────────────────
  int total_tests  = 0;
  int total_pass   = 0;
  int total_fail   = 0;

  // ── Task: drive one base_snap_t ───────────────────────────────────────────
  // Each snap field read from file as a plain int (signed OK for display;
  // hardware sees the low bits).
  task automatic drive_base_snap_a(
    input int f_valid,
    input int f_task_start, input int f_task_end,
    input int f_dma1_end,   input int f_s1_end, input int f_s2_end,
    input int f_dma3_end,   input int f_s3_end, input int f_s4_start,
    input int f_bw_s1,      input int f_bw_s3,
    input int f_s2pf_valid, input int f_s2pf_start, input int f_s2pf_end,
    input int f_s2pf_bw,
    input int f_ntok,
    input int f_pf_eid, input int f_pf_end, input int f_pf_full
  );
    base_snap_a_i.valid      = f_valid[0];
    base_snap_a_i.task_start = T_W'(f_task_start);
    base_snap_a_i.task_end   = T_W'(f_task_end);
    base_snap_a_i.dma1_end   = T_W'(f_dma1_end);
    base_snap_a_i.s2_end     = T_W'(f_s2_end);
    base_snap_a_i.dma3_end   = T_W'(f_dma3_end);
    base_snap_a_i.s4_start   = T_W'(f_s4_start);
    base_snap_a_i.bw_s1      = BW_W'(f_bw_s1);
    base_snap_a_i.bw_s3      = BW_W'(f_bw_s3);
    base_snap_a_i.s2pf_valid = f_s2pf_valid[0];
    base_snap_a_i.s2pf_start = T_W'(f_s2pf_start);
    base_snap_a_i.s2pf_end   = T_W'(f_s2pf_end);
    base_snap_a_i.s2pf_bw    = BW_W'(f_s2pf_bw);
    base_snap_a_i.s4pf_valid = 1'b0;
    base_snap_a_i.s4pf_start = '0;
    base_snap_a_i.ntok       = NTOK_W'(f_ntok);
    base_snap_a_i.pf_eid     = EID_W'(f_pf_eid);
    base_snap_a_i.pf_end     = T_W'(f_pf_end);
    base_snap_a_i.pf_full    = f_pf_full[0];
  endtask

  task automatic drive_base_snap_b(
    input int f_valid,
    input int f_task_start, input int f_task_end,
    input int f_dma1_end,   input int f_s1_end, input int f_s2_end,
    input int f_dma3_end,   input int f_s3_end, input int f_s4_start,
    input int f_bw_s1,      input int f_bw_s3,
    input int f_s2pf_valid, input int f_s2pf_start, input int f_s2pf_end,
    input int f_s2pf_bw,
    input int f_ntok,
    input int f_pf_eid, input int f_pf_end, input int f_pf_full
  );
    base_snap_b_i.valid      = f_valid[0];
    base_snap_b_i.task_start = T_W'(f_task_start);
    base_snap_b_i.task_end   = T_W'(f_task_end);
    base_snap_b_i.dma1_end   = T_W'(f_dma1_end);
    base_snap_b_i.s2_end     = T_W'(f_s2_end);
    base_snap_b_i.dma3_end   = T_W'(f_dma3_end);
    base_snap_b_i.s4_start   = T_W'(f_s4_start);
    base_snap_b_i.bw_s1      = BW_W'(f_bw_s1);
    base_snap_b_i.bw_s3      = BW_W'(f_bw_s3);
    base_snap_b_i.s2pf_valid = f_s2pf_valid[0];
    base_snap_b_i.s2pf_start = T_W'(f_s2pf_start);
    base_snap_b_i.s2pf_end   = T_W'(f_s2pf_end);
    base_snap_b_i.s2pf_bw    = BW_W'(f_s2pf_bw);
    base_snap_b_i.s4pf_valid = 1'b0;
    base_snap_b_i.s4pf_start = '0;
    base_snap_b_i.ntok       = NTOK_W'(f_ntok);
    base_snap_b_i.pf_eid     = EID_W'(f_pf_eid);
    base_snap_b_i.pf_end     = T_W'(f_pf_end);
    base_snap_b_i.pf_full    = f_pf_full[0];
  endtask

  // ── Main test loop ────────────────────────────────────────────────────────
  initial begin : tb_main
    int fd;
    int tid;

    // Per-field integer temporaries for $fscanf
    // Control [0..10]
    int f_plan_type, f_cluster_a;
    int f_s2pf_policy, f_s2pf_reserved;
    int f_force_a, f_force_b;
    int f_fs1a, f_fs3a, f_fs1b, f_fs3b;
    int f_cost_only_tie;
    // Tasks [11..24]
    int f_side_a_valid, f_side_b_valid;
    int f_start_a, f_start_b;
    int f_eid_a, f_eid_b;
    int f_ntok_a, f_ntok_b;
    int f_tok_start_a, f_tok_start_b;
    int f_sw_a, f_dn_a, f_sw_b, f_dn_b;
    // shape_t0 [25]
    int f_shape_t0;
    // rem [26..31]
    int f_rem_len, f_rem0_eid;
    int f_rem0_ntok, f_rem1_ntok;
    int f_total_conc, f_max_conc;
    // base_snap_a [32..50]
    int fa_valid, fa_task_start, fa_task_end;
    int fa_dma1_end, fa_s1_end, fa_s2_end;
    int fa_dma3_end, fa_s3_end, fa_s4_start;
    int fa_bw_s1, fa_bw_s3;
    int fa_s2pf_valid, fa_s2pf_start, fa_s2pf_end, fa_s2pf_bw;
    int fa_ntok, fa_pf_eid, fa_pf_end, fa_pf_full;
    // base_snap_b [51..69]
    int fb_valid, fb_task_start, fb_task_end;
    int fb_dma1_end, fb_s1_end, fb_s2_end;
    int fb_dma3_end, fb_s3_end, fb_s4_start;
    int fb_bw_s1, fb_bw_s3;
    int fb_s2pf_valid, fb_s2pf_start, fb_s2pf_end, fb_s2pf_bw;
    int fb_ntok, fb_pf_eid, fb_pf_end, fb_pf_full;
    // Expected outputs [70..87]
    int exp_bw_ok;
    int exp_s1a, exp_s3a, exp_s1b, exp_s3b;
    int exp_task_end_a, exp_task_end_b;
    int exp_s2_end_a, exp_s2_end_b;
    int exp_s4_start_a, exp_s4_start_b;
    int exp_s2pf_valid_a, exp_s2pf_start_a;
    int exp_s2pf_valid_b, exp_s2pf_start_b;
    int exp_cost, exp_makespan, exp_eval_valid;

    int fail_this;
    int scan_ret;

    fd = $fopen(VEC_FILE, "r");
    if (fd == 0) begin
      $error("[TB] Cannot open %s", VEC_FILE);
      $finish;
    end

    start_i      = 1'b0;
    cand_valid_i = 1'b1;
    score_makespan_only_i = 1'b0;
    tid          = 0;

    rst_ni = 1'b0;
    repeat (3) @(posedge clk_i);
    rst_ni = 1'b1;
    @(posedge clk_i);

    while (!$feof(fd)) begin
      // ── Read one line (88 fields) ─────────────────────────────────────
      scan_ret = $fscanf(fd,
        // [0..10] control
        "%d %d %d %d %d %d %d %d %d %d %d",
        f_plan_type, f_cluster_a,
        f_s2pf_policy, f_s2pf_reserved,
        f_force_a, f_force_b,
        f_fs1a, f_fs3a, f_fs1b, f_fs3b,
        f_cost_only_tie);
      if ($feof(fd)) break;

      scan_ret = $fscanf(fd,
        // [11..25] tasks + shape_t0
        "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
        f_side_a_valid, f_side_b_valid,
        f_start_a, f_start_b,
        f_eid_a, f_eid_b,
        f_ntok_a, f_ntok_b,
        f_tok_start_a, f_tok_start_b,
        f_sw_a, f_dn_a, f_sw_b, f_dn_b,
        f_shape_t0);

      scan_ret = $fscanf(fd,
        // [26..31] rem
        "%d %d %d %d %d %d",
        f_rem_len, f_rem0_eid,
        f_rem0_ntok, f_rem1_ntok,
        f_total_conc, f_max_conc);

      scan_ret = $fscanf(fd,
        // [32..50] base_snap_a
        "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
        fa_valid, fa_task_start, fa_task_end,
        fa_dma1_end, fa_s1_end, fa_s2_end,
        fa_dma3_end, fa_s3_end, fa_s4_start,
        fa_bw_s1, fa_bw_s3,
        fa_s2pf_valid, fa_s2pf_start, fa_s2pf_end, fa_s2pf_bw,
        fa_ntok, fa_pf_eid, fa_pf_end, fa_pf_full);

      scan_ret = $fscanf(fd,
        // [51..69] base_snap_b
        "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
        fb_valid, fb_task_start, fb_task_end,
        fb_dma1_end, fb_s1_end, fb_s2_end,
        fb_dma3_end, fb_s3_end, fb_s4_start,
        fb_bw_s1, fb_bw_s3,
        fb_s2pf_valid, fb_s2pf_start, fb_s2pf_end, fb_s2pf_bw,
        fb_ntok, fb_pf_eid, fb_pf_end, fb_pf_full);

      scan_ret = $fscanf(fd,
        // [70..87] expected
        "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
        exp_bw_ok,
        exp_s1a, exp_s3a, exp_s1b, exp_s3b,
        exp_task_end_a, exp_task_end_b,
        exp_s2_end_a, exp_s2_end_b,
        exp_s4_start_a, exp_s4_start_b,
        exp_s2pf_valid_a, exp_s2pf_start_a,
        exp_s2pf_valid_b, exp_s2pf_start_b,
        exp_cost, exp_makespan, exp_eval_valid);

      // ── Drive DUT inputs ──────────────────────────────────────────────
      plan_type_i          = 2'(f_plan_type);
      cluster_a_i          = f_cluster_a[0];
      s2pf_policy_i        = s2pf_policy_t'(f_s2pf_policy[1:0]);
      force_shape_a_i      = f_force_a[0];
      force_shape_b_i      = f_force_b[0];
      forced_s1a_i         = 2'(f_fs1a);
      forced_s3a_i         = 2'(f_fs3a);
      forced_s1b_i         = 2'(f_fs1b);
      forced_s3b_i         = 2'(f_fs3b);
      cost_only_tie_i      = f_cost_only_tie[0];

      side_a_valid_i       = f_side_a_valid[0];
      side_b_valid_i       = f_side_b_valid[0];
      start_a_i            = T_W'(f_start_a);
      start_b_i            = T_W'(f_start_b);
      eid_a_i              = EID_RAW_W'(f_eid_a);
      eid_b_i              = EID_RAW_W'(f_eid_b);
      ntok_a_i             = NTOK_W'(f_ntok_a);
      ntok_b_i             = NTOK_W'(f_ntok_b);
      tok_start_a_i        = NTOK_W'(f_tok_start_a);
      tok_start_b_i        = NTOK_W'(f_tok_start_b);
      sw_a_i               = f_sw_a[0];
      dn_a_i               = f_dn_a[0];
      sw_b_i               = f_sw_b[0];
      dn_b_i               = f_dn_b[0];
      shape_t0_i           = T_W'(f_shape_t0);

      rem_len_after_i      = NR_W'(f_rem_len);
      rem0_eid_i           = EID_RAW_W'(f_rem0_eid);
      rem0_ntok_i          = NTOK_W'(f_rem0_ntok);
      rem1_ntok_i          = NTOK_W'(f_rem1_ntok);
      total_conc_after_i   = T_W'(f_total_conc);
      max_conc_after_i     = T_W'(f_max_conc);

      drive_base_snap_a(fa_valid, fa_task_start, fa_task_end,
                        fa_dma1_end, fa_s1_end, fa_s2_end,
                        fa_dma3_end, fa_s3_end, fa_s4_start,
                        fa_bw_s1, fa_bw_s3,
                        fa_s2pf_valid, fa_s2pf_start, fa_s2pf_end, fa_s2pf_bw,
                        fa_ntok, fa_pf_eid, fa_pf_end, fa_pf_full);

      drive_base_snap_b(fb_valid, fb_task_start, fb_task_end,
                        fb_dma1_end, fb_s1_end, fb_s2_end,
                        fb_dma3_end, fb_s3_end, fb_s4_start,
                        fb_bw_s1, fb_bw_s3,
                        fb_s2pf_valid, fb_s2pf_start, fb_s2pf_end, fb_s2pf_bw,
                        fb_ntok, fb_pf_eid, fb_pf_end, fb_pf_full);

      // ── Start sequential evaluator and wait for completion ────────────
      start_i = 1'b1;
      @(posedge clk_i);
      start_i = 1'b0;
      wait (done_o === 1'b1);
      #(SETTLE_NS);

      // ── Check all outputs ─────────────────────────────────────────────
      total_tests++;
      fail_this = 0;

`define CHK(sig, exp, name) \
  if (int'(sig) !== exp) begin \
    if (!fail_this) \
      $display("[FAIL] tid=%0d  ntok_a=%0d ntok_b=%0d sw(%0d,%0d,%0d,%0d) s2pf_policy=%0d", \
               tid, f_ntok_a, f_ntok_b, f_sw_a, f_dn_a, f_sw_b, f_dn_b, f_s2pf_policy); \
    $display("       %-20s  got=%0d  exp=%0d", name, int'(sig), exp); \
    fail_this = 1; \
  end

      // bw_ok
      `CHK(bw_ok_o,         exp_bw_ok,       "bw_ok")
      // shapes
      `CHK(shape_s1a_o,     exp_s1a,         "shape_s1a")
      `CHK(shape_s3a_o,     exp_s3a,         "shape_s3a")
      `CHK(shape_s1b_o,     exp_s1b,         "shape_s1b")
      `CHK(shape_s3b_o,     exp_s3b,         "shape_s3b")
      // timing A
      `CHK(task_end_a_o,    exp_task_end_a,  "task_end_a")
      `CHK(s2_end_a_o,      exp_s2_end_a,    "s2_end_a")
      `CHK(s4_start_a_o,    exp_s4_start_a,  "s4_start_a")
      // timing B
      `CHK(task_end_b_o,    exp_task_end_b,  "task_end_b")
      `CHK(s2_end_b_o,      exp_s2_end_b,    "s2_end_b")
      `CHK(s4_start_b_o,    exp_s4_start_b,  "s4_start_b")
      // s2pf A
      `CHK(snap_a_o.s2pf_valid, exp_s2pf_valid_a, "s2pf_valid_a")
      if (exp_s2pf_valid_a) begin
        `CHK(snap_a_o.s2pf_start, exp_s2pf_start_a, "s2pf_start_a")
      end
      // s2pf B
      `CHK(snap_b_o.s2pf_valid, exp_s2pf_valid_b, "s2pf_valid_b")
      if (exp_s2pf_valid_b) begin
        `CHK(snap_b_o.s2pf_start, exp_s2pf_start_b, "s2pf_start_b")
      end
      // score_unit 只有在 candidate 通过 bw/S2PF 检查并拉高 eval_valid 时才会启动。
      // 对 invalid candidate, score_key_o.cost 不参与 best_reduce，不能作为有效输出检查。
      if (exp_eval_valid) begin
        `CHK(score_key_o.cost, exp_cost,     "cost")
      end
      // makespan
      `CHK(makespan_o,       exp_makespan,   "makespan")
      // eval_valid
      `CHK(eval_valid_o,     exp_eval_valid, "eval_valid")

`undef CHK

      // plan_desc pass-through checks (always, independent of bw_ok)
      if (int'(plan_desc_o.plan_type) !== f_plan_type) begin
        $display("[FAIL] tid=%0d  plan_type  got=%0d exp=%0d",
                 tid, int'(plan_desc_o.plan_type), f_plan_type);
        fail_this = 1;
      end
      if (int'(plan_desc_o.ntok_a) !== f_ntok_a && f_side_a_valid) begin
        $display("[FAIL] tid=%0d  ntok_a in plan_desc  got=%0d exp=%0d",
                 tid, int'(plan_desc_o.ntok_a), f_ntok_a);
        fail_this = 1;
      end
      if (int'(plan_desc_o.ntok_b) !== f_ntok_b && f_side_b_valid) begin
        $display("[FAIL] tid=%0d  ntok_b in plan_desc  got=%0d exp=%0d",
                 tid, int'(plan_desc_o.ntok_b), f_ntok_b);
        fail_this = 1;
      end

      if (fail_this) total_fail++;
      else           total_pass++;

      tid++;
      @(posedge clk_i);
    end

    $fclose(fd);

    // ── Summary ─────────────────────────────────────────────────────────
    $display("══════════════════════════════════════════════════════");
    $display("  sched_candidate_eval_lane testbench SUMMARY");
    $display("  Total  : %0d", total_tests);
    $display("  PASS   : %0d", total_pass);
    $display("  FAIL   : %0d", total_fail);
    $display("══════════════════════════════════════════════════════");

    if (total_fail == 0)
      $display("  RESULT : ALL PASS");
    else
      $display("  RESULT : *** FAIL ***");

    $finish;
  end : tb_main

endmodule
