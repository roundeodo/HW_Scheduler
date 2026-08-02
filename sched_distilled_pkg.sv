// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51

package sched_distilled_pkg;

  import sched_pkg::*;

  localparam int unsigned DISTILLED_PROFILE_COUNT = 28;
  localparam int unsigned DISTILLED_LOGICAL_MAX   = 6;
  localparam int unsigned DISTILLED_PHYSICAL_MAX  = 18;
  localparam int unsigned DISTILLED_S4PF_MIN_REMAINING = 9;
  localparam int unsigned DIST_TOKEN_SUM_W = 9;
  localparam int unsigned DIST_BLOCK_SUM_W = 8;
  localparam int unsigned DIST_HIST_W      = 7;
  localparam int unsigned DIST_BOUND_W     = T_W + 1;

  // Task timelines use integer ticks.  Bounds need half-tick precision because
  // two free DMA lanes can retire an odd number of lane-ticks concurrently.
  typedef logic [DIST_BOUND_W-1:0] distilled_bound_t;

  typedef enum logic [1:0] {
    DIST_MODE_TERMINAL = 2'd0,
    DIST_MODE_SYNC     = 2'd1,
    DIST_MODE_ONE_IDLE = 2'd2
  } distilled_mode_t;

  typedef enum logic [1:0] {
    DIST_FAMILY_SINGLE = 2'd0,
    DIST_FAMILY_PAIR   = 2'd1,
    DIST_FAMILY_SPLIT  = 2'd2
  } distilled_family_t;

  typedef enum logic [2:0] {
    DIST_SEL_T0 = 3'd0,
    DIST_SEL_T1 = 3'd1,
    DIST_SEL_T2 = 3'd2,
    DIST_SEL_T3 = 3'd3,
    DIST_SEL_T4 = 3'd4,
    DIST_SEL_B0 = 3'd5
  } distilled_selector_t;

  typedef struct packed {
    logic [4:0]          profile_slot;
    logic [2:0]          logical_id;
    logic                logical_last;
    distilled_family_t   family;
    distilled_selector_t selector_a;
    distilled_selector_t selector_b;
    logic                split_balanced;
    logic                c2_active;
    logic                c3_active;
    shape_t              c2_shape_s1;
    shape_t              c2_shape_s3;
    shape_t              c3_shape_s1;
    shape_t              c3_shape_s3;
    dma_binding_t        c2_dma_s1;
    dma_binding_t        c2_dma_s3;
    dma_binding_t        c2_s2pf;
    dma_binding_t        c3_dma_s1;
    dma_binding_t        c3_dma_s3;
    dma_binding_t        c3_s2pf;
  } distilled_profile_t;

  // Persistent policy state.  S2PF endpoints are derived from dma1_end and its
  // binding.  A targeted S4PF is consumed by the same concrete transition, so
  // the only persistent cache is a complete, time-zero initial residency.
  typedef struct packed {
    logic                    cur_valid;
    time_t                   task_start;
    time_t                   task_end;
    time_t                   dma1_end;
    time_t                   s2_end;
    time_t                   dma3_end;
    dma_binding_t            dma_s1;
    dma_binding_t            dma_s3;
    dma_binding_t            s2pf_dma;
    logic                    cache_valid;
    logic [EID_RAW_W-1:0]    cache_eid;
  } distilled_cluster_state_t;

  typedef struct packed {
    logic [NR_W-1:0]             count;
    logic [DIST_TOKEN_SUM_W-1:0] token_sum;
    logic [DIST_HIST_W-1:0]      odd_count;
    logic [DIST_BLOCK_SUM_W-1:0] block_sum;
    logic [3:0][DIST_HIST_W-1:0] small_hist;
    distilled_bound_t            parent_bound;
  } distilled_counters_t;

  typedef struct packed {
    logic                    cur_valid;
    time_t                   task_start;
    time_t                   task_end;
    time_t                   dma1_end;
    time_t                   s2_end;
    time_t                   dma3_end;
    dma_binding_t            dma_s1;
    dma_binding_t            dma_s3;
    dma_binding_t            s2pf_dma;
    logic                    cache_valid;
    logic [EID_RAW_W-1:0]    cache_eid;
  } distilled_bound_cluster_t;

  typedef struct packed {
    logic [NR_W-1:0]             count;
    logic [DIST_BLOCK_SUM_W-1:0] block_sum;
    logic [3:0][DIST_HIST_W-1:0] small_hist;
    distilled_bound_t            parent_bound;
  } distilled_bound_counters_t;

  typedef struct packed {
    logic  valid;
    ntok_t ntok;
  } distilled_bound_head_t;

  typedef struct packed {
    logic [1:0] valid;
    logic [1:0] cluster;
    logic [1:0] skip_s1;
    logic [1:0] has_s2pf;
  } distilled_s4pf_consumer_t;

  typedef struct packed {
    logic         cur_valid;
    time_t        task_start;
    time_t        dma1_end;
    time_t        s2_end;
    time_t        dma3_end;
    time_t        task_end;
    dma_binding_t dma_s1;
    dma_binding_t dma_s3;
    dma_binding_t s2pf_dma;
    logic         cache_valid;
  } distilled_s4pf_cluster_t;

  typedef struct packed {
    logic             valid;
    logic [4:0]       profile_slot;
    logic [4:0]       mode_index;
    logic [2:0]       logical_id;
    logic             assignment_swap;
    time_t            start;
    dma_binding_t     targeted_s4pf_c2;
    dma_binding_t     targeted_s4pf_c3;
  } distilled_action_token_t;

  typedef struct packed {
    logic [4:0]       profile_addr;
    logic             assignment_swap;
    time_t            start;
    dma_binding_t     targeted_s4pf_c2;
    dma_binding_t     targeted_s4pf_c3;
  } distilled_replay_token_t;

  typedef struct packed {
    logic                    valid;
    distilled_replay_token_t token;
    time_t                   max_end;
    logic [T_W:0]            sum_end;
    logic [1:0]              s2pf_count;
  } distilled_group_record_t;

  typedef struct packed {
    distilled_bound_t f;
    distilled_bound_t h;
    distilled_bound_t compute_bound;
    distilled_bound_t dma_bound;
    time_t             early_end;
    time_t             late_end;
    ntok_t             selected_max;
    logic [NTOK_W:0]   selected_sum;
    logic [1:0]        s2pf_count;
    logic [NR_W-1:0]   remaining_count;
    logic [NR_W-1:0]   selected_min_rank;
    logic [NR_W-1:0]   selected_max_rank;
    logic              selects_t0;
  } distilled_score_record_t;

  typedef struct packed {
    logic low_work_progress;
    logic sparse_hot_sync;
    logic mid_plateau;
    logic short_tail_plateau;
    logic large_slack_fill;
  } distilled_regime_t;

  function automatic snap_bw_view_t distilled_bw_view(
    input distilled_cluster_state_t state
  );
    snap_bw_view_t view;
    begin
      view = '0;
      view.valid       = state.cur_valid;
      view.task_start  = state.task_start;
      view.dma1_end    = state.dma1_end;
      view.s2_end      = state.s2_end;
      view.dma3_end    = state.dma3_end;
      view.dma_s1      = state.dma_s1;
      view.dma_s3      = state.dma_s3;
      view.s2pf_valid  = (state.s2pf_dma != DMA_NONE);
      view.s2pf_start  = state.dma1_end;
      view.s2pf_end    = (state.s2pf_dma != DMA_NONE) ?
                         (state.dma1_end + s3_dma_ticks(state.s2pf_dma)) : '0;
      view.s2pf_dma    = state.s2pf_dma;
      distilled_bw_view = view;
    end
  endfunction

  function automatic distilled_bound_cluster_t distilled_bound_cluster_view(
    input distilled_cluster_state_t state
  );
    distilled_bound_cluster_t view;
    begin
      view.cur_valid = state.cur_valid;
      view.task_start = state.task_start;
      view.task_end = state.task_end;
      view.dma1_end = state.dma1_end;
      view.s2_end = state.s2_end;
      view.dma3_end = state.dma3_end;
      view.dma_s1 = state.dma_s1;
      view.dma_s3 = state.dma_s3;
      view.s2pf_dma = state.s2pf_dma;
      view.cache_valid = state.cache_valid;
      view.cache_eid = state.cache_eid;
      distilled_bound_cluster_view = view;
    end
  endfunction

  function automatic distilled_bound_counters_t distilled_bound_counters_view(
    input distilled_counters_t counters
  );
    distilled_bound_counters_t view;
    begin
      view.count = counters.count;
      view.block_sum = counters.block_sum;
      view.small_hist = counters.small_hist;
      view.parent_bound = counters.parent_bound;
      distilled_bound_counters_view = view;
    end
  endfunction

  function automatic distilled_s4pf_consumer_t distilled_s4pf_consumer_view(
    input winner_plan_t plan
  );
    distilled_s4pf_consumer_t view;
    begin
      for (int task_index = 0; task_index < 2; task_index++) begin
        view.valid[task_index] = plan.task_valid[task_index];
        view.cluster[task_index] = plan.ctrl[task_index].cluster;
        view.skip_s1[task_index] = plan.ctrl[task_index].skip_s1;
        view.has_s2pf[task_index] = plan.ctrl[task_index].has_s2pf;
      end
      distilled_s4pf_consumer_view = view;
    end
  endfunction

  function automatic distilled_s4pf_cluster_t distilled_s4pf_cluster_view(
    input distilled_cluster_state_t state
  );
    distilled_s4pf_cluster_t view;
    begin
      view.cur_valid = state.cur_valid;
      view.task_start = state.task_start;
      view.dma1_end = state.dma1_end;
      view.s2_end = state.s2_end;
      view.dma3_end = state.dma3_end;
      view.task_end = state.task_end;
      view.dma_s1 = state.dma_s1;
      view.dma_s3 = state.dma_s3;
      view.s2pf_dma = state.s2pf_dma;
      view.cache_valid = state.cache_valid;
      distilled_s4pf_cluster_view = view;
    end
  endfunction

  function automatic snap_bw_view_t distilled_s4pf_bw_view(
    input distilled_s4pf_cluster_t state
  );
    snap_bw_view_t view;
    begin
      view = '0;
      view.valid = state.cur_valid;
      view.task_start = state.task_start;
      view.dma1_end = state.dma1_end;
      view.s2_end = state.s2_end;
      view.dma3_end = state.dma3_end;
      view.dma_s1 = state.dma_s1;
      view.dma_s3 = state.dma_s3;
      view.s2pf_valid = (state.s2pf_dma != DMA_NONE);
      view.s2pf_start = state.dma1_end;
      view.s2pf_end = (state.s2pf_dma != DMA_NONE) ?
                      (state.dma1_end + s3_dma_ticks(state.s2pf_dma)) : '0;
      view.s2pf_dma = state.s2pf_dma;
      distilled_s4pf_bw_view = view;
    end
  endfunction

  function automatic logic distilled_cache_s1_hit(
    input distilled_cluster_state_t state,
    input logic [EID_RAW_W-1:0]     eid
  );
    distilled_cache_s1_hit = state.cache_valid &&
                             (state.cache_eid == eid);
  endfunction

  function automatic logic distilled_cache_s3_hit(
    input distilled_cluster_state_t state,
    input logic [EID_RAW_W-1:0]     eid
  );
    distilled_cache_s3_hit = distilled_cache_s1_hit(state, eid);
  endfunction

  function automatic logic [4:0] distilled_mode_profile_base(
    input distilled_mode_t mode
  );
    unique case (mode)
      DIST_MODE_TERMINAL: distilled_mode_profile_base = 5'd0;
      DIST_MODE_SYNC:     distilled_mode_profile_base = 5'd5;
      DIST_MODE_ONE_IDLE: distilled_mode_profile_base = 5'd13;
      default:            distilled_mode_profile_base = '0;
    endcase
  endfunction

  function automatic logic [4:0] distilled_mode_profile_limit(
    input distilled_mode_t mode
  );
    unique case (mode)
      DIST_MODE_TERMINAL: distilled_mode_profile_limit = 5'd5;
      DIST_MODE_SYNC:     distilled_mode_profile_limit = 5'd13;
      DIST_MODE_ONE_IDLE: distilled_mode_profile_limit = 5'd28;
      default:            distilled_mode_profile_limit = '0;
    endcase
  endfunction

  function automatic logic [2:0] distilled_mode_logical_count(
    input distilled_mode_t mode
  );
    unique case (mode)
      DIST_MODE_TERMINAL: distilled_mode_logical_count = 3'd2;
      DIST_MODE_SYNC:     distilled_mode_logical_count = 3'd6;
      DIST_MODE_ONE_IDLE: distilled_mode_logical_count = 3'd3;
      default:            distilled_mode_logical_count = '0;
    endcase
  endfunction

  function automatic dma_binding_t distilled_single_dma(input logic cluster);
    distilled_single_dma = cluster ? DMA_XDMA : DMA_IDMA;
  endfunction

endpackage
