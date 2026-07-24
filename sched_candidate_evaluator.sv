// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — 1-lane candidate evaluator（Tick 域版本）
//
// 一个 lane 每次评估一个 candidate token：
//   candidate_identity_q -> shape_q + evaluator_scratch_q(request)
//               -> shared task timeline(A) -> pre_s2pf_task_a_q
//               -> shared task timeline(B) -> pre_s2pf_task_b_q
//              -> fixed S2PF policy -> BW check -> continuation score
//
// 本模块是带 start/done 握手的单候选 evaluator。start_i 到来后先把
// candidate token + round context 解码成组合 view，只锁存 identity/shape 和
// 一组复用的 timeline operator request，再经过 EV_MK_A/EV_MK_B/S2PF/SCORE。
// generator 不再输出完整候选 payload。
// continuation scorer 复用一套加法/比较资源，顺序放置 child top4。

import sched_pkg::*;
import sched_candidate_pkg::*;

module sched_candidate_evaluator (
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic               clear_i,
  input  logic               start_i,
  output logic               done_o,

  input  wire cand_token_t   cand_i,

  // ── 候选控制 ─────────────────────────────────────────────────────────────
  input  wire head_ctx_t [5:0] head_i,
  input  logic [NR_W-1:0]    active_count_i,
  input  logic [T_W-1:0]     total_parallel_work_i,
  input  logic [T_W-1:0]     total_serial_work_i,
  input  wire early_start_ctx_t early_i,

  // ── 当前 C2/C3 base snap；某侧没有新 task 时直接沿用 base snap ─────────
  input  wire snap_timeline_t base_timeline_a_i,
  input  wire snap_timeline_t base_timeline_b_i,
  input  wire snap_cache_t    base_cache_a_i,
  input  wire snap_cache_t    base_cache_b_i,

  output logic               bw_start_o,
  output snap_bw_view_t      bw_snap_a_o,
  output snap_bw_view_t      bw_snap_b_o,
  input  logic               bw_done_i,
  input  logic               bw_ok_i,

  // ── 评估结果 ─────────────────────────────────────────────────────────────
  output logic               eval_valid_o,
  output score_key_t         score_key_o,
  output time_t              removed_parallel_work_o,
  output time_t              removed_serial_work_o,
  output winner_plan_t       winner_plan_o,
  output snap_timeline_t     snap_timeline_a_o,
  output snap_timeline_t     snap_timeline_b_o
);

  // ── shared task-timeline scalar wires ──────────────────────────────────────
  time_t build_task_end, build_dma1_end;
  time_t build_s2_end, build_dma3_end;

  typedef struct packed {
    time_t                task_end;
    time_t                dma1_end;
    time_t                s2_end;
    time_t                dma3_end;
  } task_timing_t;

  task_timing_t pre_s2pf_task_a_q, pre_s2pf_task_a_d;
  task_timing_t pre_s2pf_task_b_q, pre_s2pf_task_b_d;

  // One physical timeline operator serves A then B.  This compact request
  // register is the operator input boundary: it cuts token/head/cache decode
  // away from the endpoint arithmetic without storing two decoded candidates.
  typedef struct packed {
    time_t  start;
    ntok_t  ntok;
    logic   skip_s1;
    logic   skip_s3;
  } timeline_request_t;
  localparam int unsigned REMOVED_WORK_W = NTOK_W + 2;
  localparam int unsigned SCRATCH_PADDING_W = T_W - NTOK_W - 2;
  typedef struct packed {
    logic [SCRATCH_PADDING_W-1:0] request_phase_bits;
    logic [REMOVED_WORK_W-1:0]    parallel_work;
    logic [REMOVED_WORK_W-1:0]    serial_work;
  } removed_work_result_t;
  typedef union packed {
    timeline_request_t   timeline_request;
    removed_work_result_t removed_work;
  } evaluator_scratch_t;
  evaluator_scratch_t evaluator_scratch_q, evaluator_scratch_d;

  snap_timeline_t raw_timeline_a;
  snap_timeline_t raw_timeline_b;
  snap_timeline_t post_s2pf_a_timeline;
  snap_timeline_t post_s2pf_b_timeline;
  logic [T_W-1:0] continuation_cost;
  logic s2pf_start;
  logic s2pf_done;
  logic s2pf_bw_start;
  snap_bw_view_t  s2pf_bw_timeline_a;
  snap_bw_view_t  s2pf_bw_timeline_b;
  s2pf_patch_t s2pf_patch;
  logic score_start;
  logic score_done;
  logic candidate_feasible;
  logic direct_makespan_candidate;
  shape_t pair_shape_s1_a;
  shape_t pair_shape_s3_a;
  shape_t pair_shape_s1_b;
  shape_t pair_shape_s3_b;
  logic   use_pair_shape_select;
  ntok_t  pair_ntok_a;
  ntok_t  pair_ntok_b;
  logic   pair_s1_cache_hit_a;
  logic   pair_full_cache_hit_a;
  logic   pair_s1_cache_hit_b;
  logic   pair_full_cache_hit_b;
  shape_t shape_s1_a_q, shape_s1_a_d;
  shape_t shape_s3_a_q, shape_s3_a_d;
  shape_t shape_s1_b_q, shape_s1_b_d;
  shape_t shape_s3_b_q, shape_s3_b_d;
  shape_t selected_shape_s1_a;
  shape_t selected_shape_s3_a;
  shape_t selected_shape_s1_b;
  shape_t selected_shape_s3_b;
  shape_t timeline_shape_s1;
  shape_t timeline_shape_s3;

  // Exactly eight states; three bits are sufficient.
  typedef enum logic [2:0] {
    EV_IDLE,
    EV_LATCH,
    EV_MK_A,
    EV_MK_B,
    EV_S2PF_START,
    EV_WAIT_S2PF,
    EV_WAIT_SCORE,
    EV_DONE
  } eval_state_t;

  eval_state_t ev_st_q, ev_st_d;

  // decoded_candidate 是 candidate token + round context 的组合展开。
  // evaluator 只锁存 token；start/eid/ntok/tok_start 均由稳定的 round
  // context 重建，避免保存第二份 decoded candidate。
  typedef struct packed {
    logic                  side_a_valid;
    logic                  side_b_valid;
    logic [T_W-1:0]        start_a;
    logic [T_W-1:0]        start_b;
    logic [EID_RAW_W-1:0]  eid_a;
    logic [EID_RAW_W-1:0]  eid_b;
    logic [NTOK_W-1:0]     ntok_a;
    logic [NTOK_W-1:0]     ntok_b;
    logic [NTOK_W-1:0]     tok_start_a;
    logic [NTOK_W-1:0]     tok_start_b;

    logic                  s1_cache_hit_a;
    logic                  full_cache_hit_a;
    logic                  s1_cache_hit_b;
    logic                  full_cache_hit_b;
  } decoded_candidate_t;

  // start_i is the valid handshake, so the sampled evaluator identity needs
  // only mode/id.  The generator's valid bit is not duplicated as state.
  typedef struct packed {
    cand_mode_t           mode;
    logic [CAND_ID_W-1:0] id;
  } candidate_identity_t;
  candidate_identity_t candidate_identity_q, candidate_identity_d;
  cand_token_t candidate_token;
  decoded_candidate_t decoded_candidate;
  typedef struct packed {
    logic   force_a;
    shape_t shape_s1_a;
    shape_t shape_s3_a;
    logic   force_b;
    shape_t shape_s1_b;
    shape_t shape_s3_b;
  } shape_override_t;
  shape_override_t shape_override;

  logic side_b_is_idle;
  head_ctx_t removed_head_first;
  head_ctx_t removed_head_second;
  time_t removed_first_parallel_work;
  time_t removed_second_parallel_work;
  time_t removed_first_serial_work;
  time_t removed_second_serial_work;
  time_t removed_parallel_work;
  time_t removed_serial_work;
  time_t candidate_makespan;

  assign side_b_is_idle = (base_timeline_b_i.task_end < base_timeline_a_i.task_end);
  // The fixed policy removes at most two entries.  Select those entries first,
  // then instantiate work arithmetic only for the selected pair.  This replaces
  // four parallel+serial work decoders that previously ran for every head slot.
  assign removed_first_parallel_work = removed_head_first.valid ?
      time_t'(parallel_work_ticks(removed_head_first.ntok)) : '0;
  assign removed_second_parallel_work = removed_head_second.valid ?
      time_t'(parallel_work_ticks(removed_head_second.ntok)) : '0;
  assign removed_first_serial_work = removed_head_first.valid ?
      time_t'(serial_work_ticks(removed_head_first.ntok)) : '0;
  assign removed_second_serial_work = removed_head_second.valid ?
      time_t'(serial_work_ticks(removed_head_second.ntok)) : '0;
  assign removed_parallel_work = removed_first_parallel_work +
                                 removed_second_parallel_work;
  assign removed_serial_work = removed_first_serial_work +
                               removed_second_serial_work;
  assign removed_parallel_work_o =
      time_t'(evaluator_scratch_q.removed_work.parallel_work);
  assign removed_serial_work_o =
      time_t'(evaluator_scratch_q.removed_work.serial_work);

  function automatic removed_work_result_t pack_removed_work_result(
    input time_t parallel_work,
    input time_t serial_work
  );
    removed_work_result_t result;
    begin
      result = '0;
      result.parallel_work = parallel_work[REMOVED_WORK_W-1:0];
      result.serial_work   = serial_work[REMOVED_WORK_W-1:0];
      pack_removed_work_result = result;
    end
  endfunction

  function automatic snap_timeline_t task_to_snap(
    input task_timing_t   task_timing,
    input logic           valid,
    input time_t          task_start,
    input shape_t         shape_s1,
    input shape_t         shape_s3,
    input logic           cluster,
    input logic           skip_s1,
    input logic           skip_s3
  );
    snap_timeline_t timeline;
    begin
      timeline = '0;
      timeline.valid      = valid;
      timeline.task_start = task_start;
      timeline.task_end   = task_timing.task_end;
      timeline.dma1_end   = task_timing.dma1_end;
      timeline.s2_end     = task_timing.s2_end;
      timeline.dma3_end   = task_timing.dma3_end;
      // 每个活动 DMA 窗口携带具体物理资源，而不是 64/128 抽象档位。
      // 单 lane 的 cluster 固定绑定由 shape_dma_binding() 统一实现。
      timeline.dma_s1     = skip_s1 ? DMA_NONE :
                            shape_dma_binding(shape_s1, cluster);
      timeline.dma_s3     = skip_s3 ? DMA_NONE :
                            shape_dma_binding(shape_s3, cluster);
      task_to_snap = timeline;
    end
  endfunction

  function automatic s2pf_policy_t candidate_s2pf_policy(input cand_token_t token);
    unique case (token.mode)
      CAND_MODE_LAST_EXPERT: begin
        candidate_s2pf_policy = (token.id == CAND_ID_W'(LAST_EXPERT_SPLIT_ID)) ?
                            S2PF_SPLIT : S2PF_DISABLED;
      end
      CAND_MODE_BOTH_IDLE: begin
        candidate_s2pf_policy = (token.id <= CAND_ID_W'(2)) ?
                            S2PF_PAIR : S2PF_SPLIT;
      end
      CAND_MODE_ONE_IDLE: begin
        candidate_s2pf_policy = cand_one_idle_is_adaptive(token) ?
                            S2PF_DISABLED : S2PF_ACTIVE_SIDE;
      end
      default:            candidate_s2pf_policy = s2pf_policy_t'('x);
    endcase
  endfunction

  logic [3:0]          candidate_remove_mask;
  logic [NR_W-1:0]     remaining_count_after;
  ntok_t               first_remaining_ntok;
  time_t               remaining_parallel_work;
  time_t               remaining_serial_work;
  time_t               largest_parallel_work;
  remaining_work_t [3:0]   remaining_work;

  function automatic remaining_work_t to_score_work(input head_ctx_t head);
    remaining_work_t work;
    begin
      work.valid = head.valid;
      work.ntok  = head.ntok;
      to_score_work = work;
    end
  endfunction

  always_comb begin
    candidate_remove_mask = cand_remove_mask(candidate_token);
    remaining_count_after = active_count_i;
    first_remaining_ntok = '0;
    largest_parallel_work = '0;
    removed_head_first = '0;
    removed_head_second = '0;
    remaining_work = '{default:'0};

    // Only four remove masks are reachable from the fixed candidate policy.
    // Constant slot maps avoid a generic rank/compact network on the score path.
    unique case (candidate_remove_mask)
      4'b0001: begin
        removed_head_first = head_i[0];
        remaining_count_after = active_count_i - NR_W'(1);
        remaining_work[0] = to_score_work(head_i[1]);
        remaining_work[1] = to_score_work(head_i[2]);
        remaining_work[2] = to_score_work(head_i[3]);
        remaining_work[3] = to_score_work(head_i[4]);
      end
      4'b0011: begin
        removed_head_first = head_i[0];
        removed_head_second = head_i[1];
        remaining_count_after = active_count_i - NR_W'(2);
        remaining_work[0] = to_score_work(head_i[2]);
        remaining_work[1] = to_score_work(head_i[3]);
        remaining_work[2] = to_score_work(head_i[4]);
        remaining_work[3] = to_score_work(head_i[5]);
      end
      4'b0110: begin
        removed_head_first = head_i[1];
        removed_head_second = head_i[2];
        remaining_count_after = active_count_i - NR_W'(2);
        remaining_work[0] = to_score_work(head_i[0]);
        remaining_work[1] = to_score_work(head_i[3]);
        remaining_work[2] = to_score_work(head_i[4]);
        remaining_work[3] = to_score_work(head_i[5]);
      end
      4'b1100: begin
        removed_head_first = head_i[2];
        removed_head_second = head_i[3];
        remaining_count_after = active_count_i - NR_W'(2);
        remaining_work[0] = to_score_work(head_i[0]);
        remaining_work[1] = to_score_work(head_i[1]);
        remaining_work[2] = to_score_work(head_i[4]);
        remaining_work[3] = to_score_work(head_i[5]);
      end
      default: begin
      end
    endcase

    // Entries beyond child remaining are invalidated explicitly; stale tail FF
    // contents therefore never enter the sequential LPT engine.
    if (remaining_count_after < NR_W'(1)) remaining_work[0] = '0;
    if (remaining_count_after < NR_W'(2)) remaining_work[1] = '0;
    if (remaining_count_after < NR_W'(3)) remaining_work[2] = '0;
    if (remaining_count_after < NR_W'(4)) remaining_work[3] = '0;

    first_remaining_ntok = remaining_work[0].ntok;
    largest_parallel_work = remaining_work[0].valid ?
        time_t'(parallel_work_ticks(remaining_work[0].ntok)) : '0;
    remaining_parallel_work = total_parallel_work_i - removed_parallel_work;
    remaining_serial_work = total_serial_work_i - removed_serial_work;
  end

  function automatic decoded_candidate_t decode_token_fn(
    input cand_token_t       token,
    input head_ctx_t [5:0]   head,
    input snap_timeline_t    base_a,
    input snap_timeline_t    base_b,
    input snap_cache_t       cache_a,
    input snap_cache_t       cache_b,
    input early_start_ctx_t  early,
    input logic              side_b_idle
  );
    decoded_candidate_t decoded;
    logic       solo_to_c3;
    logic [1:0] release_point_index;
    ntok_t      cut;
    logic [1:0] head_index_a;
    logic [1:0] head_index_b;
    time_t      both_available_time;
    time_t      idle_side_time;
    time_t      release_time [3];
    begin
      decoded = '0;
      both_available_time = (base_a.task_end > base_b.task_end) ?
                            base_a.task_end : base_b.task_end;
      idle_side_time = side_b_idle ? base_b.task_end : base_a.task_end;
      release_time[0] = idle_side_time;
      release_time[1] = early.first_release;
      release_time[2] = early.second_release;

      unique case (token.mode)
        CAND_MODE_LAST_EXPERT: begin
          solo_to_c3 = (token.id >= CAND_ID_W'(LAST_EXPERT_SOLO_SHAPES)) &&
                       (token.id < CAND_ID_W'(LAST_EXPERT_SOLO_COUNT));
          release_point_index =
              2'(token.id - CAND_ID_W'(LAST_EXPERT_EARLY0_ID)) + 2'd1;
          cut = '0;

          if (token.id < CAND_ID_W'(LAST_EXPERT_SOLO_COUNT)) begin
            if (!solo_to_c3) begin
              decoded.side_a_valid = 1'b1;
              decoded.start_a = base_a.task_end;
              decoded.eid_a = head[0].eid;
              decoded.ntok_a = head[0].ntok;
            end else begin
              decoded.side_b_valid = 1'b1;
              decoded.start_b = base_b.task_end;
              decoded.eid_b = head[0].eid;
              decoded.ntok_b = head[0].ntok;
            end
          end else if (token.id == CAND_ID_W'(LAST_EXPERT_SPLIT_ID)) begin
            cut = cand_last_expert_split_cut(head[0].ntok);
            decoded.side_a_valid = 1'b1;
            decoded.side_b_valid = 1'b1;
            decoded.start_a = both_available_time;
            decoded.start_b = both_available_time;
            decoded.eid_a = head[0].eid;
            decoded.eid_b = head[0].eid;
            decoded.ntok_a = cut;
            decoded.ntok_b = head[0].ntok - cut;
            decoded.tok_start_b = cut;
          end else begin
            if (!side_b_idle) begin
              decoded.side_a_valid = 1'b1;
              decoded.start_a = release_time[release_point_index];
              decoded.eid_a = head[0].eid;
              decoded.ntok_a = head[0].ntok;
            end else begin
              decoded.side_b_valid = 1'b1;
              decoded.start_b = release_time[release_point_index];
              decoded.eid_b = head[0].eid;
              decoded.ntok_b = head[0].ntok;
            end
          end
        end

        CAND_MODE_BOTH_IDLE: begin
          decoded.start_a = both_available_time;
          decoded.start_b = both_available_time;
          if (token.id <= CAND_ID_W'(2)) begin
            unique case (token.id)
              CAND_ID_W'(0): begin
                head_index_a = 2'd0;
                head_index_b = 2'd1;
              end
              CAND_ID_W'(1): begin
                head_index_a = 2'd1;
                head_index_b = 2'd2;
              end
              CAND_ID_W'(2): begin
                head_index_a = 2'd2;
                head_index_b = 2'd3;
              end
              default: begin
                head_index_a = 'x;
                head_index_b = 'x;
              end
            endcase
            decoded.side_a_valid = 1'b1;
            decoded.side_b_valid = 1'b1;
            decoded.eid_a = head[head_index_a].eid;
            decoded.eid_b = head[head_index_b].eid;
            decoded.ntok_a = head[head_index_a].ntok;
            decoded.ntok_b = head[head_index_b].ntok;
          end else begin
            cut = cand_both_idle_split_cut(head[0].ntok, token.id);
            decoded.side_a_valid = 1'b1;
            decoded.side_b_valid = 1'b1;
            decoded.eid_a = head[0].eid;
            decoded.eid_b = head[0].eid;
            decoded.ntok_a = cut;
            decoded.ntok_b = head[0].ntok - cut;
            decoded.tok_start_b = cut;
          end
        end

        CAND_MODE_ONE_IDLE: begin
          release_point_index = cand_one_idle_release_index(token);
          if (!side_b_idle) begin
            decoded.side_a_valid = 1'b1;
            decoded.start_a = release_time[release_point_index];
            decoded.eid_a = head[0].eid;
            decoded.ntok_a = head[0].ntok;
          end else begin
            decoded.side_b_valid = 1'b1;
            decoded.start_b = release_time[release_point_index];
            decoded.eid_b = head[0].eid;
            decoded.ntok_b = head[0].ntok;
          end
        end

        default: begin
          decoded = 'x;
        end
      endcase

      decoded.s1_cache_hit_a = decoded.side_a_valid &&
                 swiglu_hit_t(decoded.eid_a, cache_a.pf_eid,
                              cache_a.pf_end, decoded.start_a);
      decoded.full_cache_hit_a = decoded.s1_cache_hit_a && cache_a.pf_full;
      decoded.s1_cache_hit_b = decoded.side_b_valid &&
                 swiglu_hit_t(decoded.eid_b, cache_b.pf_eid,
                              cache_b.pf_end, decoded.start_b);
      decoded.full_cache_hit_b = decoded.s1_cache_hit_b && cache_b.pf_full;
      decode_token_fn = decoded;
    end
  endfunction

  function automatic shape_override_t decode_shape_override_fn(
    input cand_token_t token,
    input logic        idle_is_c3,
    input ntok_t       head0_ntok
  );
    shape_override_t override;
    logic [3:0] solo_shape_index;
    logic       solo_to_c3;
    logic       adaptive_one_idle;
    shape_t     adaptive_shape_s1;
    shape_t     adaptive_shape_s3;
    begin
      override = '0;
      adaptive_one_idle = cand_one_idle_is_adaptive(token);
      adaptive_shape_s1 = cand_one_idle_adaptive_s1(head0_ntok);
      adaptive_shape_s3 = cand_one_idle_adaptive_s3(head0_ntok);

      unique case (token.mode)
        CAND_MODE_LAST_EXPERT: begin
          solo_to_c3 = (token.id >= CAND_ID_W'(LAST_EXPERT_SOLO_SHAPES)) &&
                       (token.id < CAND_ID_W'(LAST_EXPERT_SOLO_COUNT));
          solo_shape_index = solo_to_c3 ?
                               4'(token.id - CAND_ID_W'(LAST_EXPERT_SOLO_SHAPES)) :
                               4'(token.id);
          if (token.id < CAND_ID_W'(LAST_EXPERT_SOLO_COUNT)) begin
            if (solo_to_c3) begin
              override.force_b = 1'b1;
              override.shape_s1_b = cand_last_expert_solo_s1(solo_shape_index);
              override.shape_s3_b = cand_last_expert_solo_s3(solo_shape_index);
            end else begin
              override.force_a = 1'b1;
              override.shape_s1_a = cand_last_expert_solo_s1(solo_shape_index);
              override.shape_s3_a = cand_last_expert_solo_s3(solo_shape_index);
            end
          end else if (token.id > CAND_ID_W'(LAST_EXPERT_SPLIT_ID)) begin
            if (idle_is_c3) begin
              override.force_b = 1'b1;
              override.shape_s1_b = SHAPE_C;
              override.shape_s3_b = SHAPE_C;
            end else begin
              override.force_a = 1'b1;
              override.shape_s1_a = SHAPE_C;
              override.shape_s3_a = SHAPE_C;
            end
          end
        end

        CAND_MODE_ONE_IDLE: begin
          if (idle_is_c3) begin
            override.force_b = 1'b1;
            override.shape_s1_b = adaptive_one_idle ? adaptive_shape_s1 : SHAPE_C;
            override.shape_s3_b = adaptive_one_idle ? adaptive_shape_s3 : SHAPE_C;
          end else begin
            override.force_a = 1'b1;
            override.shape_s1_a = adaptive_one_idle ? adaptive_shape_s1 : SHAPE_C;
            override.shape_s3_a = adaptive_one_idle ? adaptive_shape_s3 : SHAPE_C;
          end
        end

        // Both-idle pair/split candidates use the paired shape selector and
        // intentionally carry no forced-shape override.
        CAND_MODE_BOTH_IDLE: begin
          override = '0;
        end

        default: begin
          override = 'x;
        end
      endcase
      decode_shape_override_fn = override;
    end
  endfunction

  task automatic put_plan_slot(
    inout  winner_plan_t          p,
    input  logic                  dst,
    input  logic                  cluster,
    input  logic [EID_RAW_W-1:0]  eid,
    input  ntok_t                 ntok,
    input  ntok_t                 tok_start,
    input  shape_t                shape_s1,
    input  shape_t                shape_s3,
    input  snap_timeline_t        sn
  );
    begin
      p.token[dst].eid       = eid;
      p.token[dst].ntok      = ntok;
      p.token[dst].tok_start = tok_start;
      p.ctrl[dst].cluster    = cluster;
      p.ctrl[dst].shape_s1   = shape_s1;
      p.ctrl[dst].shape_s3   = shape_s3;
      p.ctrl[dst].skip_s1    = (sn.dma_s1 == DMA_NONE);
      p.ctrl[dst].skip_s3    = (sn.dma_s3 == DMA_NONE);
      p.ctrl[dst].has_s2pf   = sn.s2pf_valid;
    end
  endtask

  always_comb begin
    candidate_token.valid = 1'b1;
    candidate_token.mode  = candidate_identity_q.mode;
    candidate_token.id    = candidate_identity_q.id;
    decoded_candidate = decode_token_fn(candidate_token, head_i, base_timeline_a_i,
                                        base_timeline_b_i, base_cache_a_i,
                                        base_cache_b_i, early_i, side_b_is_idle);
  end

  // Pair shape selection 只对双侧有效且没有固定 shape 的 candidate 有意义。
  // forced-shape token 不驱动 pick 的 ntok/cache-hit 比较网络，减少无效切换和 fanout。
  assign use_pair_shape_select = decoded_candidate.side_a_valid &&
                                 decoded_candidate.side_b_valid &&
                                 !shape_override.force_a && !shape_override.force_b;
  assign pair_ntok_a = use_pair_shape_select ? decoded_candidate.ntok_a : '0;
  assign pair_ntok_b = use_pair_shape_select ? decoded_candidate.ntok_b : '0;
  assign pair_s1_cache_hit_a = use_pair_shape_select ?
                               decoded_candidate.s1_cache_hit_a : 1'b0;
  assign pair_full_cache_hit_a = use_pair_shape_select ?
                                 decoded_candidate.full_cache_hit_a : 1'b0;
  assign pair_s1_cache_hit_b = use_pair_shape_select ?
                               decoded_candidate.s1_cache_hit_b : 1'b0;
  assign pair_full_cache_hit_b = use_pair_shape_select ?
                                 decoded_candidate.full_cache_hit_b : 1'b0;

  // ── paired shape selector ────────────────────────────────────────────────
  sched_pair_shape_select i_pair_shape_select (
    .ntok_a_i            (pair_ntok_a),
    .ntok_b_i            (pair_ntok_b),
    .s1_cache_hit_a_i     (pair_s1_cache_hit_a),
    .full_cache_hit_a_i   (pair_full_cache_hit_a),
    .s1_cache_hit_b_i     (pair_s1_cache_hit_b),
    .full_cache_hit_b_i   (pair_full_cache_hit_b),
    .shape_s1_a_o         (pair_shape_s1_a),
    .shape_s3_a_o         (pair_shape_s3_a),
    .shape_s1_b_o         (pair_shape_s1_b),
    .shape_s3_b_o         (pair_shape_s3_b)
  );

  always_comb begin
    shape_override = decode_shape_override_fn(candidate_token, side_b_is_idle,
                                              head_i[0].ntok);
  end

  assign selected_shape_s1_a = shape_override.force_a ?
                               shape_override.shape_s1_a : pair_shape_s1_a;
  assign selected_shape_s3_a = shape_override.force_a ?
                               shape_override.shape_s3_a : pair_shape_s3_a;
  assign selected_shape_s1_b = shape_override.force_b ?
                               shape_override.shape_s1_b : pair_shape_s1_b;
  assign selected_shape_s3_b = shape_override.force_b ?
                               shape_override.shape_s3_b : pair_shape_s3_b;

  // ── shared task timeline：EV_MK_A/EV_MK_B 两拍复用同一套组合逻辑 ───────────
  assign timeline_shape_s1 = (ev_st_q == EV_MK_B) ? shape_s1_b_q : shape_s1_a_q;
  assign timeline_shape_s3 = (ev_st_q == EV_MK_B) ? shape_s3_b_q : shape_s3_a_q;
  sched_task_timeline i_task_timeline (
    .start_t_i     (evaluator_scratch_q.timeline_request.start),
    .ntok_i        (evaluator_scratch_q.timeline_request.ntok),
    .shape_s1_i    (timeline_shape_s1),
    .shape_s3_i    (timeline_shape_s3),
    .skip_s1_i     (evaluator_scratch_q.timeline_request.skip_s1),
    .skip_s3_i     (evaluator_scratch_q.timeline_request.skip_s3),
    .task_end_o    (build_task_end),
    .dma1_end_o    (build_dma1_end),
    .s2_end_o      (build_s2_end),
    .dma3_end_o    (build_dma3_end)
  );

  sched_s2pf_search i_s2pf_search (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .clear_i              (clear_i),
    .start_i              (s2pf_start),
    .done_o               (s2pf_done),
    .policy_i             (candidate_s2pf_policy(candidate_token)),
    .side_a_active_i      (decoded_candidate.side_a_valid),
    .side_b_active_i      (decoded_candidate.side_b_valid),
    .snap_a_i             (raw_timeline_a),
    .snap_b_i             (raw_timeline_b),
    .bw_start_o           (s2pf_bw_start),
    .bw_snap_a_o          (s2pf_bw_timeline_a),
    .bw_snap_b_o          (s2pf_bw_timeline_b),
    .bw_done_i            ((ev_st_q == EV_WAIT_S2PF) ? bw_done_i : 1'b0),
    .bw_ok_i              (bw_ok_i),
    .patch_o              (s2pf_patch)
  );

  // sched_s2pf_search retains its selected patch until the next start_i.
  // The evaluator never launches another search while scoring or publishing
  // the current candidate, so a duplicate accepted_patch register is unnecessary.
  assign raw_timeline_a = decoded_candidate.side_a_valid ?
                                               task_to_snap(pre_s2pf_task_a_q,
                                                            decoded_candidate.side_a_valid,
                                                            decoded_candidate.start_a,
                                                            shape_s1_a_q,
                                                            shape_s3_a_q,
                                                            1'b0,
                                                            decoded_candidate.s1_cache_hit_a,
                                                            decoded_candidate.full_cache_hit_a) :
                                               base_timeline_a_i;
  assign raw_timeline_b = decoded_candidate.side_b_valid ?
                                               task_to_snap(pre_s2pf_task_b_q,
                                                            decoded_candidate.side_b_valid,
                                                            decoded_candidate.start_b,
                                                            shape_s1_b_q,
                                                            shape_s3_b_q,
                                                            1'b1,
                                                            decoded_candidate.s1_cache_hit_b,
                                                            decoded_candidate.full_cache_hit_b) :
                                               base_timeline_b_i;
  assign post_s2pf_a_timeline = apply_s2pf_patch_timeline(
      raw_timeline_a, s2pf_patch.apply_a,
      decoded_candidate.ntok_a);
  assign post_s2pf_b_timeline = apply_s2pf_patch_timeline(
      raw_timeline_b, s2pf_patch.apply_b,
      decoded_candidate.ntok_b);

  // Generator/replay guarantees that every issued token activates at least one
  // side.  S2PF raw-trial acceptance is therefore the only feasibility fact.
  assign candidate_feasible = s2pf_patch.valid;
  // ONE_IDLE and LAST_EXPERT have no continuation search.  Decode this fact
  // once and share it between FSM control and score selection.
  assign direct_makespan_candidate =
      (candidate_token.mode != CAND_MODE_BOTH_IDLE);

  sched_continuation_score i_score (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .clear_i            (clear_i),
    .start_i            (score_start),
    .done_o             (score_done),
    .c2_task_end_i      (post_s2pf_a_timeline.task_end),
    .c3_task_end_i      (post_s2pf_b_timeline.task_end),
    .remaining_count_i      (remaining_count_after),
    .first_remaining_ntok_i (first_remaining_ntok),
    .total_parallel_work_i  (remaining_parallel_work),
    .largest_parallel_work_i(largest_parallel_work),
    .total_serial_work_i    (remaining_serial_work),
    .remaining_work_i       (remaining_work),
    .cost_o             (continuation_cost)
  );

  assign bw_start_o = s2pf_bw_start;
  assign bw_snap_a_o = s2pf_bw_timeline_a;
  assign bw_snap_b_o = s2pf_bw_timeline_b;

  always_comb begin
    ev_st_d    = ev_st_q;
    candidate_identity_d = candidate_identity_q;
    pre_s2pf_task_a_d = pre_s2pf_task_a_q;
    pre_s2pf_task_b_d = pre_s2pf_task_b_q;
    evaluator_scratch_d = evaluator_scratch_q;
    shape_s1_a_d = shape_s1_a_q;
    shape_s3_a_d = shape_s3_a_q;
    shape_s1_b_d = shape_s1_b_q;
    shape_s3_b_d = shape_s3_b_q;
    s2pf_start = 1'b0;
    score_start = 1'b0;

    unique case (ev_st_q)
      EV_IDLE: begin
        if (start_i) begin
          candidate_identity_d.mode = cand_i.mode;
          candidate_identity_d.id   = cand_i.id;
          ev_st_d = EV_LATCH;
        end
      end

      EV_LATCH: begin
        shape_s1_a_d = selected_shape_s1_a;
        shape_s3_a_d = selected_shape_s3_a;
        shape_s1_b_d = selected_shape_s1_b;
        shape_s3_b_d = selected_shape_s3_b;
        evaluator_scratch_d.timeline_request.start = decoded_candidate.start_a;
        evaluator_scratch_d.timeline_request.ntok  = decoded_candidate.ntok_a;
        evaluator_scratch_d.timeline_request.skip_s1 =
            decoded_candidate.s1_cache_hit_a;
        evaluator_scratch_d.timeline_request.skip_s3 =
            decoded_candidate.full_cache_hit_a;
        ev_st_d = EV_MK_A;
      end

      EV_MK_A: begin
        pre_s2pf_task_a_d = '0;
        if (decoded_candidate.side_a_valid) begin
          pre_s2pf_task_a_d.task_end = build_task_end;
          pre_s2pf_task_a_d.dma1_end = build_dma1_end;
          pre_s2pf_task_a_d.s2_end   = build_s2_end;
          pre_s2pf_task_a_d.dma3_end = build_dma3_end;
        end
        evaluator_scratch_d.timeline_request.start = decoded_candidate.start_b;
        evaluator_scratch_d.timeline_request.ntok  = decoded_candidate.ntok_b;
        evaluator_scratch_d.timeline_request.skip_s1 =
            decoded_candidate.s1_cache_hit_b;
        evaluator_scratch_d.timeline_request.skip_s3 =
            decoded_candidate.full_cache_hit_b;
        ev_st_d = EV_MK_B;
      end

      EV_MK_B: begin
        pre_s2pf_task_b_d = '0;
        if (decoded_candidate.side_b_valid) begin
          pre_s2pf_task_b_d.task_end = build_task_end;
          pre_s2pf_task_b_d.dma1_end = build_dma1_end;
          pre_s2pf_task_b_d.s2_end   = build_s2_end;
          pre_s2pf_task_b_d.dma3_end = build_dma3_end;
        end
        ev_st_d = EV_S2PF_START;
      end

      EV_S2PF_START: begin
        s2pf_start = 1'b1;
        ev_st_d    = EV_WAIT_S2PF;
      end

      EV_WAIT_S2PF: begin
        if (s2pf_done) begin
          if (candidate_feasible) begin
            if (direct_makespan_candidate) begin
              evaluator_scratch_d.removed_work = pack_removed_work_result(
                  removed_parallel_work, removed_serial_work);
              ev_st_d = EV_DONE;
            end else begin
              score_start = 1'b1;
              ev_st_d     = EV_WAIT_SCORE;
            end
          end else begin
            ev_st_d = EV_DONE;
          end
        end
      end

      EV_WAIT_SCORE: begin
        if (score_done) begin
          evaluator_scratch_d.removed_work = pack_removed_work_result(
              removed_parallel_work, removed_serial_work);
          ev_st_d = EV_DONE;
        end
      end

      EV_DONE: begin
        ev_st_d = EV_IDLE;
      end

    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ev_st_q <= EV_IDLE;
      candidate_identity_q <= '0;
      evaluator_scratch_q <= '0;
      pre_s2pf_task_a_q <= '0;
      pre_s2pf_task_b_q <= '0;
      shape_s1_a_q <= '0;
      shape_s3_a_q <= '0;
      shape_s1_b_q <= '0;
      shape_s3_b_q <= '0;
    end else if (clear_i) begin
      ev_st_q <= EV_IDLE;
      candidate_identity_q <= '0;
      evaluator_scratch_q <= '0;
      pre_s2pf_task_a_q <= '0;
      pre_s2pf_task_b_q <= '0;
      shape_s1_a_q <= '0;
      shape_s3_a_q <= '0;
      shape_s1_b_q <= '0;
      shape_s3_b_q <= '0;
    end else begin
      ev_st_q <= ev_st_d;
      candidate_identity_q <= candidate_identity_d;
      evaluator_scratch_q <= evaluator_scratch_d;
      pre_s2pf_task_a_q <= pre_s2pf_task_a_d;
      pre_s2pf_task_b_q <= pre_s2pf_task_b_d;
      shape_s1_a_q <= shape_s1_a_d;
      shape_s3_a_q <= shape_s3_a_d;
      shape_s1_b_q <= shape_s1_b_d;
      shape_s3_b_q <= shape_s3_b_d;
    end
  end

  assign done_o = (ev_st_q == EV_DONE);

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni && (ev_st_q == EV_LATCH)) begin
      assert (decoded_candidate.side_a_valid || decoded_candidate.side_b_valid);
    end
  end
`endif

  always_comb begin
    snap_timeline_a_o = post_s2pf_a_timeline;
    snap_timeline_b_o = post_s2pf_b_timeline;
    candidate_makespan =
        (post_s2pf_a_timeline.task_end > post_s2pf_b_timeline.task_end) ?
        post_s2pf_a_timeline.task_end : post_s2pf_b_timeline.task_end;

    eval_valid_o = (ev_st_q == EV_DONE) && candidate_feasible;

    score_key_o.cost = direct_makespan_candidate ?
                       candidate_makespan : continuation_cost;
    score_key_o.remaining_count  = remaining_count_after;
    score_key_o.current_makespan = candidate_makespan;

    winner_plan_o = '0;
    if (decoded_candidate.side_a_valid) begin
      put_plan_slot(winner_plan_o, 1'b0, 1'b0,
                    decoded_candidate.eid_a, decoded_candidate.ntok_a,
                    decoded_candidate.tok_start_a, shape_s1_a_q, shape_s3_a_q,
                    post_s2pf_a_timeline);
    end
    if (decoded_candidate.side_b_valid) begin
      put_plan_slot(winner_plan_o, decoded_candidate.side_a_valid, 1'b1,
                    decoded_candidate.eid_b, decoded_candidate.ntok_b,
                    decoded_candidate.tok_start_b, shape_s1_b_q, shape_s3_b_q,
                    post_s2pf_b_timeline);
    end
    winner_plan_o.task_valid =
        (decoded_candidate.side_a_valid && decoded_candidate.side_b_valid) ? 2'b11 :
        (decoded_candidate.side_a_valid || decoded_candidate.side_b_valid) ? 2'b01 :
        2'b00;
  end

endmodule
