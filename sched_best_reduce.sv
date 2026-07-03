// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — best candidate reducer
//
// One-lane version: keep only the compact identity of the best candidate seen
// in the current round.  The full plan/snap is intentionally not stored here:
// schedule_core replays best_id_o once after enumeration and recomputes the
// winning plan/snap through the same eval lane.

import sched_pkg::*;

module sched_best_reduce (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  input  logic                         clear_i,

  input  logic                         cand_valid_i,
  input  logic [CAND_ID_W-1:0]         cand_id_i,
  input  score_key_t                   cand_score_i,
  input  logic [3:0]                   cand_remove_slot_mask_i,

  output logic                         best_valid_o,
  output logic [CAND_ID_W-1:0]         best_id_o,
  output score_key_t                   best_score_o,
  output logic [1:0]                   best_remove_count_o,
  output logic [3:0]                   best_remove_slot_mask_o,
  output logic                         accepted_o
);

  logic                 best_valid_q;
  logic [CAND_ID_W-1:0] best_id_q;
  score_key_t           best_score_q;
  logic [1:0]           best_remove_count_q;
  logic [3:0]           best_remove_slot_mask_q;

  logic [1:0]      cand_remove_count;

  // cand_better 对应 C 里的 cand_better() 比较规则。
  //
  // 比较优先级从高到低：
  //   1. cost      越小越好：continuation_cost 或 makespan-only 分数
  //   2. rem_len   越小越好：本轮 commit 后剩余 expert 更少
  //   3. snap_max  越小越好：当前两个 cluster 的 makespan 更小
  //   4. snap_min  越大越好：makespan 相同则偏向更均衡/更晚的较早 cluster
  //   5. cand_id   越小越好：完全相等时保留更早枚举到的候选，匹配 C 的稳定性
  //
  // 分层比较比拼成一个超宽 key 更适合 timing：高优先级字段不相等时，
  // 不需要让低优先级字段参与最终选择。这样代码表达了真实数据依赖，
  // 综合器也更容易做分层比较/局部优化。
  function automatic logic cand_score_better(
    input score_key_t           cand_s,
    input logic [CAND_ID_W-1:0] cand_id,
    input score_key_t           best_s,
    input logic [CAND_ID_W-1:0] best_id
  );
    begin
      if (cand_s.cost != best_s.cost) begin
        cand_score_better = (cand_s.cost < best_s.cost);
      end else if (cand_s.rem_len != best_s.rem_len) begin
        cand_score_better = (cand_s.rem_len < best_s.rem_len);
      end else if (cand_s.snap_max != best_s.snap_max) begin
        cand_score_better = (cand_s.snap_max < best_s.snap_max);
      end else if (cand_s.snap_min != best_s.snap_min) begin
        cand_score_better = (cand_s.snap_min > best_s.snap_min);
      end else begin
        cand_score_better = (cand_id < best_id);
      end
    end
  endfunction

  always_comb begin
    // 4-bit popcount 写成两级 pair-sum，而不是 for-loop 线性累加链。
    // remove_count 只用于 metadata；真正 compact 仍使用 4-bit slot mask。
    cand_remove_count = ({1'b0, cand_remove_slot_mask_i[0]} +
                         {1'b0, cand_remove_slot_mask_i[1]}) +
                        ({1'b0, cand_remove_slot_mask_i[2]} +
                         {1'b0, cand_remove_slot_mask_i[3]});
  end

  // 每 eval 完一个 candidate，eval_lane 会在结果有效的那个 cycle 拉高 cand_valid_i。
  // accepted_o=1 表示当前 candidate 比 best_*_q 中缓存的 best 更好；
  // 下一拍 always_ff 会用当前 candidate 覆盖 best cache。
  assign accepted_o = cand_valid_i &&
                      (!best_valid_q ||
                       cand_score_better(cand_score_i, cand_id_i,
                                         best_score_q, best_id_q));

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      best_valid_q        <= 1'b0;
      best_id_q           <= '0;
      best_score_q        <= '0;
      best_remove_count_q <= '0;
      best_remove_slot_mask_q <= '0;
    end else if (clear_i) begin
      // 每个 scheduling round 开始时清空 best cache。
      // 第一条合法候选一定会被接受。
      best_valid_q        <= 1'b0;
      best_id_q           <= '0;
      best_score_q        <= '0;
      best_remove_count_q <= '0;
      best_remove_slot_mask_q <= '0;
    end else if (accepted_o) begin
      // 当前候选胜出：只缓存 candidate_id、score 和紧凑 remove 下标。
      // full plan/snap 在 round 结束后通过 replay(best_id_q) 重新计算。
      best_valid_q        <= 1'b1;
      best_id_q           <= cand_id_i;
      best_score_q        <= cand_score_i;
      best_remove_count_q <= cand_remove_count;
      best_remove_slot_mask_q <= cand_remove_slot_mask_i;
    end
  end

  assign best_valid_o        = best_valid_q;
  assign best_id_o           = best_id_q;
  assign best_score_o        = best_score_q;
  assign best_remove_count_o = best_remove_count_q;
  assign best_remove_slot_mask_o = best_remove_slot_mask_q;

endmodule
