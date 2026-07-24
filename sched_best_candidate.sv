// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — best candidate reducer
//
// One-lane version: keep only the compact identity of the best candidate seen
// in the current round.  The full plan/snap is intentionally not stored here:
// moe_scheduler_core replays best_token_o once after enumeration and recomputes
// the winning plan/timeline through the same candidate evaluator.

import sched_pkg::*;
import sched_candidate_pkg::*;

module sched_best_candidate (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  input  logic                         clear_i,

  input  logic                         candidate_valid_i,
  input  wire cand_token_t             candidate_token_i,
  input  wire score_key_t              candidate_score_i,

  output cand_token_t                  best_token_o,
  output logic [1:0]                   best_remove_count_o,
  output logic [3:0]                   best_remove_slot_mask_o
);

  cand_token_t          best_token_q;
  score_key_t           best_score_q;
  logic                 candidate_replaces_best;
  logic [3:0]           best_remove_mask;

  // cand_better 对应 C 里的 cand_better() 比较规则。
  //
  // 比较优先级从高到低：
  //   1. cost      越小越好：continuation_cost 或 makespan-only 分数
  //   2. remaining_count  越小越好：本轮 commit 后剩余 expert 更少
  //   3. current_makespan 越小越好：当前两个 cluster 的 makespan 更小
  //   4. token.id  越小越好：fixed policy 下的稳定 tie-break
  //
  // 分层比较比拼成一个超宽 key 更适合 timing：高优先级字段不相等时，
  // 不需要让低优先级字段参与最终选择。这样代码表达了真实数据依赖，
  // 综合器也更容易做分层比较/局部优化。
  function automatic logic cand_score_better(
    input score_key_t           cand_s,
    input cand_token_t          cand_token,
    input score_key_t           best_s,
    input cand_token_t          best_token
  );
    begin
      if (cand_s.cost != best_s.cost) begin
        cand_score_better = (cand_s.cost < best_s.cost);
      end else if (cand_s.remaining_count != best_s.remaining_count) begin
        cand_score_better = (cand_s.remaining_count < best_s.remaining_count);
      end else if (cand_s.current_makespan != best_s.current_makespan) begin
        cand_score_better = (cand_s.current_makespan < best_s.current_makespan);
      end else begin
        cand_score_better = (cand_token.id < best_token.id);
      end
    end
  endfunction

  // 每评估完一个 candidate，evaluator 会在结果有效的 cycle 拉高 cand_valid_i。
  // candidate_replaces_best=1 表示当前 candidate 比 best_*_q 中缓存的 best 更好；
  // 下一拍 always_ff 会用当前 candidate 覆盖 best cache。
  assign candidate_replaces_best = candidate_valid_i &&
                                   (!best_token_q.valid ||
                                    cand_score_better(candidate_score_i,
                                                      candidate_token_i,
                                                      best_score_q, best_token_q));

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      best_token_q        <= '0;
      best_score_q        <= '0;
    end else if (clear_i) begin
      // 每个 scheduling round 开始时清空 best cache。
      // 第一条合法候选一定会被接受。
      best_token_q        <= '0;
      best_score_q        <= '0;
    end else if (candidate_replaces_best) begin
      // 当前候选胜出：只缓存 token 和 score。
      // remove_count/mask 是 token 的纯函数，在输出端组合重算，避免派生 FF。
      // full plan/snap 在 round 结束后通过 replay(best_token_q) 重新计算。
      best_token_q        <= candidate_token_i;
      best_score_q        <= candidate_score_i;
    end
  end

  // Remove mask is the single decoded fact.  Every legal two-expert mask sets
  // bit 1 or bit 2; deriving count from that mask avoids a second mode/id
  // decoder on the replay/commit control path.
  assign best_remove_mask = cand_remove_mask(best_token_q);
  assign best_token_o = best_token_q;
  assign best_remove_slot_mask_o = best_token_q.valid ? best_remove_mask : 4'b0000;
  assign best_remove_count_o = !best_token_q.valid ? 2'd0 :
                               cand_remove_count_from_mask(best_remove_mask);

endmodule
