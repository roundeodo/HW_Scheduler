// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — candidate token generator
//
// 本模块只负责按当前 round 状态顺序枚举有效 candidate token：
//   token = {valid, mode, id}
// 不在这里构造 task payload、cache hit、shape、rem-after 或 remove mask。
// 这些字段由 eval_lane 在 EV_LATCH 入口按 token 解码，避免 generator 输出宽
// payload 扇出到 core/eval/best/replay。

import sched_pkg::*;
import sched_candidate_pkg::*;

module sched_candidate_generator (
  input  logic                 clk_i,
  input  logic                 rst_ni,

  input  logic                 start_i,
  input  logic                 advance_i,
  output logic                 busy_o,
  output logic                 done_o,

  input  time_t                c2_task_end_i,
  input  time_t                c3_task_end_i,
  input  early_start_ctx_t     early_i,
  input  head_ctx_t [3:0]      head_i,
  input  logic [NR_W-1:0]      active_count_i,

  output cand_token_t          cand_o,
  output logic                 cand_valid_o
);

  typedef enum logic [1:0] {ST_IDLE, ST_SEARCH, ST_EMIT, ST_DONE} state_t;

  state_t st_q, st_d;
  cand_mode_t mode_q, mode_d;
  logic [CAND_ID_W-1:0] idx_q, idx_d;
  logic [CAND_ID_W-1:0] last_idx;

  logic both_idle;
  logic idle_is_c3;
  time_t idle_t;
  logic [1:0] ntpts;

  assign both_idle = (c2_task_end_i == c3_task_end_i);
  assign idle_is_c3 = (c3_task_end_i < c2_task_end_i);
  assign idle_t = idle_is_c3 ? c3_task_end_i : c2_task_end_i;
  assign ntpts = 2'(1 + early_i.count);

  function automatic logic candidate_id_possible(
    input cand_mode_t mode,
    input logic [CAND_ID_W-1:0] id
  );
    logic [1:0] early_tpt_idx;
    ntok_t cut;
    begin
      candidate_id_possible = 1'b0;
      early_tpt_idx = '0;
      cut = '0;

      unique case (mode)
        CAND_MODE_SINGLE: begin
          if (id < CAND_ID_W'(SINGLE_SOLO_COUNT)) begin
            candidate_id_possible = head_i[0].valid;
          end else if (id == CAND_ID_W'(SINGLE_SPLIT_ID)) begin
            candidate_id_possible = head_i[0].valid &&
                                    cand_single_split_valid(head_i[0].ntok);
          end else if (id <= CAND_ID_W'(SINGLE_LAST_ID)) begin
            early_tpt_idx = 2'(id - CAND_ID_W'(SINGLE_EARLY0_ID)) + 2'd1;
            candidate_id_possible = head_i[0].valid && !both_idle &&
                                    (early_tpt_idx < ntpts);
          end
        end

        CAND_MODE_BOTH: begin
          unique case (id)
            CAND_ID_W'(0): begin
              candidate_id_possible = (active_count_i >= NR_W'(2)) &&
                                      head_i[0].valid && head_i[1].valid;
            end
            CAND_ID_W'(1): begin
              candidate_id_possible = (active_count_i >= NR_W'(3)) &&
                                      head_i[1].valid && head_i[2].valid;
            end
            CAND_ID_W'(2): begin
              candidate_id_possible = (active_count_i >= NR_W'(4)) &&
                                      head_i[2].valid && head_i[3].valid;
            end
            CAND_ID_W'(3), CAND_ID_W'(4): begin
              cut = cand_both_split_cut(head_i[0].ntok, id);
              candidate_id_possible = head_i[0].valid &&
                                      (cut > '0) && (cut < head_i[0].ntok) &&
                                      cand_both_split_valid(head_i[0].ntok, id);
            end
            default: candidate_id_possible = 1'b0;
          endcase
        end

        CAND_MODE_NOT_BOTH: begin
          candidate_id_possible = head_i[0].valid && (id < CAND_ID_W'(ntpts));
        end

        default: candidate_id_possible = 1'b0;
      endcase
    end
  endfunction

  assign last_idx = cand_mode_last_id(mode_q);

  always_comb begin
    st_d = st_q;
    mode_d = mode_q;
    idx_d = idx_q;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          if (active_count_i == NR_W'(0)) begin
            st_d = ST_DONE;
          end else begin
            if (active_count_i == NR_W'(1)) begin
              mode_d = CAND_MODE_SINGLE;
            end else if (both_idle) begin
              mode_d = CAND_MODE_BOTH;
            end else begin
              mode_d = CAND_MODE_NOT_BOTH;
            end
            idx_d = '0;
            st_d = ST_SEARCH;
          end
        end
      end

      ST_SEARCH: begin
        if (idx_q > last_idx) begin
          st_d = ST_DONE;
        end else if (candidate_id_possible(mode_q, idx_q)) begin
          st_d = ST_EMIT;
        end else begin
          idx_d = idx_q + CAND_ID_W'(1);
        end
      end

      ST_EMIT: begin
        if (advance_i) begin
          idx_d = idx_q + CAND_ID_W'(1);
          st_d = ST_SEARCH;
        end
      end

      ST_DONE: begin
        st_d = ST_IDLE;
      end

      default: begin
        st_d = ST_IDLE;
      end
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q <= ST_IDLE;
      mode_q <= CAND_MODE_SINGLE;
      idx_q <= '0;
    end else begin
      st_q <= st_d;
      mode_q <= mode_d;
      idx_q <= idx_d;
    end
  end

  always_comb begin
    cand_o = '0;
    cand_o.valid = (st_q == ST_EMIT);
    cand_o.mode  = mode_q;
    cand_o.id    = idx_q;
  end

  assign cand_valid_o = cand_o.valid;
  assign busy_o = (st_q == ST_EMIT);
  assign done_o = (st_q == ST_DONE);

endmodule
