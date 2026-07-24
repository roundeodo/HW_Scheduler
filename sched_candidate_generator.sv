// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — candidate token generator
//
// 本模块只负责按当前 round 状态顺序枚举有效 candidate token：
//   token = {valid, mode, id}
// 不在这里构造 task payload、cache hit、shape、rem-after 或 remove mask。
// 这些字段由 candidate evaluator 在入口按 token 解码，避免 generator 输出宽
// payload 扇出到 core/eval/best/replay。

import sched_pkg::*;
import sched_candidate_pkg::*;

module sched_candidate_generator (
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  logic                 clear_i,

  input  logic                 start_i,
  input  logic                 advance_i,
  output logic                 done_o,

  input  logic                 both_idle_i,
  input  logic [1:0]           early_count_i,
  input  logic [3:0]           head_valid_i,
  input  ntok_t                head0_ntok_i,
  input  logic [NR_W-1:0]      active_count_i,

  output cand_token_t          cand_o
);

  typedef enum logic [1:0] {ST_IDLE, ST_SEARCH, ST_EMIT, ST_DONE} state_t;

  state_t st_q, st_d;
  logic [CAND_ID_W-1:0] candidate_id_q, candidate_id_d;
  logic [CAND_ID_W-1:0] last_candidate_id;
  cand_mode_t round_mode;

  logic [1:0] release_point_count;

  assign release_point_count = 2'(1 + early_count_i);

  function automatic logic candidate_id_is_valid(
    input cand_mode_t mode,
    input logic [CAND_ID_W-1:0] id
  );
    logic [1:0] release_point_index;
    begin
      candidate_id_is_valid = 1'b0;
      release_point_index = '0;

      unique case (mode)
        CAND_MODE_LAST_EXPERT: begin
          if (id < CAND_ID_W'(LAST_EXPERT_SOLO_COUNT)) begin
            candidate_id_is_valid = head_valid_i[0];
          end else if (id == CAND_ID_W'(LAST_EXPERT_SPLIT_ID)) begin
            candidate_id_is_valid = head_valid_i[0] &&
                                    cand_last_expert_split_valid(head0_ntok_i);
          end else if (id <= CAND_ID_W'(LAST_EXPERT_LAST_ID)) begin
            release_point_index =
                2'(id - CAND_ID_W'(LAST_EXPERT_EARLY0_ID)) + 2'd1;
            candidate_id_is_valid = head_valid_i[0] && !both_idle_i &&
                                    (release_point_index < release_point_count);
          end
        end

        CAND_MODE_BOTH_IDLE: begin
          unique case (id)
            CAND_ID_W'(0): begin
              candidate_id_is_valid = (active_count_i >= NR_W'(2)) &&
                                      head_valid_i[0] && head_valid_i[1];
            end
            CAND_ID_W'(1): begin
              candidate_id_is_valid = (active_count_i >= NR_W'(3)) &&
                                      head_valid_i[1] && head_valid_i[2];
            end
            CAND_ID_W'(2): begin
              candidate_id_is_valid = (active_count_i >= NR_W'(4)) &&
                                      head_valid_i[2] && head_valid_i[3];
            end
            CAND_ID_W'(3), CAND_ID_W'(4): begin
              candidate_id_is_valid = head_valid_i[0] &&
                                      cand_both_idle_split_valid(head0_ntok_i, id);
            end
            default: candidate_id_is_valid = 1'b0;
          endcase
        end

        CAND_MODE_ONE_IDLE: begin
          if (id <= CAND_ID_W'(ONE_IDLE_FIXED_LAST)) begin
            candidate_id_is_valid = head_valid_i[0] &&
                                    (id[1:0] < release_point_count);
          end else begin
            // Adaptive C/C is identical to the base candidate for ntok<=2.
            // Suppress it in the generator instead of carrying duplicate work
            // into timeline/BW evaluation.
            release_point_index =
                2'(id - CAND_ID_W'(ONE_IDLE_ADAPTIVE_FIRST));
            candidate_id_is_valid = head_valid_i[0] &&
                                    (head0_ntok_i >= NTOK_W'(3)) &&
                                    (release_point_index < release_point_count);
          end
        end

        default: candidate_id_is_valid = 1'b0;
      endcase
    end
  endfunction

  assign round_mode = (active_count_i == NR_W'(1)) ? CAND_MODE_LAST_EXPERT :
                      both_idle_i ? CAND_MODE_BOTH_IDLE : CAND_MODE_ONE_IDLE;
  assign last_candidate_id = cand_mode_last_id(round_mode);

  always_comb begin
    st_d = st_q;
    candidate_id_d = candidate_id_q;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          if (active_count_i == NR_W'(0)) begin
            st_d = ST_DONE;
          end else begin
            candidate_id_d = '0;
            st_d = ST_SEARCH;
          end
        end
      end

      ST_SEARCH: begin
        if (candidate_id_q > last_candidate_id) begin
          st_d = ST_DONE;
        end else if (candidate_id_is_valid(round_mode, candidate_id_q)) begin
          st_d = ST_EMIT;
        end else begin
          candidate_id_d = candidate_id_q + CAND_ID_W'(1);
        end
      end

      ST_EMIT: begin
        if (advance_i) begin
          candidate_id_d = candidate_id_q + CAND_ID_W'(1);
          st_d = ST_SEARCH;
        end
      end

      ST_DONE: begin
        st_d = ST_IDLE;
      end

    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q <= ST_IDLE;
      candidate_id_q <= '0;
    end else if (clear_i) begin
      st_q <= ST_IDLE;
      candidate_id_q <= '0;
    end else begin
      st_q <= st_d;
      candidate_id_q <= candidate_id_d;
    end
  end

  always_comb begin
    cand_o = '0;
    cand_o.valid = (st_q == ST_EMIT);
    cand_o.mode  = round_mode;
    cand_o.id    = candidate_id_q;
  end

  assign done_o = (st_q == ST_DONE);

`ifndef SYNTHESIS
  // generator 的输出契约：综合路径只发合法 token；非法 token 应在 TB/仿真中暴露。
  always_ff @(posedge clk_i) begin
    if (rst_ni && (st_q == ST_EMIT)) begin
      assert (candidate_id_is_valid(round_mode, candidate_id_q))
        else $error("sched_candidate_generator emitted invalid token: mode=%0d id=%0d",
                    round_mode, candidate_id_q);
    end
  end
`endif

endmodule
