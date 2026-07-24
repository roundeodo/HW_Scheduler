// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// Candidate token package
//
// candidate_generator 只跨模块发送 mode/id token。task payload、remove mask、
// shape policy、cache hit 和 continuation 摘要都在 evaluator 入口按 token 解码。
// 这样 generator 输出保持很窄，避免构造和传输完整候选对象。

package sched_candidate_pkg;

  import sched_pkg::*;

  typedef enum logic [1:0] {
    CAND_MODE_LAST_EXPERT   = 2'd0,
    CAND_MODE_BOTH_IDLE     = 2'd1,
    CAND_MODE_ONE_IDLE      = 2'd2
  } cand_mode_t;

  typedef struct packed {
    logic                 valid;
    cand_mode_t           mode;
    logic [CAND_ID_W-1:0] id;
  } cand_token_t;

  localparam int unsigned LAST_EXPERT_SOLO_SHAPES = 5;
  localparam int unsigned LAST_EXPERT_SOLO_COUNT  = 2 * LAST_EXPERT_SOLO_SHAPES;
  localparam int unsigned LAST_EXPERT_SPLIT_ID    = LAST_EXPERT_SOLO_COUNT;
  localparam int unsigned LAST_EXPERT_EARLY0_ID   = LAST_EXPERT_SPLIT_ID + 1;
  localparam int unsigned LAST_EXPERT_LAST_ID     = LAST_EXPERT_EARLY0_ID + 1;
  localparam int unsigned ONE_IDLE_FIXED_LAST     = 2;
  localparam int unsigned ONE_IDLE_ADAPTIVE_FIRST = 3;
  localparam int unsigned ONE_IDLE_ADAPTIVE_LAST  = 5;

  function automatic logic [CAND_ID_W-1:0] cand_mode_last_id(input cand_mode_t mode);
    unique case (mode)
      CAND_MODE_LAST_EXPERT:   cand_mode_last_id = CAND_ID_W'(LAST_EXPERT_LAST_ID);
      CAND_MODE_BOTH_IDLE:     cand_mode_last_id = CAND_ID_W'(4);
      CAND_MODE_ONE_IDLE:    cand_mode_last_id = CAND_ID_W'(ONE_IDLE_ADAPTIVE_LAST);
      default:               cand_mode_last_id = 'x;
    endcase
  endfunction

  function automatic ntok_t cand_last_expert_split_cut(input ntok_t ntok);
    cand_last_expert_split_cut = ceil_div2_ntok(ntok);
  endfunction

  function automatic logic cand_last_expert_split_valid(input ntok_t ntok);
    cand_last_expert_split_valid = (ntok >= NTOK_W'(2));
  endfunction

  function automatic shape_t cand_last_expert_solo_s1(input logic [3:0] shape_index);
    unique case (shape_index)
      4'd0:          cand_last_expert_solo_s1 = SHAPE_C; // C/C
      4'd1, 4'd2,
      4'd4:          cand_last_expert_solo_s1 = SHAPE_A; // A/A, A/C, A/B
      4'd3:          cand_last_expert_solo_s1 = SHAPE_B; // B/B
      default:       cand_last_expert_solo_s1 = 'x;
    endcase
  endfunction

  function automatic shape_t cand_last_expert_solo_s3(input logic [3:0] shape_index);
    unique case (shape_index)
      4'd0, 4'd2: cand_last_expert_solo_s3 = SHAPE_C; // C/C, A/C
      4'd1:       cand_last_expert_solo_s3 = SHAPE_A; // A/A
      4'd3, 4'd4: cand_last_expert_solo_s3 = SHAPE_B; // B/B, A/B
      default:    cand_last_expert_solo_s3 = 'x;
    endcase
  endfunction

  function automatic ntok_t cand_both_idle_split_cut(
    input ntok_t ntok,
    input logic [CAND_ID_W-1:0] candidate_id
  );
    unique case (candidate_id)
      CAND_ID_W'(3): cand_both_idle_split_cut = ceil_div2_ntok(ntok);
      CAND_ID_W'(4): cand_both_idle_split_cut = NTOK_W'(2);
      default:       cand_both_idle_split_cut = 'x;
    endcase
  endfunction

  function automatic logic cand_both_idle_split_valid(
    input ntok_t ntok,
    input logic [CAND_ID_W-1:0] candidate_id
  );
    unique case (candidate_id)
      CAND_ID_W'(3): cand_both_idle_split_valid = (ntok >= NTOK_W'(2));
      CAND_ID_W'(4): cand_both_idle_split_valid = (ntok >= NTOK_W'(5));
      default:       cand_both_idle_split_valid = 'x;
    endcase
  endfunction

  function automatic logic cand_one_idle_is_adaptive(input cand_token_t token);
    cand_one_idle_is_adaptive = (token.mode == CAND_MODE_ONE_IDLE) &&
                                (token.id >= CAND_ID_W'(ONE_IDLE_ADAPTIVE_FIRST));
  endfunction

  function automatic logic [1:0] cand_one_idle_release_index(input cand_token_t token);
    cand_one_idle_release_index = cand_one_idle_is_adaptive(token) ?
        2'(token.id - CAND_ID_W'(ONE_IDLE_ADAPTIVE_FIRST)) : token.id[1:0];
  endfunction

  function automatic shape_t cand_one_idle_adaptive_s1(input ntok_t ntok);
    if (ntok >= NTOK_W'(7)) begin
      cand_one_idle_adaptive_s1 = SHAPE_A;
    end else if (ntok >= NTOK_W'(3)) begin
      cand_one_idle_adaptive_s1 = SHAPE_B;
    end else begin
      cand_one_idle_adaptive_s1 = SHAPE_C;
    end
  endfunction

  function automatic shape_t cand_one_idle_adaptive_s3(input ntok_t ntok);
    cand_one_idle_adaptive_s3 = (ntok >= NTOK_W'(3)) ? SHAPE_B : SHAPE_C;
  endfunction

  function automatic logic [3:0] cand_remove_mask(input cand_token_t token);
    unique case (token.mode)
      CAND_MODE_BOTH_IDLE: begin
        unique case (token.id)
          CAND_ID_W'(0): cand_remove_mask = 4'b0011; // top0/top1
          CAND_ID_W'(1): cand_remove_mask = 4'b0110; // top1/top2
          CAND_ID_W'(2): cand_remove_mask = 4'b1100; // top2/top3
          CAND_ID_W'(3),
          CAND_ID_W'(4): cand_remove_mask = 4'b0001; // split top0
          default:       cand_remove_mask = 'x;
        endcase
      end
      CAND_MODE_LAST_EXPERT,
      CAND_MODE_ONE_IDLE: cand_remove_mask = 4'b0001;
      default:            cand_remove_mask = 'x;
    endcase
  endfunction

  function automatic logic [1:0] cand_remove_count_from_mask(input logic [3:0] mask);
    cand_remove_count_from_mask = (|mask[2:1]) ? 2'd2 : 2'd1;
  endfunction

`ifndef SYNTHESIS
  function automatic logic cand_remove_mask_legal(input logic [3:0] mask);
    cand_remove_mask_legal = (mask == 4'b0001) ||
                             (mask == 4'b0011) ||
                             (mask == 4'b0110) ||
                             (mask == 4'b1100);
  endfunction
`endif

endpackage
