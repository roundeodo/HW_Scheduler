// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// Candidate token package
//
// candidate_generator 只跨模块发送 mode/id token。task payload、remove mask、
// shape policy、cache hit 和 continuation 摘要都在 eval_lane 入口按 token 解码。
// 这样 generator 输出保持很窄，避免构造和传输完整候选对象。

package sched_candidate_pkg;

  import sched_pkg::*;

  typedef enum logic [1:0] {
    CAND_MODE_SINGLE   = 2'd0,
    CAND_MODE_BOTH     = 2'd1,
    CAND_MODE_NOT_BOTH = 2'd2
  } cand_mode_t;

  typedef struct packed {
    logic                 valid;
    cand_mode_t           mode;
    logic [CAND_ID_W-1:0] id;
  } cand_token_t;

  localparam int unsigned SINGLE_SOLO_SHAPES = 5;
  localparam int unsigned SINGLE_SOLO_COUNT  = 2 * SINGLE_SOLO_SHAPES;
  localparam int unsigned SINGLE_SPLIT_ID    = SINGLE_SOLO_COUNT;
  localparam int unsigned SINGLE_EARLY0_ID   = SINGLE_SPLIT_ID + 1;
  localparam int unsigned SINGLE_LAST_ID     = SINGLE_EARLY0_ID + 1;

  function automatic logic [CAND_ID_W-1:0] cand_mode_last_id(input cand_mode_t mode);
    unique case (mode)
      CAND_MODE_SINGLE:   cand_mode_last_id = CAND_ID_W'(SINGLE_LAST_ID);
      CAND_MODE_BOTH:     cand_mode_last_id = CAND_ID_W'(4);
      CAND_MODE_NOT_BOTH: cand_mode_last_id = CAND_ID_W'(2);
      default:            cand_mode_last_id = '0;
    endcase
  endfunction

  function automatic ntok_t cand_single_split_cut(input ntok_t ntok);
    cand_single_split_cut = ceil_div2_ntok(ntok);
  endfunction

  function automatic logic cand_single_split_valid(input ntok_t ntok);
    ntok_t cut;
    begin
      cut = cand_single_split_cut(ntok);
      cand_single_split_valid = (ntok >= NTOK_W'(2)) && (cut > '0) && (cut < ntok);
    end
  endfunction

  function automatic shape_t cand_single_solo_s1(input logic [3:0] slot);
    unique case (slot)
      4'd0:    cand_single_solo_s1 = SHAPE_C; // C/C
      4'd3:    cand_single_solo_s1 = SHAPE_B; // B/B
      default: cand_single_solo_s1 = SHAPE_A; // A/A, A/C, A/B
    endcase
  endfunction

  function automatic shape_t cand_single_solo_s3(input logic [3:0] slot);
    unique case (slot)
      4'd0, 4'd2: cand_single_solo_s3 = SHAPE_C; // C/C, A/C
      4'd3, 4'd4: cand_single_solo_s3 = SHAPE_B; // B/B, A/B
      default:    cand_single_solo_s3 = SHAPE_A; // A/A
    endcase
  endfunction

  function automatic ntok_t cand_both_split_cut(input ntok_t ntok, input logic [CAND_ID_W-1:0] id);
    cand_both_split_cut = (id == CAND_ID_W'(3)) ? ceil_div2_ntok(ntok) : NTOK_W'(2);
  endfunction

  function automatic logic cand_both_split_valid(input ntok_t ntok, input logic [CAND_ID_W-1:0] id);
    if (id == CAND_ID_W'(3)) begin
      cand_both_split_valid = (ntok >= NTOK_W'(2));
    end else begin
      cand_both_split_valid = (ntok >= NTOK_W'(5));
    end
  endfunction

  function automatic logic [3:0] cand_remove_mask(input cand_token_t token);
    unique case (token.mode)
      CAND_MODE_BOTH: begin
        unique case (token.id)
          CAND_ID_W'(0): cand_remove_mask = 4'b0011; // top0/top1
          CAND_ID_W'(1): cand_remove_mask = 4'b0110; // top1/top2
          CAND_ID_W'(2): cand_remove_mask = 4'b1100; // top2/top3
          default:       cand_remove_mask = 4'b0001; // split top0
        endcase
      end
      default: cand_remove_mask = 4'b0001;           // single/not-both use top0
    endcase
  endfunction

  function automatic logic [1:0] cand_remove_count(input cand_token_t token);
    cand_remove_count = ((token.mode == CAND_MODE_BOTH) &&
                         (token.id <= CAND_ID_W'(2))) ? 2'd2 : 2'd1;
  endfunction

  function automatic logic cand_remove_mask_legal(input logic [3:0] mask);
    cand_remove_mask_legal = (mask == 4'b0001) ||
                             (mask == 4'b0011) ||
                             (mask == 4'b0110) ||
                             (mask == 4'b1100);
  endfunction

endpackage
