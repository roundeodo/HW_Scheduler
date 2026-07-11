// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - compact 64-bit task-word packer
//
// This is the only RTL block that knows how to turn a committed single-task
// descriptor into the TASK_FIFO_DATA ABI word.  sched_pkg only owns the bit
// layout constants; lowering arithmetic stays local to this leaf block.

import sched_pkg::*;

module sched_task_word_pack (
  input  task_desc_t task_i,
  input  slot_id_t   local_slot_i,
  input  logic [7:0] s4pf_desc_i,
  output logic [63:0] word_o
);

  ntok_t tail_s2;
  ntok_t tail_s4;
  ntok_t m_s2_exec;
  ntok_t m_s4_exec;
  logic [TASK_WORD_CTRL_W-1:0] ctrl_word;

  function automatic ntok_t stage_tail(
    input ntok_t  ntok,
    input shape_t shape,
    input logic   skip
  );
    begin
      if (skip) begin
        stage_tail = ntok;
      end else begin
        unique case (shape)
          SHAPE_A: stage_tail = (ntok > ntok_t'(8)) ? (ntok - ntok_t'(8)) : '0;
          SHAPE_B: stage_tail = (ntok > ntok_t'(4)) ? (ntok - ntok_t'(4)) : '0;
          default: stage_tail = (ntok > ntok_t'(2)) ? (ntok - ntok_t'(2)) : '0;
        endcase
      end
    end
  endfunction

  always_comb begin
    tail_s2 = stage_tail(task_i.ntok, task_i.s1, task_i.skip_s1);
    tail_s4 = stage_tail(task_i.ntok, task_i.s3, task_i.skip_s3);

    // Shape C consumes 2 tokens per tile, so execution M tiles are ceil(M/2).
    m_s2_exec = ceil_div2_ntok(tail_s2);
    m_s4_exec = ceil_div2_ntok(tail_s4);

    ctrl_word = '0;
    ctrl_word[0]    = task_i.skip_s1;
    ctrl_word[1]    = task_i.skip_s3;
    ctrl_word[3:2]  = task_i.s1;
    ctrl_word[5:4]  = task_i.s3;
    ctrl_word[6]    = task_i.cluster;
    ctrl_word[12:7] = local_slot_i;

    word_o = '0;
    word_o[TASK_WORD_EID_LSB +: EID_RAW_W]          = task_i.eid;
    word_o[TASK_WORD_TOKEN_START_LSB +: NTOK_W]     = task_i.tok_start;
    word_o[TASK_WORD_NTOK_LSB +: NTOK_W]            = task_i.ntok;
    word_o[TASK_WORD_HAS_S2PF_LSB]                  = task_i.has_s2pf;
    word_o[TASK_WORD_CTRL_LSB +: TASK_WORD_CTRL_W]  = ctrl_word;
    word_o[TASK_WORD_M_S2_LSB +: NTOK_W]            = m_s2_exec;
    word_o[TASK_WORD_M_S4_LSB +: NTOK_W]            = m_s4_exec;
    word_o[TASK_WORD_S4PF_DESC_LSB +: 8]            = s4pf_desc_i;
  end

endmodule
