// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - compact 64-bit task-word packer
//
// This is the only RTL block that knows how to turn a committed single-task
// descriptor into the TASK_STREAM ABI word.  sched_pkg only owns the bit
// layout constants; lowering arithmetic stays local to this leaf block.

import sched_pkg::*;

module sched_task_word_pack (
  input  wire task_desc_t task_i,
  input  slot_id_t   local_slot_i,
  input  logic [7:0] s4pf_desc_i,
  output logic [63:0] word_o
);

  logic [TASK_COUNT_W-1:0] full_tile_count;
  logic [TASK_COUNT_W-1:0] s2_tile_count;
  logic [TASK_COUNT_W-1:0] s4_tile_count;
  logic [TASK_WORD_CTRL_W-1:0] packed_control;

  function automatic logic [TASK_COUNT_W-1:0] stage_tile_count(
    input logic [TASK_COUNT_W-1:0] tiles,
    input shape_t shape,
    input logic   skip
  );
    logic [TASK_COUNT_W-1:0] consumed_tiles;
    begin
      consumed_tiles = '0;
      if (!skip) begin
        unique case (shape)
          SHAPE_A: consumed_tiles = TASK_COUNT_W'(4);
          SHAPE_B: consumed_tiles = TASK_COUNT_W'(2);
          SHAPE_C: consumed_tiles = TASK_COUNT_W'(1);
          default: consumed_tiles = 'x;
        endcase
      end
      stage_tile_count = (tiles > consumed_tiles) ?
                         (tiles - consumed_tiles) : '0;
    end
  endfunction

  always_comb begin
    full_tile_count = TASK_COUNT_W'(ceil_div2_ntok(task_i.ntok));
    s2_tile_count = stage_tile_count(
        full_tile_count, task_i.shape_s1, task_i.skip_s1);
    s4_tile_count = stage_tile_count(
        full_tile_count, task_i.shape_s3, task_i.skip_s3);

    packed_control = '0;
    packed_control[0]    = task_i.skip_s1;
    packed_control[1]    = task_i.skip_s3;
    packed_control[3:2]  = task_i.shape_s1;
    packed_control[5:4]  = task_i.shape_s3;
    packed_control[6]    = task_i.cluster;
    packed_control[12:7] = local_slot_i;

    word_o = '0;
    word_o[TASK_WORD_EID_LSB +: EID_RAW_W]          = task_i.eid;
    word_o[TASK_WORD_TOKEN_START_LSB +: NTOK_W]     = task_i.tok_start;
    word_o[TASK_WORD_NTOK_LSB +: NTOK_W]            = task_i.ntok;
    word_o[TASK_WORD_HAS_S2PF_LSB]                  = task_i.has_s2pf;
    word_o[TASK_WORD_CTRL_LSB +: TASK_WORD_CTRL_W]  = packed_control;
    word_o[TASK_WORD_M_S2_LSB +: TASK_COUNT_W]       =
        s2_tile_count;
    word_o[TASK_WORD_S1_BOTH_LSB]                    = task_i.dma_s1_both;
    word_o[TASK_WORD_M_S4_LSB +: TASK_COUNT_W]       =
        s4_tile_count;
    word_o[TASK_WORD_LATE_BOTH_LSB]                  = task_i.dma_late_both;
    word_o[TASK_WORD_S4PF_DESC_LSB +: 8]            = s4pf_desc_i;
  end

endmodule
