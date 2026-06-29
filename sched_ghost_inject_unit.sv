// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler — S4 ghost prefetch injector
//
// Mirrors the C scheduler's conservative ghost insertion before candidate
// enumeration. Ghost means the next task placed on this cluster may skip S1;
// it is not a full cache hit, so S3/down is not skipped.

import sched_pkg::*;

module sched_ghost_inject_unit (
  input  eval_snap_t snap_i,
  output eval_snap_t snap_o
);

  always_comb begin
    snap_o = snap_i;

    if (snap_i.valid &&
        (snap_i.pf_eid == PF_EID_NONE) &&
        (snap_i.dma1_end <= snap_i.s4_start) &&
        (snap_i.s4_start + GHOST_WINDOW_TICKS <= snap_i.task_end)) begin
      snap_o.pf_eid  = PF_EID_GHOST;
      snap_o.pf_end  = snap_i.task_end;
      snap_o.pf_full = 1'b0;
      snap_o.s4pf_valid = 1'b1;
      snap_o.s4pf_start = snap_i.s4_start;
    end
  end

endmodule
