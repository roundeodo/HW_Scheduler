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
  input  snap_timeline_t timeline_i,
  input  snap_cache_t    cache_i,
  output snap_timeline_t timeline_o,
  output snap_cache_t    cache_o
);

  always_comb begin
    timeline_o = timeline_i;
    cache_o    = cache_i;

    if (timeline_i.valid &&
        (cache_i.pf_eid == PF_EID_NONE) &&
        (timeline_i.dma1_end <= timeline_i.s4_start) &&
        (timeline_i.s4_start + GHOST_WINDOW_TICKS <= timeline_i.task_end)) begin
      cache_o.pf_eid  = PF_EID_GHOST;
      cache_o.pf_end  = timeline_i.task_end;
      cache_o.pf_full = 1'b0;
      timeline_o.s4pf_valid = 1'b1;
      timeline_o.s4pf_start = timeline_i.s4_start;
    end
  end

endmodule
