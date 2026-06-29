// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - try_s2pf_pair primitive（4-lane sequential version）
//
// C 语义：先看 no-PF baseline，再优先选择 prefetch 数量更多的方案；
// 同样 prefetch 数量下，选择 s2pf_start 之和更小的方案。原 RTL 同拍
// 展开 25 个 sched_bw_ok。本版本复用 4 个 lane：
//   - class2: A+B 都做 S2PF, 16 组, 4 cycles
//   - class1: A-only/B-only, 8 组, 2 cycles
//   - class0: no PF baseline, 1 个 raw bw_ok
// 若 class2 已经找到可行解，则不会继续扫 class1/class0 的候选比较。

import sched_pkg::*;

module sched_s2pf_pair (
  input  logic       clk_i,
  input  logic       rst_ni,
  input  logic       start_i,
  output logic       busy_o,
  output logic       done_o,

  input  logic       enable_i,
  input  logic       single_latest_only_i,
  input  logic       side_a_active_i,
  input  logic       side_b_active_i,
  input  logic [1:0] shape_s3_a_i,
  input  logic [1:0] shape_s3_b_i,
  input  eval_snap_t snap_a_i,
  input  eval_snap_t snap_b_i,

  output logic       ok_o,
  output eval_snap_t snap_a_o,
  output eval_snap_t snap_b_o
);

  localparam int unsigned LANES = 4;

  typedef enum logic [1:0] {
    ST_IDLE,
    ST_SCAN_BOTH,
    ST_SCAN_ONE,
    ST_DONE
  } state_t;

  state_t st_q, st_d;

  logic [1:0] group_q, group_d;
  logic       latest_only_q, latest_only_d;
  logic       scan_one_after_both_q, scan_one_after_both_d;

  logic       best_valid_q, best_valid_d;
  logic [1:0] best_class_q, best_class_d;
  logic [T_W:0] best_start_sum_q, best_start_sum_d;
  logic       best_a_pf_q, best_a_pf_d;
  logic       best_b_pf_q, best_b_pf_d;
  logic [2:0] best_a_sel_q, best_a_sel_d;
  logic [2:0] best_b_sel_q, best_b_sel_d;

  logic [T_W-1:0] ca [4];
  logic [T_W-1:0] cb [4];
  logic           ca_valid [4];
  logic           cb_valid [4];
  logic [T_W-1:0] hi_a;
  logic [T_W-1:0] hi_b;
  logic           can_a;
  logic           can_b;

  logic       raw_ok;

  eval_snap_t lane_a [LANES];
  eval_snap_t lane_b [LANES];
  logic       lane_valid [LANES];
  logic       lane_ok [LANES];
  logic [1:0] lane_class [LANES];
  logic [T_W:0] lane_start_sum [LANES];
  logic       lane_a_pf [LANES];
  logic       lane_b_pf [LANES];
  logic [2:0] lane_a_sel [LANES];
  logic [2:0] lane_b_sel [LANES];

  function automatic logic can_apply_s2pf(
    input eval_snap_t sn,
    input logic [1:0] shape_s3,
    input logic [T_W-1:0] pf_start
  );
    logic [T_W-1:0] dur;
    begin
      dur = kTd3(shape_s3);
      can_apply_s2pf = sn.valid &&
                       (sn.bw_s3 != BW_0) &&
                       (pf_start >= sn.task_start) &&
                       (pf_start + dur <= sn.s2_end);
    end
  endfunction

  function automatic eval_snap_t apply_s2pf(
    input eval_snap_t sn,
    input logic [1:0] shape_s3,
    input logic [T_W-1:0] pf_start
  );
    eval_snap_t ret;
    logic [T_W-1:0] pf_end;
    begin
      ret    = sn;
      pf_end = pf_start + kTd3(shape_s3);
      if (can_apply_s2pf(sn, shape_s3, pf_start)) begin
        ret.s2pf_valid = 1'b1;
        ret.s2pf_start = pf_start;
        ret.s2pf_end   = pf_end;
        ret.s2pf_bw    = alloc_bw(shape_s3);
        ret.dma3_end   = sn.s2_end;
        ret.s3_end     = sn.s2_end;
        ret.s4_start   = sn.s2_end;
        ret.bw_s3      = BW_0;
        ret.task_end   = sn.s2_end + best_s4_t(sn.ntok);
      end
      apply_s2pf = ret;
    end
  endfunction

  function automatic logic [T_W-1:0] select_a_start(input logic [2:0] sel);
    select_a_start = sel[2] ? hi_a : ca[sel[1:0]];
  endfunction

  function automatic logic [T_W-1:0] select_b_start(input logic [2:0] sel);
    select_b_start = sel[2] ? hi_b : cb[sel[1:0]];
  endfunction

  sched_bw_ok i_raw_bw_ok (
    .snap_a_i (snap_a_i),
    .snap_b_i (snap_b_i),
    .ok_o     (raw_ok)
  );

  for (genvar g = 0; g < LANES; g++) begin : gen_lane_bw_ok
    sched_bw_ok i_bw_ok (
      .snap_a_i (lane_a[g]),
      .snap_b_i (lane_b[g]),
      .ok_o     (lane_ok[g])
    );
  end

  always_comb begin
    for (int i = 0; i < 4; i++) begin
      ca[i]       = '0;
      cb[i]       = '0;
      ca_valid[i] = 1'b0;
      cb_valid[i] = 1'b0;
    end

    can_a = enable_i && side_a_active_i &&
            can_apply_s2pf(snap_a_i, shape_s3_a_i, snap_a_i.task_start);
    can_b = enable_i && side_b_active_i &&
            can_apply_s2pf(snap_b_i, shape_s3_b_i, snap_b_i.task_start);

    hi_a = can_a ? (snap_a_i.s2_end - kTd3(shape_s3_a_i)) : '0;
    hi_b = can_b ? (snap_b_i.s2_end - kTd3(shape_s3_b_i)) : '0;

    if (can_a) begin
      ca[0]       = snap_a_i.task_start;
      ca_valid[0] = 1'b1;
      if ((snap_a_i.dma1_end >= snap_a_i.task_start) &&
          (snap_a_i.dma1_end <= hi_a) &&
          (snap_a_i.dma1_end != snap_a_i.task_start)) begin
        ca[1]       = snap_a_i.dma1_end;
        ca_valid[1] = 1'b1;
      end
      if ((snap_a_i.s1_end >= snap_a_i.task_start) &&
          (snap_a_i.s1_end <= hi_a) &&
          (snap_a_i.s1_end != snap_a_i.dma1_end)) begin
        ca[2]       = snap_a_i.s1_end;
        ca_valid[2] = 1'b1;
      end
      if (hi_a != snap_a_i.task_start) begin
        ca[3]       = hi_a;
        ca_valid[3] = 1'b1;
      end
    end

    if (can_b) begin
      cb[0]       = snap_b_i.task_start;
      cb_valid[0] = 1'b1;
      if ((snap_b_i.dma1_end >= snap_b_i.task_start) &&
          (snap_b_i.dma1_end <= hi_b) &&
          (snap_b_i.dma1_end != snap_b_i.task_start)) begin
        cb[1]       = snap_b_i.dma1_end;
        cb_valid[1] = 1'b1;
      end
      if ((snap_b_i.s1_end >= snap_b_i.task_start) &&
          (snap_b_i.s1_end <= hi_b) &&
          (snap_b_i.s1_end != snap_b_i.dma1_end)) begin
        cb[2]       = snap_b_i.s1_end;
        cb_valid[2] = 1'b1;
      end
      if (hi_b != snap_b_i.task_start) begin
        cb[3]       = hi_b;
        cb_valid[3] = 1'b1;
      end
    end
  end

  always_comb begin
    for (int l = 0; l < LANES; l++) begin
      logic [3:0] both_idx;
      logic [2:0] one_idx;
      logic [1:0] ia;
      logic [1:0] ib;

      lane_a[l]         = snap_a_i;
      lane_b[l]         = snap_b_i;
      lane_valid[l]     = 1'b0;
      lane_class[l]     = 2'd0;
      lane_start_sum[l] = '0;
      lane_a_pf[l]      = 1'b0;
      lane_b_pf[l]      = 1'b0;
      lane_a_sel[l]     = '0;
      lane_b_sel[l]     = '0;

      both_idx = {group_q, 2'(l)};
      one_idx  = {group_q[0], 2'(l)};
      ia       = both_idx[3:2];
      ib       = both_idx[1:0];

      unique case (st_q)
        ST_SCAN_BOTH: begin
          lane_valid[l]     = ca_valid[ia] && cb_valid[ib];
          lane_a[l]         = apply_s2pf(snap_a_i, shape_s3_a_i, ca[ia]);
          lane_b[l]         = apply_s2pf(snap_b_i, shape_s3_b_i, cb[ib]);
          lane_class[l]     = 2'd2;
          lane_start_sum[l] = {1'b0, ca[ia]} + {1'b0, cb[ib]};
          lane_a_pf[l]      = 1'b1;
          lane_b_pf[l]      = 1'b1;
          lane_a_sel[l]     = {1'b0, ia};
          lane_b_sel[l]     = {1'b0, ib};
        end

        ST_SCAN_ONE: begin
          if (latest_only_q) begin
            if (l == 0) begin
              if (side_a_active_i) begin
                lane_valid[l]     = can_a;
                lane_a[l]         = apply_s2pf(snap_a_i, shape_s3_a_i, hi_a);
                lane_class[l]     = 2'd1;
                lane_start_sum[l] = {1'b0, hi_a};
                lane_a_pf[l]      = 1'b1;
                lane_a_sel[l]     = 3'd4;
              end else if (side_b_active_i) begin
                lane_valid[l]     = can_b;
                lane_b[l]         = apply_s2pf(snap_b_i, shape_s3_b_i, hi_b);
                lane_class[l]     = 2'd1;
                lane_start_sum[l] = {1'b0, hi_b};
                lane_b_pf[l]      = 1'b1;
                lane_b_sel[l]     = 3'd4;
              end
            end
          end else if (one_idx < 3'd4) begin
            lane_valid[l]     = ca_valid[one_idx[1:0]];
            lane_a[l]         = apply_s2pf(snap_a_i, shape_s3_a_i, ca[one_idx[1:0]]);
            lane_class[l]     = 2'd1;
            lane_start_sum[l] = {1'b0, ca[one_idx[1:0]]};
            lane_a_pf[l]      = 1'b1;
            lane_a_sel[l]     = {1'b0, one_idx[1:0]};
          end else begin
            lane_valid[l]     = cb_valid[one_idx[1:0]];
            lane_b[l]         = apply_s2pf(snap_b_i, shape_s3_b_i, cb[one_idx[1:0]]);
            lane_class[l]     = 2'd1;
            lane_start_sum[l] = {1'b0, cb[one_idx[1:0]]};
            lane_b_pf[l]      = 1'b1;
            lane_b_sel[l]     = {1'b0, one_idx[1:0]};
          end
        end

        default: begin
          // Lanes idle in ST_IDLE/ST_DONE; raw no-PF is checked separately.
        end
      endcase
    end
  end

  always_comb begin
    st_d                  = st_q;
    group_d               = group_q;
    latest_only_d         = latest_only_q;
    scan_one_after_both_d = scan_one_after_both_q;

    best_valid_d     = best_valid_q;
    best_class_d     = best_class_q;
    best_start_sum_d = best_start_sum_q;
    best_a_pf_d      = best_a_pf_q;
    best_b_pf_d      = best_b_pf_q;
    best_a_sel_d     = best_a_sel_q;
    best_b_sel_d     = best_b_sel_q;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          best_valid_d     = (snap_a_i.valid || snap_b_i.valid) && raw_ok;
          best_class_d     = 2'd0;
          best_start_sum_d = '0;
          best_a_pf_d      = 1'b0;
          best_b_pf_d      = 1'b0;
          best_a_sel_d     = '0;
          best_b_sel_d     = '0;
          group_d          = '0;
          latest_only_d    = single_latest_only_i;

          if (!enable_i) begin
            st_d = ST_DONE;
          end else if (single_latest_only_i) begin
            if ((side_a_active_i && can_a) || (side_b_active_i && can_b)) begin
              st_d = ST_SCAN_ONE;
            end else begin
              st_d = ST_DONE;
            end
            scan_one_after_both_d = 1'b0;
          end else if (side_a_active_i && side_b_active_i) begin
            scan_one_after_both_d = can_a || can_b;
            if (can_a && can_b) begin
              st_d = ST_SCAN_BOTH;
            end else if (can_a || can_b) begin
              st_d = ST_SCAN_ONE;
            end else begin
              st_d = ST_DONE;
            end
          end else begin
            scan_one_after_both_d = 1'b0;
            st_d = ST_DONE;
          end
        end
      end

      ST_SCAN_BOTH: begin
        for (int l = 0; l < LANES; l++) begin
          if (lane_valid[l] && lane_ok[l]) begin
            if (!best_valid_d ||
                (lane_class[l] > best_class_d) ||
                ((lane_class[l] == best_class_d) &&
                 (lane_start_sum[l] < best_start_sum_d))) begin
              best_valid_d     = 1'b1;
              best_class_d     = lane_class[l];
              best_start_sum_d = lane_start_sum[l];
              best_a_pf_d      = lane_a_pf[l];
              best_b_pf_d      = lane_b_pf[l];
              best_a_sel_d     = lane_a_sel[l];
              best_b_sel_d     = lane_b_sel[l];
            end
          end
        end

        if (group_q == 2'd3) begin
          // class2 优先级最高；只要找到 A+B prefetch，就不再扫 class1。
          if (best_valid_d && (best_class_d == 2'd2)) begin
            st_d = ST_DONE;
          end else if (scan_one_after_both_q) begin
            group_d = '0;
            st_d    = ST_SCAN_ONE;
          end else begin
            st_d = ST_DONE;
          end
        end else begin
          group_d = group_q + 2'd1;
        end
      end

      ST_SCAN_ONE: begin
        for (int l = 0; l < LANES; l++) begin
          if (lane_valid[l] && lane_ok[l]) begin
            if (!best_valid_d ||
                (lane_class[l] > best_class_d) ||
                ((lane_class[l] == best_class_d) &&
                 (lane_start_sum[l] < best_start_sum_d))) begin
              best_valid_d     = 1'b1;
              best_class_d     = lane_class[l];
              best_start_sum_d = lane_start_sum[l];
              best_a_pf_d      = lane_a_pf[l];
              best_b_pf_d      = lane_b_pf[l];
              best_a_sel_d     = lane_a_sel[l];
              best_b_sel_d     = lane_b_sel[l];
            end
          end
        end

        if (latest_only_q || (group_q == 2'd1)) begin
          st_d = ST_DONE;
        end else begin
          group_d = group_q + 2'd1;
        end
      end

      ST_DONE: begin
        st_d = ST_IDLE;
      end

      default: st_d = ST_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q                  <= ST_IDLE;
      group_q               <= '0;
      latest_only_q         <= 1'b0;
      scan_one_after_both_q <= 1'b0;
      best_valid_q          <= 1'b0;
      best_class_q          <= '0;
      best_start_sum_q      <= '0;
      best_a_pf_q           <= 1'b0;
      best_b_pf_q           <= 1'b0;
      best_a_sel_q          <= '0;
      best_b_sel_q          <= '0;
    end else begin
      st_q                  <= st_d;
      group_q               <= group_d;
      latest_only_q         <= latest_only_d;
      scan_one_after_both_q <= scan_one_after_both_d;
      best_valid_q          <= best_valid_d;
      best_class_q          <= best_class_d;
      best_start_sum_q      <= best_start_sum_d;
      best_a_pf_q           <= best_a_pf_d;
      best_b_pf_q           <= best_b_pf_d;
      best_a_sel_q          <= best_a_sel_d;
      best_b_sel_q          <= best_b_sel_d;
    end
  end

  always_comb begin
    snap_a_o = best_a_pf_q ? apply_s2pf(snap_a_i, shape_s3_a_i,
                                        select_a_start(best_a_sel_q)) : snap_a_i;
    snap_b_o = best_b_pf_q ? apply_s2pf(snap_b_i, shape_s3_b_i,
                                        select_b_start(best_b_sel_q)) : snap_b_i;
  end

  assign ok_o   = best_valid_q;
  assign busy_o = (st_q != ST_IDLE) && (st_q != ST_DONE);
  assign done_o = (st_q == ST_DONE);

endmodule
