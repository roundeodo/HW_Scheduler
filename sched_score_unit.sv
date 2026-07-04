// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// MoE Hardware Scheduler - continuation_cost primitive (tick-domain version)
//
// This module follows moe_scheduler.c::continuation_cost():
//   rem_len=0  -> max(c2.task_end, c3.task_end)
//   rem_len=1  -> sim1(), evaluated by a small FSM
//   rem_len=2  -> exact 2-task expression (same as greedy_h nr==2)
//   rem_len>2  -> greedy_h(sum/max best_conc)
//
// The old reference implementation instantiated all sim1 solo cases in
// parallel.  This version only enters the sim1 FSM when rem_len_i==1 and
// reuses one mk_snap datapath for the solo cases.  The split case is replayed
// combinationally in its final state instead of storing two snap records.

import sched_pkg::*;

module sched_score_unit (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  input  logic                  start_i,
  output logic                  busy_o,
  output logic                  done_o,

  input  snap_timeline_t        c2_timeline_i,
  input  snap_timeline_t        c3_timeline_i,
  input  snap_cache_t           c2_cache_i,
  input  snap_cache_t           c3_cache_i,
  input  logic [NR_W-1:0]       rem_len_i,
  input  logic [EID_RAW_W-1:0]  rem0_eid_i,
  input  logic [NTOK_W-1:0]     rem0_ntok_i,
  input  logic [NTOK_W-1:0]     rem1_ntok_i,
  input  logic [T_W-1:0]        total_conc_i,
  input  logic [T_W-1:0]        max_conc_i,
  output logic                  bw_start_o,
  output snap_timeline_t        bw_snap_a_o,
  output snap_timeline_t        bw_snap_b_o,
  input  logic                  bw_done_i,
  input  logic                  bw_ok_i,
  output logic [T_W-1:0]        cost_o
);

  localparam logic [T_W-1:0] INF_T = {T_W{1'b1}};
  // 状态含义：
  //   ST_IDLE           等待一个新的 candidate score 请求。
  //   ST_SIM1_SOLO      rem_len==1 时，复用一套 mk_snap 枚举 4 个 solo 收尾。
  //   ST_SIM1_SPLIT_S2PF 等待 split A/B 的 S2PF 搜索完成，再更新 best cost。
  //   ST_DONE           cost_o 有效，等待 start_i 拉低后回到 IDLE。
  typedef enum logic [2:0] {
    ST_IDLE,
    ST_SIM1_SOLO,
    ST_SIM1_SPLIT_S2PF,
    ST_DONE
  } state_t;

  state_t st_q, st_d;

  logic [1:0]    solo_idx_q, solo_idx_d;
  logic [T_W-1:0] best_cost_q, best_cost_d;
  logic [T_W-1:0] cost_q, cost_d;

  // t_now/tl 是两个 cluster 都完成当前 snap 后的较晚时刻；
  // te 是较早空闲的 cluster 时刻。fast path 和 sim1 都复用这组时间基准。
  logic [T_W-1:0] t_now;
  logic [T_W-1:0] te;
  logic [T_W-1:0] tl;

  // rem_len==1 时，最后一个 expert 在 t_now 时刻检查 C2/C3 的 cache 命中。
  logic sw_c2;
  logic dn_c2;
  logic sw_c3;
  logic dn_c3;

  logic [NTOK_W:0]   split_tmp;
  logic [NTOK_W-1:0] split_a_ntok;
  logic [NTOK_W-1:0] split_b_ntok;
  logic [1:0]        split_s1a;
  logic [1:0]        split_s3a;
  logic [1:0]        split_s1b;
  logic [1:0]        split_s3b;

  // solo_idx 编码：
  //   0: C2 + shape C
  //   1: C2 + shape B
  //   2: C3 + shape C
  //   3: C3 + shape B
  // shape B 只有对应 cluster 没有 swiglu hit 时才合法，匹配 C sim1()。
  logic              solo_to_c3;
  logic              solo_shape_b;
  logic              solo_valid;

  // 复用的 mk_snap 输入。当前版本只用它枚举 solo cases。
  logic [T_W-1:0]    mk_start;
  logic [NTOK_W-1:0] mk_ntok;
  logic [1:0]        mk_shape_s1;
  logic [1:0]        mk_shape_s3;
  logic              mk_skip_s1;
  logic              mk_skip_s3;

  logic [T_W-1:0]    mk_task_start;
  logic [T_W-1:0]    mk_task_end;
  logic [T_W-1:0]    mk_dma1_end;
  logic [T_W-1:0]    mk_s2_end;
  logic [T_W-1:0]    mk_dma3_end;
  logic [T_W-1:0]    mk_s4_start;
  logic [BW_W-1:0]   mk_bw_s1;
  logic [BW_W-1:0]   mk_bw_s3;
  logic [NTOK_W-1:0] unused_m_s2;
  logic [NTOK_W-1:0] unused_m_s4;
  logic              unused_skip_s2;
  logic              unused_skip_s4;
  logic [1:0]        unused_dma_s1;
  logic [1:0]        unused_dma_s3;

  // split replay 使用两套组合 mk_snap 直接生成 A/B 两侧，不写 full snap FF。
  logic [T_W-1:0] split_a_task_start, split_a_task_end, split_a_dma1_end;
  logic [T_W-1:0] split_a_s2_end, split_a_dma3_end;
  logic [T_W-1:0] split_a_s4_start;
  logic [BW_W-1:0] split_a_bw_s1, split_a_bw_s3;
  logic [NTOK_W-1:0] split_a_unused_m_s2, split_a_unused_m_s4;
  logic split_a_unused_skip_s2, split_a_unused_skip_s4;
  logic [1:0] split_a_unused_dma_s1, split_a_unused_dma_s3;

  logic [T_W-1:0] split_b_task_start, split_b_task_end, split_b_dma1_end;
  logic [T_W-1:0] split_b_s2_end, split_b_dma3_end;
  logic [T_W-1:0] split_b_s4_start;
  logic [BW_W-1:0] split_b_bw_s1, split_b_bw_s3;
  logic [NTOK_W-1:0] split_b_unused_m_s2, split_b_unused_m_s4;
  logic split_b_unused_skip_s2, split_b_unused_skip_s4;
  logic [1:0] split_b_unused_dma_s1, split_b_unused_dma_s3;

  snap_timeline_t split_a_snap;
  snap_timeline_t split_b_snap;
  snap_timeline_t split_pf_a;
  snap_timeline_t split_pf_b;
  snap_timeline_t split_bw_snap_a;
  snap_timeline_t split_bw_snap_b;
  logic       split_ok;
  logic       split_s2pf_start;
  logic       split_s2pf_bw_start;
  logic       split_s2pf_done;

  sched_pick_shapes i_sim1_split_pick_shapes (
    .ntok_a_i (split_a_ntok),
    .ntok_b_i (split_b_ntok),
    .sw_a_i   (sw_c2),
    .dn_a_i   (dn_c2),
    .sw_b_i   (sw_c3),
    .dn_b_i   (dn_c3),
    .t0_i     (t_now),
    .s1a_o    (split_s1a),
    .s3a_o    (split_s3a),
    .s1b_o    (split_s1b),
    .s3b_o    (split_s3b)
  );

  sched_mk_snap i_reused_mk_snap (
    .start_t_i     (mk_start),
    .ntok_i        (mk_ntok),
    .shape_s1_i    (mk_shape_s1),
    .shape_s3_i    (mk_shape_s3),
    .skip_s1_i     (mk_skip_s1),
    .skip_s3_i     (mk_skip_s3),
    .task_start_o  (mk_task_start),
    .task_end_o    (mk_task_end),
    .dma1_end_o    (mk_dma1_end),
    .s2_end_o      (mk_s2_end),
    .dma3_end_o    (mk_dma3_end),
    .s4_start_o    (mk_s4_start),
    .bw_s1_o       (mk_bw_s1),
    .bw_s3_o       (mk_bw_s3),
    .m_s2_exec_o   (unused_m_s2),
    .m_s4_exec_o   (unused_m_s4),
    .skip_s2_o     (unused_skip_s2),
    .skip_s4_o     (unused_skip_s4),
    .dma_s1_o      (unused_dma_s1),
    .dma_s3_o      (unused_dma_s3)
  );

  sched_mk_snap i_split_mk_snap_a (
    .start_t_i     (t_now),
    .ntok_i        (split_a_ntok),
    .shape_s1_i    (split_s1a),
    .shape_s3_i    (split_s3a),
    .skip_s1_i     (sw_c2),
    .skip_s3_i     (dn_c2),
    .task_start_o  (split_a_task_start),
    .task_end_o    (split_a_task_end),
    .dma1_end_o    (split_a_dma1_end),
    .s2_end_o      (split_a_s2_end),
    .dma3_end_o    (split_a_dma3_end),
    .s4_start_o    (split_a_s4_start),
    .bw_s1_o       (split_a_bw_s1),
    .bw_s3_o       (split_a_bw_s3),
    .m_s2_exec_o   (split_a_unused_m_s2),
    .m_s4_exec_o   (split_a_unused_m_s4),
    .skip_s2_o     (split_a_unused_skip_s2),
    .skip_s4_o     (split_a_unused_skip_s4),
    .dma_s1_o      (split_a_unused_dma_s1),
    .dma_s3_o      (split_a_unused_dma_s3)
  );

  sched_mk_snap i_split_mk_snap_b (
    .start_t_i     (t_now),
    .ntok_i        (split_b_ntok),
    .shape_s1_i    (split_s1b),
    .shape_s3_i    (split_s3b),
    .skip_s1_i     (sw_c3),
    .skip_s3_i     (dn_c3),
    .task_start_o  (split_b_task_start),
    .task_end_o    (split_b_task_end),
    .dma1_end_o    (split_b_dma1_end),
    .s2_end_o      (split_b_s2_end),
    .dma3_end_o    (split_b_dma3_end),
    .s4_start_o    (split_b_s4_start),
    .bw_s1_o       (split_b_bw_s1),
    .bw_s3_o       (split_b_bw_s3),
    .m_s2_exec_o   (split_b_unused_m_s2),
    .m_s4_exec_o   (split_b_unused_m_s4),
    .skip_s2_o     (split_b_unused_skip_s2),
    .skip_s4_o     (split_b_unused_skip_s4),
    .dma_s1_o      (split_b_unused_dma_s1),
    .dma_s3_o      (split_b_unused_dma_s3)
  );

  sched_s2pf_pair i_sim1_split_s2pf (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .start_i              (split_s2pf_start),
    .busy_o               (),
    .done_o               (split_s2pf_done),
    .policy_i             (S2PF_SPLIT_LITE),
    .side_a_active_i      (split_a_snap.valid),
    .side_b_active_i      (split_b_snap.valid),
    .shape_s3_a_i         (split_s3a),
    .shape_s3_b_i         (split_s3b),
    .snap_a_i             (split_a_snap),
    .snap_b_i             (split_b_snap),
    .bw_start_o           (split_s2pf_bw_start),
    .bw_snap_a_o          (split_bw_snap_a),
    .bw_snap_b_o          (split_bw_snap_b),
    .bw_done_i            (bw_done_i),
    .bw_ok_i              (bw_ok_i),
    .ok_o                 (split_ok),
    .snap_a_o             (split_pf_a),
    .snap_b_o             (split_pf_b)
  );

  assign bw_start_o = split_s2pf_bw_start;
  assign bw_snap_a_o = split_bw_snap_a;
  assign bw_snap_b_o = split_bw_snap_b;

  function automatic logic [T_W-1:0] min_t(
    input logic [T_W-1:0] a,
    input logic [T_W-1:0] b
  );
    min_t = (a < b) ? a : b;
  endfunction

  function automatic logic [T_W-1:0] max_t(
    input logic [T_W-1:0] a,
    input logic [T_W-1:0] b
  );
    max_t = (a > b) ? a : b;
  endfunction

  function automatic logic [T_W-1:0] csa3_sum_t(
    input logic [T_W-1:0] a,
    input logic [T_W-1:0] b,
    input logic [T_W-1:0] c
  );
    logic [T_W-1:0] sum_bits;
    logic [T_W-1:0] carry_bits;
    logic [T_W:0]   final_sum;
    begin
      // Carry-save form for a+b+c.  The intermediate bt0+bt1 value is not
      // used anywhere, so do not force two serial carry-propagate adders.
      // Synthesis can map the xor/majority layer as a CSA compressor and use
      // only one final CPA for the visible result.
      sum_bits   = a ^ b ^ c;
      carry_bits = (a & b) | (a & c) | (b & c);
      final_sum  = {1'b0, sum_bits} + {carry_bits, 1'b0};
      csa3_sum_t = final_sum[T_W-1:0];
    end
  endfunction

  function automatic logic [T_W-1:0] fast_cost(
    input logic [NR_W-1:0]   rem_len,
    input logic [T_W-1:0]    te_i,
    input logic [T_W-1:0]    tl_i,
    input logic [NTOK_W-1:0] rem0_ntok,
    input logic [NTOK_W-1:0] rem1_ntok,
    input logic [T_W-1:0]    total_conc,
    input logic [T_W-1:0]    max_conc
  );
    logic [T_W-1:0] bc0;
    logic [T_W-1:0] bc1;
    logic [T_W-1:0] pc;
    logic [T_W-1:0] ser;
    logic [T_W-1:0] serc;
    logic [T_W-1:0] bt0;
    logic [T_W-1:0] bt1;
    logic [T_W-1:0] half_sum;
    logic [T_W-1:0] extra;
    begin
      // rem_len!=1 的快速路径不会进入 sim1 FSM：
      //   0：当前 makespan
      //   2：C 中 greedy_h nr==2 的 exact 2-task tail expression
      //   >2：greedy_h 的 aggregate 近似
      if (rem_len == NR_W'(0)) begin
        fast_cost = tl_i;
      end else if (rem_len == NR_W'(2)) begin
        bc0  = best_conc_ticks(rem0_ntok);
        bc1  = best_conc_ticks(rem1_ntok);
        bt0  = best_task_ticks(rem0_ntok);
        bt1  = best_task_ticks(rem1_ntok);
        pc   = tl_i + max_t(bc0, bc1);
        ser  = csa3_sum_t(te_i, bt0, bt1);
        serc = max_t(ser, tl_i);
        fast_cost = min_t(pc, serc);
      end else begin
        half_sum = {1'b0, total_conc[T_W-1:1]};
        extra    = max_t(max_conc, half_sum);
        fast_cost = tl_i + extra;
      end
    end
  endfunction

  always_comb begin
    // 当前两个 cluster 的完成时间基准。tl=t_now，保留 te/tl 命名是为了
    // 对齐 C 代码中 greedy_h/continuation_cost 的表达式。
    t_now = max_t(c2_timeline_i.task_end, c3_timeline_i.task_end);
    te    = min_t(c2_timeline_i.task_end, c3_timeline_i.task_end);
    tl    = t_now;

    sw_c2 = swiglu_hit_t(rem0_eid_i, c2_cache_i.pf_eid, c2_cache_i.pf_end, t_now);
    dn_c2 = down_hit_t(rem0_eid_i, c2_cache_i.pf_eid, c2_cache_i.pf_end,
                       c2_cache_i.pf_full, t_now);
    sw_c3 = swiglu_hit_t(rem0_eid_i, c3_cache_i.pf_eid, c3_cache_i.pf_end, t_now);
    dn_c3 = down_hit_t(rem0_eid_i, c3_cache_i.pf_eid, c3_cache_i.pf_end,
                       c3_cache_i.pf_full, t_now);

    split_tmp    = {1'b0, rem0_ntok_i} + {{NTOK_W{1'b0}}, 1'b1};
    split_a_ntok = NTOK_W'(split_tmp >> 1);
    split_b_ntok = rem0_ntok_i - split_a_ntok;

    // 默认把复用 mk_snap 接到当前 solo_idx 对应的 solo case。
    solo_to_c3    = solo_idx_q[1];
    solo_shape_b  = solo_idx_q[0];
    solo_valid    = !solo_shape_b || !(solo_to_c3 ? sw_c3 : sw_c2);

    mk_start    = t_now;
    mk_ntok     = rem0_ntok_i;
    mk_shape_s1 = solo_shape_b ? SHAPE_B : SHAPE_C;
    mk_shape_s3 = solo_shape_b ? SHAPE_B : SHAPE_C;
    mk_skip_s1  = solo_shape_b ? 1'b0 : (solo_to_c3 ? sw_c3 : sw_c2);
    mk_skip_s3  = solo_shape_b ? 1'b0 : (solo_to_c3 ? dn_c3 : dn_c2);

    // SIM1 split replay：A/B 两侧 snap 都由组合 mk_snap 直接重算出来，
    // 只在 ST_SIM1_SPLIT_S2PF 这一拍使用，不保存 full snap 临时寄存器。
    split_a_snap = '0;
    split_a_snap.valid      = (rem0_ntok_i >= NTOK_W'(2));
    split_a_snap.task_start = split_a_task_start;
    split_a_snap.task_end   = split_a_task_end;
    split_a_snap.dma1_end   = split_a_dma1_end;
    split_a_snap.s2_end     = split_a_s2_end;
    split_a_snap.dma3_end   = split_a_dma3_end;
    split_a_snap.s4_start   = split_a_s4_start;
    split_a_snap.bw_s1      = split_a_bw_s1;
    split_a_snap.bw_s3      = split_a_bw_s3;
    split_a_snap.ntok       = split_a_ntok;

    split_b_snap = '0;
    split_b_snap.valid      = (rem0_ntok_i >= NTOK_W'(2));
    split_b_snap.task_start = split_b_task_start;
    split_b_snap.task_end   = split_b_task_end;
    split_b_snap.dma1_end   = split_b_dma1_end;
    split_b_snap.s2_end     = split_b_s2_end;
    split_b_snap.dma3_end   = split_b_dma3_end;
    split_b_snap.s4_start   = split_b_s4_start;
    split_b_snap.bw_s1      = split_b_bw_s1;
    split_b_snap.bw_s3      = split_b_bw_s3;
    split_b_snap.ntok       = split_b_ntok;
  end

  always_comb begin
    // 默认保持寄存器状态；每个状态只改自己负责的字段，避免锁存器。
    st_d        = st_q;
    solo_idx_d  = solo_idx_q;
    best_cost_d = best_cost_q;
    cost_d      = cost_q;
    split_s2pf_start = 1'b0;

    unique case (st_q)
      ST_IDLE: begin
        if (start_i) begin
          if (rem_len_i == NR_W'(1)) begin
            // 只有最后剩一个 expert 时才进入 sim1 精确收尾评估。
            best_cost_d = INF_T;
            solo_idx_d  = 2'd0;
            st_d        = ST_SIM1_SOLO;
          end else begin
            // 其它 rem_len 直接走一拍 fast path，不枚举 solo/split。
            cost_d = fast_cost(rem_len_i, te, tl, rem0_ntok_i, rem1_ntok_i,
                               total_conc_i, max_conc_i);
            st_d   = ST_DONE;
          end
        end
      end

      ST_SIM1_SOLO: begin
        logic [T_W-1:0] next_best;
        next_best = best_cost_q;
        // 当前 solo_idx 的 mk_snap 输出已经在组合路径上给出；
        // 如果该 solo case 合法，就用 task_end 更新 best_cost。
        if (solo_valid) begin
          next_best = min_t(best_cost_q, mk_task_end);
        end
        best_cost_d = next_best;

        if (solo_idx_q == 2'd3) begin
          // 四个 solo case 都试完后，ntok>=2 才继续尝试 split 收尾。
          if (rem0_ntok_i >= NTOK_W'(2)) begin
            split_s2pf_start = 1'b1;
            st_d = ST_SIM1_SPLIT_S2PF;
          end else begin
            cost_d = (next_best == INF_T) ?
                     (t_now + best_task_ticks(rem0_ntok_i)) : next_best;
            st_d = ST_DONE;
          end
        end else begin
          // 继续枚举下一个 solo case。
          solo_idx_d = solo_idx_q + 2'd1;
        end
      end

      ST_SIM1_SPLIT_S2PF: begin
        logic [T_W-1:0] split_ms;
        logic [T_W-1:0] next_best;
        // split A/B snap 由组合 mk_snap 重算；这里只在 S2PF 搜索完成后读取结果。
        split_ms  = '0;
        next_best = best_cost_q;
        if (split_s2pf_done) begin
          split_ms  = max_t(split_pf_a.task_end, split_pf_b.task_end);
          if (split_ok) begin
            next_best = min_t(best_cost_q, split_ms);
          end
          cost_d = (next_best == INF_T) ?
                   (t_now + best_task_ticks(rem0_ntok_i)) : next_best;
          best_cost_d = next_best;
          st_d = ST_DONE;
        end
      end

      ST_DONE: begin
        // done_o 在本状态为 1。要求 start_i 已拉低后再回 IDLE，
        // 避免同一个 start pulse 被重复采样成新请求。
        if (!start_i) begin
          st_d = ST_IDLE;
        end
      end

      default: st_d = ST_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q        <= ST_IDLE;
      solo_idx_q  <= '0;
      best_cost_q <= INF_T;
      cost_q      <= '0;
    end else begin
      st_q        <= st_d;
      solo_idx_q  <= solo_idx_d;
      best_cost_q <= best_cost_d;
      cost_q      <= cost_d;
    end
  end

  // busy_o 表示 score 正在处理当前请求；done_o 表示 cost_o 当前有效。
  assign busy_o = (st_q != ST_IDLE) && (st_q != ST_DONE);
  assign done_o = (st_q == ST_DONE);
  assign cost_o = cost_q;

endmodule
