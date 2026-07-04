// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// Thin regbus wrapper for the MoE scheduler datapath.
//
// 这个文件不是单纯的 register bank；它承担 CVA6 MMIO 协议和 scheduler
// datapath 之间的运行时状态管理，所以代码会比较长。主要包含五类逻辑：
//
//   1. MMIO 地址译码：把 64-bit regbus 读写转换成 init/start/pop/refill 等脉冲。
//   2. 输入窗口管理：保存当前 top4 head 和 reserve[4]，每轮 commit 后 compact/refill。
//   3. 自动连续运行：一轮 done 后，如果 head/reserve 足够，自动启动下一轮。
//   4. 输出 FIFO 暴露：把 schedule_core 的 depth=4 plan queue 映射成 indexed MMIO entry。
//
// S4PF inline patch 已在 sched_schedule_core enqueue plan entry 时生成并写进
// PLAN_ENTRY_DATAx[63:56]。wrapper read path 只做 indexed mux，不再做 read-time
// walk 或 patch 推导。
//
// The SoC side is expected to use:
//   soc_narrow_xbar.out_moe_scheduler -> atomic filter -> axi_to_reg -> this module
//
// Register map, 64-bit word addressed:
//   0x00 CTRL      W: bit0 init, bit1 start
//   0x08 STATUS    R: bit0 busy, bit1 done_sticky, bit2 remove_valid,
//                     bit3 plan_valid, bit4 plan_queue_full,
//                     bit5 active_empty, bit6 refill_req,
//                     bits[9:8] remove_count, bits[17:16] plan_count,
//                     bits[25:24] head plan_slot_valid,
//                     bits[31:28] refill_count, bits[35:32] reserve_count,
//                     bits[39:36] plan_fifo_count,
//                     bits[47:40] plan_count for entries 0..3
//   0x10 CONFIG    R/W: bits[7:0] cache_eid_c2, bits[15:8] cache_eid_c3,
//                       bits[16 +: NR_W] active_count, bits[32 +: T_W] total_conc
//   0x68 ROUND_COMMIT W: bits[2:0] pop_count
//   0xa8 HEAD_QUAD        W: compact head16 x4 for initial top4
//   0xb0 RESERVE_QUAD     W: compact head16 x4 for initial reserve[4]
//   0xb8 HEAD_PUSH_QUAD   W: compact head16 x4 appended to reserve[4]
//   0x100 + i*0x20 PLAN_ENTRY_DATA0 R: FIFO entry i task 0, including inline S4PF patch
//   0x108 + i*0x20 PLAN_ENTRY_DATA1 R: FIFO entry i task 1, including inline S4PF patch
//                            head16 layout: ntok, eid, valid.

// 默认 regbus request/response 类型。SoC 集成时可以通过 parameter type 换成
// axi_to_reg 真实输出结构；本地 testbench 可以直接用这里的默认结构。
typedef struct packed {
  logic [47:0] addr;
  logic        write;
  logic [63:0] wdata;
  logic [7:0]  wstrb;
  logic        valid;
} moe_scheduler_default_reg_req_t;

typedef struct packed {
  logic [63:0] rdata;
  logic        error;
  logic        ready;
} moe_scheduler_default_reg_rsp_t;

module moe_scheduler_reg_wrapper
  import sched_pkg::*;
#(
  parameter type reg_req_t = moe_scheduler_default_reg_req_t,
  parameter type reg_rsp_t = moe_scheduler_default_reg_rsp_t
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  input  reg_req_t reg_req_i,
  output reg_rsp_t reg_rsp_o
);

  // ────────────────────────────────────────────────────────────────────────
  // 1. 64-bit word-addressed register map
  // ────────────────────────────────────────────────────────────────────────
  // word_addr = byte_addr[8:3]。例如 REG_STATUS=6'h01 对应 byte offset 0x08。
  // PLAN_ENTRY_BASE 之后每个 FIFO entry 使用 0x20 byte stride，当前只用 DATA0/1。
  localparam logic [5:0] REG_CTRL             = 6'h00;
  localparam logic [5:0] REG_STATUS           = 6'h01;
  localparam logic [5:0] REG_CONFIG           = 6'h02;
  localparam logic [5:0] REG_ROUND_COMMIT     = 6'h0d;
  localparam logic [5:0] REG_HEAD_QUAD        = 6'h15;
  localparam logic [5:0] REG_RESERVE_QUAD     = 6'h16;
  localparam logic [5:0] REG_HEAD_PUSH_QUAD   = 6'h17;
  localparam logic [5:0] REG_PLAN_ENTRY_BASE  = 6'h20;

  // ────────────────────────────────────────────────────────────────────────
  // 2. MMIO payload bit layout
  // ────────────────────────────────────────────────────────────────────────
  // PLAN_ENTRY_DATAx 的 64-bit payload 在 sched_schedule_core enqueue 时已完成
  // pack；这里不再在 read path 计算 shape/lowering/S4PF patch。
  localparam int unsigned HEAD16_NTOK_LSB  = 0;
  localparam int unsigned HEAD16_EID_LSB   = HEAD16_NTOK_LSB + NTOK_W;
  localparam int unsigned HEAD16_VALID_LSB = HEAD16_EID_LSB + EID_RAW_W;

  // ────────────────────────────────────────────────────────────────────────
  // 3. MMIO request decode and local control pulses
  // ────────────────────────────────────────────────────────────────────────
  // 这些信号都是组合译码结果，不是持久状态。真正状态都在后面的 *_q 中。
  logic [5:0]  word_addr;
  logic        write_req;
  logic [63:0] wr_data;
  logic [63:0] rd_data;
  logic        addr_hit;
  logic        plan_entry_read;
  logic [5:0]  plan_entry_delta;
  logic [1:0]  plan_entry_idx;
  logic [1:0]  plan_entry_sub;

  logic init_pulse;
  logic start_pulse;
  logic remove_ready_pulse;
  logic [2:0] plan_pop_count;
  logic [2:0] plan_pop_count_eff;
  logic [2:0] plan_fifo_count_sat;
  logic auto_run_q;
  logic ctrl_init_pulse;
  logic ctrl_start_pulse;
  logic round_commit_req;
  logic head_push_req;

  // ────────────────────────────────────────────────────────────────────────
  // 4. Persistent wrapper state
  // ────────────────────────────────────────────────────────────────────────
  // cache_eid_*_q: batch 开始时 C2/C3 中已驻留的 expert id。
  // 8'hff 表示 invalid/no cache；低 EID_RAW_W bit 是 raw expert id。
  logic [7:0]      cache_eid_c2_q;
  logic [7:0]      cache_eid_c3_q;
  // head/reserve 是 wrapper 持久 top window。best_conc 是 ntok 的纯函数，
  // 不作为持久状态保存，remove 时按固定 mask 组合计算。
  head_ctx_t [3:0] head_q;
  head_ctx_t [3:0] reserve_q;
  // compact_head / commit_head_next / push_head 都是组合中间结果：
  //   compact_head      = remove 后保序压缩出来的 top window；
  //   commit_head_next  = compact 后再从 reserve tail refill 的下一轮 head；
  //   push_head         = CVA6 HEAD_PUSH_QUAD 写进来的最多4个新 expert。
  // 它们不跨 cycle 保存，只有 remove/write 时才会被写入 head_q/reserve_q。
  head_ctx_t [3:0] compact_head;
  head_ctx_t [3:0] commit_head_next;
  head_ctx_t [3:0] push_head;
  time_t           head_bc_comb [3:0];
  // active_count_q: 当前 batch 还未被 scheduler 消耗的 active expert 数。
  // total_conc_q: 这些 active expert 的 best_conc 总和，用于 continuation cost。
  logic [NR_W-1:0] active_count_q;
  logic [T_W-1:0]  total_conc_q;
  logic [2:0]      head_count_q;
  logic [2:0]      reserve_count_q;
  logic [2:0]      push_count;
  logic [3:0]      reserve_count_after_push;
  // remove/compact/refill 的组合控制信号。
  // removed_conc 用于从 total_conc_q 中扣掉本轮被消耗 expert 的 best_conc。
  logic [T_W+1:0]  removed_conc;
  logic [2:0]      keep_count;
  logic [2:0]      reserve_pop_count;
  logic [2:0]      target_head_count;
  logic [2:0]      fill_need;
  logic [NR_W-1:0] active_after_remove;
  logic            head_complete_current;
  logic            head_complete_after_remove;
  logic            auto_remove_pulse;
  logic            auto_start_after_remove;
  logic            auto_resume_pulse;
  logic            refill_req;
  logic [3:0]      refill_count;

  // ────────────────────────────────────────────────────────────────────────
  // 5. schedule_core outputs exposed by this wrapper
  // ────────────────────────────────────────────────────────────────────────
  // plan_* signals 由 sched_schedule_core 持有；wrapper 只负责 MMIO indexed mux
  // 以及向 core 反馈 CVA6 已 pop 多少 FIFO entry。
  logic                         remove_valid;
  logic [1:0]                   remove_count;
  logic [3:0]                   remove_slot_mask;
  logic                         plan_valid;
  logic [PLANQ_DEPTH-1:0][1:0]  plan_slot_valid;
  logic [PLANQ_DEPTH-1:0][1:0][63:0] plan_data;
  logic [PLANQ_DEPTH-1:0][1:0]  plan_count;
  logic [3:0]                   plan_fifo_count;
  logic                         plan_queue_full;
  logic                         busy;
  logic                         done;
  logic                         done_seen_q;

  // ────────────────────────────────────────────────────────────────────────
  // 6. Small pack/unpack helpers
  // ────────────────────────────────────────────────────────────────────────
  // HEAD16 是 CVA6 写入 top/reserve 的 compact expert 描述：
  //   {valid, eid, ntok}，总共 16 bit。
  function automatic head_ctx_t unpack_head16(input logic [15:0] word);
    head_ctx_t ctx;
    begin
      ctx = '0;
      ctx.ntok  = word[HEAD16_NTOK_LSB +: NTOK_W];
      ctx.eid   = word[HEAD16_EID_LSB +: EID_RAW_W];
      ctx.valid = word[HEAD16_VALID_LSB];
      unpack_head16 = ctx;
    end
  endfunction

  function automatic logic [2:0] popcount4(input logic [3:0] v);
    logic [1:0] cnt01;
    logic [1:0] cnt23;
    begin
      cnt01 = {1'b0, v[0]} + {1'b0, v[1]};
      cnt23 = {1'b0, v[2]} + {1'b0, v[3]};
      popcount4 = {1'b0, cnt01} + {1'b0, cnt23};
    end
  endfunction

  // ────────────────────────────────────────────────────────────────────────
  // 7. MMIO write decode
  // ────────────────────────────────────────────────────────────────────────
  // CTRL 写入只产生一拍 pulse；CONFIG/HEAD/RESERVE 写入会在 always_ff 中落寄存器。
  assign word_addr          = reg_req_i.addr[8:3];
  assign write_req          = reg_req_i.valid && reg_req_i.write;
  assign wr_data            = reg_req_i.wdata[63:0];
  assign ctrl_init_pulse    = write_req && (word_addr == REG_CTRL) && wr_data[0];
  assign ctrl_start_pulse   = write_req && (word_addr == REG_CTRL) && wr_data[1];
  assign round_commit_req   = write_req && (word_addr == REG_ROUND_COMMIT);
  assign head_push_req      = write_req && (word_addr == REG_HEAD_PUSH_QUAD);
  // CVA6 通过 ROUND_COMMIT.pop_count 告诉 wrapper：已经读走 FIFO 前几个 entry。
  // pop_count_eff 会被饱和到当前 FIFO count，避免非法 pop 影响内部状态。
  assign plan_pop_count     = round_commit_req ? wr_data[2:0] : 3'd0;
  assign plan_fifo_count_sat = (plan_fifo_count > 4'd4) ? 3'd4 : plan_fifo_count[2:0];
  assign plan_pop_count_eff = (plan_pop_count > plan_fifo_count_sat) ?
                              plan_fifo_count_sat : plan_pop_count;
  // PLAN_ENTRY_DATA0/1 的地址解析：
  //   plan_entry_idx = FIFO entry index, 0..3；
  //   plan_entry_sub = 0 表示 task0，1 表示 task1。
  assign plan_entry_read    = (word_addr >= REG_PLAN_ENTRY_BASE) &&
                              (word_addr < (REG_PLAN_ENTRY_BASE + 6'(PLANQ_DEPTH * 4)));
  assign plan_entry_delta   = word_addr - REG_PLAN_ENTRY_BASE;
  assign plan_entry_idx     = plan_entry_delta[3:2];
  assign plan_entry_sub     = plan_entry_delta[1:0];

  assign init_pulse         = ctrl_init_pulse;
  // start_pulse 可能来自 CVA6 显式写 CTRL.start，也可能来自 auto-run。
  assign start_pulse        = ctrl_start_pulse || auto_start_after_remove ||
                              auto_resume_pulse;
  assign remove_ready_pulse = auto_remove_pulse;
  assign push_head[0]       = unpack_head16(wr_data[15:0]);
  assign push_head[1]       = unpack_head16(wr_data[31:16]);
  assign push_head[2]       = unpack_head16(wr_data[47:32]);
  assign push_head[3]       = unpack_head16(wr_data[63:48]);
  assign push_count         = popcount4({push_head[3].valid, push_head[2].valid,
                                         push_head[1].valid, push_head[0].valid});
  assign reserve_count_after_push = {1'b0, reserve_count_q} + {1'b0, push_count};
  for (genvar hbc = 0; hbc < 4; hbc++) begin : gen_head_bc_comb
    assign head_bc_comb[hbc] = head_q[hbc].valid ?
                               time_t'(best_conc_ticks(head_q[hbc].ntok)) : '0;
  end

  // ────────────────────────────────────────────────────────────────────────
  // 8. Top4 compact/refill control
  // ────────────────────────────────────────────────────────────────────────
  // Lite policy only removes {top0}, {top0,top1}, {top1,top2}, or {top2,top3}.
  // Use fixed-case compaction instead of generic prefix/rank + variable index
  // writes.  This maps to small muxes on the normal path and reduces fanout.
  always_comb begin
    compact_head = '{default: '0};
    commit_head_next = head_q;
    removed_conc = '0;
    keep_count = '0;
    reserve_pop_count = '0;
    active_after_remove = active_count_q;
    target_head_count = '0;
    fill_need = '0;
    refill_req = 1'b0;
    refill_count = '0;
    head_complete_current = 1'b0;
    head_complete_after_remove = 1'b0;

    unique case (remove_slot_mask)
      4'b0000: begin
        compact_head = head_q;
        keep_count   = head_count_q;
        removed_conc = '0;
      end
      4'b0001: begin
        compact_head[0] = head_q[1];
        compact_head[1] = head_q[2];
        compact_head[2] = head_q[3];
        keep_count      = head_count_q - 3'd1;
        removed_conc    = {2'b0, head_bc_comb[0]};
      end
      4'b0011: begin
        compact_head[0] = head_q[2];
        compact_head[1] = head_q[3];
        keep_count      = head_count_q - 3'd2;
        removed_conc    = {2'b0, head_bc_comb[0]} + {2'b0, head_bc_comb[1]};
      end
      4'b0110: begin
        compact_head[0] = head_q[0];
        compact_head[1] = head_q[3];
        keep_count      = head_count_q - 3'd2;
        removed_conc    = {2'b0, head_bc_comb[1]} + {2'b0, head_bc_comb[2]};
      end
      4'b1100: begin
        compact_head[0] = head_q[0];
        compact_head[1] = head_q[1];
        keep_count      = head_count_q - 3'd2;
        removed_conc    = {2'b0, head_bc_comb[2]} + {2'b0, head_bc_comb[3]};
      end
      default: begin
        // 非法 remove mask。正常 fast path 只允许
        // 0001/0011/0110/1100；这里仅保持组合默认值，避免 latch。
        compact_head = head_q;
        keep_count   = head_count_q;
        removed_conc = '0;
      end
    endcase

    // 没有 remove 时，下一轮 head 保持不变；有 remove 时，先使用 compact 结果。
    commit_head_next = remove_ready_pulse ? compact_head : head_q;

    if (remove_ready_pulse) begin
      // remove 后更新 active 数，并计算下一轮理论上应该持有多少 head：
      // active_after_remove >=4 时需要 top4，否则只需要剩余 active 数。
      active_after_remove = active_count_q - NR_W'(remove_count);
      target_head_count = (active_after_remove > NR_W'(4)) ? 3'd4 :
                          active_after_remove[2:0];
      fill_need = (target_head_count > keep_count) ?
                  (target_head_count - keep_count) : 3'd0;
      // reserve_pop_count 表示这次从 reserve 队头搬几个 expert 到 compact 后的 head 尾部。
      reserve_pop_count = (fill_need > reserve_count_q) ? reserve_count_q : fill_need;
      unique case (remove_slot_mask)
        4'b0001: begin
          if (reserve_pop_count != 3'd0) begin
            commit_head_next[3] = reserve_q[0];
          end
        end
        4'b0011, 4'b0110, 4'b1100: begin
          if (reserve_pop_count >= 3'd1) begin
            commit_head_next[2] = reserve_q[0];
          end
          if (reserve_pop_count >= 3'd2) begin
            commit_head_next[3] = reserve_q[1];
          end
        end
        default: begin
        end
      endcase
    end else begin
      active_after_remove = active_count_q;
      target_head_count = (active_count_q > NR_W'(4)) ? 3'd4 : active_count_q[2:0];
    end

    // head_complete_* 用于 auto-run 判断：如果当前/commit 后 head 已经够下一轮使用，
    // wrapper 可以不等 CVA6 再写 HEAD_PUSH，直接自动启动 schedule_core。
    head_complete_current = (head_count_q >= target_head_count);
    head_complete_after_remove =
        (remove_ready_pulse ? ((keep_count + reserve_pop_count) >= target_head_count) :
                              head_complete_current);

    // refill_req 告诉 CVA6：RTL 内部 head+reserve 已经装不满后续 active expert，
    // 需要从 L3 sorted expert stream 再 push 1..4 个到 reserve。
    if (active_count_q > NR_W'(head_count_q + reserve_count_q)) begin
      logic [2:0] free_slots;
      logic [NR_W-1:0] remaining_unloaded;
      free_slots = 3'd4 - reserve_count_q;
      remaining_unloaded = active_count_q - NR_W'(head_count_q + reserve_count_q);
      if (free_slots != 3'd0) begin
        refill_req = 1'b1;
        refill_count = (remaining_unloaded > NR_W'(free_slots)) ?
                       {1'b0, free_slots} : {1'b0, remaining_unloaded[2:0]};
      end
    end
  end

  // ────────────────────────────────────────────────────────────────────────
  // 10. Auto-run handshake
  // ────────────────────────────────────────────────────────────────────────
  // auto_run_q=1 后，CVA6 不需要每轮写 CTRL.start。wrapper 在 core done 后：
  //   1. 自动 acknowledge/remove 当前 round；
  //   2. 如果 head/reserve 足够且 plan FIFO 未满，自动启动下一轮。
  // head_push_req 会阻止同拍 auto remove，避免 CVA6 正在补 reserve 时状态同时移动。
  assign auto_remove_pulse = auto_run_q && done_seen_q && remove_valid && !head_push_req;
  assign auto_start_after_remove = auto_remove_pulse &&
                                   (active_after_remove != NR_W'(0)) &&
                                   head_complete_after_remove &&
                                   !plan_queue_full;
  assign auto_resume_pulse = auto_run_q && done_seen_q && !remove_valid && !head_push_req &&
                             (active_count_q != NR_W'(0)) &&
                             head_complete_current &&
                             !plan_queue_full;

  // ────────────────────────────────────────────────────────────────────────
  // 11. Wrapper state registers
  // ────────────────────────────────────────────────────────────────────────
  // 这个 always_ff 保存 wrapper 自己拥有的状态：
  //   - cache_eid / head / reserve / active_count / total_conc；
  //   - done_seen sticky bit；
  //   - auto_run enable。
  // schedule_core 内部 plan FIFO 不在这里保存，它通过输出端口暴露给 wrapper。
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cache_eid_c2_q <= 8'hff;
      cache_eid_c3_q <= 8'hff;
      head_q         <= '{default: '0};
      reserve_q      <= '{default: '0};
      active_count_q <= '0;
      total_conc_q   <= '0;
      head_count_q   <= '0;
      reserve_count_q <= '0;
      done_seen_q    <= 1'b0;
      auto_run_q     <= 1'b0;
    end else begin
      if (write_req) begin
        unique case (word_addr)
          REG_CONFIG: begin
            // Batch/init 配置：cache 初始驻留 expert、active_count 和 total_conc。
            cache_eid_c2_q <= wr_data[7:0];
            cache_eid_c3_q <= wr_data[15:8];
            active_count_q <= wr_data[16 +: NR_W];
            total_conc_q   <= wr_data[32 +: T_W];
          end
          REG_HEAD_QUAD: begin
            // 初始 top4 window。CVA6 一次写 4 个 compact head16。
            for (int h = 0; h < 4; h++) begin
              head_q[h] <= push_head[h];
            end
            head_count_q <= popcount4({wr_data[63], wr_data[47],
                                       wr_data[31], wr_data[15]});
          end
          REG_HEAD_PUSH_QUAD: begin
            // 后续 refill。协议要求 push_head 是 compact quad；append 位置只由
            // reserve_count_q 决定，用 fixed case 避免 variable-index write。
            unique case (reserve_count_q)
              3'd0: begin
                reserve_q[0] <= push_head[0];
                reserve_q[1] <= push_head[1];
                reserve_q[2] <= push_head[2];
                reserve_q[3] <= push_head[3];
              end
              3'd1: begin
                reserve_q[1] <= push_head[0];
                reserve_q[2] <= push_head[1];
                reserve_q[3] <= push_head[2];
              end
              3'd2: begin
                reserve_q[2] <= push_head[0];
                reserve_q[3] <= push_head[1];
              end
              3'd3: begin
                reserve_q[3] <= push_head[0];
              end
              default: begin
              end
            endcase
            reserve_count_q <= (reserve_count_after_push > 4'd4) ?
                               3'd4 : reserve_count_after_push[2:0];
          end
          REG_RESERVE_QUAD: begin
            // 初始化 reserve[4]，通常 batch 开始时写入 top4 之后的后续 expert。
            for (int r = 0; r < 4; r++) begin
              reserve_q[r] <= push_head[r];
            end
            reserve_count_q <= popcount4({wr_data[63], wr_data[47],
                                          wr_data[31], wr_data[15]});
          end
          default: ;
        endcase
      end

      if (remove_ready_pulse) begin
        // 一轮 commit 后，wrapper 接收 core 的 remove metadata：
        //   1. compact/refill head；
        //   2. active_count 扣掉 remove_count；
        //   3. total_conc 扣掉被消耗 expert 的 best_conc；
        //   4. reserve 队头左移 reserve_pop_count。
        head_q          <= commit_head_next;
        head_count_q    <= keep_count + reserve_pop_count;
        active_count_q  <= active_count_q - NR_W'(remove_count);
        total_conc_q    <= total_conc_q - T_W'(removed_conc);
        reserve_count_q <= reserve_count_q - reserve_pop_count;
        unique case (reserve_pop_count)
          3'd0: begin
          end
          3'd1: begin
            reserve_q[0] <= reserve_q[1];
            reserve_q[1] <= reserve_q[2];
            reserve_q[2] <= reserve_q[3];
            reserve_q[3] <= '0;
          end
          3'd2: begin
            reserve_q[0] <= reserve_q[2];
            reserve_q[1] <= reserve_q[3];
            reserve_q[2] <= '0;
            reserve_q[3] <= '0;
          end
          default: begin
            reserve_q <= '{default: '0};
          end
        endcase
      end

      // done_seen_q 是 sticky done。新 init/start 清零，core done 后置位。
      if (init_pulse || start_pulse) begin
        done_seen_q <= 1'b0;
      end else if (done) begin
        done_seen_q <= 1'b1;
      end

      // auto_run_q 控制 wrapper 是否在每轮 done 后自动 remove/start。
      if (init_pulse) begin
        auto_run_q <= ctrl_start_pulse;
      end else if (ctrl_start_pulse) begin
        auto_run_q <= 1'b1;
      end else if (remove_ready_pulse &&
                   ((active_count_q - NR_W'(remove_count)) == NR_W'(0))) begin
        auto_run_q <= 1'b0;
      end
    end
  end

  // ────────────────────────────────────────────────────────────────────────
  // 11. MMIO read mux
  // ────────────────────────────────────────────────────────────────────────
  // regbus 是 1-cycle ready，读数据完全组合产生。
  // PLAN_ENTRY_DATA0/1 已在 core enqueue 时 pack 好，read path 只做 indexed mux。
  always_comb begin
    rd_data  = '0;
    addr_hit = 1'b1;

    unique case (word_addr)
      REG_CTRL: begin
        // CTRL 是 write-only pulse register，读回 0。
        rd_data = '0;
      end
      REG_STATUS: begin
        // 主状态寄存器：CVA6 用它判断 busy/done、FIFO 数量、是否需要 refill。
        rd_data[0]      = busy;
        rd_data[1]      = done_seen_q;
        rd_data[2]      = remove_valid;
        rd_data[3]      = plan_valid;
        rd_data[4]      = plan_queue_full;
        rd_data[5]      = (active_count_q == NR_W'(0));
        rd_data[6]      = refill_req;
        rd_data[9:8]    = remove_count;
        rd_data[17:16]  = plan_count[0];
        rd_data[25:24]  = plan_slot_valid[0];
        rd_data[31:28]  = refill_count;
        rd_data[35:32]  = {1'b0, reserve_count_q};
        rd_data[39:36]  = plan_fifo_count;
        for (int q = 0; q < PLANQ_DEPTH; q++) begin
          rd_data[40 + q * 2 +: 2] = plan_count[q];
        end
      end
      REG_CONFIG: begin
        // 配置寄存器可读回，便于软件确认当前 batch 初始配置。
        rd_data[7:0]           = cache_eid_c2_q;
        rd_data[15:8]          = cache_eid_c3_q;
        rd_data[16 +: NR_W]    = active_count_q;
        rd_data[32 +: T_W]     = total_conc_q;
      end
      REG_ROUND_COMMIT, REG_HEAD_QUAD, REG_RESERVE_QUAD, REG_HEAD_PUSH_QUAD: begin
        // 这些是 write-only command/data register，读回 0。
        rd_data = '0;
      end
      default: begin
        if (plan_entry_read && (int'(plan_entry_idx) < PLANQ_DEPTH) &&
            (plan_entry_sub <= 2'd1)) begin
          // Indexed FIFO read。CVA6 可以读 entry 0..3 的 DATA0/DATA1；
          // 真正 pop 由后续 ROUND_COMMIT.pop_count 完成。
          rd_data = plan_data[plan_entry_idx][plan_entry_sub[0]];
        end else begin
          addr_hit = 1'b0;
        end
      end
    endcase
  end

  // regbus response：当前 wrapper 不插 wait state，所有访问 ready=1。
  // addr_hit=0 时返回 error，主要用于捕捉非法地址或 PLAN_ENTRY subword。
  assign reg_rsp_o.rdata = rd_data;
  assign reg_rsp_o.error = reg_req_i.valid && !addr_hit;
  assign reg_rsp_o.ready = 1'b1;

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni && remove_ready_pulse) begin
      assert (sched_candidate_pkg::cand_remove_mask_legal(remove_slot_mask))
        else $error("moe_scheduler_reg_wrapper received illegal remove_slot_mask=%b",
                    remove_slot_mask);
    end
  end
`endif

  // ────────────────────────────────────────────────────────────────────────
  // 13. Scheduler datapath core
  // ────────────────────────────────────────────────────────────────────────
  // wrapper 负责准备 head/cache/active_count，并把这些作为一轮调度输入送进 core。
  // core 负责真正的 candidate generation/eval/commit/FIFO enqueue。
  // wrapper 再通过 remove_ready_i 和 plan_pop_count_i 告诉 core：
  //   - 当前 round 的 remove metadata 已经被 wrapper 接收；
  //   - CVA6 已经 pop 了几个 FIFO entry。
  sched_schedule_core i_sched_schedule_core (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .init_i             (init_pulse),
    .start_i            (start_pulse),
    .cache_eid_c2_i     (cache_eid_c2_q),
    .cache_eid_c3_i     (cache_eid_c3_q),
    .head_i             (head_q),
    .active_count_i     (active_count_q),
    .total_conc_i       (total_conc_q),
    .remove_ready_i     (remove_ready_pulse),
    .remove_valid_o     (remove_valid),
    .remove_count_o     (remove_count),
    .remove_slot_mask_o (remove_slot_mask),
    .plan_pop_count_i   (plan_pop_count_eff),
    .plan_valid_o       (plan_valid),
    .plan_slot_valid_o  (plan_slot_valid),
    .plan_data_o        (plan_data),
    .plan_count_o       (plan_count),
    .plan_queue_full_o  (plan_queue_full),
    .plan_queue_count_o (plan_fifo_count),
    .busy_o             (busy),
    .done_o             (done)
  );

endmodule
