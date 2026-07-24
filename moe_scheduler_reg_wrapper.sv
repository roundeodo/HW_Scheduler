// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51
//
// Thin 64-bit regbus wrapper for the MoE scheduler datapath.
//
// The production protocol has seven contiguous words:
//   0x00 CONFIG        W  cache ids, active count, parallel/serial work
//   0x08 WINDOW0       W  sorted expert ranks 0..3
//   0x10 WINDOW1       W  sorted expert ranks 4..7
//   0x18 WINDOW2_START W  sorted expert ranks 8..11, then init and start
//   0x20 REFILL_QUAD   W  append 1..4 sorted experts to reserve
//   0x28 EVENT_WAIT    R  wait for refill, drain watermark, or batch completion
//   0x30 TASK_STREAM   R  return and atomically pop one dense 64-bit task
//
// WINDOW0..2 carry exactly the 192-bit head[6]+reserve[6] initial window.  The
// wrapper requests refill only at reserve low-water, so one REFILL_QUAD normally
// transports four experts.  A pending round result is acknowledged only when
// reserve can restore the next top6 window; the core therefore never observes a
// partially refilled round context.

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

  localparam logic [2:0] REG_CONFIG        = 3'd0;
  localparam logic [2:0] REG_WINDOW0       = 3'd1;
  localparam logic [2:0] REG_WINDOW1       = 3'd2;
  localparam logic [2:0] REG_WINDOW2_START = 3'd3;
  localparam logic [2:0] REG_REFILL_QUAD   = 3'd4;
  localparam logic [2:0] REG_EVENT_WAIT    = 3'd5;
  localparam logic [2:0] REG_TASK_STREAM   = 3'd6;

  localparam logic [3:0] TASK_DRAIN_WATERMARK = 4'd4;

  localparam int unsigned HEAD16_NTOK_LSB  = 0;
  localparam int unsigned HEAD16_EID_LSB   = HEAD16_NTOK_LSB + NTOK_W;
  localparam int unsigned HEAD16_VALID_LSB = HEAD16_EID_LSB + EID_RAW_W;

  // ── MMIO request decode ────────────────────────────────────────────────
  logic [2:0]  word_addr;
  logic        write_req;
  logic        read_req;
  logic [63:0] wr_data;
  logic [63:0] rd_data;
  logic        window2_start_req;
  logic        refill_write_req;
  logic        event_wait_read;
  logic        task_stream_read;
  logic        task_stream_pop;
  logic        event_pending;

  assign word_addr = reg_req_i.addr[5:3];
  assign write_req = reg_req_i.valid && reg_req_i.write;
  assign read_req  = reg_req_i.valid && !reg_req_i.write;
  assign wr_data   = reg_req_i.wdata[63:0];

  assign window2_start_req = write_req && (word_addr == REG_WINDOW2_START);
  assign refill_write_req  = write_req && (word_addr == REG_REFILL_QUAD);
  assign event_wait_read    = read_req && (word_addr == REG_EVENT_WAIT);
  assign task_stream_read   = read_req && (word_addr == REG_TASK_STREAM);

  // ── Persistent input-window state ──────────────────────────────────────
  pf_eid_t         initial_cache_eid_c2_q;
  pf_eid_t         initial_cache_eid_c3_q;
  head_ctx_t [5:0] head_q;
  head_ctx_t [5:0] reserve_q;
  logic [NR_W-1:0] active_count_q;
  time_t            total_parallel_work_q;
  time_t            total_serial_work_q;
  logic [2:0]       head_count_q;
  logic [2:0]       reserve_count_q;
  logic             auto_run_q;

  head_ctx_t [3:0] mmio_quad_entries;
  logic [2:0]      mmio_quad_valid_count;
  logic [1:0]      window1_head_valid_count;
  logic [1:0]      window1_reserve_valid_count;

  function automatic head_ctx_t unpack_head16(input logic [15:0] word);
    head_ctx_t descriptor;
    begin
      descriptor = '0;
      descriptor.ntok  = word[HEAD16_NTOK_LSB +: NTOK_W];
      descriptor.eid   = word[HEAD16_EID_LSB +: EID_RAW_W];
      descriptor.valid = word[HEAD16_VALID_LSB];
      unpack_head16 = descriptor;
    end
  endfunction

  function automatic logic [2:0] compact_count4(input logic [3:0] valid_mask);
    unique case (valid_mask)
      4'b0000: compact_count4 = 3'd0;
      4'b0001: compact_count4 = 3'd1;
      4'b0011: compact_count4 = 3'd2;
      4'b0111: compact_count4 = 3'd3;
      4'b1111: compact_count4 = 3'd4;
      default: compact_count4 = 3'd0;
    endcase
  endfunction

  assign mmio_quad_entries[0] = unpack_head16(wr_data[15:0]);
  assign mmio_quad_entries[1] = unpack_head16(wr_data[31:16]);
  assign mmio_quad_entries[2] = unpack_head16(wr_data[47:32]);
  assign mmio_quad_entries[3] = unpack_head16(wr_data[63:48]);
  assign mmio_quad_valid_count = compact_count4(
      {mmio_quad_entries[3].valid, mmio_quad_entries[2].valid,
       mmio_quad_entries[1].valid, mmio_quad_entries[0].valid});
  // WINDOW1 is one four-entry sorted prefix split after two entries.  Reuse the
  // quad count instead of decoding its two halves with two extra case trees.
  assign window1_head_valid_count = (mmio_quad_valid_count >= 3'd2) ?
                                    2'd2 : mmio_quad_valid_count[1:0];
  assign window1_reserve_valid_count = (mmio_quad_valid_count > 3'd2) ?
                                       (mmio_quad_valid_count[1:0] - 2'd2) : 2'd0;

  // ── schedule_core interface ────────────────────────────────────────────
  logic       init_pulse;
  logic       start_pulse;
  logic       remove_ready_pulse;
  logic       remove_valid;
  logic [1:0] remove_count;
  logic [3:0] remove_slot_mask;
  time_t      removed_parallel_work;
  time_t      removed_serial_work;

  logic        task_fifo_valid;
  logic [63:0] task_fifo_read_data;
  logic [3:0]  task_fifo_count;
  logic        task_fifo_full;
  logic        busy;

  // ── Fixed compact/refill projection ────────────────────────────────────
  head_ctx_t [5:0] next_head_after_commit;
  logic [2:0]      survivor_count;
  logic [2:0]      reserve_pop_count;
  logic [2:0]      next_head_count;
  logic [2:0]      current_target_head_count;
  logic [2:0]      target_head_after_remove;
  logic [2:0]      head_refill_after_remove;
  logic [2:0]      reserve_count_after_remove;
  logic [3:0]      loaded_count_after_remove;
  logic [NR_W-1:0] unloaded_count_after_remove;
  logic            next_round_lookahead_ready;
  logic [NR_W-1:0] active_after_remove;
  logic            current_head_ready;
  logic            remove_resources_ready;
  logic            auto_remove_pulse;
  logic            auto_start_after_remove;
  logic            auto_resume_pulse;
  logic            refill_req;
  logic [2:0]      refill_count;
  logic [3:0]      loaded_expert_count;
  logic [NR_W-1:0] unloaded_expert_count;

  always_comb begin
    next_head_after_commit = head_q;
    survivor_count = head_count_q;

    unique case (remove_slot_mask)
      4'b0001: begin
        next_head_after_commit[0] = head_q[1];
        next_head_after_commit[1] = head_q[2];
        next_head_after_commit[2] = head_q[3];
        next_head_after_commit[3] = head_q[4];
        next_head_after_commit[4] = head_q[5];
        next_head_after_commit[5] = '0;
        survivor_count = head_count_q - 3'd1;
      end
      4'b0011: begin
        next_head_after_commit[0] = head_q[2];
        next_head_after_commit[1] = head_q[3];
        next_head_after_commit[2] = head_q[4];
        next_head_after_commit[3] = head_q[5];
        next_head_after_commit[4] = '0;
        next_head_after_commit[5] = '0;
        survivor_count = head_count_q - 3'd2;
      end
      4'b0110: begin
        next_head_after_commit[0] = head_q[0];
        next_head_after_commit[1] = head_q[3];
        next_head_after_commit[2] = head_q[4];
        next_head_after_commit[3] = head_q[5];
        next_head_after_commit[4] = '0;
        next_head_after_commit[5] = '0;
        survivor_count = head_count_q - 3'd2;
      end
      4'b1100: begin
        next_head_after_commit[0] = head_q[0];
        next_head_after_commit[1] = head_q[1];
        next_head_after_commit[2] = head_q[4];
        next_head_after_commit[3] = head_q[5];
        next_head_after_commit[4] = '0;
        next_head_after_commit[5] = '0;
        survivor_count = head_count_q - 3'd2;
      end
      default: begin
      end
    endcase

    active_after_remove = remove_valid ?
        (active_count_q - NR_W'(remove_count)) : active_count_q;
    current_target_head_count = (active_count_q > NR_W'(6)) ?
                                3'd6 : active_count_q[2:0];
    target_head_after_remove = (active_after_remove > NR_W'(6)) ?
                               3'd6 : active_after_remove[2:0];
    head_refill_after_remove =
        (target_head_after_remove > survivor_count) ?
        (target_head_after_remove - survivor_count) : 3'd0;
    reserve_pop_count = head_refill_after_remove;

    if (reserve_count_q >= reserve_pop_count) begin
      reserve_count_after_remove = reserve_count_q - reserve_pop_count;
    end else begin
      reserve_count_after_remove = '0;
    end
    loaded_count_after_remove =
        {1'b0, target_head_after_remove} +
        {1'b0, reserve_count_after_remove};
    unloaded_count_after_remove =
        (active_after_remove > NR_W'(loaded_count_after_remove)) ?
        (active_after_remove - NR_W'(loaded_count_after_remove)) : '0;

    // If descriptors still remain outside the 6+6 window, keep two reserve
    // entries after this commit.  The following round may consume two experts;
    // admitting it with a smaller lookahead would make its result depend on
    // when software services REFILL_QUAD.
    next_round_lookahead_ready =
        (unloaded_count_after_remove == NR_W'(0)) ||
        (reserve_count_after_remove >= 3'd2);

    // A legal candidate consumes at most two experts.  A commit becomes
    // visible only after reserve covers both the immediate head refill and the
    // next-round lookahead contract.
    remove_resources_ready = !remove_valid ||
        (active_after_remove == NR_W'(0)) ||
        ((reserve_count_q >= reserve_pop_count) &&
         next_round_lookahead_ready);

    unique case (remove_slot_mask)
      4'b0001: begin
        if (reserve_pop_count == 3'd1) begin
          next_head_after_commit[5] = reserve_q[0];
        end
      end
      4'b0011, 4'b0110, 4'b1100: begin
        if (reserve_pop_count >= 3'd1) begin
          next_head_after_commit[4] = reserve_q[0];
        end
        if (reserve_pop_count == 3'd2) begin
          next_head_after_commit[5] = reserve_q[1];
        end
      end
      default: begin
      end
    endcase

    next_head_count = survivor_count + reserve_pop_count;
    current_head_ready = (head_count_q >= current_target_head_count);

    // reserve_count<=2 guarantees at least four free entries.  The request is
    // therefore naturally a full 64-bit transfer except for the final tail.
    loaded_expert_count = {1'b0, head_count_q} + {1'b0, reserve_count_q};
    unloaded_expert_count = (active_count_q > NR_W'(loaded_expert_count)) ?
        (active_count_q - NR_W'(loaded_expert_count)) : '0;
    refill_req = auto_run_q && (reserve_count_q <= 3'd2) &&
                 (unloaded_expert_count != NR_W'(0));
    refill_count = '0;
    if (refill_req) begin
      refill_count = (unloaded_expert_count >= NR_W'(4)) ?
                     3'd4 : unloaded_expert_count[2:0];
    end
  end

  assign auto_remove_pulse = auto_run_q && remove_valid &&
                             !refill_write_req && remove_resources_ready;
  assign auto_start_after_remove = auto_remove_pulse &&
                                   (active_after_remove != NR_W'(0)) &&
                                   !task_fifo_full;
  assign auto_resume_pulse = auto_run_q && !busy && !remove_valid &&
                             !refill_write_req &&
                             (active_count_q != NR_W'(0)) &&
                             current_head_ready && !task_fifo_full;

  assign init_pulse         = window2_start_req;
  assign start_pulse        = window2_start_req || auto_start_after_remove ||
                              auto_resume_pulse;
  assign remove_ready_pulse = auto_remove_pulse;

  // ── Event and streaming read protocol ──────────────────────────────────
  assign event_pending = refill_req ||
                         (task_fifo_count >= TASK_DRAIN_WATERMARK) ||
                         (active_count_q == NR_W'(0));

  always_comb begin
    rd_data = '0;
    unique case (word_addr)
      REG_EVENT_WAIT: begin
        rd_data[0]    = (active_count_q == NR_W'(0));
        rd_data[1]    = refill_req;
        rd_data[4:2]  = refill_count;
        rd_data[8:5]  = task_fifo_count;
      end
      REG_TASK_STREAM: begin
        rd_data = task_fifo_read_data;
      end
      default: begin
      end
    endcase
  end

  always_comb begin
    reg_rsp_o.rdata = rd_data;
    reg_rsp_o.error = 1'b0;
    reg_rsp_o.ready = 1'b1;
    if (event_wait_read) begin
      reg_rsp_o.ready = event_pending;
    end else if (task_stream_read) begin
      reg_rsp_o.ready = task_fifo_valid;
    end
  end

  assign task_stream_pop = task_stream_read && reg_rsp_o.ready;

  // ── Wrapper state update ────────────────────────────────────────────────
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      initial_cache_eid_c2_q <= PF_EID_NONE;
      initial_cache_eid_c3_q <= PF_EID_NONE;
      head_q                  <= '{default: '0};
      reserve_q               <= '{default: '0};
      active_count_q          <= '0;
      total_parallel_work_q   <= '0;
      total_serial_work_q     <= '0;
      head_count_q            <= '0;
      reserve_count_q         <= '0;
      auto_run_q              <= 1'b0;
    end else begin
      if (write_req) begin
        unique case (word_addr)
          REG_CONFIG: begin
            initial_cache_eid_c2_q <= wr_data[7] ? PF_EID_NONE :
                                              encode_eid(wr_data[EID_RAW_W-1:0]);
            initial_cache_eid_c3_q <= wr_data[15] ? PF_EID_NONE :
                                              encode_eid(wr_data[8 +: EID_RAW_W]);
            active_count_q        <= wr_data[16 +: NR_W];
            total_parallel_work_q <= wr_data[32 +: T_W];
            total_serial_work_q   <= wr_data[48 +: T_W];
          end

          REG_WINDOW0: begin
            head_q[0] <= mmio_quad_entries[0];
            head_q[1] <= mmio_quad_entries[1];
            head_q[2] <= mmio_quad_entries[2];
            head_q[3] <= mmio_quad_entries[3];
            head_q[4] <= '0;
            head_q[5] <= '0;
            head_count_q <= mmio_quad_valid_count;
          end

          REG_WINDOW1: begin
            head_q[4]    <= mmio_quad_entries[0];
            head_q[5]    <= mmio_quad_entries[1];
            reserve_q[0] <= mmio_quad_entries[2];
            reserve_q[1] <= mmio_quad_entries[3];
            reserve_q[2] <= '0;
            reserve_q[3] <= '0;
            reserve_q[4] <= '0;
            reserve_q[5] <= '0;
            head_count_q <= head_count_q + 3'(window1_head_valid_count);
            reserve_count_q <= 3'(window1_reserve_valid_count);
          end

          REG_WINDOW2_START: begin
            reserve_q[2] <= mmio_quad_entries[0];
            reserve_q[3] <= mmio_quad_entries[1];
            reserve_q[4] <= mmio_quad_entries[2];
            reserve_q[5] <= mmio_quad_entries[3];
            reserve_count_q <= reserve_count_q + mmio_quad_valid_count;
          end

          REG_REFILL_QUAD: begin
            // Fast protocol only writes refill at reserve low-water (0..2).
            unique case (reserve_count_q)
              3'd0: begin
                reserve_q[0] <= mmio_quad_entries[0];
                reserve_q[1] <= mmio_quad_entries[1];
                reserve_q[2] <= mmio_quad_entries[2];
                reserve_q[3] <= mmio_quad_entries[3];
              end
              3'd1: begin
                reserve_q[1] <= mmio_quad_entries[0];
                reserve_q[2] <= mmio_quad_entries[1];
                reserve_q[3] <= mmio_quad_entries[2];
                reserve_q[4] <= mmio_quad_entries[3];
              end
              3'd2: begin
                reserve_q[2] <= mmio_quad_entries[0];
                reserve_q[3] <= mmio_quad_entries[1];
                reserve_q[4] <= mmio_quad_entries[2];
                reserve_q[5] <= mmio_quad_entries[3];
              end
              default: begin
              end
            endcase
            reserve_count_q <= reserve_count_q + mmio_quad_valid_count;
          end

          default: begin
          end
        endcase
      end

      if (remove_ready_pulse) begin
        head_q                 <= next_head_after_commit;
        head_count_q           <= next_head_count;
        active_count_q         <= active_after_remove;
        total_parallel_work_q  <= total_parallel_work_q - removed_parallel_work;
        total_serial_work_q    <= total_serial_work_q - removed_serial_work;
        reserve_count_q        <= reserve_count_q - reserve_pop_count;
        unique case (reserve_pop_count)
          3'd0: begin
          end
          3'd1: begin
            reserve_q[0] <= reserve_q[1];
            reserve_q[1] <= reserve_q[2];
            reserve_q[2] <= reserve_q[3];
            reserve_q[3] <= reserve_q[4];
            reserve_q[4] <= reserve_q[5];
            reserve_q[5] <= '0;
          end
          3'd2: begin
            reserve_q[0] <= reserve_q[2];
            reserve_q[1] <= reserve_q[3];
            reserve_q[2] <= reserve_q[4];
            reserve_q[3] <= reserve_q[5];
            reserve_q[4] <= '0;
            reserve_q[5] <= '0;
          end
          default: begin
          end
        endcase
      end

      if (window2_start_req) begin
        auto_run_q <= 1'b1;
      end else if (remove_ready_pulse &&
                   (active_after_remove == NR_W'(0))) begin
        auto_run_q <= 1'b0;
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni) begin
      assert (head_count_q <= 3'd6);
      assert (reserve_count_q <= 3'd6);
      assert (task_fifo_count <= 4'(TASKQ_DEPTH));
      if (remove_ready_pulse) begin
        assert (sched_candidate_pkg::cand_remove_mask_legal(remove_slot_mask));
        assert (remove_resources_ready);
      end
      if (refill_write_req) begin
        assert (reserve_count_q <= 3'd2);
        assert (mmio_quad_valid_count == refill_count);
      end
      if (task_stream_pop) begin
        assert (task_fifo_valid);
      end
    end
  end
`endif

  moe_scheduler_core i_scheduler_core (
    .clk_i                    (clk_i),
    .rst_ni                   (rst_ni),
    .init_i                   (init_pulse),
    .start_i                  (start_pulse),
    .initial_cache_eid_c2_i   (initial_cache_eid_c2_q),
    .initial_cache_eid_c3_i   (initial_cache_eid_c3_q),
    .head_i                   (head_q),
    .active_count_i           (active_count_q),
    .total_parallel_work_i    (total_parallel_work_q),
    .total_serial_work_i      (total_serial_work_q),
    .remove_ready_i           (remove_ready_pulse),
    .remove_valid_o           (remove_valid),
    .remove_count_o           (remove_count),
    .remove_slot_mask_o       (remove_slot_mask),
    .remove_parallel_work_o   (removed_parallel_work),
    .remove_serial_work_o     (removed_serial_work),
    .task_fifo_pop_i          (task_stream_pop),
    .task_fifo_valid_o        (task_fifo_valid),
    .task_fifo_read_data_o    (task_fifo_read_data),
    .task_fifo_full_o         (task_fifo_full),
    .task_fifo_count_o        (task_fifo_count),
    .busy_o                   (busy)
  );

endmodule
