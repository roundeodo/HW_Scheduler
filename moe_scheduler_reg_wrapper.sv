// Copyright KU Leuven / MiCAS Lab
// SPDX-License-Identifier: SHL-0.51

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
  import sched_distilled_pkg::*;
#(
  parameter type reg_req_t = moe_scheduler_default_reg_req_t,
  parameter type reg_rsp_t = moe_scheduler_default_reg_rsp_t
) (
  input  logic     clk_i,
  input  logic     rst_ni,
  input  reg_req_t reg_req_i,
  output reg_rsp_t reg_rsp_o
);

  typedef struct packed {
    logic [DIST_TOKEN_SUM_W-1:0] token_sum;
    logic [DIST_HIST_W-1:0]      odd_count;
    logic [DIST_BLOCK_SUM_W-1:0] block_sum;
    logic [3:0][DIST_HIST_W-1:0] small_hist;
  } initial_aggregate_t;

  localparam logic [3:0] REG_CONFIG        = 4'h0;
  localparam logic [3:0] REG_WINDOW0       = 4'h1;
  localparam logic [3:0] REG_WINDOW1       = 4'h2;
  localparam logic [3:0] REG_WINDOW2       = 4'h3;
  localparam logic [3:0] REG_REFILL_QUAD   = 4'h4;
  localparam logic [3:0] REG_EVENT_WAIT    = 4'h5;
  localparam logic [3:0] REG_TASK_STREAM   = 4'h6;
  localparam logic [3:0] REG_AGGREGATE     = 4'h7;
  localparam logic [3:0] REG_WINDOW3_START = 4'h8;

  localparam int unsigned DESC_NTOK_LSB = 0;
  localparam int unsigned DESC_EID_LSB = DESC_NTOK_LSB + NTOK_W;
  localparam int unsigned DESC_VALID_LSB = DESC_EID_LSB + EID_RAW_W;

  logic [3:0] word_addr;
  logic write_req;
  logic read_req;
  logic [63:0] wr_data;
  logic window_start_write;
  logic refill_write;
  logic event_read;
  logic task_read;
  logic task_pop;

  assign word_addr = reg_req_i.addr[6:3];
  assign write_req = reg_req_i.valid && reg_req_i.write;
  assign read_req = reg_req_i.valid && !reg_req_i.write;
  assign wr_data = reg_req_i.wdata;
  assign refill_write = write_req && (word_addr == REG_REFILL_QUAD);
  assign event_read = read_req && (word_addr == REG_EVENT_WAIT);
  assign task_read = read_req && (word_addr == REG_TASK_STREAM);

  head_ctx_t [3:0] mmio_entries;
  logic [2:0] mmio_count;

  function automatic head_ctx_t unpack_descriptor(input logic [15:0] word);
    head_ctx_t descriptor;
    begin
      descriptor = '0;
      descriptor.ntok = word[DESC_NTOK_LSB +: NTOK_W];
      descriptor.eid = word[DESC_EID_LSB +: EID_RAW_W];
      descriptor.valid = word[DESC_VALID_LSB];
      unpack_descriptor = descriptor;
    end
  endfunction

  function automatic logic [2:0] prefix_count(input logic [3:0] valid);
    unique case (valid)
      4'b0000: prefix_count = 3'd0;
      4'b0001: prefix_count = 3'd1;
      4'b0011: prefix_count = 3'd2;
      4'b0111: prefix_count = 3'd3;
      4'b1111: prefix_count = 3'd4;
      default: prefix_count = 3'd0;
    endcase
  endfunction

  assign mmio_entries[0] = unpack_descriptor(wr_data[15:0]);
  assign mmio_entries[1] = unpack_descriptor(wr_data[31:16]);
  assign mmio_entries[2] = unpack_descriptor(wr_data[47:32]);
  assign mmio_entries[3] = unpack_descriptor(wr_data[63:48]);
  assign mmio_count = prefix_count({mmio_entries[3].valid,
                                    mmio_entries[2].valid,
                                    mmio_entries[1].valid,
                                    mmio_entries[0].valid});

  pf_eid_t initial_cache_c2_q;
  pf_eid_t initial_cache_c3_q;
  logic [NR_W-1:0] active_count_q;
  initial_aggregate_t initial_aggregate_q;
  distilled_counters_t initial_counters;
  head_ctx_t [HOT_CAPACITY-1:0] hot_q;
  head_ctx_t [COLD_CAPACITY-1:0] cold_q;
  logic [3:0] hot_count_q;
  logic [2:0] cold_count_q;
  logic auto_run_q;

  logic core_init;
  logic core_start;
  logic core_busy;
  logic remove_valid;
  logic remove_ready;
  logic [1:0] remove_count;
  logic [EID_RAW_W-1:0] remove_eid_a;
  logic [EID_RAW_W-1:0] remove_eid_b;
  logic task_valid;
  logic [63:0] task_data;
  logic task_full;
  logic [3:0] task_count;

  head_ctx_t [HOT_CAPACITY-1:0] compact_hot;
  head_ctx_t [COLD_CAPACITY-1:0] compact_cold;
  logic [3:0] compact_hot_count;
  logic [2:0] compact_cold_count;
  head_ctx_t [HOT_CAPACITY-1:0] next_hot;
  head_ctx_t [COLD_CAPACITY-1:0] next_cold;
  logic [3:0] next_hot_count;
  logic [2:0] next_cold_count;
  logic [NR_W-1:0] active_after_remove;
  logic [4:0] loaded_after_remove;
  logic [NR_W-1:0] hidden_after_remove;
  logic [3:0] desired_hot_after_remove;

  logic [4:0] loaded_count;
  logic [NR_W-1:0] hidden_count;
  logic [3:0] top_reserve_count;
  logic [2:0] bottom_reserve_count;
  logic refill_request;
  logic refill_active_q;
  logic [2:0] refill_top_count;
  logic [2:0] refill_bottom_count;
  logic [2:0] refill_top_remaining_q;
  logic [2:0] refill_bottom_remaining_q;
  logic [2:0] event_refill_top_count;
  logic [2:0] event_refill_bottom_count;
  logic [3:0] refill_remaining_count;
  logic [2:0] refill_top_take;
  logic [2:0] refill_bottom_take;
  logic [3:0] desired_hot_count;
  logic [2:0] desired_cold_count;
  logic [3:0] top_deficit;
  logic [2:0] bottom_deficit;
  logic [NR_W-1:0] hidden_after_top;
  logic [2:0] bottom_refill_budget;
  head_ctx_t [HOT_CAPACITY-1:0] refill_hot;
  head_ctx_t [COLD_CAPACITY-1:0] refill_cold;
  logic [3:0] refill_hot_count_next;
  logic [2:0] refill_cold_count_next;
  logic window_ready;
  logic event_pending;
  logic run_start;
  logic [3:0] initial_hot_count;
  logic [2:0] initial_cold_count;

  logic [HOT_CAPACITY-1:0] hot_keep;
  logic [COLD_CAPACITY-1:0] cold_keep;
  logic [3:0] hot_prefix [0:4][HOT_CAPACITY-1:0];
  logic [2:0] cold_prefix [0:3][COLD_CAPACITY-1:0];

  always_comb begin
    window_start_write = 1'b0;
    if (write_req) begin
      if (active_count_q <= NR_W'(4))
        window_start_write = word_addr == REG_WINDOW0;
      else if (active_count_q <= NR_W'(8))
        window_start_write = word_addr == REG_WINDOW1;
      else if (active_count_q <= NR_W'(12))
        window_start_write = word_addr == REG_WINDOW2;
      else
        window_start_write = word_addr == REG_WINDOW3_START;
    end
  end

  for (genvar slot = 0; slot < HOT_CAPACITY; slot++) begin : gen_hot_prefix
    assign hot_keep[slot] = hot_q[slot].valid &&
        (hot_q[slot].eid != remove_eid_a) &&
        ((remove_count != 2'd2) || (hot_q[slot].eid != remove_eid_b));
    assign hot_prefix[0][slot] = 4'(hot_keep[slot]);
    if (slot >= 1)
      assign hot_prefix[1][slot] = hot_prefix[0][slot] + hot_prefix[0][slot-1];
    else
      assign hot_prefix[1][slot] = hot_prefix[0][slot];
    if (slot >= 2)
      assign hot_prefix[2][slot] = hot_prefix[1][slot] + hot_prefix[1][slot-2];
    else
      assign hot_prefix[2][slot] = hot_prefix[1][slot];
    if (slot >= 4)
      assign hot_prefix[3][slot] = hot_prefix[2][slot] + hot_prefix[2][slot-4];
    else
      assign hot_prefix[3][slot] = hot_prefix[2][slot];
    if (slot >= 8)
      assign hot_prefix[4][slot] = hot_prefix[3][slot] + hot_prefix[3][slot-8];
    else
      assign hot_prefix[4][slot] = hot_prefix[3][slot];
  end

  for (genvar slot = 0; slot < COLD_CAPACITY; slot++) begin : gen_cold_prefix
    assign cold_keep[slot] = cold_q[slot].valid &&
        (cold_q[slot].eid != remove_eid_a) &&
        ((remove_count != 2'd2) || (cold_q[slot].eid != remove_eid_b));
    assign cold_prefix[0][slot] = 3'(cold_keep[slot]);
    if (slot >= 1)
      assign cold_prefix[1][slot] = cold_prefix[0][slot] + cold_prefix[0][slot-1];
    else
      assign cold_prefix[1][slot] = cold_prefix[0][slot];
    if (slot >= 2)
      assign cold_prefix[2][slot] = cold_prefix[1][slot] + cold_prefix[1][slot-2];
    else
      assign cold_prefix[2][slot] = cold_prefix[1][slot];
    if (slot >= 4)
      assign cold_prefix[3][slot] = cold_prefix[2][slot] + cold_prefix[2][slot-4];
    else
      assign cold_prefix[3][slot] = cold_prefix[2][slot];
  end

  always_comb begin
    compact_hot = '{default: '0};
    compact_cold = '{default: '0};
    compact_hot_count = hot_prefix[4][HOT_CAPACITY-1];
    compact_cold_count = cold_prefix[3][COLD_CAPACITY-1];
    for (int slot = 0; slot < HOT_CAPACITY; slot++) begin
      for (int destination = 0; destination < HOT_CAPACITY; destination++)
        if (hot_keep[slot] &&
            (hot_prefix[4][slot] == 4'(destination + 1)))
          compact_hot[destination] = hot_q[slot];
    end
    for (int slot = 0; slot < COLD_CAPACITY; slot++) begin
      for (int destination = 0; destination < COLD_CAPACITY; destination++)
        if (cold_keep[slot] &&
            (cold_prefix[3][slot] == 3'(destination + 1)))
          compact_cold[destination] = cold_q[slot];
    end

    active_after_remove = active_count_q - NR_W'(remove_count);
    next_hot = compact_hot;
    next_cold = compact_cold;
    next_hot_count = compact_hot_count;
    next_cold_count = compact_cold_count;
    loaded_after_remove = compact_hot_count + compact_cold_count;
    hidden_after_remove = (active_after_remove > NR_W'(loaded_after_remove)) ?
                          (active_after_remove - NR_W'(loaded_after_remove)) : '0;
    desired_hot_after_remove = (active_after_remove >= NR_W'(HOT_CAPACITY)) ?
                               4'(HOT_CAPACITY) : 4'(active_after_remove);
    if (hidden_after_remove == NR_W'(0)) begin
      for (int move = 0; move < COLD_CAPACITY; move++) begin
        if ((next_hot_count < desired_hot_after_remove) &&
            (next_cold_count != 3'd0)) begin
          next_hot[next_hot_count] = next_cold[next_cold_count-1'b1];
          next_cold[next_cold_count-1'b1] = '0;
          next_hot_count++;
          next_cold_count--;
        end
      end
    end
  end

  always_comb begin
    loaded_count = hot_count_q + cold_count_q;
    hidden_count = (active_count_q > NR_W'(loaded_count)) ?
                   (active_count_q - NR_W'(loaded_count)) : '0;
    top_reserve_count = (hot_count_q > 4'(TOP_VISIBLE)) ?
                        (hot_count_q - 4'(TOP_VISIBLE)) : '0;
    bottom_reserve_count = (cold_count_q > 3'(BOTTOM_VISIBLE)) ?
                           (cold_count_q - 3'(BOTTOM_VISIBLE)) : '0;
    desired_hot_count = (active_count_q >= NR_W'(HOT_CAPACITY)) ?
                        4'(HOT_CAPACITY) : 4'(active_count_q);
    desired_cold_count = (active_count_q > NR_W'(desired_hot_count)) ?
        (((active_count_q - NR_W'(desired_hot_count)) >= NR_W'(COLD_CAPACITY)) ?
         3'(COLD_CAPACITY) : 3'(active_count_q - NR_W'(desired_hot_count))) : '0;
    top_deficit = (desired_hot_count > hot_count_q) ?
                  (desired_hot_count - hot_count_q) : '0;
    bottom_deficit = (desired_cold_count > cold_count_q) ?
                     (desired_cold_count - cold_count_q) : '0;
    refill_request = auto_run_q && (hidden_count != NR_W'(0)) &&
                     ((top_reserve_count <= 4'd1) ||
                      (bottom_reserve_count <= 3'd1));
    refill_top_count = '0;
    refill_bottom_count = '0;
    hidden_after_top = hidden_count;
    bottom_refill_budget = 3'd4;
    if (refill_request) begin
      refill_top_count = (top_deficit > 4'd4) ? 3'd4 :
          ((NR_W'(top_deficit) > hidden_count) ?
           3'(hidden_count) : 3'(top_deficit));
      hidden_after_top = hidden_count - NR_W'(refill_top_count);
      if (refill_top_count > 3'd2)
        bottom_refill_budget = 3'd6 - refill_top_count;
      refill_bottom_count = (bottom_deficit > bottom_refill_budget) ?
                            bottom_refill_budget : bottom_deficit;
      if (NR_W'(refill_bottom_count) > hidden_after_top)
        refill_bottom_count = 3'(hidden_after_top);
    end
    window_ready = (hot_count_q >=
        ((active_count_q >= NR_W'(TOP_VISIBLE)) ?
         4'(TOP_VISIBLE) : 4'(active_count_q))) &&
        ((active_count_q <= NR_W'(hot_count_q)) || (cold_count_q != 3'd0));

    initial_hot_count = (active_count_q >= NR_W'(HOT_CAPACITY)) ?
                        4'(HOT_CAPACITY) : 4'(active_count_q);
    initial_cold_count = (active_count_q > NR_W'(initial_hot_count)) ?
        (((active_count_q - NR_W'(initial_hot_count)) >= NR_W'(COLD_CAPACITY)) ?
         3'(COLD_CAPACITY) :
         3'(active_count_q - NR_W'(initial_hot_count))) : '0;
  end

  assign event_refill_top_count = refill_active_q ?
      refill_top_remaining_q : refill_top_count;
  assign event_refill_bottom_count = refill_active_q ?
      refill_bottom_remaining_q : refill_bottom_count;
  assign refill_remaining_count = {1'b0, refill_top_remaining_q} +
                                  {1'b0, refill_bottom_remaining_q};

  always_comb begin
    refill_hot = hot_q;
    refill_cold = cold_q;
    refill_hot_count_next = hot_count_q;
    refill_cold_count_next = cold_count_q;
    refill_top_take = '0;
    refill_bottom_take = '0;
    if (refill_write && refill_active_q) begin
      refill_top_take = (mmio_count > refill_top_remaining_q) ?
                        refill_top_remaining_q : mmio_count;
      refill_bottom_take = mmio_count - refill_top_take;
      for (int slot = 0; slot < 4; slot++) begin
        if (slot < refill_top_take) begin
          refill_hot[refill_hot_count_next] = mmio_entries[slot];
          refill_hot_count_next++;
        end else if (slot < mmio_count) begin
          refill_cold[refill_cold_count_next] = mmio_entries[slot];
          refill_cold_count_next++;
        end
      end

      if ((4'(mmio_count) == refill_remaining_count) &&
          (hidden_count == NR_W'(mmio_count))) begin
        for (int move = 0; move < COLD_CAPACITY; move++) begin
          if ((refill_hot_count_next < desired_hot_count) &&
              (refill_cold_count_next != 3'd0)) begin
            refill_hot[refill_hot_count_next] =
                refill_cold[refill_cold_count_next-1'b1];
            refill_cold[refill_cold_count_next-1'b1] = '0;
            refill_hot_count_next++;
            refill_cold_count_next--;
          end
        end
      end
    end
  end

  assign remove_ready = auto_run_q && remove_valid && !refill_write;
  assign run_start = auto_run_q && !core_busy && !remove_valid &&
                     (active_count_q != NR_W'(0)) && !refill_request &&
                     !refill_active_q &&
                     window_ready && !task_full && !window_start_write;
  assign core_init = window_start_write;
  assign core_start = run_start;

  always_comb begin
    initial_counters = '0;
    initial_counters.count = active_count_q;
    initial_counters.token_sum = initial_aggregate_q.token_sum;
    initial_counters.odd_count = initial_aggregate_q.odd_count;
    initial_counters.block_sum = initial_aggregate_q.block_sum;
    initial_counters.small_hist = initial_aggregate_q.small_hist;
  end

  assign event_pending = refill_active_q || refill_request ||
                         task_valid ||
                         ((active_count_q == NR_W'(0)) && !core_busy);

  always_comb begin
    reg_rsp_o.rdata = '0;
    reg_rsp_o.error = 1'b0;
    reg_rsp_o.ready = 1'b1;
    if (word_addr == REG_EVENT_WAIT) begin
      reg_rsp_o.rdata[0] = (active_count_q == NR_W'(0)) && !core_busy;
      reg_rsp_o.rdata[1] = refill_active_q || refill_request;
      reg_rsp_o.rdata[4:2] = event_refill_top_count;
      reg_rsp_o.rdata[7:5] = event_refill_bottom_count;
      reg_rsp_o.rdata[11:8] = task_count;
    end else if (word_addr == REG_TASK_STREAM) begin
      reg_rsp_o.rdata = task_data;
    end
    if (event_read)
      reg_rsp_o.ready = event_pending;
    else if (task_read)
      reg_rsp_o.ready = task_valid;
  end
  assign task_pop = task_read && reg_rsp_o.ready;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      initial_cache_c2_q <= PF_EID_NONE;
      initial_cache_c3_q <= PF_EID_NONE;
      active_count_q <= '0;
      initial_aggregate_q <= '0;
      hot_q <= '{default: '0};
      cold_q <= '{default: '0};
      hot_count_q <= '0;
      cold_count_q <= '0;
      auto_run_q <= 1'b0;
      refill_active_q <= 1'b0;
      refill_top_remaining_q <= '0;
      refill_bottom_remaining_q <= '0;
    end else begin
      if (write_req) begin
        unique case (word_addr)
          REG_CONFIG: begin
            initial_cache_c2_q <= wr_data[7] ? PF_EID_NONE :
                encode_eid(wr_data[EID_RAW_W-1:0]);
            initial_cache_c3_q <= wr_data[15] ? PF_EID_NONE :
                encode_eid(wr_data[8 +: EID_RAW_W]);
            active_count_q <= wr_data[16 +: NR_W];
          end
          REG_WINDOW0: begin
            hot_q <= '{default: '0};
            cold_q <= '{default: '0};
            for (int slot = 0; slot < 4; slot++)
              hot_q[slot] <= mmio_entries[slot];
            hot_count_q <= '0;
            cold_count_q <= '0;
            auto_run_q <= 1'b0;
            refill_active_q <= 1'b0;
            refill_top_remaining_q <= '0;
            refill_bottom_remaining_q <= '0;
          end
          REG_WINDOW1: begin
            for (int slot = 0; slot < 4; slot++)
              hot_q[slot+4] <= mmio_entries[slot];
          end
          REG_WINDOW2: begin
            hot_q[8] <= mmio_entries[0];
            cold_q[0] <= mmio_entries[1];
            cold_q[1] <= mmio_entries[2];
            cold_q[2] <= mmio_entries[3];
          end
          REG_WINDOW3_START: begin
            cold_q[3] <= mmio_entries[0];
            cold_q[4] <= mmio_entries[1];
          end
          REG_REFILL_QUAD: begin
            if (refill_active_q) begin
              hot_q <= refill_hot;
              cold_q <= refill_cold;
              hot_count_q <= refill_hot_count_next;
              cold_count_q <= refill_cold_count_next;
              if (4'(mmio_count) == refill_remaining_count) begin
                refill_active_q <= 1'b0;
                refill_top_remaining_q <= '0;
                refill_bottom_remaining_q <= '0;
              end else begin
                refill_top_remaining_q <=
                    refill_top_remaining_q - refill_top_take;
                refill_bottom_remaining_q <=
                    refill_bottom_remaining_q - refill_bottom_take;
              end
            end
          end
          REG_AGGREGATE: begin
            initial_aggregate_q.token_sum <=
                wr_data[0 +: DIST_TOKEN_SUM_W];
            initial_aggregate_q.odd_count <=
                wr_data[9 +: DIST_HIST_W];
            initial_aggregate_q.block_sum <=
                wr_data[16 +: DIST_BLOCK_SUM_W];
            for (int bucket = 0; bucket < 4; bucket++)
              initial_aggregate_q.small_hist[bucket] <=
                  wr_data[25 + 7*bucket +: DIST_HIST_W];
          end
          default: begin
          end
        endcase
      end

      if (window_start_write) begin
        hot_count_q <= initial_hot_count;
        cold_count_q <= initial_cold_count;
        auto_run_q <= 1'b1;
        refill_active_q <= 1'b0;
        refill_top_remaining_q <= '0;
        refill_bottom_remaining_q <= '0;
      end

      if (!refill_active_q && refill_request) begin
        refill_active_q <= 1'b1;
        refill_top_remaining_q <= refill_top_count;
        refill_bottom_remaining_q <= refill_bottom_count;
      end

      if (remove_ready) begin
        hot_q <= next_hot;
        cold_q <= next_cold;
        hot_count_q <= next_hot_count;
        cold_count_q <= next_cold_count;
        active_count_q <= active_after_remove;
        if (active_after_remove == NR_W'(0))
          auto_run_q <= 1'b0;
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (rst_ni) begin
      assert (hot_count_q <= 4'(HOT_CAPACITY));
      assert (cold_count_q <= 3'(COLD_CAPACITY));
      assert ((hot_count_q + cold_count_q) <= active_count_q);
      if (refill_request && !refill_active_q) begin
        assert (refill_top_count <= 3'd4);
        assert (refill_bottom_count <= 3'd4);
        assert ((refill_top_count + refill_bottom_count) <= 4'd6);
        assert (NR_W'(refill_top_count + refill_bottom_count) <= hidden_count);
        assert ((refill_top_count + refill_bottom_count) != 4'd0);
      end
      if (refill_write) begin
        assert (refill_active_q);
        assert (mmio_count != 3'd0);
        assert (4'(mmio_count) <= refill_remaining_count);
        assert (refill_bottom_take <= refill_bottom_remaining_q);
      end
      if (remove_ready)
        assert (remove_count inside {2'd1, 2'd2});
    end
  end
`endif

  head_ctx_t bottom_view;
  always_comb begin
    bottom_view = '0;
    if (cold_count_q != 3'd0)
      bottom_view = cold_q[0];
    else if (hot_count_q != 4'd0)
      bottom_view = hot_q[hot_count_q-1'b1];
  end

  moe_scheduler_core i_scheduler_core (
    .clk_i                  (clk_i),
    .rst_ni                 (rst_ni),
    .init_i                 (core_init),
    .start_i                (core_start),
    .initial_cache_eid_c2_i (initial_cache_c2_q),
    .initial_cache_eid_c3_i (initial_cache_c3_q),
    .hot_i                  (hot_q[7:0]),
    .bottom_i               (bottom_view),
    .initial_counters_i     (initial_counters),
    .remove_ready_i         (remove_ready),
    .remove_valid_o         (remove_valid),
    .remove_count_o         (remove_count),
    .remove_eid_a_o         (remove_eid_a),
    .remove_eid_b_o         (remove_eid_b),
    .task_fifo_pop_i        (task_pop),
    .task_fifo_valid_o      (task_valid),
    .task_fifo_read_data_o  (task_data),
    .task_fifo_full_o       (task_full),
    .task_fifo_count_o      (task_count),
    .busy_o                 (core_busy)
  );

endmodule
