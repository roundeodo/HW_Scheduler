`timescale 1ns/1ps

module tb_distilled_regime_classify;
  import sched_pkg::*;
  import sched_distilled_pkg::*;

  distilled_mode_t mode_i;
  distilled_counters_t counters_i;
  head_ctx_t [4:0] top_i;
  distilled_cluster_state_t c2_i;
  distilled_cluster_state_t c3_i;
  distilled_regime_t regime_o;

  int fd;
  int vectors;
  int failures;
  int header [0:15];
  int top [0:4][0:2];

  sched_distilled_regime_classify dut (
    .mode_i            (mode_i),
    .count_i           (counters_i.count),
    .token_sum_i       (counters_i.token_sum),
    .odd_count_i       (counters_i.odd_count),
    .one_block_count_i (counters_i.small_hist[0]),
    .t0_ntok_i         (top_i[0].ntok),
    .t1_valid_i        (top_i[1].valid),
    .t1_ntok_i         (top_i[1].ntok),
    .t4_valid_i        (top_i[4].valid),
    .t4_ntok_i         (top_i[4].ntok),
    .c2_end_i          (c2_i.task_end),
    .c3_end_i          (c3_i.task_end),
    .regime_o          (regime_o)
  );

  initial begin
    mode_i = DIST_MODE_TERMINAL;
    counters_i = '0;
    top_i = '{default: '0};
    c2_i = '0;
    c3_i = '0;
    failures = 0;
    fd = $fopen("distilled_regime_vectors.txt", "r");
    if (fd == 0) $fatal(1, "cannot open regime vectors");
    if ($fscanf(fd, "%d\n", vectors) != 1) $fatal(1, "missing vector count");
    for (int row = 0; row < vectors; row++) begin
      if ($fscanf(fd,
          "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          header[0], header[1], header[2], header[3], header[4], header[5],
          header[6], header[7], header[8], header[9], header[10], header[11],
          header[12], header[13], header[14], header[15]) != 16)
        $fatal(1, "bad header row=%0d", row);
      for (int slot = 0; slot < 5; slot++)
        if ($fscanf(fd, "%d %d %d\n", top[slot][0], top[slot][1],
                   top[slot][2]) != 3)
          $fatal(1, "bad top row=%0d", row);

      mode_i = distilled_mode_t'(header[0]);
      counters_i = '0;
      counters_i.count = NR_W'(header[1]);
      counters_i.token_sum = DIST_TOKEN_SUM_W'(header[2]);
      counters_i.odd_count = DIST_HIST_W'(header[3]);
      counters_i.block_sum = DIST_BLOCK_SUM_W'(header[4]);
      for (int bucket = 0; bucket < 4; bucket++)
        counters_i.small_hist[bucket] = DIST_HIST_W'(header[5+bucket]);
      c2_i.task_end = time_t'(header[9]);
      c3_i.task_end = time_t'(header[10]);
      for (int slot = 0; slot < 5; slot++) begin
        top_i[slot].valid = top[slot][0];
        top_i[slot].eid = EID_RAW_W'(top[slot][1]);
        top_i[slot].ntok = ntok_t'(top[slot][2]);
      end
      #1;
      if ({regime_o.low_work_progress, regime_o.sparse_hot_sync,
           regime_o.mid_plateau, regime_o.short_tail_plateau,
           regime_o.large_slack_fill} !==
          {header[11][0], header[12][0], header[13][0],
           header[14][0], header[15][0]}) begin
        $display("[FAIL] row=%0d got=%b expected=%0d%0d%0d%0d%0d", row,
                 regime_o, header[11], header[12], header[13], header[14],
                 header[15]);
        failures++;
      end
    end
    $fclose(fd);
    if (failures == 0)
      $display("[RESULT] PASS distilled_regime_classify vectors=%0d", vectors);
    else
      $display("[RESULT] FAIL distilled_regime_classify failures=%0d", failures);
    $finish;
  end
endmodule
