`timescale 1ns/1ps

module tb_distilled_profile_decode;
  import sched_pkg::*;
  import sched_distilled_pkg::*;

  distilled_mode_t mode_i;
  logic [4:0] mode_index_i;
  logic [4:0] profile_addr_i;
  distilled_profile_t profile_o;

  int fd;
  int count;
  int failures;
  int value [0:23];

  sched_distilled_profile_decode dut (.*);

  task automatic check_field(input string name, input int got, input int expected);
    if (got !== expected) begin
      $display("[FAIL] slot=%0d field=%s got=%0d expected=%0d",
               value[2], name, got, expected);
      failures++;
    end
  endtask

  initial begin
    failures = 0;
    fd = $fopen("distilled_profile_vectors.txt", "r");
    if (fd == 0) $fatal(1, "cannot open distilled_profile_vectors.txt");
    if ($fscanf(fd, "%d\n", count) != 1) $fatal(1, "missing vector count");

    for (int row = 0; row < count; row++) begin
      if ($fscanf(fd,
          "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          value[0], value[1], value[2], value[3], value[4], value[5],
          value[6], value[7], value[8], value[9], value[10], value[11],
          value[12], value[13], value[14], value[15], value[16], value[17],
          value[18], value[19], value[20], value[21], value[22], value[23]) != 24)
        $fatal(1, "malformed profile vector %0d", row);
      mode_i = distilled_mode_t'(value[0]);
      mode_index_i = 5'(value[1]);
      profile_addr_i = distilled_mode_profile_base(mode_i) + mode_index_i;
      #1;
      check_field("slot", profile_o.profile_slot, value[2]);
      check_field("logical", profile_o.logical_id, value[3]);
      check_field("family", profile_o.family, value[4]);
      check_field("selector_a", profile_o.selector_a, value[5]);
      check_field("selector_b", profile_o.selector_b, value[6]);
      check_field("balanced", profile_o.split_balanced, value[7]);
      check_field("c2_active", profile_o.c2_active, value[8]);
      check_field("c3_active", profile_o.c3_active, value[9]);
      check_field("c2_s1", profile_o.c2_shape_s1, value[10]);
      check_field("c2_s3", profile_o.c2_shape_s3, value[11]);
      check_field("c3_s1", profile_o.c3_shape_s1, value[12]);
      check_field("c3_s3", profile_o.c3_shape_s3, value[13]);
      check_field("c2_dma_s1", profile_o.c2_dma_s1, value[14]);
      check_field("c2_dma_s3", profile_o.c2_dma_s3, value[15]);
      check_field("c2_s2pf", profile_o.c2_s2pf, value[16]);
      check_field("c3_dma_s1", profile_o.c3_dma_s1, value[17]);
      check_field("c3_dma_s3", profile_o.c3_dma_s3, value[18]);
      check_field("c3_s2pf", profile_o.c3_s2pf, value[19]);
      if (value[8]) begin
        check_field("c2_s1_cached_encoding", value[20], value[14] == DMA_NONE);
        if (value[16] == DMA_NONE)
          check_field("c2_s3_cached_encoding", value[21], value[15] == DMA_NONE);
      end
      if (value[9]) begin
        check_field("c3_s1_cached_encoding", value[22], value[17] == DMA_NONE);
        if (value[19] == DMA_NONE)
          check_field("c3_s3_cached_encoding", value[23], value[18] == DMA_NONE);
      end
    end

    profile_addr_i = 5'd28;
    #1;
    check_field("invalid_slot", profile_o.profile_slot, 5'h1f);
    $fclose(fd);
    if (failures == 0)
      $display("[RESULT] PASS distilled_profile_decode profiles=%0d", count);
    else
      $display("[RESULT] FAIL distilled_profile_decode failures=%0d", failures);
    $finish;
  end
endmodule
