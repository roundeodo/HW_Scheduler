`timescale 1ns/1ps

module tb_distilled_timeline;
  import sched_pkg::*;

  time_t start_i;
  ntok_t ntok_i;
  shape_t shape_s1_i;
  shape_t shape_s3_i;
  logic s1_cached_i;
  logic s3_cached_i;
  dma_binding_t dma_s1_i;
  dma_binding_t dma_s3_i;
  dma_binding_t s2pf_dma_i;
  snap_timeline_t timeline_o;
  time_t compute_end_o;

  int fd;
  int count;
  int failures;
  int value [0:21];

  sched_distilled_timeline dut (.*);

  task automatic check_field(input string name, input int got, input int expected);
    if (got !== expected) begin
      $display("[FAIL] row slot=%0d C%0d ntok=%0d field=%s got=%0d expected=%0d",
               value[0], value[1], value[2], name, got, expected);
      failures++;
    end
  endtask

  initial begin
    failures = 0;
    fd = $fopen("distilled_timeline_vectors.txt", "r");
    if (fd == 0) $fatal(1, "cannot open distilled_timeline_vectors.txt");
    if ($fscanf(fd, "%d\n", count) != 1) $fatal(1, "missing vector count");
    for (int row = 0; row < count; row++) begin
      if ($fscanf(fd,
          "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
          value[0], value[1], value[2], value[3], value[4], value[5],
          value[6], value[7], value[8], value[9], value[10], value[11],
          value[12], value[13], value[14], value[15], value[16], value[17],
          value[18], value[19], value[20], value[21]) != 22)
        $fatal(1, "malformed timeline vector %0d", row);
      ntok_i = ntok_t'(value[2]);
      start_i = time_t'(value[3]);
      shape_s1_i = shape_t'(value[4]);
      shape_s3_i = shape_t'(value[5]);
      s1_cached_i = value[6];
      s3_cached_i = value[7];
      dma_s1_i = dma_binding_t'(value[8]);
      dma_s3_i = dma_binding_t'(value[9]);
      s2pf_dma_i = dma_binding_t'(value[10]);
      #1;
      check_field("task_end", timeline_o.task_end, value[11]);
      check_field("dma1_end", timeline_o.dma1_end, value[12]);
      check_field("s2_end", timeline_o.s2_end, value[13]);
      check_field("dma3_end", timeline_o.dma3_end, value[14]);
      check_field("compute_end", compute_end_o, value[15]);
      check_field("s2pf_valid", timeline_o.s2pf_valid, value[16]);
      check_field("s2pf_start", timeline_o.s2pf_start, value[17]);
      check_field("s2pf_end", timeline_o.s2pf_end, value[18]);
      check_field("dma_s1", timeline_o.dma_s1, value[19]);
      check_field("dma_s3", timeline_o.dma_s3, value[20]);
      check_field("s2pf_dma", timeline_o.s2pf_dma, value[21]);
    end
    $fclose(fd);
    if (failures == 0)
      $display("[RESULT] PASS distilled_timeline vectors=%0d", count);
    else
      $display("[RESULT] FAIL distilled_timeline failures=%0d", failures);
    $finish;
  end
endmodule
