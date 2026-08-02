set output_dir [expr {$argc > 0 ? [lindex $argv 0] : "/tmp/scheduler_distilled_ooc"}]
file mkdir $output_dir
set scheduler_dir [file dirname [file normalize [info script]]]

set sources [list \
  $scheduler_dir/sched_pkg.sv \
  $scheduler_dir/sched_distilled_pkg.sv \
  $scheduler_dir/sched_distilled_profile_decode.sv \
  $scheduler_dir/sched_distilled_timeline.sv \
  $scheduler_dir/sched_bandwidth_check.sv \
  $scheduler_dir/sched_distilled_transition_eval.sv \
  $scheduler_dir/sched_distilled_bound_score.sv \
  $scheduler_dir/sched_distilled_pair_compare.sv \
  $scheduler_dir/sched_distilled_regime_classify.sv \
  $scheduler_dir/sched_distilled_start_iter.sv \
  $scheduler_dir/sched_distilled_target_s4pf.sv \
  $scheduler_dir/sched_distilled_round_engine.sv \
  $scheduler_dir/sched_task_word_pack.sv \
  $scheduler_dir/moe_scheduler_core.sv \
  $scheduler_dir/moe_scheduler_reg_wrapper.sv]

read_verilog -sv $sources
synth_design -top moe_scheduler_reg_wrapper \
  -part xcvp1802-lsvc4072-2MP-e-S -mode out_of_context \
  -flatten_hierarchy rebuilt
create_clock -name scheduler_clk -period 25.000 [get_ports clk_i]

write_checkpoint -force $output_dir/scheduler_distilled_synth.dcp
report_utilization -hierarchical -file $output_dir/utilization_hier.rpt
report_timing_summary -delay_type max -max_paths 20 \
  -file $output_dir/timing_summary.rpt
report_high_fanout_nets -fanout_greater_than 32 -timing \
  -file $output_dir/high_fanout.rpt
report_design_analysis -logic_level_distribution \
  -of_timing_paths [get_timing_paths -max_paths 100] \
  -file $output_dir/logic_levels.rpt
exit
