# Questa regressions for the current scheduler RTL.
# Source questasim_2022.4.rc before invoking these targets.

RTL_DIR := ..
TB_DIR  := .
WORK    := work

VLOG := vlog
VSIM := vsim

CORE_RTL_SRCS := \
    $(RTL_DIR)/sched_pkg.sv \
    $(RTL_DIR)/sched_distilled_pkg.sv \
    $(RTL_DIR)/sched_distilled_profile_decode.sv \
    $(RTL_DIR)/sched_distilled_timeline.sv \
    $(RTL_DIR)/sched_bandwidth_check.sv \
    $(RTL_DIR)/sched_distilled_transition_eval.sv \
    $(RTL_DIR)/sched_distilled_bound_score.sv \
    $(RTL_DIR)/sched_distilled_pair_compare.sv \
    $(RTL_DIR)/sched_distilled_regime_classify.sv \
    $(RTL_DIR)/sched_distilled_start_iter.sv \
    $(RTL_DIR)/sched_distilled_target_s4pf.sv \
    $(RTL_DIR)/sched_distilled_round_engine.sv \
    $(RTL_DIR)/sched_task_word_pack.sv \
    $(RTL_DIR)/moe_scheduler_core.sv

WRAPPER_RTL_SRCS := $(CORE_RTL_SRCS) $(RTL_DIR)/moe_scheduler_reg_wrapper.sv

WRAPPER_TB        := $(TB_DIR)/tb_scheduler_reg_wrapper.sv
WRAPPER_VECTOR_FILE := distilled_wrapper_vectors.txt
WRAPPER_GENERATOR := ../../Idea_Model/gen_scheduler_rtl_distilled_wrapper_vectors.py
PROFILE_VECTOR_FILE := distilled_profile_vectors.txt
PROFILE_GENERATOR := ../../Idea_Model/gen_scheduler_rtl_distilled_profile_vectors.py
PROFILE_TB        := $(TB_DIR)/tb_distilled_profile_decode.sv
TIMELINE_VECTOR_FILE := distilled_timeline_vectors.txt
TIMELINE_GENERATOR := ../../Idea_Model/gen_scheduler_rtl_distilled_timeline_vectors.py
TIMELINE_TB        := $(TB_DIR)/tb_distilled_timeline.sv
TRANSITION_VECTOR_FILE := distilled_transition_vectors.txt
TRANSITION_GENERATOR := ../../Idea_Model/gen_scheduler_rtl_distilled_transition_vectors.py
TRANSITION_TB        := $(TB_DIR)/tb_distilled_transition_eval.sv
BOUND_VECTOR_FILE := distilled_bound_vectors.txt
BOUND_GENERATOR := ../../Idea_Model/gen_scheduler_rtl_distilled_bound_vectors.py
BOUND_TB        := $(TB_DIR)/tb_distilled_bound_score.sv
COMPARE_VECTOR_FILE := distilled_compare_vectors.txt
COMPARE_GENERATOR := ../../Idea_Model/gen_scheduler_rtl_distilled_compare_vectors.py
COMPARE_TB        := $(TB_DIR)/tb_distilled_pair_compare.sv
REGIME_VECTOR_FILE := distilled_regime_vectors.txt
REGIME_GENERATOR := ../../Idea_Model/gen_scheduler_rtl_distilled_regime_vectors.py
REGIME_TB        := $(TB_DIR)/tb_distilled_regime_classify.sv
ROUND_VECTOR_FILE := distilled_round_vectors.txt
ROUND_GENERATOR := ../../Idea_Model/gen_scheduler_rtl_distilled_round_vectors.py
ROUND_TB        := $(TB_DIR)/tb_distilled_round_engine.sv
ROUND_VALIDATION_VECTOR_FILE := distilled_round_validation10k_vectors.txt
S4_ROUND_VECTOR_FILE := distilled_s4_round_vectors.txt
S4_ROUND_GENERATOR := ../../Idea_Model/gen_scheduler_rtl_distilled_s4_round_vectors.py

.PHONY: all verify-wrapper verify-dma-resource \
        verify-distilled-profile profile-gen profile-compile profile-sim \
        verify-distilled-timeline timeline-gen timeline-compile timeline-sim \
        verify-distilled-transition transition-gen transition-compile transition-sim \
        verify-distilled-bound bound-gen bound-compile bound-sim \
        verify-distilled-compare compare-gen compare-compile compare-sim \
	verify-distilled-regime regime-gen regime-compile regime-sim \
	verify-distilled-round round-gen round-compile round-sim \
	verify-distilled-round-validation10k round-validation10k-gen \
	round-validation10k-sim \
	verify-distilled-s4-round s4-round-gen s4-round-sim \
        wrapper-compile wrapper-sim dma-resource-compile dma-resource-sim clean

all: verify-wrapper

profile-gen: $(PROFILE_GENERATOR)
	python3 $(PROFILE_GENERATOR) > $(PROFILE_VECTOR_FILE)

profile-compile: $(WORK) $(RTL_DIR)/sched_pkg.sv \
                 $(RTL_DIR)/sched_distilled_pkg.sv \
                 $(RTL_DIR)/sched_distilled_profile_decode.sv $(PROFILE_TB)
	$(VLOG) -work $(WORK) -sv +acc \
	    $(RTL_DIR)/sched_pkg.sv $(RTL_DIR)/sched_distilled_pkg.sv \
	    $(RTL_DIR)/sched_distilled_profile_decode.sv $(PROFILE_TB)

profile-sim:
	$(VSIM) -batch -work $(WORK) tb_distilled_profile_decode \
	    -do "run -all; quit -f" -logfile distilled_profile_sim.log
	@grep -E "PASS|FAIL|RESULT" distilled_profile_sim.log || true
	@! grep -q "\[RESULT\] FAIL" distilled_profile_sim.log

verify-distilled-profile: profile-gen profile-compile profile-sim

timeline-gen: $(TIMELINE_GENERATOR)
	python3 $(TIMELINE_GENERATOR) > $(TIMELINE_VECTOR_FILE)

timeline-compile: $(WORK) $(RTL_DIR)/sched_pkg.sv \
                  $(RTL_DIR)/sched_distilled_timeline.sv $(TIMELINE_TB)
	$(VLOG) -work $(WORK) -sv +acc $(RTL_DIR)/sched_pkg.sv \
	    $(RTL_DIR)/sched_distilled_timeline.sv $(TIMELINE_TB)

timeline-sim:
	$(VSIM) -batch -work $(WORK) tb_distilled_timeline \
	    -do "run -all; quit -f" -logfile distilled_timeline_sim.log
	@grep -E "PASS|FAIL|RESULT" distilled_timeline_sim.log || true
	@! grep -q "\[RESULT\] FAIL" distilled_timeline_sim.log

verify-distilled-timeline: timeline-gen timeline-compile timeline-sim

transition-gen: $(TRANSITION_GENERATOR)
	python3 $(TRANSITION_GENERATOR) > $(TRANSITION_VECTOR_FILE)

transition-compile: $(WORK) $(RTL_DIR)/sched_pkg.sv \
                    $(RTL_DIR)/sched_distilled_pkg.sv \
                    $(RTL_DIR)/sched_distilled_profile_decode.sv \
                    $(RTL_DIR)/sched_distilled_timeline.sv \
                    $(RTL_DIR)/sched_bandwidth_check.sv \
                    $(RTL_DIR)/sched_distilled_transition_eval.sv $(TRANSITION_TB)
	$(VLOG) -work $(WORK) -sv +acc \
	    $(RTL_DIR)/sched_pkg.sv $(RTL_DIR)/sched_distilled_pkg.sv \
	    $(RTL_DIR)/sched_distilled_profile_decode.sv \
	    $(RTL_DIR)/sched_distilled_timeline.sv \
	    $(RTL_DIR)/sched_bandwidth_check.sv \
	    $(RTL_DIR)/sched_distilled_transition_eval.sv $(TRANSITION_TB)

transition-sim:
	$(VSIM) -batch -work $(WORK) tb_distilled_transition_eval \
	    -do "run -all; quit -f" -logfile distilled_transition_sim.log
	@grep -E "PASS|FAIL|RESULT" distilled_transition_sim.log || true
	@! grep -q "\[RESULT\] FAIL" distilled_transition_sim.log

verify-distilled-transition: transition-gen transition-compile transition-sim

bound-gen: $(BOUND_GENERATOR)
	python3 $(BOUND_GENERATOR) > $(BOUND_VECTOR_FILE)

bound-compile: $(WORK) $(RTL_DIR)/sched_pkg.sv \
               $(RTL_DIR)/sched_distilled_pkg.sv \
               $(RTL_DIR)/sched_distilled_bound_score.sv $(BOUND_TB)
	$(VLOG) -work $(WORK) -sv +acc $(RTL_DIR)/sched_pkg.sv \
	    $(RTL_DIR)/sched_distilled_pkg.sv \
	    $(RTL_DIR)/sched_distilled_bound_score.sv $(BOUND_TB)

bound-sim:
	$(VSIM) -batch -work $(WORK) tb_distilled_bound_score \
	    -do "run -all; quit -f" -logfile distilled_bound_sim.log
	@grep -E "PASS|FAIL|RESULT" distilled_bound_sim.log || true
	@! grep -q "\[RESULT\] FAIL" distilled_bound_sim.log

verify-distilled-bound: bound-gen bound-compile bound-sim

compare-gen: $(COMPARE_GENERATOR)
	python3 $(COMPARE_GENERATOR) > $(COMPARE_VECTOR_FILE)

compare-compile: $(WORK) $(RTL_DIR)/sched_pkg.sv \
                 $(RTL_DIR)/sched_distilled_pkg.sv \
                 $(RTL_DIR)/sched_distilled_pair_compare.sv $(COMPARE_TB)
	$(VLOG) -work $(WORK) -sv +acc $(RTL_DIR)/sched_pkg.sv \
	    $(RTL_DIR)/sched_distilled_pkg.sv \
	    $(RTL_DIR)/sched_distilled_pair_compare.sv $(COMPARE_TB)

compare-sim:
	$(VSIM) -batch -work $(WORK) tb_distilled_pair_compare \
	    -do "run -all; quit -f" -logfile distilled_compare_sim.log
	@grep -E "PASS|FAIL|RESULT" distilled_compare_sim.log || true
	@! grep -q "\[RESULT\] FAIL" distilled_compare_sim.log

verify-distilled-compare: compare-gen compare-compile compare-sim

regime-gen: $(REGIME_GENERATOR)
	python3 $(REGIME_GENERATOR) > $(REGIME_VECTOR_FILE)

regime-compile: $(WORK) $(RTL_DIR)/sched_pkg.sv \
                $(RTL_DIR)/sched_distilled_pkg.sv \
                $(RTL_DIR)/sched_distilled_regime_classify.sv $(REGIME_TB)
	$(VLOG) -work $(WORK) -sv +acc $(RTL_DIR)/sched_pkg.sv \
	    $(RTL_DIR)/sched_distilled_pkg.sv \
	    $(RTL_DIR)/sched_distilled_regime_classify.sv $(REGIME_TB)

regime-sim:
	$(VSIM) -batch -work $(WORK) tb_distilled_regime_classify \
	    -do "run -all; quit -f" -logfile distilled_regime_sim.log
	@grep -E "PASS|FAIL|RESULT" distilled_regime_sim.log || true
	@! grep -q "\[RESULT\] FAIL" distilled_regime_sim.log

verify-distilled-regime: regime-gen regime-compile regime-sim

round-gen: $(ROUND_GENERATOR)
	python3 $(ROUND_GENERATOR) > $(ROUND_VECTOR_FILE)

round-compile: $(WORK) $(RTL_DIR)/sched_pkg.sv \
               $(RTL_DIR)/sched_distilled_pkg.sv \
               $(RTL_DIR)/sched_distilled_profile_decode.sv \
               $(RTL_DIR)/sched_distilled_timeline.sv \
               $(RTL_DIR)/sched_bandwidth_check.sv \
               $(RTL_DIR)/sched_distilled_transition_eval.sv \
               $(RTL_DIR)/sched_distilled_bound_score.sv \
               $(RTL_DIR)/sched_distilled_pair_compare.sv \
               $(RTL_DIR)/sched_distilled_regime_classify.sv \
               $(RTL_DIR)/sched_distilled_start_iter.sv \
               $(RTL_DIR)/sched_distilled_target_s4pf.sv \
               $(RTL_DIR)/sched_distilled_round_engine.sv $(ROUND_TB)
	$(VLOG) -work $(WORK) -sv +acc $(filter %.sv,$^)

round-sim:
	$(VSIM) -batch -work $(WORK) tb_distilled_round_engine \
	    -do "run -all; quit -f" -logfile distilled_round_sim.log
	@grep -E "PASS|FAIL|RESULT" distilled_round_sim.log || true
	@! grep -q "\[RESULT\] FAIL" distilled_round_sim.log
	@! grep -q "\*\* Fatal" distilled_round_sim.log

verify-distilled-round: round-gen round-compile round-sim

round-validation10k-gen: $(ROUND_GENERATOR)
	python3 $(ROUND_GENERATOR) --suite validation --round-limit 10000 \
	    > $(ROUND_VALIDATION_VECTOR_FILE)

round-validation10k-sim:
	$(VSIM) -batch -work $(WORK) tb_distilled_round_engine \
	    +VECTOR_FILE=$(ROUND_VALIDATION_VECTOR_FILE) \
	    -do "run -all; quit -f" -logfile distilled_round_validation10k_sim.log
	@grep -E "PASS|FAIL|RESULT" distilled_round_validation10k_sim.log || true
	@! grep -q "\[RESULT\] FAIL" distilled_round_validation10k_sim.log
	@! grep -q "\*\* Fatal" distilled_round_validation10k_sim.log

verify-distilled-round-validation10k: round-validation10k-gen round-compile \
	round-validation10k-sim

s4-round-gen: $(S4_ROUND_GENERATOR)
	python3 $(S4_ROUND_GENERATOR) > $(S4_ROUND_VECTOR_FILE)

s4-round-sim:
	$(VSIM) -batch -work $(WORK) tb_distilled_round_engine \
	    +VECTOR_FILE=$(S4_ROUND_VECTOR_FILE) \
	    -do "run -all; quit -f" -logfile distilled_s4_round_sim.log
	@grep -E "PASS|FAIL|RESULT" distilled_s4_round_sim.log || true
	@! grep -q "\[RESULT\] FAIL" distilled_s4_round_sim.log
	@! grep -q "\*\* Fatal" distilled_s4_round_sim.log

verify-distilled-s4-round: s4-round-gen round-compile s4-round-sim

$(WORK):
	vlib $(WORK)

wrapper-gen: $(WRAPPER_GENERATOR)
	python3 $(WRAPPER_GENERATOR) > $(WRAPPER_VECTOR_FILE)

wrapper-compile: $(WORK) $(WRAPPER_RTL_SRCS) $(WRAPPER_TB)
	$(VLOG) -work $(WORK) -sv +acc $(WRAPPER_RTL_SRCS) $(WRAPPER_TB)

wrapper-sim:
	$(VSIM) -batch -work $(WORK) tb_scheduler_reg_wrapper \
	    -do "run -all; quit -f" -logfile wrapper_sim.log
	@grep -E "PASS|FAIL|RESULT" wrapper_sim.log || true
	@! grep -q "\[RESULT\] FAIL" wrapper_sim.log
	@! grep -q "\*\* Fatal" wrapper_sim.log

verify-wrapper: wrapper-gen wrapper-compile wrapper-sim

dma-resource-compile: $(WORK) $(RTL_DIR)/sched_pkg.sv \
                       $(RTL_DIR)/sched_bandwidth_check.sv tb_dma_resource_check.sv
	$(VLOG) -work $(WORK) -sv +acc \
	    $(RTL_DIR)/sched_pkg.sv $(RTL_DIR)/sched_bandwidth_check.sv \
	    tb_dma_resource_check.sv

dma-resource-sim:
	$(VSIM) -batch -work $(WORK) tb_dma_resource_check \
	    -do "run -all; quit -f" -logfile dma_resource_sim.log
	@grep -E "PASS|FAIL|RESULT" dma_resource_sim.log || true

verify-dma-resource: dma-resource-compile dma-resource-sim

clean:
	rm -rf $(WORK) \
	    $(PROFILE_VECTOR_FILE) distilled_profile_sim.log \
	    $(TIMELINE_VECTOR_FILE) distilled_timeline_sim.log \
	    $(TRANSITION_VECTOR_FILE) distilled_transition_sim.log \
	    $(BOUND_VECTOR_FILE) distilled_bound_sim.log \
	    wrapper_sim.log transcript
	rm -f dma_resource_sim.log
	rm -f modelsim.ini
