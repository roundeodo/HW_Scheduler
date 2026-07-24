# Questa regressions for the current scheduler RTL.
# Source questasim_2022.4.rc before invoking these targets.

RTL_DIR := ..
TB_DIR  := .
WORK    := work

VLOG := vlog
VSIM := vsim

CORE_RTL_SRCS := \
    $(RTL_DIR)/sched_pkg.sv \
    $(RTL_DIR)/sched_candidate_pkg.sv \
    $(RTL_DIR)/sched_bandwidth_check.sv \
    $(RTL_DIR)/sched_task_timeline.sv \
    $(RTL_DIR)/sched_pair_shape_select.sv \
    $(RTL_DIR)/sched_s2pf_search.sv \
    $(RTL_DIR)/sched_continuation_score.sv \
    $(RTL_DIR)/sched_candidate_evaluator.sv \
    $(RTL_DIR)/sched_candidate_generator.sv \
    $(RTL_DIR)/sched_best_candidate.sv \
    $(RTL_DIR)/sched_task_word_pack.sv \
    $(RTL_DIR)/moe_scheduler_core.sv

WRAPPER_RTL_SRCS := $(CORE_RTL_SRCS) $(RTL_DIR)/moe_scheduler_reg_wrapper.sv

SCORE_VECTOR_FILE := score_vectors.txt
CORE_VECTOR_FILE  := core_vectors.txt
SCORE_GENERATOR   := $(TB_DIR)/gen_score_vectors.py
CORE_GENERATOR    := $(TB_DIR)/gen_core_vectors.py
SCORE_TB          := $(TB_DIR)/tb_continuation_score.sv
CORE_TB           := $(TB_DIR)/tb_scheduler_core.sv
WRAPPER_TB        := $(TB_DIR)/tb_scheduler_reg_wrapper.sv

.PHONY: all verify-score verify-core verify-wrapper verify-dma-resource \
        score-gen score-compile score-sim \
        core-gen core-compile core-sim \
        wrapper-compile wrapper-sim dma-resource-compile dma-resource-sim clean

all: verify-core

$(WORK):
	vlib $(WORK)

score-gen: $(SCORE_GENERATOR)
	python3 $(SCORE_GENERATOR) > $(SCORE_VECTOR_FILE)

score-compile: $(WORK) $(RTL_DIR)/sched_pkg.sv \
               $(RTL_DIR)/sched_continuation_score.sv $(SCORE_TB)
	$(VLOG) -work $(WORK) -sv +acc \
	    $(RTL_DIR)/sched_pkg.sv $(RTL_DIR)/sched_continuation_score.sv $(SCORE_TB)

score-sim:
	$(VSIM) -batch -work $(WORK) tb_continuation_score \
	    -do "run -all; quit -f" -logfile score_sim.log
	@grep -E "PASS|FAIL|RESULT" score_sim.log || true

verify-score: score-gen score-compile score-sim

core-gen: $(CORE_GENERATOR)
	python3 $(CORE_GENERATOR) > $(CORE_VECTOR_FILE)

core-compile: $(WORK) $(CORE_RTL_SRCS) $(CORE_TB)
	$(VLOG) -work $(WORK) -sv +acc $(CORE_RTL_SRCS) $(CORE_TB)

core-sim:
	$(VSIM) -batch -work $(WORK) tb_scheduler_core \
	    -do "run -all; quit -f" -logfile core_sim.log
	@grep -E "PASS|FAIL|RESULT" core_sim.log || true

verify-core: core-gen core-compile core-sim

wrapper-compile: $(WORK) $(WRAPPER_RTL_SRCS) $(WRAPPER_TB)
	$(VLOG) -work $(WORK) -sv +acc $(WRAPPER_RTL_SRCS) $(WRAPPER_TB)

wrapper-sim:
	$(VSIM) -batch -work $(WORK) tb_scheduler_reg_wrapper \
	    -do "run -all; quit -f" -logfile wrapper_sim.log
	@grep -E "PASS|FAIL|RESULT" wrapper_sim.log || true

verify-wrapper: core-gen wrapper-compile wrapper-sim

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
	rm -rf $(WORK) $(SCORE_VECTOR_FILE) $(CORE_VECTOR_FILE) \
	    score_sim.log core_sim.log wrapper_sim.log transcript
	rm -f dma_resource_sim.log
	rm -f modelsim.ini
