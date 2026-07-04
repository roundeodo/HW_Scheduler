# Makefile — scheduler RTL testbenches
#
# Workflow:
#   make          → compile C generator, generate vectors, run QuestaSim
#   make gen      → generate vectors only
#   make compile  → vlog/vopt only
#   make sim      → run simulation only (assumes compile + gen done)
#   make clean    → remove build artefacts
#
# Prerequisites on the server:
#   - GCC available in PATH (or adjust CC below)
#   - QuestaSim sourced: source /esat/micas-data/data/design/scripts/questasim_2022.4.rc
#
# All RTL files live in the parent directory (../):
RTL_DIR := ..
TB_DIR  := .
WORK    := work

CC      := gcc
CFLAGS  := -std=c99 -O2 -Wall -Wextra

VLOG    := vlog
VSIM    := vsim

# RTL source list (order: package first, then primitives, then top)
RTL_SRCS := \
    $(RTL_DIR)/sched_pkg.sv          \
    $(RTL_DIR)/sched_candidate_pkg.sv \
    $(RTL_DIR)/sched_bw_ok_seq.sv    \
    $(RTL_DIR)/sched_mk_snap.sv      \
    $(RTL_DIR)/sched_pick_shapes.sv  \
    $(RTL_DIR)/sched_s2pf_pair.sv    \
    $(RTL_DIR)/sched_score_unit.sv   \
    $(RTL_DIR)/sched_candidate_eval_lane.sv

TB_SRCS := $(TB_DIR)/tb_eval_lane.sv

VEC_FILE  := eval_vectors.txt
GEN_BIN   := gen_eval_vectors
GEN_SRC   := $(TB_DIR)/gen_eval_vectors.c
VERIFY_BIN := verify_compact_lowering
VERIFY_SRC := $(TB_DIR)/verify_compact_lowering.c
SCHED_VEC_FILE := schedule_vectors.txt
SCHED_GEN_BIN  := gen_schedule_vectors
SCHED_GEN_SRC  := $(TB_DIR)/gen_schedule_vectors.c
SCHED_C    := ../../HeMAiA/target/sw/host/apps/offload_bingo_hw/single_chip/workloads/multi_cluster_MoE/moe_scheduler.c
SCHED_INC  := ../../HeMAiA/target/sw/host/apps/offload_bingo_hw/single_chip/workloads/multi_cluster_MoE

CORE_RTL_SRCS := \
    $(RTL_DIR)/sched_pkg.sv                  \
    $(RTL_DIR)/sched_candidate_pkg.sv        \
    $(RTL_DIR)/sched_bw_ok_seq.sv            \
    $(RTL_DIR)/sched_mk_snap.sv              \
    $(RTL_DIR)/sched_pick_shapes.sv          \
    $(RTL_DIR)/sched_s2pf_pair.sv            \
    $(RTL_DIR)/sched_score_unit.sv           \
    $(RTL_DIR)/sched_candidate_eval_lane.sv  \
    $(RTL_DIR)/sched_candidate_generator.sv  \
    $(RTL_DIR)/sched_best_reduce.sv          \
    $(RTL_DIR)/sched_commit_unit.sv          \
    $(RTL_DIR)/sched_ghost_inject_unit.sv    \
    $(RTL_DIR)/sched_plan_pack.sv            \
    $(RTL_DIR)/sched_schedule_core.sv

SCHED_TB_SRCS := $(TB_DIR)/tb_schedule_core.sv
WRAPPER_RTL_SRCS := $(CORE_RTL_SRCS) $(RTL_DIR)/moe_scheduler_reg_wrapper.sv
WRAPPER_TB_SRCS := $(TB_DIR)/tb_scheduler_reg_wrapper.sv

.PHONY: all gen compile sim verify-lowering schedule-gen schedule-compile schedule-sim verify-schedule-core wrapper-compile wrapper-sim verify-scheduler-reg-wrapper clean

all: gen compile sim

# ── Step 1: build C generator ────────────────────────────────────────────────
$(GEN_BIN): $(GEN_SRC)
	$(CC) $(CFLAGS) -o $@ $<

# ── Step 2: run generator → produce eval_vectors.txt ────────────────────────
gen: $(GEN_BIN)
	./$(GEN_BIN) > $(VEC_FILE)
	@echo "[GEN] $(VEC_FILE) ready ($$(wc -l < $(VEC_FILE)) vectors)"

# ── Step 3: compile RTL + TB with QuestaSim ──────────────────────────────────
compile: $(RTL_SRCS) $(TB_SRCS)
	$(VLOG) -work $(WORK) -sv \
	    +acc \
	    $(RTL_SRCS) $(TB_SRCS)

# ── Step 4: run simulation ────────────────────────────────────────────────────
sim:
	$(VSIM) -batch \
	    -work $(WORK) \
	    tb_eval_lane \
	    -do "run -all; quit -f" \
	    -logfile sim.log
	@echo ""
	@grep -E "PASS|FAIL|RESULT|══" sim.log || true

verify-lowering: $(VERIFY_SRC) $(SCHED_C)
	$(CC) $(CFLAGS) -DMOE_SCHEDULER_ENABLE_LEGACY_CHECK \
	    -I$(SCHED_INC) \
	    -o $(VERIFY_BIN) $(VERIFY_SRC) $(SCHED_C)
	./$(VERIFY_BIN)

$(SCHED_GEN_BIN): $(SCHED_GEN_SRC) $(SCHED_C)
	$(CC) $(CFLAGS) -I$(SCHED_INC) -o $@ $(SCHED_GEN_SRC) $(SCHED_C)

schedule-gen: $(SCHED_GEN_BIN)
	./$(SCHED_GEN_BIN) > $(SCHED_VEC_FILE)
	@echo "[GEN] $(SCHED_VEC_FILE) ready ($$(wc -l < $(SCHED_VEC_FILE)) lines)"

schedule-compile: $(CORE_RTL_SRCS) $(SCHED_TB_SRCS)
	$(VLOG) -work $(WORK) -sv \
	    +acc \
	    $(CORE_RTL_SRCS) $(SCHED_TB_SRCS)

schedule-sim:
	$(VSIM) -batch \
	    -work $(WORK) \
	    tb_schedule_core \
	    -do "run -all; quit -f" \
	    -logfile schedule_core_sim.log
	@echo ""
	@grep -E "PASS|FAIL|RESULT" schedule_core_sim.log || true

verify-schedule-core: schedule-gen schedule-compile schedule-sim

wrapper-compile: $(WRAPPER_RTL_SRCS) $(WRAPPER_TB_SRCS)
	$(VLOG) -work $(WORK) -sv \
	    +acc \
	    $(WRAPPER_RTL_SRCS) $(WRAPPER_TB_SRCS)

wrapper-sim:
	$(VSIM) -batch \
	    -work $(WORK) \
	    tb_scheduler_reg_wrapper \
	    -do "run -all; quit -f" \
	    -logfile scheduler_reg_wrapper_sim.log
	@echo ""
	@grep -E "PASS|FAIL|RESULT" scheduler_reg_wrapper_sim.log || true

verify-scheduler-reg-wrapper: schedule-gen wrapper-compile wrapper-sim

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -rf $(WORK) $(GEN_BIN) $(VERIFY_BIN) $(SCHED_GEN_BIN) \
	    $(VEC_FILE) $(SCHED_VEC_FILE) sim.log schedule_core_sim.log \
	    scheduler_reg_wrapper_sim.log transcript
	rm -f modelsim.ini
