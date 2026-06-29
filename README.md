# MoE Scheduler Hardware

当前目录只保留纯 slave scheduler datapath。旧的 TopK frontend、request compactor、内部 `rem_sram/plan_sram`、lowering、AXI writeback/full-offload 路径已经删除，避免和最终“CVA6 控制 + L3 保存完整状态”的方案混在一起。

## 当前 RTL 层次

```text
sched_schedule_core
  ├── sched_ghost_inject_unit x2
  ├── sched_candidate_generator
  ├── sched_candidate_eval_lane
  │   ├── sched_pick_shapes
  │   ├── sched_mk_snap x2
  │   ├── sched_s2pf_pair
  │   ├── sched_bw_ok
  │   └── sched_score_unit
  ├── sched_best_reduce        // compact best_id + score + remove slot mask
  └── sched_commit_unit        // depth=2 round output package
```

`sched_pkg.sv` 是唯一的全局参数/类型包。当前默认 `E_MAX=64`；`EID_W/EID_RAW_W/NR_W/T_W` 会随之推导，其中 `T_W=16`，避免 `task_end/total_conc` 溢出。

## Pure Slave 边界

调度器内部没有 SRAM，也不会主动读写 L3。完整 sorted `rem[]`、完整 `plan[]` 和 token/TopK 数据都由 CVA6/L3 保存。

batch 开始时，CVA6 把初始 `top4[4]`、`active_count`、`total_conc` 写入 `moe_scheduler_reg_wrapper`，然后用一次 `CTRL_INIT|CTRL_START` 拉起第一轮。当前唯一初始化入口是 compact `HEAD_PAIR0/1`，每个 64-bit word 写入两个 `head32`。wrapper 内部把 `top4[4]` 当作一个 4-entry sorted window 来维护；每轮结束时根据 winner 输出的 slot mask 做 compact，CVA6 只需要用 staged `HEAD_PUSH_PAIR` 补充 0/1/2 个新 expert。

round 结束后，core 输出：

- `remove_valid_o` / `remove_count_o` / `remove_slot_mask_o`：CVA6 先读这组小 metadata，知道当前 top4 window 的哪些 slot 被消耗。
- `plan_valid_o` / `plan_task_desc_o[0..1]`：depth=2 plan holding queue 的队头，本轮最多两条 compact single-task descriptor。
- `plan_count_o`：告诉软件当前队头应向 L3 plan buffer 写入 1 条还是 2 条 task；L3 plan write pointer 由 CVA6 软件维护。
- `plan_allow_s4pf_o[0..1]`：对应 task 是否留下 S4 prefetch 窗口，供 batch 结束后统一 lowering 使用。
- `plan_queue_full_o`：plan queue 满时阻止继续启动新 round，避免覆盖尚未写回 L3 的 plan。

CVA6 通过 `PLAN_FIFO_STATUS/DATA0/DATA1` 读取这个 depth=2 plan holding queue 的队头。`PLAN_FIFO_STATUS` 同时给出 FIFO empty/full、`plan_count`、`remove_count` 和 `slot_valid`。旧 `PLAN_META/PLAN0/PLAN1/PLAN_FIFO_META` alias 已删除，避免双协议长期维护。

CVA6 读走 plan 队头后，先把下一轮需要补充的新 expert 写入 `HEAD_PUSH_PAIR` staging 寄存器，然后写一次 `ROUND_COMMIT`。`HEAD_PUSH_PAIR` 的低/高 32 bit 分别是 staged push0/push1，`push_count` 决定 `ROUND_COMMIT` 消费几个。`ROUND_COMMIT` 在 RTL 内部按固定顺序执行 `plan_pop -> remove_ready/compact -> push staged head0/1 -> start_next`。旧的单独 `PLAN_FIFO_POP`、`CTRL.remove_ready/plan_pop`、`HEAD_PUSH0/1` 和单个 `HEAD_PUSH` 外部入口已删除。L3 plan 保存 compact task desc + `allow_s4pf` 即可；full snap 只保存在 RTL 内部用于下一轮调度，不需要写入 L3。

## Datapath 等价验证

`tb/` 目录现在有两层验证：

- `make verify-lowering`：纯 C 检查，比较 legacy timing/snap lowering、当前公开 `moe_schedule()`、以及 `moe_make_hw_plan()+moe_lower_hw_plan()` 的最终 `tasks[]/dma_ops[]` 是否一致。
- `make verify-schedule-core`：完整 RTL datapath 检查。`gen_schedule_vectors.c` 调用 `moe_make_hw_plan()` 生成 512 个 request 的 golden compact plan；`tb_schedule_core.sv` 模拟 CVA6，维护 L3 rem active list，每轮喂 `top4/active_count/total_conc`，读 `remove_slot_mask` 更新 rem，读 `plan_*` 与 golden 逐项比较。
- `make verify-scheduler-reg-wrapper`：把 `moe_scheduler_reg_wrapper` 也纳入测试。testbench 只在 batch 初始化时写 `HEAD_PAIR0/1/CONFIG`，并用同一次 `CTRL_INIT|CTRL_START` 启动第一轮；之后每轮通过 `PLAN_FIFO_STATUS/DATA0/DATA1` 读 plan/remove metadata，与 golden plan 对比；更新 top window 时写 staged `HEAD_PUSH_PAIR`，再用 `ROUND_COMMIT` 一次性完成 plan pop、remove compact、head append 和下一轮 start。

运行 Questa 前需要先配置环境：

```bash
source /esat/micas-data/data/design/scripts/questasim_2022.4.rc
make -C Scheduler_hw/tb verify-schedule-core
```

当前完整 RTL-vs-C compact plan 对比已经跑通并通过：

```text
[RESULT] PASS tests=512 rounds=13404 plan_entries=16812
```

曾经最早失败出现在 `tid=23`。根因是误把 `rem_len==2 && ntok_sum>4` 的 score 改成了 `nr>2` aggregate 路径；但 C 的 `continuation_cost()` 落到 `greedy_h()` 后，`greedy_h(nr==2)` 仍然使用 exact 2-task expression。修正后 RTL datapath 与 `moe_make_hw_plan()` compact golden 对齐。

当前 S4 prefetch / ghost 已纳入 BW 模型：snap 只保存 `s4pf_valid + s4pf_start`, end 和 BW 由固定常量推导；`sched_bw_ok` 将 S4PF 作为第 5 类 segment 做 cross check。
