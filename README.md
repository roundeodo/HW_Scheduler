# MoE Scheduler Hardware

当前目录只保留 pure-slave scheduler datapath。scheduler 不主动读写 L3，也没有内部 SRAM。完整 sorted rem stream、完整 plan/lowered args、token metadata 仍由 CVA6/L3 维护；RTL 内部只保存当前 top window、reserve window、snap/state 和输出 FIFO 所需的寄存器。

## 当前 RTL 层次

```text
moe_scheduler_reg_wrapper
  └── sched_schedule_core
      ├── sched_ghost_inject_unit x2
      ├── sched_candidate_generator
      ├── sched_candidate_eval_lane
      │   ├── sched_pick_shapes
      │   ├── sched_mk_snap x2
      │   ├── sched_s2pf_pair
      │   ├── sched_bw_ok
      │   └── sched_score_unit
      ├── sched_best_reduce
      └── sched_commit_unit
```

`sched_pkg.sv` 是唯一的全局参数/类型包。当前默认 `E_MAX=64`；`EID_W/EID_RAW_W/NR_W/T_W` 会随之推导，其中 `T_W=16`，避免 `task_end/total_conc` 溢出。

## 10K Reserve/Auto-Run 协议

batch 初始化时，CVA6 写入：

- `CONFIG`：`cache_eid_c2/c3`、`active_count`、`total_conc`。
- `HEAD_PAIR0/1`：初始 top4，即 sorted rem stream 的前 4 个 valid expert。
- `RESERVE_PAIR0/1`：reserve[4]，即 sorted rem stream 的后 4 个 valid expert。
- `CTRL_INIT|CTRL_START`：初始化并启动第一轮。

steady-state 下，RTL 自己执行 round-to-round 状态推进：

1. `sched_schedule_core` 完成一轮 candidate evaluation，输出 winner 对应的 compact task descriptor、`allow_s4pf`、`remove_count` 和内部 `remove_slot_mask`。
2. `moe_scheduler_reg_wrapper` 根据 `remove_slot_mask` 对内部 `head_q[4]` 做保持顺序的 prefix/rank compact。
3. wrapper 从 `reserve_q[4]` 头部 pop 需要的 entry，补到 compact 后的 top window 尾部。
4. 如果 top window 已满足下一轮所需 entry 且输出 FIFO 未满，wrapper 自动启动下一轮。
5. 如果 reserve 不足，wrapper 拉高 `refill_req` 并给出 `refill_count`，CVA6 从 L3 sorted rem stream 继续取 entry，通过 `HEAD_PUSH_PAIR` append 到 reserve 尾部。

当前 `HEAD_PUSH_PAIR` 不再表示“直接补当前 round top4”，而是 reserve refill 入口。每个 word 仍然包含两个 `head16`。

## 输出 FIFO

`sched_schedule_core` 内部保存 depth=4 的 round packet FIFO。每个 FIFO entry 是一个 round packet：

- `plan_count`：本轮输出 1 条还是 2 条 task。
- `remove_count`：本轮消耗 1 个还是 2 个 expert。
- `plan_task_desc[0..1]`：compact single-task descriptor。
- `plan_allow_s4pf[0..1]`：对应 task 是否留下 S4 prefetch window。
- `plan_local_slot[0..1]`：RTL 为该 task 分配的 per-cluster dynamic args slot。

`STATUS` 是 fast hot path 的事件/metadata 入口，包含 `plan_valid`、`plan_count`、
`refill_req/refill_count` 和 `active_empty`。`PLAN_FIFO_STATUS` 仍可读取 FIFO head
metadata，但只用于 debug/check/TB，不要求 host fast path 每个 FIFO head 都读它。
`PLAN_FIFO_STATUS` 暴露：

- FIFO empty/full/count；
- head `plan_count/remove_count/slot_valid`；
- `reserve_count/refill_req/refill_count/active_empty`。

CVA6 drain FIFO 时读取 `PLAN_FIFO_PATCH` 和 `PLAN_FIFO_DATA0[/1]`，`DATA1` 由
`STATUS.plan_count` 决定。当前 word 已包含 lightweight lowering scalar，CVA6
仍负责写 dynamic args memory 和地址相关 prelower，但不再需要从 shape/ntok
重新推导 `skip_s2/skip_s4/dma_for_shape/m_s*_exec`。读完 FIFO head 后，CVA6 写
`ROUND_COMMIT.plan_pop=1` pop FIFO。reserve refill 只通过
`HEAD_PUSH_PAIR` 完成，`ROUND_COMMIT` 不携带 push metadata。

## 当前仍未硬化的部分

已经落地的是 10K 的 reserve/auto-run/FIFO 协议主干、10K-E 的 lightweight
lowering scalar、10K-F/G/H 的 RTL local slot counter、`local_slot` 输出和 host
pointer cache，以及 10K-J 的 RTL S4PF pending target/patch record。10K-I 采用
host batch-static lookup，不新增独立 RTL offset FIFO。

当前 FIFO 输出仍不是完整 dynamic args patch record；RTL 只输出 compact task、
lowering scalar 和 S4PF patch record，CVA6 继续负责写 dynamic args memory。

## MMIO 寄存器摘要

```text
0x00 CTRL              W bit0 init, bit1 start
0x08 STATUS            R busy/done/plan/refill/active metadata
0x10 CONFIG            R/W cache_eid_c2/c3, active_count, total_conc
0x68 ROUND_COMMIT      W bit0 plan_pop
0x80 PLAN_FIFO_STATUS  R FIFO head + reserve/refill metadata
0x90 PLAN_FIFO_DATA0   R FIFO head task 0 + allow_s4pf + local_slot + lower scalar
0x98 PLAN_FIFO_DATA1   R FIFO head task 1 + allow_s4pf + local_slot + lower scalar
0xa0 PLAN_FIFO_PATCH   R task0/task1 对应的 S4PF patch record
0xa8 HEAD_PAIR0        W initial top4 slot 0/1
0xb0 HEAD_PAIR1        W initial top4 slot 2/3
0xb8 HEAD_PUSH_PAIR    W append up to two entries to reserve[4]
0xc0 RESERVE_PAIR0     W initial reserve slot 0/1
0xc8 RESERVE_PAIR1     W initial reserve slot 2/3
```

`head16` layout 当前为：

```text
bits [NTOK_W-1:0]              ntok
bits [NTOK_W +: EID_RAW_W]     eid
bit  [NTOK_W+EID_RAW_W]        valid
```

`best_conc` 不再由 CVA6 写入，也不保存在 `head_q/reserve_q` FF 中；RTL 用 `best_conc_t(ntok)` 在需要更新 `total_conc` 或估算 continuation cost 时组合计算。

`PLAN_FIFO_DATA0/1` 的 64-bit compact plan word 当前为：

```text
bits [31:0]  task_desc_t
bit  [32]    allow_s4pf
bits [38:33] local_slot, 对应 dynamic args ctrl[19:14]
bit  [39]    skip_s2
bit  [40]    skip_s4
bits [42:41] dma_s1_for_shape
bits [44:43] dma_s3_for_shape
bits [53:45] m_s2_exec
bits [62:54] m_s4_exec
bit  [63]    reserved
```

`local_slot` 由 `sched_schedule_core` 的 C2/C3 两个 6-bit counter 产生。CVA6 lowering
仍负责写 dynamic args memory，但不再分配 slot；fast path 使用 `c2_next_arg/c3_next_arg`
pointer cache，避免每个 task 做 `stage_base + local_slot * slot_bytes` 的 64-bit 乘加。
lower scalar 由 `lower_scalar_from_task()` 组合生成，不存入 planq FIFO，因此不增加跨
round FF。

## 验证

`tb/` 目录有三层验证：

- `make verify-lowering`：纯 C 检查 legacy schedule、public schedule 和 compact-plan lowering 的最终 `tasks[]/dma_ops[]` 是否一致。
- `make verify-schedule-core`：完整 RTL datapath 检查。`gen_schedule_vectors.c` 调用 `moe_make_hw_plan()` 生成 512 个 request 的 golden compact plan；`tb_schedule_core.sv` 驱动 core 并逐项比较 compact plan 和 per-cluster `local_slot`。
- `make verify-scheduler-reg-wrapper`：把 wrapper MMIO、reserve refill、auto-run、FIFO drain、plan word `local_slot` 解码和 S4PF patch record 纳入测试。

运行 Questa 前需要：

```bash
source /esat/micas-data/data/design/scripts/questasim_2022.4.rc
make -C Scheduler_hw/tb verify-schedule-core
make -C Scheduler_hw/tb verify-scheduler-reg-wrapper
```

当前已完成的回归结果：

```text
verify-schedule-core:
[RESULT] PASS tests=512 rounds=13404 plan_entries=16812

verify-scheduler-reg-wrapper:
[RESULT] PASS tests=512 rounds=13404 plan_entries=16812
```

该 wrapper TB 已逐字段检查 MMIO wrapper、reserve refill、auto-run、FIFO drain、
`local_slot`、lower scalar 和 S4PF patch record。
