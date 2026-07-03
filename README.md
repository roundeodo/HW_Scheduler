# MoE Scheduler Hardware

当前目录只保留 pure-slave scheduler datapath。scheduler 不主动读写 L3，也没有内部 SRAM。完整 sorted rem stream、完整 lowered args、token metadata 仍由 CVA6/L3 维护；RTL 内部只保存当前 top window、reserve window、snap/state 和 depth=4 输出 FIFO 所需寄存器。

## RTL 层次

```text
moe_scheduler_reg_wrapper
  └── sched_schedule_core
      ├── sched_ghost_inject_unit x2
      ├── sched_bw_ok_seq                # single shared BW checker
      ├── sched_candidate_generator
      ├── sched_candidate_eval_lane
      │   ├── sched_pick_shapes
      │   ├── sched_mk_snap x2
      │   ├── sched_s2pf_pair            # BW client
      │   └── sched_score_unit
      ├── sched_best_reduce
      └── sched_commit_unit              # BW client
```

`sched_pkg.sv` 是唯一的全局参数/类型包。当前默认 `E_MAX=64`，`PLANQ_DEPTH=4`。

## Batch Init

CVA6 在 batch 开始时写入：

- `CONFIG`：`cache_eid_c2/c3`、`active_count`、`total_conc`。
- `HEAD_QUAD`：初始 top4，即 sorted rem stream 的前 4 个 valid expert。
- `RESERVE_QUAD`：reserve[4]，即 sorted rem stream 的后 4 个 valid expert。
- `CTRL_INIT|CTRL_START`：初始化并启动第一轮。

`head16` layout：

```text
bits [8:0]   ntok
bits [14:9]  eid
bit  [15]    valid
```

`HEAD_QUAD`、`RESERVE_QUAD`、`HEAD_PUSH_QUAD` 都是一个 64-bit word 携带 4 个 `head16`：

```text
bits [15:0]   head0
bits [31:16]  head1
bits [47:32]  head2
bits [63:48]  head3
```

`best_conc` 不由 CVA6 写入，也不保存在 `head_q/reserve_q` FF 中；RTL 用 `best_conc_t(ntok)` 组合计算。

## Auto-Run

steady-state 下，RTL 自己推进 round：

1. `sched_schedule_core` 完成一轮 candidate evaluation，输出 winner 对应的 task descriptor、`allow_s4pf`、`remove_count` 和内部 `remove_slot_mask`。
2. `moe_scheduler_reg_wrapper` 根据 `remove_slot_mask` 对 `head_q[4]` 做保持顺序的 prefix/rank compact。
3. wrapper 从 `reserve_q[4]` 头部 pop 需要的 entry，补到 compact 后 top window 尾部。
4. 如果 top window 满足下一轮所需 entry 且输出 FIFO 未满，wrapper 自动启动下一轮。
5. 如果 reserve 不足，wrapper 拉高 `refill_req` 并给出 `refill_count`；CVA6 从 L3 sorted rem stream 继续取最多 4 个 entry，通过 `HEAD_PUSH_QUAD` append 到 reserve 尾部。

`HEAD_PUSH_QUAD` 是 reserve refill 入口，不直接重写当前 top4。

## Candidate Policy

当前 RTL 是 hardware-lite policy，不再枚举软件 full scheduler 的全部 both-idle
candidate。

`both_idle && active_count>=2` 时只保留 5 个外层候选：

```text
0: PAIR(top0 -> C2, top1 -> C3)
1: PAIR(top1 -> C2, top2 -> C3)
2: PAIR(top2 -> C2, top3 -> C3)
3: SPLIT(top0), half_ceil
4: SPLIT(top0), front_m2 (cut=2)
```

删除内容包括 `PAIR(top0,top2/top3)`、`PAIR(top1,top3)`、PAIR reverse
direction，以及 `SPLIT(top0)` 的 front_m4/front_m8/tail-side cuts。`nr==1`
和 `not_both_idle` 的候选集合暂时保持不变。

S2PF 仍使用 `sched_s2pf_pair` 的 lite policy，并由单个共享 `sched_bw_ok_seq`
顺序检查 BW。

## Output FIFO

`sched_schedule_core` 内部保存 depth=4 的 completed-round FIFO。每个 FIFO entry 是一个 round packet：

- `plan_count`：本轮输出 1 条还是 2 条 task。
- `remove_count`：本轮消耗 1 个还是 2 个 expert。
- `plan_task_desc[0..1]`：compact single-task descriptor。
- `plan_allow_s4pf[0..1]`：对应 task 是否留下 S4 prefetch window。
- `plan_local_slot[0..1]`：RTL 为该 task 分配的 per-cluster dynamic args slot。

`STATUS` 是 fast path 的事件/metadata 入口：

```text
bit  [3]     plan_valid
bit  [5]     active_empty
bit  [6]     refill_req
bits [31:28] refill_count
bits [39:36] plan_fifo_count
bits [47:40] plan_count vector, 2 bits per FIFO entry
```

`PLAN_FIFO_STATUS` 只用于 debug/check/TB，fast path 不需要读它。

CVA6 可以一次 drain FIFO 中 1..4 个 completed rounds：

1. 读 `STATUS`，得到 `plan_fifo_count` 和每个 entry 的 `plan_count`。
2. 对 entry `i` 读 indexed payload：
   - `PLAN_ENTRY_DATA0(i)`：task0。
   - `PLAN_ENTRY_DATA1(i)`：仅当该 entry `plan_count=2` 时读取。
3. lowering 后写 `ROUND_COMMIT.pop_count=N`，一次 pop 已 drain 的 N 个 FIFO entry。

S4PF patch 在 `sched_schedule_core` 的 plan FIFO enqueue 阶段生成。core 维护每个 cluster 的 tail pending record，把 patch 直接内联到 FIFO 中保存的 `PLAN_ENTRY_DATAx[63:56]`。wrapper read path 只做 indexed mux，不再对 `planq[0..3]` 做 read-time walk。

## MMIO Register Map

```text
0x00 CTRL              W bit0 init, bit1 start
0x08 STATUS            R busy/done/plan/refill/active metadata
0x10 CONFIG            R/W cache_eid_c2/c3, active_count, total_conc
0x68 ROUND_COMMIT      W bits[2:0] pop_count
0x80 PLAN_FIFO_STATUS  R debug/check head metadata
0xa8 HEAD_QUAD         W initial top4, head16 x4
0xb0 RESERVE_QUAD      W initial reserve[4], head16 x4
0xb8 HEAD_PUSH_QUAD    W append up to 4 head16 into reserve[4]

0x100 + i*0x20 PLAN_ENTRY_DATA0  R FIFO entry i task0 plan word
0x108 + i*0x20 PLAN_ENTRY_DATA1  R FIFO entry i task1 plan word
```

`PLAN_ENTRY_DATAx` 64-bit word layout：

```text
bits [5:0]   expert id
bits [14:6]  token_start_rank
bits [23:15] ntokens
bit  [24]    has_s2pf
bits [37:25] compact ctrl word
bits [46:38] m_s2_exec
bits [55:47] m_s4_exec
bits [63:56] inline S4PF patch byte
```

`ctrl` bits：

```text
bit  [0]     skip_s1
bit  [1]     skip_s3
bits [3:2]   shape_s1
bits [5:4]   shape_s3
bit  [6]     cluster
bits [12:7]  local_slot
```

CVA6 根据 compact ctrl、`m_s2_exec/m_s4_exec` 和 shape 重新生成写入 dynamic args 的 20-bit runtime ctrl。inline S4PF patch byte layout 是 `valid/no_copy/local_slot[5:0]`；patch target expert 和 cluster 分别来自同一个 DATA word 的 `eid` 和 `ctrl.cluster`。

## 验证

`tb/` 目录有三层验证：

- `make verify-lowering`：纯 C 检查 legacy schedule、public schedule 和 compact-plan lowering 的最终 `tasks[]/dma_ops[]` 是否一致。
- `make verify-schedule-core`：完整 RTL datapath 检查。`gen_schedule_vectors.c` 调用 `moe_make_hw_plan()` 生成 512 个 request 的 golden compact plan；`tb_schedule_core.sv` 驱动 core 并逐项比较 compact plan 和 per-cluster `local_slot`。
- `make verify-scheduler-reg-wrapper`：把 MMIO wrapper、reserve refill、auto-run、indexed FIFO drain、compact `ctrl` word 和 inline S4PF patch 纳入测试。

运行 Questa 前需要：

```bash
source /esat/micas-data/data/design/scripts/questasim_2022.4.rc
make -C Scheduler_hw/tb verify-schedule-core
make -C Scheduler_hw/tb verify-scheduler-reg-wrapper
```

当前回归结果：

```text
verify-schedule-core:
[RESULT] FAIL tests=512 pass=508 fail_count=8 rounds=13404 plan_entries=16812

verify-scheduler-reg-wrapper:
[RESULT] FAIL tests=512 pass=508 fail_count=10 rounds=13404 plan_entries=16812
```

上述 fail 来自当前 RTL lite candidate policy 与现有 golden compact plan 的少量字段差异；
RTL/TB 均可编译，wrapper read-time S4PF walk 已移除。
