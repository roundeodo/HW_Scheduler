# MoE Scheduler Hardware

当前目录只保留 pure-slave scheduler datapath。scheduler 不主动读写 L3，也没有内部 SRAM。完整 sorted rem stream、完整 lowered args、token metadata 仍由 CVA6/L3 维护；RTL 内部只保存当前 top window、reserve window、snap/state 和 depth=8 dense task FIFO 所需寄存器。

## RTL 层次

```text
moe_scheduler_reg_wrapper
  └── sched_schedule_core
      ├── sched_bw_ok_seq x2             # pointer-only eval/commit BW checkers
      ├── sched_candidate_generator
      ├── sched_candidate_eval_lane
      │   ├── sched_pick_shapes
      │   ├── sched_mk_timeline              # shared by EV_MK_A/EV_MK_B
      │   ├── sched_s2pf_pair            # BW client
      │   └── sched_score_unit
      ├── sched_best_reduce
      └── sched_task_word_pack                # shared compact task-word packer
```

`sched_pkg.sv` 只保留全局参数、ABI bit layout、窄基础类型、shape/best 小 helper 和必要 plan/task 类型。candidate issue 类型放在 `sched_candidate_pkg.sv`，64-bit task word 打包放在 `sched_task_word_pack.sv`。当前默认 `E_MAX=64`，`TASKQ_DEPTH=8`。

## Register Boundaries

当前第一条显式 datapath 边界在 `sched_candidate_eval_lane` 入口：

```text
sched_candidate_generator
  -> EV_LATCH: token/side_q + shape_q
  -> EV_MK_A : raw_task_a_q     # task-only timeline, no S2PF/S4PF/cache fields
  -> EV_MK_B : raw_task_b_q
  -> EV_S2PF: sched_s2pf_pair
  -> EV_SCORE: sched_score_unit
       -> optional SIM1 split request reuses the same sched_s2pf_pair
  -> EV_DONE
```

`EV_LATCH` 不再保存完整 candidate payload，也不保存 start/eid/ntok/tok_start
这类可由 token 和本轮 top window 重建的字段。它只锁存
`cand_token_t`、A/B side-valid、A/B swiglu/down hit 和最终 shape。mk/plan 阶段
需要 task 字段时，由 `req_q.token` 加本轮稳定的 head/base context 在本地组合
重建。base timeline/cache 是 round-level 状态，由 `sched_schedule_core` 持有，
在当前 candidate 评估期间保持稳定。A/B 两侧复用同一套
`sched_mk_timeline`，不再并行实例化两套 mk_timeline 组合逻辑。
`EV_MK_A/B` 的寄存器边界不保存完整
snap 结构，只保存不能由 token/context 重建的 `task_end/dma1_end/s2_end/
dma3_end/s4_start/bw_s1/bw_s3`。`task_start/ntok/valid` 在消费点由
token decode 和 side-valid 补回。S2PF 入口需要完整
`snap_timeline_t` 时，由 eval lane 在本地组合展开；BW 请求跨模块只传
`snap_bw_view_t`，避免把完整 timeline 继续广播给 core/BW checker。
cache-hit 所需的
`pf_eid/pf_end/pf_full` 不在 raw 阶段打拍，而是根据 side-valid 从 base cache
或 empty cache 组合选择。

`sched_score_unit` 不再私有例化 `sched_s2pf_pair`。当 `rem_len==1` 的 SIM1
split replay 需要 S2PF 时，score_unit 只输出 split A/B snap、shape 和 start
request；`sched_candidate_eval_lane` 复用本 lane 已有的唯一 `sched_s2pf_pair`，
并把 patch 返回给 score_unit。为了防止 score split 覆盖当前 candidate 的 S2PF
结果，eval lane 在 candidate S2PF 完成时保存一份最小 `s2pf_patch_t`。该 patch
只包含 `ok/has_a/pf_start_a/has_b/pf_start_b`；`pf_end/task_end` 由原始
timeline、shape 和 `best_s4_ticks(ntok)` 在本地推导，不作为 FF 状态保存。

`sched_s2pf_pair` 输入仍接收当前 trial 所需的 `snap_timeline_t`，但 BW
输出端口已经裁剪为 `snap_bw_view_t`。它不接收、不保存、不透传 cache identity 字段；在
`start_i` 时只锁存 S2PF 事务需要的小标量：

```text
policy, scan_idx
can[2], hi[2], dma_start_valid[2]
result_ok, result_has_pf[2], result_start[2]
```

best trial 只保存 `pf_start` 标量，不保存 patched snap，也不保存 full selector
后再反复取 start。调用者仍需要在 `busy_o` 期间保持 snap_a/b 输入稳定；
当前唯一实例由 eval lane FSM 约束输入稳定，并被 candidate S2PF 和 SIM1 split
S2PF 分时复用。

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

`best_conc` 不由 CVA6 写入，也不保存在 `head_q/reserve_q` FF 中；RTL 用 `best_conc_ticks(ntok)` 组合计算，并只在需要时扩展到 `time_t`。

## Auto-Run

steady-state 下，RTL 自己推进 round：

1. `sched_schedule_core` 完成一轮 candidate evaluation，在 core 内生成 winner 对应的 task descriptor、cluster-local `allow_s4pf`，并通过 remove handshake 向 wrapper 提交 `remove_count/remove_slot_mask`。
2. `moe_scheduler_reg_wrapper` 根据四种合法 `remove_slot_mask` 对 `head_q[4]` 做 fixed-case compact。
3. wrapper 从 `reserve_q[4]` 头部 pop 需要的 entry，补到 compact 后 top window 尾部。
4. 如果 top window 满足下一轮所需 entry 且输出 FIFO 未满，wrapper 自动启动下一轮。
5. 如果 reserve 不足，wrapper 拉高 `refill_req` 并给出 `refill_count`；CVA6 从 L3 sorted rem stream 继续取最多 4 个 entry，通过 `HEAD_PUSH_QUAD` append 到 reserve 尾部。

当前 candidate token contract 只允许四种 committed remove mask：
`0001`、`0011`、`0110`、`1100`。`0000` 只表示当前没有有效 remove
metadata，不是可提交模式。core/wrapper 对非法 mask 只保留防 latch 的默认赋值；
仿真中通过 assertion 捕捉非法 mask，不在综合路径中支持泛化 compact。

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
direction，以及 `SPLIT(top0)` 的 front_m4/front_m8/tail-side cuts。

`nr==1` 时保留 13 个候选：

```text
0..9   : SOLO(top0)，C2/C3 各 5 种高频 shape
         C/C, A/A, A/C, B/B, A/B
10     : SPLIT(top0), ceil(ntok/2)
11..12 : not-both-idle early-start SOLO，只保留 release1/release2
```

`nr==1` 删除低频 SOLO shape：B/A、B/C、C/A、C/B；删除
half_floor split；early-start 删除 idle_t，只保留忙侧 release endpoint。
`not_both_idle` 的候选集合暂时保持不变。

S2PF 仍使用 `sched_s2pf_pair` 的 lite policy，并由 `sched_bw_ok_seq`
顺序检查 BW。当前 core 例化两组 pointer-only BW checker：一组服务
eval/replay，一组服务 commit S4PF。`sched_bw_ok_seq` 内部使用有序 interval
sweep：每拍只比较 A/B 当前 pointer 指向的 segment。它不再保存
`seg_a_q[4]/seg_b_q[4]`，只保存 A/B pointer、busy/done/ok。调用者必须在
checker busy 期间保持 `snap_a_i/snap_b_i` 稳定；eval lane 和 commit FSM
分别满足这个约束。发现超带宽立即 early stop；正常完成时检查次数由有效
segment 数决定，避免固定扫描 25 对 cross product。

当前 S2PF placement 只由 `sched_s2pf_pair` 产生，并严格要求 down-weight
DMA 完整落在 `[dma1_end, s2_end]`。这与 workload DFG 中 down-weight load
依赖 S1 weight load 完成的约束一致。hardware-lite 只尝试：

```text
PAIR   : both@dma1_end, raw
SPLIT  : both@dma1_end, B-only@dma1_end, raw
SINGLE : active@dma1_end, raw
```

`sched_s2pf_pair.apply_s2pf` 也不再重复做 placement 合法性检查；合法性在
trial 生成阶段由 `try_valid` 和 `can[2]` 保证，apply
只负责 patch timeline 字段。这样避免同一个 candidate trial 里重复 duration
decode、endpoint compare 和 BW decode。

compact plan 只携带 `has_s2pf`，没有可编程 start timestamp，因此 earliest、
latest 或中间 placement 不能作为不同硬件 action。`sched_s2pf_pair` 在
`start_i` 时只锁存 `policy/can`；`can` 直接检查
`dma1_end + dur <= s2_end`。模板按最终优先级顺序访问，第一条通过 BW 的
side mask 立即结束搜索，不保存 `best_class/best_start_sum`，也不存在并行
comparator tree。

BW segment builder 不再使用动态 compact list。每侧固定 4 个 slot：

```text
slot0: S1 DMA
slot1: S2PF down-weight DMA
slot2: S3 down-weight DMA（与 slot1 互斥）
slot3: next-S1 ghost prefetch
```

`s2pf_valid` 和 S3 DMA 在当前 RTL 生产路径中互斥：`sched_s2pf_pair`
置位 `s2pf_valid` 时同步清零 `bw_s3`。因此不再为 S2PF tail 和 S3 DMA
分别保留独立 slot。

无效 slot 只拉低 `valid`，sweep 自动跳过 invalid slot。这样避免
`seg[idx]`/`idx++` 形成 variable-index write 和 compact mux 网络。
segment 记录也不再保存完整 2-bit BW 编码，只保存 `is_128`：全局 BW
检查只需要知道重叠区间是否包含 128 B/cc。64+64 合法，任何包含 128 的
cross-cluster overlap 非法，因此 sweep 阶段不需要加法器。
各 slot 只依赖 upstream 的 compact timeline contract：
跳过的 DMA 由 `BW_0` 表示，合法 S2PF 由 `s2pf_valid` 表示。BW checker
不再重复检查 `dma_end > start` 这类生产端已保证的不变量。

下一专家 S1 prefetch 从 `dma3_end` 开始，可与当前专家剩余的 S3 compute 和
S4 compute 重叠；它必须在 `task_end` 前完整结束。S4PF 只在
`sched_schedule_core` 的 commit 子状态机中做一次 BW check。
commit 阶段如果某个 cluster 的 post-down-DMA window 合法且通过 BW 检查，core 会同时：

- 生成 cluster-local `allow_s4pf`，用于 dense task 发布阶段维护 pending task 并形成 self-contained S4PF descriptor。
- 直接把 `s4pf_valid/s4pf_start` 和 `PF_EID_GHOST` 写入下一轮 persistent
  timeline/cache。

因此 `sched_schedule_core` 不再在下一轮开头保留独立 ghost-injection FSM，
也不再为同一个 S4PF window 做第二次 BW check。

## Output FIFO

`sched_schedule_core` 内部保存 depth=8 的 dense task FIFO。每个 entry 恰好是一条
64-bit task word。FIFO 不保存 round-level metadata，solo round 也只占一个 entry。

`remove_count/remove_slot_mask` 不作为 FIFO entry 的 shadow 字段保存，也不暴露给
CVA6。core 在 commit 后通过内部 valid/ready 事务把它们交给 wrapper，用于 top4
compact/refill；task FIFO 只保存 lowering/drain 真正需要的 payload。

FIFO 使用 circular `head + occupancy` 结构：只保存 3-bit head pointer 和 4-bit
count，tail 地址由二者组合推出，不保存重复 tail pointer。生产端采用单写口，commit
FSM 每拍最多发布一条 task，避免 4-word 多写口和宽 crossbar。

`STATUS` 是 fast path 的事件/metadata 入口：

```text
bit  [3]     task_valid
bit  [5]     active_empty
bit  [6]     refill_req
bits [31:28] refill_count
bits [39:36] task_fifo_count
```

CVA6 和 TB 只读 `STATUS` 获取 task FIFO 数量与 refill metadata；
没有 remove/status debug shadow，也没有额外 FIFO 状态读路径。

CVA6 可以一次 drain FIFO 中 1..8 条 task：

1. 读 `STATUS` 得到 `task_fifo_count=N`。
2. 连续读取 `TASK_FIFO_DATA(i) = 0x100 + i*8`，`i=0..N-1`。
3. lowering 后写 `TASK_POP.pop_count=N`，一次 pop 已 drain 的 N 条 task word。

每个 cluster 最多保留一条尚未知道未来 target 的 pending task。下一条同
cluster task 到来时，core 用其 eid 完成前一条 task 的 `S4PF_DESC[63:56]`，然后才
将前一条发布到 FIFO；batch 最后一条 task 以无 S4PF target 的形式 flush。因此每条
已发布 task 自己携带 S4PF target，CVA6 不再反向修改已经 lowered 的前一个 L3 slot。

## MMIO Register Map

```text
0x00 CTRL              W bit0 init, bit1 start
0x08 STATUS            R task/refill/active metadata
0x10 CONFIG            W cache_eid_c2/c3, active_count, total_conc
0x68 TASK_POP          W bits[3:0] pop_count
0xa8 HEAD_QUAD         W initial top4, head16 x4
0xb0 RESERVE_QUAD      W initial reserve[4], head16 x4
0xb8 HEAD_PUSH_QUAD    W append up to 4 head16 into reserve[4]

0x100 + i*0x08 TASK_FIFO_DATA(i) R dense FIFO task word, i=0..7
```

`TASK_FIFO_DATA` 64-bit word layout：

```text
bits [5:0]   expert id
bits [14:6]  token_start_rank
bits [23:15] ntokens
bit  [24]    has_s2pf
bits [37:25] compact ctrl word
bits [46:38] m_s2_exec
bits [55:47] m_s4_exec
bits [63:56] self-contained S4PF descriptor
```

`S4PF descriptor` bits：`bit0 valid`、`bit1 no_copy`、`bits[7:2] target_eid`。

`ctrl` bits：

```text
bit  [0]     skip_s1
bit  [1]     skip_s3
bits [3:2]   shape_s1
bits [5:4]   shape_s3
bit  [6]     cluster
bits [12:7]  local_slot
```

CVA6 根据 compact ctrl、`m_s2_exec/m_s4_exec` 和 shape 重新生成写入 dynamic args 的 20-bit runtime ctrl。S4PF descriptor 已经自包含 target expert：CVA6 只需检查 `valid/no_copy`，然后将 `target_eid` 写入当前 task 的 S4 prefetch DMA slot，不再维护跨 task pending pointer，也不再回写早先已 lowering 的 slot。

## 验证

`tb/` 目录有两层当前 RTL/ABI 验证：

- `make verify-schedule-core`：完整 RTL datapath 检查。`gen_schedule_vectors.c` 调用 `moe_make_hw_plan()` 生成 512 个 request 的 golden task stream；`tb_schedule_core.sv` 驱动 core 并逐项比较 task word 和 per-cluster `local_slot`。
- `make verify-scheduler-reg-wrapper`：把 MMIO wrapper、reserve refill、auto-run、dense task FIFO drain、compact `ctrl` word 和 self-contained S4PF target descriptor 纳入测试。

运行 Questa 前需要：

```bash
source /esat/micas-data/data/design/scripts/questasim_2022.4.rc
make -C Scheduler_hw/tb verify-schedule-core
make -C Scheduler_hw/tb verify-scheduler-reg-wrapper
```

当前 dense task FIFO 协议的 Questa 2022.4 全量结果：

```text
verify-schedule-core:
[RESULT] PASS tests=512 rounds=12060 plan_entries=16935

verify-scheduler-reg-wrapper:
[RESULT] PASS tests=512 drain_batches=15384 plan_entries=16935
```

wrapper TB 原始日志的统计字段仍名为 `rounds`，但在 auto-run + dense FIFO
协议下它统计的是 wrapper event/drain 循环，不是 algorithm round 数。

## FF Budget

按默认 `E_MAX=64/T_W=16/TASKQ_DEPTH=8`，根据源码中实际 `*_q`
状态逐项统计，wrapper 加完整 core 约为 **1544 bit FF**。该数字是综合前逻辑
寄存器预算，不包含 testbench、`ifndef SYNTHESIS` assertion history，也不等同于
FPGA implementation 的物理 FF report。

```text
wrapper head/reserve/config/count                 174
core persistent/FIFO/slot/pending/control         973
BW checker x2                                      22
candidate generator                                 6
eval scratch                                      226
shared S2PF engine                                 77
score FSM                                          20
best reducer                                       46
------------------------------------------------------
total                                            1544 bit
```

其中 dense task FIFO 是 `8 x 64 + head[2:0] + count[3:0] = 519 bit`；
C2/C3 persistent timeline 约 274 bit；两个 cluster 各保存一条尚未知道
S4PF target 的 pending task，共 78 bit。pending state 是生成自包含 S4PF
descriptor 所必需的跨 task 状态，不是第二份输出 FIFO。
