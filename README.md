# Bounded Distilled MoE Scheduler RTL

本目录实现 `bounded-distilled-top5-bottom1-targeted-s4pf` 双 cluster 调度策略。
RTL 观察 `top5 + bottom1`，顺序执行 28 个硬编码 physical profile，并与
`Idea_Model/scheduler_rtl_distilled_policy.py` 保持整数、确定性 lockstep。

## 设计边界

- 软件在 L3 保存完整降序 expert stream；RTL 不复制 rem/plan SRAM。
- wrapper 保存 `hot9 + cold5`：可见窗口为 `hot[0:4] + cold[0]`，其余
  `hot[5:8] + cold[1:4]` 是各 4 项的本地候补。
- 每项 descriptor 为 `{valid, eid[5:0], ntok[8:0]}`，共 16 bit。
- core 保存 C2/C3 状态、aggregate counters、每 cluster 一个 pending task，
  以及八个 47-bit entry 的 FF FIFO；64-bit task word 只在 FIFO head 处生成。
- 顶层是 `moe_scheduler_reg_wrapper`；RTL 不含 AXI master、DMA writer、
  Bingo 或完整 SoC。

## 硬件数据流

```text
hot9 + cold5 + aggregate state
            |
            v
28-entry combinational profile ROM
            |
            v
shared start iterator -> shared transition/BW evaluator
            |
            v
per-logical-action baseline/target reducer
            |
            v
shared bound scorer -> shared regime comparator
            |
            v
winner replay/commit -> pending S4PF target resolution
            |
            v
eight-entry compact FF FIFO
```

只有当前 profile、当前 logical group 的局部 winner 和一个 global incumbent
被保存；logical group 结束后立即 score 并折叠到 global incumbent，不保存
winner bank。宽 timeline 和 plan 不做 per-candidate 复制。baseline 与
targeted-S4PF 复用同一 transition evaluator，target 只有使当前 `max_end`
严格减小时才替换 baseline。

状态机允许以下条件跳过：非法 selector、缓存条件不符、无可用 start、BW
失败、S4PF 剩余 expert 少于 9、计算窗口不足、无 logical winner，以及无需
refill 的轮次。不存在为了固定拍数而继续执行的空 pass。

## DMA 与 prefetch

- C2 single lane 为 iDMA，C3 single lane 为 xDMA，`BOTH` 同时占两条 lane。
- S1、S2PF、S3 和 targeted-S4PF 均使用显式 DMA mask 做区间冲突检查。
- S2PF 模式由 28 个 frozen profile 决定，支持合法的 SINGLE/BOTH 组合。
- S4PF 对具体的下一条同 cluster consumer 评估，依次尝试本地 SINGLE、BOTH、
  OFF；计算窗口和全局 BW 必须同时合法。
- 前一条 task 在 target 未知时留在每 cluster 的 pending register；看到下一条
  同 cluster task 后再写入自包含 S4PF descriptor。batch 尾部直接 flush OFF。

## MMIO 协议

地址是 64-bit register word 的 byte offset。

| Offset | 名称 | 方向 | 语义 |
|---:|---|---|---|
| `0x00` | `CONFIG` | W | cache eid、active count |
| `0x08` | `WINDOW0` | W | `hot[0:3]` |
| `0x10` | `WINDOW1` | W | `hot[4:7]` |
| `0x18` | `WINDOW2` | W | 初始化序列 `[8:11]` |
| `0x20` | `REFILL_QUAD` | W | 同一 refill 连续写 1..2 个 quad |
| `0x28` | `EVENT_WAIT` | R | 阻塞到 refill、FIFO watermark 或 batch done |
| `0x30` | `TASK_STREAM` | R | 阻塞读 FIFO head，握手即 pop |
| `0x38` | `AGGREGATE` | W | token/block/histogram counters |
| `0x40` | `WINDOW3_START` | W | 初始化序列 `[12:13]`，并 init/start |

`EVENT_WAIT` 返回：`done[0]`、`refill_req[1]`、`top_count[4:2]`、
`bottom_count[7:5]`、`task_count[11:8]`。初始化序列固定为最多 9 项 top prefix，
随后最多 5 项 cold-to-hot bottom suffix。单次 refill 每侧不超过 4、合计不超过
6；软件将 top 放在前、bottom 放在后，连续写一个或两个 `REFILL_QUAD`。RTL
锁存 credit，最后一拍写握手就是 refill completion，不需要 ACK、TASK_POP 或
轮询 status。output FIFO watermark 为 6，refill、output 和 done 共用同一个
阻塞 event。

64-bit `TASK_STREAM` 保留原有低位 task/control 排列。仅将原先空闲的 bit
定义为 `S1_BOTH[46]` 和 `LATE_BOTH[55]`；`M_S2[45:38]`、`M_S4[54:47]`
仍是 8-bit tile count。`S4PF_DESC[63:56]` 为
`{target_eid[5:0], op[1:0]}`，其中 `NONE=0`、`SINGLE=1`、`BOTH=2`、
`NO_COPY=3`。reader、阻塞语义和读取顺序不变。

## 模块

- `sched_distilled_profile_decode.sv`: 28-entry hard-wired profile ROM。
- `sched_distilled_start_iter.sv`: 固定 start-point iterator。
- `sched_distilled_timeline.sv`: 单 task timeline 算术。
- `sched_bandwidth_check.sv`: pointer-based ordered DMA interval sweep。
- `sched_distilled_transition_eval.sv`: 共享 transition evaluator。
- `sched_distilled_target_s4pf.sv`: target-aware SINGLE/BOTH/OFF trial。
- `sched_distilled_bound_score.sv`: 顺序 compute/DMA lower bound scorer。
- `sched_distilled_regime_classify.sv`: frozen regime predicates。
- `sched_distilled_pair_compare.sv`: 单个全局 comparator。
- `sched_distilled_round_engine.sv`: local reduction、global score 和 replay。
- `moe_scheduler_core.sv`: persistent state、pending target 和 task FIFO。
- `moe_scheduler_reg_wrapper.sv`: hot/cold 窗口、refill 和 blocking MMIO。

## 验证边界

Questa 回归由 Python golden 直接生成 vector：

```bash
source /esat/micas-data/data/design/scripts/questasim_2022.4.rc
make -C Scheduler_hw/tb verify-distilled-profile
make -C Scheduler_hw/tb verify-distilled-timeline
make -C Scheduler_hw/tb verify-dma-resource
make -C Scheduler_hw/tb verify-distilled-transition
make -C Scheduler_hw/tb verify-distilled-bound
make -C Scheduler_hw/tb verify-distilled-compare
make -C Scheduler_hw/tb verify-distilled-regime
make -C Scheduler_hw/tb verify-distilled-round
make -C Scheduler_hw/tb verify-distilled-s4-round
make -C Scheduler_hw/tb verify-wrapper
```

40 MHz 检查只对上述 scheduler source 以 `moe_scheduler_reg_wrapper` 为 top
执行 Vivado OOC synthesis；它不是完整 SoC placement/routing 结论。

本轮同条件 OOC 优化前后结果：

| 版本 | LUT | FF | LUTRAM | WNS @ 40 MHz | 最坏逻辑级数 |
|---|---:|---:|---:|---:|---:|
| 原始较少功能基线 | 6220 | 1590 | 0 | - | - |
| 新策略功能版 | 8539 | 1424 | 40 | +12.274 ns | 46 |
| 协议升级前资源优化版 | 6300 | 1382 | 0 | +14.150 ns | 41 |
| 4+4 reserve / FIFO8 初版 | 7136 | 1711 | 0 | +13.835 ns | 42 |
| 当前条件跳过 / FIFO8 版 | 7117 | 1707 | 0 | +14.036 ns | 42 |

当前版相对原始基线增加 897 LUT（14.42%）和 117 FF（7.36%），满足放宽后的
15% LUT/FF 上限；LUTRAM、SRL、BRAM 和 DSP 均为 0。相对协议升级前版本的增量
用于 `hot9+cold5` 本地窗口、锁存式双拍 refill transaction 和八个 47-bit FF
output entry。主要策略侧收益仍来自共享 DMA checker、删除不可达 partial-cache
状态以及压缩时间/计数/replay/task entry 位宽。当前控制还会跳过不可能的 S4PF
搜索、S3 cached 下六个恒无效 offset，以及三个只负责启动下级 FSM 的空状态。
1849-round 回归由 `8,904,780 ns` 降至 `8,302,020 ns`，即减少 60,276 个
10 ns 测试时钟周期（6.77%）。
