# Bounded Distilled Scheduler RTL Implementation

## 1. Storage boundary

wrapper 物理保存 `hot9 + cold5`。可见窗口固定为 `hot[0:4] + cold[0]`，
`hot[5:8] + cold[1:4]` 是各 4 项候补。core 接口保留窄的 `hot[7:0]` 总线和
一个 `bottom` view，但策略 selector 只读取 T0..T4/B0；候补 descriptor 不进入
profile decode、timeline 或 score 的组合网络，`hot[8]` 只参与 wrapper 内部
compact/refill。

持久状态包括：

- C2/C3 的 task endpoint、DMA binding 和完整初始 cache 状态；
- count、token/block aggregate 和 small-block histogram；
- 每 cluster 一个尚未解析 S4PF target 的 task；
- 八个 47-bit entry 的环形 FF FIFO。

不保存完整 candidate timeline、candidate plan 或 rem array。

## 2. Profile execution

28 个 physical profile 由 5-bit ROM index 解码，按 terminal 5、sync 8、
one-idle 15 分组。profile 带 `logical_id/logical_last`，硬件在同一 logical
action 内只保留一个 baseline token 和一个 targeted-S4PF token。一个 logical
group 结束后立即 replay/score，并与单个 global incumbent 比较；没有
final-token bank。

SINGLE profile 复用 `sched_distilled_start_iter` 枚举有效起点。PAIR/SPLIT 没有
动态起点时直接进入 transition。进入 evaluator 前，硬件按 family 检查 selector
valid、PAIR eid 不同、有效 cluster 数、SPLIT 最小 token 数和不平衡 split 的
偶数约束；结构上不可能合法的 profile 直接跳过。profile mode 不匹配或 start
iterator 无输出时同样直接进入下一 profile。PAIR 的第二 assignment 从
`ST_NEXT_ACTION` 直接启动 evaluator，不经过独立 start state。

## 3. Shared datapaths

以下算术单元均只有一套：

- transition/timeline evaluator；
- DMA interval checker；
- targeted-S4PF trial controller；
- compute/DMA bound scorer；
- regime classifier 和 global comparator。

S4PF baseline 与 target trial 顺序复用 transition evaluator。score 阶段只对
每个 logical winner 做一次完整 replay，不再次展开该 group 的 physical
profiles。bound 输入在
`ST_SCORE_EVAL_WAIT -> ST_BOUND_START` 边界寄存，截断 profile/selector 到
bound 的长组合路径；这个边界使用已有状态转换，不增加额外 FSM state。

跨模块接口使用窄 view：bound 只接收需要的 cluster/counter/head 字段，S4PF
只接收 consumer 的 valid/cluster/skip_s1/has_s2pf 以及必要 timeline 字段。

## 4. Bandwidth circuit

DMA checker 将每侧可达区间按 S1、S2PF 或 S3、S4PF 顺序生成。两个 3-bit
pointer 逐项 sweep，不建通用 segment queue。重叠区间仅在 DMA mask 不相交时
合法；`BOTH` 与任意有效 DMA 重叠均非法。

transition evaluator 和 targeted-S4PF controller 分时复用 round engine 内唯一的
checker。owner 位只在发起检查时更新，两个 client 在状态机上互斥。

## 5. Bounds and comparison

bound scorer 保留一套多拍数据通路，但将除 6 改为精确的常数组合商/余数，
删除 bit-serial divisor 状态和寄存器。top5 仍顺序消费；同一 histogram bucket
每拍最多执行两次有依赖的 greedy assignment，DMA lower bound 每拍扫描两个
interval。三项 work 求和先用显式 3:2 carry-save compressor 压成 sum/carry，
再进行一次 carry-propagate addition；没有适合 4:2 compressor 的四个同宽独立
操作数，因此不增加该网络。

global comparator 每拍按 frozen F/H/C/D、regime 和 tie-break 顺序比较最多两个
相邻字段，第二字段只在第一字段相等时生效，并在首个不同字段立即结束当前
key。无 winner 时直接结束，不进入 commit。

## 6. Commit and FIFO

logical winner 的完整 score replay 结果被直接用于 global compare。最终 global
winner 提交时复用同一 transition evaluator 的 `rebuild_only` 模式，只构造 1..2 个
timeline endpoint、child state、normalized plan、remove 信息和 counter；该路径
跳过 BW、gain、bound 和 comparator。这样不需要为完整 child/plan 增加 winner
寄存器，也不复制 timeline 组合数据通路。core 先与同 cluster pending task 解析
S4PF target，再写 FIFO；当前 task 随后成为新的 pending。batch 结束按 C2、C3
顺序 flush，未找到 consumer 的 descriptor 为 OFF。

FIFO 使用 3-bit head/tail 和 4-bit count；push 只写 tail，pop 只推进 head，
同拍 pop/push 不会覆盖或跳过 entry。payload 显式约束为 FF 实现，禁止 LUTRAM
和 SRL。`TASK_STREAM` 的 successful read handshake 是唯一 pop 条件。

## 7. Window and refill

提交删除 1 或 2 个 eid 后，wrapper 分别 compact hot/cold。keep bit 通过
offset 1/2/4/8（cold 为 1/2/4）的并行前缀网络生成稳定目的下标，再并行写入
compact view；该网络是组合 wire，不新增 queue storage。若所有剩余 expert 已
装入本地窗口，可从 cold 尾部移到 hot 尾部；存在 hidden expert 时保持 top
stream 与 bottom stream 的独立顺序。

初始化 start write 由 `active_count` 决定：1..4 为 WINDOW0、5..8 为 WINDOW1、
9..12 为 WINDOW2、13..14 为 WINDOW3_START。软件仅写到最后一个有效窗口，
因此 8-expert 序列不再写空 WINDOW2 和 WINDOW3_START；前后两个 fence 保留，
等待完整 SoC 上进一步证明 CVA6/MMIO ordering 后再缩减。

refill 仅在 hidden 非零且 top 或 bottom 候补不超过 1 时请求。top deficit 优先，
剩余配额给 bottom；每侧最多 4、合计最多 6。RTL 锁存本次 top/bottom credit，
软件按 top 后 bottom 的顺序向同一 `REFILL_QUAD` 连续写 1 或 2 拍，最后一拍
完成事务，无单独 ACK。第一个 finalized task 进入 FIFO 后立即唤醒 CVA6，
使完整 task record lowering 与后续 RTL round 重叠。窗口未就绪、refill 未完成
或 FIFO 满时只产生 backpressure，不运行无效 round。

## 8. Synthesis interpretation

验收频率为 40 MHz，即 25.000 ns。OOC synthesis top 仅为
`moe_scheduler_reg_wrapper` 及其 Scheduler_hw 子模块。资源、fanout 和时序报告
可用于模块级风险判断，但不能代替集成后的 clock-tree、placement 和 routing。

同一 Vivado 2025.2 OOC flow 的本轮结果为：

| 版本 | LUT | FF | LUTRAM | WNS | worst levels |
|---|---:|---:|---:|---:|---:|
| 原始较少功能基线 | 6220 | 1590 | 0 | - | - |
| 新策略功能版 | 8539 | 1424 | 40 | +12.274 ns | 46 |
| 协议升级前资源优化版 | 6300 | 1382 | 0 | +14.150 ns | 41 |
| 4+4 reserve / FIFO8 初版 | 7136 | 1711 | 0 | +13.835 ns | 42 |
| 当前条件跳过 / FIFO8 版 | 7117 | 1707 | 0 | +14.036 ns | 42 |
| 当前增量 target / FIFO8 版 | 7057 | 1707 | 0 | +13.937 ns | 42 |
| 当前执行流 / 前缀 compact 版 | 7592 | 1685 | 0 | +14.618 ns | 39 |

当前版为 7592 LUT、1685 FF，满足 8000 LUT / 1750 FF 上限；LUTRAM、SRL、
BRAM、URAM 和 DSP 均为 0，40 MHz OOC setup slack 为 +14.618 ns。round engine
为 6852 LUT / 590 FF，其中本体 3064 LUT / 290 FF；bound、pair comparator 和
transition evaluator 分别为 1224/131、403/10 和 1471/108 LUT/FF。

1849-round 回归由本轮优化前的 `8,267,470 ns` 降至 `4,089,370 ns`，减少
417,810 个 10 ns 测试时钟（50.54%）。48 个 S4 定向 round 从 `303,840 ns`
降至 `208,520 ns`，减少 9,532 个时钟（31.37%）。除结构无效 profile 与 release
group 跳过外，S3 cached 时每个 release 只检查唯一可达的 offset；没有可行 S4PF
binding 时不启动 target FSM；target rebuild 复用 baseline endpoint，只重建 S1
由 miss 变为 hit 的 cluster。

相对旧功能较少的 scheduler，新策略新增资源中必要部分包括 28-entry frozen
profile decode、精确 bound/regime score、target-aware S4PF SINGLE/BOTH BW trial、
aggregate histogram/counters、hot9+cold5 refill 窗口、锁存式双拍 refill 以及
八项 task FIFO。已经删除的
非必要部分包括旧 candidate 模块链、不可达 partial-cache reservation、重复
cache/S2PF endpoint 状态、replay `profile_slot`/target `s4pf_count` 和复位型
FIFO payload FF。task FIFO 使用八个 47-bit compact FF entry；时间域、split
offset 和 block sum 分别按已证明的协议上界压缩。完整锁存 global winner 的
child/plan/counter 实验为 7675 LUT / 1851 FF，超过 FF 上限；复制两套组合
timeline 重建单元为 8772 LUT / 1683 FF，超过 LUT 上限。最终共享 endpoint-only
rebuild 为 7592 LUT / 1685 FF，是同时满足两个资源约束的实现。
