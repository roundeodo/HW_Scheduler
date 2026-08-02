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
动态起点时直接进入 transition。selector 无效、profile mode 不匹配或 start
iterator 无输出时直接跳到下一 profile。

## 3. Shared datapaths

以下算术单元均只有一套：

- transition/timeline evaluator；
- DMA interval checker；
- targeted-S4PF trial controller；
- compute/DMA bound scorer；
- regime classifier 和 global comparator。

S4PF baseline 与 target trial 顺序复用 transition evaluator。global 阶段只
replay 每个 logical winner，不再次展开 28 个 profile。bound 输入在
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

bound scorer 使用多拍 FSM：常数除法逐 bit 执行，top5 和 histogram 顺序
累计；DMA lower bound 每轮依次扫描 4 个现有 DMA 区间，再在 APPLY 周期消费
下一个 event。它不实例化组合除法器、8-event 最小值树、乘法器或 per-head
并行 scorer。

global comparator 每拍比较一个 key field，严格按 frozen F/H/C/D、regime 和
tie-break 顺序执行，并在首个不同字段立即结束当前 key。无 winner 时直接
结束，不进入 commit。

## 6. Commit and FIFO

winning token 只 replay 一次以生成 child state 和 normalized plan。core 先与
同 cluster pending task 解析 S4PF target，再写 FIFO；当前 task 随后成为新的
pending。batch 结束按 C2、C3 顺序 flush，未找到 consumer 的 descriptor 为
OFF。

FIFO 使用 3-bit head/tail 和 4-bit count；push 只写 tail，pop 只推进 head，
同拍 pop/push 不会覆盖或跳过 entry。payload 显式约束为 FF 实现，禁止 LUTRAM
和 SRL。`TASK_STREAM` 的 successful read handshake 是唯一 pop 条件。

## 7. Window and refill

提交删除 1 或 2 个 eid 后，wrapper 分别 compact hot/cold。若所有剩余 expert
已装入本地窗口，可从 cold 尾部移到 hot 尾部；存在 hidden expert 时保持 top
stream 与 bottom stream 的独立顺序。

refill 仅在 hidden 非零且 top 或 bottom 候补不超过 1 时请求。top deficit 优先，
剩余配额给 bottom；每侧最多 4、合计最多 6。RTL 锁存本次 top/bottom credit，
软件按 top 后 bottom 的顺序向同一 `REFILL_QUAD` 连续写 1 或 2 拍，最后一拍
完成事务，无单独 ACK。窗口未就绪、refill 未完成或 FIFO 满时只产生
backpressure，不运行无效 round。

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

当前版相对原始基线增加 897 LUT（14.42%）和 117 FF（7.36%），在放宽后的
15% 上限内；LUTRAM、SRL、BRAM 和 DSP 均为 0。40 MHz OOC setup slack 为
+14.036 ns。1849-round 回归由 `8,904,780 ns` 降至 `8,302,020 ns`，减少
60,276 个 10 ns 测试时钟周期（6.77%）。除 release group 跳过外，S3 cached
时每个 release 只检查唯一可达的 offset；没有任何可行 S4PF binding 时不启动
target FSM；三个不锁存数据、只产生 start pulse 的外层状态已并入前级完成周期。

相对旧功能较少的 scheduler，新策略新增资源中必要部分包括 28-entry frozen
profile decode、精确 bound/regime score、target-aware S4PF SINGLE/BOTH BW trial、
aggregate histogram/counters、hot9+cold5 refill 窗口、锁存式双拍 refill 以及
八项 task FIFO。已经删除的
非必要部分包括旧 candidate 模块链、不可达 partial-cache reservation、重复
cache/S2PF endpoint 状态、replay `profile_slot`/target `s4pf_count` 和复位型
FIFO payload FF。task FIFO 使用八个 47-bit compact FF entry；时间域、split
offset 和 block sum 分别按已证明的协议上界压缩。尝试过的
C2/C3 request 寄存共享与显式 comparator 共享均因综合后 LUT/FF 增加而撤回。
基于 28 个固定 profile 特化 child-head compaction 可将最坏逻辑级数从 42 降到
34，但 OOC LUT 增至 7298，超过 15% 上限，因此也已撤回，当前实现保留共享的
顺序 compaction 电路。
