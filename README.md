# MoE Scheduler RTL

本目录实现面向硬件的 MoE 双 cluster 调度器。当前 RTL 只有一套实现，不保留旧候选协议、旧模块名或兼容路径。

## 设计边界

- 完整 sorted expert stream 和最终 args/plan 位于 L3，由 CVA6 管理。
- wrapper 保存当前 `head[6]` 和 `reserve[6]`，每条 expert descriptor 为 16 bit：`{valid,eid[5:0],ntok[8:0]}`。
- core 保存 C2/C3 timeline、cache identity、local slot、两个 unresolved S4PF task 和 depth-8 task FIFO。CONFIG 的 8-bit cache eid 在 wrapper 写入时压成 7-bit tagged eid，不保存无语义位。
- RTL 不包含 rem SRAM、plan SRAM、AXI master 或 DMA writer。
- 当前 HeMAiA 软件 ABI 尚未同步到本目录的 top6+reserve6 协议；不要用旧软件二进制验证这版 RTL。

## 数据流

```text
CVA6 CONFIG + HEAD[6] + RESERVE[6]
                  |
                  v
moe_scheduler_reg_wrapper
  fixed-mask compact/refill + auto-run
                  |
                  v
moe_scheduler_core
  round early-start predecode
                  |
                  v
sched_candidate_generator
  compact {valid, mode, id} token
                  |
                  v
sched_candidate_evaluator
  decode/pick -> shared timeline/remove-work scratch register
  -> timeline A -> timeline B
  -> S2PF search -> BW check -> continuation score
                  |
                  v
sched_best_candidate
  retain only best token + score
                  |
                  v
replay(best token)
  -> commit S4PF BW checks
  -> resolve previous same-cluster S4PF target
                  |
                  v
depth-8 circular task FIFO -> CVA6
```

`best` 只保存 compact token 和 score。枚举结束后 replay 一次 winning token，避免保存宽 winner timeline/plan 寄存器。
同一组 27-bit scratch FF 在前半程保存 timeline operator request，在候选完成时
改存 winner 的 removed parallel/serial work；wrapper 直接复用，
不再第二次展开 per-head work 算术。

## 候选策略

`cand_token_t` 只有 `valid/mode/id`。generator 只枚举 token，evaluator 按当前 round context 解码实际 task。

### 最后一个 expert

- C2/C3 各 5 个高频 solo shape：`C/C`、`A/A`、`A/C`、`B/B`、`A/B`。
- 一个 half-ceil split。
- busy side 的前两个有效 release time。

### 两侧同时 idle

- `PAIR(top0,top1)`
- `PAIR(top1,top2)`
- `PAIR(top2,top3)`
- `SPLIT(top0,half-ceil)`
- `SPLIT(top0,front-2)`

### 只有一侧 idle

- idle/release0/release1 上的固定 `C/C`。
- 同三个 start 上由 `ntok` 选择的 adaptive shape。

remove mask 只可能是 `0001`、`0011`、`0110`、`1100`。wrapper 对这四种模式使用固定连接，不实现通用 prefix compact 网络。

## S2PF 与 DMA 资源

`sched_s2pf_search` 按最终优先级顺序检查固定模板，第一项合法结果立即结束：

- pair：`both@dma1_end`，然后 raw。
- split：`both@dma1_end`、`B-only@dma1_end`，然后 raw。
- active side：`active@dma1_end`，然后 raw。
- disabled：只检查 raw。

所有保留模板的 start 都固定为本侧 `dma1_end`，因此 S2PF patch 只传
`valid/apply_a/apply_b`，不保存或传输重复的 start timestamp。

每个 DMA 区间在 RTL timeline 中明确保存 `{xDMA,iDMA}` 资源掩码，而不是
抽象的 64/128 B/cc 档位。固定分配规则为：C2 的单路搬运使用 iDMA，C3 的
单路搬运使用 xDMA，Shape C 使用 `BOTH`；S2PF 继承被替代 S3 的绑定，S4PF
使用本 cluster 的固定单路绑定。该规则使输出 task word 可以由
`cluster+shape` 无歧义重建绑定，不额外扩宽 ABI。

`sched_bandwidth_check` 不保存完整 segment queue。输入 timeline 在 busy 期间保持稳定，checker 每侧组合产生三个可达有序区间：S1、S2PF/S3 二选一、S4PF；两个 3-bit one-hot pointer 顺序扫描。重叠区间仅在 DMA mask 不相交时合法：iDMA+xDMA 可以并行，同名 DMA、BOTH 与任意 DMA 的重叠均非法。

eval 与 commit 各有一套 pointer-only checker，避免共享宽 mux 和跨 client 的输入保持状态。

## 评分

- last-expert 和 one-idle 候选直接使用 committed child makespan。
- both-idle 候选比较 aggregate greedy estimate 与 top4 four-step LPT projection。
- LPT 每拍放置一个 remaining work item，复用一套 compare/add datapath。
- 三个及以上独立时间项采用 3:2 compressor 后接一次 carry-propagate addition；两项加法保持普通表达式。

## MMIO 协议

所有地址为 wrapper 内 64-bit register word 的 byte offset。

| Offset | 名称 | 方向 | 内容 |
|---:|---|---|---|
| `0x00` | `CONFIG` | W | cache eid、active count、total parallel/serial work |
| `0x08` | `WINDOW0` | W | sorted rank 0..3 |
| `0x10` | `WINDOW1` | W | sorted rank 4..7，即 head4..5、reserve0..1 |
| `0x18` | `WINDOW2_START` | W | sorted rank 8..11，即 reserve2..5；该写同时 init/start |
| `0x20` | `REFILL_QUAD` | W | 向 reserve 尾部追加 1..4 条 descriptor |
| `0x28` | `EVENT_WAIT` | R | 阻塞到 refill、FIFO watermark 或 batch 完成；返回 refill/FIFO count |
| `0x30` | `TASK_STREAM` | R | 返回 FIFO head；成功读握手同时 pop 一条 64-bit task |

wrapper 自动接受 `remove_valid`，对 head 做 compact/refill，并在输入完整且 FIFO 未满时启动下一轮。CVA6 只在 `EVENT_WAIT.refill_req` 时补充 `refill_count` 条 descriptor。每次 `TASK_STREAM` 读都原子消费一条 task，输出协议没有独立读索引、读指针或确认写事务。

若提交后仍有 descriptor 尚未装入 6+6 window，wrapper 要求 reserve 在本次 head refill 后至少保留两条。该约束覆盖下一轮最多删除两个 expert 的情况，使 refill 响应延迟只造成 backpressure，不改变候选评估使用的 sorted window 或最终调度序列。

每个 task word 已包含 lowering 所需的 compact control、`m_s2/m_s4` 和自包含 S4PF descriptor。S4PF target 属于下一条同 cluster task；core 用每 cluster 一个 pending record 在 enqueue 前解析，不需要 CVA6 回写旧 plan entry。

## 模块

- `sched_pkg.sv`: 全局类型、ABI layout、窄算术 helper。
- `sched_candidate_pkg.sv`: candidate token contract 和固定策略 decode。
- `sched_candidate_generator.sv`: 顺序 token generator。
- `sched_candidate_evaluator.sv`: 单 lane 多拍 evaluator。
- `sched_task_timeline.sv`: task timeline 组合 datapath。
- `sched_pair_shape_select.sv`: pair shape 选择。
- `sched_s2pf_search.sv`: 固定模板 S2PF search。
- `sched_bandwidth_check.sv`: pointer-only ordered interval sweep。
- `sched_continuation_score.sv`: greedy/LPT continuation score。
- `sched_best_candidate.sv`: compact best reducer。
- `sched_task_word_pack.sv`: 唯一 task word packer。
- `moe_scheduler_core.sv`: round FSM、replay、commit、pending S4PF、task FIFO。
- `moe_scheduler_reg_wrapper.sv`: MMIO、top6/reserve6、auto-run/refill。

## 验证

Questa 必须在 sandbox 外运行：

```bash
source /esat/micas-data/data/design/scripts/questasim_2022.4.rc
make -C Scheduler_hw/tb verify-score
make -C Scheduler_hw/tb verify-core
make -C Scheduler_hw/tb verify-wrapper
```

当前回归结果：

```text
[RESULT] PASS continuation_score tests=2048
[RESULT] PASS scheduler_core tests=512
[RESULT] PASS scheduler_reg_wrapper tests=512
```

Python golden 的函数名仍沿用 Idea_Model 中的历史 API 名称；该名字不属于 RTL 模块或 MMIO 协议。
