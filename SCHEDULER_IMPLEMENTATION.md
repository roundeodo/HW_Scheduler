# MoE Scheduler RTL 实现说明

## 1. 目标

当前实现针对 ASIC/FPGA 数据通路重新组织调度算法：使用 compact token、顺序搜索、资源复用、固定连接 compaction 和 replay，避免把软件中的候选对象、通用列表处理和全量搜索机械展开成组合网络。

约束如下：

- `E_MAX=64`，expert id 为 6 bit。
- `head[6] + reserve[6]`，每项 16 bit，共 192 bit。
- 时间统一使用 11264 cycles/tick，`T_W=16`。
- 输出为 depth-8 circular FIFO，每项 64 bit。
- 软件保留完整 sorted stream 和最终 L3 args storage。

## 2. 状态所有权

### Wrapper

- C2/C3 initial cache eid。
- head6、reserve6 及其 3-bit count。
- active expert count。
- remaining parallel/serial work aggregate。
- auto-run enable。

wrapper 不保存 `best_conc` shadow，也不重复计算 winner 的 remove work。evaluator 在 replay 中产生 `removed_parallel_work/removed_serial_work`，与 remove metadata 一起交给 wrapper 更新 aggregate。

### Core

- C2/C3 timeline 和 cache identity。
- C2/C3 local slot counter。
- round early-start context。
- unresolved S4PF task，每 cluster 一项。
- circular task FIFO data/head/count。
- round FSM 和 commit allow bit。

### Evaluator

- sampled candidate mode/id identity；valid 由 start handshake 表示。
- 四个 selected shape。
- 一组 27-bit scratch：前半程保存 timeline operator request，完成时复用为
  removed parallel/serial work 结果。
- A/B minimal raw timing，只保存四个跨多拍需要的 endpoint。
- evaluator FSM state；结果 valid 由 retained S2PF result 组合推出。
- evaluator、S2PF 和 score FSM 状态。

S2PF search 自己保持最终 `valid/apply`，直到下一次 `start_i`；所有保留模板的
start 固定为本侧 `dma1_end`，因此不保存 start FF。evaluator
不再复制第二份 accepted patch。不保存 full candidate、full raw snap、cache
shadow 或 lowering/debug 字段。

### 逻辑寄存器预算

按默认参数 `E_MAX=64`、`T_W=16`、`TASKQ_DEPTH=8` 从 RTL 状态字段逐项计算：

| 区域 | bit |
|---|---:|
| wrapper：cache eid、head6、reserve6、aggregate/count/auto-run | 252 |
| core persistent/control：timeline/cache/slot/early/S4PF/remove/FSM | 347 |
| depth-8 task FIFO：8x64 data + head/count | 519 |
| per-cluster pending task + valid + emit index | 79 |
| candidate evaluator scratch（含 27-bit request/work union） | 172 |
| S2PF search state | 9 |
| continuation score state | 67 |
| candidate generator | 6 |
| best candidate token + score | 46 |
| two pointer-only BW checkers | 16 |
| **合计** | **1513** |

其中 FIFO payload 占 512 bit；不计 FIFO payload 时为 1001 bit。这是源码层面的
逻辑状态位，不是综合后的 FD/LUTRAM/BRAM utilization；综合器是否把 FIFO data
映射成存储资源需要以后端报告确认。

## 3. Round 控制

```text
ROUND_START
  latch early-start context
      |
EVAL_START
  clear best + start generator
      |
EVAL_ISSUE <----------------------+
  sample token                     |
      |                            |
EVAL_WAIT                          |
  decode/pick/mkA/mkB/S2PF/score   |
      |                            |
  generator advance ---------------+
      |
REPLAY_START / REPLAY_WAIT
  recompute full winner from best token
      |
COMMIT_S4PF_C2 / COMMIT_S4PF_C3
  sequential BW legality checks
      |
COMMIT_EMIT_PREV / COMMIT_EMIT_CUR
  resolve pending target + one-port FIFO write
      |
COMMIT_FINISH
  update timeline/cache/slot + remove_valid
```

`remove_valid/ready` 是唯一 round completion transaction。不存在第二套 `done` 协议。

## 4. Candidate 表达

跨 generator/evaluator 的数据只有：

```systemverilog
typedef struct packed {
  logic                 valid;
  cand_mode_t           mode;
  logic [CAND_ID_W-1:0] id;
} cand_token_t;
```

evaluator 使用 token、head、base timeline/cache 和 early context 组合恢复两侧 task identity。generator 不构造 eid/ntok/start/shape/remove mask payload，因此 head 数据不会扇出到一个宽 candidate bus。

`sched_best_candidate` 只保存 `best_token_q + best_score_q`。remove mask/count 由 token 组合推出，winner plan 在 replay 中重建。

## 5. 时间 datapath

`sched_task_timeline` 在一个组合 stage 中完成一侧 timeline：

- shape 只解码一次，生成 stage-local attributes。
- `ceil(ntok/2)` 只计算一次，S2/S4 duration 通过 shift 复用。
- `s2_end/dma3_end/task_end` 共享 `start+s1_compute+s2_duration` 的第一层
  3:2 compressor；各 endpoint 分支最后只做一次 CPA，不重复展开公共前缀。
- 不输出 lowering-only scalar。

A/B 复用同一个 timeline instance 和一组 27-bit request/work scratch，以两拍换取
组合面积和输入 fanout 的下降；request register 把 token/head/cache decode 与
endpoint arithmetic 分开，但不增加 evaluator cycle。

## 6. S2PF datapath

`sched_s2pf_search` 是 priority-first fixed-template FSM：

1. 生成当前 template 的 side mask；start 固定取本侧 `dma1_end`。
2. 形成带具体 `{xDMA,iDMA}` 绑定的窄 `snap_bw_view_t`。
3. 发起 DMA 资源冲突检查。
4. 第一项合法结果立即完成。

trial 顺序已经编码最终优先级，因此不保存 provisional winner、start sum 或 class comparator。

输出 `s2pf_patch_t` 只有 `valid/apply_a/apply_b`。`pf_start/pf_end/task_end`
在使用点由 timeline/shape/ntok 计算，不作为 patch FF 或跨模块宽总线。

## 7. DMA 资源 checker

每侧仅有三个可达区间：

1. S1 DMA。
2. S2PF 或 S3 DMA，二者互斥。
3. S4PF window。

每个 segment 保存具体资源掩码：`IDMA=01`、`XDMA=10`、`BOTH=11`。
单路资源由 cluster 固定分配，Shape C 占用两条 lane。S2PF/S4PF 都固定使用
`BOTH`：S2PF 为 `dma1_end` 起始的 1-tick 区间，S4PF 为 `dma3_end`
起始的 2-tick 区间。checker 使用两个 3-bit one-hot pointer 做
ordered interval sweep。segment 从稳定输入组合生成，不保存 6 个 segment
record。每拍只比较当前两个区间：

```text
overlap = a.hi > b.lo && b.hi > a.lo
bad     = overlap && |(a.dma_mask & b.dma_mask)
```

结束较早的一侧前进；相同 end 时两侧同时前进。遇到 `bad` 立即结束。

eval 和 commit 分别实例化一个小 checker。相较共享 checker，这避免宽 client mux、owner state 和跨阶段输入保持寄存器。

## 8. Continuation score

`sched_continuation_score` 的输入只有 work summary，不接 expert identity。

- `rem=0/1/2` 使用固定表达式。
- `rem>2` 先计算 aggregate greedy cost，再用四拍把 child top4 顺序放到较空闲 cluster。
- tail aggregate 根据两侧 load gap 计算最终 LPT bound。
- 一套 comparator/adder 被四次 placement 复用。

该结构避免 parallel child expansion 和多候选 comparator tree。

## 9. Compact/refill

合法 remove mask 只有四种，因此 wrapper 直接连接 survivor：

```text
0001: [1,2,3,4,5]
0011: [2,3,4,5]
0110: [0,3,4,5]
1100: [0,1,4,5]
```

reserve pop 只可能是 0/1/2，使用 fixed shift case。reserve append 根据 count 使用 fixed destination case。正常综合路径没有 variable-index write network 或 generic prefix compact。

MMIO quad/pair 由协议保证 valid 是紧凑前缀，数量译码只覆盖
`0000/0001/0011/0111/1111` 与 `00/01/11`，不为不可达的稀疏 mask
保留通用 popcount 加法树。

被删除 expert 的 parallel/serial work 只在 evaluator 中对选中的 1 或 2 个 head
计算一次；replay 结果直接供 wrapper 更新 aggregate，不再在两个模块各展开四套
work helper。

## 10. Dense FIFO 与 S4PF target

每个 FIFO entry 是单 task 64-bit word，不按 round 存 pair structure。一轮产生 1 或 2 个 task 时分别 enqueue 1 或 2 项。

S4PF target 只有看到下一条同 cluster task 时才能确定。core 为 C2/C3 各保存一个 pending record：

- 当前 task 允许 S4PF：先进入对应 pending。
- 下一条同 cluster task 到来：用其 eid 完成旧 pending 的 S4PF descriptor，再写 FIFO。
- 当前 task 不允许 S4PF：直接写 FIFO。
- batch 结束：flush 剩余 pending，S4PF descriptor 无 target。

因此 FIFO 中每个 word 都已自包含，CVA6 不需要 pending patch 或回写旧 entry。

## 11. MMIO 快速路径

- read path 只有阻塞 `EVENT_WAIT` 和当前 FIFO head 的 `TASK_STREAM`。
- 没有 plan-status/debug register。
- xbar 已完成地址区间路由，wrapper 不重复实现非法 local-offset error 检查。
- 三个 64-bit window write 一次装入完整 6+6 初始窗口，最后一个 write 同时 init/start。
- refill 使用一个 packed quad write，一次追加 1..4 个 16-bit descriptor。
- wrapper 自动 remove/start；CVA6 不做 per-round 启动事务。
- 每次 `TASK_STREAM` 读返回并原子 pop 一条 task，删除 indexed read 和独立 pop transaction。
- 提交门控保留下一轮最多两条 expert 的 reserve lookahead，refill 延迟只产生 backpressure。

## 12. 命名与综合原则

- 模块名描述功能，不携带版本号。
- `parallel_work/serial_work` 描述评分语义，不使用含糊的 `best_conc/best_task` 状态名。
- `C2/C3` 用于物理 cluster state，`A/B` 只用于 evaluator 的局部两侧数据。
- 只保存事实状态，不保存由 token、ntok 或 count 可重建的派生状态。
- 两操作数加法交给综合器；只有 3 个以上独立操作数才显式使用 compressor。
- 顺序搜索、one-hot pointer 和固定 case 优先于通用 crossbar/priority network。
- 非法 token/remove mask 只在 `ifndef SYNTHESIS` assertion 中检查，不进入综合恢复路径。

## 13. 当前验证

```text
continuation score: 2048 cases PASS
core batch trace:    512 cases PASS
wrapper MMIO batch:  512 cases PASS
```

wrapper 回归覆盖 top6/reserve6 初始化、多轮 auto-run、fixed compact、refill、dense FIFO read/pop 和最终 makespan。

## 14. 集成状态

`Scheduler_hw/Bender.yml` 已只列出当前模块。`HeMAiA` 内已有 checkout/生成产物仍可能引用旧 RTL；当前任务明确不修改软件端，因此下一次 SoC 集成必须同时更新 HeMAiA source mapping 与 CVA6 ABI，不能混用旧 top4 register layout。
