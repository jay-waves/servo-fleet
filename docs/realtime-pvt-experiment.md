# PVT 实时调度实验

实验代码位于 `rt-linux`；`main` 只保留跨平台的 2.5 ms 绝对时间循环、原子反馈快照、2 ms stale 判定和统计。WSL 只用于编译与回环功能检查，不能产出实时性结论。

## 构建与前检

```bash
cmake -S . -B build/rt-release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DFLEET_BUILD_TESTS=ON
cmake --build build/rt-release -j
ctest --test-dir build/rt-release --output-on-failure

uname -a
lscpu -e=CPU,CORE,SOCKET,MAXMHZ,ONLINE
grep -E 'CONFIG_PREEMPT(_RT)?=|CONFIG_SCHED_DEBUG=' /boot/config-$(uname -r)
ulimit -r
```

目标机应使用 PREEMPT_RT 内核。CPU0 作为 RX/IRQ 核，CPU4~7 作为大核；不设置 `isolcpus`，CPU4~7 不独占。回环 UDP 没有真实网卡 IRQ，因此本实验验证 RX 线程迁移，不验证硬件 IRQ 路径。调度或亲和性设置失败会非零退出，且程序会读回核验实际策略。

默认不调用进程级 `mlockall`，避免它只影响部分组。若要测试锁页，所有组统一增加 `--mlock`，并提前检查 `ulimit -l`。

## 负载模型

拓扑固定为 `2 UDP × 2 CAN port × 8 CAN ID = 32 motors`。每个 CAN port 独立按 1 Mbit/s、每帧保守 128 bit 串行化 PVT 命令与 Status1：

```text
8 motors × 2 directions × 128 bit × 400 Hz = 819.2 kbit/s / CAN port
```

因此每条物理 CAN 约有 18% 的仲裁、位填充和误差余量。Mock 不会补发突发：每个 port 的实际 Status1 发送间隔不小于 128 us。反馈延迟为 50~350 us，并由固定种子、CAN port、CAN ID 和 PVT 命令值的哈希确定；相同逻辑命令在各实验组得到相同延迟，不会因某组丢包而错位。

Control 每 2500 us 按绝对时间唤醒并始终发送全部 32 个 PVT。快照超过 2 ms 只会被拒绝并计入 `stale`，不会降低下一轮总线负载；任一电机默认连续 3 轮 stale 会使该次运行无效。`--stale-fail-cycles 0` 只供 WSL smoke 使用，正式实验禁止关闭。

## WCET 预试验

先固定 Control 到一个大核，运行至少 120 s：

```bash
build/rt-release/realtime_pvt_stress \
  --profile fair --control-cpus 4 --duration 120 --warmup 5 \
  --seed 0x5EED1234 --output wcet.csv
```

以 `control_exec_max_us` 作为本次观测到的 WCET 上界样本，并结合多轮 p99/max 选参数。周期固定 2500 us；若做探索性 deadline 测试，必须满足 `observed WCET < runtime <= deadline <= 2500 us`。一次压力测试不能证明理论 WCET。

## 主实验矩阵

在“不隔离 CPU”的前提下，主矩阵收紧为三个可公平执行的组：

| profile | RX | Control |
| --- | --- | --- |
| `baseline` | 完全不配置（继承） | 完全不配置（继承） |
| `fair` | `SCHED_OTHER`, CPU0 | `SCHED_OTHER`, CPU4~7 |
| `fifo` | `SCHED_OTHER`, CPU0 | `SCHED_FIFO`, CPU4~7 |

每组至少重复 10 次，使用同一种子但随机化组顺序，并保存实际顺序。示例单次命令：

```bash
sudo build/rt-release/realtime_pvt_stress \
  --profile fifo --duration 60 --warmup 5 --seed 0x5EED1234 \
  --irq-cpu 0 --control-cpus 4,5,6,7 --fifo-priority 80 \
  --output realtime-results.csv
```

不要在有的组使用 `sudo`、锁页或额外后台负载而在其他组不使用。正式采样前固定 governor/频率策略，并记录温度、内核版本、命令行和实验顺序。

`SCHED_DEADLINE` 的线程 affinity 必须覆盖所在 scheduler root domain。PREEMPT_RT 不改变这一规则，所以“只亲和 CPU4~7、但不建立相应 root domain”不是有效配置；原 B/C 不进入主矩阵。`deadline-cpuset` 仅供已由管理员把 CPU4~7 建成独立 root domain 的探索试验，这会改变“不隔离 CPU”的前提，结果必须单列，不能与以上三组作公平因果比较：

```bash
sudo build/rt-release/realtime_pvt_stress \
  --profile deadline-cpuset --duration 60 --warmup 5 \
  --control-cpus 4,5,6,7 --runtime-us 500 --deadline-us 2000 \
  --seed 0x5EED1234 --output deadline-exploratory.csv
```

## 指标与内核观测

- `jitter_*`：实际唤醒减绝对计划时间。
- `control_exec_*`：一次 Control job 的执行时间。
- `e2e_*`：PVT 提交到对应 Status1 首次被 Control 使用。
- `rx_control_*`：Linux `SO_TIMESTAMPNS` 软件 RX 时间戳到 Control 使用；这是“内核接收路径 → Control”的近似值。
- `deadline_misses`：job 完成越过下一 release；stale/rejected job 也统计。
- `cpu_utilization`、自愿/非自愿切换：只覆盖采样窗口，不含 warmup；非自愿切换是 Control 抢占次数的近似值。
- `stale`：缺失、不一致、有电机错误或内核 RX 时间戳年龄超过 2 ms 的“电机·周期”数。stale 样本仍进入端到端尾延迟统计，避免删掉最坏值。

同时采集全进程/内核调度数据：

```bash
sudo perf stat -e task-clock,context-switches,cpu-migrations,cycles,instructions \
  build/rt-release/realtime_pvt_stress \
  --profile fifo --duration 60 --warmup 5 --seed 0x5EED1234 \
  --output perf-run.csv

sudo perf sched record -a -- \
  build/rt-release/realtime_pvt_stress \
  --profile fifo --duration 60 --warmup 5 --seed 0x5EED1234
sudo perf sched latency
```

最终报告同时保留 CSV、stderr 中的 mock 命令/反馈计数、`perf stat`、`perf sched latency`、内核配置和 CPU 频率信息。任何非零退出、`mock_errors`、连续 stale、命令数不足或调度配置失败的运行都应剔除并单独说明原因。
