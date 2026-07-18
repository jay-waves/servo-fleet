# PVT 实时调度实验

实验代码位于 `rt-linux` 分支；`main` 只包含跨平台的 2.5 ms PVT 稳定循环、原子反馈快照和 stale 统计。当前 WSL 只用于编译、回环功能和失败路径检查，不能作为实时性结果。

## 构建与准备

```bash
cmake -S . -B build/rt-release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DFLEET_BUILD_TESTS=ON
cmake --build build/rt-release -j
ctest --test-dir build/rt-release --output-on-failure
```

目标机先确认 CPU 编号和内核支持：

```bash
lscpu -e=CPU,CORE,SOCKET,MAXMHZ,ONLINE
grep -E 'CONFIG_PREEMPT(_RT)?=|CONFIG_SCHED_DEBUG=' /boot/config-$(uname -r)
ulimit -r
ulimit -l
```

CPU0 在本实验中代表 RX/IRQ 核，CPU4~7 代表大核；本地回环没有真实网卡 IRQ，因此这里只验证线程迁移和调度，不验证硬件 IRQ 链路，也不配置 CPU 隔离。`SCHED_FIFO`、`SCHED_DEADLINE` 和 `mlockall` 需要相应权限，可用 root 运行，或按现场安全策略授予 `CAP_SYS_NICE`、`CAP_IPC_LOCK`。配置失败时程序会非零退出，不会静默降级。

## 先测 WCET 与参数

固定 RX 在 CPU0、Control 单独放 CPU4，用较长时间估算单核控制计算尾延迟：

```bash
build/rt-release/realtime_pvt_stress \
  --profile fair --control-cpus 4 --duration 120 --warmup 5 \
  --seed 0x5EED1234 --no-mlock --output wcet.csv
```

查看 `control_exec_max_us` 和 `control_exec_p99_us`。控制周期固定为 2500 us；默认 deadline 为 2000 us、runtime 为 500 us。根据目标机 WCET 留出余量后，用 `--runtime-us` 和 `--deadline-us` 调整，并保持：

```text
WCET < runtime <= deadline <= 2500 us
```

## 五组实验

每组使用相同种子，建议至少重复 10 次。以下命令会把每次结果追加到同一 CSV：

```bash
for profile in baseline fair fifo deadline-control deadline-all; do
  for repeat in $(seq 1 10); do
    sudo build/rt-release/realtime_pvt_stress \
      --profile "$profile" --duration 60 --warmup 5 \
      --seed 0x5EED1234 --irq-cpu 0 --control-cpus 4,5,6,7 \
      --runtime-us 500 --deadline-us 2000 --output realtime-results.csv
  done
done
```

配置对应关系：

| profile | RX | Control |
| --- | --- | --- |
| `baseline` | 完全不配置（继承） | 完全不配置（继承） |
| `fair` | `SCHED_OTHER`, CPU0 | `SCHED_OTHER`, CPU4~7 |
| `fifo` | `SCHED_OTHER`, CPU0 | `SCHED_FIFO`, CPU4~7 |
| `deadline-control` | `SCHED_OTHER`, CPU0 | `SCHED_DEADLINE`, CPU4~7 |
| `deadline-all` | `SCHED_DEADLINE`, CPU4~7 | `SCHED_DEADLINE`, CPU4~7 |

压测创建两个 UDP 回环 mock，每路 16 个电机。Control 以 400 Hz 批量发送 PVT；每个 mock 用固定种子产生 50~350 us 延迟，并按每帧 128 us（1 Mbit/s 经典 CAN 保守估算）串行返回 Status1。16 × 400 × 128 = 819.2 kbit/s，因此命令和反馈各自不超过每路的方向带宽上限。该模型分别限制网关发令方向和反馈方向，不等价于把双向报文放在一条半双工物理 CAN 上做仲裁；实机总线利用率仍需用 CAN 分析仪确认。

CSV 重点字段：

- `jitter_*`：实际唤醒时间减绝对计划时间；计划时间始终累加 2500 us，不随本轮迟到漂移。
- `control_exec_*`：读取 32 份快照、拒绝 stale、计算目标并提交批命令的耗时，可作为 WCET 样本。
- `e2e_*`：批命令提交前时间戳到对应 Status1 首次被 Control 使用的延迟。
- `rx_control_*`：SDK 从内核 `recvfrom` 返回后记录的 RX 时间戳到 Control 使用快照的延迟；`max` 是“内核 socket 出队 → Control 使用”的近似最大值。
- `deadline_misses`：本轮 Control 完成时间越过下一次 2.5 ms release 的次数。
- `cpu_utilization`：Control 线程 CPU time / wall time；`involuntary_switches` 作为调度抢占次数，`voluntary_switches` 主要包含周期等待。
- `stale`：快照缺失、不来自同一 Status1，或年龄超过 5 ms 的累计“电机·周期”次数。

需要同时观察内核调度时，可单独运行一组：

```bash
sudo perf stat -e task-clock,context-switches,cpu-migrations,cycles,instructions \
  build/rt-release/realtime_pvt_stress \
  --profile deadline-control --duration 60 --warmup 5 \
  --seed 0x5EED1234 --output perf-run.csv

sudo perf sched record -a -- \
  build/rt-release/realtime_pvt_stress \
  --profile deadline-control --duration 60 --warmup 5 --seed 0x5EED1234
sudo perf sched latency
```

报告时保留内核版本、CPU governor/频率、命令行、CSV、`perf stat` 和 `perf sched latency`；不要混用不同种子或不同 runtime/deadline 的结果。
