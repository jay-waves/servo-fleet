Linux 实时 PVT 压测与调度对照实验见
[docs/realtime-pvt-experiment.md](docs/realtime-pvt-experiment.md)。

### 项目架构图

* udp_channel 封装 ASIO，管理发送队列、收发包
* xxx_driver 设备驱动，主要封装设备协议，可以管理多个设备
* xxx_device 是设备本体，存储实际设备状态
* xxx_device_interface 是设备接口，一个装饰器类，只有接口
* runtime 统一管理 channel, driver, device 的生命周期


## 编程细节

### C++代码风格

使用标准库命名方式：
* 宏、常量 使用 UPPER_SNAKE_CASE
* 除此之外全部使用 snake_case
* 最大行宽为 100c ，但日志或格式化字符可以更宽。

### 量纲约定

* current: ampere 
* voltage: volt

### 错误处理

**尽可能不使用异常机制**。使用 `std::expected`，`std::error_code` 或 `std::optional` 来表示异常状态，定义如下：

```cpp
// in device.h

using err_code = std::error_code;

template<typename T> using result = std::expected<T, std::error_code>;

using fail = std::unexpected<std::error_code>;

```

异常码定义见 `enum class fleet_err` 。该类型与 `std::error_code` 完全兼容，可以传入 `std::expected`

```cpp
enum class fleet_err {
  ok = 0,
  invalid_argument,   // 参数错误
  unsupported,        // 类型、功能不支持
  busy,               // 队列满、总线忙、设备忙
  timeout,            
  cancelled,
  io_error,           // socket, fd, driver error 
  protocol_error,     // 非法帧格式
  device_error,       // 设备明确错误
};

```

## 电机型号范围

当前库只保留 ENCOS EC 系列伺服电机型号表，公共 API 入口为 `#include <fleet/fleet.h>`。
旧的 IDM/A1 型号配置（如 A64/A43/A34，或 idm64/idm43/idm34 命名）已删除。

## 性能测试

单 UDP 携带 32 CAN 帧，跑满 400Hz，带宽大概 1.46Mbps。也就是说 30 左右个电机，以 1.5Mbps 带宽进行控制，几乎没有余量。


背压性能测试：瞬间灌入 24 个 128B UDP 报文，最大带宽控制为 102.4kbps
- 实际带宽误差在 80%～115%
- 平均包间隔 8～12 ms
- 间隔 p99 不超过 50 ms
- 瞬时最大排队延迟在 120～500 ms

```bash
# single catch2 benchmark 
./build/debug-x64-clang/fleet_cpp_test.exe "[benchmark]"

# with perf 
sudo perf record -g -F100 -- ./build/debug-x64-clang/fleet_cpp_test.exe "[benchmark]"

sudo perf script --demangle | llvm-cxxfilt-20 > out.txt
```

说明：perf --demangle 无法识别 C++ concept ，建议用 llvm-cxxfilt-20 来反编译函数名。另外，建议控制 perf 的采样时间。

## 交叉编译

#### aarch64 

```bash
# sysroot 
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

cmake --preset release-aarch64
cmake --build --preset release-aarch64
```

#### x64 (gcc11)

```bash
cmake --preset release-x64
cmake --build --preset release-x64
```

#### x64 (msvc)

```bash
cmake --preset debug-x64-msvc
cmake --build --preset debug-x64-msvc
```

#### 其他编译宏

* `FLEET_BUILD_STATIC`
* `FLEET_BUILD_TESTS`
