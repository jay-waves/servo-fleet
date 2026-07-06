/*
    Copyright PNDBotics 2026
*/
#pragma once

#include "device_motor.h"

#include <memory>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

namespace fleet {

using namespace motor;

struct fleet_config {
    struct motor_config {
        std::string name;
        std::string channel;
        std::string model;
        can_port_t can_port = 0;
        can_id_t can_id = 0;
    };

    struct channel_config {
        std::string name;

        /*
            控制 UDP 发送的最大带宽，用于控制发送速度，防止打爆硬件
        */
        size_t max_bandwidth_bps = 1'000'000;

        /*
            控制 UDP 发送队列可积压的包数量。队列满时返回 fleet_err::busy
        */
        uint32_t send_queue_slots = 1024;

        // TOPOLOGY
        std::string remote_address = "127.0.0.1";
        uint16_t remote_port = 9999;
        uint16_t local_port = 0;
    };

    std::vector<channel_config> channels;
    std::vector<motor_config> motors;
};

/*
    Runtime config commands are handled immediately by the runtime. They are not
    sent to devices and affect subsequent command(...) calls.
*/
struct net_control_command {
    /*
        Configure response waiting for subsequent network request/response commands.
        Motor streaming commands are still sent best-effort and do not wait for responses.
    */
    milliseconds timeout = 100ms;
    uint8_t retries = 0;
};

/*
    Drop pending runtime-owned sends before subsequent commands are submitted.
*/
struct discard_pending_command {};

template <typename T> struct IsMotorCommand : std::false_type {};
template <> struct IsMotorCommand<motor::pvt_command> : std::true_type {};
template <> struct IsMotorCommand<motor::position_command> : std::true_type {};
template <> struct IsMotorCommand<motor::velocity_command> : std::true_type {};
template <> struct IsMotorCommand<motor::torque_command> : std::true_type {};
template <> struct IsMotorCommand<motor::current_command> : std::true_type {};
template <> struct IsMotorCommand<motor::brake_command> : std::true_type {};
template <> struct IsMotorCommand<motor::config_set_command> : std::true_type {};
template <> struct IsMotorCommand<motor::config_query_command> : std::true_type {};
template <typename T> struct IsRuntimeConfigCommand : std::false_type {};
template <> struct IsRuntimeConfigCommand<net_control_command> : std::true_type {};
template <> struct IsRuntimeConfigCommand<discard_pending_command> : std::true_type {};
template <typename T> concept MotorCommand = IsMotorCommand<std::remove_cvref_t<T>>::value;
template <typename T> concept RuntimeConfigCommand = IsRuntimeConfigCommand<std::remove_cvref_t<T>>::value;

template <typename T>
concept Command = MotorCommand<T> || RuntimeConfigCommand<T>;



// fleet runtime
std::error_code init(fleet_config options);
void shutdown() noexcept;
bool ok() noexcept;
template <IsDevice T> std::vector<std::shared_ptr<T>> lookup(); // lookup devices
																//
																//

/*
    TODO: 从头到尾审查并发发令的线程安全性

    批处理采用 best-effort 语义，不保证先后执行顺序。发生错误时，部分命令可能已经产生
    发送或入队等副作用，不会回退。返回的 error_code 仅表示批处理中发生的某个错误，
    不对应特定命令，也不表示设备是否执行成功。设备状态仍通过 device_interface 接口访问

    支持对某种命令 T 的批处理，包括底层高度优化的 IO 分发和网络包组合。
*/
template <Command T> std::error_code command(const T &cmd);

template <Command T> std::error_code command(std::span<const T> cmds);

} // namespace fleet
