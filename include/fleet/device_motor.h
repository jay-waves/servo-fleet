/*
    Copyright PNDBotics 2026
*/
#pragma once

#include "device.h"

#include <array>
#include <functional>
#include <variant>

/*
  自研 H7/ENCOS 电机控制协议
*/

namespace fleet::motor {

using can_port_t = uint8_t;
using can_id_t = uint32_t;
using device_id_t = uint32_t;

struct can_addr_t {
    can_port_t can_port = 0;
    can_id_t can_id = 0;

    friend bool operator==(const can_addr_t &, const can_addr_t &) = default;
};

struct can_addr_hash {
    size_t operator()(const can_addr_t &address) const noexcept {
        size_t value = std::hash<can_port_t>{}(address.can_port);
        value ^= std::hash<can_id_t>{}(address.can_id) + 0x9e3779b9 + (value << 6U) + (value >> 2U);
        return value;
    }
};

enum class reply_mode : uint8_t {
    none = 0,
    status1 = 1,  // return MotorErr, Position, Velocity, Current, Temp, MosfetTemp
    position = 2, // return MotorErr, Position, Current, Temp
    velocity = 3, // return MotorErr, Velocity, Current, Temp
};

enum class motor_err : uint8_t {
    none = 0,
    over_temp = 1,
    over_current = 2,
    over_voltage = 3,
    under_voltage = 4,
    encoder_fail = 5,
    brake_over_voltage = 6,
    driver = 7,
    comm = 0x1E,
    unknown = 0x1F,
};

template <typename T> struct range_t {
    T min;
    T max;
};

struct pid_gains {
    float kp = 0.0;
    float kd = 0.0;
    float ki = 0.0;
};

// only return feedback1
struct pvt_command {
	device_id_t device_id;
    pid_gains gains = {.ki = 0.0};
    float position_r;
    float velocity_radps;
    float torque_ff_nm;
};

struct position_command {
	device_id_t device_id;
    float position_deg;
    float velocity_limit_rpm;
    float current_limit;
    reply_mode reply = reply_mode::status1;
};

struct velocity_command {
	device_id_t device_id;
    float velocity_rpm;
    float current_limit;
    reply_mode reply = reply_mode::status1;
};

struct torque_command {
	device_id_t device_id;
    float torque_nm;
    reply_mode reply = reply_mode::status1;
};

struct current_command {
	device_id_t device_id;
    float current;
    reply_mode reply = reply_mode::status1;
};

struct damping_brake {};
struct energy_brake {
    float current_threshold;
};
struct regen_brake {
    float current_threshold;
};
/*
  - true, 打开抱闸，电磁抱闸打开，电机正常控制
  - false，关闭抱闸，电磁抱闸吸合，电机无法转动
*/
// TODO: 默认走 0x4, 而非 0x3
struct engage_brake {
    bool open = false;
};

using brake_control = std::variant<damping_brake, energy_brake, regen_brake, engage_brake>;

struct brake_command {
	device_id_t device_id;
    brake_control control = engage_brake{};
    reply_mode reply;
};

enum class config_set_code : uint8_t {
    accel = 0x01,
    comm_mode = 0x02,
    torque_ratio = 0x04,
    pvt_kp = 0x05,
    pvt_kd = 0x06,
    pvt_position_limit_r = 0x07,
    pvt_velocity_limit_radps = 0x08,
    pvt_torque_limit_nm = 0x09,
    pvt_current_limit = 0x0A,
    can_timeout = 0x0B,
    current_loop_pi = 0x0C,
    velocity_loop_pi = 0x0D,
    position_loop_pd = 0x0E,
};

enum class config_query_code : uint8_t {
    position = 1,
    velocity = 2,
    current = 3,
    power = 4,
    acceleration = 5,
    flux_observer_gain = 6,
    disturbance_compensation_coeff = 7,
    feedback_compensation_coeff = 8,
    damping_coeff = 9,
    torque_ratio = 22,
    pvt_kp_range = 23,
    pvt_kd_range = 24,
    pvt_position_limit_r = 25,
    pvt_velocity_limit_radps = 26,
    pvt_torque_limit_nm = 27,
    pvt_current_limit = 28,
    mcu_uuid = 29,
    version = 30,
    can_timeout = 31,
    current_loop_pi = 32,
    velocity_loop_pi = 33,
    position_loop_pd = 34,
    brake_status = 37,
};

// TODO：改成 UNION
using config_set_value = std::variant<float, uint8_t, double, range_t<uint16_t>, range_t<float>,
                                      milliseconds, pid_gains>;

struct config_set_command {
	device_id_t device_id;
    config_set_code code = config_set_code::accel;
    config_set_value value = 0.0f;
    // bool ack = false; @jay-waves: 开启后，配置失败会返回一个 0，暂时不支持该参数。
    // 如果需要查看配置是否成功，请通过 Sample<T> 来自行判断
};

struct config_query_command {
	device_id_t device_id;
    config_query_code code = config_query_code::position;
};

struct version_info_t {
    uint8_t hw_major = 0;
    uint8_t hw_minor = 0;
    uint8_t hw_patch = 0;
    uint8_t fw_major = 0;
    uint8_t fw_minor = 0;
    uint8_t fw_patch = 0;
};

struct mcu_uuid_fragment_t {
    uint8_t index = 0;
    std::array<uint8_t, 4> bytes{};
};

struct mcu_uuid_t {
    std::array<uint8_t, 12> bytes{};
    uint8_t received_mask = 0;

    bool complete() const noexcept { return (received_mask & 0x07U) == 0x07U; }
};

/*
    Status1 中供 PVT 控制器使用的字段必须作为同一份快照读取，避免控制线程在
    多次 getter 调用之间混入新的反馈帧。seq/time 一致表示字段来自同一报文。
*/
struct pvt_feedback_snapshot {
    sample_t<motor_err> error;
    sample_t<float> position_deg;
    sample_t<float> velocity_rpm;
    sample_t<float> current;

    bool received() const noexcept {
        return error.received() && position_deg.received() && velocity_rpm.received() &&
               current.received();
    }

    bool coherent() const noexcept {
        return received() && error.seq == position_deg.seq && error.seq == velocity_rpm.seq &&
               error.seq == current.seq && error.time == position_deg.time &&
               error.time == velocity_rpm.time && error.time == current.time;
    }

    bool fresh(time_point now, duration max_age) const noexcept {
        return coherent() && error.fresh(now, max_age);
    }
};

class motor_interface : public fleet::device_interface {
  public:
    ~motor_interface() override = default;

	    virtual sample_t<motor_err> motor_error() const = 0;
	    virtual pvt_feedback_snapshot pvt_feedback() const = 0;
	    // Position queried from the motor is the accumulated angle since power-on, in degrees.
	    virtual sample_t<float> position_deg() const = 0;
    virtual sample_t<float> velocity_rpm() const = 0;
    virtual sample_t<float> current() const = 0;
    virtual sample_t<float> power() const = 0;
    virtual sample_t<float> acceleration_radps2() const = 0;

    virtual sample_t<float> motor_temp_c() const = 0;
    virtual sample_t<float> mosfet_temp_c() const = 0;

    virtual sample_t<float> flux_observer_gain() const = 0;
    virtual sample_t<float> disturbance_compensation_coeff() const = 0;
    virtual sample_t<float> feedback_compensation_coeff() const = 0;
    virtual sample_t<float> damping_coeff() const = 0;
    virtual sample_t<float> torque_ratio() const = 0;
    virtual sample_t<range_t<float>> pvt_kp_range() const = 0;
    virtual sample_t<range_t<float>> pvt_kd_range() const = 0;
    virtual sample_t<range_t<float>> pvt_position_limit_r() const = 0;
    virtual sample_t<range_t<float>> pvt_velocity_limit_radps() const = 0;
    virtual sample_t<range_t<float>> pvt_torque_limit_nm() const = 0;
    virtual sample_t<range_t<float>> pvt_current_limit() const = 0;
    virtual sample_t<mcu_uuid_t> mcu_uuid() const = 0;
    virtual sample_t<version_info_t> version() const = 0;
    virtual sample_t<milliseconds> can_timeout() const = 0;
    virtual sample_t<pid_gains> current_loop_pi() const = 0;
    virtual sample_t<pid_gains> velocity_loop_pi() const = 0;
    virtual sample_t<pid_gains> position_loop_pd() const = 0;
    virtual sample_t<bool> brake_status() const = 0;

  protected:
    explicit motor_interface(device_id_t id, std::string name)
        : fleet::device_interface(id, std::move(name)) {}
};

} // namespace fleet::motor
