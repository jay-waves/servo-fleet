/**
 * Copyright PNDBotics 2026
 */

#pragma once

#include <array>
#include <variant>

#include "codec_bit.h"
#include "fleet/fleet.h"

/**

  +---------+-------------+-------------+-----+
  | MSG_CNT | CAN MSG 0   | ...         | N   |
  | 1 B     | 14 B        | repeat      |     |
  +---------+-------------+-------------+-----+

  Each CAN message entry:

  ```text
  +------+------------+------+----------+
  | PORT | ID         | LEN  | DATA     |
  | 1 B  | 4 B        | 1 B  | 8 B      |
  +------+------------+------+----------+
        big-endian
  ```

  `ID` uses bit31 as the extended frame flag:

  ```text
  standard frame: ID = can_id, can_id <= 0x7FF
  extended frame: ID = 0x80000000 | can_id, can_id <= 0x1FFFFFFF
  ```

  `LEN` is the active CAN data length, from 0 to 8. `DATA` always occupies 8
  bytes in the UDP packet.

 */

namespace fleet {

using namespace motor;

inline constexpr uint8_t MAX_CAN_DATA_LEN = 8;
inline constexpr uint8_t MAX_FRAMES_PER_PACKET = 32;
inline constexpr uint32_t EXTENDED_ID_FLAG = 0x80000000U;
inline constexpr uint32_t BROADCAST_CAN_ID = 0x7FF;

template <size_t MaxDataLen = MAX_CAN_DATA_LEN> struct basic_can_frame {
    can_port_t port = 0;
    can_id_t id = 0;
    bool extended = false;
    std::array<uint8_t, MaxDataLen> data{};
    uint8_t len = 0;

    [[nodiscard]] bool valid() const noexcept {
        const auto max_id = extended ? 0x1FFFFFFFu : 0x7FFu;
        return id <= max_id && len <= MaxDataLen;
    }
    auto as_bytes_mut() noexcept { return bytes_mut(data).first(len); }
    auto as_bytes_view() const noexcept { return bytes_view(data).first(len); }
};

using can_frame = basic_can_frame<MAX_CAN_DATA_LEN>;
/**
 * @brief CAN FD Frame is not supported right now
 */
using can_fd_frame = basic_can_frame<64>;

template <BitField Field> inline result<uint32_t> read(const can_frame &frame) {
    return read<Field>(bytes_view{frame.data.data(), frame.len});
}

template <BitField Field> inline err_code write(can_frame &frame, uint32_t raw) {
    return write<Field>(bytes_mut{frame.data.data(), frame.len}, raw);
}

namespace can_field {

namespace pvt {
using Mode = Bits<0, 3>;
using Kp = Bits<3, 12>;
using Kd = Bits<15, 9>;
using Position = Bits<24, 16>;
using Velocity = Bits<40, 12>;
using Torque = Bits<52, 12>;
} // namespace pvt

namespace position_control {
using Mode = Bits<0, 3>;
using PositionDeg = U32<3>;
using VelocityRpmX10 = Bits<35, 15>;
using CurrentAmpX10 = Bits<50, 12>;
using Reply = Bits<62, 2>;
} // namespace position_control

namespace velocity_control {
using Mode = Bits<0, 3>;
using Reserved = Bits<3, 3>;
using Reply = Bits<6, 2>;
using Rpm = U32<8>;
using CurrentAmpX10 = U16<40>;
} // namespace velocity_control

namespace torque_control {
using Mode = Bits<0, 3>;
using State = Bits<3, 3>;
using Reply = Bits<6, 2>;
using Value = U16<8>;
} // namespace torque_control

namespace brake_control {
using Mode = Bits<0, 3>;
using Reserved = Bits<3, 5>;
using Engaged = U8<8>;
} // namespace brake_control

namespace config_query {
using Mode = Bits<0, 3>;
using Reserved = Bits<3, 5>;
using Key = U8<8>;
} // namespace config_query

namespace config {
using Mode = Bits<0, 3>;
using Reserved = Bits<3, 3>;
using Reply = Bits<6, 2>; // 部分指令
using Key = U8<8>;
using Value = U16<16>;
using Min = U16<16>;
using Max = U16<32>;
} // namespace config

namespace feedback1 {
using Type = Bits<0, 3>;
using Error = Bits<3, 5>;
using Position = U16<8>;
using Velocity = Bits<24, 12>;
using Current = Bits<36, 12>;
using MotorTemp = U8<48>;
using MosfetTemp = U8<56>;
} // namespace feedback1

namespace feedback2 {
using Type = Bits<0, 3>;
using Error = Bits<3, 5>;
using PositionDeg = U32<8>;
using CurrentX100 = U16<40>;
using Temp = U8<56>;
} // namespace feedback2

namespace feedback3 {
using Type = Bits<0, 3>;
using Error = Bits<3, 5>;
using Rpm = U32<8>;
using CurrentX100 = U16<40>;
using Temp = U8<56>;
} // namespace feedback3

namespace feedback_config {
using Type = Bits<0, 3>;
using Error = Bits<3, 5>;
using Key = U8<8>;
using Status = U8<16>;
} // namespace feedback_config

// @jay-waves: 文档里每个报文开头的 0xFF 0xFE 究竟是什么东西???
namespace feedback_query {
using Type = Bits<0, 3>;
using Error = Bits<3, 5>;
using Key = U8<8>;
} // namespace feedback_query

namespace feedback_brake {
using Type = Bits<0, 3>;
using Error = Bits<3, 5>;
using Engaged = U8<8>;
} // namespace feedback_brake

} // namespace can_field

enum class command_code : uint8_t {
    pvt_control = 0x00,
    position_control = 0x01,
    velocity_control = 0x02,
    torque_control = 0x03,
    brake_control = 0x04,
    motor_config = 0x06,
    motor_config_query = 0x07,
};

// 9.1.4 电流、力矩控制、刹车控制指令
enum class torque_control_mode : uint8_t {
    current = 0,                // CurrentCommand
    torque = 1,                 // TorqueCommand
    variable_damping_brake = 2, // BrakeCommand
    rheostatic_brake = 3,       // BrakeCommand
    regenerative_brake = 4,     // BrakeCommand
    electromagnetic_brake = 5,  // 默认不使用, 用 brake_control 协议
};

struct status_feedback {
    motor_err error = motor_err::none;
    float position_r;
    float velocity_radps;
    float current;
    float motor_temp_c;
    float mosfet_temp_c;
};

struct position_feedback {
    motor_err error = motor_err::none;
    float position;
    float current;
    float motor_temp_c;
};

struct velocity_feedback {
    motor_err error = motor_err::none;
    float velocity;
    float current;
    float motor_temp_c;
};

struct query_feedback {
    motor_err error = motor_err::none;
    config_query_code code = config_query_code::position;

    float position_deg;
    float velocity_rpm;
    float current;
    float power;
    float acceleration_radps2;

    float flux_observer_gain;
    float disturbance_compensation_coeff;
    float feedback_compensation_coeff;
    float damping_coeff;
    float torque_ratio; // rate 100

    range_t<float> pvt_kp_range;
    range_t<float> pvt_kd_range;
    range_t<float> pvt_position_limit_r;
    range_t<float> pvt_velocity_limit_radps;
    range_t<float> pvt_torque_limit_nm;
    range_t<float> pvt_current_limit;

    mcu_uuid_fragment_t mcu_uuid_fragment;
    version_info_t version;
    milliseconds can_timeout;
    pid_gains current_loop_pi;
    pid_gains velocity_loop_pi;
    pid_gains position_loop_pd;

    bool brake_status;
};

struct config_ack_feeback {
    motor_err error = motor_err::none;
    config_set_code code = config_set_code::accel;
    bool accepted = false;
};

struct brake_status_feedback {
    motor_err error = motor_err::none;
    bool engaged = false;
};

using feedback = std::variant<status_feedback, position_feedback, velocity_feedback,
                              config_ack_feeback, query_feedback, brake_status_feedback>;

namespace can_codec {

err_code make_pvt(can_frame &frame, can_addr_t address, const pvt_command &command);
err_code make_position(can_frame &frame, can_addr_t address, const position_command &command);
err_code make_velocity(can_frame &frame, can_addr_t address, const velocity_command &command);
err_code make_torque(can_frame &frame, can_addr_t address, const torque_command &command);
err_code make_current(can_frame &frame, can_addr_t address, const current_command &command);
err_code make_brake(can_frame &frame, can_addr_t address, const brake_command &command);
err_code make_config(can_frame &frame, can_addr_t address, const config_set_command &command);
err_code make_config_query(can_frame &frame, can_addr_t address, const config_query_command &command);

result<can_frame> make_pvt(can_addr_t address, const pvt_command &command);
result<can_frame> make_position(can_addr_t address, const position_command &command);
result<can_frame> make_velocity(can_addr_t address, const velocity_command &command);
result<can_frame> make_torque(can_addr_t address, const torque_command &command);
result<can_frame> make_current(can_addr_t address, const current_command &command);
result<can_frame> make_brake(can_addr_t address, const brake_command &command);
result<can_frame> make_config(can_addr_t address, const config_set_command &command);
result<can_frame> make_config_query(can_addr_t address, const config_query_command &command);

template <Command T> err_code encode(can_frame &frame, can_addr_t address, const T &command) {
    if constexpr (std::same_as<T, pvt_command>) {
        return make_pvt(frame, address, command);
    } else if constexpr (std::same_as<T, position_command>) {
        return make_position(frame, address, command);
    } else if constexpr (std::same_as<T, velocity_command>) {
        return make_velocity(frame, address, command);
    } else if constexpr (std::same_as<T, torque_command>) {
        return make_torque(frame, address, command);
    } else if constexpr (std::same_as<T, current_command>) {
        return make_current(frame, address, command);
    } else if constexpr (std::same_as<T, brake_command>) {
        return make_brake(frame, address, command);
    } else if constexpr (std::same_as<T, config_set_command>) {
        return make_config(frame, address, command);
    } else if constexpr (std::same_as<T, config_query_command>) {
        return make_config_query(frame, address, command);
    } else {
        return fleet_err::unsupported;
    }
}

template <Command T> result<can_frame> encode(can_addr_t address, const T &command) {
    can_frame frame;
    if (auto err = encode(frame, address, command)) {
        return fail(err);
    }
    return frame;
}

result<feedback> parse_feedback(const can_frame &frame,
                                const range_t<float> *status_current_range = nullptr);

} // namespace can_codec

} // namespace fleet
