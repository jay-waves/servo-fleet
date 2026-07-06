#include "codec_can.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>

namespace fleet {

using namespace motor;

namespace {

constexpr uint8_t ok_status = 1;
constexpr float pi = 3.14159265358979323846f;

void init_frame(can_frame &frame, uint32_t id, uint8_t port, uint8_t len) {
    frame = can_frame{}; // 清零
    frame.port = port;
    frame.id = id;
    frame.len = len;
}

inline constexpr auto i16_x100_to_f32 = [](uint32_t raw) noexcept -> float {
    const auto value = static_cast<int16_t>(raw & 0xFFFFu);
    return static_cast<float>(value) / 100.0f;
};

inline constexpr auto u8_temp_to_celsius = [](uint32_t raw) noexcept -> float {
    const auto value = static_cast<uint8_t>(raw & 0xFFu);
    return (static_cast<float>(value) - 50.0f) / 2.0f;
};

float decode_symmetric(uint32_t raw, float range, float max_raw) noexcept {
    return (static_cast<float>(raw) / max_raw) * (2.0f * range) - range;
}

float decode_linear(uint32_t raw, range_t<float> range, float max_raw) noexcept {
    return range.min + (static_cast<float>(raw) / max_raw) * (range.max - range.min);
}

err_code read_u16_be(float& out, bytes_view bytes, size_t offset) {
    if (offset + 2 > bytes.size()) {
        return fleet_err::protocol_error;
    }
    out = static_cast<float>((uint16_t(bytes[offset]) << 8U) | uint16_t(bytes[offset + 1]));
    return {};
}

err_code read_u16_be(uint16_t& out, bytes_view bytes, size_t offset) {
    if (offset + 2 > bytes.size()) {
        return fleet_err::protocol_error;
    }
    out = static_cast<uint16_t>((uint16_t(bytes[offset]) << 8U) | uint16_t(bytes[offset + 1]));
    return {};
}


err_code read_i16_be(int16_t& out, bytes_view bytes, size_t offset) {
    if (offset + 2 > bytes.size()) {
        return fleet_err::protocol_error;
    }
    out = static_cast<int16_t>((uint16_t(bytes[offset]) << 8U) | uint16_t(bytes[offset + 1]));
    return {};
}

err_code read_f32_be(float& out, bytes_view bytes, size_t offset) {
    if (offset + 4 > bytes.size()) {
        return fleet_err::protocol_error;
    }

    const auto raw = (static_cast<uint32_t>(bytes[offset]) << 24U) |
                     (static_cast<uint32_t>(bytes[offset + 1]) << 16U) |
                     (static_cast<uint32_t>(bytes[offset + 2]) << 8U) |
                     static_cast<uint32_t>(bytes[offset + 3]);
    out = bits_to_f32(raw);
    return {};
}

err_code checked_u16_scaled(uint16_t& out, float value, float scale, uint16_t maxx = UINT16_MAX) {
    const auto scaled = static_cast<double>(value) * static_cast<double>(scale);
    if (!std::isfinite(scaled)) {
        return fleet_err::invalid_argument;
    }

    // Float inputs near decimal protocol limits may be rejected due to representation error.
    const auto rounded = std::round(scaled);
    if (rounded < 0.0 || rounded > static_cast<double>(maxx)) {
        return fleet_err::invalid_argument;
    }
    out = static_cast<uint16_t>(rounded);
    return {};
}

err_code checked_i16_scaled(int16_t& out, float value, float scale) {
    const auto scaled = static_cast<double>(value) * static_cast<double>(scale);
    if (!std::isfinite(scaled)) {
        return fleet_err::invalid_argument;
    }

    // Float inputs near decimal protocol limits may be rejected due to representation error.
    const auto rounded = std::round(scaled);
    if (rounded < static_cast<double>(INT16_MIN) || rounded > static_cast<double>(INT16_MAX)) {
        return fleet_err::invalid_argument;
    }
    out = static_cast<int16_t>(rounded);
    return {};
}

result<query_feedback> decode_query_result(config_query_code code, 
                                            motor::motor_err error, bytes_view payload) {
    query_feedback result;
    result.code = code;
    result.error = error;

    using enum config_query_code;
    switch (code) {
    case position: 
    case velocity: 
    case current:
    case power: {
        float v;
        if (auto e = read_f32_be(v, payload, 0))
            return fail(e);
        
        switch (code) {
        case position: 
            result.position_deg = v;
            break;
        case velocity: 
            result.velocity_rpm = v;
            break;
        case current:
            result.current = v;
            break;
        case power: 
            result.power = v;
            break;
        default:
            return fail(fleet_err::invalid_argument);
        }
        return result;
    }
    case acceleration:
    case flux_observer_gain:
    case disturbance_compensation_coeff:
    case feedback_compensation_coeff:
    case damping_coeff:
    case torque_ratio: {
        float v;
        if (auto e = read_u16_be(v, payload, 0)) 
            return fail(e);

        switch (code) {
        case acceleration:
            result.acceleration_radps2  = v;
            break;
        case flux_observer_gain:
            result.flux_observer_gain = v;
            break;
        case disturbance_compensation_coeff:
            result.disturbance_compensation_coeff = v;
            break;
        case feedback_compensation_coeff:
            result.feedback_compensation_coeff = v;
            break;
        case damping_coeff:
            result.damping_coeff = v;
            break;
        case torque_ratio:
            result.torque_ratio = v / 100.0f;
            break;
        default:
            return fail(fleet_err::invalid_argument);
        }
        return result;
    }
    case pvt_kp_range:
    case pvt_kd_range: {
        range_t<float> range;
        if (auto e = read_u16_be(range.min, payload, 0)) return fail(e);
        if (auto e = read_u16_be(range.max, payload, 2)) return fail(e);

        if (code == pvt_kp_range)
            result.pvt_kp_range = range;
        else 
            result.pvt_kd_range = range;
        
        return result;
    }
    case pvt_position_limit_r:
    case pvt_velocity_limit_radps:
    case pvt_torque_limit_nm:
    case pvt_current_limit: {
        int16_t minn, maxx; 
        if (auto e = read_i16_be(minn, payload, 0)) return fail(e);
        if (auto e = read_i16_be(maxx, payload, 2)) return fail(e);
        const float scale =
            (code == pvt_torque_limit_nm || code == pvt_current_limit) ? 10.0f : 100.0f;
        auto range = range_t<float>{.min = minn / scale, .max = maxx / scale};

        switch (code) {
        case pvt_position_limit_r:
            result.pvt_position_limit_r = range;
            break;
        case pvt_velocity_limit_radps:
            result.pvt_velocity_limit_radps = range;
            break;
        case pvt_torque_limit_nm:
            result.pvt_torque_limit_nm = range;
            break;
        case pvt_current_limit: 
            result.pvt_current_limit = range;
            break;
        default:
            return fail(fleet_err::invalid_argument);
        }
        return result;
    }
    case mcu_uuid:
        if (payload.size() < 5)
            return fail(fleet_err::protocol_error);
        result.mcu_uuid_fragment = mcu_uuid_fragment_t{
            .index = payload[0], .bytes = {payload[1], payload[2], payload[3], payload[4]}};
        return result;
    case version:
        if (payload.size() < 6)
            return fail(fleet_err::protocol_error);
        result.version = version_info_t{.hw_major = payload[0],
                                      .hw_minor = payload[1],
                                      .hw_patch = payload[2],
                                      .fw_major = payload[3],
                                      .fw_minor = payload[4],
                                      .fw_patch = payload[5]};
        return result;
    case can_timeout: {
        uint16_t v;
        if (auto e = read_u16_be(v, payload, 0)) 
            return fail(e);

        result.can_timeout = milliseconds{v};
        return result;
    }
    case current_loop_pi: {
        pid_gains pid;

        if (auto e = read_u16_be(pid.kp, payload, 0)) return fail(e);
        if (auto e = read_u16_be(pid.ki, payload, 2)) return fail(e);

        pid.kp /= 10000.0f;
        pid.ki /= 10.0f;
        result.current_loop_pi = pid;
        return result;
    }
    case velocity_loop_pi: {
        pid_gains pid;

        if (auto e = read_u16_be(pid.kp, payload, 0)) return fail(e);
        if (auto e = read_u16_be(pid.ki, payload, 2)) return fail(e);

        pid.kp /= 100000.0f;
        pid.ki /= 100000.0f;
        result.velocity_loop_pi = pid;
        return result;
    }
    case position_loop_pd: {
        pid_gains pid;

        if (auto e = read_u16_be(pid.kp, payload, 0)) return fail(e);
        if (auto e = read_u16_be(pid.kd, payload, 2)) return fail(e);

        pid.kp /= 100000.0f;
        pid.kd /= 100000.0f;
        result.position_loop_pd = pid;
        return result;
    }
    case brake_status:
        if (payload.empty())
            return fail(fleet_err::protocol_error);
        result.brake_status = payload[0] != 0;
        return result;
    }

    return fail(fleet_err::unsupported);
}

err_code make_config_value(can_frame &frame, can_addr_t address, 
                            config_set_code code, uint8_t len, uint16_t value) {
    init_frame(frame, address.can_id, address.can_port, len);
    namespace f = can_field::config;

    if(auto e = write<f::Mode>(frame, enum_u8(command_code::motor_config))) return e;
    // TRY(write<f::Reply>(frame, enum_u8(reply)));
    if(auto e = write<f::Key>(frame, enum_u8(code))) return e;
    if(auto e = write<f::Value>(frame, value)) return e;
    return {};
}

err_code make_config_range(can_frame &frame, can_addr_t address, 
                            config_set_code code, uint16_t min, uint16_t max) {
    init_frame(frame, address.can_id, address.can_port, 6);
    namespace f = can_field::config;

    if(auto e = write<f::Mode>(frame, enum_u8(command_code::motor_config))) return e;
    if(auto e = write<f::Key>(frame, enum_u8(code))) return e;
    if(auto e = write<f::Min>(frame, min)) return e;
    if(auto e = write<f::Max>(frame, max)) return e;
    return {};
}

result<uint16_t> checked_symmetric(double value, double range, uint16_t center, uint16_t max) {
    if (!std::isfinite(value) || !std::isfinite(range) || range <= 0.0) {
        return fail(fleet_err::invalid_argument);
    }

    const auto scaled_value =
        value / range * static_cast<double>(center) + static_cast<double>(center);
    if (scaled_value < static_cast<double>(std::numeric_limits<long long>::min()) ||
        scaled_value > static_cast<double>(std::numeric_limits<long long>::max())) {
        return fail(fleet_err::invalid_argument);
    }
    const auto scaled = llround(scaled_value);
    if (scaled < 0 || scaled > max) {
        return fail(fleet_err::invalid_argument);
    }
    return static_cast<uint16_t>(scaled);
}

result<uint16_t> checked_pvt_gain(double value, double max_value, uint16_t max_raw) {
    if (!std::isfinite(value) || !std::isfinite(max_value) || max_value <= 0.0) {
        return fail(fleet_err::invalid_argument);
    }

    const auto scaled_value = value / max_value * static_cast<double>(max_raw);
    if (scaled_value < static_cast<double>(std::numeric_limits<long long>::min()) ||
        scaled_value > static_cast<double>(std::numeric_limits<long long>::max())) {
        return fail(fleet_err::invalid_argument);
    }
    const auto scaled = llround(scaled_value);
    if (scaled < 0 || scaled > max_raw) {
        return fail(fleet_err::invalid_argument);
    }
    return static_cast<uint16_t>(scaled);
}

motor::motor_err parse_error(uint32_t value) {
    switch (static_cast<uint8_t>(value)) {
    case 0:
        return motor::motor_err::none;
    case 1:
        return motor::motor_err::over_temp;
    case 2:
        return motor::motor_err::over_current;
    case 3:
        return motor::motor_err::over_voltage;
    case 4:
        return motor::motor_err::under_voltage;
    case 5:
        return motor::motor_err::encoder_fail;
    case 6:
        return motor::motor_err::brake_over_voltage;
    case 7:
        return motor::motor_err::driver;
    case 0x1E:
        return motor::motor_err::comm;
    default:
        return motor::motor_err::unknown;
    }
}

result<feedback> parse_broadcast_response(const can_frame &frame) {
    if (frame.id != BROADCAST_CAN_ID) {
        return fail(fleet_err::unsupported);
    }

    if (frame.len == 5 && frame.data[0] == 0xFF && frame.data[1] == 0xFF && frame.data[2] == 0x01) {
        config_ack_feeback response;
        response.code = config_set_code::comm_mode;
        response.accepted = true;
        return feedback{response};
    }

    if (frame.len >= 4 && frame.data[2] == 0x01) {
        config_ack_feeback response;
        response.accepted = frame.data[3] != 0;
        return feedback{response};
    }

    return fail(fleet_err::unsupported);
}

err_code require_frame_len(const can_frame &frame, uint8_t min_len) {
    if (frame.len < min_len) {
        return fleet_err::protocol_error;
    }
    return {};
}

result<feedback> parse_status1_response(const can_frame &frame, motor::motor_err error,
                                        const range_t<float> *current_range) {
    if(auto e = require_frame_len(frame, 8)) 
        return fail(e);

    using namespace can_field::feedback1;
    auto position = read<Position>(frame);
    auto velocity = read<Velocity>(frame);
    auto current = read<Current>(frame);
    auto motor_temp = read<MotorTemp>(frame).transform(u8_temp_to_celsius);
    auto mosfet_temp = read<MosfetTemp>(frame).transform(u8_temp_to_celsius);
    if (!position || !velocity || !current || !motor_temp || !mosfet_temp) {
        return fail(fleet_err::protocol_error);
    }

    return feedback{status_feedback{
        .error = error,
        .position_r = decode_symmetric(*position, 12.5f, 65535.0f),
        .velocity_radps = decode_symmetric(*velocity, 18.0f, 4095.0f),
        .current = current_range != nullptr ? decode_linear(*current, *current_range, 4095.0f)
                                            : static_cast<float>(*current) / 4095.0f,
        .motor_temp_c = *motor_temp,
        .mosfet_temp_c = *mosfet_temp,
    }};
}

result<feedback> parse_position_response(const can_frame &frame, motor::motor_err error) {
    if (auto err = require_frame_len(frame, 8)) {
        return fail(err);
    }

    using namespace can_field::feedback2;

    auto position = read<PositionDeg>(frame).transform(bits_to_f32);
    auto current = read<CurrentX100>(frame).transform(i16_x100_to_f32);
    auto temp = read<Temp>(frame).transform(u8_temp_to_celsius);

    if (!position || !current || !temp) {
        return fail(fleet_err::protocol_error);
    }

    return feedback{position_feedback{
        .error = error,
        .position = *position,
        .current = *current,
        .motor_temp_c = *temp,
    }};
}

result<feedback> parse_velocity_response(const can_frame &frame, motor::motor_err error) {
    if (auto err = require_frame_len(frame, 8)) {
        return fail(err);
    }

    namespace f = can_field::feedback3;

    auto velocity = read<f::Rpm>(frame).transform(bits_to_f32);
    auto current = read<f::CurrentX100>(frame).transform(i16_x100_to_f32);
    auto temp = read<f::Temp>(frame).transform(u8_temp_to_celsius);

    if (!velocity || !current || !temp) {
        return fail(fleet_err::protocol_error);
    }

    velocity_feedback feed{
        .error = error,
        .velocity = *velocity,
        .current = *current,
        .motor_temp_c = *temp,
    };

    return feedback{feed};
}

result<feedback> parse_config_ack_response(const can_frame &frame, motor::motor_err error) {
    if(auto e = require_frame_len(frame, 3)) 
        return fail(e);

    using namespace can_field::feedback_config;

    auto key = read<Key>(frame);
    auto status = read<Status>(frame);
    if (!key || !status) {
        return fail(fleet_err::protocol_error);
    }

    return feedback{config_ack_feeback{
        .error = error,
        .code = static_cast<config_set_code>(*key),
        .accepted = (*status == ok_status),
    }};
}

result<feedback> parse_query_response(const can_frame &frame, motor::motor_err error) {
    if(auto e = require_frame_len(frame, 2)) 
        return fail(e);

    using namespace can_field::feedback_query;

    auto payload = bytes_view{frame.data.data() + 2, static_cast<size_t>(frame.len - 2)};
    return read<Key>(frame)
        .and_then([&](uint32_t raw) {
            return decode_query_result(static_cast<config_query_code>(raw), error, payload);
        })
        .transform([](const query_feedback &value) { return feedback{value}; })
        .or_else([](err_code error) -> result<feedback> { return fail(error); });
}

result<feedback> parse_brake_response(const can_frame &frame, motor::motor_err error) {
    if(auto e = require_frame_len(frame, 2)) 
        return fail(e);

    using namespace can_field::feedback_brake;

    auto engaged = read<Engaged>(frame);
    if (!engaged)
        return fail(fleet_err::protocol_error);

    return feedback{brake_status_feedback{
        .error = error,
        .engaged = (*engaged != 0),
    }};
}

} // namespace

namespace can_codec {

err_code make_pvt(can_frame &frame, can_addr_t address, const pvt_command &command) {
    init_frame(frame, address.can_id, address.can_port, 8);
    auto raw = frame.as_bytes_mut();

    namespace f = can_field::pvt;
    auto kp = checked_pvt_gain(command.gains.kp, 500.0, 4095);
    auto kd = checked_pvt_gain(command.gains.kd, 5.0, 511);
    auto position = checked_symmetric(command.position_r, 12.5, 32767, 0xFFFF);
    auto velocity = checked_symmetric(command.velocity_radps, 18.0, 2047, 0x0FFF);
    auto torque = checked_symmetric(command.torque_ff_nm, 30.0, 2047, 0x0FFF);
    if (!kp) return kp.error();
    if (!kd) return kd.error();
    if (!position) return position.error();
    if (!velocity) return velocity.error();
    if (!torque) return torque.error();

    if(auto e = write<f::Mode>(raw, enum_u8(command_code::pvt_control))) return e;
    if(auto e = write<f::Kp>(raw, *kp)) return e;
    if(auto e = write<f::Kd>(raw, *kd)) return e;
    if(auto e = write<f::Position>(raw, *position)) return e;
    if(auto e = write<f::Velocity>(raw, *velocity)) return e;
    if(auto e = write<f::Torque>(raw, *torque)) return e;
    return {};
}

result<can_frame> make_pvt(can_addr_t address, const pvt_command &command) {
    can_frame frame;
    if (auto err = make_pvt(frame, address, command)) return fail(err);
    return frame;
}

err_code make_position(can_frame &frame, can_addr_t address, const position_command &cmd) {
    init_frame(frame, address.can_id, address.can_port, 8);

    namespace f = can_field::position_control;

    uint16_t velocity_limit, current_limit;
    if (auto e = checked_u16_scaled(velocity_limit, cmd.velocity_limit_rpm, 10.0f, 0x7fff)) return e;
    if (auto e = checked_u16_scaled(current_limit, cmd.current_limit, 10.0f, 0x0fff)) return e;

    if(auto e = write<f::Mode>(frame, enum_u8(command_code::position_control))) return e;
    if(auto e = write<f::PositionDeg>(frame, f32_to_bits(cmd.position_deg))) return e;
    if(auto e = write<f::VelocityRpmX10>(frame, velocity_limit)) return e;
    if(auto e = write<f::CurrentAmpX10>(frame, current_limit)) return e;
    if(auto e = write<f::Reply>(frame, enum_u8(cmd.reply))) return e;
    return {};
}

result<can_frame> make_position(can_addr_t address, const position_command &command) {
    can_frame frame;
    if (auto err = make_position(frame, address, command)) return fail(err);
    return frame;
}

err_code make_velocity(can_frame &frame, can_addr_t address, const velocity_command &cmd) {
    init_frame(frame, address.can_id, address.can_port, 7);
    namespace vc = can_field::velocity_control;

    uint16_t current_limit;
    if (auto e = checked_u16_scaled(current_limit, cmd.current_limit, 10.0f)) return e;

    if(auto e = write<vc::Mode>(frame, enum_u8(command_code::velocity_control))) return e;
    if(auto e = write<vc::Reserved>(frame, uint8_t{0})) return e;
    if(auto e = write<vc::Reply>(frame, enum_u8(cmd.reply))) return e;
    if(auto e = write<vc::Rpm>(frame, f32_to_bits(cmd.velocity_rpm))) return e;
    if(auto e = write<vc::CurrentAmpX10>(frame, current_limit)) return e;
    return {};
}

result<can_frame> make_velocity(can_addr_t address, const velocity_command &command) {
    can_frame frame;
    if (auto err = make_velocity(frame, address, command)) return fail(err);
    return frame;
}

static err_code encode_torque_control(can_frame &frame, can_addr_t address, 
        torque_control_mode code, float value, reply_mode reply) {
    init_frame(frame, address.can_id, address.can_port, 3);
    namespace f = can_field::torque_control;

    int16_t raw;
    if (auto e = checked_i16_scaled(raw, value, 100.0)) return e;

    if(auto e = write<f::Mode>(frame, enum_u8(command_code::torque_control))) return e;
    if(auto e = write<f::State>(frame, enum_u8(code))) return e;
    if(auto e = write<f::Reply>(frame, enum_u8(reply))) return e;
    if(auto e = write<f::Value>(frame, static_cast<uint16_t>(raw))) return e;
    return {};
}

err_code make_torque(can_frame &frame, can_addr_t address, const torque_command &cmd) {
    return encode_torque_control(frame, address, torque_control_mode::torque, cmd.torque_nm, cmd.reply);
}

result<can_frame> make_torque(can_addr_t address, const torque_command &command) {
    can_frame frame;
    if (auto err = make_torque(frame, address, command)) return fail(err);
    return frame;
}

err_code make_current(can_frame &frame, can_addr_t address, const current_command &cmd) {
    return encode_torque_control(frame, address, torque_control_mode::current, cmd.current, cmd.reply);
}

result<can_frame> make_current(can_addr_t address, const current_command &command) {
    can_frame frame;
    if (auto err = make_current(frame, address, command)) return fail(err);
    return frame;
}

err_code make_brake(can_frame &frame, can_addr_t address, const brake_command &cmd) {
    init_frame(frame, address.can_id, address.can_port, 2);
    auto raw = frame.as_bytes_mut();

    return std::visit(
        [&](const auto &ctrl) -> err_code {
            using T = std::decay_t<decltype(ctrl)>;
            if constexpr (std::is_same_v<T, damping_brake>) {
                return encode_torque_control(frame, address,
                                             torque_control_mode::variable_damping_brake, 0,
                                             cmd.reply);

            } else if constexpr (std::is_same_v<T, energy_brake>) {
                return encode_torque_control(frame, address, torque_control_mode::regenerative_brake,
                                             ctrl.current_threshold, cmd.reply);

            } else if constexpr (std::is_same_v<T, regen_brake>) {
                return encode_torque_control(frame, address, torque_control_mode::regenerative_brake,
                                             ctrl.current_threshold, cmd.reply);

            } else if constexpr (std::is_same_v<T, engage_brake>) {
                namespace f = can_field::brake_control;
                if(auto e = write<f::Mode>(raw, enum_u8(command_code::brake_control))) return e;
                if(auto e = write<f::Reserved>(raw, uint8_t{0})) return e;
                if(auto e = write<f::Engaged>(raw, static_cast<uint8_t>(ctrl.open ? 1 : 0))) return e;
                return {};
            }
        },
        cmd.control);
}

result<can_frame> make_brake(can_addr_t address, const brake_command &command) {
    can_frame frame;
    if (auto err = make_brake(frame, address, command)) return fail(err);
    return frame;
}

static err_code encode_config_payload(can_frame &frame, can_addr_t address, 
        config_set_code code, uint8_t mode) {
    if (code != config_set_code::comm_mode) {
        return fleet_err::invalid_argument;
    }

    init_frame(frame, address.can_id, address.can_port, 3);
    namespace f = can_field::config;
    if(auto e = write<f::Mode>(frame, enum_u8(command_code::motor_config))) return e;
    // TRY(write<f::Reply>(frame, enum_u8(reply))); fill this field if ack
    if(auto e = write<f::Key>(frame, enum_u8(code))) return e;
    frame.data[2] = mode;
    return {};
}

static err_code encode_config_payload(can_frame &frame, can_addr_t address, 
        config_set_code code, double ratio) {
    if (code != config_set_code::torque_ratio) {
        return fleet_err::invalid_argument;
    }
    uint16_t raw;
    if (auto e = checked_u16_scaled(raw, static_cast<float>(ratio), 100.0f)) return e;
    return make_config_value(frame, address, code, 4, raw);
}

static err_code encode_config_payload(can_frame &frame, can_addr_t address, 
        config_set_code code, float acceleration) {
    if (code != config_set_code::accel) {
        return fleet_err::invalid_argument;
    }
    uint16_t raw;
    if (auto e = checked_u16_scaled(raw, acceleration, 100.0f)) return e;
    return make_config_value(frame, address, code, 4, raw);
}

static err_code encode_config_payload(can_frame &frame, can_addr_t address, 
        config_set_code code, range_t<uint16_t> range) {
    switch (code) {
    case config_set_code::pvt_kp:
    case config_set_code::pvt_kd:
        return make_config_range(frame, address, code, range.min, range.max);
    default:
        return fleet_err::invalid_argument;
    }
}

static err_code encode_config_payload(can_frame &frame, can_addr_t address, 
        config_set_code code, range_t<float> range) {
    switch (code) {
    case config_set_code::pvt_kp:
    case config_set_code::pvt_kd: {
        uint16_t minn, maxx;
        if (auto e = checked_u16_scaled(minn, range.min, 1.0f)) return e;
        if (auto e = checked_u16_scaled(maxx, range.max, 1.0f)) return e;
        return make_config_range(frame, address, code, minn, maxx);
    }
    case config_set_code::pvt_position_limit_r:
    case config_set_code::pvt_velocity_limit_radps: {
        int16_t minn, maxx;
        if (auto e = checked_i16_scaled(minn, range.min, 100.0f)) return e;
        if (auto e = checked_i16_scaled(maxx, range.max, 100.0f)) return e;
        return make_config_range(frame, address, code, static_cast<uint16_t>(minn),
                                 static_cast<uint16_t>(maxx));
    }
    case config_set_code::pvt_torque_limit_nm:
    case config_set_code::pvt_current_limit: {
        int16_t minn, maxx;
        if (auto e = checked_i16_scaled(minn, range.min, 10.0f)) return e;
        if (auto e = checked_i16_scaled(maxx, range.max, 10.0f)) return e;
        return make_config_range(frame, address, code, static_cast<uint16_t>(minn),
                                 static_cast<uint16_t>(maxx)); // 补码
    }
    default:
        return fleet_err::invalid_argument;
    }
}

static err_code encode_config_payload(can_frame &frame, can_addr_t address, 
            config_set_code code, milliseconds timeout) {
    if (code != config_set_code::can_timeout) {
        return fleet_err::invalid_argument;
    }
    if (timeout.count() < 0 || timeout.count() > std::numeric_limits<uint16_t>::max()) {
        return fleet_err::invalid_argument;
    }
    return make_config_value(frame, address, code, 4, static_cast<uint16_t>(timeout.count()));
}

static err_code encode_config_payload(can_frame &frame, can_addr_t address, 
        config_set_code code, pid_gains gains) {
    switch (code) {
    case config_set_code::current_loop_pi: {
        uint16_t kp, ki;
        if (auto e = checked_u16_scaled(kp, gains.kp, 10000.0f)) return e;
        if (auto e = checked_u16_scaled(ki, gains.ki, 10.0f)) return e;
        return make_config_range(frame, address, code, kp, ki);
    }
    case config_set_code::velocity_loop_pi: {
        uint16_t kp, ki;
        if (auto e = checked_u16_scaled(kp, gains.kp, 100000.0f)) return e;
        if (auto e = checked_u16_scaled(ki, gains.ki, 100000.0f)) return e;
        return make_config_range(frame, address, code, kp, ki);
    }
    case config_set_code::position_loop_pd: {
        uint16_t kp, kd;
        if (auto e = checked_u16_scaled(kp, gains.kp, 100000.0f)) return e;
        if (auto e = checked_u16_scaled(kd, gains.kd, 100000.0f)) return e;
        return make_config_range(frame, address, code, kp, kd);
    }
    default:
        return fleet_err::invalid_argument;
    }
}

err_code make_config(can_frame &frame, can_addr_t address, const config_set_command &cmd) {
    // if (cmd.reply != ReplyMode::none && cmd.reply != ReplyMode::status1)
    //   return fail(PndErr::invalid_argument);
    // ack??

    return std::visit(
        [&](const auto &value) -> err_code {
            return encode_config_payload(frame, address, cmd.code, value);
        },
        cmd.value);
}

result<can_frame> make_config(can_addr_t address, const config_set_command &command) {
    can_frame frame;
    if (auto err = make_config(frame, address, command)) return fail(err);
    return frame;
}

err_code make_config_query(can_frame &frame, can_addr_t address, const config_query_command &command) {
    const auto code = command.code;
    init_frame(frame, address.can_id, address.can_port, 2);
    namespace f = can_field::config_query;

    if(auto e = write<f::Mode>(frame, enum_u8(command_code::motor_config_query))) return e;
    if(auto e = write<f::Key>(frame, static_cast<uint8_t>(code))) return e;
    return {};
}

result<can_frame> make_config_query(can_addr_t address, const config_query_command &command) {
    can_frame frame;
    if (auto err = make_config_query(frame, address, command)) return fail(err);
    return frame;
}

result<feedback> parse_feedback(const can_frame &frame, const range_t<float> *status_current_range) {
    if (frame.len == 0 || frame.len > MAX_CAN_DATA_LEN)
        return fail(fleet_err::protocol_error);

    if (frame.id == BROADCAST_CAN_ID) {
        auto response = parse_broadcast_response(frame);
        if (response) {
            return response;
        }
    }

    auto type = read<can_field::feedback1::Type>(frame);
    auto error = read<can_field::feedback1::Error>(frame);
    if (!type || !error) {
        return fail(fleet_err::protocol_error);
    }

    switch (*type) {
    case 1:
        return parse_status1_response(frame, parse_error(*error), status_current_range);
    case 2:
        return parse_position_response(frame, parse_error(*error));
    case 3:
        return parse_velocity_response(frame, parse_error(*error));
    case 4:
        return parse_config_ack_response(frame, parse_error(*error));
    case 5:
        return parse_query_response(frame, parse_error(*error));
    case 6:
        return parse_brake_response(frame, parse_error(*error));
    default:
        return fail(fleet_err::unsupported);
    }
}

} // namespace can_codec

} // namespace fleet
