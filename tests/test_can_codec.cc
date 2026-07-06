#include <doctest/doctest.h>

#include "codec_can.h"
#include "log.h"

#include <array>
#include <bit>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>

namespace {

namespace test_field {
using Mode = fleet::Bits<0, 3>;
using Kp = fleet::Bits<3, 12>;
using Kd = fleet::Bits<15, 9>;
using Pos = fleet::Bits<24, 16>;
using Spd = fleet::Bits<40, 12>;
using Torque = fleet::Bits<52, 12>;

using Byte0 = fleet::U8<0>;
using Word0 = fleet::U16<0>;
using Float0 = fleet::U32<0>;

using OutOfRange = fleet::Bits<60, 8>;
} // namespace test_field

using namespace fleet;

enum class TestCommand : std::uint8_t {
    idle = 0,
    torque_control = 1,
    position_control = 2,
    velocity_control = 3,
};

template <class Err> void require_ok(const Err &err) {
    // 你的 Error 用法是：if (err) return unexpected(err);
    // 所以 bool(err) == true 表示有错误。
    REQUIRE_FALSE(static_cast<bool>(err));
}

template <class Result> void require_invalid_argument(const Result &result) {
    REQUIRE_FALSE(result);
    REQUIRE(result.error() == make_error_code(fleet_err::invalid_argument));
}

std::string frame_data_hex(const can_frame &frame) {
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (uint8_t i = 0; i < frame.len; ++i) {
        if (i != 0) {
            out << ' ';
        }
        out << "0x" << std::setw(2) << static_cast<unsigned>(frame.data[i]);
    }
    return out.str();
}

template <size_t N> void require_frame_bytes(const can_frame &frame, const std::array<uint8_t, N> &expected) {
	FLEET_DEBUG_KV(
			KV("msg", "CAN Frame"),
			KV("id", frame.id),
			KV("port", frame.port),
			KV("len", frame.len),
			KV("data", frame_data_hex(frame))
			);
    REQUIRE(frame.len == N);
    for (size_t i = 0; i < N; ++i) {
		FLEET_DEBUG_KV(
				KV("msg", "CAN"),
				KV("byte", i),
				KV("actual", fmt::format("0x{:02X}", frame.data[i])),
				KV("expected", fmt::format("0x{:02X}", expected[i]))
				);
        REQUIRE(frame.data[i] == expected[i]);
    }
}

pvt_command make_pvt_command(float kp, float kd, float position, float velocity, float torque) {
    pvt_command command;
    command.gains = pid_gains{.kp = kp, .kd = kd, .ki = 0.0f};
    command.position_r = position;
    command.velocity_radps = velocity;
    command.torque_ff_nm = torque;
    return command;
}

void require_pvt_fields(const can_frame &frame, uint32_t kp, uint32_t kd, uint32_t position, uint32_t velocity,
                        uint32_t torque) {
    REQUIRE(frame.len == 8);
    REQUIRE(*read<can_field::pvt::Mode>(frame) == enum_u8(command_code::pvt_control));
    REQUIRE(*read<can_field::pvt::Kp>(frame) == kp);
    REQUIRE(*read<can_field::pvt::Kd>(frame) == kd);
    REQUIRE(*read<can_field::pvt::Position>(frame) == position);
    REQUIRE(*read<can_field::pvt::Velocity>(frame) == velocity);
    REQUIRE(*read<can_field::pvt::Torque>(frame) == torque);
}

can_frame make_status_feedback_frame(uint16_t position, uint16_t velocity, uint16_t current) {
    can_frame frame{};
    frame.id = 0x123;
    frame.len = 8;
    REQUIRE_FALSE(write<can_field::feedback1::Type>(frame, uint8_t{1}));
    REQUIRE_FALSE(write<can_field::feedback1::Error>(frame, uint8_t{0}));
    REQUIRE_FALSE(write<can_field::feedback1::Position>(frame, position));
    REQUIRE_FALSE(write<can_field::feedback1::Velocity>(frame, velocity));
    REQUIRE_FALSE(write<can_field::feedback1::Current>(frame, current));
    REQUIRE_FALSE(write<can_field::feedback1::MotorTemp>(frame, uint8_t{100}));
    REQUIRE_FALSE(write<can_field::feedback1::MosfetTemp>(frame, uint8_t{100}));
    return frame;
}

} // namespace

TEST_CASE("errors are propagated when can_field exceeds available frame length") {
    can_frame frame{};
    frame.len = 1;

    auto write_err = write<test_field::OutOfRange>(frame, 0xff);
    REQUIRE(static_cast<bool>(write_err));

    auto read_result = read<test_field::OutOfRange>(frame);
    REQUIRE_FALSE(read_result);
}

TEST_CASE("typed read propagates raw read failure") {
    can_frame frame{};
    frame.len = 1;

    auto result = read<test_field::OutOfRange>(frame).transform(to_u8);

    REQUIRE_FALSE(result);
}

TEST_CASE("position command writes degree as float bit pattern") {
    position_command command;
    command.position_deg = 3.25f;
    command.velocity_limit_rpm = 120.0f;
    command.current_limit = 5.0f;
    command.reply = reply_mode::position;

    auto frame = can_codec::make_position({.can_id = 0x123}, command);

    REQUIRE(frame);
    REQUIRE(*read<can_field::position_control::Mode>(*frame) == enum_u8(command_code::position_control));
    REQUIRE(*read<can_field::position_control::PositionDeg>(*frame) == std::bit_cast<uint32_t>(3.25f));
}

TEST_CASE("PVT command matches app_can pvt_ctrl bit layout") {
    auto command = make_pvt_command(125.0, 2.5, 1.25, -3.0, 6.0);

    auto frame = can_codec::make_pvt({.can_port = 2, .can_id = 0x123}, command);

    REQUIRE(frame);
    require_frame_bytes(*frame, std::array<uint8_t, 8>{0x08, 0x01, 0x00, 0x8c, 0xcc, 0x6a, 0xa9, 0x98});
    REQUIRE(frame->id == 0x123);
    REQUIRE(frame->port == 2);
    REQUIRE(*read<can_field::pvt::Mode>(*frame) == enum_u8(command_code::pvt_control));
    REQUIRE(*read<can_field::pvt::Kp>(*frame) == 1024);
    REQUIRE(*read<can_field::pvt::Kd>(*frame) == 256);
    REQUIRE(*read<can_field::pvt::Position>(*frame) == 36044);
    REQUIRE(*read<can_field::pvt::Velocity>(*frame) == 1706);
    REQUIRE(*read<can_field::pvt::Torque>(*frame) == 2456);
}

TEST_CASE("PVT command encodes neutral values to protocol centers") {
    auto frame = can_codec::make_pvt({.can_port = 1, .can_id = 0x123}, make_pvt_command(0.0, 0.0, 0.0, 0.0, 0.0));

    REQUIRE(frame);
    require_frame_bytes(*frame, std::array<uint8_t, 8>{0x00, 0x00, 0x00, 0x7f, 0xff, 0x7f, 0xf7, 0xff});
    require_pvt_fields(*frame, 0, 0, 32767, 2047, 2047);
}

TEST_CASE("PVT command encodes lower physical bounds") {
    auto frame = can_codec::make_pvt({.can_id = 0x123}, make_pvt_command(0.0, 0.0, -12.5, -18.0, -30.0));

    REQUIRE(frame);
    require_frame_bytes(*frame, std::array<uint8_t, 8>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    require_pvt_fields(*frame, 0, 0, 0, 0, 0);
}

TEST_CASE("PVT command encodes upper physical bounds without overflowing field width") {
    auto frame = can_codec::make_pvt({.can_id = 0x123}, make_pvt_command(500.0, 5.0, 12.5, 18.0, 30.0));

    REQUIRE(frame);
    require_frame_bytes(*frame, std::array<uint8_t, 8>{0x1f, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xef, 0xfe});
    require_pvt_fields(*frame, 4095, 511, 65534, 4094, 4094);
}

TEST_CASE("PVT command rejects values outside protocol field ranges") {
    auto high = can_codec::make_pvt({.can_id = 0x123}, make_pvt_command(1000.0, 10.0, 100.0, 100.0, 100.0));
    require_invalid_argument(high);

    auto low = can_codec::make_pvt({.can_id = 0x123}, make_pvt_command(-1.0, -1.0, -100.0, -100.0, -100.0));
    require_invalid_argument(low);

    require_invalid_argument(can_codec::make_pvt(
        {.can_id = 0x123}, make_pvt_command(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 0.0f, 0.0f)));
    require_invalid_argument(can_codec::make_pvt(
        {.can_id = 0x123}, make_pvt_command(0.0f, 0.0f, std::numeric_limits<float>::infinity(), 0.0f, 0.0f)));
}

TEST_CASE("PVT command data format keeps all fields inside one 8-byte frame") {
    auto frame = can_codec::make_pvt({.can_port = 3, .can_id = 0x456}, make_pvt_command(250.0, 1.25, 6.25, 9.0, 15.0));

    REQUIRE(frame);
    REQUIRE(frame->id == 0x456);
    REQUIRE(frame->port == 3);
    REQUIRE(frame->len == 8);
    require_pvt_fields(*frame, 2048, 128, 49151, 3071, 3071);
}

TEST_CASE("status feedback decodes position and velocity as symmetric protocol ranges") {
    struct case_t {
        uint16_t position;
        uint16_t velocity;
        float expected_position_r;
        float expected_velocity_radps;
    };
    const std::array<case_t, 3> cases{{
        {.position = 0, .velocity = 0, .expected_position_r = -12.5f, .expected_velocity_radps = -18.0f},
        {.position = 32767, .velocity = 2047, .expected_position_r = 0.0f, .expected_velocity_radps = 0.0f},
        {.position = 65535, .velocity = 4095, .expected_position_r = 12.5f, .expected_velocity_radps = 18.0f},
    }};

    for (const auto &test : cases) {
        const auto response =
            can_codec::parse_feedback(make_status_feedback_frame(test.position, test.velocity, 0));

        REQUIRE(response);
        const auto &status = std::get<status_feedback>(*response);
        REQUIRE(status.position_r == doctest::Approx(test.expected_position_r).epsilon(0.001));
        REQUIRE(std::abs(status.velocity_radps - test.expected_velocity_radps) < 0.01f);
    }
}

TEST_CASE("status feedback decodes current through supplied motor range") {
    const auto frame = make_status_feedback_frame(32767, 2047, 4095);
    const range_t<float> current_range{.min = -10.0F, .max = 10.0F};

    const auto response = can_codec::parse_feedback(frame, &current_range);

    REQUIRE(response);
    const auto &status = std::get<status_feedback>(*response);
    CHECK(status.current == doctest::Approx(10.0F));
}

TEST_CASE("position command matches app_can pos_ctrl bit layout") {
    position_command command;
    command.position_deg = 3.25f;
    command.velocity_limit_rpm = 120.0f;
    command.current_limit = 5.0f;
    command.reply = reply_mode::position;

    auto frame = can_codec::make_position({.can_port = 1, .can_id = 0x321}, command);

    REQUIRE(frame);
    require_frame_bytes(*frame, std::array<uint8_t, 8>{0x28, 0x0a, 0x00, 0x00, 0x01, 0x2c, 0x00, 0xca});
    REQUIRE(frame->id == 0x321);
    REQUIRE(frame->port == 1);
    REQUIRE(*read<can_field::position_control::Mode>(*frame) == enum_u8(command_code::position_control));
    REQUIRE(*read<can_field::position_control::PositionDeg>(*frame) == f32_to_bits(3.25));
    REQUIRE(*read<can_field::position_control::VelocityRpmX10>(*frame) == 1200);
    REQUIRE(*read<can_field::position_control::CurrentAmpX10>(*frame) == 50);
    REQUIRE(*read<can_field::position_control::Reply>(*frame) == enum_u8(reply_mode::position));
}

TEST_CASE("position and velocity commands reject scaled values outside field ranges") {
    position_command position;
    position.position_deg = 0.0f;
    position.reply = reply_mode::position;

    position.velocity_limit_rpm = 3276.7f;
    position.current_limit = 409.5f;
    auto max_position = can_codec::make_position({.can_id = 0x123}, position);
    REQUIRE(max_position);
    REQUIRE(*read<can_field::position_control::VelocityRpmX10>(*max_position) == 32767);
    REQUIRE(*read<can_field::position_control::CurrentAmpX10>(*max_position) == 4095);

    position.velocity_limit_rpm = 3276.8f;
    position.current_limit = 1.0f;
    require_invalid_argument(can_codec::make_position({.can_id = 0x123}, position));

    position.velocity_limit_rpm = 1.0f;
    position.current_limit = 409.6f;
    require_invalid_argument(can_codec::make_position({.can_id = 0x123}, position));

    position.velocity_limit_rpm = -0.1f;
    position.current_limit = 1.0f;
    require_invalid_argument(can_codec::make_position({.can_id = 0x123}, position));

    position.velocity_limit_rpm = std::numeric_limits<float>::infinity();
    position.current_limit = 1.0f;
    require_invalid_argument(can_codec::make_position({.can_id = 0x123}, position));

    velocity_command velocity;
    velocity.velocity_rpm = 0.0f;
    velocity.current_limit = 6553.5f;
    auto max_velocity = can_codec::make_velocity({.can_id = 0x123}, velocity);
    REQUIRE(max_velocity);
    REQUIRE(*read<can_field::velocity_control::CurrentAmpX10>(*max_velocity) == 65535);

    velocity.current_limit = 6553.6f;
    require_invalid_argument(can_codec::make_velocity({.can_id = 0x123}, velocity));

    velocity.current_limit = -0.1f;
    require_invalid_argument(can_codec::make_velocity({.can_id = 0x123}, velocity));
}

TEST_CASE("velocity command writes rpm as float bit pattern") {
    velocity_command command;
    command.velocity_rpm = 1500.5f;
    command.current_limit = 4.0f;
    command.reply = reply_mode::velocity;

    auto frame = can_codec::make_velocity({.can_id = 0x123}, command);

    REQUIRE(frame);
    REQUIRE(*read<can_field::velocity_control::Rpm>(*frame) == std::bit_cast<uint32_t>(1500.5f));
}

TEST_CASE("velocity command matches app_can vel_ctrl bit layout") {
    velocity_command command;
    command.velocity_rpm = 1500.5f;
    command.current_limit = 4.0f;
    command.reply = reply_mode::velocity;

    auto frame = can_codec::make_velocity({.can_port = 1, .can_id = 0x456}, command);

    REQUIRE(frame);
    require_frame_bytes(*frame, std::array<uint8_t, 7>{0x43, 0x44, 0xbb, 0x90, 0x00, 0x00, 0x28});
    REQUIRE(frame->id == 0x456);
    REQUIRE(frame->port == 1);
    REQUIRE(*read<can_field::velocity_control::Mode>(*frame) == enum_u8(command_code::velocity_control));
    REQUIRE(*read<can_field::velocity_control::Reserved>(*frame) == 0);
    REQUIRE(*read<can_field::velocity_control::Reply>(*frame) == enum_u8(reply_mode::velocity));
    REQUIRE(*read<can_field::velocity_control::Rpm>(*frame) == f32_to_bits(1500.5));
    REQUIRE(*read<can_field::velocity_control::CurrentAmpX10>(*frame) == 40);
}

TEST_CASE("torque command keeps torque control mode") {
    torque_command command;
    command.torque_nm = 1.25f;

    auto frame = can_codec::make_torque({.can_id = 0x123}, command);

    REQUIRE(frame);
    REQUIRE(*read<can_field::torque_control::Mode>(*frame) == enum_u8(command_code::torque_control));
    REQUIRE(*read<can_field::torque_control::State>(*frame) == enum_u8(torque_control_mode::torque));
}

TEST_CASE("torque and current commands match app_can tor_ctrl bit layout") {
    torque_command torque;
    torque.torque_nm = 1.25f;
    torque.reply = reply_mode::status1;

    auto torque_frame = can_codec::make_torque({.can_id = 0x123}, torque);

    REQUIRE(torque_frame);
    require_frame_bytes(*torque_frame, std::array<uint8_t, 3>{0x65, 0x00, 0x7d});
    REQUIRE(*read<can_field::torque_control::Mode>(*torque_frame) == enum_u8(command_code::torque_control));
    REQUIRE(*read<can_field::torque_control::State>(*torque_frame) == enum_u8(torque_control_mode::torque));
    REQUIRE(*read<can_field::torque_control::Reply>(*torque_frame) == enum_u8(reply_mode::status1));
    REQUIRE(*read<can_field::torque_control::Value>(*torque_frame) == 125);

    current_command current;
    current.current = 2.5f;
    current.reply = reply_mode::position;

    auto current_frame = can_codec::make_current({.can_id = 0x123}, current);

    REQUIRE(current_frame);
    require_frame_bytes(*current_frame, std::array<uint8_t, 3>{0x62, 0x00, 0xfa});
    REQUIRE(*read<can_field::torque_control::Mode>(*current_frame) == enum_u8(command_code::torque_control));
    REQUIRE(*read<can_field::torque_control::State>(*current_frame) == enum_u8(torque_control_mode::current));
    REQUIRE(*read<can_field::torque_control::Reply>(*current_frame) == enum_u8(reply_mode::position));
    REQUIRE(*read<can_field::torque_control::Value>(*current_frame) == 250);
}

TEST_CASE("torque current and brake commands reject scaled int16 overflows") {
    torque_command torque;
    torque.torque_nm = 327.67f;
    auto max_torque = can_codec::make_torque({.can_id = 0x123}, torque);
    if (max_torque) {
        REQUIRE(static_cast<int16_t>(*read<can_field::torque_control::Value>(*max_torque)) == 32767);
    }

    torque.torque_nm = 327.68f;
    require_invalid_argument(can_codec::make_torque({.can_id = 0x123}, torque));

    torque.torque_nm = -327.68f;
    auto min_torque = can_codec::make_torque({.can_id = 0x123}, torque);
    if (min_torque) {
        REQUIRE(static_cast<int16_t>(*read<can_field::torque_control::Value>(*min_torque)) == -32768);
    }

    torque.torque_nm = -327.69f;
    require_invalid_argument(can_codec::make_torque({.can_id = 0x123}, torque));

    current_command current;
    current.current = std::numeric_limits<float>::quiet_NaN();
    require_invalid_argument(can_codec::make_current({.can_id = 0x123}, current));

    brake_command brake;
    brake.control = regen_brake{.current_threshold = 327.68f};
    require_invalid_argument(can_codec::make_brake({.can_id = 0x123}, brake));
}

TEST_CASE("brake command matches app_can brake_ctrl bit layout") {
    brake_command command;
    command.control = engage_brake{.open = true};
    command.reply = reply_mode::none;

    auto frame = can_codec::make_brake({.can_id = 0x123}, command);

    REQUIRE(frame);
    require_frame_bytes(*frame, std::array<uint8_t, 2>{0x80, 0x01});
    REQUIRE(*read<can_field::brake_control::Mode>(*frame) == enum_u8(command_code::brake_control));
    REQUIRE(*read<can_field::brake_control::Reserved>(*frame) == 0);
    REQUIRE(*read<can_field::brake_control::Engaged>(*frame) == 1);
}

TEST_CASE("config command matches app_can motor_cfg scalar layout") {
    auto frame = can_codec::make_config({.can_port = 2, .can_id = 0x123}, config_set_command{
                                                                              .code = config_set_code::can_timeout,
                                                                              .value = 1000ms,
                                                                          });

    REQUIRE(frame);
    require_frame_bytes(*frame, std::array<uint8_t, 4>{0xc0, 0x0b, 0x03, 0xe8});
    REQUIRE(frame->id == 0x123);
    REQUIRE(frame->port == 2);
    REQUIRE(*read<can_field::config::Mode>(*frame) == enum_u8(command_code::motor_config));
    REQUIRE(*read<can_field::config::Reply>(*frame) == enum_u8(reply_mode::none));
    REQUIRE(*read<can_field::config::Key>(*frame) == enum_u8(config_set_code::can_timeout));
    REQUIRE(*read<can_field::config::Value>(*frame) == 1000);
}

TEST_CASE("config command supports ack bit for app_can accel and comm mode") {
    auto accel = can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                               .code = config_set_code::accel,
                                                               .value = 12.34f,
                                                           });
    auto comm = can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                              .code = config_set_code::comm_mode,
                                                              .value = uint8_t{2},
                                                          });

    REQUIRE(accel);
    // require_frame_bytes(*accel, std::array<uint8_t, 4>{0xc1, 0x01, 0x04, 0xd2});
    REQUIRE(*read<can_field::config::Mode>(*accel) == enum_u8(command_code::motor_config));
    REQUIRE(*read<can_field::config::Reply>(*accel) == 0);
    REQUIRE(*read<can_field::config::Key>(*accel) == enum_u8(config_set_code::accel));
    REQUIRE(*read<can_field::config::Value>(*accel) == 1234);

    REQUIRE(comm);
    // require_frame_bytes(*comm, std::array<uint8_t, 3>{0xc1, 0x02, 0x02});
    REQUIRE(*read<can_field::config::Mode>(*comm) == enum_u8(command_code::motor_config));
    REQUIRE(*read<can_field::config::Reply>(*comm) == 0);
    REQUIRE(*read<can_field::config::Key>(*comm) == enum_u8(config_set_code::comm_mode));
}

TEST_CASE("config command matches app_can range and signed scaling layout") {
    auto kp = can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                            .code = config_set_code::pvt_kp,
                                                            .value = range_t{.min = 10.0f, .max = 500.0f},
                                                        });
    auto pos = can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                             .code = config_set_code::pvt_position_limit_r,
                                                             .value = range_t{.min = -12.5f, .max = 12.5f},
                                                         });
    auto torque = can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                                .code = config_set_code::pvt_torque_limit_nm,
                                                                .value = range_t{.min = -30.0f, .max = 30.0f},
                                                            });

    REQUIRE(kp);
    require_frame_bytes(*kp, std::array<uint8_t, 6>{0xc0, 0x05, 0x00, 0x0a, 0x01, 0xf4});
    REQUIRE(*read<can_field::config::Min>(*kp) == 10);
    REQUIRE(*read<can_field::config::Max>(*kp) == 500);

    REQUIRE(pos);
    require_frame_bytes(*pos, std::array<uint8_t, 6>{0xc0, 0x07, 0xfb, 0x1e, 0x04, 0xe2});
    REQUIRE(static_cast<int16_t>(*read<can_field::config::Min>(*pos)) == -1250);
    REQUIRE(static_cast<int16_t>(*read<can_field::config::Max>(*pos)) == 1250);

    REQUIRE(torque);
    require_frame_bytes(*torque, std::array<uint8_t, 6>{0xc0, 0x09, 0xfe, 0xd4, 0x01, 0x2c});
    REQUIRE(static_cast<int16_t>(*read<can_field::config::Min>(*torque)) == -300);
    REQUIRE(static_cast<int16_t>(*read<can_field::config::Max>(*torque)) == 300);
}

TEST_CASE("config command rejects values that overflow scaled payload fields") {
    require_invalid_argument(can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                                           .code = config_set_code::accel,
                                                                           .value = 655.36f,
                                                                       }));
    require_invalid_argument(can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                                           .code = config_set_code::accel,
                                                                           .value = -0.01f,
                                                                       }));
    require_invalid_argument(can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                                           .code = config_set_code::torque_ratio,
                                                                           .value = 655.36,
                                                                       }));
    require_invalid_argument(
        can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                      .code = config_set_code::pvt_kp,
                                                      .value = range_t{.min = 0.0f, .max = 65536.0f},
                                                  }));
    require_invalid_argument(
        can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                      .code = config_set_code::pvt_position_limit_r,
                                                      .value = range_t{.min = -327.69f, .max = 327.67f},
                                                  }));
    require_invalid_argument(
        can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                      .code = config_set_code::pvt_current_limit,
                                                      .value = range_t{.min = -3276.8f, .max = 3276.8f},
                                                  }));
    require_invalid_argument(can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                                           .code = config_set_code::can_timeout,
                                                                           .value = milliseconds{65536},
                                                                       }));
    require_invalid_argument(
        can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                      .code = config_set_code::current_loop_pi,
                                                      .value = pid_gains{.kp = 6.5536f, .ki = 0.0f},
                                                  }));
    require_invalid_argument(can_codec::make_config(
        {.can_id = 0x123}, config_set_command{
                               .code = config_set_code::velocity_loop_pi,
                               .value = pid_gains{.kp = 0.0f, .ki = std::numeric_limits<float>::infinity()},
                           }));
}

TEST_CASE("config command matches app_can pi pd scaling layout") {
    auto current = can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                                 .code = config_set_code::current_loop_pi,
                                                                 .value = pid_gains{.kp = 0.1234f, .ki = 56.7f},
                                                             });
    auto velocity = can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                                  .code = config_set_code::velocity_loop_pi,
                                                                  .value = pid_gains{.kp = 0.01234f, .ki = 0.05678f},
                                                              });
    auto position = can_codec::make_config({.can_id = 0x123}, config_set_command{
                                                                  .code = config_set_code::position_loop_pd,
                                                                  .value = pid_gains{.kp = 0.01234f, .kd = 0.05678f},
                                                              });

    REQUIRE(current);
    require_frame_bytes(*current, std::array<uint8_t, 6>{0xc0, 0x0c, 0x04, 0xd2, 0x02, 0x37});

    REQUIRE(velocity);
    require_frame_bytes(*velocity, std::array<uint8_t, 6>{0xc0, 0x0d, 0x04, 0xd2, 0x16, 0x2e});

    REQUIRE(position);
    require_frame_bytes(*position, std::array<uint8_t, 6>{0xc0, 0x0e, 0x04, 0xd2, 0x16, 0x2e});
}

TEST_CASE("config query command matches app_can motor_cfg_query bit layout") {
    auto frame = can_codec::make_config_query({.can_port = 2, .can_id = 0x123},
                                              config_query_command{.code = config_query_code::can_timeout});

    REQUIRE(frame);
    require_frame_bytes(*frame, std::array<uint8_t, 2>{0xe0, 0x1f});
    REQUIRE(frame->id == 0x123);
    REQUIRE(frame->port == 2);
    REQUIRE(*read<can_field::config_query::Mode>(*frame) == enum_u8(command_code::motor_config_query));
    REQUIRE(*read<can_field::config_query::Key>(*frame) == enum_u8(config_query_code::can_timeout));
}

TEST_CASE("config query response is decoded into semantic motor query result") {
    can_frame frame{};
    frame.id = 0x123;
    frame.len = 4;
    frame.data[0] = 0xa0;
    frame.data[1] = enum_u8(config_query_code::can_timeout);
    frame.data[2] = 0x03;
    frame.data[3] = 0xe8;

    auto response = can_codec::parse_feedback(frame);

    REQUIRE(response);
    const auto &result = std::get<query_feedback>(*response);
    REQUIRE(result.code == config_query_code::can_timeout);
    REQUIRE(result.can_timeout == 1000ms);
}

TEST_CASE("config query float response uses common CAN big endian float encoding") {
    struct case_t {
        config_query_code code;
        float value;
    };
    const std::array<case_t, 4> cases{{
        {.code = config_query_code::position, .value = 196.5f},
        {.code = config_query_code::velocity, .value = -12.25f},
        {.code = config_query_code::current, .value = 0.8f},
        {.code = config_query_code::power, .value = 42.125f},
    }};

    for (const auto &test : cases) {
        can_frame frame{};
        frame.id = 0x123;
        frame.len = 6;
        frame.data[0] = 0xa0;
        frame.data[1] = enum_u8(test.code);

        const auto raw = f32_to_bits(test.value);
        frame.data[2] = static_cast<uint8_t>(raw >> 24U);
        frame.data[3] = static_cast<uint8_t>(raw >> 16U);
        frame.data[4] = static_cast<uint8_t>(raw >> 8U);
        frame.data[5] = static_cast<uint8_t>(raw);

        auto response = can_codec::parse_feedback(frame);

        REQUIRE(response);
        const auto &result = std::get<query_feedback>(*response);
        REQUIRE(result.code == test.code);
        switch (test.code) {
        case config_query_code::position:
            REQUIRE(result.position_deg == doctest::Approx(test.value));
            break;
        case config_query_code::velocity:
            REQUIRE(result.velocity_rpm == doctest::Approx(test.value));
            break;
        case config_query_code::current:
            REQUIRE(result.current == doctest::Approx(test.value));
            break;
        case config_query_code::power:
            REQUIRE(result.power == doctest::Approx(test.value));
            break;
        default:
            FAIL("unexpected query code");
        }
    }
}

TEST_CASE("motor debug manual velocity command golden frames") {
    struct case_t {
        float rpm;
        std::array<uint8_t, 7> expected;
    };
    const std::array<case_t, 3> cases{{
        {20.0f, {0x43, 0x41, 0xa0, 0x00, 0x00, 0x00, 0x32}},
        {60.0f, {0x43, 0x42, 0x70, 0x00, 0x00, 0x00, 0x32}},
        {-60.0f, {0x43, 0xc2, 0x70, 0x00, 0x00, 0x00, 0x32}},
    }};

    for (const auto &test : cases) {
        velocity_command command;
        command.velocity_rpm = test.rpm;
        command.current_limit = 5.0f;
        command.reply = reply_mode::velocity;

        FLEET_INFO("debug manual velocity golden frame rpm={}", test.rpm);
        auto frame = can_codec::make_velocity({.can_id = 1}, command);

        REQUIRE(frame);
        require_frame_bytes(*frame, test.expected);
    }
}

TEST_CASE("motor debug manual current command golden frames") {
    struct case_t {
        float ampere;
        std::array<uint8_t, 3> expected;
    };
    const std::array<case_t, 3> cases{{
        {3.0f, {0x62, 0x01, 0x2c}},
        {5.0f, {0x62, 0x01, 0xf4}},
        {-5.0f, {0x62, 0xfe, 0x0c}},
    }};

    for (const auto &test : cases) {
        current_command command;
        command.current = test.ampere;
        command.reply = reply_mode::position;

        FLEET_INFO("debug manual current golden frame current_a={}", test.ampere);
        auto frame = can_codec::make_current({.can_id = 1}, command);

        REQUIRE(frame);
        require_frame_bytes(*frame, test.expected);
    }
}

TEST_CASE("motor debug manual torque command golden frames") {
    struct case_t {
        float newton_meter;
        std::array<uint8_t, 3> expected;
    };
    const std::array<case_t, 3> cases{{
        {3.0f, {0x66, 0x01, 0x2c}},
        {5.0f, {0x66, 0x01, 0xf4}},
        {-5.0f, {0x66, 0xfe, 0x0c}},
    }};

    for (const auto &test : cases) {
        torque_command command;
        command.torque_nm = test.newton_meter;
        command.reply = reply_mode::position;

        FLEET_INFO("debug manual torque golden frame torque_nm={}", test.newton_meter);
        auto frame = can_codec::make_torque({.can_id = 1}, command);

        REQUIRE(frame);
        require_frame_bytes(*frame, test.expected);
    }
}

TEST_CASE("motor debug manual position command golden frames") {
    struct case_t {
        float degree;
        std::array<uint8_t, 8> expected;
    };
    const std::array<case_t, 3> cases{{
        {0.0f, {0x20, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0xca}},
        {90.0f, {0x28, 0x56, 0x80, 0x00, 0x00, 0x32, 0x00, 0xca}},
        {-90.0f, {0x38, 0x56, 0x80, 0x00, 0x00, 0x32, 0x00, 0xca}},
    }};

    for (const auto &test : cases) {
        position_command command;
        command.position_deg = test.degree;
        command.velocity_limit_rpm = 20.0f;
        command.current_limit = 5.0f;
        command.reply = reply_mode::position;

        FLEET_INFO("debug manual position golden frame degree={}", test.degree);
        auto frame = can_codec::make_position({.can_id = 1}, command);

        REQUIRE(frame);
        require_frame_bytes(*frame, test.expected);
    }
}
