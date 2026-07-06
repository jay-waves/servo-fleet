#include <doctest/doctest.h>

#include "channel.h"
#include "driver_motor.h"
#include "codec_motor.h"
#include "fleet/fleet.h"

#include <asio.hpp>

#include <cmath>
#include <chrono>
#include <string>
#include <thread>

namespace {

using asio::ip::udp;
using namespace fleet;
using namespace std::chrono_literals;

TEST_CASE("runtime reports uninitialized service") {
    shutdown();

    CHECK_FALSE(ok());
    CHECK(command(velocity_command{.device_id = 0x1234}) == fleet_err::io_error);
}

TEST_CASE("motor samples preserve update time for caller-side freshness checks") {
    fleet_config::motor_config config {
        .name = "motor",
        .channel = "hello",
        .model = "EC-A2806-P2-36",
        .can_port = 0,
        .can_id = 1,
    };
    motor_device motor(1, config);

    motor.apply(status_feedback{
        .error = motor_err::none,
        .position_r = 1.0F,
        .velocity_radps = 2.0F,
        .current = 3.0F,
        .motor_temp_c = 40.0F,
        .mosfet_temp_c = 45.0F,
    });

    REQUIRE(motor.position_deg());
    REQUIRE(motor.motor_error());

    std::this_thread::sleep_for(15ms);
    const auto position = motor.position_deg();
    const auto error = motor.motor_error();

    CHECK(position);
    CHECK(position.value > 57.0F);
    CHECK(std::chrono::steady_clock::now() - position.time > 10ms);
    CHECK(error.value == motor_err::none);
}

TEST_CASE("motor model table supplies pvt limits and status current range") {
    fleet_config::motor_config config{
        .name = "motor",
        .channel = "can",
        .model = "EC-A2806-P2-36",
        .can_port = 0,
        .can_id = 1,
    };
    motor_device motor(1, config);

    REQUIRE(motor.pvt_position_limit_r());
    CHECK(motor.pvt_position_limit_r().value.min == doctest::Approx(-12.5F));
    CHECK(motor.pvt_position_limit_r().value.max == doctest::Approx(12.5F));
    CHECK(motor.pvt_position_limit_r().fresh(std::chrono::steady_clock::now(), 1ms));

    motor.apply(query_feedback{
        .code = config_query_code::pvt_kd_range,
        .pvt_kd_range = {.min = 100.0F, .max = 200.0F},
    });
    REQUIRE(motor.pvt_kd_range());
    CHECK(motor.pvt_kd_range().value.min == doctest::Approx(100.0F));
    CHECK(motor.pvt_kd_range().value.max == doctest::Approx(200.0F));
}

TEST_CASE("runtime routes logical motor commands to configured endpoint and CAN bus") {
    shutdown();

    asio::io_context io;
    udp::socket endpoint0_receiver(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));
    udp::socket endpoint1_receiver(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));

    REQUIRE_FALSE(init({
        .channels =
            {
                {
                    .name = "can0",
                    .remote_address = "127.0.0.1",
                    .remote_port = endpoint0_receiver.local_endpoint().port(),
                    .local_port = 0,
                },
                {
                    .name = "can1",
                    .remote_address = "127.0.0.1",
                    .remote_port = endpoint1_receiver.local_endpoint().port(),
                    .local_port = 0,
                },
            },
        .motors =
            {
                {.name = "left", .channel = "can0", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 1},
                {.name = "right", .channel = "can1", .model = "EC-A2806-P2-36", .can_port = 2, .can_id = 3},
            },
    }));

    const auto motors = lookup<motor::motor_interface>();
    REQUIRE(motors.size() == 2);
    CHECK(std::string{motors[0]->name()} == "left");
    CHECK(std::string{motors[1]->name()} == "right");

    const std::array commands{
        velocity_command{
            .device_id = 1,
            .velocity_rpm = 120.0F,
            .current_limit = 3.0F,
        },
        velocity_command{
            .device_id = 0,
            .velocity_rpm = 60.0F,
            .current_limit = 2.0F,
        },
        velocity_command{
            .device_id = 100,
            .velocity_rpm = 30.0F,
            .current_limit = 1.0F,
        },
    };
    CHECK(command(std::span<const velocity_command>(commands)) == fleet_err::invalid_argument);

    const auto receive_frame = [](udp::socket &receiver) {
        std::array<uint8_t, 256> buffer{};
        udp::endpoint sender;
        asio::error_code error;
        const auto size = receiver.receive_from(asio::buffer(buffer), sender, 0, error);
        REQUIRE_FALSE(error);

        std::vector<can_frame> frames;
        const auto collect = [](void *context, const can_frame &frame) {
            static_cast<std::vector<can_frame> *>(context)->push_back(frame);
        };
        REQUIRE_FALSE(motor_codec::decode_each(bytes_view(buffer.data(), size), &frames, collect));
        REQUIRE(frames.size() == 1);
        return frames.front();
    };

    const auto endpoint0_frame = receive_frame(endpoint0_receiver);
    CHECK(endpoint0_frame.port == 0);
    CHECK(endpoint0_frame.id == 1);
    CHECK(endpoint0_frame.len == 7);

    const auto endpoint1_frame = receive_frame(endpoint1_receiver);
    CHECK(endpoint1_frame.port == 2);
    CHECK(endpoint1_frame.id == 3);
    CHECK(endpoint1_frame.len == 7);

    shutdown();
}

TEST_CASE("runtime binds multiple motors to one channel driver") {
    shutdown();

    REQUIRE_FALSE(init({
        .channels =
            {
                {.name = "can"},
            },
        .motors =
            {
                {.name = "left", .channel = "can", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 1},
                {.name = "right", .channel = "can", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 2},
            },
    }));

    CHECK(lookup<motor::motor_interface>().size() == 2);
    shutdown();
}

TEST_CASE("encos channel rejects new packets when the send queue is full") {
    asio::io_context io;
    udp::socket receiver(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));

    auto channel = std::make_unique<realtime_udp_channel>(
        "can",
        udp_channel_options{
            .remote_address = "127.0.0.1",
            .remote_port = receiver.local_endpoint().port(),
            .local_port = 0,
        });
    motor_driver driver(std::move(channel));

    can_frame frame;
    frame.id = 1;
    frame.len = 1;

    constexpr size_t MAX_EXPECTED_QUEUE_DEPTH = 2048;

    size_t accepted = 0;
    size_t rejected = 0;
    while (accepted + rejected < MAX_EXPECTED_QUEUE_DEPTH) {
        const auto error = driver.send(frame);
        if (!error) {
            ++accepted;
        } else {
            CHECK(error == fleet_err::busy);
            ++rejected;
            break;
        }
    }

    driver.stop();

    CHECK(accepted > 0);
    CHECK(accepted < MAX_EXPECTED_QUEUE_DEPTH);
    CHECK(rejected == 1);
}

TEST_CASE("encos channel dispatches feedback through direct CAN address routes") {
    const auto local_port = [] {
        asio::io_context io;
        udp::socket socket(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));
        return socket.local_endpoint().port();
    }();

    asio::io_context io;
    udp::socket sender(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));
    auto channel = std::make_unique<realtime_udp_channel>(
        "can",
        udp_channel_options{
            .remote_address = "127.0.0.1",
            .remote_port = sender.local_endpoint().port(),
            .local_port = local_port,
        });
    motor_driver driver(std::move(channel));
    fleet_config::motor_config config1 {
        .name = "yyyh",
        .channel = "xxx",
        .model = "EC-A2806-P2-36",
        .can_port = 0,
        .can_id = 1,
    };
    fleet_config::motor_config config2 {
        .name = "yyyh",
        .channel = "xxx",
        .model = "EC-A2806-P2-36",
        .can_port = 3,
        .can_id = 17,
    };
    motor_device first(0, config1);
    motor_device second(1, config2);
    driver.bind_motor(first);
    driver.bind_motor(second);
    driver.start();

    can_frame feedback;
    feedback.port = 3;
    feedback.id = 17;
    feedback.len = 8;
    REQUIRE_FALSE(write<can_field::feedback1::Type>(feedback, uint8_t{1}));
    REQUIRE_FALSE(write<can_field::feedback1::Error>(feedback, uint8_t{0}));
    REQUIRE_FALSE(write<can_field::feedback1::Position>(feedback, uint16_t{32767}));
    REQUIRE_FALSE(write<can_field::feedback1::Velocity>(feedback, uint16_t{2047}));
    REQUIRE_FALSE(write<can_field::feedback1::Current>(feedback, uint16_t{0}));
    REQUIRE_FALSE(write<can_field::feedback1::MotorTemp>(feedback, uint8_t{100}));
    REQUIRE_FALSE(write<can_field::feedback1::MosfetTemp>(feedback, uint8_t{100}));

    bytes packet(motor_codec::MAX_PACKET_SZ);
    auto packet_size =
        motor_codec::encode_into(bytes_mut(packet), TYPE_CAN2UDP, std::span<const can_frame>(&feedback, 1));
    REQUIRE(packet_size);
    packet.resize(*packet_size);
    sender.send_to(asio::buffer(packet),
                   udp::endpoint(asio::ip::make_address("127.0.0.1"), local_port));

    sample_t<float> second_position;
    sample_t<float> second_current;
    for (size_t i = 0; i < 50 && (!second_position || !second_current); ++i) {
        std::this_thread::sleep_for(2ms);
        second_position = second.position_deg();
        second_current = second.current();
    }

    driver.stop();

    CHECK_FALSE(first.position_deg());
    REQUIRE(second_position);
    CHECK(std::fabs(second_position.value) < 0.02F);
    REQUIRE(second_current);
    CHECK(second_current.value == doctest::Approx(-10.0F));
}

TEST_CASE("runtime rejects duplicate topology names") {
    shutdown();

    SUBCASE("duplicate channel name") {
        CHECK(init({
                  .channels =
                      {
                          {.name = "can"},
                          {.name = "can"},
                      },
              }) == fleet_err::invalid_argument);
    }

    SUBCASE("duplicate device name") {
        CHECK(init({
                  .channels =
                      {
                          {.name = "can"},
                      },
                  .motors =
                      {
                          {.name = "status", .channel = "can", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 1},
                          {.name = "status", .channel = "can", .model = "EC-A2806-P2-36", .can_port = 1, .can_id = 2},
                      },
              }) == fleet_err::invalid_argument);
    }

    SUBCASE("unknown channel reference") {
        CHECK(init({
                  .motors =
                      {
                          {.name = "left", .channel = "missing", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 1},
                      },
              }) == fleet_err::invalid_argument);
    }

    SUBCASE("duplicate motor address") {
        CHECK(init({
                  .channels =
                      {
                          {.name = "can"},
                      },
                  .motors =
                      {
                          {.name = "left", .channel = "can", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 1},
                          {.name = "right", .channel = "can", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 1},
                      },
              }) == fleet_err::invalid_argument);
    }

    SUBCASE("motor CAN id exceeds the standard frame range") {
        CHECK(init({
                  .channels =
                      {
                          {.name = "can"},
                      },
                  .motors =
                      {
                          {.name = "motor", .channel = "can", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 0x800},
                      },
              }) == fleet_err::invalid_argument);
    }

    SUBCASE("zero motor channel bandwidth") {
        CHECK(init({
                  .channels =
                      {
                          {.name = "can", .max_bandwidth_bps = 0},
                      },
                  .motors =
                      {
                          {.name = "motor", .channel = "can", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 1},
                      },
              }) == fleet_err::invalid_argument);
    }

    SUBCASE("unsupported motor model") {
        CHECK(init({
                  .channels =
                      {
                          {.name = "can"},
                      },
                  .motors =
                      {
                          {.name = "motor", .channel = "can", .model = "missing", .can_port = 0, .can_id = 1},
                      },
              }) == fleet_err::invalid_argument);
    }

    shutdown();
}

} // namespace
