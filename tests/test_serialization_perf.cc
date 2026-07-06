#define ANKERL_NANOBENCH_IMPLEMENT
#include <nanobench.h>

#include "driver_motor.h"
#include "fleet/fleet.h"
#include "channel.h"

#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define BENCH_REQUIRE(expression)                                                                  \
    do {                                                                                           \
        if (!(expression)) {                                                                       \
            std::cerr << "benchmark setup failed: " #expression " at " << __FILE__ << ":"          \
                      << __LINE__ << '\n';                                                         \
            std::exit(EXIT_FAILURE);                                                               \
        }                                                                                          \
    } while (false)

#define BENCH_REQUIRE_FALSE(expression) BENCH_REQUIRE(!(expression))

namespace {

using namespace fleet;

constexpr size_t FRAME_COUNT = MAX_FRAMES_PER_PACKET;
constexpr size_t OPS_PER_SAMPLE = 5000;
constexpr uint64_t BENCH_WARMUP_ITERS = 2;
constexpr uint64_t BENCH_MIN_EPOCH_ITERS = 10;
constexpr auto BENCH_MIN_EPOCH_TIME = std::chrono::milliseconds{20};
constexpr uint32_t BENCH_SEND_QUEUE_SLOTS = 65'536;
constexpr uint16_t UDP_BENCH_REMOTE_PORT = 58437;

void require_frame(std::vector<can_frame> &frames, result<can_frame> frame) {
    BENCH_REQUIRE(frame);
    frames.push_back(*frame);
}

struct DecodeFeedbackContext {
    size_t parsed = 0;
    bool ok = true;
    std::vector<feedback> *feedbacks = nullptr;
};

void parse_feedback_frame(void *context, const can_frame &frame) {
    auto &decode = *static_cast<DecodeFeedbackContext *>(context);
    auto response = can_codec::parse_feedback(frame);
    if (!response) {
        decode.ok = false;
        return;
    }
    ++decode.parsed;
    if (decode.feedbacks != nullptr) {
        decode.feedbacks->push_back(std::move(*response));
    }
}

can_frame make_status_frame(uint32_t id, uint8_t port, uint16_t seed) {
    can_frame frame;
    frame.id = id;
    frame.port = port;
    frame.len = 8;

    BENCH_REQUIRE_FALSE(write<can_field::feedback1::Type>(frame, uint8_t{1}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback1::Error>(frame, uint8_t{0}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback1::Position>(frame, static_cast<uint16_t>(0x1000U + seed)));
    BENCH_REQUIRE_FALSE(write<can_field::feedback1::Velocity>(frame, static_cast<uint16_t>(0x300U + (seed & 0xFFU))));
    BENCH_REQUIRE_FALSE(write<can_field::feedback1::Current>(frame, static_cast<uint16_t>(0x200U + (seed & 0xFFU))));
    BENCH_REQUIRE_FALSE(write<can_field::feedback1::MotorTemp>(frame, static_cast<uint8_t>(80U + (seed & 0x0FU))));
    BENCH_REQUIRE_FALSE(write<can_field::feedback1::MosfetTemp>(frame, static_cast<uint8_t>(90U + (seed & 0x0FU))));
    return frame;
}

can_frame make_position_feedback_frame(uint32_t id, uint8_t port, uint16_t seed) {
    can_frame frame;
    frame.id = id;
    frame.port = port;
    frame.len = 8;

    BENCH_REQUIRE_FALSE(write<can_field::feedback2::Type>(frame, uint8_t{2}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback2::Error>(frame, uint8_t{0}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback2::PositionDeg>(frame, f32_to_bits(10.0F + seed)));
    BENCH_REQUIRE_FALSE(write<can_field::feedback2::CurrentX100>(frame, static_cast<uint16_t>(100 + seed)));
    BENCH_REQUIRE_FALSE(write<can_field::feedback2::Temp>(frame, static_cast<uint8_t>(85 + (seed & 0x0FU))));
    return frame;
}

can_frame make_velocity_feedback_frame(uint32_t id, uint8_t port, uint16_t seed) {
    can_frame frame;
    frame.id = id;
    frame.port = port;
    frame.len = 8;

    BENCH_REQUIRE_FALSE(write<can_field::feedback3::Type>(frame, uint8_t{3}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback3::Error>(frame, uint8_t{0}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback3::Rpm>(frame, f32_to_bits(120.0F + seed)));
    BENCH_REQUIRE_FALSE(write<can_field::feedback3::CurrentX100>(frame, static_cast<uint16_t>(120 + seed)));
    BENCH_REQUIRE_FALSE(write<can_field::feedback3::Temp>(frame, static_cast<uint8_t>(88 + (seed & 0x0FU))));
    return frame;
}

can_frame make_config_ack_feedback_frame(uint32_t id, uint8_t port) {
    can_frame frame;
    frame.id = id;
    frame.port = port;
    frame.len = 3;

    BENCH_REQUIRE_FALSE(write<can_field::feedback_config::Type>(frame, uint8_t{4}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback_config::Error>(frame, uint8_t{0}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback_config::Key>(frame, enum_u8(config_set_code::accel)));
    BENCH_REQUIRE_FALSE(write<can_field::feedback_config::Status>(frame, uint8_t{1}));
    return frame;
}

can_frame make_query_feedback_frame(uint32_t id, uint8_t port, uint16_t seed) {
    can_frame frame;
    frame.id = id;
    frame.port = port;
    frame.len = 6;

    BENCH_REQUIRE_FALSE(write<can_field::feedback_query::Type>(frame, uint8_t{5}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback_query::Error>(frame, uint8_t{0}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback_query::Key>(frame, enum_u8(config_query_code::position)));
    const auto position = f32_to_bits(30.0F + seed);
    frame.data[2] = static_cast<uint8_t>(position >> 24U);
    frame.data[3] = static_cast<uint8_t>(position >> 16U);
    frame.data[4] = static_cast<uint8_t>(position >> 8U);
    frame.data[5] = static_cast<uint8_t>(position);
    return frame;
}

can_frame make_brake_feedback_frame(uint32_t id, uint8_t port, uint16_t seed) {
    can_frame frame;
    frame.id = id;
    frame.port = port;
    frame.len = 2;

    BENCH_REQUIRE_FALSE(write<can_field::feedback_brake::Type>(frame, uint8_t{6}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback_brake::Error>(frame, uint8_t{0}));
    BENCH_REQUIRE_FALSE(write<can_field::feedback_brake::Engaged>(frame, static_cast<uint8_t>(seed & 1U)));
    return frame;
}

std::vector<can_frame> make_response_frames() {
    std::vector<can_frame> frames;
    frames.reserve(FRAME_COUNT);
    for (size_t i = 0; i < FRAME_COUNT; ++i) {
        const auto id = static_cast<uint32_t>(0x100U + i);
        const auto seed = static_cast<uint16_t>(i);
        switch (i % 6U) {
        case 0:
            frames.push_back(make_status_frame(id, 0, seed));
            break;
        case 1:
            frames.push_back(make_position_feedback_frame(id, 0, seed));
            break;
        case 2:
            frames.push_back(make_velocity_feedback_frame(id, 0, seed));
            break;
        case 3:
            frames.push_back(make_config_ack_feedback_frame(id, 0));
            break;
        case 4:
            frames.push_back(make_query_feedback_frame(id, 0, seed));
            break;
        default:
            frames.push_back(make_brake_feedback_frame(id, 0, seed));
            break;
        }
    }
    return frames;
}

bytes make_response_wire() {
    auto frames = make_response_frames();
    bytes wire(motor_codec::MAX_PACKET_SZ);
    auto size = motor_codec::encode_into(bytes_mut(wire), TYPE_CAN2UDP, frames);
    BENCH_REQUIRE(size);
    wire.resize(*size);
    return wire;
}

std::vector<feedback> make_response_feedbacks(const bytes &bytes) {
    std::vector<feedback> feedbacks;
    feedbacks.reserve(FRAME_COUNT);

    DecodeFeedbackContext context{.feedbacks = &feedbacks};
    BENCH_REQUIRE_FALSE(motor_codec::decode_each(bytes, &context, &parse_feedback_frame));
    BENCH_REQUIRE(context.ok);
    return feedbacks;
}

std::vector<can_frame> make_command_frames() {
    std::vector<can_frame> frames;
    frames.reserve(FRAME_COUNT);

    for (size_t i = 0; i < FRAME_COUNT; ++i) {
        const can_addr_t address{
            .can_port = static_cast<can_port_t>(i % 2U),
            .can_id = static_cast<can_id_t>(1U + i),
        };

        switch (i % 8U) {
        case 0:
            require_frame(frames, can_codec::make_pvt(
                                      address,
                                      pvt_command{
                                          .gains = {.kp = 125.0F, .kd = 2.5F},
                                          .position_r = 1.0F,
                                          .velocity_radps = -2.0F,
                                          .torque_ff_nm = 3.0F,
                                      }));
            break;
        case 1:
            require_frame(frames, can_codec::make_position(
                                      address,
                                      position_command{
                                          .position_deg = 10.0F + static_cast<float>(i),
                                          .velocity_limit_rpm = 120.0F,
                                          .current_limit = 5.0F,
                                      }));
            break;
        case 2:
            require_frame(frames, can_codec::make_velocity(
                                      address,
                                      velocity_command{
                                          .velocity_rpm = 60.0F + static_cast<float>(i),
                                          .current_limit = 4.0F,
                                      }));
            break;
        case 3:
            require_frame(frames, can_codec::make_torque(
                                      address,
                                      torque_command{
                                          .torque_nm = 2.0F + static_cast<float>(i % 4U),
                                      }));
            break;
        case 4:
            require_frame(frames, can_codec::make_current(
                                      address,
                                      current_command{
                                          .current = 1.0F + static_cast<float>(i % 4U),
                                      }));
            break;
        case 5:
            require_frame(frames, can_codec::make_brake(
                                      address,
                                      brake_command{
                                          .control = engage_brake{.open = true},
                                          .reply = reply_mode::status1,
                                      }));
            break;
        case 6:
            require_frame(frames, can_codec::make_config(
                                      address,
                                      config_set_command{
                                          .code = config_set_code::can_timeout,
                                          .value = 1000ms,
                                      }));
            break;
        default:
            require_frame(frames, can_codec::make_config_query(
                                      address,
                                      config_query_command{.code = config_query_code::position}));
            break;
        }
    }
    return frames;
}

std::unique_ptr<motor_driver> make_send_driver() {
    auto channel = std::make_unique<realtime_udp_channel>(
        "encos_send_bench",
        udp_channel_options{
            .remote_address = "127.0.0.1",
            .remote_port = UDP_BENCH_REMOTE_PORT,
            .local_port = 0,
            .max_bandwidth_bps = std::numeric_limits<size_t>::max(),
            .send_queue_slots = BENCH_SEND_QUEUE_SLOTS,
        });
    channel->start();
    return std::make_unique<motor_driver>(std::move(channel));
}

std::vector<std::unique_ptr<motor_device>> make_benchmark_motors() {
    std::vector<std::unique_ptr<motor_device>> motors;
    motors.reserve(FRAME_COUNT);

    for (size_t i = 0; i < FRAME_COUNT; ++i) {
        fleet_config::motor_config config{
            .name = "bench_motor_" + std::to_string(i),
            .channel = "encos_receive_bench",
            .model = "EC-A2806-P2-36",
            .can_port = 0,
            .can_id = static_cast<can_id_t>(0x100U + i),
        };
        motors.push_back(std::make_unique<motor_device>(static_cast<device_id_t>(i + 1U), config));
    }
    return motors;
}

std::unique_ptr<motor_driver> make_receive_driver(std::span<const std::unique_ptr<motor_device>> motors) {
    auto driver = std::make_unique<motor_driver>();
    for (const auto &motor : motors) {
        driver->bind_motor(*motor);
    }
    return driver;
}

size_t serialize_command_frames(std::span<const can_frame> frames) {
    bytes bytes(motor_codec::MAX_PACKET_SZ);
    auto size = motor_codec::encode_into(bytes_mut(bytes), TYPE_UDP2CAN, frames);
    if (!size) {
        return 0;
    }
    return *size;
}

size_t deserialize_full_udp_packet(const bytes &bytes) {
    DecodeFeedbackContext context;
    if (motor_codec::decode_each(bytes, &context, &parse_feedback_frame)) {
        return 0;
    }
    return context.ok ? context.parsed : 0;
}

size_t serialize_many_udp_packets(std::span<const can_frame> frames, size_t count) {
    size_t total = 0;
    for (size_t i = 0; i < count; ++i) {
        total += serialize_command_frames(frames);
    }
    return total;
}

size_t deserialize_many_udp_packets(const bytes &bytes, size_t count) {
    size_t total = 0;
    for (size_t i = 0; i < count; ++i) {
        total += deserialize_full_udp_packet(bytes);
    }
    return total;
}

size_t motor_send_many(motor_driver &driver, std::span<const can_frame> frames, size_t count) {
    size_t sent = 0;
    for (size_t i = 0; i < count; ++i) {
        if (driver.send(frames)) {
            return sent;
        }
        sent += frames.size();
    }
    return sent;
}

size_t motor_recieve_many(motor_driver &driver, bytes_view wire, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        driver.benchmark_handle_packet(wire);
    }
    return count * FRAME_COUNT;
}

size_t apply_device_state_many(std::span<const std::unique_ptr<motor_device>> motors,
                               std::span<const feedback> feedbacks, size_t count) {
    size_t applied = 0;
    for (size_t i = 0; i < count; ++i) {
        for (size_t frame = 0; frame < feedbacks.size(); ++frame) {
            applied += motors[frame % motors.size()]->apply(feedbacks[frame]) ? 1U : 0U;
        }
    }
    return applied;
}

} // namespace

int main() {
    auto send_driver = make_send_driver();
    const auto command_frames = make_command_frames();

    auto motors = make_benchmark_motors();
    auto receive_driver = make_receive_driver(std::span<const std::unique_ptr<motor_device>>(motors));
    const auto response_wire = make_response_wire();
    const auto response_feedbacks = make_response_feedbacks(response_wire);

    BENCH_REQUIRE(serialize_command_frames(command_frames) > 0);
    BENCH_REQUIRE(motor_send_many(*send_driver, command_frames, 4) == FRAME_COUNT * 4);
    BENCH_REQUIRE(deserialize_full_udp_packet(response_wire) == FRAME_COUNT);
    BENCH_REQUIRE(motor_recieve_many(*receive_driver, response_wire, 4) == FRAME_COUNT * 4);
    BENCH_REQUIRE(apply_device_state_many(motors, response_feedbacks, 4) > 0);
    BENCH_REQUIRE(motors.front()->position_deg());

    ankerl::nanobench::Bench bench;
    bench.title("encos driver CPU path performance")
        .relative(true)
        .warmup(BENCH_WARMUP_ITERS)
        .minEpochIterations(BENCH_MIN_EPOCH_ITERS)
        .minEpochTime(BENCH_MIN_EPOCH_TIME);

    bench.run("send baseline: serialize mixed 32 CAN frames x5000", [&] {
        const auto bytes = serialize_many_udp_packets(command_frames, OPS_PER_SAMPLE);
        ankerl::nanobench::doNotOptimizeAway(bytes);
    });

    bench.run("send full: encos_driver mixed encode, queue, coroutine, UDP send x5000", [&] {
        send_driver->discard_pending_sends();
        const auto sent = motor_send_many(*send_driver, command_frames, OPS_PER_SAMPLE);
        BENCH_REQUIRE(sent == FRAME_COUNT * OPS_PER_SAMPLE);
        ankerl::nanobench::doNotOptimizeAway(sent);
    });

    bench.run("receive baseline: deserialize and parse mixed 32 CAN frames x5000", [&] {
        const auto parsed = deserialize_many_udp_packets(response_wire, OPS_PER_SAMPLE);
        ankerl::nanobench::doNotOptimizeAway(parsed);
    });

    bench.run("receive full: encos_driver mixed decode, route, apply 32 CAN frames x5000", [&] {
        const auto received = motor_recieve_many(*receive_driver, response_wire, OPS_PER_SAMPLE);
        ankerl::nanobench::doNotOptimizeAway(received);
    });

    bench.run("device state: apply mixed 32 CAN feedbacks x5000", [&] {
        const auto applied = apply_device_state_many(motors, response_feedbacks, OPS_PER_SAMPLE);
        ankerl::nanobench::doNotOptimizeAway(applied);
    });

    return EXIT_SUCCESS;
}
