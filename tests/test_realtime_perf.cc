#include <doctest/doctest.h>

#include "channel.h"
#include "codec_motor.h"

#include <asio.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <numeric>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace {

using asio::ip::udp;
using namespace fleet;
using namespace std::chrono_literals;

using clock = std::chrono::steady_clock;
using time_point = clock::time_point;

constexpr size_t FRAME_COUNT = MAX_FRAMES_PER_PACKET;
constexpr size_t LOCAL_ITERATIONS = 20'000;
constexpr size_t NETWORK_WARMUP = 100;
constexpr size_t NETWORK_ITERATIONS = 2'000;
constexpr auto RESPONSE_TIMEOUT = 100ms;
constexpr std::string_view CAN_PORTS_ENV = "FLEET_REALTIME_CAN_PORTS";

// A 1 kHz motor-control loop has a 1 ms end-to-end cycle budget. Local codec
// work is limited to one tenth of that budget, leaving time for control logic.
constexpr double MAX_CODEC_MEAN_US = 100.0;
constexpr double MAX_CODEC_P99_US = 100.0;
constexpr double MIN_CODEC_ROUND_TRIPS_PER_SEC = 10'000.0;

// Loopback includes two UdpLink paths, thread scheduling, peer decode/encode,
// and the response. The tail limit is stricter than the per-sample timeout.
constexpr double MIN_CONTROL_CYCLES_PER_SEC = 1'000.0;
constexpr double MAX_RTT_P99_US = 1'000.0;
constexpr double MAX_RTT_US = 10'000.0;
constexpr double MAX_ONE_WAY_P99_US = 500.0;
constexpr double MAX_PEER_PROCESS_P99_US = 150.0;

constexpr size_t BACKPRESSURE_PACKET_SIZE = 128;
constexpr size_t BACKPRESSURE_PACKET_COUNT = 24;
constexpr size_t BACKPRESSURE_BANDWIDTH_BPS = BACKPRESSURE_PACKET_SIZE * 8 * 100;
constexpr auto BACKPRESSURE_TIMEOUT = 2s;

double elapsed_us(time_point start, time_point end) {
    return std::chrono::duration<double, std::micro>(end - start).count();
}

struct LatencyStats {
    double mean_us = 0.0;
    double p50_us = 0.0;
    double p95_us = 0.0;
    double p99_us = 0.0;
    double max_us = 0.0;
};

double percentile(const std::vector<double> &sorted, double ratio) {
    const auto index =
        static_cast<size_t>(std::ceil(ratio * static_cast<double>(sorted.size()))) - 1U;
    return sorted[std::min(index, sorted.size() - 1U)];
}

LatencyStats summarize(std::vector<double> samples) {
    REQUIRE_FALSE(samples.empty());
    std::sort(samples.begin(), samples.end());
    const auto sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    return {
        .mean_us = sum / static_cast<double>(samples.size()),
        .p50_us = percentile(samples, 0.50),
        .p95_us = percentile(samples, 0.95),
        .p99_us = percentile(samples, 0.99),
        .max_us = samples.back(),
    };
}

void report_stats(const char *name, const LatencyStats &stats) {
    INFO(name << " mean=" << stats.mean_us << "us"
                       << " p50=" << stats.p50_us << "us"
                       << " p95=" << stats.p95_us << "us"
                       << " p99=" << stats.p99_us << "us"
                       << " max=" << stats.max_us << "us");
}

result<std::vector<can_port_t>> configured_can_ports() {
    const char *value = std::getenv(CAN_PORTS_ENV.data());
    if (value == nullptr || *value == '\0') {
        return std::vector<can_port_t>{0, 1, 2, 3};
    }

    std::vector<can_port_t> ports;
    std::string_view remaining(value);
    while (!remaining.empty()) {
        const auto separator = remaining.find(',');
        auto token = remaining.substr(0, separator);
        const auto first = token.find_first_not_of(" \t");
        const auto last = token.find_last_not_of(" \t");
        if (first == std::string_view::npos) {
            return fail(fleet_err::invalid_argument);
        }
        token = token.substr(first, last - first + 1U);

        unsigned int port = 0;
        const auto [end, error] = std::from_chars(token.data(), token.data() + token.size(), port);
        if (error != std::errc{} || end != token.data() + token.size() || port > UINT8_MAX) {
            return fail(fleet_err::invalid_argument);
        }

        const auto port2 = static_cast<can_port_t>(port);
        if (std::find(ports.begin(), ports.end(), port2) != ports.end()) {
            return fail(fleet_err::invalid_argument);
        }
        ports.push_back(port2);

        if (separator == std::string_view::npos) {
            break;
        }
        remaining.remove_prefix(separator + 1U);
    }

    if (ports.empty()) {
        return fail(fleet_err::invalid_argument);
    }
    return ports;
}

void report_can_ports(std::span<const can_port_t> ports) {
    std::string configured;
    for (const auto port : ports) {
        if (!configured.empty()) {
            configured += ",";
        }
        configured += std::to_string(static_cast<unsigned int>(port));
    }
    INFO("CAN ports=" << configured);
}

std::vector<can_frame> make_full_packet(std::span<const can_port_t> ports, uint32_t sequence = 0) {
    std::vector<can_frame> packet;
    packet.reserve(FRAME_COUNT);

    for (size_t i = 0; i < FRAME_COUNT; ++i) {
        can_frame frame;
        frame.port = ports[i % ports.size()];
        frame.id = static_cast<uint32_t>(0x100U + i);
        frame.len = MAX_CAN_DATA_LEN;
        for (size_t byte = 0; byte < frame.data.size(); ++byte) {
            frame.data[byte] = static_cast<uint8_t>((i * 17U + byte) & 0xFFU);
        }
        packet.push_back(frame);
    }

    for (size_t byte = 0; byte < sizeof(sequence); ++byte) {
        packet.front().data[byte] = static_cast<uint8_t>(sequence >> (byte * 8U));
    }
    return packet;
}

uint32_t packet_sequence(std::span<const can_frame> packet) {
    uint32_t sequence = 0;
    for (size_t byte = 0; byte < sizeof(sequence); ++byte) {
        sequence |= static_cast<uint32_t>(packet.front().data[byte]) << (byte * 8U);
    }
    return sequence;
}

void collect_frame(void *context, const can_frame &frame) {
    static_cast<std::vector<can_frame> *>(context)->push_back(frame);
}

bool packet_uses_ports(std::span<const can_frame> packet, std::span<const can_port_t> ports) {
    if (packet.size() != FRAME_COUNT || ports.empty()) {
        return false;
    }
    for (size_t i = 0; i < packet.size(); ++i) {
        if (packet[i].port != ports[i % ports.size()]) {
            return false;
        }
    }
    return true;
}

std::pair<uint16_t, uint16_t> reserve_loopback_ports() {
    asio::io_context io;
    udp::socket first(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));
    udp::socket second(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));
    return {first.local_endpoint().port(), second.local_endpoint().port()};
}

} // namespace

TEST_CASE("full motor packets meet local codec latency budget") {
    const auto ports = configured_can_ports();
    REQUIRE(ports);
    report_can_ports(*ports);

    const auto request = make_full_packet(std::span<const can_port_t>(*ports));
    bytes wire(motor_codec::MAX_PACKET_SZ);
    auto encoded_size = motor_codec::encode_into(bytes_mut(wire), TYPE_UDP2CAN, request);
    REQUIRE(encoded_size);
    wire.resize(*encoded_size);

    std::vector<double> encode_us;
    std::vector<double> decode_us;
    encode_us.reserve(LOCAL_ITERATIONS);
    decode_us.reserve(LOCAL_ITERATIONS);
    size_t decoded_frames = 0;
    bool codec_ok = true;

    const auto stress_started_at = clock::now();
    for (size_t i = 0; i < LOCAL_ITERATIONS; ++i) {
        const auto encode_started_at = clock::now();
        const auto encode_result = motor_codec::encode_into(bytes_mut(wire), TYPE_UDP2CAN, request);
        const auto encode_finished_at = clock::now();
        if (!encode_result) {
            codec_ok = false;
            break;
        }

        std::vector<can_frame> decoded;
        decoded.reserve(FRAME_COUNT);
        const auto decode_error =
            motor_codec::decode_each(bytes_view(wire).first(*encode_result), &decoded, &collect_frame);
        const auto decode_finished_at = clock::now();
        if (decode_error || !packet_uses_ports(decoded, std::span<const can_port_t>(*ports))) {
            codec_ok = false;
            break;
        }
        decoded_frames += decoded.size();

        encode_us.push_back(elapsed_us(encode_started_at, encode_finished_at));
        decode_us.push_back(elapsed_us(encode_finished_at, decode_finished_at));
    }
    const auto stress_finished_at = clock::now();

    REQUIRE(codec_ok);
    REQUIRE(encode_us.size() == LOCAL_ITERATIONS);
    REQUIRE(decode_us.size() == LOCAL_ITERATIONS);
    const auto encode = summarize(std::move(encode_us));
    const auto decode = summarize(std::move(decode_us));
    const auto round_trips_per_sec =
        static_cast<double>(LOCAL_ITERATIONS) /
        std::chrono::duration<double>(stress_finished_at - stress_started_at).count();

    report_stats("serialize", encode);
    report_stats("deserialize", decode);
    INFO("codec round trips=" << round_trips_per_sec << "/s");

    REQUIRE(decoded_frames == LOCAL_ITERATIONS * FRAME_COUNT);
    CHECK(encode.mean_us <= MAX_CODEC_MEAN_US);
    CHECK(encode.p99_us <= MAX_CODEC_P99_US);
    CHECK(decode.mean_us <= MAX_CODEC_MEAN_US);
    CHECK(decode.p99_us <= MAX_CODEC_P99_US);
    CHECK(round_trips_per_sec >= MIN_CODEC_ROUND_TRIPS_PER_SEC);
}

// TODO: Rebuild the bidirectional UDP realtime benchmark on top of the callback receive API.

TEST_CASE("small queued burst has stable bandwidth and bounded latency") {
    asio::io_context io;
    udp::socket receiver(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));
    receiver.non_blocking(true);

    realtime_udp_channel channel({
        .remote_address = "127.0.0.1",
        .remote_port = receiver.local_endpoint().port(),
        .local_port = 0,
        .max_bandwidth_bps = BACKPRESSURE_BANDWIDTH_BPS,
    });

    bytes packet(BACKPRESSURE_PACKET_SIZE, 0x42);
    const auto submitted_at = clock::now();
    for (size_t index = 0; index < BACKPRESSURE_PACKET_COUNT; ++index) {
        packet[0] = static_cast<uint8_t>(index);
        REQUIRE_FALSE(channel.send(packet));
    }
    channel.start();

    std::array<uint8_t, BACKPRESSURE_PACKET_SIZE> buffer{};
    std::vector<time_point> received_at;
    received_at.reserve(BACKPRESSURE_PACKET_COUNT);
    const auto deadline = clock::now() + BACKPRESSURE_TIMEOUT;
    while (received_at.size() < BACKPRESSURE_PACKET_COUNT && clock::now() < deadline) {
        udp::endpoint sender;
        asio::error_code error;
        const auto size = receiver.receive_from(asio::buffer(buffer), sender, 0, error);
        if (!error) {
            REQUIRE(size == BACKPRESSURE_PACKET_SIZE);
            REQUIRE(buffer[0] == static_cast<uint8_t>(received_at.size()));
            received_at.push_back(clock::now());
        } else if (error == asio::error::would_block || error == asio::error::try_again) {
            std::this_thread::sleep_for(1ms);
        } else {
            FAIL("UDP receive failed: " << error.message());
        }
    }
    channel.stop();

    REQUIRE(received_at.size() == BACKPRESSURE_PACKET_COUNT);

    constexpr size_t EXPECTED_BURST_BYTES = 512 * 2;
    constexpr size_t burst_packets = EXPECTED_BURST_BYTES / BACKPRESSURE_PACKET_SIZE;
    static_assert(burst_packets > 0 && burst_packets < BACKPRESSURE_PACKET_COUNT);

    std::vector<double> paced_intervals_us;
    paced_intervals_us.reserve(BACKPRESSURE_PACKET_COUNT - burst_packets);
    for (size_t index = burst_packets; index < received_at.size(); ++index) {
        paced_intervals_us.push_back(elapsed_us(received_at[index - 1], received_at[index]));
    }

    std::vector<double> queue_latency_us;
    queue_latency_us.reserve(received_at.size());
    for (const auto received : received_at) {
        queue_latency_us.push_back(elapsed_us(submitted_at, received));
    }

    const auto interval = summarize(std::move(paced_intervals_us));
    const auto latency = summarize(std::move(queue_latency_us));
    const auto paced_seconds =
        std::chrono::duration<double>(received_at.back() - received_at[burst_packets - 1]).count();
    const auto paced_bits =
        static_cast<double>((BACKPRESSURE_PACKET_COUNT - burst_packets) *
                            BACKPRESSURE_PACKET_SIZE * 8);
    const auto actual_bandwidth_bps = paced_bits / paced_seconds;
    const auto bandwidth_ratio =
        actual_bandwidth_bps / static_cast<double>(BACKPRESSURE_BANDWIDTH_BPS);

    report_stats("backpressure interval", interval);
    report_stats("backpressure queue latency", latency);
    INFO("target bandwidth=" << BACKPRESSURE_BANDWIDTH_BPS
                                      << "bps actual=" << actual_bandwidth_bps
                                      << "bps ratio=" << bandwidth_ratio);

    CHECK(bandwidth_ratio >= 0.80);
    CHECK(bandwidth_ratio <= 1.15);
    CHECK(interval.mean_us >= 8'000.0);
    CHECK(interval.mean_us <= 12'000.0);
    CHECK(interval.p99_us <= 50'000.0);
    CHECK(latency.max_us >= 120'000.0);
    CHECK(latency.max_us <= 500'000.0);
}
