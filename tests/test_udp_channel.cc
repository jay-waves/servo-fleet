#include <doctest/doctest.h>

#include "channel.h"

#include <asio.hpp>

#include <algorithm>
#include <array>
#include <condition_variable>
#include <mutex>
#include <vector>

namespace {

using asio::ip::udp;
using namespace fleet;
using namespace std::chrono_literals;

uint16_t reserve_loopback_port() {
    asio::io_context io;
    udp::socket socket(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));
    return socket.local_endpoint().port();
}

struct ReceiveState {
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<bytes> received;
    bool called = false;
};

void collect_packets(realtime_udp_channel &transport, ReceiveState &state) {
    transport.set_rx_callback([&state](bytes_view packet) {
        {
            std::lock_guard lock(state.mutex);
            state.called = true;
            state.received.emplace_back(packet.begin(), packet.end());
        }
        state.cv.notify_one();
    });
}

bytes single_byte_packet(size_t value) {
    return bytes{static_cast<uint8_t>(value)};
}

} // namespace

TEST_CASE("async udp transport sends bytes to the configured endpoint") {
    asio::io_context io;
    udp::socket receiver(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));
    const auto receiver_port = receiver.local_endpoint().port();

    realtime_udp_channel transport({
        .remote_address = "127.0.0.1",
        .remote_port = receiver_port,
        .local_port = 0,
    });

    const bytes outbound{0xAA, 0x01, 0x02, 0xBB};
    transport.start();
    REQUIRE_FALSE(transport.send(outbound));

    std::array<uint8_t, 16> buffer{};
    udp::endpoint sender;
    asio::error_code err;
    const auto size = receiver.receive_from(asio::buffer(buffer), sender, 0, err);

    REQUIRE_FALSE(err);
    REQUIRE(size == outbound.size());
    CHECK(std::equal(outbound.begin(), outbound.end(), buffer.begin()));
    transport.stop();
}

TEST_CASE("queued udp transport rejects new packets when full") {
    constexpr uint32_t SEND_QUEUE_SLOTS = 3;

    realtime_udp_channel transport({
        .remote_address = "127.0.0.1",
        .remote_port = reserve_loopback_port(),
        .local_port = 0,
        .send_queue_slots = SEND_QUEUE_SLOTS,
    });

    for (size_t index = 0; index < SEND_QUEUE_SLOTS; ++index) {
        const auto packet = single_byte_packet(index);
        REQUIRE_FALSE(transport.send(packet));
    }
    const bytes extra{0xFF};
    CHECK(transport.send(extra) == fleet_err::busy);
    transport.stop();
}

TEST_CASE("queued udp transport can discard pending sends") {
    constexpr uint32_t SEND_QUEUE_SLOTS = 2;

    realtime_udp_channel transport({
        .remote_address = "127.0.0.1",
        .remote_port = reserve_loopback_port(),
        .local_port = 0,
        .send_queue_slots = SEND_QUEUE_SLOTS,
    });

    REQUIRE_FALSE(transport.send(bytes{0x01}));
    REQUIRE_FALSE(transport.send(bytes{0x02}));
    CHECK(transport.send(bytes{0x03}) == fleet_err::busy);

    transport.discard_pending_sends();
    CHECK_FALSE(transport.send(bytes{0x04}));
    transport.stop();
}

TEST_CASE("async udp transport rejects sends after stop") {
    realtime_udp_channel transport({
        .remote_address = "127.0.0.1",
        .remote_port = reserve_loopback_port(),
        .local_port = 0,
    });

    transport.stop();
    CHECK(transport.send(bytes{0x01}) == fleet_err::cancelled);
    auto packet = transport.prepare_send();
    REQUIRE_FALSE(packet);
    CHECK(packet.error() == fleet_err::cancelled);
}

TEST_CASE("async udp transport limits payload bandwidth with a bounded burst") {
    constexpr size_t PACKET_SIZE = 512;

    asio::io_context io;
    udp::socket receiver(io, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0));
    realtime_udp_channel transport({
        .remote_address = "127.0.0.1",
        .remote_port = receiver.local_endpoint().port(),
        .local_port = 0,
        .max_bandwidth_bps = PACKET_SIZE * 8 * 100,
    });
    const bytes packet(PACKET_SIZE, 0x42);

    REQUIRE_FALSE(transport.send(packet));
    REQUIRE_FALSE(transport.send(packet));
    REQUIRE_FALSE(transport.send(packet));
    transport.start();

    std::array<uint8_t, PACKET_SIZE> buffer{};
    udp::endpoint sender;
    std::array<std::chrono::steady_clock::time_point, 3> received_at;
    for (auto &time : received_at) {
        receiver.receive_from(asio::buffer(buffer), sender);
        time = std::chrono::steady_clock::now();
    }

    const auto minimum_wait = std::chrono::duration<double>(
        static_cast<double>(PACKET_SIZE * 8) / (PACKET_SIZE * 8 * 100));
    CHECK(received_at[2] - received_at[1] >= minimum_wait / 2);
    transport.stop();
}

TEST_CASE("async udp transport receives complete datagrams without merging adjacent sends") {
    const auto local_port = reserve_loopback_port();
    realtime_udp_channel transport({
        .remote_address = "127.0.0.1",
        .remote_port = 0,
        .local_port = local_port,
    });
    ReceiveState state;
    collect_packets(transport, state);
    transport.start();

    asio::io_context io;
    udp::socket sender(io, udp::endpoint(udp::v4(), 0));
    const udp::endpoint target(asio::ip::make_address("127.0.0.1"), local_port);
    const bytes first{0x01, 0x02};
    const bytes second{0x03, 0x04, 0x05};

    sender.send_to(asio::buffer(first), target);
    sender.send_to(asio::buffer(second), target);

    {
        std::unique_lock lock(state.mutex);
        REQUIRE(state.cv.wait_for(lock, 200ms, [&] { return state.received.size() == 2; }));
        CHECK(state.received[0] == first);
        CHECK(state.received[1] == second);
    }
    transport.stop();
}

TEST_CASE("async udp transport receive can wait without a datagram") {
    const auto local_port = reserve_loopback_port();
    realtime_udp_channel transport({
        .remote_address = "127.0.0.1",
        .remote_port = 0,
        .local_port = local_port,
    });
    ReceiveState state;
    collect_packets(transport, state);
    transport.start();

    {
        std::unique_lock lock(state.mutex);
        CHECK_FALSE(state.cv.wait_for(lock, 10ms, [&] { return state.called; }));
    }

    transport.stop();
}

TEST_CASE("udp channel dispatches received packets through callback") {
    const auto local_port = reserve_loopback_port();
    asio::io_context io;
    udp::socket sender(io, udp::endpoint(udp::v4(), 0));
    udp_channel transport({
        .remote_address = "127.0.0.1",
        .remote_port = sender.local_endpoint().port(),
        .local_port = local_port,
    });
    ReceiveState state;
    transport.set_rx_callback([&state](bytes_view packet) {
        {
            std::lock_guard lock(state.mutex);
            state.called = true;
            state.received.emplace_back(packet.begin(), packet.end());
        }
        state.cv.notify_one();
    });

    const bytes outbound{0x10, 0x20, 0x30};
    sender.send_to(asio::buffer(outbound), udp::endpoint(asio::ip::make_address("127.0.0.1"), local_port));

    {
        std::unique_lock lock(state.mutex);
        REQUIRE(state.cv.wait_for(lock, 200ms, [&] { return state.received.size() == 1; }));
        CHECK(state.received.front() == outbound);
    }
    transport.stop();
}

TEST_CASE("udp channel can wait without a datagram") {
    udp_channel transport({
        .remote_address = "127.0.0.1",
        .remote_port = reserve_loopback_port(),
        .local_port = reserve_loopback_port(),
    });
    ReceiveState state;
    transport.set_rx_callback([&state](bytes_view) {
        {
            std::lock_guard lock(state.mutex);
            state.called = true;
        }
        state.cv.notify_one();
    });

    {
        std::unique_lock lock(state.mutex);
        CHECK_FALSE(state.cv.wait_for(lock, 10ms, [&] { return state.called; }));
    }
    transport.stop();
}
