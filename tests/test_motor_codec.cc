#include <doctest/doctest.h>

#include "codec_motor.h"
#include "fleet/fleet.h"

#include <algorithm>
#include <array>
#include <initializer_list>
#include <unordered_map>

namespace {

fleet::can_frame make_frame(uint8_t port, uint32_t id, std::initializer_list<uint8_t> data) {
    fleet::can_frame frame;
    frame.port = port;
    frame.id = id;
    frame.len = static_cast<std::uint8_t>(data.size());
    std::copy(data.begin(), data.end(), frame.data.begin());
    return frame;
}

fleet::can_frame make_extended_frame(uint8_t port, uint32_t id, std::initializer_list<uint8_t> data) {
    auto frame = make_frame(port, id, data);
    frame.extended = true;
    return frame;
}

template <size_t N>
void check_entry(const fleet::bytes &bytes, size_t entry_offset, uint8_t port, uint32_t can_id,
                 std::uint8_t len, const std::array<uint8_t, N> &data) {
    REQUIRE(bytes.size() >= entry_offset + fleet::motor_codec::CAN_ENTRY_SZ);
    CHECK(bytes[entry_offset] == port);
    CHECK(bytes[entry_offset + 1] == static_cast<uint8_t>((can_id >> 24U) & 0xFFU));
    CHECK(bytes[entry_offset + 2] == static_cast<uint8_t>((can_id >> 16U) & 0xFFU));
    CHECK(bytes[entry_offset + 3] == static_cast<uint8_t>((can_id >> 8U) & 0xFFU));
    CHECK(bytes[entry_offset + 4] == static_cast<uint8_t>(can_id & 0xFFU));
    CHECK(bytes[entry_offset + 5] == len);
    for (size_t i = 0; i < fleet::MAX_CAN_DATA_LEN; ++i) {
        const auto expected = i < data.size() ? data[i] : uint8_t{0};
        CHECK(bytes[entry_offset + 6 + i] == expected);
    }
}

void collect_frame(void *context, const fleet::can_frame &frame) {
    static_cast<std::vector<fleet::can_frame> *>(context)->push_back(frame);
}

void count_frame(void *context, const fleet::can_frame &) {
    ++*static_cast<size_t *>(context);
}

fleet::result<fleet::bytes> encode_wire(
    uint8_t type, std::span<const fleet::can_frame> frames) {
    fleet::bytes wire(fleet::motor_codec::MAX_PACKET_SZ);
    auto size = fleet::motor_codec::encode_into(fleet::bytes_mut(wire), type, frames);
    if (!size) {
        return fleet::fail(size.error());
    }
    wire.resize(*size);
    return wire;
}

fleet::result<std::vector<fleet::can_frame>> decode_frames(fleet::bytes_view wire) {
    std::vector<fleet::can_frame> frames;
    if (auto err = fleet::motor_codec::decode_each(wire, &frames, &collect_frame)) {
        return fleet::fail(err);
    }
    return frames;
}

} // namespace

TEST_CASE("udp codec serializes and parses a packet with can frames") {
    std::vector<fleet::can_frame> packet;
    packet.push_back(make_frame(1, 0x123, {0x10, 0x20, 0x30}));
    packet.push_back(make_frame(2, 0x456, {0x40, 0x50}));

    auto encoded = encode_wire(fleet::TYPE_UDP2CAN, packet);
    REQUIRE(encoded);
    REQUIRE(encoded->front() == fleet::motor_codec::STX);
    REQUIRE(encoded->back() == fleet::motor_codec::ETX);

    auto decoded = decode_frames(*encoded);
    REQUIRE(decoded);
    REQUIRE(decoded->size() == 2);
    CHECK(decoded->at(0).port == 1);
    CHECK(decoded->at(0).id == 0x123);
    CHECK(decoded->at(0).len == 3);
    CHECK(decoded->at(0).data[0] == 0x10);
    CHECK(decoded->at(0).data[1] == 0x20);
    CHECK(decoded->at(0).data[2] == 0x30);
    CHECK(decoded->at(1).port == 2);
    CHECK(decoded->at(1).id == 0x456);
    CHECK(decoded->at(1).len == 2);
}

TEST_CASE("udp codec dispatches decoded frames without constructing a packet") {
    std::vector<fleet::can_frame> packet;
    packet.push_back(make_frame(1, 0x123, {0x10, 0x20}));
    packet.push_back(make_frame(2, 0x456, {0x30}));

    const auto encoded = encode_wire(fleet::TYPE_UDP2CAN, packet);
    REQUIRE(encoded);

    std::vector<fleet::can_frame> frames;
    REQUIRE_FALSE(fleet::motor_codec::decode_each(*encoded, &frames, &collect_frame));
    REQUIRE(frames.size() == 2);
    CHECK(frames[0].port == 1);
    CHECK(frames[0].id == 0x123);
    CHECK(frames[1].port == 2);
    CHECK(frames[1].id == 0x456);
}

TEST_CASE("udp codec encodes the documented UDP2CAN binary layout") {
    std::vector<fleet::can_frame> packet;
    packet.push_back(make_frame(2, 0x123, {0x10, 0x20, 0x30}));

    auto encoded = encode_wire(fleet::TYPE_UDP2CAN, packet);

    REQUIRE(encoded);
    const fleet::bytes expected{
        0xAA,                   // STX
        0x01,                   // TYPE: UDP to CAN
        0x00, 0x0F,             // payload length: msg cnt + 1 CAN msg
        0x01,                   // msg cnt
        0x02,                   // CAN port
        0x00, 0x00, 0x01, 0x23, // CAN id, big-endian
        0x03,                   // active data len
        0x10, 0x20, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0xD0, // CRC16-CCITT, little-endian
        0xBB,                                                       // ETX
    };
    CHECK(*encoded == expected);
}

TEST_CASE("udp codec can encode into caller-provided buffer") {
    std::vector<fleet::can_frame> packet;
    packet.push_back(make_frame(2, 0x123, {0x10, 0x20, 0x30}));

    fleet::bytes bytes(fleet::motor_codec::MAX_PACKET_SZ);
    auto size = fleet::motor_codec::encode_into(fleet::bytes_mut(bytes), fleet::TYPE_UDP2CAN, packet);
    REQUIRE(size);
    bytes.resize(*size);

    const fleet::bytes expected{
        0xAA, 0x01, 0x00, 0x0F, 0x01, 0x02, 0x00, 0x00, 0x01, 0x23, 0x03,
        0x10, 0x20, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0xD0, 0xBB,
    };
    CHECK(bytes == expected);
}

TEST_CASE("udp codec batches commands for multiple motors on the same CAN port") {
    constexpr fleet::can_port_t port = 1;

    std::vector<fleet::can_frame> packet;
    packet.push_back(make_frame(port, 1, {0x43, 0x41, 0xA0, 0x00, 0x00, 0x00, 0x32}));
    packet.push_back(make_frame(port, 2, {0x62, 0x01, 0x2C}));
    packet.push_back(make_frame(port, 3, {0x20, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0xCA}));

    auto encoded = encode_wire(fleet::TYPE_UDP2CAN, packet);
    REQUIRE(encoded);
    auto &wire = *encoded;

    REQUIRE(wire.size() == 50);
    CHECK(wire[0] == fleet::motor_codec::STX);
    CHECK(wire[1] == fleet::TYPE_UDP2CAN);
    CHECK(wire[2] == 0x00);
    CHECK(wire[3] == 0x2B);
    CHECK(wire[4] == 0x03);
    check_entry(wire, 5, port, 1, 7,
                std::array<uint8_t, 7>{0x43, 0x41, 0xA0, 0x00, 0x00, 0x00, 0x32});
    check_entry(wire, 19, port, 2, 3, std::array<uint8_t, 3>{0x62, 0x01, 0x2C});
    check_entry(wire, 33, port, 3, 8,
                std::array<uint8_t, 8>{0x20, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0xCA});
    CHECK(wire.back() == fleet::motor_codec::ETX);

    const auto crc_offset = wire.size() - 3;
    const auto crc = fleet::motor_codec::crc16_ccitt(fleet::bytes_view(wire).subspan(1, wire.size() - 4));
    CHECK(wire[crc_offset] == static_cast<std::uint8_t>(crc & 0xFFU));
    CHECK(wire[crc_offset + 1] == static_cast<std::uint8_t>(crc >> 8U));
}

TEST_CASE("udp codec decodes and dispatches batched same-port motor frames by CAN id") {
    constexpr fleet::can_port_t port = 1;
    std::vector<fleet::can_frame> outbound;
    outbound.push_back(make_frame(port, 1, {0x43, 0x41, 0xA0, 0x00, 0x00, 0x00, 0x32}));
    outbound.push_back(make_frame(port, 2, {0x62, 0x01, 0x2C}));
    outbound.push_back(
        make_frame(port, 3, {0x20, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0xCA}));

    auto wire = encode_wire(fleet::TYPE_UDP2CAN, outbound);
    REQUIRE(wire);

    auto decoded = decode_frames(*wire);
    REQUIRE(decoded);
    REQUIRE(decoded->size() == 3);

    std::unordered_map<std::uint32_t, fleet::can_frame> delivered;
    for (const auto &frame : *decoded) {
        REQUIRE(frame.port == port);
        delivered.emplace(frame.id, frame);
    }

    REQUIRE(delivered.size() == 3);
    REQUIRE(delivered.contains(1));
    REQUIRE(delivered.contains(2));
    REQUIRE(delivered.contains(3));
    CHECK(delivered.at(1).len == 7);
    CHECK(delivered.at(1).data[0] == 0x43);
    CHECK(delivered.at(2).len == 3);
    CHECK(delivered.at(2).data[0] == 0x62);
    CHECK(delivered.at(3).len == 8);
    CHECK(delivered.at(3).data[0] == 0x20);
}

TEST_CASE("udp codec supports mixed standard and extended can frames") {
    std::vector<fleet::can_frame> packet;
    packet.push_back(make_frame(1, 0x123, {0x10}));
    packet.push_back(make_extended_frame(2, 0x18FF50E5, {0x20, 0x30}));

    auto encoded = encode_wire(fleet::TYPE_UDP2CAN, packet);
    REQUIRE(encoded);

    auto decoded = decode_frames(*encoded);
    REQUIRE(decoded);
    REQUIRE(decoded->size() == 2);
    CHECK_FALSE(decoded->at(0).extended);
    CHECK(decoded->at(0).id == 0x123);
    CHECK(decoded->at(0).len == 1);
    CHECK(decoded->at(0).data[0] == 0x10);
    CHECK(decoded->at(1).extended);
    CHECK(decoded->at(1).id == 0x18FF50E5);
    CHECK(decoded->at(1).len == 2);
    CHECK(decoded->at(1).data[0] == 0x20);
    CHECK(decoded->at(1).data[1] == 0x30);
}

TEST_CASE("udp codec decodes payload lengths wider than one byte") {
    std::vector<fleet::can_frame> packet;
    packet.reserve(fleet::MAX_FRAMES_PER_PACKET);
    for (std::uint8_t i = 0; i < fleet::MAX_FRAMES_PER_PACKET; ++i) {
        packet.push_back(make_frame(0, static_cast<std::uint32_t>(0x100U + i), {i}));
    }

    auto encoded = encode_wire(fleet::TYPE_UDP2CAN, packet);
    REQUIRE(encoded);
    REQUIRE(encoded->at(2) == 0x01);
    REQUIRE(encoded->at(3) == 0xC1);

    auto decoded = decode_frames(*encoded);
    REQUIRE(decoded);
    REQUIRE(decoded->size() == fleet::MAX_FRAMES_PER_PACKET);
}

TEST_CASE("udp codec encodes extended can id flag in wire id") {
    std::vector<fleet::can_frame> packet;
    packet.push_back(make_extended_frame(2, 0x18FF50E5, {0x20}));

    auto encoded = encode_wire(fleet::TYPE_UDP2CAN, packet);

    REQUIRE(encoded);
    REQUIRE(encoded->size() >= 10);
    CHECK(encoded->at(6) == 0x98);
    CHECK(encoded->at(7) == 0xFF);
    CHECK(encoded->at(8) == 0x50);
    CHECK(encoded->at(9) == 0xE5);
}

TEST_CASE("udp codec rejects concatenated packets instead of treating them as one sticky frame") {
    std::vector<fleet::can_frame> packet;
    packet.push_back(make_frame(0, 0x101, {0x01}));

    auto first = encode_wire(fleet::TYPE_UDP2CAN, packet);
    auto second = encode_wire(fleet::TYPE_UDP2CAN, packet);
    REQUIRE(first);
    REQUIRE(second);

    fleet::bytes concatenated = *first;
    concatenated.insert(concatenated.end(), second->begin(), second->end());

    size_t decoded = 0;
    const auto err = fleet::motor_codec::decode_each(concatenated, &decoded, &count_frame);
    REQUIRE(err);
    CHECK(err == fleet::make_error_code(fleet::fleet_err::protocol_error));

    CHECK_FALSE(fleet::motor_codec::decode_each(*first, &decoded, &count_frame));
    CHECK_FALSE(fleet::motor_codec::decode_each(*second, &decoded, &count_frame));
}

TEST_CASE("udp codec reports crc mismatch") {
    std::vector<fleet::can_frame> packet;
    packet.push_back(make_frame(0, 0x102, {0xAA, 0x55}));

    auto encoded = encode_wire(fleet::TYPE_UDP2CAN, packet);
    REQUIRE(encoded);
    encoded->at(5) ^= 0x01;

    size_t decoded = 0;
    const auto err = fleet::motor_codec::decode_each(*encoded, &decoded, &count_frame);
    REQUIRE(err);
    CHECK(err == fleet::make_error_code(fleet::fleet_err::protocol_error));
}
