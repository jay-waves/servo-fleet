#include "codec_motor.h"
#include "checksum.h"

#include <algorithm>

namespace fleet {
namespace {

constexpr uint32_t HEADER_SZ = 4;
constexpr uint32_t CRC_SZ = 2;
constexpr uint32_t TRAILER_SZ = 1;
constexpr uint32_t MSG_CNT_SZ = 1;

namespace wire {
constexpr uint32_t STX_OFFSET = 0;
constexpr uint32_t TYPE_OFFSET = 1;
constexpr uint32_t PAYLOAD_LEN_OFFSET = 2;
constexpr uint32_t MSG_CNT_OFFSET = HEADER_SZ;

constexpr uint32_t ENTRY_PORT_OFFSET = 0;
constexpr uint32_t ENTRY_WIRE_ID_OFFSET = 1;
constexpr uint32_t ENTRY_LEN_OFFSET = 5;
constexpr uint32_t ENTRY_DATA_OFFSET = 6;
} // namespace wire

struct HeaderInfo {
    uint8_t type = 0;
    uint16_t payload_len = 0;
};

uint32_t packet_size(uint16_t payload_len) {
    return HEADER_SZ + payload_len + CRC_SZ + TRAILER_SZ;
}

bool supported_type(uint8_t type) { return type == TYPE_CAN2UDP || type == TYPE_UDP2CAN; }

uint32_t payload_len_for(uint32_t frame_count) { return MSG_CNT_SZ + frame_count * motor_codec::CAN_ENTRY_SZ; }

uint16_t read_u16_le(bytes_view bytes, uint32_t offset) {
    return static_cast<uint16_t>(static_cast<uint16_t>(bytes[offset]) |
                                 (static_cast<uint16_t>(bytes[offset + 1]) << 8U));
}

uint16_t read_u16_be(bytes_view bytes, uint32_t offset) {
    return static_cast<uint16_t>((static_cast<uint16_t>(bytes[offset]) << 8U) |
                                 static_cast<uint16_t>(bytes[offset + 1]));
}

uint32_t read_u32_be(bytes_view bytes, uint32_t offset) {
    return (static_cast<uint32_t>(bytes[offset]) << 24U) |
           (static_cast<uint32_t>(bytes[offset + 1]) << 16U) |
           (static_cast<uint32_t>(bytes[offset + 2]) << 8U) |
           static_cast<uint32_t>(bytes[offset + 3]);
}

void write_u16_be(bytes_mut bytes, uint32_t offset, uint16_t value) {
    bytes[offset] = static_cast<uint8_t>(value >> 8U);
    bytes[offset + 1] = static_cast<uint8_t>(value);
}

void write_u32_be(bytes_mut bytes, uint32_t offset, uint32_t value) {
    bytes[offset] = static_cast<uint8_t>(value >> 24U);
    bytes[offset + 1] = static_cast<uint8_t>(value >> 16U);
    bytes[offset + 2] = static_cast<uint8_t>(value >> 8U);
    bytes[offset + 3] = static_cast<uint8_t>(value);
}

uint32_t wire_id_from(const can_frame &frame) { return frame.extended ? (frame.id | EXTENDED_ID_FLAG) : frame.id; }

void apply_wire_id(can_frame &frame, uint32_t wire_id) {
    frame.extended = (wire_id & EXTENDED_ID_FLAG) != 0;
    frame.id = wire_id & ~EXTENDED_ID_FLAG;
}

result<HeaderInfo> read_header(bytes_view bytes) {
    if (bytes.size() < motor_codec::MIN_PACKET_SZ || bytes[wire::STX_OFFSET] != motor_codec::STX ||
        bytes.back() != motor_codec::ETX) {
        return fail(fleet_err::protocol_error);
    }

    const auto type = bytes[wire::TYPE_OFFSET];
    const auto payload_len = read_u16_be(bytes, wire::PAYLOAD_LEN_OFFSET);
    if (!supported_type(type)) {
        return fail(fleet_err::unsupported);
    }
    if (bytes.size() != packet_size(payload_len) || payload_len < MSG_CNT_SZ) {
        return fail(fleet_err::protocol_error);
    }

    return HeaderInfo{type, payload_len};
}

err_code verify_crc(bytes_view bytes, uint16_t payload_len) {
    const auto crc_offset = HEADER_SZ + payload_len;
    const auto crc_calc = motor_codec::crc16_ccitt(bytes.subspan(1, 3 + payload_len));
    const auto crc_recv = read_u16_le(bytes, crc_offset);
    if (crc_calc != crc_recv) {
        return fleet_err::protocol_error;
    }
    return {};
}

result<can_frame> read_can_entry(bytes_view entry) {
    can_frame frame;
    frame.port = entry[wire::ENTRY_PORT_OFFSET];
    frame.len = entry[wire::ENTRY_LEN_OFFSET];
    apply_wire_id(frame, read_u32_be(entry, wire::ENTRY_WIRE_ID_OFFSET));
    if (!frame.valid()) {
        return fail(fleet_err::protocol_error);
    }

    std::copy_n(entry.begin() + wire::ENTRY_DATA_OFFSET, MAX_CAN_DATA_LEN, frame.data.begin());
    return frame;
}

void write_can_entry(bytes_mut entry, const can_frame &frame) {
    entry[wire::ENTRY_PORT_OFFSET] = frame.port;
    write_u32_be(entry, wire::ENTRY_WIRE_ID_OFFSET, wire_id_from(frame));
    entry[wire::ENTRY_LEN_OFFSET] = frame.len;
    std::copy(frame.data.begin(), frame.data.end(), entry.begin() + wire::ENTRY_DATA_OFFSET);
}

} // namespace

namespace motor_codec {

result<uint32_t> encode_into(bytes_mut out, uint8_t type, std::span<const can_frame> frames) {
    if (frames.size() > MAX_FRAMES_PER_PACKET) {
        return fail(fleet_err::protocol_error);
    }

    const auto frame_count = static_cast<uint32_t>(frames.size());
    const auto payload_len = payload_len_for(frame_count);
    if (payload_len > UINT16_MAX) {
        return fail(fleet_err::protocol_error);
    }

    const auto size = packet_size(static_cast<uint16_t>(payload_len));
    if (out.size() < size) {
        return fail(fleet_err::invalid_argument);
    }
    for (const auto &frame : frames) {
        if (!frame.valid()) {
            return fail(fleet_err::protocol_error);
        }
    }

    auto packet = out.first(static_cast<size_t>(size));
    packet[wire::STX_OFFSET] = STX;
    packet[wire::TYPE_OFFSET] = type;
    write_u16_be(packet, wire::PAYLOAD_LEN_OFFSET, static_cast<uint16_t>(payload_len));
    packet[wire::MSG_CNT_OFFSET] = static_cast<uint8_t>(frame_count);

    uint32_t offset = HEADER_SZ + MSG_CNT_SZ;
    for (const auto &frame : frames) {
        write_can_entry(packet.subspan(offset, CAN_ENTRY_SZ), frame);
        offset += CAN_ENTRY_SZ;
    }

    const auto crc = crc16_ccitt(bytes_view(packet).subspan(1, 3 + payload_len));
    packet[offset] = static_cast<uint8_t>(crc & 0xFFU);
    packet[offset + 1] = static_cast<uint8_t>(crc >> 8U);
    packet[offset + CRC_SZ] = ETX;
    return size;
}

err_code decode_each(bytes_view bytes, void *context, void (*handler)(void *context, const can_frame &frame)) {
    if (handler == nullptr)
        return fleet_err::invalid_argument;

    const auto header = read_header(bytes);
    if (!header)
        return header.error();

    auto err = verify_crc(bytes, header->payload_len);
    if (err) return err;

    const auto frame_count = bytes[wire::MSG_CNT_OFFSET];
    if (header->payload_len != payload_len_for(frame_count) || frame_count > MAX_FRAMES_PER_PACKET) {
        return fleet_err::protocol_error;
    }

    uint32_t offset = HEADER_SZ + MSG_CNT_SZ;
    for (uint8_t index = 0; index < frame_count; ++index) {
        auto frame = read_can_entry(bytes.subspan(offset, CAN_ENTRY_SZ));
        if (!frame)
            return frame.error();
        offset += CAN_ENTRY_SZ;
        handler(context, *frame);
    }
    return {};
}

uint16_t crc16_ccitt(bytes_view bytes) noexcept {
    return fleet::crc16_ccitt(bytes);
}

} // namespace motor_codec

} // namespace fleet
