#include "codec_bit.h"

#include <cstring>

namespace fleet {

result<uint32_t> read_u32_be(const uint8_t *bytes, size_t bytes_len, size_t bit_offset, size_t bit_len) {

    if (bytes == nullptr || bit_len == 0 || bit_len > 32 || bit_offset + bit_len > bytes_len * 8) {
        return fail(fleet_err::invalid_argument);
    }

    if (bit_offset % 8 == 0 && bit_len % 8 == 0) {
        const auto byte_offset = bit_offset / 8;
        uint32_t value = 0;
        for (size_t i = 0; i < bit_len / 8; ++i) {
            value = (value << 8U) | bytes[byte_offset + i];
        }
        return value;
    }

    uint32_t value = 0;
    for (size_t i = 0; i < bit_len; ++i) {
        const size_t bit = bit_offset + i;
        const auto byte = bytes[bit / 8];
        const auto shift = static_cast<uint8_t>(7 - (bit % 8));
        value = (value << 1U) | ((byte >> shift) & 0x01U);
    }
    return value;
}

err_code write_u32_be(uint8_t *bytes, size_t bytes_len, size_t bit_offset, size_t bit_len, uint32_t value) {

    if (bytes == nullptr || bit_len == 0 || bit_len > 32 || bit_offset + bit_len > bytes_len * 8) {
        return fleet_err::invalid_argument;
    }
    if (bit_len < 32 && (value >> bit_len) != 0) {
        return fleet_err::invalid_argument;
    }

    if (bit_offset % 8 == 0 && bit_len % 8 == 0) {
        const auto byte_offset = bit_offset / 8;
        for (size_t i = 0; i < bit_len / 8; ++i) {
            const auto shift = static_cast<uint32_t>(bit_len - 8 - i * 8);
            bytes[byte_offset + i] = static_cast<uint8_t>(value >> shift);
        }
        return {};
    }

    for (size_t i = 0; i < bit_len; ++i) {
        const size_t bit = bit_offset + i;
        const auto mask = static_cast<uint8_t>(1U << (7 - (bit % 8)));
        const bool set = ((value >> (bit_len - 1 - i)) & 0x01U) != 0;
        if (set) {
            bytes[bit / 8] = static_cast<uint8_t>(bytes[bit / 8] | mask);
        } else {
            bytes[bit / 8] = static_cast<uint8_t>(bytes[bit / 8] & ~mask);
        }
    }
    return {};
}

} // namespace fleet
