/*
    用于位级别的网络协议序列化、反序列化
*/

#pragma once
#include <bit>
#include <cstdint>
#include <type_traits>

#include "error.h"
#include "fleet/device.h"

namespace fleet {

template <size_t Offset, size_t Len> struct Bits {
    static constexpr size_t offset = Offset;
    static constexpr size_t len = Len;

    static_assert(Len > 0);
    static_assert(Len <= 32);
};

template <size_t Offset> using U8 = Bits<Offset, 8>;
template <size_t Offset> using U16 = Bits<Offset, 16>;
template <size_t Offset> using U32 = Bits<Offset, 32>;

result<uint32_t> read_u32_be(const uint8_t *bytes, size_t bytes_len, size_t bit_offset, size_t bit_len);
err_code write_u32_be(uint8_t *bytes, size_t bytes_len, size_t bit_offset, size_t bit_len, uint32_t value);

template <typename F>
concept BitField = requires {
    { F::offset } -> std::convertible_to<size_t>;
    { F::len } -> std::convertible_to<size_t>;
};

constexpr uint32_t mask32(size_t bits) noexcept {
    return bits >= 32 ? 0xFFFF'FFFFu : ((1u << bits) - 1u);
}

template <BitField Field> constexpr uint32_t field_mask() noexcept { return mask32(Field::len); }

template <BitField Field> inline result<uint32_t> read_aligned(bytes_view bytes) {
    static_assert(Field::offset % 8 == 0);
    static_assert(Field::len == 8 || Field::len == 16 || Field::len == 32);

    constexpr auto byte_offset = Field::offset / 8;

    if (bytes.size() < byte_offset + Field::len / 8) {
        return fail(fleet_err::invalid_argument);
    }

    if constexpr (Field::len == 8) {
        return bytes[byte_offset];
    } else if constexpr (Field::len == 16) {
        return (static_cast<uint32_t>(bytes[byte_offset]) << 8U) |
               static_cast<uint32_t>(bytes[byte_offset + 1]);
    } else if constexpr (Field::len == 32) {
        return (static_cast<uint32_t>(bytes[byte_offset]) << 24U) |
               (static_cast<uint32_t>(bytes[byte_offset + 1]) << 16U) |
               (static_cast<uint32_t>(bytes[byte_offset + 2]) << 8U) |
               static_cast<uint32_t>(bytes[byte_offset + 3]);
    }
}

template <BitField Field> inline err_code write_aligned(bytes_mut bytes, uint32_t raw) {
    static_assert(Field::offset % 8 == 0);
    static_assert(Field::len == 8 || Field::len == 16 || Field::len == 32);

    constexpr auto byte_offset = Field::offset / 8;

    if (bytes.size() < byte_offset + Field::len / 8) {
        return fleet_err::invalid_argument;
    }

    if constexpr (Field::len == 8) {
        bytes[byte_offset] = static_cast<uint8_t>(raw);
    } else if constexpr (Field::len == 16) {
        bytes[byte_offset] = static_cast<uint8_t>(raw >> 8U);
        bytes[byte_offset + 1] = static_cast<uint8_t>(raw);
    } else if constexpr (Field::len == 32) {
        bytes[byte_offset] = static_cast<uint8_t>(raw >> 24U);
        bytes[byte_offset + 1] = static_cast<uint8_t>(raw >> 16U);
        bytes[byte_offset + 2] = static_cast<uint8_t>(raw >> 8U);
        bytes[byte_offset + 3] = static_cast<uint8_t>(raw);
    }
    return {};
}

template <BitField Field> inline result<uint32_t> read(bytes_view bytes) {
    if constexpr (Field::offset % 8 == 0 &&
                  (Field::len == 8 || Field::len == 16 || Field::len == 32)) {
        return read_aligned<Field>(bytes);
    }
    return read_u32_be(bytes.data(), bytes.size(), Field::offset, Field::len);
}

template <BitField Field> inline err_code write(bytes_mut bytes, uint32_t raw) {
    if ((raw & ~field_mask<Field>()) != 0) {
        return fleet_err::invalid_argument;
    }

    if constexpr (Field::offset % 8 == 0 &&
                  (Field::len == 8 || Field::len == 16 || Field::len == 32)) {
        return write_aligned<Field>(bytes, raw);
    }
    return write_u32_be(bytes.data(), bytes.size(), Field::offset, Field::len, raw);
}

/**
 * Type Cast Helpers...
 */

template <typename E> constexpr uint32_t enum_u32(E value) noexcept {
    static_assert(std::is_enum_v<E>);
    return static_cast<uint32_t>(static_cast<std::underlying_type_t<E>>(value));
}

template <typename E> constexpr uint16_t enum_u16(E value) noexcept {
    static_assert(std::is_enum_v<E>);
    return static_cast<uint16_t>(static_cast<std::underlying_type_t<E>>(value));
}

template <typename E> constexpr uint8_t enum_u8(E value) noexcept {
    static_assert(std::is_enum_v<E>);
    return static_cast<uint8_t>(static_cast<std::underlying_type_t<E>>(value));
}

// Notice, uint8_t to uint32_t may cause overflow
template <typename T> constexpr auto cast_to() noexcept {
    return [](auto v) { return static_cast<T>(v); };
}

inline constexpr auto to_u8 = cast_to<uint8_t>();
inline constexpr auto to_u16 = cast_to<uint16_t>();
inline constexpr auto to_u32 = cast_to<uint32_t>();

inline float bits_to_f32(uint32_t raw) noexcept { return std::bit_cast<float>(raw); }

inline uint32_t f32_to_bits(double v) noexcept {
    return std::bit_cast<uint32_t>(static_cast<float>(v));
}

inline double bits_to_f64(uint32_t raw) noexcept {
    return static_cast<double>(std::bit_cast<float>(raw));
}


} // namespace fleet
