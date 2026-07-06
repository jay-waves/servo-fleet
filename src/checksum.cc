#include "checksum.h"

#include <array>

namespace fleet {
namespace {

constexpr uint16_t CRC16_CCITT_POLY = 0x1021U;

constexpr uint16_t crc16_ccitt_table_entry(uint8_t index) noexcept {
    auto crc = static_cast<uint16_t>(static_cast<uint16_t>(index) << 8U);
    for (uint8_t bit = 0; bit < 8U; ++bit) {
        if ((crc & 0x8000U) != 0) {
            crc = static_cast<uint16_t>((crc << 1U) ^ CRC16_CCITT_POLY);
        } else {
            crc = static_cast<uint16_t>(crc << 1U);
        }
    }
    return crc;
}

constexpr std::array<uint16_t, 256> make_crc16_ccitt_table() noexcept {
    std::array<uint16_t, 256> table{};
    for (size_t index = 0; index < table.size(); ++index) {
        table[index] = crc16_ccitt_table_entry(static_cast<uint8_t>(index));
    }
    return table;
}

alignas(64) constexpr auto CRC16_CCITT_TABLE = make_crc16_ccitt_table();

constexpr uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte) noexcept {
    const auto index = static_cast<uint8_t>((crc >> 8U) ^ byte);
    return static_cast<uint16_t>((crc << 8U) ^ CRC16_CCITT_TABLE[index]);
}

constexpr std::array<uint16_t, 256> make_crc16_ccitt_slice_table(
    const std::array<uint16_t, 256> &previous) noexcept {
    std::array<uint16_t, 256> table{};
    for (size_t index = 0; index < table.size(); ++index) {
        table[index] = crc16_ccitt_update(previous[index], 0);
    }
    return table;
}

alignas(64) constexpr auto CRC16_CCITT_TABLE_1 = make_crc16_ccitt_slice_table(CRC16_CCITT_TABLE);
alignas(64) constexpr auto CRC16_CCITT_TABLE_2 = make_crc16_ccitt_slice_table(CRC16_CCITT_TABLE_1);
alignas(64) constexpr auto CRC16_CCITT_TABLE_3 = make_crc16_ccitt_slice_table(CRC16_CCITT_TABLE_2);
alignas(64) constexpr auto CRC16_CCITT_TABLE_4 = make_crc16_ccitt_slice_table(CRC16_CCITT_TABLE_3);
alignas(64) constexpr auto CRC16_CCITT_TABLE_5 = make_crc16_ccitt_slice_table(CRC16_CCITT_TABLE_4);
alignas(64) constexpr auto CRC16_CCITT_TABLE_6 = make_crc16_ccitt_slice_table(CRC16_CCITT_TABLE_5);
alignas(64) constexpr auto CRC16_CCITT_TABLE_7 = make_crc16_ccitt_slice_table(CRC16_CCITT_TABLE_6);

} // namespace

uint16_t crc16_ccitt(bytes_view bytes) noexcept {
    uint16_t crc = 0x0000U;
    const auto *data = bytes.data();
    const auto size = bytes.size();

    size_t offset = 0;
    for (; offset + 8 <= size; offset += 8) {
        crc = static_cast<uint16_t>(
            CRC16_CCITT_TABLE_7[static_cast<uint8_t>((crc >> 8U) ^ data[offset])] ^
            CRC16_CCITT_TABLE_6[static_cast<uint8_t>((crc & 0xFFU) ^ data[offset + 1])] ^
            CRC16_CCITT_TABLE_5[data[offset + 2]] ^
            CRC16_CCITT_TABLE_4[data[offset + 3]] ^
            CRC16_CCITT_TABLE_3[data[offset + 4]] ^
            CRC16_CCITT_TABLE_2[data[offset + 5]] ^
            CRC16_CCITT_TABLE_1[data[offset + 6]] ^
            CRC16_CCITT_TABLE[data[offset + 7]]);
    }
    for (; offset < size; ++offset) {
        crc = crc16_ccitt_update(crc, data[offset]);
    }
    return crc;
}

} // namespace fleet
