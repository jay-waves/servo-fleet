/*
    Copyright PNDBotics 2026
*/
#pragma once

#include "codec_can.h"

/**
  ENCOS 电机通信协议

  +------+-------+----------+-------------------------+----------+------+
  | STX  | TYPE  | LENGTH   | PAYLOAD                 | CHECKSUM | ETX  |
  | 1 B  | 1 B   | 2 B      | LENGTH bytes            | 2 B      | 1 B  |
  +------+-------+----------+-------------------------+----------+------+
  | 0xAA | 0/1   | big-end. | MSG_CNT + CAN messages  | little   | 0xBB |
  +------+-------+----------+-------------------------+----------+------+

  `TYPE`

  电机使用 CAN-over-UDP
  * 0, CAN --> UDP
  * 1, UDP --> CAN
  * 2, JSON over UDP #TODO

  `LENGTH` is the payload length in bytes:

  LENGTH = 1 + MSG_CNT * 14

  `CHECKSUM` is CRC16-CCITT:

  poly: 0x1021
  init: 0x0000
  input: TYPE + LENGTH + PAYLOAD, starting at frame[1]
  output bytes: low byte first, high byte second
 */

namespace fleet {

inline constexpr uint8_t TYPE_CAN2UDP = 0x00;
inline constexpr uint8_t TYPE_UDP2CAN = 0x01;
inline constexpr uint8_t TYPE_JSON = 0x02;

namespace motor_codec {

inline constexpr uint8_t STX = 0xAA;
inline constexpr uint8_t ETX = 0xBB;
inline constexpr uint32_t CAN_ENTRY_SZ = 14;
inline constexpr uint32_t MIN_PACKET_SZ = 8;
inline constexpr uint32_t MAX_PACKET_SZ = MIN_PACKET_SZ + MAX_FRAMES_PER_PACKET * CAN_ENTRY_SZ;

result<uint32_t> encode_into(bytes_mut out, uint8_t type, std::span<const can_frame> frames);
err_code decode_each(bytes_view bytes, void *context, void (*handler)(void *context, const can_frame &frame));
uint16_t crc16_ccitt(bytes_view bytes) noexcept;
} // namespace motor_codec

} // namespace fleet
