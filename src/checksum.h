#pragma once

#include "fleet/device.h"

#include <cstdint>

namespace fleet {

uint16_t crc16_ccitt(bytes_view bytes) noexcept;

} // namespace fleet
