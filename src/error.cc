#include "fleet/fleet.h"

#include <string>

namespace fleet {
namespace {

class motor_error_category final : public std::error_category {
  public:
    [[nodiscard]] const char *name() const noexcept override { return "FLEET_motor"; }

    [[nodiscard]] std::string message(int value) const override {
        switch (static_cast<fleet_err>(value)) {
        case fleet_err::ok: return "ok";
        case fleet_err::invalid_argument: return "invalid_argument";
        case fleet_err::unsupported: return "unsupported";
        case fleet_err::busy: return "busy";
        case fleet_err::timeout: return "timeout";
        case fleet_err::cancelled: return "cancelled";
        case fleet_err::io_error: return "io_error";
        case fleet_err::protocol_error: return "protocol_error";
        case fleet_err::device_error: return "device_error";
        }
        return "unknown motor error";
    }
};

} // namespace

const std::error_category &error_category() noexcept {
    static const motor_error_category category;
    return category;
}

std::error_code make_error_code(fleet_err code) noexcept {
    return {static_cast<int>(code), error_category()};
}

} // namespace fleet
