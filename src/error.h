#pragma once

#include <expected>
#include <system_error>

namespace fleet {

using err_code = std::error_code;

template <typename T> using result = std::expected<T, err_code>;
using fail = std::unexpected<err_code>;

} // namespace fleet
