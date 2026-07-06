/*
    Copyright PNDBotics 2026
*/
#pragma once

#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

#if defined(_WIN32)
#define FLEET_EXPORT __declspec(dllexport)
#else
#define FLEET_EXPORT
#endif

namespace fleet {

using err_code = std::error_code;

/*
    注意和 https://wiki.pndbotics.com/actuator/software/fleet_cpp_sdk/ 中的执行器错误码区分开
*/
enum class fleet_err {
    ok = 0,
    invalid_argument, // 参数错误
    unsupported,      // 类型、功能不支持
    busy,             // 队列满、总线忙、设备忙
    timeout,
    cancelled,
    io_error,       // socket, fd, driver error
    protocol_error, // 非法帧格式
    device_error,   // 设备明确错误
};

/**
 * Utility Type Definition
 */
using namespace std::chrono_literals;
using milliseconds = std::chrono::milliseconds;
using seconds = std::chrono::seconds;
using time_point = std::chrono::steady_clock::time_point;
using duration = std::chrono::steady_clock::duration;

using bytes = std::vector<uint8_t>;
using bytes_view = std::span<const uint8_t>;
using bytes_mut = std::span<uint8_t>;

const std::error_category &error_category() noexcept;
std::error_code make_error_code(fleet_err code) noexcept;

using device_id_t = uint32_t;

class device_interface {
  public:
    virtual ~device_interface() = default;

    device_id_t id() const noexcept { return id_; }
    std::string_view name() const noexcept { return name_; }
    time_point updated_at() const noexcept { return updated_at_; }

  protected:
    explicit device_interface(device_id_t id, std::string name) : id_(id), name_(std::move(name)) {}

    void set_updated_at(time_point time) noexcept { updated_at_ = time; }

  private:
    device_id_t id_ = 0;
    std::string name_;
    time_point updated_at_{};
};

template <class T>
concept IsDevice = std::derived_from<T, device_interface>;

/*
    sample_t 只表示缓存到的一次状态采样：seq != 0 表示至少收到过该字段，
    time 表示该值的更新时间。

    SDK 不判断该值是否足够新鲜。控制循环应使用自己的时钟与 time 比较，
    并按字段和业务场景选择合适的过期阈值。
*/
template <typename T> struct sample_t {
    time_point time{};
    uint64_t seq = 0;
    T value{};

    bool received() const noexcept { return seq != 0; }

    bool fresh(time_point now, milliseconds max_age) const noexcept {
        return received() && now - time <= max_age;
    }

    explicit operator bool() const noexcept { return received(); }
};

} // namespace fleet

template <> struct std::is_error_code_enum<fleet::fleet_err> : std::true_type {};
