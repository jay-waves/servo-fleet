/* Linux-only realtime experiment configuration. Kept on the rt-linux branch. */
#pragma once

#include "device.h"

#include <chrono>
#include <string_view>
#include <vector>

namespace fleet {

enum class linux_scheduler_policy {
    inherit,
    other,
    fifo,
    deadline,
};

struct linux_thread_config {
    linux_scheduler_policy policy = linux_scheduler_policy::inherit;
    std::vector<int> cpus;
    int fifo_priority = 80;
    std::chrono::nanoseconds runtime{};
    std::chrono::nanoseconds deadline{};
    std::chrono::nanoseconds period{};
    bool lock_memory = false;

    bool configured() const noexcept {
        return policy != linux_scheduler_policy::inherit || !cpus.empty() || lock_memory;
    }
};

/* Returns a generic-category errno on Linux configuration failure. */
FLEET_EXPORT err_code tune_current_thread(const linux_thread_config &config,
                                          std::string_view name) noexcept;

} // namespace fleet
