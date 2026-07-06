#pragma once

#include <string_view>

namespace fleet {

struct thread_tuning {
    int cpu = -1;
    int fifo_priority = 0;
    bool lock_memory = false;
    std::string_view name;
};

void tune_current_thread(const thread_tuning &tuning) noexcept;

} // namespace fleet
