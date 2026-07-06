#include "thread_tuning.h"

#include "log.h"

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#endif

namespace fleet {
namespace {

#ifdef __linux__
void set_affinity(pthread_t thread, const thread_tuning &tuning) noexcept {
    if (tuning.cpu < 0)
        return;

    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(tuning.cpu, &cpus);
    if (const int err = pthread_setaffinity_np(thread, sizeof(cpus), &cpus); err != 0) {
        FLEET_WARN("thread '{}' cpu affinity failed cpu={} err={}", tuning.name, tuning.cpu, err);
    }
}

void set_fifo(pthread_t thread, const thread_tuning &tuning) noexcept {
    if (tuning.fifo_priority <= 0)
        return;

    sched_param param{};
    param.sched_priority = tuning.fifo_priority;
    if (const int err = pthread_setschedparam(thread, SCHED_FIFO, &param); err != 0) {
        FLEET_WARN("thread '{}' SCHED_FIFO failed priority={} err={}", tuning.name, tuning.fifo_priority, err);
    }
}
#endif

} // namespace

void tune_current_thread(const thread_tuning &tuning) noexcept {
#ifdef __linux__
    const auto self = pthread_self();
    set_affinity(self, tuning);
    set_fifo(self, tuning);
    if (tuning.lock_memory && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        FLEET_WARN("thread '{}' mlockall failed", tuning.name);
    }
#else
    (void)tuning;
#endif
}

} // namespace fleet
