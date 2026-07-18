#include "thread_tuning.h"

#include "log.h"

#include <cerrno>
#include <string>
#include <system_error>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace fleet {
namespace {

err_code generic_error(int value) noexcept {
    return value == 0 ? err_code{} : err_code(value, std::generic_category());
}

#ifdef __linux__

#ifndef SCHED_DEADLINE
#define SCHED_DEADLINE 6
#endif

struct sched_attr {
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;
    int32_t sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};

err_code set_name(std::string_view name) noexcept {
    if (name.empty())
        return {};
    const std::string short_name{name.substr(0, 15)};
    return generic_error(pthread_setname_np(pthread_self(), short_name.c_str()));
}

err_code set_affinity(const linux_thread_config &config) noexcept {
    if (config.cpus.empty())
        return {};

    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    for (const auto cpu : config.cpus) {
        if (cpu < 0 || cpu >= CPU_SETSIZE)
            return generic_error(EINVAL);
        CPU_SET(cpu, &cpus);
    }
    if (const auto error = pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus))
        return generic_error(error);

    cpu_set_t effective;
    CPU_ZERO(&effective);
    if (const auto error = pthread_getaffinity_np(pthread_self(), sizeof(effective), &effective))
        return generic_error(error);
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (CPU_ISSET(cpu, &cpus) != CPU_ISSET(cpu, &effective))
            return generic_error(EINVAL);
    }
    return {};
}

err_code set_posix_policy(int policy, int priority) noexcept {
    sched_param parameter{};
    parameter.sched_priority = priority;
    return generic_error(pthread_setschedparam(pthread_self(), policy, &parameter));
}

err_code set_deadline(const linux_thread_config &config) noexcept {
    if (config.runtime <= std::chrono::nanoseconds::zero() ||
        config.runtime > config.deadline || config.deadline > config.period) {
        return generic_error(EINVAL);
    }

    sched_attr attribute{
        .size = sizeof(sched_attr),
        .sched_policy = SCHED_DEADLINE,
        .sched_flags = 0,
        .sched_nice = 0,
        .sched_priority = 0,
        .sched_runtime = static_cast<uint64_t>(config.runtime.count()),
        .sched_deadline = static_cast<uint64_t>(config.deadline.count()),
        .sched_period = static_cast<uint64_t>(config.period.count()),
    };
    if (syscall(SYS_sched_setattr, 0, &attribute, 0) == 0)
        return {};
    return generic_error(errno);
}

err_code set_policy(const linux_thread_config &config) noexcept {
    switch (config.policy) {
    case linux_scheduler_policy::inherit:
        return {};
    case linux_scheduler_policy::other:
        return set_posix_policy(SCHED_OTHER, 0);
    case linux_scheduler_policy::fifo:
        if (config.fifo_priority < sched_get_priority_min(SCHED_FIFO) ||
            config.fifo_priority > sched_get_priority_max(SCHED_FIFO)) {
            return generic_error(EINVAL);
        }
        return set_posix_policy(SCHED_FIFO, config.fifo_priority);
    case linux_scheduler_policy::deadline:
        return set_deadline(config);
    }
    return generic_error(EINVAL);
}

err_code verify_policy(const linux_thread_config &config) noexcept {
    if (config.policy == linux_scheduler_policy::inherit)
        return {};
    const auto effective = sched_getscheduler(0);
    if (effective < 0)
        return generic_error(errno);
    const int expected = config.policy == linux_scheduler_policy::other    ? SCHED_OTHER
                         : config.policy == linux_scheduler_policy::fifo  ? SCHED_FIFO
                                                                          : SCHED_DEADLINE;
    return effective == expected ? err_code{} : generic_error(EINVAL);
}

#endif

} // namespace

err_code tune_current_thread(const linux_thread_config &config, std::string_view name) noexcept {
#ifdef __linux__
    if (const auto error = set_name(name)) {
        FLEET_WARN("thread '{}' set name failed error={}", name, error.message());
        return error;
    }
    if (const auto error = set_affinity(config)) {
        FLEET_WARN("thread '{}' set affinity failed error={}", name, error.message());
        return error;
    }
    if (const auto error = set_policy(config)) {
        if (config.policy == linux_scheduler_policy::deadline) {
            FLEET_WARN("thread '{}' SCHED_DEADLINE failed error={}; require CAP_SYS_NICE and "
                       "an affinity mask spanning its scheduler root domain",
                       name, error.message());
        } else {
            FLEET_WARN("thread '{}' set scheduler failed policy={} error={}", name,
                       static_cast<int>(config.policy), error.message());
        }
        return error;
    }
    if (const auto error = verify_policy(config)) {
        FLEET_WARN("thread '{}' scheduler read-back mismatch error={}", name, error.message());
        return error;
    }
    if (config.lock_memory && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        const auto error = generic_error(errno);
        FLEET_WARN("thread '{}' mlockall failed error={}", name, error.message());
        return error;
    }
    return {};
#else
    (void)name;
    return config.configured() ? make_error_code(fleet_err::unsupported) : err_code{};
#endif
}

} // namespace fleet
