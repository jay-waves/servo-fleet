#include "log.h"
#include "fleet/fleet.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

namespace fleet {
namespace {

constexpr uint16_t remote_port = 9999;
constexpr uint16_t local_port = 0;
constexpr can_id_t can_id = 0x01;

constexpr float pi = 3.14159265358979323846f;
constexpr float deg_to_rad = pi / 180.0f;
constexpr float rad_to_deg = 180.0f / pi;
constexpr float pvt_position_limit_r = 12.5f;

constexpr milliseconds command_period = 10ms;
constexpr milliseconds query_wait = 100ms;
constexpr milliseconds feedback_max_age = 250ms;
constexpr milliseconds startup_wait = 300ms;
constexpr milliseconds settle_duration = 1500ms;
constexpr milliseconds tune_duration = 12000ms;

constexpr float pvt_kp = 18.0f;
constexpr float pvt_kd = 0.4f;
constexpr float tune_amplitude_deg = 3.0f;
constexpr float tune_frequency_hz = 0.1f;
constexpr float max_velocity_radps = 0.35f;
constexpr float pvt_current_limit_a = 0.8f;
constexpr float feedback_current_warn_a = 1.0f;
constexpr float feedback_velocity_warn_rpm = 10000.0f;
constexpr float torque_ff_nm = 0.0f;
constexpr float stop_torque_nm = 0.0f;

const fleet_config pvt_config{
    .channels =
        {
            {
                .name = "pvt_201",
                .remote_address = "10.10.10.201",
                .remote_port = remote_port,
                .local_port = local_port,
            },
            {
                .name = "pvt_202",
                .remote_address = "10.10.10.202",
                .remote_port = remote_port,
                .local_port = local_port,
            },
        },
    .motors =
        {
            {.name = "pvt_201_can0", .channel = "pvt_201", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = can_id},
            {.name = "pvt_201_can1", .channel = "pvt_201", .model = "EC-A2806-P2-36", .can_port = 1, .can_id = can_id},
            {.name = "pvt_202_can0", .channel = "pvt_202", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = can_id},
            {.name = "pvt_202_can1", .channel = "pvt_202", .model = "EC-A2806-P2-36", .can_port = 1, .can_id = can_id},
        },
};

using motor_ptr = std::shared_ptr<motor::motor_interface>;

const fleet_config::channel_config *find_channel(std::string_view name) {
    for (const auto &channel : pvt_config.channels) {
        if (channel.name == name) {
            return &channel;
        }
    }
    return nullptr;
}

void log_separator(std::string_view title) {
    FLEET_INFO("");
    FLEET_INFO("========== {} ==========", title);
}

void log_motor_section(const fleet_config::motor_config &config, device_id_t device_id) {
    log_separator("pvt tune motor");

    if (const auto *channel = find_channel(config.channel)) {
        FLEET_INFO("channel name={} remote={}:{} local_port={}", channel->name,
                 channel->remote_address, channel->remote_port, channel->local_port);
    } else {
        FLEET_WARN("channel name={} not found", config.channel);
    }

    FLEET_INFO("motor name={} device={} can_port={} can_id=0x{:X}", config.name, device_id,
             config.can_port, config.can_id);
}

bool valid_pvt_position(float position_r) {
    return std::isfinite(position_r) && position_r >= -pvt_position_limit_r &&
           position_r <= pvt_position_limit_r;
}

std::optional<float> checked_pvt_position(float position_r, std::string_view context) {
    if (valid_pvt_position(position_r)) {
        return position_r;
    }

    FLEET_WARN("pvt tune invalid {} position_deg={} outside pvt command range", context,
             position_r * rad_to_deg);
    return std::nullopt;
}

std::optional<float> fresh_pvt_position_r(const motor_ptr &motor, std::string_view context) {
    const auto now = std::chrono::steady_clock::now();
    const auto position = motor->position_deg();
    if (!position.fresh(now, feedback_max_age)) {
        FLEET_WARN("pvt tune stale {} position feedback", context);
        return std::nullopt;
    }

    return checked_pvt_position(position.value * deg_to_rad, context);
}

void report_feedback_health(float position_deg, bool position_fresh, float velocity_rpm,
                            bool velocity_fresh, float current_a, bool current_fresh) {
    if (!position_fresh) {
        FLEET_WARN("pvt tune abnormal feedback: position stale or missing");
    } else if (!valid_pvt_position(position_deg * deg_to_rad)) {
        FLEET_WARN("pvt tune abnormal feedback: position_deg={} outside pvt range", position_deg);
    }

    if (!velocity_fresh) {
        FLEET_WARN("pvt tune abnormal feedback: velocity stale or missing");
    } else if (!std::isfinite(velocity_rpm) ||
               std::abs(velocity_rpm) > feedback_velocity_warn_rpm) {
        FLEET_WARN("pvt tune abnormal feedback: velocity_rpm={}", velocity_rpm);
    }

    if (!current_fresh) {
        FLEET_WARN("pvt tune abnormal feedback: current stale or missing");
    } else if (!std::isfinite(current_a) || std::abs(current_a) >= feedback_current_warn_a) {
        FLEET_WARN("pvt tune abnormal feedback: current_a={}", current_a);
    }
}

bool send_query(const motor_ptr &motor, config_query_code code) {
    if (auto err = command(config_query_command{
            .device_id = motor->id(),
            .code = code,
        })) {
        FLEET_WARN("pvt tune query failed code={} error={}", static_cast<int>(code), err.message());
        return false;
    }

    std::this_thread::sleep_for(query_wait);
    return true;
}

std::optional<float> current_position_r(const motor_ptr &motor) {
    FLEET_INFO("pvt tune query origin position");
    if (send_query(motor, config_query_code::position)) {
        return fresh_pvt_position_r(motor, "origin");
    }
    return std::nullopt;
}

void print_feedback(const motor_ptr &motor) {
    FLEET_INFO("pvt tune query feedback position/velocity/current");
    (void)send_query(motor, config_query_code::position);
    (void)send_query(motor, config_query_code::velocity);
    (void)send_query(motor, config_query_code::current);

    const auto position = motor->position_deg();
    const auto velocity = motor->velocity_rpm();
    const auto current = motor->current();
    const auto now = std::chrono::steady_clock::now();
    const auto position_fresh = position.fresh(now, feedback_max_age);
    const auto velocity_fresh = velocity.fresh(now, feedback_max_age);
    const auto current_fresh = current.fresh(now, feedback_max_age);
    const auto position_value = position_fresh ? position.value : 0.0f;
    const auto velocity_value = velocity_fresh ? velocity.value : 0.0f;
    const auto current_value = current_fresh ? current.value : 0.0f;

    FLEET_INFO("pvt tune feedback position_deg={} position_fresh={} velocity_rpm={} "
             "velocity_fresh={} current_a={} current_fresh={}",
             position_value, position_fresh, velocity_value, velocity_fresh, current_value,
             current_fresh);
    report_feedback_health(position_value, position_fresh, velocity_value, velocity_fresh,
                           current_value, current_fresh);
}

void print_tracking_error(const motor_ptr &motor, float target_position_r,
                          float target_velocity_radps) {
    const auto position = motor->position_deg();
    const auto velocity = motor->velocity_rpm();
    const auto current = motor->current();
    const auto now = std::chrono::steady_clock::now();
    const auto position_fresh = position.fresh(now, feedback_max_age);
    const auto velocity_fresh = velocity.fresh(now, feedback_max_age);
    const auto current_fresh = current.fresh(now, feedback_max_age);

    const auto actual_position_r =
        position_fresh
            ? checked_pvt_position(position.value * deg_to_rad, "feedback").value_or(target_position_r)
            : target_position_r;
    const auto actual_velocity_radps =
        velocity_fresh ? velocity.value * 2.0f * pi / 60.0f : 0.0f;
    const auto position_error_r = target_position_r - actual_position_r;
    const auto velocity_error_radps = target_velocity_radps - actual_velocity_radps;

    FLEET_INFO("pvt tune tracking target_deg={} actual_deg={} error_deg={} target_radps={} "
             "actual_radps={} error_radps={} current_a={} sample_fresh=pos:{} vel:{} cur:{}",
             target_position_r * rad_to_deg, actual_position_r * rad_to_deg,
             position_error_r * rad_to_deg, target_velocity_radps, actual_velocity_radps,
             velocity_error_radps, current_fresh ? current.value : 0.0f, position_fresh,
             velocity_fresh, current_fresh);
}

void print_pvt_limits(const motor_ptr &motor) {
    FLEET_INFO("pvt tune query pvt limits");
    (void)send_query(motor, config_query_code::pvt_kp_range);
    (void)send_query(motor, config_query_code::pvt_kd_range);
    (void)send_query(motor, config_query_code::pvt_position_limit_r);
    (void)send_query(motor, config_query_code::pvt_velocity_limit_radps);
    (void)send_query(motor, config_query_code::pvt_torque_limit_nm);

    const auto now = std::chrono::steady_clock::now();
    if (const auto range = motor->pvt_kp_range(); range.fresh(now, feedback_max_age)) {
        FLEET_INFO("pvt tune kp range min={} max={}", range.value.min, range.value.max);
    } else {
        FLEET_WARN("pvt tune kp range stale or missing");
    }
    if (const auto range = motor->pvt_kd_range(); range.fresh(now, feedback_max_age)) {
        FLEET_INFO("pvt tune kd range min={} max={}", range.value.min, range.value.max);
    } else {
        FLEET_WARN("pvt tune kd range stale or missing");
    }
    if (const auto range = motor->pvt_position_limit_r(); range.fresh(now, feedback_max_age)) {
        FLEET_INFO("pvt tune position limit rad min={} max={}", range.value.min, range.value.max);
    } else {
        FLEET_WARN("pvt tune position limit stale or missing");
    }
    if (const auto range = motor->pvt_velocity_limit_radps(); range.fresh(now, feedback_max_age)) {
        FLEET_INFO("pvt tune velocity limit radps min={} max={}", range.value.min, range.value.max);
    } else {
        FLEET_WARN("pvt tune velocity limit stale or missing");
    }
    if (const auto range = motor->pvt_torque_limit_nm(); range.fresh(now, feedback_max_age)) {
        FLEET_INFO("pvt tune torque limit nm min={} max={}", range.value.min, range.value.max);
    } else {
        FLEET_WARN("pvt tune torque limit stale or missing");
    }
}

std::optional<range_t<float>> set_pvt_current_limit(const motor_ptr &motor) {
    FLEET_INFO("pvt tune query original pvt current limit");
    (void)send_query(motor, config_query_code::pvt_current_limit);
    const auto original = motor->pvt_current_limit();
    const auto now = std::chrono::steady_clock::now();
    if (!original.fresh(now, feedback_max_age)) {
        FLEET_WARN("pvt tune read current limit stale or missing");
        return std::nullopt;
    }

    FLEET_INFO("pvt tune set pvt current limit min={} max={}", -pvt_current_limit_a,
             pvt_current_limit_a);
    if (auto err = command(config_set_command{
            .device_id = motor->id(),
            .code = config_set_code::pvt_current_limit,
            .value = range_t<float>{.min = -pvt_current_limit_a, .max = pvt_current_limit_a},
        })) {
        FLEET_WARN("pvt tune set current limit failed error={}", err.message());
        return std::nullopt;
    }

    std::this_thread::sleep_for(query_wait);
    return original.value;
}

void restore_pvt_current_limit(const motor_ptr &motor, const std::optional<range_t<float>> &limit) {
    if (!motor || !limit) {
        return;
    }

    FLEET_INFO("pvt tune restore pvt current limit min={} max={}", limit->min, limit->max);
    if (auto err = command(config_set_command{
            .device_id = motor->id(),
            .code = config_set_code::pvt_current_limit,
            .value = limit.value(),
        })) {
        FLEET_WARN("pvt tune restore current limit failed error={}", err.message());
    }
}

bool release_brake(const motor_ptr &motor) {
    FLEET_INFO("pvt tune release brake");
    if (auto err = command(brake_command{
            .device_id = motor->id(),
            .control = engage_brake{.open = true},
            .reply = reply_mode::status1,
        })) {
        FLEET_WARN("pvt tune release brake failed error={}", err.message());
        return false;
    }

    std::this_thread::sleep_for(startup_wait);
    return true;
}

bool send_pvt(const motor_ptr &motor, float position_r, float velocity_radps) {
    if (!valid_pvt_position(position_r)) {
        FLEET_WARN("pvt tune skip invalid command position_deg={}", position_r * rad_to_deg);
        return false;
    }

    if (auto err = command(pvt_command{
            .device_id = motor->id(),
            .gains = pid_gains{.kp = pvt_kp, .kd = pvt_kd, .ki = 0.0f},
            .position_r = position_r,
            .velocity_radps = velocity_radps,
            .torque_ff_nm = torque_ff_nm,
        })) {
        FLEET_WARN("pvt tune command failed error={}", err.message());
        return false;
    }
    return true;
}

bool hold_position(const motor_ptr &motor, float position_r, milliseconds duration) {
    FLEET_INFO("pvt tune hold position duration_ms={} position_deg={}", duration.count(),
             position_r * rad_to_deg);
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
        if (!send_pvt(motor, position_r, 0.0f)) {
            return false;
        }
        std::this_thread::sleep_for(command_period);
    }
    return true;
}

bool run_pvt_tuning(const motor_ptr &motor) {
    log_separator("pvt tune run trajectory");
    const auto origin = current_position_r(motor);
    if (!origin) {
        FLEET_WARN("pvt tune abort: origin position is not usable for pvt command");
        return false;
    }
    const auto origin_r = *origin;
    const auto amplitude_r = tune_amplitude_deg * deg_to_rad;
    const auto omega = 2.0f * pi * tune_frequency_hz;

    FLEET_WARN("pvt tune starts: motor can move. origin_deg={} kp={} kd={} amplitude_deg={} "
             "frequency_hz={} duration_ms={} current_limit_a={}",
             origin_r * rad_to_deg, pvt_kp, pvt_kd, tune_amplitude_deg, tune_frequency_hz,
             tune_duration.count(), pvt_current_limit_a);

    if (!hold_position(motor, origin_r, settle_duration)) {
        return false;
    }

    const auto start = std::chrono::steady_clock::now();
    auto last_print = start;
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = now - start;
        if (elapsed >= tune_duration) {
            break;
        }

        const auto t = std::chrono::duration<float>(elapsed).count();
        const auto position = origin_r + amplitude_r * std::sin(omega * t);
        const auto velocity = std::clamp(amplitude_r * omega * std::cos(omega * t),
                                         -max_velocity_radps, max_velocity_radps);
        if (!send_pvt(motor, position, velocity)) {
            return false;
        }

        if (now - last_print >= 1s) {
            print_tracking_error(motor, position, velocity);
            last_print = now;
        }

        std::this_thread::sleep_for(command_period);
    }

    FLEET_INFO("pvt tune return to origin position_deg={}", origin_r * rad_to_deg);
    return hold_position(motor, origin_r, settle_duration);
}

void stop_motor(const motor_ptr &motor) {
    if (!motor) {
        return;
    }

    FLEET_INFO("pvt tune stop motor torque_nm={}", stop_torque_nm);
    if (auto err = command(torque_command{
            .device_id = motor->id(),
            .torque_nm = stop_torque_nm,
            .reply = reply_mode::status1,
        })) {
        FLEET_WARN("pvt tune stop torque failed error={}", err.message());
    }
}

int run() {
    log_separator("pvt tune init");
    FLEET_INFO("pvt tune init channels=10.10.10.201:{},10.10.10.202:{} local_port={} can_id=0x{:X}",
             remote_port, remote_port, local_port, can_id);

    if (auto err = init(pvt_config)) {
        FLEET_ERROR("pvt tune init failed error={}", err.message());
        return 1;
    }

    const auto motors = lookup<motor_interface>();
    if (motors.size() != pvt_config.motors.size()) {
        FLEET_ERROR("pvt tune expected motors={} got={}", pvt_config.motors.size(), motors.size());
        shutdown();
        return 1;
    }

    bool ok = true;
    for (size_t i = 0; i < motors.size(); ++i) {
        const auto &motor = motors[i];
        const auto &config = pvt_config.motors[i];
        log_motor_section(config, motor->id());

        log_separator("pvt tune prepare");
        print_pvt_limits(motor);
        const auto original_current_limit = set_pvt_current_limit(motor);
        print_feedback(motor);

        auto motor_ok = original_current_limit.has_value();
        motor_ok &= release_brake(motor);
        if (motor_ok) {
            motor_ok &= run_pvt_tuning(motor);
        }

        log_separator("pvt tune cleanup");
        stop_motor(motor);
        restore_pvt_current_limit(motor, original_current_limit);
        print_feedback(motor);
        ok &= motor_ok;
    }

    shutdown();

    if (ok) {
        FLEET_INFO("pvt tune passed");
    } else {
        FLEET_ERROR("pvt tune failed");
    }
    return ok ? 0 : 1;
}

} // namespace
} // namespace fleet

int main() {
    try {
        return fleet::run();
    } catch (const std::exception &e) {
        FLEET_ERROR("pvt tune fatal {}", e.what());
        fleet::shutdown();
        return 1;
    }
}
