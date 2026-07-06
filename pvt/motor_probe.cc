#include "log.h"
#include "fleet/fleet.h"

#include <chrono>
#include <thread>

namespace fleet {
namespace {

constexpr uint16_t remote_port = 9999;
constexpr uint16_t local_port = 0;

constexpr milliseconds timeout = 200ms;

constexpr bool run_low_speed_velocity_control = true;
constexpr milliseconds velocity_control_duration = 15s;
constexpr milliseconds velocity_control_period = 200ms;
constexpr milliseconds velocity_timeout_settle = 500ms;
constexpr double velocity_command_rpm = 5.0;
constexpr float velocity_current_limit_a = 0.8f;
constexpr float pvt_velocity_radps = 0.0f;
constexpr float pvt_torque_nm = 0.0f;
constexpr double torque_command_nm = 0.0;
constexpr double current_command_a = 0.0;

const fleet_config probe_config{
    .channels =
        {
            {
                .name = "probe_201",
                .remote_address = "10.10.10.201",
                .remote_port = remote_port,
                .local_port = local_port,
            },
            {
                .name = "probe_202",
                .remote_address = "10.10.10.202",
                .remote_port = remote_port,
                .local_port = local_port,
            },
        },
    .motors =
        {
            {.name = "probe_201_can0", .channel = "probe_201", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 0x01},
            {.name = "probe_201_can1", .channel = "probe_201", .model = "EC-A2806-P2-36", .can_port = 1, .can_id = 0x01},
            {.name = "probe_202_can0", .channel = "probe_202", .model = "EC-A2806-P2-36", .can_port = 0, .can_id = 0x01},
            {.name = "probe_202_can1", .channel = "probe_202", .model = "EC-A2806-P2-36", .can_port = 1, .can_id = 0x01},
        },
};

const std::vector<fleet_config::motor_config> &probe_motors() { return probe_config.motors; }

const fleet_config::channel_config *find_channel(std::string_view name) {
    for (const auto &channel : probe_config.channels) {
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

void log_motor_section(const fleet_config::motor_config &probe_motor, device_id_t device_id) {
    log_separator("probe motor");

    if (const auto *channel = find_channel(probe_motor.channel)) {
        FLEET_INFO("channel name={} remote={}:{} local_port={}", channel->name,
                 channel->remote_address, channel->remote_port, channel->local_port);
    } else {
        FLEET_WARN("channel name={} not found", probe_motor.channel);
    }

    FLEET_INFO("motor name={} device={} can_port={} can_id=0x{:X}", probe_motor.name, device_id,
             probe_motor.can_port, probe_motor.can_id);
}

std::string_view query_name(config_query_code code) {
    using enum config_query_code;
    switch (code) {
    case position: return "position";
    case velocity: return "velocity";
    case current: return "current";
    case power: return "power";
    case acceleration: return "acceleration";
    case flux_observer_gain: return "flux_observer_gain";
    case disturbance_compensation_coeff: return "disturbance_compensation_coeff";
    case feedback_compensation_coeff: return "feedback_compensation_coeff";
    case damping_coeff: return "damping_coeff";
    case torque_ratio: return "torque_ratio";
    case pvt_kp_range: return "pvt_kp_range";
    case pvt_kd_range: return "pvt_kd_range";
    case pvt_position_limit_r: return "pvt_position_limit_r";
    case pvt_velocity_limit_radps: return "pvt_velocity_limit_radps";
    case pvt_torque_limit_nm: return "pvt_torque_limit_nm";
    case pvt_current_limit: return "pvt_current_limit";
    case mcu_uuid: return "mcu_uuid";
    case version: return "version";
    case can_timeout: return "can_timeout";
    case current_loop_pi: return "current_loop_pi";
    case velocity_loop_pi: return "velocity_loop_pi";
    case position_loop_pd: return "position_loop_pd";
    case brake_status: return "brake_status";
    }
    return "query";
}

bool print_query_result(const motor::motor_interface &motor,
                        const fleet_config::motor_config &probe_motor, config_query_code code) {
    using enum config_query_code;
    switch (code) {
    case version:
        if (const auto feedback = motor.version()) {
            const auto &v = feedback.value;
			FLEET_DEBUG_KV(
					KV("msg", "probe ok"),
					KV("id", probe_motor.can_id),
					KV("port", probe_motor.can_port),
					KV("version", "=>"),
					KV("hw", fmt::format("{}.{}.{}", v.hw_major, v.hw_minor, v.hw_patch)),
					KV("fw", fmt::format("{}.{}.{}\n", v.fw_major, v.fw_minor, v.fw_patch))
					);
            return true;
        }
        break;
	case position:
		if (const auto feedback = motor.position_deg()) {
			FLEET_DEBUG_KV(
				KV("msg", "probe ok"),
				KV("id", probe_motor.can_id),
				KV("port", probe_motor.can_port),
				KV("position_deg", feedback.value)
			);
			return true;
		}
		break;

	case velocity:
		if (const auto feedback = motor.velocity_rpm()) {
			FLEET_DEBUG_KV(
				KV("msg", "probe ok"),
				KV("id", probe_motor.can_id),
				KV("port", probe_motor.can_port),
				KV("velocity_rpm", feedback.value)
			);
			return true;
		}
		break;

	case current:
		if (const auto feedback = motor.current()) {
			FLEET_DEBUG_KV(
				KV("msg", "probe ok"),
				KV("id", probe_motor.can_id),
				KV("port", probe_motor.can_port),
				KV("current_a", feedback.value)
			);
			return true;
		}
		break;

	case power:
		if (const auto feedback = motor.power()) {
			FLEET_DEBUG_KV(
				KV("msg", "probe ok"),
				KV("id", probe_motor.can_id),
				KV("port", probe_motor.can_port),
				KV("power_w", feedback.value)
			);
			return true;
		}
		break;

	case can_timeout:
		if (const auto feedback = motor.can_timeout()) {
			FLEET_DEBUG_KV(
				KV("msg", "probe ok"),
				KV("id", probe_motor.can_id),
				KV("port", probe_motor.can_port),
				KV("can_timeout_ms", feedback.value.count())
			);
			return true;
		}
		break;

	case pvt_position_limit_r:
		if (const auto feedback = motor.pvt_position_limit_r()) {
			FLEET_DEBUG_KV(
				KV("msg", "probe ok"),
				KV("id", probe_motor.can_id),
				KV("port", probe_motor.can_port),
				KV("pvt_min", feedback.value.min),
				KV("pvt_max", feedback.value.max)
			);
			return true;
		}
		break;

	case brake_status:
		if (const auto feedback = motor.brake_status()) {
			FLEET_DEBUG_KV(
				KV("msg", "probe ok"),
				KV("id", probe_motor.can_id),
				KV("port", probe_motor.can_port),
				KV("brake_engaged", feedback.value)
			);
			return true;
		}
		break;

	default:
		break;
	}

	FLEET_DEBUG_KV(
		KV("msg", "probe fail"),
		KV("id", probe_motor.can_id),
		KV("port", probe_motor.can_port),
		KV("query", query_name(code)),
		KV("reason", "missing expected field")
	);

	return false;
}

bool run_query(const std::shared_ptr<motor::motor_interface> &motor,
               const fleet_config::motor_config &probe_motor, config_query_code code) {
    FLEET_INFO("probe send query {} id=0x{:X} port={}", query_name(code), probe_motor.can_id,
             probe_motor.can_port);
    if (auto err = command(config_query_command{
            .device_id = motor->id(),
            .code = code,
        })) {
        FLEET_WARN("probe fail id=0x{:X} port={} query={} error={}\n", probe_motor.can_id,
                 probe_motor.can_port, query_name(code), err.message());
        return false;
    }

    std::this_thread::sleep_for(timeout);
    return print_query_result(*motor, probe_motor, code);
}

bool run_can_timeout_config_probe(const std::shared_ptr<motor::motor_interface> &motor,
                                  const fleet_config::motor_config &probe_motor) {
    if (!run_query(motor, probe_motor, config_query_code::can_timeout)) {
        return false;
    }

    const auto original = motor->can_timeout();
    if (!original) {
        return false;
    }

    const auto temporary = original.value == 1000ms ? 1200ms : 1000ms;
    FLEET_INFO("probe set config id=0x{:X} port={} can_timeout_ms={}", probe_motor.can_id,
             probe_motor.can_port, temporary.count());
    if (auto err = command(config_set_command{
            .device_id = motor->id(),
            .code = config_set_code::can_timeout,
            .value = temporary,
        })) {
        FLEET_WARN("probe fail set can_timeout error={}\n", err.message());
        return false;
    }

    if (!run_query(motor, probe_motor, config_query_code::can_timeout)) {
        return false;
    }
    if (const auto updated = motor->can_timeout(); !updated || updated.value != temporary) {
        FLEET_WARN("probe fail id=0x{:X} port={} can_timeout verify failed\n", probe_motor.can_id,
                 probe_motor.can_port);
        return false;
    }

    FLEET_INFO("probe restore config id=0x{:X} port={} can_timeout_ms={}", probe_motor.can_id,
             probe_motor.can_port, original.value.count());
    if (auto err = command(config_set_command{
            .device_id = motor->id(),
            .code = config_set_code::can_timeout,
            .value = original.value,
        })) {
        FLEET_WARN("probe fail restore can_timeout error={}\n", err.message());
        return false;
    }

    if (!run_query(motor, probe_motor, config_query_code::can_timeout)) {
        return false;
    }
    if (const auto restored = motor->can_timeout(); !restored || restored.value != original.value) {
        FLEET_WARN("probe fail id=0x{:X} port={} can_timeout restore verify failed\n",
                 probe_motor.can_id, probe_motor.can_port);
        return false;
    }
    return true;
}

bool send_velocity_batch(std::span<const std::shared_ptr<motor::motor_interface>> bound_motors) {
    std::vector<velocity_command> commands;
    commands.reserve(bound_motors.size());
    for (const auto &motor : bound_motors) {
        commands.push_back({
            .device_id = motor->id(),
            .velocity_rpm = velocity_command_rpm,
            .current_limit = velocity_current_limit_a,
        });
    }

    if (auto err = command(std::span<const velocity_command>(commands))) {
        FLEET_WARN("probe fail send velocity batch error={}", err.message());
        return false;
    }
    return true;
}

bool send_pvt_batch(std::span<const std::shared_ptr<motor::motor_interface>> bound_motors) {
    std::vector<pvt_command> commands;
    commands.reserve(bound_motors.size());
    for (const auto &motor : bound_motors) {
        commands.push_back(pvt_command{
            .device_id = motor->id(),
            .gains = pid_gains{},
            .position_r = 0.0f,
            .velocity_radps = pvt_velocity_radps,
            .torque_ff_nm = pvt_torque_nm,
        });
    }

    if (auto err = command(std::span<const pvt_command>(commands))) {
        FLEET_WARN("probe fail send pvt batch error={}", err.message());
        return false;
    }
    return true;
}

bool send_position_batch(std::span<const std::shared_ptr<motor::motor_interface>> bound_motors) {
    std::vector<position_command> commands;
    commands.reserve(bound_motors.size());
    for (const auto &motor : bound_motors) {
        const auto position = motor->position_deg();
        commands.push_back(position_command{
            .device_id = motor->id(),
            .position_deg = position ? position.value : 0.0f,
            .velocity_limit_rpm = velocity_command_rpm,
            .current_limit = velocity_current_limit_a,
        });
    }

    if (auto err = command(std::span<const position_command>(commands))) {
        FLEET_WARN("probe fail send position batch error={}\n", err.message());
        return false;
    }
    return true;
}

bool send_torque_batch(std::span<const std::shared_ptr<motor::motor_interface>> bound_motors) {
    std::vector<torque_command> commands;
    commands.reserve(bound_motors.size());
    for (const auto &motor : bound_motors) {
        commands.push_back({
            .device_id = motor->id(),
            .torque_nm = torque_command_nm,
        });
    }

    if (auto err = command(std::span<const torque_command>(commands))) {
        FLEET_WARN("probe fail send torque batch error={}\n", err.message());
        return false;
    }
    return true;
}

bool send_current_batch(std::span<const std::shared_ptr<motor::motor_interface>> bound_motors) {
    std::vector<current_command> commands;
    commands.reserve(bound_motors.size());
    for (const auto &motor : bound_motors) {
        commands.push_back({
            .device_id = motor->id(),
            .current = current_command_a,
        });
    }

    if (auto err = command(std::span<const current_command>(commands))) {
        FLEET_WARN("probe fail send current batch error={}\n", err.message());
        return false;
    }
    return true;
}

bool send_brake_batch(std::span<const std::shared_ptr<motor::motor_interface>> bound_motors) {
    std::vector<brake_command> commands;
    commands.reserve(bound_motors.size());
    for (const auto &motor : bound_motors) {
        commands.push_back(brake_command{.device_id = motor->id(),
                                         .control = engage_brake{
                                             .open = false,
                                         }});
    }

    if (auto err = command(std::span<const brake_command>(commands))) {
        FLEET_WARN("probe fail send brake batch error={}\n", err.message());
        return false;
    }
    return true;
}

bool run_control_command_probe(
    std::span<const std::shared_ptr<motor::motor_interface>> bound_motors) {
    log_separator("probe control command smoke test");
    FLEET_WARN("probe control command smoke test can move motors briefly");

    bool ok = true;
    FLEET_INFO("probe send pvt batch");
    ok &= send_pvt_batch(bound_motors);
    std::this_thread::sleep_for(velocity_control_period);

    FLEET_INFO("probe send position batch");
    ok &= send_position_batch(bound_motors);
    std::this_thread::sleep_for(velocity_control_period);

    FLEET_INFO("probe send velocity batch rpm={} current_limit_a={}", velocity_command_rpm,
             velocity_current_limit_a);
    ok &= send_velocity_batch(bound_motors);
    std::this_thread::sleep_for(velocity_control_period);

    FLEET_INFO("probe send torque batch torque_nm={}", torque_command_nm);
    ok &= send_torque_batch(bound_motors);
    std::this_thread::sleep_for(velocity_control_period);

    FLEET_INFO("probe send current batch current_a={}", current_command_a);
    ok &= send_current_batch(bound_motors);
    std::this_thread::sleep_for(velocity_control_period);

    FLEET_INFO("probe send brake release batch");
    ok &= send_brake_batch(bound_motors);
    return ok;
}

bool run_low_speed_velocity_probe(
    std::span<const std::shared_ptr<motor::motor_interface>> bound_motors) {
    log_separator("probe low-speed velocity control");
    FLEET_WARN("probe low-speed velocity control can move motors rpm={}", velocity_command_rpm);

    const auto start = std::chrono::steady_clock::now();

    FLEET_INFO("probe run low-speed velocity duration_ms={} period_ms={} rpm={} current_limit_a={}",
             velocity_control_duration.count(), velocity_control_period.count(),
             velocity_command_rpm, velocity_current_limit_a);

    while (std::chrono::steady_clock::now() - start < velocity_control_duration) {
        if (!send_velocity_batch(bound_motors)) {
            return false;
        }
        std::this_thread::sleep_for(velocity_control_period);
    }

    FLEET_INFO("probe stop sending velocity commands and wait for command timeout");

    std::this_thread::sleep_for(velocity_timeout_settle);

    bool ok = true;
    const auto &configured_motors = probe_motors();
    for (size_t i = 0; i < bound_motors.size(); ++i) {
        ok &= run_query(bound_motors[i], configured_motors[i], config_query_code::velocity);
    }
    return ok;
}

int run() {
    log_separator("probe init");
    FLEET_INFO("probe init channels=10.10.10.201:{},10.10.10.202:{} local_port={}",
             remote_port, remote_port, local_port);

    if (auto err = init(probe_config)) {
        FLEET_ERROR("probe init failed error={}", err.message());
        return 1;
    }

    auto bound_motors = lookup<motor_interface>();
    const auto &configured_motors = probe_motors();
    if (bound_motors.size() != configured_motors.size()) {
        FLEET_ERROR("probe expected motors={} got={}", configured_motors.size(), bound_motors.size());
        shutdown();
        return 1;
    }

    bool ok = true;
    for (size_t i = 0; i < bound_motors.size(); ++i) {
        const auto &probe_motor = configured_motors[i];
        log_motor_section(probe_motor, bound_motors[i]->id());
        ok &= run_query(bound_motors[i], probe_motor, config_query_code::version);
        ok &= run_query(bound_motors[i], probe_motor, config_query_code::position);
        ok &= run_query(bound_motors[i], probe_motor, config_query_code::velocity);
        ok &= run_query(bound_motors[i], probe_motor, config_query_code::current);
        ok &= run_query(bound_motors[i], probe_motor, config_query_code::power);
        ok &= run_query(bound_motors[i], probe_motor, config_query_code::brake_status);
        ok &= run_query(bound_motors[i], probe_motor, config_query_code::can_timeout);
        ok &= run_query(bound_motors[i], probe_motor, config_query_code::pvt_position_limit_r);
        ok &= run_can_timeout_config_probe(bound_motors[i], probe_motor);
    }

    if (run_low_speed_velocity_control) {
        ok &= run_control_command_probe(
            std::span<const std::shared_ptr<motor::motor_interface>>(bound_motors));
        ok &= run_low_speed_velocity_probe(
            std::span<const std::shared_ptr<motor::motor_interface>>(bound_motors));
    }

    shutdown();

    if (ok) {
        FLEET_INFO("probe passed");
    } else {
        FLEET_ERROR("probe failed");
    }
    return ok ? 0 : 1;
}

} // namespace
} // namespace fleet

int main() {
    try {
        return fleet::run();
    } catch (const std::exception &e) {
        FLEET_ERROR("probe fatal {}", e.what());
        fleet::shutdown();
        return 1;
    }
}
