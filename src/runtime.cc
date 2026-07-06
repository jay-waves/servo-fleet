#include "runtime.h"

#include "log.h"
#include "thread_tuning.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <utility>

namespace fleet {
namespace {

using std::span;
using std::string_view;
using std::vector;

template <class T> std::optional<uint32_t> find_driver_slot(const box_t<T> &drivers, string_view channel_name) {
    const auto driver = std::ranges::find(drivers, channel_name, [](const auto &item) { return item->channel_name(); });
    if (driver == drivers.end())
        return std::nullopt;
    return static_cast<uint32_t>(driver - drivers.begin());
}

template <class T> bool uses_channel(const T &devices, string_view channel_name) {
    return std::ranges::any_of(
        devices, [&](const auto &device) { return device.channel == channel_name; });
}

bool channel_exists(const fleet_config &config, string_view channel_name) {
    return std::ranges::any_of(
        config.channels, [&](const auto &channel) { return channel.name == channel_name; });
}

size_t device_name_count(const fleet_config &config, string_view name) {
    return std::ranges::count(config.motors, name, [](const auto &device) { return device.name; });
}

err_code validate_device(const fleet_config &config, string_view name, string_view channel_name) {
    if (name.empty() || device_name_count(config, name) != 1) {
        FLEET_WARN("runtime duplicate or empty device name='{}'", name);
        return fleet_err::invalid_argument;
    }

    if (!channel_exists(config, channel_name)) {
        FLEET_WARN("runtime device name='{}' has invalid channel='{}'", name, channel_name);
        return fleet_err::invalid_argument;
    }
    return {};
}

} // namespace

runtime_t::runtime_t(fleet_config options) : options_(std::move(options)) {}

runtime_t::~runtime_t() {
	FLEET_DEBUG_KV(
			KV("msg", "Runtime Shutdown"),
			KV("motors", motors_.size())
			);
    for (auto &driver : motor_drivers_) driver->stop();
}

err_code runtime_t::init() {
    try {
        if (auto error = validate_config())
            return error;
        init_drivers();
        if (auto error = bind_motors())
            return error;
    } catch (...) {
        FLEET_ERROR("runtime init failed");
        return fleet_err::io_error;
    }

    log_config();
    return {};
}

err_code runtime_t::validate_config() const {
    for (const auto &channel : options_.channels) {
        const auto name_count = std::ranges::count(
            options_.channels, channel.name, [](const auto &item) { return item.name; });
        if (channel.name.empty() || name_count != 1)
            return fleet_err::invalid_argument;
    }

    for (const auto &motor : options_.motors) {
        if (auto error = validate_device(options_, motor.name, motor.channel))
            return error;

        const auto address_count = std::ranges::count_if(options_.motors, [&](const auto &item) {
            return item.channel == motor.channel && item.can_port == motor.can_port &&
                   item.can_id == motor.can_id;
        });
        if (motor.can_id > 0x7FFU || address_count != 1) {
            return fleet_err::invalid_argument;
        }
    }

    for (const auto &channel : options_.channels) {
        if (uses_channel(options_.motors, channel.name) &&
            (channel.max_bandwidth_bps == 0 || channel.send_queue_slots == 0))
            return fleet_err::invalid_argument;
    }

    return {};
}

void runtime_t::init_drivers() {
    for (const auto &channel : options_.channels) {
        auto is_this_channel = [&channel](const auto& dev) { return dev.channel == channel.name; };

        if (std::ranges::any_of(options_.motors, is_this_channel)) {
            motor_drivers_.push_back(std::make_unique<motor_driver>(channel));
        }
    }
}

err_code runtime_t::bind_motors() {
    for (const auto &config : options_.motors) {
        const auto driver_slot = find_driver_slot(motor_drivers_, config.channel);
        if (!driver_slot)
            return fleet_err::invalid_argument;
        if (find_motor_ec_spec(config.model) == nullptr)
            return fleet_err::invalid_argument;

        const auto device_id = static_cast<device_id_t>(device_bindings_.size());
        auto device = std::make_shared<motor_device>(device_id, config);

        const auto device_slot = motor_drivers_[*driver_slot]->bind_motor(*device);
        motors_.push_back(std::move(device));
        device_bindings_.push_back({
            .driver_slot = *driver_slot,
            .device_slot = device_slot,
        });
    }
    return {};
}

err_code runtime_t::start() {
    try {
        if (options_.command_cpu >= 0) {
            tune_current_thread({
                .cpu = options_.command_cpu,
                .fifo_priority = options_.command_fifo_priority,
                .lock_memory = options_.lock_memory,
                .name = "fleet command",
            });
        }
        for (auto &driver : motor_drivers_) driver->start();
    } catch (...) {
        for (auto &driver : motor_drivers_) driver->stop();
        return fleet_err::io_error;
    }
    return {};
}

err_code runtime_t::discard_pending_commands() noexcept {
    for (auto &driver : motor_drivers_) { 
		driver->discard_pending_sends(); 
	}

	return fleet_err::ok;
}

err_code runtime_t::update_net_control(const net_control_command &control) {
    (void)control;
    return {};
}

void runtime_t::log_config() const {
	FLEET_DEBUG_KV(
			KV("msg", "Runtime Config"),
			KV("motors", motors_.size())
			);
    for (size_t index = 0; index < options_.channels.size(); ++index) {
		const auto& ch = options_.channels[index];
		FLEET_DEBUG_KV(
				KV("msg", "Runtime Config Channel:"),
				KV("id", index),
				KV("name", ch.name),
				KV("remote_addr", ch.remote_address),
				KV("remote_port", ch.remote_port),
				KV("max_bandwidth_mbps", ch.max_bandwidth_bps / 1000),
				KV("send_queue_slots", ch.send_queue_slots)
				);
    }
}

const runtime_t::device_binding *runtime_t::find_device_binding(device_id_t device_id) const noexcept {
    return device_id < device_bindings_.size() ? &device_bindings_[device_id] : nullptr;
}

template <Command T> err_code runtime_t::send_command(const T &command) {
    if constexpr (std::same_as<std::remove_cvref_t<T>, discard_pending_command>) {
        return discard_pending_commands();
    } else if constexpr (std::same_as<std::remove_cvref_t<T>, net_control_command>) {
        return update_net_control(command);
    } else {
        return send_commands(std::span<const T>(&command, 1));
    }
}

template <Command T> err_code runtime_t::send_commands(std::span<const T> commands) {
    if constexpr (MotorCommand<T>) {
        return send_motor_commands(commands);
    } else {
        return fleet_err::unsupported;
    }
}

template <MotorCommand T> err_code runtime_t::send_motor_commands(std::span<const T> commands) {
    vector<typename motor_driver::batch_commands<T>> batches;
    batches.reserve(motor_drivers_.size());
    for (auto &driver : motor_drivers_)
        batches.emplace_back(*driver);

    err_code error;
    for (const auto &command : commands) {
        const auto *binding = find_device_binding(command.device_id);
        if (binding == nullptr) {
            error = fleet_err::invalid_argument;
            continue;
        }

        if (auto command_error = 
                    batches[binding->driver_slot].push(binding->device_slot, command)) {
            error = command_error;
        }
    }

    for (auto &batch : batches) {
        if (auto flush_error = batch.flush()) error = flush_error;
    }
    return error;
}

namespace {

std::atomic<std::shared_ptr<runtime_t>> runtime;

std::shared_ptr<runtime_t> require_runtime() {
    return runtime.load(std::memory_order_acquire);
}

} // namespace

err_code init(fleet_config options) {
    std::shared_ptr<runtime_t> next;
    try {
        next = std::make_shared<runtime_t>(std::move(options));
    } catch (...) {
        return fleet_err::io_error;
    }
    if (auto error = next->init())
        return error;
    if (auto error = next->start())
        return error;
    runtime.store(next, std::memory_order_release);
    return {};
}

void shutdown() noexcept {
    runtime.store({}, std::memory_order_release);
}

bool ok() noexcept { return static_cast<bool>(require_runtime()); }

template <class T> class always_false : std::false_type {};

template <IsDevice T> bucket_t<T> lookup() {
    auto current = require_runtime();
    if (!current)
        return {};
    if constexpr (std::same_as<T, motor::motor_interface>) {
        return current->motors();
    } else {
        static_assert(always_false<T>::value, "unsupported device type");
    }
}

template <Command T> err_code command(const T &command) {
    auto current = require_runtime();
    if (!current)
        return fleet_err::io_error;
    return current->send_command(command);
}

template <Command T> err_code command(std::span<const T> commands) {
    auto current = require_runtime();
    if (!current)
        return fleet_err::io_error;
    return current->send_commands(commands);
}

template bucket_t<motor::motor_interface> lookup<motor::motor_interface>();

template err_code command(const velocity_command &);
template err_code command(const position_command &);
template err_code command(const pvt_command &);
template err_code command(const torque_command &);
template err_code command(const current_command &);
template err_code command(const brake_command &);
template err_code command(const config_set_command &);
template err_code command(const config_query_command &);
template err_code command(const net_control_command &);
template err_code command(const discard_pending_command &);

template err_code command(span<const velocity_command>);
template err_code command(span<const position_command>);
template err_code command(span<const pvt_command>);
template err_code command(span<const torque_command>);
template err_code command(span<const current_command>);
template err_code command(span<const brake_command>);
template err_code command(span<const config_set_command>);
template err_code command(span<const config_query_command>);
} // namespace fleet
