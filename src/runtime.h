/*
    Copyright PNDBotics 2026
*/
#pragma once

#include "driver_motor.h"

namespace fleet {

template <class T> using bucket_t = std::vector<std::shared_ptr<T>>;
template <class T> using box_t = std::vector<std::unique_ptr<T>>;

class runtime_t : public std::enable_shared_from_this<runtime_t> {
  public:
    explicit runtime_t(fleet_config options);
    ~runtime_t();

    err_code init();
    err_code start();

    bucket_t<motor::motor_interface> motors() const { return motors_; }

    template <Command T> err_code send_command(const T &command);

    template <Command T> err_code send_commands(std::span<const T> commands);

  private:
    struct device_binding {
        uint32_t driver_slot = 0;
        uint32_t device_slot = 0;
    };

    err_code validate_config() const;
    void init_drivers();
    err_code bind_motors();
    void log_config() const;
    err_code discard_pending_commands() noexcept;
    err_code update_net_control(const net_control_command &control);

    template <MotorCommand T> err_code send_motor_commands(std::span<const T> commands);

    const device_binding *find_device_binding(device_id_t id) const noexcept;

    fleet_config options_;
    bucket_t<motor::motor_interface> motors_;
    box_t<motor_driver> motor_drivers_;
    std::vector<device_binding> device_bindings_;
};

} // namespace fleet
