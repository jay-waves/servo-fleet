#pragma once

#include <atomic>
#include <mutex>
#include <unordered_map>

#include "fleet/device_motor.h"
#include "fleet/fleet.h"
#include "codec_can.h"

namespace fleet {

struct motor_ec_spec;

class motor_device final : public motor::motor_interface {
  public:
    motor_device(device_id_t device_id, const fleet_config::motor_config& config);

    sample_t<motor_err> motor_error() const override;
    sample_t<float> position_deg() const override;
    sample_t<float> velocity_rpm() const override;
    sample_t<float> current() const override;
    sample_t<float> power() const override;
    sample_t<float> acceleration_radps2() const override;

    sample_t<float> motor_temp_c() const override;
    sample_t<float> mosfet_temp_c() const override;

    sample_t<float> flux_observer_gain() const override;
    sample_t<float> disturbance_compensation_coeff() const override;
    sample_t<float> feedback_compensation_coeff() const override;
    sample_t<float> damping_coeff() const override;
    sample_t<float> torque_ratio() const override;
    sample_t<range_t<float>> pvt_kp_range() const override;
    sample_t<range_t<float>> pvt_kd_range() const override;
    sample_t<range_t<float>> pvt_position_limit_r() const override;
    sample_t<range_t<float>> pvt_velocity_limit_radps() const override;
    sample_t<range_t<float>> pvt_torque_limit_nm() const override;
    sample_t<range_t<float>> pvt_current_limit() const override;
    sample_t<mcu_uuid_t> mcu_uuid() const override;
    sample_t<version_info_t> version() const override;
    sample_t<milliseconds> can_timeout() const override;
    sample_t<pid_gains> current_loop_pi() const override;
    sample_t<pid_gains> velocity_loop_pi() const override;
    sample_t<pid_gains> position_loop_pd() const override;
    sample_t<bool> brake_status() const override;

    bool apply(const feedback &response);
    bool apply(const feedback &response, time_point now);

    can_addr_t address() const { return addr_; }
    const motor_ec_spec *ec_spec() const noexcept { return ec_spec_; }

  private:
    void apply_feedback_value(const status_feedback &value, time_point now, uint64_t seq);
    void apply_feedback_value(const position_feedback &value, time_point now, uint64_t seq);
    void apply_feedback_value(const velocity_feedback &value, time_point now, uint64_t seq);
    void apply_feedback_value(const brake_status_feedback &value, time_point now, uint64_t seq);
    void apply_feedback_value(const query_feedback &value, time_point now, uint64_t seq);
    void apply_feedback_value(const config_ack_feeback &value, time_point now, uint64_t seq); // TODO

    std::atomic<uint64_t> sample_seq_ = 0;
    can_addr_t addr_;
    const motor_ec_spec *ec_spec_ = nullptr;

    struct motion_state {
        sample_t<float> position_deg;
        sample_t<float> velocity_rpm;
        sample_t<float> acceleration_radps2;
    };

    struct electrical_state {
        sample_t<float> current;
        sample_t<float> power;
    };

    struct thermal_state {
        sample_t<float> motor_temp_c;
        sample_t<float> mosfet_temp_c;
    };

    struct config_state {
        sample_t<float> flux_observer_gain;
        sample_t<float> disturbance_compensation_coeff;
        sample_t<float> feedback_compensation_coeff;
        sample_t<float> damping_coeff;
        sample_t<float> torque_ratio;
        sample_t<range_t<float>> pvt_kp_range;
        sample_t<range_t<float>> pvt_kd_range;
        sample_t<range_t<float>> pvt_position_limit_r;
        sample_t<range_t<float>> pvt_velocity_limit_radps;
        sample_t<range_t<float>> pvt_torque_limit_nm;
        sample_t<range_t<float>> pvt_current_limit;
        sample_t<mcu_uuid_t> mcu_uuid;
        sample_t<version_info_t> version;
        sample_t<milliseconds> can_timeout;
        sample_t<pid_gains> current_loop_pi;
        sample_t<pid_gains> velocity_loop_pi;
        sample_t<pid_gains> position_loop_pd;
    };

    struct brake_state {
        sample_t<bool> status;
    };

    struct error_state {
        sample_t<motor_err> motor_error;
    };

    mutable std::mutex state_mutex_; // 细粒度锁？
    motion_state motion_;
    electrical_state electrical_;
    thermal_state thermal_;
    config_state config_;
    brake_state brake_;
    error_state error_;
};

} // namespace fleet

