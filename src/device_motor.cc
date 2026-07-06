#include "device_motor.h"
#include "driver_motor.h"

#include <optional>
#include <type_traits>
#include <utility>
#include <variant>

namespace fleet {

namespace {

constexpr float pi = 3.14159265358979323846f;
constexpr float rad_to_deg = 180.0f / pi;
constexpr float radps_to_rpm = 60.0f / (2.0f * pi);

template <typename T> inline
sample_t<T> read_sample(std::mutex &mutex, const sample_t<T> &sample) {
    std::lock_guard lock(mutex);
    return sample;
}

template <typename T> inline
void set_sample(sample_t<T> &sample, const T &value, time_point time, uint64_t seq) {
    sample.time = time;
    sample.seq = seq;
    sample.value = value;
}

template <typename T> inline
void set_sample(sample_t<T> &sample, const std::optional<T> &value, time_point time, uint64_t seq) {
    if (value) {
        set_sample(sample, *value, time, seq);
    }
}

sample_t<range_t<float>> table_sample(range_t<float> value) {
    return sample_t<range_t<float>>{
        .time = std::chrono::steady_clock::now(),
        .seq = 1,
        .value = value,
    };
}

void set_mcu_uuid_fragment(sample_t<mcu_uuid_t> &sample, const mcu_uuid_fragment_t &fragment, time_point time,
                           uint64_t seq) {
    if (fragment.index < 1 || fragment.index > 3) {
        return;
    }

    auto uuid = sample.value;
    const auto offset = static_cast<size_t>(fragment.index - 1U) * fragment.bytes.size();
    std::copy(fragment.bytes.begin(), fragment.bytes.end(), uuid.bytes.begin() + offset);
    uuid.received_mask |= static_cast<uint8_t>(1U << (fragment.index - 1U));
    set_sample(sample, uuid, time, seq);
}

} // namespace

motor_device::motor_device(device_id_t device_id, const fleet_config::motor_config& config)
    : motor::motor_interface(device_id, config.name),
      addr_({.can_port = config.can_port, .can_id = config.can_id}),
      ec_spec_(find_motor_ec_spec(config.model)) {}


sample_t<motor_err> motor_device::motor_error() const {
    return read_sample(state_mutex_, error_.motor_error);
}

sample_t<float> motor_device::position_deg() const {
    return read_sample(state_mutex_, motion_.position_deg);
}

sample_t<float> motor_device::velocity_rpm() const {
    return read_sample(state_mutex_, motion_.velocity_rpm);
}

sample_t<float> motor_device::acceleration_radps2() const {
    return read_sample(state_mutex_, motion_.acceleration_radps2);
}

sample_t<float> motor_device::current() const {
    return read_sample(state_mutex_, electrical_.current);
}

sample_t<float> motor_device::power() const {
    return read_sample(state_mutex_, electrical_.power);
}

sample_t<float> motor_device::motor_temp_c() const {
    return read_sample(state_mutex_, thermal_.motor_temp_c);
}

sample_t<float> motor_device::mosfet_temp_c() const {
    return read_sample(state_mutex_, thermal_.mosfet_temp_c);
}

sample_t<float> motor_device::flux_observer_gain() const {
    return read_sample(state_mutex_, config_.flux_observer_gain);
}

sample_t<float> motor_device::disturbance_compensation_coeff() const {
    return read_sample(state_mutex_, config_.disturbance_compensation_coeff);
}

sample_t<float> motor_device::feedback_compensation_coeff() const {
    return read_sample(state_mutex_, config_.feedback_compensation_coeff);
}

sample_t<float> motor_device::damping_coeff() const {
    return read_sample(state_mutex_, config_.damping_coeff);
}

sample_t<float> motor_device::torque_ratio() const {
    return read_sample(state_mutex_, config_.torque_ratio);
}

sample_t<range_t<float>> motor_device::pvt_kp_range() const {
    auto sample = read_sample(state_mutex_, config_.pvt_kp_range);
    return sample || ec_spec_ == nullptr ? sample : table_sample(ec_spec_->kp);
}

sample_t<range_t<float>> motor_device::pvt_kd_range() const {
    auto sample = read_sample(state_mutex_, config_.pvt_kd_range);
    return sample || ec_spec_ == nullptr ? sample : table_sample(ec_spec_->kd);
}

sample_t<range_t<float>> motor_device::pvt_position_limit_r() const {
    auto sample = read_sample(state_mutex_, config_.pvt_position_limit_r);
    return sample || ec_spec_ == nullptr ? sample : table_sample(ec_spec_->pos_rad);
}

sample_t<range_t<float>> motor_device::pvt_velocity_limit_radps() const {
    return read_sample(state_mutex_, config_.pvt_velocity_limit_radps);
}

sample_t<range_t<float>> motor_device::pvt_torque_limit_nm() const {
    return read_sample(state_mutex_, config_.pvt_torque_limit_nm);
}

sample_t<range_t<float>> motor_device::pvt_current_limit() const {
    return read_sample(state_mutex_, config_.pvt_current_limit);
}

sample_t<mcu_uuid_t> motor_device::mcu_uuid() const {
    return read_sample(state_mutex_, config_.mcu_uuid);
}

sample_t<version_info_t> motor_device::version() const {
    return read_sample(state_mutex_, config_.version);
}

sample_t<milliseconds> motor_device::can_timeout() const {
    return read_sample(state_mutex_, config_.can_timeout);
}

sample_t<pid_gains> motor_device::current_loop_pi() const {
    return read_sample(state_mutex_, config_.current_loop_pi);
}

sample_t<pid_gains> motor_device::velocity_loop_pi() const {
    return read_sample(state_mutex_, config_.velocity_loop_pi);
}

sample_t<pid_gains> motor_device::position_loop_pd() const {
    return read_sample(state_mutex_, config_.position_loop_pd);
}

sample_t<bool> motor_device::brake_status() const {
    return read_sample(state_mutex_, brake_.status);
}

bool motor_device::apply(const feedback &response) {
    return apply(response, std::chrono::steady_clock::now());
}

bool motor_device::apply(const feedback &response, time_point now) {
    return std::visit(
        [&](const auto &value) {
            apply_feedback_value(value, now, ++sample_seq_);
            return !std::is_same_v<std::decay_t<decltype(value)>, config_ack_feeback>;
        },
        response);
}

void motor_device::apply_feedback_value(const status_feedback &value, time_point now,
                                        uint64_t seq) {
    std::lock_guard lock(state_mutex_);
    set_sample(error_.motor_error, value.error, now, seq);
    set_sample(motion_.position_deg, value.position_r * rad_to_deg, now, seq);
    set_sample(motion_.velocity_rpm, value.velocity_radps * radps_to_rpm, now, seq);
    set_sample(electrical_.current, value.current, now, seq);
    set_sample(thermal_.motor_temp_c, value.motor_temp_c, now, seq);
    set_sample(thermal_.mosfet_temp_c, value.mosfet_temp_c, now, seq);
    set_updated_at(now);
}

void motor_device::apply_feedback_value(const position_feedback &value, time_point now,
                                        uint64_t seq) {
    std::lock_guard lock(state_mutex_);
    set_sample(error_.motor_error, value.error, now, seq);
    set_sample(motion_.position_deg, value.position, now, seq);
    set_sample(electrical_.current, value.current, now, seq);
    set_sample(thermal_.motor_temp_c, value.motor_temp_c, now, seq);
    set_updated_at(now);
}

void motor_device::apply_feedback_value(const velocity_feedback &value, time_point now,
                                        uint64_t seq) {
    std::lock_guard lock(state_mutex_);
    set_sample(error_.motor_error, value.error, now, seq);
    set_sample(motion_.velocity_rpm, value.velocity, now, seq);
    set_sample(electrical_.current, value.current, now, seq);
    set_sample(thermal_.motor_temp_c, value.motor_temp_c, now, seq);
    set_updated_at(now);
}

void motor_device::apply_feedback_value(const brake_status_feedback &value, time_point now,
                                        uint64_t seq) {
    std::lock_guard lock(state_mutex_);
    set_sample(error_.motor_error, value.error, now, seq);
    set_sample(brake_.status, value.engaged, now, seq);
    set_updated_at(now);
}

void motor_device::apply_feedback_value(const query_feedback &value, time_point now, uint64_t seq) {
    std::lock_guard lock(state_mutex_);
    set_sample(error_.motor_error, value.error, now, seq);

    using enum config_query_code;
    switch (value.code) {
    case position:
        set_sample(motion_.position_deg, value.position_deg, now, seq);
        break;
    case velocity:
        set_sample(motion_.velocity_rpm, value.velocity_rpm, now, seq);
        break;
    case current:
        set_sample(electrical_.current, value.current, now, seq);
        break;
    case power:
        set_sample(electrical_.power, value.power, now, seq);
        break;
    case acceleration:
        set_sample(motion_.acceleration_radps2, value.acceleration_radps2, now, seq);
        break;
    case flux_observer_gain:
        set_sample(config_.flux_observer_gain, value.flux_observer_gain, now, seq);
        break;
    case disturbance_compensation_coeff:
        set_sample(config_.disturbance_compensation_coeff, value.disturbance_compensation_coeff, now, seq);
        break;
    case feedback_compensation_coeff:
        set_sample(config_.feedback_compensation_coeff, value.feedback_compensation_coeff, now, seq);
        break;
    case damping_coeff:
        set_sample(config_.damping_coeff, value.damping_coeff, now, seq);
        break;
    case torque_ratio:
        set_sample(config_.torque_ratio, value.torque_ratio, now, seq);
        break;
    case pvt_kp_range:
        set_sample(config_.pvt_kp_range, value.pvt_kp_range, now, seq);
        break;
    case pvt_kd_range:
        set_sample(config_.pvt_kd_range, value.pvt_kd_range, now, seq);
        break;
    case pvt_position_limit_r:
        set_sample(config_.pvt_position_limit_r, value.pvt_position_limit_r, now, seq);
        break;
    case pvt_velocity_limit_radps:
        set_sample(config_.pvt_velocity_limit_radps, value.pvt_velocity_limit_radps, now, seq);
        break;
    case pvt_torque_limit_nm:
        set_sample(config_.pvt_torque_limit_nm, value.pvt_torque_limit_nm, now, seq);
        break;
    case pvt_current_limit:
        set_sample(config_.pvt_current_limit, value.pvt_current_limit, now, seq);
        break;
    case mcu_uuid:
        set_mcu_uuid_fragment(config_.mcu_uuid, value.mcu_uuid_fragment, now, seq);
        break;
    case version:
        set_sample(config_.version, value.version, now, seq);
        break;
    case can_timeout:
        set_sample(config_.can_timeout, value.can_timeout, now, seq);
        break;
    case current_loop_pi:
        set_sample(config_.current_loop_pi, value.current_loop_pi, now, seq);
        break;
    case velocity_loop_pi:
        set_sample(config_.velocity_loop_pi, value.velocity_loop_pi, now, seq);
        break;
    case position_loop_pd:
        set_sample(config_.position_loop_pd, value.position_loop_pd, now, seq);
        break;
    case brake_status:
        set_sample(brake_.status, value.brake_status, now, seq);
        break;
    }
    set_updated_at(now);
}

void motor_device::apply_feedback_value(const config_ack_feeback &, time_point, uint64_t) {
    // update last_error
}

} // namespace fleet
