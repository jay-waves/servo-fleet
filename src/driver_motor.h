/*
    Copyright PNDBotics 2026
*/
#pragma once

#include <array>
#include <string_view>

#include "codec_motor.h"
#include "device_motor.h"

namespace fleet {

class realtime_udp_channel;

struct motor_ec_spec {
	std::string_view model;

	range_t<float> kp;
	range_t<float> kd;

	range_t<float> speed_rad; // SPD 
	range_t<float> pos_rad; // POS
	range_t<float> torque_nm; // TOR 
	range_t<float> current_a; // CUR 					
	
	float kt;
	float inertia; // kg * mm^2
};

const motor_ec_spec *find_motor_ec_spec(std::string_view model) noexcept;

struct motor_route {
    motor_device *device = nullptr;
    const motor_ec_spec *spec = nullptr;
};

class motor_driver final {
  public:
    motor_driver() = default;
    explicit motor_driver(const fleet_config::channel_config &config);
    explicit motor_driver(std::unique_ptr<realtime_udp_channel> channel);
    ~motor_driver();

    std::string_view channel_name() const noexcept;

    void start();
    void stop() noexcept;
    uint32_t bind_motor(motor_device &device);
    err_code send(const can_frame &frame);
    err_code send(std::span<const can_frame> frames);
    void discard_pending_sends() noexcept;
    void benchmark_handle_packet(bytes_view packet);

    template <MotorCommand T> class batch_commands {
      public:
        explicit batch_commands(motor_driver &driver) : driver_(&driver) {}

        err_code push(uint32_t device_slot, const T &command) {
            if (device_slot >= driver_->devices_.size())
                return fleet_err::invalid_argument;

            auto *device = driver_->devices_[device_slot];
            const auto can_addr = device->address();
            auto &frame = frames_[frame_count_];
            if (auto err = can_codec::encode(frame, can_addr, command))
                return err;

            ++frame_count_;
            return frame_count_ == frames_.size() ? flush() : err_code{};
        }

        err_code flush() {
            if (frame_count_ == 0) return {};

            const auto error =
                driver_->send(std::span<const can_frame>(frames_).first(frame_count_));
            frame_count_ = 0;
            return error;
        }

      private:
        motor_driver *driver_;
        std::array<can_frame, MAX_FRAMES_PER_PACKET> frames_{};
        size_t frame_count_ = 0;
    };

  private:
    void handle_packet(bytes_view packet);
    static void handle_decoded_frame(void *context, const can_frame &frame);
    void handle_frame(can_port_t port, const can_frame &frame, time_point received_at);

    std::unique_ptr<realtime_udp_channel> channel_;
    std::vector<motor_device *> devices_; // 正向索引
    std::vector<std::vector<motor_route>> motor_routes_; // 反向索引
};

} // namespace fleet
