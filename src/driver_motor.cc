#include "driver_motor.h"
#include "codec_motor.h"
#include "log.h"
#include "channel.h"

#include <algorithm>
#include <optional>

namespace fleet {
namespace {

// ========== ENCOS EC 系列力位混控协议范围2026.2.27 ==========
static constexpr auto EC_MOTOR_TABLE = std::to_array<motor_ec_spec>({
// ========== 行星系列 ==========
{
    "EC-A2806-P2-36",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},      
    {-12, 12}, {-10, 10},
    1.35f, 4.26f
},
{
    "EC-A4310-P2-36",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},      
    {-30, 30}, {-30, 30},
    1.4f, 19
},
{
    "EC-A4315-P2-36",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},      
    {-70, 70}, {-30, 30},
    2.8f, 25
},
{
    "EC-A6408-P2-25",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},      
    {-60, 60}, {-60, 60},
    2.35f, 62
},
{
    "EC-A6416-P2-25",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},      
    {-120, 120}, {-60, 60},
    2.74f, 104
},
{
    "EC-A8112-P1-18",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},      
    {-90, 90}, {-60, 60},
    2.1f, 149
},
{
    "EC-A8116-P1-18",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},      
    {-150, 150}, {-70, 70},
    2.35f, 189
},

// ========== 高扭矩系列 ==========
{
    "EC-A10020-P1-12/6",
    {0, 500}, {0, 50},
    {-18, 18}, {-12.5, 12.5},      
    {-150, 150}, {-70, 70},
    2.5f, 485
},
{
    "EC-A13720-P1-11.4",
    {0, 500}, {0, 50},
    {-18, 18}, {-12.5, 12.5},      
    {-400, 400}, {-220, 220},
    2.5f, 1514
},
{
    "EC-A13715-P1-12.67",
    {0, 500}, {0, 50},
    {-18, 18}, {-12.5, 12.5},      
    {-320, 320}, {-220, 220},
    2.5f, 1237
},

// ========== 行星中空 ==========
{
    "EC-A4310-P2-36H",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},      
    {-30, 30}, {-30, 30},
    1.4f, 18
},
{
    "EC-A6408-P2-16H",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},
    {-45, 45}, {-30, 30},
    2.15f, 62.581f
},
{
    "EC-A6408-P2-30.25H",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},
    {-60, 60}, {-60, 60},
    2.45f, 63
},
{
    "EC-A6408-P2-30.25HB",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},
    {-60, 60}, {-60, 60},
    2.20f, 74
},
{
    "EC-A6416-P2-30.25H",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},
    {-120, 120}, {-60, 60},
    2.65f, 102.678f
},
{
    "EC-A8112-P1-18H",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},
    {-90, 90}, {-60, 60},
    2.1f, 98
},
{
    "EC-A8116-P1-18H",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},
    {-130, 130}, {-70, 70},
    2.35f, 197
},
{
    "EC-A10020-P2-24",
    {0, 500}, {0, 50},
    {-18, 18}, {-12.5, 12.5},
    {-300, 300}, {-140, 140},
    2.6f, 477
},

// ========== 谐波系列 ==========
{
    "EC-A3814-H14-107",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},
    {-60, 60}, {-20, 20},
    4.2f, 3
},
{
    "EC-A5013-H17-100",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},
    {-90, 90}, {-30, 30},
    5.9f, 10
},
{
    "EC-A6013-H20-100",
    {0, 500}, {0, 5},
    {-18, 18}, {-12.5, 12.5},
    {-130, 130}, {-35, 35},
    5.6f, 24
},


});

struct decode_context {
    motor_driver *driver = nullptr;
    time_point received_at{};
};
std::optional<can_id_t> parse_broadcast_can_id(const can_frame &frame) {
    if (frame.id != BROADCAST_CAN_ID || frame.len != 5) {
        return std::nullopt;
    }
    if (frame.data[0] != 0xFF || frame.data[1] != 0xFF || frame.data[2] != 0x01) {
        return std::nullopt;
    }
    return static_cast<can_id_t>((static_cast<uint16_t>(frame.data[3]) << 8U) | frame.data[4]);
}

} // namespace

std::string_view motor_driver::channel_name() const noexcept { return channel_->name(); }

const motor_ec_spec *find_motor_ec_spec(std::string_view model) noexcept {
    if (model.empty()) {
        return nullptr;
    }

    const auto found = std::ranges::find(EC_MOTOR_TABLE, model, &motor_ec_spec::model);
    return found == EC_MOTOR_TABLE.end() ? nullptr : &*found;
}

motor_driver::motor_driver(const fleet_config::channel_config &config)
    : motor_driver(std::make_unique<realtime_udp_channel>(
          config.name,
          udp_channel_options{
              .remote_address = config.remote_address,
              .remote_port = config.remote_port,
              .local_port = config.local_port,
              .max_bandwidth_bps = config.max_bandwidth_bps,
              .send_queue_slots = config.send_queue_slots,
              .rx_thread = config.rx_thread,
          })) {}

motor_driver::motor_driver(std::unique_ptr<realtime_udp_channel> channel) : channel_(std::move(channel)) {}

motor_driver::~motor_driver() { stop(); }

void motor_driver::start() {
    channel_->set_timestamped_rx_callback([this](bytes_view packet, time_point received_at) {
        handle_packet(packet, received_at);
    });
    channel_->start();
}

void motor_driver::stop() noexcept {
    if (channel_) {
        channel_->stop();
    }
}

uint32_t motor_driver::bind_motor(motor_device &device) {
    const auto device_slot = static_cast<uint32_t>(devices_.size());
    devices_.push_back(&device);

    const auto address = device.address();

    if (motor_routes_.size() <= address.can_port) {
        motor_routes_.resize(static_cast<size_t>(address.can_port) + 1);
    }
    auto &port_routes = motor_routes_[address.can_port];
    if (port_routes.size() <= address.can_id) {
        port_routes.resize(static_cast<size_t>(address.can_id) + 1);
    }
    port_routes[address.can_id] = motor_route{.device = &device, .spec = device.ec_spec()};
    return device_slot;
}

err_code motor_driver::send(const can_frame &frame) {
    return send(std::span<const can_frame>(&frame, 1));
}

err_code motor_driver::send(std::span<const can_frame> frames) {
    if (frames.empty())
        return {};

    auto packet = channel_->prepare_send();
    if (!packet)
        return packet.error();

    auto encoded_size = motor_codec::encode_into(packet->buffer(), TYPE_UDP2CAN, frames);
    if (!encoded_size) {
        FLEET_WARN("encos driver '{}' encode failed frames={}", channel_->name(), frames.size());
        return encoded_size.error();
    }
    return channel_->commit_send(std::move(*packet), *encoded_size);
}

void motor_driver::discard_pending_sends() noexcept {
    channel_->discard_pending_sends();
}

void motor_driver::benchmark_handle_packet(bytes_view packet) {
    handle_packet(packet);
}

void motor_driver::handle_packet(bytes_view bytes) {
    handle_packet(bytes, std::chrono::steady_clock::now());
}

void motor_driver::handle_packet(bytes_view bytes, time_point received_at) {
    decode_context context{
        .driver = this,
        .received_at = received_at,
    };
    if (const auto error = motor_codec::decode_each(bytes, &context, &motor_driver::handle_decoded_frame)) {
        FLEET_WARN("encos driver '{}' decode failed bytes={}", channel_->name(), bytes.size());
    }
}

void motor_driver::handle_decoded_frame(void *context, const can_frame &frame) {
    auto &decode = *static_cast<decode_context *>(context);
    decode.driver->handle_frame(frame.port, frame, decode.received_at);
}

void motor_driver::handle_frame(can_port_t port, const can_frame &frame, time_point received_at) {
    // if (const auto can_id = parse_broadcast_can_id(frame)) {
    //     return;
    // }

    if (frame.id == BROADCAST_CAN_ID) {
        (void)can_codec::parse_feedback(frame);
        return;
    }

    if (port >= motor_routes_.size() || frame.id >= motor_routes_[port].size() ||
        motor_routes_[port][frame.id].device == nullptr) {
        FLEET_DEBUG("encos driver '{}' ignore unrouted frame id=0x{:X} port={}",
                  channel_->name(), frame.id, port);
        return;
    }

    const auto &route = motor_routes_[port][frame.id];
    auto response = can_codec::parse_feedback(frame, route.spec != nullptr ? &route.spec->current_a : nullptr);
    if (!response) {
        return;
    }
    route.device->apply(*response, received_at);
}

} // namespace fleet
