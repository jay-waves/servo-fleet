/*
    Copyright PNDBotics 2026
*/

#pragma once

#include "error.h"
#include "fleet/device.h"
#include "fleet/realtime.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace fleet {

class motor_driver;

struct udp_channel_options {
    std::string remote_address = "127.0.0.1";
    uint16_t remote_port = 9999;
    uint16_t local_port = 0;
    size_t max_bandwidth_bps = 1'000'000;
    uint32_t send_queue_slots = 1024;
    linux_thread_config rx_thread;
};

/*
 * udp channel launch a background thread to handle udp transport. because of this, udp channel
 * will not block the main thread.
 */
class udp_channel {
  public:
    explicit udp_channel(udp_channel_options options);
    udp_channel(std::string name, udp_channel_options options);
    ~udp_channel();

    udp_channel(const udp_channel &) = delete;
    udp_channel &operator=(const udp_channel &) = delete;
    udp_channel(udp_channel &&) = delete;
    udp_channel &operator=(udp_channel &&) = delete;

    const std::string &name() const noexcept;

    void stop() noexcept;
    void flush_pending_sends(std::chrono::milliseconds timeout = std::chrono::milliseconds{20}) noexcept;

    /*
     * no thread-safe was promised, please just set this during the initilization.
     */
    void set_rx_callback(std::function<void(bytes_view)> cb);
    err_code send(bytes_view data);

  private:
    struct impl;
    std::unique_ptr<impl> impl_;
};


/*
    高性能异步 UDP 传输层封装。
    通过令牌桶（token bucket）进行带宽限流，可以避免 UDP 发送速度过快导致对端硬件 DoS。需要注意，这只是背压兜底，而不是明确的周期发令控制，
    如果需要按固定周期进行发令，请在上层（send_command 之上）进行明确节拍控制。我们只保证尽量低抖动来发令，并限制最大带宽。
    如果发令速度超过最大带宽限制，可能会发生抖动延迟，这是预期的限流行为。

    通过内存池（packet_pool）来减少内存分配抖动。默认单个 UDP 包最大 512B（UDP 协议最大支持 65507B，以太网 MTU 默认为 1500B，
    因为 UDP 不是可靠协议，推荐 1200B 来避免 IP 分片）。内存池默认分配 512B * 1024（slot），通过 prepare_send() 接口可申请单个内存槽


    接受 channel_options 中的两个额外参数：
    * max_bandwidth_bps: 发送最大带宽
    * send_queue_slots: 发送队列可积压的 UDP 包数量。队列满时返回 fleet_err::busy
*/
class realtime_udp_channel {
  public:
    class packet_buffer {
      public:
        packet_buffer() = default;
        ~packet_buffer() { release(); }

        packet_buffer(const packet_buffer &) = delete;
        packet_buffer &operator=(const packet_buffer &) = delete;
        packet_buffer(packet_buffer &&other) noexcept;
        packet_buffer &operator=(packet_buffer &&other) noexcept;

        bytes_mut buffer() const noexcept { return buffer_; }

      private:
        friend class realtime_udp_channel;

        packet_buffer(realtime_udp_channel *owner, uint32_t slot, bytes_mut buffer)
            : owner_(owner), slot_(slot), buffer_(buffer) {}

        void release() noexcept;

        realtime_udp_channel *owner_ = nullptr;
        uint32_t slot_ = 0;
        bytes_mut buffer_;
    };

    explicit realtime_udp_channel(udp_channel_options options);
    realtime_udp_channel(std::string name, udp_channel_options options);
    ~realtime_udp_channel();

    realtime_udp_channel(const realtime_udp_channel &) = delete;
    realtime_udp_channel &operator=(const realtime_udp_channel &) = delete;
    realtime_udp_channel(realtime_udp_channel &&) = delete;
    realtime_udp_channel &operator=(realtime_udp_channel &&) = delete;

    const std::string &name() const noexcept;

    void start();
    void stop() noexcept;
    void discard_pending_sends() noexcept;
    /*
        callback 在 channel 的 rx_thread 中执行。
        参数 bytes_view 指向 channel 内部接收缓冲区，只在本次 callback 返回前有效。
    */
    void set_rx_callback(std::function<void(bytes_view)> cb);
    void set_timestamped_rx_callback(
        std::function<void(bytes_view, std::chrono::steady_clock::time_point)> cb);

    /*
        有单次拷贝的接口
    */
    err_code send(bytes_view data);
    /*
        零拷贝接口，prepare() 申请内存，commit() 提交
    */
    result<packet_buffer> prepare_send();
    err_code commit_send(packet_buffer &&packet, uint32_t size);

  private:
    void release_packet(uint32_t slot) noexcept;

    struct impl;
    std::unique_ptr<impl> impl_;

};

} // namespace fleet
