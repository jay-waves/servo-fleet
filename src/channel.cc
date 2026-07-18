#include "log.h"
#include "channel.h"
#include "thread_tuning.h"

#include <asio.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <future>
#include <utility>
#include <vector>

#ifdef __linux__
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

namespace fleet {
namespace {

using asio::ip::udp;
using std::chrono::steady_clock;
using std::vector;

constexpr size_t MAX_UDP_SIZE = 65507;
constexpr size_t UDP_PACKET_CAP = 128;
constexpr size_t MOTOR_UDP_PACKET_CAP = 512;
constexpr size_t SEND_RETRY_COUNT = 3;
constexpr auto SEND_RETRY_INITIAL_BACKOFF = 100ms;
constexpr int UDP_SOCKET_BUFFER_BYTES = 1 * 1024 * 1024;

err_code to_fleet_error(const asio::error_code &error) {
    if (!error)
        return {};
    if (error == asio::error::operation_aborted || error == asio::error::timed_out) {
        return fleet_err::cancelled;
    }
    return fleet_err::io_error;
}

bool is_expected_sender(const udp::endpoint &remote, const udp::endpoint &sender) noexcept {
    return sender.address() == remote.address() && (remote.port() == 0 || sender.port() == remote.port());
}

void tune_socket_buffers(udp::socket &socket) {
    asio::error_code error;
    socket.set_option(asio::socket_base::receive_buffer_size(UDP_SOCKET_BUFFER_BYTES), error);
    if (error) {
        FLEET_DEBUG_KV(KV("msg", "UDP receive buffer tune failed"), KV("error", error.message()));
    }

    socket.set_option(asio::socket_base::send_buffer_size(UDP_SOCKET_BUFFER_BYTES), error);
    if (error) {
        FLEET_DEBUG_KV(KV("msg", "UDP send buffer tune failed"), KV("error", error.message()));
    }
}

#ifdef __linux__
void enable_kernel_rx_timestamps(udp::socket &socket) {
    const int enabled = 1;
    if (::setsockopt(socket.native_handle(), SOL_SOCKET, SO_TIMESTAMPNS, &enabled,
                     sizeof(enabled)) != 0) {
        throw std::system_error(errno, std::generic_category(),
                                "enable SO_TIMESTAMPNS");
    }
}

struct timestamped_datagram {
    size_t size = 0;
    udp::endpoint sender;
    steady_clock::time_point received_at{};
    asio::error_code error;
};

timestamped_datagram receive_timestamped(udp::socket &socket,
                                         std::span<uint8_t> buffer) noexcept {
    timestamped_datagram result;
    sockaddr_in source{};
    iovec data{.iov_base = buffer.data(), .iov_len = buffer.size()};
    alignas(cmsghdr) std::array<char, CMSG_SPACE(sizeof(timespec))> control{};
    msghdr message{};
    message.msg_name = &source;
    message.msg_namelen = sizeof(source);
    message.msg_iov = &data;
    message.msg_iovlen = 1;
    message.msg_control = control.data();
    message.msg_controllen = control.size();

    const auto size = ::recvmsg(socket.native_handle(), &message, MSG_DONTWAIT);
    if (size < 0) {
        result.error = asio::error_code(errno, asio::error::get_system_category());
        return result;
    }
    result.size = static_cast<size_t>(size);
    result.sender = udp::endpoint(
        asio::ip::address_v4(ntohl(source.sin_addr.s_addr)), ntohs(source.sin_port));

    std::optional<timespec> kernel_time;
    for (auto *header = CMSG_FIRSTHDR(&message); header != nullptr;
         header = CMSG_NXTHDR(&message, header)) {
        if (header->cmsg_level == SOL_SOCKET && header->cmsg_type == SCM_TIMESTAMPNS &&
            header->cmsg_len >= CMSG_LEN(sizeof(timespec))) {
            timespec value{};
            std::memcpy(&value, CMSG_DATA(header), sizeof(value));
            kernel_time = value;
            break;
        }
    }

    const auto steady_now = steady_clock::now();
    if (!kernel_time) {
        result.received_at = steady_now;
        return result;
    }
    const auto system_now = std::chrono::system_clock::now();
    const auto packet_system = std::chrono::system_clock::time_point{
        std::chrono::seconds(kernel_time->tv_sec) + std::chrono::nanoseconds(kernel_time->tv_nsec)};
    const auto age = system_now - packet_system;
    result.received_at = steady_now -
        std::chrono::duration_cast<steady_clock::duration>(age);
    return result;
}
#endif

vector<uint8_t> make_send_storage(uint32_t send_queue_slots, uint32_t packet_cap) {
    if (packet_cap == 0 || send_queue_slots > UINT32_MAX / packet_cap) {
        throw std::invalid_argument("async udp send queue slots is too large");
    }
    return vector<uint8_t>(static_cast<size_t>(send_queue_slots * packet_cap));
}

} // namespace

template <typename T> class fixed_ring_queue {
  public:
    explicit fixed_ring_queue(size_t capacity) : storage_(capacity) {}

    fixed_ring_queue(const fixed_ring_queue &) = delete;
    fixed_ring_queue &operator=(const fixed_ring_queue &) = delete;

    bool try_push(T value) {
        std::lock_guard lock(mutex_);
        if (count_ == storage_.size())
            return false;
        storage_[tail_] = std::move(value);
        tail_ = next(tail_);
        ++count_;
        return true;
    }

    bool try_pop(T &value) {
        std::lock_guard lock(mutex_);
        if (count_ == 0)
            return false;
        value = std::move(storage_[head_]);
        head_ = next(head_);
        --count_;
        return true;
    }

    size_t size() const noexcept {
        std::lock_guard lock(mutex_);
        return count_;
    }

  private:
    size_t next(size_t index) const noexcept {
        ++index;
        return index == storage_.size() ? 0 : index;
    }

    mutable std::mutex mutex_;
    std::vector<T> storage_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
};

/*
    Transport Rate Limiter

    refill() 目前不是线程安全的，注意这一点。

    * burst_bytes: 目前设置为 2 * udp_pcket_cap，当空闲一会后突发流量，允许 busrt_bytes 流量短时通过，避免每个包都被严格限流，使发送更加平滑
*/
class token_bucket {
  public:
    token_bucket(size_t max_bandwidth_bps, size_t burst_bytes)
        : limit_bytes_per_sec_(static_cast<double>(max_bandwidth_bps / 8.0)),
          burst_bytes_(burst_bytes),
          available_bytes_(static_cast<double>(burst_bytes)),
          last_refill_(steady_clock::now()) {}

    token_bucket(const token_bucket &) = delete;
    token_bucket &operator=(const token_bucket &) = delete;

    err_code wait(size_t bytes, const std::stop_token &stop_token) {
        if (bytes == 0)
            return {};
        if (limit_bytes_per_sec_ < 1e-12 || burst_bytes_ == 0)
            return fleet_err::invalid_argument;

        while (!stop_token.stop_requested()) {
            refill();

            const auto requested = static_cast<double>(bytes);
            if (available_bytes_ >= requested) {
                available_bytes_ = std::max(0.0, available_bytes_ - requested);
                return {};
            }

            std::unique_lock lock(wait_mutex_);
            wait_cv_.wait_for(lock, stop_token, wait_duration(requested - available_bytes_),
                              [] { return false; });
        }

        return fleet_err::cancelled;
    }

    void cancel() noexcept {
        wait_cv_.notify_all();
    }

    void reset() noexcept {
        available_bytes_ = static_cast<double>(burst_bytes_);
        last_refill_ = steady_clock::now();
    }

  private:
    void refill() noexcept {
        const auto now = steady_clock::now();
        const auto secs = std::chrono::duration<double>(now - last_refill_).count();

        if (secs <= 0.0) return;

        available_bytes_ = std::min(
                static_cast<double>(burst_bytes_),
                available_bytes_ + secs * limit_bytes_per_sec_
        );
        last_refill_ = now;
    }

    steady_clock::duration wait_duration(double missing_bytes) const noexcept {
        const std::chrono::duration<double> secs {missing_bytes / limit_bytes_per_sec_};
        auto wait = std::chrono::duration_cast<steady_clock::duration>(secs);

        if (wait <= steady_clock::duration::zero())
            wait = steady_clock::duration{1};
        return wait;
    }

    std::mutex wait_mutex_;
    std::condition_variable_any wait_cv_;
    size_t burst_bytes_ = 0;
    const double limit_bytes_per_sec_;
    double available_bytes_ = 0.0;
    steady_clock::time_point last_refill_;
};

/*
    Arena Allocator for udp packets
*/
class packet_pool {
  public:
    explicit packet_pool(vector<uint8_t> &storage, uint32_t slot_count, uint32_t packet_cap)
        : send_storage_(&storage),
          packet_cap_(packet_cap),
          packet_sizes_(slot_count),
          free_slots_(slot_count),
          pending_slots_(slot_count) {
        for (uint32_t slot = 0; slot < slot_count; ++slot) {
            free_slots_.try_push(slot);
        }
    }

    packet_pool(const packet_pool &) = delete;
    packet_pool &operator=(const packet_pool &) = delete;

    result<uint32_t> acquire_slot() {
        uint32_t slot = 0;
        if (free_slots_.try_pop(slot))
            return slot;
        return fail(fleet_err::busy);
    }

    bytes_mut slot_buffer(uint32_t slot) noexcept {
        return bytes_mut(
                send_storage_->data() + static_cast<size_t>(slot * packet_cap_),
                static_cast<size_t>(packet_cap_)
        );
    }

    void release_slot(uint32_t slot) noexcept {
        packet_sizes_[slot] = 0;
        free_slots_.try_push(slot);
    }

    err_code enqueue(uint32_t slot, uint32_t size) {
        packet_sizes_[slot] = size;
        if (!pending_slots_.try_push(slot)) {
            release_slot(slot);
            return fleet_err::busy;
        }
        return {};
    }

    bool try_pop_pending(uint32_t &slot, uint32_t &size) {
        if (!pending_slots_.try_pop(slot))
            return false;
        size = packet_sizes_[slot];
        return true;
    }

    size_t pending_count() const noexcept { return pending_slots_.size(); }

    void discard_pending() noexcept {
        uint32_t slot = 0;
        uint32_t size = 0;
        while (try_pop_pending(slot, size)) {
            release_slot(slot);
        }
    }

  private:
    vector<uint8_t> *send_storage_ = nullptr;
    uint32_t packet_cap_ = 0;
    vector<uint32_t> packet_sizes_;
    fixed_ring_queue<uint32_t> free_slots_;
    fixed_ring_queue<uint32_t> pending_slots_;
};

class udp_send_queue {
  public:
    udp_send_queue(uint32_t send_queue_slots, uint32_t packet_cap)
        : send_storage_(make_send_storage(send_queue_slots, packet_cap)),
          packets_(send_storage_, send_queue_slots, packet_cap) {}

    udp_send_queue(const udp_send_queue &) = delete;
    udp_send_queue &operator=(const udp_send_queue &) = delete;

    result<uint32_t> acquire_slot() {
        return packets_.acquire_slot();
    }

    bytes_mut slot_buffer(uint32_t slot) noexcept { return packets_.slot_buffer(slot); }

    void release_slot(uint32_t slot) noexcept { packets_.release_slot(slot); }

    err_code enqueue(uint32_t slot, uint32_t size) {
        const auto error = packets_.enqueue(slot, size);
        if (error)
            return error;

        pending_cv_.notify_one();
        return {};
    }

    bool try_pop_pending(uint32_t &slot, uint32_t &size) {
        return packets_.try_pop_pending(slot, size);
    }

    size_t pending_count() const noexcept { return packets_.pending_count(); }

    bool wait_pop_pending(uint32_t &slot, uint32_t &size, const std::stop_token &stop_token) {
        while (!stop_token.stop_requested()) {
            if (try_pop_pending(slot, size))
                return true;

            std::unique_lock lock(pending_mutex_);
            pending_cv_.wait(lock, stop_token, [this] { return packets_.pending_count() != 0; });
        }
        return false;
    }

    void cancel_wait() noexcept { pending_cv_.notify_all(); }

    void discard_pending() noexcept { packets_.discard_pending(); }

  private:
    std::mutex pending_mutex_;
    std::condition_variable_any pending_cv_;
    std::vector<uint8_t> send_storage_;
    packet_pool packets_;
};

struct udp_channel::impl {
    std::string name;
    asio::io_context io;
    udp::socket socket;
    udp::endpoint remote;
    std::atomic_bool stopped{false};
    udp_send_queue send_queue;
    std::function<void(bytes_view)> receive_callback;
    std::optional<std::jthread> worker;

    impl(std::string channel_name, udp_channel_options options)
        : name(std::move(channel_name)),
          socket(io, udp::endpoint(udp::v4(), options.local_port)),
          remote(asio::ip::make_address(options.remote_address), options.remote_port),
          send_queue(options.send_queue_slots, UDP_PACKET_CAP) {
        tune_socket_buffers(socket);
        socket.non_blocking(true);
        worker.emplace([this](std::stop_token stop_token) { run(stop_token); });
    }

    void run(const std::stop_token &stop_token) {
        std::array<uint8_t, MAX_UDP_SIZE> buffer{};

        while (!stop_token.stop_requested() && !stopped.load(std::memory_order_acquire)) {
            send_pending(stop_token);
            receive_available(buffer);
            std::this_thread::sleep_for(std::chrono::microseconds{100});
        }
    }

    void send_pending(const std::stop_token &stop_token) {
        uint32_t slot = 0;
        uint32_t size = 0;
        while (!stop_token.stop_requested() && send_queue.try_pop_pending(slot, size)) {
            send_slot(slot, size);
            send_queue.release_slot(slot);
        }
    }

    void send_slot(uint32_t slot, uint32_t size) {
        if (!socket.is_open())
            return;

        asio::error_code error;
        socket.send_to(asio::buffer(send_queue.slot_buffer(slot).first(size)), remote, 0, error);
        if (error && error != asio::error::operation_aborted) {
            FLEET_DEBUG_KV(
                    KV("msg", "UDP Channel send failed"),
                    KV("name", name),
                    KV("bytes", size),
                    KV("error", error.message())
                    );
        }
    }

    void receive_available(std::array<uint8_t, MAX_UDP_SIZE> &buffer) {
        while (socket.is_open()) {
            udp::endpoint sender;
            asio::error_code error;
            const auto size = socket.receive_from(asio::buffer(buffer), sender, 0, error);
            if (!error) {
                if (!is_expected_sender(remote, sender))
                    continue;

                if (receive_callback)
                    receive_callback(bytes_view(buffer.data(), size));
                continue;
            }

            if (error == asio::error::would_block || error == asio::error::try_again ||
                error == asio::error::connection_reset) {
                return;
            }
            if (error != asio::error::operation_aborted && !stopped.load(std::memory_order_acquire)) {
                FLEET_DEBUG_KV(
                        KV("msg", "UDP Channel receive failed"),
                        KV("name", name),
                        KV("error", error.message())
                        );
            }
            return;
        }
    }

    void set_receive_callback(std::function<void(bytes_view)> callback) {
        receive_callback = std::move(callback);
    }

    void stop() noexcept {
        if (stopped.exchange(true))
            return;

        receive_callback = {};
        if (worker)
            worker->request_stop();
        send_queue.cancel_wait();
        worker.reset();

        asio::error_code ignored;
        socket.cancel(ignored);
        socket.close(ignored);
    }

    void flush_pending_sends(std::chrono::milliseconds timeout) noexcept {
        const auto deadline = steady_clock::now() + timeout;
        while (send_queue.pending_count() != 0 && steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::microseconds{100});
        }
    }
};

udp_channel::udp_channel(udp_channel_options options) : udp_channel({}, std::move(options)) {}

udp_channel::udp_channel(std::string name, udp_channel_options options)
    : impl_(std::make_unique<impl>(std::move(name), std::move(options))) {}

udp_channel::~udp_channel() { stop(); }

const std::string &udp_channel::name() const noexcept { return impl_->name; }

void udp_channel::stop() noexcept {
    if (impl_)
        impl_->stop();
}

void udp_channel::flush_pending_sends(std::chrono::milliseconds timeout) noexcept {
    if (impl_)
        impl_->flush_pending_sends(timeout);
}

void udp_channel::set_rx_callback(std::function<void(bytes_view)> callback) {
    impl_->set_receive_callback(std::move(callback));
}

err_code udp_channel::send(bytes_view data) {
    if (data.size() > UDP_PACKET_CAP)
        return fleet_err::invalid_argument;
    if (impl_->stopped.load(std::memory_order_acquire))
        return fleet_err::cancelled;

    auto slot = impl_->send_queue.acquire_slot();
    if (!slot)
        return slot.error();

    std::copy(data.begin(), data.end(), impl_->send_queue.slot_buffer(*slot).begin());
    return impl_->send_queue.enqueue(*slot, static_cast<uint32_t>(data.size()));
}

struct realtime_udp_channel::impl {
    std::string name;
    udp_channel_options options;

    asio::io_context io;
    udp::socket socket;
    udp::endpoint remote;

    token_bucket limit;
    std::atomic_bool stopped{false};
    udp_send_queue send_queue;
    std::function<void(bytes_view, steady_clock::time_point)> receive_callback;
    std::optional<std::jthread> rx_thread;
    std::optional<std::jthread> tx_thread;

    impl(std::string channel_name, udp_channel_options channel_options)
        : name(std::move(channel_name)), options(std::move(channel_options)),
          socket(io),
          remote(asio::ip::make_address(options.remote_address), options.remote_port),
          limit(options.max_bandwidth_bps, MOTOR_UDP_PACKET_CAP * 2),
          send_queue(options.send_queue_slots, MOTOR_UDP_PACKET_CAP) {
        socket.open(udp::v4());
        tune_socket_buffers(socket);
#ifdef __linux__
        enable_kernel_rx_timestamps(socket);
#endif
        socket.bind(udp::endpoint(udp::v4(), options.local_port));
        socket.non_blocking(true);
    }

    void ensure_started() {
        if (rx_thread || tx_thread)
            return;
        if (stopped.load()) {
            throw std::logic_error("stopped async udp channel cannot be restarted");
        }
        std::promise<err_code> rx_ready;
        auto rx_result = rx_ready.get_future();
        rx_thread.emplace([this, ready = std::move(rx_ready)](std::stop_token stop_token) mutable {
            rx_loop(stop_token, std::move(ready));
        });
        if (const auto error = rx_result.get()) {
            rx_thread->request_stop();
            rx_thread.reset();
            throw std::system_error(error);
        }
        tx_thread.emplace([this](std::stop_token stop_token) { tx_loop(stop_token); });
    }

    inline bool is_stopped() const noexcept {
        return stopped.load(std::memory_order_acquire);
    }

    void request_stop() noexcept {
        if (rx_thread)
            rx_thread->request_stop();
        if (tx_thread)
            tx_thread->request_stop();
        limit.cancel();
        send_queue.cancel_wait();
    }

    void set_receive_callback(
        std::function<void(bytes_view, steady_clock::time_point)> callback) {
        receive_callback = std::move(callback);
    }

    void discard_pending_sends() noexcept { send_queue.discard_pending(); }

    void tx_loop(const std::stop_token &stop_token) {
        while (!stop_token.stop_requested() && !is_stopped()) {
            uint32_t slot = 0;
            uint32_t size = 0;
            if (!send_queue.wait_pop_pending(slot, size, stop_token))
                break;

            const auto wait_error = limit.wait(size, stop_token);
            if (wait_error) {
                if (wait_error != fleet_err::cancelled) {
					FLEET_DEBUG_KV(
							KV("msg", "Async Udp Channel rate limit failed"),
							KV("name", name),
							KV("error", wait_error.message())
							);
                }
                send_queue.release_slot(slot);
                continue;
            }

            if (is_stopped()) {
                send_queue.release_slot(slot);
                continue;
            }

            if (socket.is_open()) {
                auto backoff = SEND_RETRY_INITIAL_BACKOFF;
                for (size_t attempt = 0;
                            attempt < SEND_RETRY_COUNT && !stop_token.stop_requested() && !is_stopped();
                            ++attempt) {
                    asio::error_code error;
                    socket.send_to(
                            asio::buffer(send_queue.slot_buffer(slot).first(size)),
                            remote,
                            0, error
                    );
                    if (!error || error == asio::error::operation_aborted)
                        break;
                    if (error == asio::error::would_block || error == asio::error::try_again) {
                        if (attempt + 1 == SEND_RETRY_COUNT) {
							FLEET_DEBUG_KV(
									KV("msg", "Async Udp Channel Queued Send Busy"),
									KV("name", name),
									KV("retires", SEND_RETRY_COUNT)
									);
                            break;
                        }
                        std::this_thread::sleep_for(backoff);
                        backoff *= 2;
                        continue;
                    }
					FLEET_DEBUG_KV(
							KV("msg", "Async Udp Channel Queued Send Failed"),
							KV("name", name),
							KV("error", error.message())
							);
                    break;
                }
            }

            send_queue.release_slot(slot);
        }
    }

    void rx_loop(const std::stop_token &stop_token, std::promise<err_code> ready) {
        const auto tuning_error = tune_current_thread(options.rx_thread, name + "-rx");
        ready.set_value(tuning_error);
        if (tuning_error)
            return;
        std::array<uint8_t, MAX_UDP_SIZE> buffer{};
        while (!stop_token.stop_requested() && !is_stopped()) {
#ifdef __linux__
            const auto packet = receive_timestamped(socket, buffer);
            const auto &error = packet.error;
            if (!error) {
                if (!is_expected_sender(remote, packet.sender))
                    continue;
                if (receive_callback) {
                    receive_callback(bytes_view(buffer.data(), packet.size), packet.received_at);
                }
                continue;
            }
#else
            udp::endpoint sender;
            asio::error_code error;
            const auto size = socket.receive_from(asio::buffer(buffer), sender, 0, error);
            if (!error) {
                if (!is_expected_sender(remote, sender))
                    continue;
                if (receive_callback) {
                    receive_callback(bytes_view(buffer.data(), size), steady_clock::now());
                }
                continue;
            }
#endif

            if (error == asio::error::would_block || error == asio::error::try_again) {
                std::this_thread::sleep_for(std::chrono::microseconds{100});
                continue;
            }
            if (error == asio::error::connection_reset) {
                continue;
            }
            if (error != asio::error::operation_aborted && !is_stopped()) {
				FLEET_DEBUG_KV(
						KV("msg", "Async Udp Channel Recieve Failed"),
						KV("name", name),
						KV("error", error.message())
						);
            }
        }
    }
};

realtime_udp_channel::realtime_udp_channel(udp_channel_options options)
    : realtime_udp_channel({}, std::move(options)) {}

realtime_udp_channel::realtime_udp_channel(std::string name, udp_channel_options options)
    : impl_(std::make_unique<impl>(std::move(name), std::move(options))) {}

realtime_udp_channel::~realtime_udp_channel() { stop(); }

const std::string &realtime_udp_channel::name() const noexcept { return impl_->name; }

void realtime_udp_channel::start() {
    impl_->ensure_started();
}

void realtime_udp_channel::set_rx_callback(std::function<void(bytes_view)> callback) {
    impl_->set_receive_callback(
        [callback = std::move(callback)](bytes_view bytes, steady_clock::time_point) {
            callback(bytes);
        });
}

void realtime_udp_channel::set_timestamped_rx_callback(
    std::function<void(bytes_view, steady_clock::time_point)> callback) {
    impl_->set_receive_callback(std::move(callback));
}

void realtime_udp_channel::stop() noexcept {
    if (!impl_ || impl_->stopped.exchange(true))
        return;

    asio::error_code ignored;
    impl_->request_stop();
    impl_->tx_thread.reset();
    impl_->rx_thread.reset();
    impl_->socket.cancel(ignored);
    impl_->socket.close(ignored);
}

void realtime_udp_channel::discard_pending_sends() noexcept {
    if (impl_)
        impl_->discard_pending_sends();
}

err_code realtime_udp_channel::send(bytes_view data) {
    if (data.size() > MOTOR_UDP_PACKET_CAP)
        return fleet_err::invalid_argument;

    auto packet = prepare_send();
    if (!packet)
        return packet.error();
    std::copy(data.begin(), data.end(), packet->buffer().begin());
    return commit_send(std::move(*packet), static_cast<uint32_t>(data.size()));
}

auto realtime_udp_channel::prepare_send() -> result<packet_buffer> {
    if (impl_->stopped.load(std::memory_order_acquire))
        return fail(fleet_err::cancelled);

    auto slot = impl_->send_queue.acquire_slot();
    if (!slot)
        return fail(slot.error());
    return packet_buffer(this, *slot, impl_->send_queue.slot_buffer(*slot));
}

err_code realtime_udp_channel::commit_send(packet_buffer &&packet, uint32_t size) {
    if (packet.owner_ != this || size > MOTOR_UDP_PACKET_CAP)
        return fleet_err::invalid_argument;
    if (impl_->stopped.load(std::memory_order_acquire))
        return fleet_err::cancelled;

    const auto slot = packet.slot_;
    packet.owner_ = nullptr;
    packet.buffer_ = {};
    return impl_->send_queue.enqueue(slot, size);
}

realtime_udp_channel::packet_buffer::packet_buffer(packet_buffer &&other) noexcept
    : owner_(std::exchange(other.owner_, nullptr)), slot_(other.slot_),
      buffer_(std::exchange(other.buffer_, {})) {}

realtime_udp_channel::packet_buffer &
realtime_udp_channel::packet_buffer::operator=(packet_buffer &&other) noexcept {
    if (this == &other)
        return *this;
    release();
    owner_ = std::exchange(other.owner_, nullptr);
    slot_ = other.slot_;
    buffer_ = std::exchange(other.buffer_, {});
    return *this;
}

void realtime_udp_channel::packet_buffer::release() noexcept {
    if (owner_ != nullptr)
        owner_->release_packet(slot_);
    owner_ = nullptr;
    buffer_ = {};
}

void realtime_udp_channel::release_packet(uint32_t slot) noexcept {
    if (impl_)
        impl_->send_queue.release_slot(slot);
}

} // namespace fleet
