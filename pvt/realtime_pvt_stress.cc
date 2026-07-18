#include "codec_can.h"
#include "codec_motor.h"
#include "fleet/fleet.h"

#include <asio.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <numeric>
#include <optional>
#include <queue>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <sys/resource.h>
#include <thread>
#include <vector>

namespace fleet {
namespace {

using asio::ip::udp;
using clock_type = std::chrono::steady_clock;
using clock_time = clock_type::time_point;

constexpr auto control_period = 2500us;
constexpr auto feedback_max_age = 2ms;
constexpr size_t bus_count = 2;
constexpr size_t can_ports_per_bus = 2;
constexpr size_t motors_per_can_port = 8;
constexpr size_t motors_per_bus = can_ports_per_bus * motors_per_can_port;
constexpr size_t motor_count = bus_count * motors_per_bus;
constexpr uint32_t can_bitrate = 1'000'000;
constexpr uint32_t classic_can_frame_bits = 128;
constexpr auto can_frame_time =
    std::chrono::nanoseconds{1'000'000'000ULL * classic_can_frame_bits / can_bitrate};
constexpr uint32_t default_seed = 0x5EED1234U;
constexpr float pi = 3.14159265358979323846F;

struct options {
    std::string profile = "baseline";
    std::chrono::seconds duration = 30s;
    std::chrono::seconds warmup = 2s;
    uint32_t seed = default_seed;
    int irq_cpu = 0;
    std::vector<int> control_cpus{4, 5, 6, 7};
    int fifo_priority = 80;
    std::chrono::microseconds deadline_runtime = 500us;
    std::chrono::microseconds deadline = 2000us;
    uint32_t stale_fail_cycles = 3;
    bool lock_memory = false;
    std::string output;
};

struct distribution {
    double mean = 0.0;
    double p50 = 0.0;
    double p99 = 0.0;
    double max = 0.0;
};

struct benchmark_result {
    std::string profile;
    uint32_t seed = 0;
    uint64_t cycles = 0;
    uint64_t commands_sent = 0;
    uint64_t feedback_received = 0;
    uint64_t stale_feedback = 0;
    uint64_t command_errors = 0;
    uint64_t deadline_misses = 0;
    uint64_t voluntary_switches = 0;
    uint64_t involuntary_switches = 0;
    double cpu_utilization = 0.0;
    distribution jitter_us;
    distribution control_exec_us;
    distribution end_to_end_us;
    distribution rx_to_control_us;
    std::string error;
};

double elapsed_us(clock_time first, clock_time second) {
    return std::chrono::duration<double, std::micro>(second - first).count();
}

distribution summarize(std::vector<double> samples) {
    if (samples.empty())
        return {};
    std::sort(samples.begin(), samples.end());
    const auto at = [&](double quantile) {
        const auto index = static_cast<size_t>(
            std::ceil(quantile * static_cast<double>(samples.size()))) - 1U;
        return samples[std::min(index, samples.size() - 1U)];
    };
    return {
        .mean = std::accumulate(samples.begin(), samples.end(), 0.0) /
                static_cast<double>(samples.size()),
        .p50 = at(0.50),
        .p99 = at(0.99),
        .max = samples.back(),
    };
}

int64_t thread_cpu_time_ns() {
    timespec value{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0)
        return 0;
    return static_cast<int64_t>(value.tv_sec) * 1'000'000'000LL + value.tv_nsec;
}

struct pending_feedback {
    clock_time due;
    uint64_t order = 0;
    can_frame frame;
    udp::endpoint peer;
};

struct pending_later {
    bool operator()(const pending_feedback &lhs, const pending_feedback &rhs) const noexcept {
        if (lhs.due != rhs.due)
            return lhs.due > rhs.due;
        return lhs.order > rhs.order;
    }
};

class motor_mock_bus {
  public:
    explicit motor_mock_bus(uint32_t seed)
        : socket_(io_, udp::endpoint(asio::ip::make_address("127.0.0.1"), 0)), seed_(seed) {
        socket_.non_blocking(true);
    }

    ~motor_mock_bus() { stop(); }

    uint16_t port() const { return socket_.local_endpoint().port(); }

    void start() { worker_.emplace([this](std::stop_token token) { run(token); }); }

    void stop() {
        if (!worker_)
            return;
        worker_->request_stop();
        worker_.reset();
        asio::error_code ignored;
        socket_.close(ignored);
    }

    uint64_t commands() const noexcept { return commands_.load(); }
    uint64_t feedbacks() const noexcept { return feedbacks_.load(); }
    uint64_t decode_errors() const noexcept { return decode_errors_.load(); }

  private:
    struct prepared_feedback {
        std::chrono::microseconds delay{};
        clock_time ready{};
        can_frame frame;
        udp::endpoint peer;
    };

    static void collect_frame(void *context, const can_frame &frame) {
        static_cast<std::vector<can_frame> *>(context)->push_back(frame);
    }

    static uint64_t mix(uint64_t value) noexcept {
        value += 0x9E3779B97F4A7C15ULL;
        value = (value ^ (value >> 30U)) * 0xBF58476D1CE4E5B9ULL;
        value = (value ^ (value >> 27U)) * 0x94D049BB133111EBULL;
        return value ^ (value >> 31U);
    }

    std::chrono::microseconds feedback_delay(const can_frame &frame) const noexcept {
        const auto position = read<can_field::pvt::Position>(frame).value_or(0U);
        const uint64_t key = static_cast<uint64_t>(seed_) << 32U |
                             static_cast<uint64_t>(frame.port) << 24U |
                             static_cast<uint64_t>(frame.id) << 16U | position;
        return std::chrono::microseconds{50U + mix(key) % 301U};
    }

    std::optional<can_frame> make_status1(const can_frame &command) {
        const auto mode = read<can_field::pvt::Mode>(command);
        const auto position = read<can_field::pvt::Position>(command);
        const auto velocity = read<can_field::pvt::Velocity>(command);
        if (!mode || !position || !velocity || *mode != static_cast<uint8_t>(command_code::pvt_control))
            return std::nullopt;

        can_frame status;
        status.port = command.port;
        status.id = command.id;
        status.len = 8;
        if (write<can_field::feedback1::Type>(status, 1U) ||
            write<can_field::feedback1::Error>(status, 0U) ||
            write<can_field::feedback1::Position>(status, *position) ||
            write<can_field::feedback1::Velocity>(status, *velocity) ||
            write<can_field::feedback1::Current>(status, 2048U) ||
            write<can_field::feedback1::MotorTemp>(status, 130U) ||
            write<can_field::feedback1::MosfetTemp>(status, 130U)) {
            return std::nullopt;
        }
        return status;
    }

    void receive_commands() {
        std::array<std::vector<prepared_feedback>, can_ports_per_bus> prepared;
        for (auto &items : prepared)
            items.reserve(motors_per_can_port);
        while (true) {
            std::array<uint8_t, motor_codec::MAX_PACKET_SZ> bytes{};
            udp::endpoint peer;
            asio::error_code error;
            const auto size = socket_.receive_from(asio::buffer(bytes), peer, 0, error);
            if (error == asio::error::would_block || error == asio::error::try_again)
                break;
            if (error) {
                if (error != asio::error::operation_aborted)
                    ++decode_errors_;
                break;
            }

            std::vector<can_frame> frames;
            frames.reserve(motors_per_bus);
            if (motor_codec::decode_each(bytes_view(bytes.data(), size), &frames, &collect_frame)) {
                ++decode_errors_;
                continue;
            }

            for (const auto &frame : frames) {
                if (frame.port >= can_ports_per_bus) {
                    ++decode_errors_;
                    continue;
                }
                const auto status = make_status1(frame);
                if (!status) {
                    ++decode_errors_;
                    continue;
                }
                ++commands_;
                prepared[static_cast<size_t>(frame.port)].push_back({
                    .delay = feedback_delay(frame),
                    .frame = *status,
                    .peer = peer,
                });
            }
        }

        const auto received_at = clock_type::now();
        for (size_t port = 0; port < can_ports_per_bus; ++port) {
            auto &items = prepared[port];
            auto &bus_cursor = next_bus_slot_[port];
            bus_cursor = std::max(bus_cursor, received_at);
            for (auto &item : items) {
                bus_cursor += can_frame_time;
                item.ready = bus_cursor + item.delay;
            }
            std::ranges::sort(items, {}, &prepared_feedback::ready);
            for (const auto &item : items) {
                bus_cursor = std::max(bus_cursor, item.ready) + can_frame_time;
                pending_[port].push({.due = bus_cursor,
                                     .order = next_order_++,
                                     .frame = item.frame,
                                     .peer = item.peer});
            }
        }
    }

    void send_due_feedback() {
        const auto now = clock_type::now();
        for (size_t port = 0; port < can_ports_per_bus; ++port) {
            auto &queue = pending_[port];
            if (queue.empty() || queue.top().due > now)
                continue;
            if (last_feedback_sent_[port] != clock_time{} &&
                now - last_feedback_sent_[port] < can_frame_time)
                continue;
            const auto item = queue.top();
            queue.pop();

            std::array<uint8_t, motor_codec::MAX_PACKET_SZ> bytes{};
            const auto size = motor_codec::encode_into(bytes_mut(bytes), TYPE_CAN2UDP,
                                                       std::span<const can_frame>(&item.frame, 1));
            if (!size) {
                ++decode_errors_;
                continue;
            }
            asio::error_code error;
            socket_.send_to(asio::buffer(bytes.data(), *size), item.peer, 0, error);
            if (error) {
                ++decode_errors_;
            } else {
                ++feedbacks_;
                last_feedback_sent_[port] = clock_type::now();
            }
        }
    }

    void run(const std::stop_token &token) {
        while (!token.stop_requested()) {
            receive_commands();
            send_due_feedback();
            std::this_thread::sleep_for(20us);
        }
    }

    asio::io_context io_;
    udp::socket socket_;
    uint32_t seed_;
    std::optional<std::jthread> worker_;
    std::array<std::priority_queue<pending_feedback, std::vector<pending_feedback>, pending_later>,
               can_ports_per_bus>
        pending_;
    std::array<clock_time, can_ports_per_bus> next_bus_slot_{};
    std::array<clock_time, can_ports_per_bus> last_feedback_sent_{};
    uint64_t next_order_ = 0;
    std::atomic<uint64_t> commands_{0};
    std::atomic<uint64_t> feedbacks_{0};
    std::atomic<uint64_t> decode_errors_{0};
};

std::vector<int> parse_cpu_list(std::string_view text) {
    std::vector<int> result;
    while (!text.empty()) {
        const auto comma = text.find(',');
        const auto token = text.substr(0, comma);
        int cpu = -1;
        const auto [end, error] = std::from_chars(token.data(), token.data() + token.size(), cpu);
        if (error != std::errc{} || end != token.data() + token.size() || cpu < 0)
            throw std::runtime_error("invalid CPU list");
        result.push_back(cpu);
        if (comma == std::string_view::npos)
            break;
        text.remove_prefix(comma + 1);
    }
    if (result.empty())
        throw std::runtime_error("empty CPU list");
    return result;
}

options parse_options(int argc, char **argv) {
    options parsed;
    const auto value = [&](int &index) -> std::string_view {
        if (++index >= argc)
            throw std::runtime_error("missing option value");
        return argv[index];
    };
    for (int index = 1; index < argc; ++index) {
        const std::string_view arg = argv[index];
        if (arg == "--profile") {
            parsed.profile = value(index);
        } else if (arg == "--duration") {
            parsed.duration = std::chrono::seconds(std::stoul(std::string(value(index))));
        } else if (arg == "--warmup") {
            parsed.warmup = std::chrono::seconds(std::stoul(std::string(value(index))));
        } else if (arg == "--seed") {
            parsed.seed = static_cast<uint32_t>(std::stoul(std::string(value(index)), nullptr, 0));
        } else if (arg == "--irq-cpu") {
            parsed.irq_cpu = std::stoi(std::string(value(index)));
        } else if (arg == "--control-cpus") {
            parsed.control_cpus = parse_cpu_list(value(index));
        } else if (arg == "--fifo-priority") {
            parsed.fifo_priority = std::stoi(std::string(value(index)));
        } else if (arg == "--runtime-us") {
            parsed.deadline_runtime = std::chrono::microseconds(std::stoul(std::string(value(index))));
        } else if (arg == "--deadline-us") {
            parsed.deadline = std::chrono::microseconds(std::stoul(std::string(value(index))));
        } else if (arg == "--stale-fail-cycles") {
            parsed.stale_fail_cycles = static_cast<uint32_t>(
                std::stoul(std::string(value(index))));
        } else if (arg == "--mlock") {
            parsed.lock_memory = true;
        } else if (arg == "--output") {
            parsed.output = value(index);
        } else if (arg == "--help") {
            std::cout << "realtime_pvt_stress --profile baseline|fair|fifo|deadline-cpuset "
                         "[--duration SEC] [--warmup SEC] [--seed N] [--irq-cpu N] "
                         "[--control-cpus 4,5,6,7] [--runtime-us N] [--deadline-us N] "
                         "[--stale-fail-cycles N] [--mlock] [--output FILE]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown option: " + std::string(arg));
        }
    }
    if (parsed.duration <= 0s || parsed.deadline_runtime <= 0us ||
        parsed.deadline_runtime > parsed.deadline || parsed.deadline > control_period) {
        throw std::runtime_error("invalid duration/deadline parameters");
    }
    return parsed;
}

struct scheduling_pair {
    linux_thread_config rx;
    linux_thread_config control;
};

scheduling_pair scheduling_for(const options &opts) {
    scheduling_pair config;
    config.control.lock_memory = opts.lock_memory;
    const auto deadline = linux_thread_config{
        .policy = linux_scheduler_policy::deadline,
        .cpus = opts.control_cpus,
        .runtime = opts.deadline_runtime,
        .deadline = opts.deadline,
        .period = control_period,
    };
    if (opts.profile == "baseline") {
        return config;
    }

    config.rx.policy = linux_scheduler_policy::other;
    config.rx.cpus = {opts.irq_cpu};
    config.control.policy = linux_scheduler_policy::other;
    config.control.cpus = opts.control_cpus;
    if (opts.profile == "fair") {
        return config;
    }
    if (opts.profile == "fifo") {
        config.control.policy = linux_scheduler_policy::fifo;
        config.control.fifo_priority = opts.fifo_priority;
        return config;
    }
    if (opts.profile == "deadline-cpuset") {
        config.rx = deadline;
        config.rx.runtime = std::min(200us, opts.deadline_runtime);
        config.control = deadline;
        config.control.lock_memory = opts.lock_memory;
        return config;
    }
    throw std::runtime_error("unknown profile: " + opts.profile);
}

fleet_config make_config(const std::array<uint16_t, bus_count> &ports,
                         const linux_thread_config &rx_config) {
    fleet_config config;
    for (size_t bus = 0; bus < bus_count; ++bus) {
        const auto channel_name = "mock" + std::to_string(bus);
        config.channels.push_back({
            .name = channel_name,
            .max_bandwidth_bps = can_bitrate * can_ports_per_bus,
            .send_queue_slots = 1024,
            .remote_address = "127.0.0.1",
            .remote_port = ports[bus],
            .local_port = 0,
            .rx_thread = rx_config,
        });
        for (size_t motor = 0; motor < motors_per_bus; ++motor) {
            const auto can_port = motor / motors_per_can_port;
            const auto can_id = motor % motors_per_can_port + 1U;
            config.motors.push_back({
                .name = channel_name + "_motor" + std::to_string(motor),
                .channel = channel_name,
                .model = "EC-A2806-P2-36",
                .can_port = static_cast<can_port_t>(can_port),
                .can_id = static_cast<can_id_t>(can_id),
            });
        }
    }
    return config;
}

float position_for_raw(uint16_t raw) {
    return (static_cast<float>(raw) - 32767.0F) * 12.5F / 32767.0F;
}

uint16_t raw_from_snapshot(const motor::pvt_feedback_snapshot &snapshot) {
    const auto position_r = snapshot.position_deg.value * pi / 180.0F;
    const auto raw = std::llround((position_r + 12.5F) / 25.0F * 65535.0F);
    return static_cast<uint16_t>(std::clamp<long long>(raw, 0, 65535));
}

benchmark_result run_control(const options &opts, const linux_thread_config &tuning,
                             std::span<const std::shared_ptr<motor_interface>> motors) {
    benchmark_result result{.profile = opts.profile, .seed = opts.seed};
    const auto expected_cycles = static_cast<size_t>(
        (opts.duration + opts.warmup) / control_period + 16);
    std::vector<double> jitter;
    std::vector<double> execution;
    std::vector<double> end_to_end;
    std::vector<double> rx_to_control;
    jitter.reserve(expected_cycles);
    execution.reserve(expected_cycles);
    end_to_end.reserve(expected_cycles * motor_count);
    rx_to_control.reserve(expected_cycles * motor_count);

    std::vector<pvt_command> commands;
    commands.reserve(motor_count);
    std::array<uint64_t, motor_count> last_feedback_seq{};
    std::array<uint8_t, motor_count> consecutive_stale{};
    std::array<bool, motor_count> feedback_valid{};
    std::array<clock_time, 65536> sent_at{};
    std::array<bool, 65536> sent_valid{};

    if (const auto error = tune_current_thread(tuning, "fleet-control")) {
        result.error = error.message();
        return result;
    }

    rusage usage_start{};
    rusage usage_end{};
    int64_t cpu_started = 0;
    clock_time measurement_started{};
    bool measurement_stats_started = false;
    const auto started = clock_type::now();
    const auto measure_from = started + opts.warmup;
    const auto finish_at = measure_from + opts.duration;
    auto next_time = started;
    uint64_t iteration = 0;

    const auto send_targets = [&](uint16_t raw, clock_time now, bool measuring) {
        commands.clear();
        for (size_t index = 0; index < motors.size(); ++index) {
            commands.push_back({
                .device_id = motors[index]->id(),
                .gains = {.kp = 18.0F, .kd = 0.4F, .ki = 0.0F},
                .position_r = position_for_raw(raw),
                .velocity_radps = 0.0F,
                .torque_ff_nm = 0.0F,
            });
        }
        sent_at[raw] = now;
        sent_valid[raw] = true;
        if (!commands.empty()) {
            if (const auto error = command(std::span<const pvt_command>(commands))) {
                ++result.command_errors;
            } else if (measuring) {
                result.commands_sent += commands.size();
            }
        }
    };

    send_targets(20'000U, started, false);
    while (next_time + control_period <= finish_at) {
        next_time += control_period;
        std::this_thread::sleep_until(next_time);
        const auto woke_at = clock_type::now();
        const bool measuring = woke_at >= measure_from;
        bool deadline_missed = woke_at >= next_time + control_period;
        if (measuring) {
            if (!measurement_stats_started) {
                getrusage(RUSAGE_THREAD, &usage_start);
                cpu_started = thread_cpu_time_ns();
                measurement_started = woke_at;
                measurement_stats_started = true;
            }
            ++result.cycles;
            jitter.push_back(std::max(0.0, elapsed_us(next_time, woke_at)));
        }

        bool feedback_failure = false;
        for (size_t index = 0; index < motors.size(); ++index) {
            const auto snapshot = motors[index]->pvt_feedback();
            feedback_valid[index] = snapshot.fresh(woke_at, feedback_max_age) &&
                                    snapshot.error.value == motor_err::none;

            if (snapshot.coherent() && snapshot.position_deg.seq != last_feedback_seq[index]) {
                last_feedback_seq[index] = snapshot.position_deg.seq;
                if (measuring) {
                    ++result.feedback_received;
                    rx_to_control.push_back(elapsed_us(snapshot.position_deg.time, woke_at));
                    const auto raw = raw_from_snapshot(snapshot);
                    if (sent_valid[raw])
                        end_to_end.push_back(elapsed_us(sent_at[raw], woke_at));
                }
            }

            if (!feedback_valid[index]) {
                if (measuring) {
                    ++result.stale_feedback;
                    if (++consecutive_stale[index] >= opts.stale_fail_cycles &&
                        opts.stale_fail_cycles != 0U)
                        feedback_failure = true;
                }
            } else {
                consecutive_stale[index] = 0;
            }
        }

        if (feedback_failure) {
            result.error = std::to_string(opts.stale_fail_cycles) +
                           " consecutive stale feedback cycles (2 ms limit)";
            if (measuring && deadline_missed)
                ++result.deadline_misses;
            break;
        }

        ++iteration;
        const auto raw = static_cast<uint16_t>(20'000U + (iteration % 25'000U));
        send_targets(raw, woke_at, measuring);
        const auto finished = clock_type::now();
        if (measuring) {
            execution.push_back(elapsed_us(woke_at, finished));
            if (finished >= next_time + control_period)
                deadline_missed = true;
            if (deadline_missed)
                ++result.deadline_misses;
        }
    }

    const auto stopped = clock_type::now();
    const auto cpu_stopped = thread_cpu_time_ns();
    getrusage(RUSAGE_THREAD, &usage_end);
    if (measurement_stats_started) {
        result.cpu_utilization =
            static_cast<double>(cpu_stopped - cpu_started) /
            static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    stopped - measurement_started)
                                    .count());
        result.voluntary_switches =
            static_cast<uint64_t>(usage_end.ru_nvcsw - usage_start.ru_nvcsw);
        result.involuntary_switches =
            static_cast<uint64_t>(usage_end.ru_nivcsw - usage_start.ru_nivcsw);
    }
    result.jitter_us = summarize(std::move(jitter));
    result.control_exec_us = summarize(std::move(execution));
    result.end_to_end_us = summarize(std::move(end_to_end));
    result.rx_to_control_us = summarize(std::move(rx_to_control));
    return result;
}

std::string csv_header() {
    return "profile,seed,cycles,commands,feedback,stale,command_errors,deadline_misses,"
           "cpu_utilization,voluntary_switches,involuntary_switches,jitter_mean_us,jitter_p50_us,"
           "jitter_p99_us,jitter_max_us,control_exec_mean_us,control_exec_p99_us,control_exec_max_us,"
           "e2e_mean_us,e2e_p99_us,e2e_max_us,rx_control_mean_us,rx_control_p99_us,rx_control_max_us,error";
}

std::string csv_row(const benchmark_result &r) {
    std::ostringstream out;
    out << r.profile << ',' << r.seed << ',' << r.cycles << ',' << r.commands_sent << ','
        << r.feedback_received << ',' << r.stale_feedback << ',' << r.command_errors << ','
        << r.deadline_misses << ',' << r.cpu_utilization << ',' << r.voluntary_switches << ','
        << r.involuntary_switches << ',' << r.jitter_us.mean << ',' << r.jitter_us.p50 << ','
        << r.jitter_us.p99 << ',' << r.jitter_us.max << ',' << r.control_exec_us.mean << ','
        << r.control_exec_us.p99 << ',' << r.control_exec_us.max << ',' << r.end_to_end_us.mean << ','
        << r.end_to_end_us.p99 << ',' << r.end_to_end_us.max << ',' << r.rx_to_control_us.mean << ','
        << r.rx_to_control_us.p99 << ',' << r.rx_to_control_us.max << ',' << r.error;
    return out.str();
}

void write_result(const options &opts, const benchmark_result &result) {
    std::cout << csv_header() << '\n' << csv_row(result) << '\n';
    if (opts.output.empty())
        return;
    const bool add_header = !std::filesystem::exists(opts.output) ||
                            std::filesystem::file_size(opts.output) == 0;
    std::ofstream output(opts.output, std::ios::app);
    if (!output)
        throw std::runtime_error("cannot open output file");
    if (add_header)
        output << csv_header() << '\n';
    output << csv_row(result) << '\n';
}

int run(int argc, char **argv) {
    const auto opts = parse_options(argc, argv);
    const auto scheduling = scheduling_for(opts);

    std::array<std::unique_ptr<motor_mock_bus>, bus_count> mocks;
    std::array<uint16_t, bus_count> ports{};
    for (size_t bus = 0; bus < bus_count; ++bus) {
        mocks[bus] = std::make_unique<motor_mock_bus>(opts.seed + static_cast<uint32_t>(bus));
        ports[bus] = mocks[bus]->port();
        mocks[bus]->start();
    }

    if (const auto error = init(make_config(ports, scheduling.rx))) {
        std::cerr << "runtime init failed: " << error.message() << '\n';
        return 2;
    }
    const auto motors = lookup<motor_interface>();
    if (motors.size() != motor_count) {
        shutdown();
        std::cerr << "expected 32 motors, got " << motors.size() << '\n';
        return 2;
    }

    std::promise<benchmark_result> completion;
    auto result_future = completion.get_future();
    std::jthread control([&](std::stop_token) {
        completion.set_value(run_control(opts, scheduling.control, motors));
    });
    control.join();
    auto result = result_future.get();
    shutdown();

    uint64_t mock_commands = 0;
    uint64_t mock_feedback = 0;
    uint64_t mock_errors = 0;
    for (auto &mock : mocks) {
        mock_commands += mock->commands();
        mock_feedback += mock->feedbacks();
        mock_errors += mock->decode_errors();
        mock->stop();
    }
    if (mock_errors != 0 && result.error.empty())
        result.error = "mock_errors=" + std::to_string(mock_errors);
    std::cerr << "mock_commands=" << mock_commands << " mock_feedback=" << mock_feedback
              << " mock_errors=" << mock_errors << " can_frame_time_ns="
              << can_frame_time.count() << " per_can_port_load_bps="
              << motors_per_can_port * 2U * classic_can_frame_bits *
                     (1'000'000ULL /
                      std::chrono::duration_cast<std::chrono::microseconds>(control_period).count())
              << '\n';
    write_result(opts, result);
    return result.error.empty() ? 0 : 3;
}

} // namespace
} // namespace fleet

int main(int argc, char **argv) {
    try {
        return fleet::run(argc, argv);
    } catch (const std::exception &error) {
        fleet::shutdown();
        std::cerr << "fatal: " << error.what() << '\n';
        return 1;
    }
}
