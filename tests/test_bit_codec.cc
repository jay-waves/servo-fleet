#include <doctest/doctest.h>

#include "codec_bit.h"
#include "fleet/fleet.h"

#include <array>
#include <bit>
#include <cstdint>
#include <limits>

namespace {

namespace test_field {
using Mode = fleet::Bits<0, 3>;
using Kp = fleet::Bits<3, 12>;
using Kd = fleet::Bits<15, 9>;
using Pos = fleet::Bits<24, 16>;
using Spd = fleet::Bits<40, 12>;
using Torque = fleet::Bits<52, 12>;

using Byte0 = fleet::U8<0>;
using Word0 = fleet::U16<0>;
using Float0 = fleet::U32<0>;

using OutOfRange = fleet::Bits<60, 8>;
} // namespace test_field

using namespace fleet;

enum class TestCommand : std::uint8_t {
    idle = 0,
    torque_control = 1,
    position_control = 2,
    velocity_control = 3,
};

template <class Err> void require_ok(const Err &err) {
    // 你的 Error 用法是：if (err) return unexpected(err);
    // 所以 bool(err) == true 表示有错误。
    REQUIRE_FALSE(static_cast<bool>(err));
}

} // namespace

TEST_CASE("raw fields can be written and read back from a byte span") {
    std::array<uint8_t, 8> bytes{};

    require_ok(write<test_field::Mode>(bytes_mut(bytes), 0b101));
    require_ok(write<test_field::Kp>(bytes_mut(bytes), 0xabc));
    require_ok(write<test_field::Kd>(bytes_mut(bytes), 0x12f));

    auto mode = read<test_field::Mode>(bytes_view(bytes));
    auto kp = read<test_field::Kp>(bytes_view(bytes));
    auto kd = read<test_field::Kd>(bytes_view(bytes));

    REQUIRE(mode);
    REQUIRE(kp);
    REQUIRE(kd);

    REQUIRE(*mode == 0b101);
    REQUIRE(*kp == 0xabc);
    REQUIRE(*kd == 0x12f);
}

TEST_CASE("raw fields read values from fixed binary data") {
    const std::array<uint8_t, 8> bytes{0xb5, 0x79, 0x2f, 0, 0, 0, 0, 0};

    auto mode = read<test_field::Mode>(bytes_view(bytes));
    auto kp = read<test_field::Kp>(bytes_view(bytes));
    auto kd = read<test_field::Kd>(bytes_view(bytes));

    REQUIRE(mode);
    REQUIRE(kp);
    REQUIRE(kd);
    REQUIRE(*mode == 0b101);
    REQUIRE(*kp == 0xabc);
    REQUIRE(*kd == 0x12f);
}

TEST_CASE("raw fields write expected binary data") {
    std::array<uint8_t, 8> bytes{};

    require_ok(write<test_field::Mode>(bytes_mut(bytes), 0b101));
    require_ok(write<test_field::Kp>(bytes_mut(bytes), 0xabc));
    require_ok(write<test_field::Kd>(bytes_mut(bytes), 0x12f));

    const std::array<uint8_t, 8> expected{0xb5, 0x79, 0x2f, 0, 0, 0, 0, 0};
    REQUIRE(bytes == expected);
}

TEST_CASE("writing one can_field does not corrupt neighbouring fields") {
    std::array<uint8_t, 8> bytes{};

    require_ok(write<test_field::Mode>(bytes_mut(bytes), 0b011));
    require_ok(write<test_field::Kp>(bytes_mut(bytes), 0x123));
    require_ok(write<test_field::Kd>(bytes_mut(bytes), 0x1ab));

    require_ok(write<test_field::Kp>(bytes_mut(bytes), 0x456));

    REQUIRE(*read<test_field::Mode>(bytes_view(bytes)) == 0b011);
    REQUIRE(*read<test_field::Kp>(bytes_view(bytes)) == 0x456);
    REQUIRE(*read<test_field::Kd>(bytes_view(bytes)) == 0x1ab);
}

TEST_CASE("typed integer codec reads and writes uint8_t, uint16_t and uint32_t") {
    std::array<uint8_t, 8> bytes{};

    require_ok(write<test_field::Byte0>(bytes_mut(bytes), uint8_t{0x7f}));
    REQUIRE(*read<test_field::Byte0>(bytes_view(bytes)).transform(to_u8) == 0x7f);

    require_ok(write<test_field::Word0>(bytes_mut(bytes), uint16_t{0x1234}));
    REQUIRE(read<test_field::Word0>(bytes_view(bytes)).transform(to_u16) == 0x1234);

    require_ok(write<test_field::Pos>(bytes_mut(bytes), uint32_t{0xbeef}));
    REQUIRE(read<test_field::Pos>(bytes_view(bytes)).transform(to_u32) == 0xbeef);
}

TEST_CASE("typed integer codec reads values from fixed binary data") {
    const std::array<uint8_t, 8> bytes{0x12, 0x34, 0, 0xbe, 0xef, 0, 0, 0};

    auto byte0 = read<test_field::Byte0>(bytes_view(bytes)).transform(to_u8);
    auto word0 = read<test_field::Word0>(bytes_view(bytes)).transform(to_u16);
    auto pos = read<test_field::Pos>(bytes_view(bytes)).transform(to_u32);

    REQUIRE(byte0);
    REQUIRE(word0);
    REQUIRE(pos);
    REQUIRE(*byte0 == 0x12);
    REQUIRE(*word0 == 0x1234);
    REQUIRE(*pos == 0xbeef);
}

TEST_CASE("typed integer codec writes expected binary data") {
    std::array<uint8_t, 8> bytes{};

    require_ok(write<test_field::Word0>(bytes_mut(bytes), uint16_t{0x1234}));
    require_ok(write<test_field::Pos>(bytes_mut(bytes), uint32_t{0xbeef}));

    const std::array<uint8_t, 8> expected{0x12, 0x34, 0, 0xbe, 0xef, 0, 0, 0};
    REQUIRE(bytes == expected);
}

TEST_CASE("enum codec stores enum through its underlying integer value") {
    std::array<uint8_t, 8> bytes{};

    require_ok(write<test_field::Mode>(bytes_mut(bytes), enum_u8(TestCommand::torque_control)));

    // auto command = read<test_field::Mode>(bytes_view(bytes));
    auto raw = read<test_field::Mode>(bytes_view(bytes));

    // REQUIRE(command);
    REQUIRE(raw);

    // REQUIRE(*command == TestCommand::torque_control);
    REQUIRE(*raw == 1);
}

TEST_CASE("enum codec reads command from fixed binary data") {
    const std::array<uint8_t, 8> bytes{0x60, 0, 0, 0, 0, 0, 0, 0};

    auto command = read<test_field::Mode>(bytes_view(bytes)).transform([](uint32_t raw) {
        return static_cast<TestCommand>(raw);
    });

    REQUIRE(command);
    REQUIRE(*command == TestCommand::velocity_control);
}

TEST_CASE("enum codec writes expected binary data") {
    std::array<uint8_t, 8> bytes{};

    require_ok(write<test_field::Mode>(bytes_mut(bytes), enum_u8(TestCommand::velocity_control)));

    const std::array<uint8_t, 8> expected{0x60, 0, 0, 0, 0, 0, 0, 0};
    REQUIRE(bytes == expected);
}

TEST_CASE("signed integer codec preserves two's-complement bit pattern") {
    std::array<uint8_t, 8> bytes{};

    require_ok(write<test_field::Word0>(bytes_mut(bytes), static_cast<uint16_t>(int16_t{-123})));

    auto raw = read<test_field::Word0>(bytes_view(bytes));
    auto value = read<test_field::Word0>(bytes_view(bytes)).transform([](uint32_t v) {
        return static_cast<int16_t>(v);
    });

    REQUIRE(raw);
    REQUIRE(value);

    REQUIRE(*raw == static_cast<uint16_t>(-123));
    REQUIRE(*value == -123);
}

TEST_CASE("signed integer codec reads two's-complement value from fixed binary data") {
    const std::array<uint8_t, 8> bytes{0xff, 0x85, 0, 0, 0, 0, 0, 0};

    auto value = read<test_field::Word0>(bytes_view(bytes)).transform([](const uint32_t v) {
        return static_cast<int16_t>(v);
    });

    REQUIRE(value);
    REQUIRE(*value == -123);
}

TEST_CASE("signed integer codec writes expected two's-complement binary data") {
    std::array<uint8_t, 8> bytes{};

    require_ok(write<test_field::Word0>(bytes_mut(bytes), static_cast<uint16_t>(int16_t{-123})));

    const std::array<uint8_t, 8> expected{0xff, 0x85, 0, 0, 0, 0, 0, 0};
    REQUIRE(bytes == expected);
}

TEST_CASE("float codec writes expected binary data") {
    std::array<uint8_t, 8> bytes{};

    require_ok(write<test_field::Float0>(bytes_mut(bytes), std::bit_cast<uint32_t>(3.25f)));

    const std::array<uint8_t, 8> expected{0x40, 0x50, 0, 0, 0, 0, 0, 0};
    REQUIRE(bytes == expected);
}
