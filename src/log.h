#pragma once

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

#define FLEET_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define FLEET_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define FLEET_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define FLEET_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)

#include <string_view>

// only for slow debug
template <class T> struct KV { std::string_view k; const T& v; };

template <class T> KV(std::string_view, T const&) -> KV<T>;


template<typename... Args>
void FLEET_DEBUG_KV(Args&&... args) {
	std::string buf;
	buf.reserve(256);

	auto append = [&buf](auto const&x){ 
		fmt::format_to(std::back_inserter(buf), "\033[34m{}\033[0m={}, ", x.k, x.v);
	};

	(append(args), ...);
	FLEET_INFO(buf);
}

