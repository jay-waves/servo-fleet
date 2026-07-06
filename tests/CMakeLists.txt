add_executable(fleet_cpp_tests
  test_main.cc
  test_golden_frames.cc
  test_motor_client.cc
  test_bit_codec.cc
  test_can_codec.cc
  test_motor_codec.cc
  test_udp_channel.cc
  test_realtime_perf.cc
)

target_link_libraries(fleet_cpp_tests
  PRIVATE
    ${FLEET_TARGET}
    spdlog::spdlog
    asio::asio
    nlohmann_json::nlohmann_json
    doctest::doctest
)

target_include_directories(fleet_cpp_tests PRIVATE ${PROJECT_SOURCE_DIR}/src)
set_target_properties(fleet_cpp_tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
add_test(NAME fleet_cpp_tests COMMAND fleet_cpp_tests)

add_executable(fleet_cpp_bench test_serialization_perf.cc)
target_link_libraries(fleet_cpp_bench
  PRIVATE
    ${FLEET_TARGET}
    spdlog::spdlog
    asio::asio
    nlohmann_json::nlohmann_json
    nanobench_headers
)

target_include_directories(fleet_cpp_bench PRIVATE ${PROJECT_SOURCE_DIR}/src)

set_target_properties(fleet_cpp_bench PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
