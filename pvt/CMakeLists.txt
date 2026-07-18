add_executable(motor_probe motor_probe.cc)
target_include_directories(motor_probe PRIVATE ${PROJECT_SOURCE_DIR}/src ${asio_SOURCE_DIR}/asio/include)
target_link_libraries(motor_probe PRIVATE ${FLEET_TARGET} spdlog::spdlog)
set_target_properties(motor_probe PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_executable(motor_pvt_control motor_pvt_control.cc)
target_include_directories(motor_pvt_control PRIVATE ${PROJECT_SOURCE_DIR}/src ${asio_SOURCE_DIR}/asio/include)
target_link_libraries(motor_pvt_control PRIVATE ${FLEET_TARGET} spdlog::spdlog)
set_target_properties(motor_pvt_control PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_executable(realtime_pvt_stress realtime_pvt_stress.cc)
  target_include_directories(realtime_pvt_stress PRIVATE ${PROJECT_SOURCE_DIR}/src ${asio_SOURCE_DIR}/asio/include)
  target_link_libraries(realtime_pvt_stress PRIVATE ${FLEET_TARGET} spdlog::spdlog Threads::Threads)
  set_target_properties(realtime_pvt_stress PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
  add_test(NAME realtime_pvt_stress_smoke
           COMMAND realtime_pvt_stress --profile baseline --duration 1 --warmup 0 --no-mlock)
  set_tests_properties(realtime_pvt_stress_smoke PROPERTIES TIMEOUT 10)
endif()
