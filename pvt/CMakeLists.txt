add_executable(motor_probe motor_probe.cc)
target_include_directories(motor_probe PRIVATE ${PROJECT_SOURCE_DIR}/src ${asio_SOURCE_DIR}/asio/include)
target_link_libraries(motor_probe PRIVATE ${FLEET_TARGET} spdlog::spdlog)
set_target_properties(motor_probe PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_executable(motor_pvt_control motor_pvt_control.cc)
target_include_directories(motor_pvt_control PRIVATE ${PROJECT_SOURCE_DIR}/src ${asio_SOURCE_DIR}/asio/include)
target_link_libraries(motor_pvt_control PRIVATE ${FLEET_TARGET} spdlog::spdlog)
set_target_properties(motor_pvt_control PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
