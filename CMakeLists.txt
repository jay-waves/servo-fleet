cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_policy(SET CMP0156 NEW)

project(fleet_cpp_can VERSION 0.2.0 LANGUAGES CXX)

set(FLEET_TARGET fleet_cpp_can)
set(FLEET_ALIAS fleet_cpp_can::${FLEET_TARGET})

option(FLEET_BUILD_TESTS "Build ${FLEET_TARGET} tests" OFF)
option(FLEET_BUILD_PYBIND "Build experimental pybind11 bindings" OFF)
option(FLEET_BUILD_STATIC "Also build ${FLEET_TARGET} as a static library" OFF)

include(GNUInstallDirs)
include(cmake/dependencies.cmake)

add_compile_definitions(
	SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
	SPDLOG_NO_SOURCE_LOC
)

set(FLEET_SOURCES
  src/channel.cc
  src/device_motor.cc
  src/driver_motor.cc
  src/error.cc
  src/runtime.cc
  src/thread_tuning.cc
  src/codec_bit.cc
  src/checksum.cc
  src/codec_can.cc
  src/codec_motor.cc)

add_library(${FLEET_TARGET}_obj OBJECT ${FLEET_SOURCES})
target_compile_features(${FLEET_TARGET}_obj PUBLIC cxx_std_23)
target_compile_definitions(${FLEET_TARGET}_obj PUBLIC)
set_target_properties(${FLEET_TARGET}_obj PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${FLEET_TARGET}_obj
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(${FLEET_TARGET}_obj
  PRIVATE
    spdlog::spdlog
    asio::asio
    nlohmann_json::nlohmann_json)

set(FLEET_PLATFORM_LIBS)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  find_package(Threads REQUIRED)
  list(APPEND FLEET_PLATFORM_LIBS Threads::Threads)
  target_link_libraries(${FLEET_TARGET}_obj PRIVATE ${FLEET_PLATFORM_LIBS})
endif()

# dll
add_library(${FLEET_TARGET} SHARED)
target_link_libraries(${FLEET_TARGET} PUBLIC ${FLEET_TARGET}_obj PRIVATE ${FLEET_PLATFORM_LIBS})


# lib
if(FLEET_BUILD_STATIC)
  add_library(${FLEET_TARGET}_static STATIC)
  target_link_libraries(${FLEET_TARGET}_static PUBLIC ${FLEET_TARGET}_obj PRIVATE ${FLEET_PLATFORM_LIBS})
  set_target_properties(${FLEET_TARGET}_static PROPERTIES OUTPUT_NAME ${FLEET_TARGET})
endif()

if(FLEET_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
  add_subdirectory(pvt)
endif()

# if(FLEET_BUILD_PYBIND)
#   add_subdirectory(pybind)
# endif()
