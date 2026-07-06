include(FetchContent)

FetchContent_Declare(
  asio
  URL https://github.com/chriskohlhoff/asio/archive/refs/tags/asio-1-34-2.zip
)

set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  spdlog
  URL https://github.com/gabime/spdlog/archive/refs/tags/v1.17.0.zip
)

FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.zip
)

FetchContent_MakeAvailable(
  asio
  spdlog
  nlohmann_json)

if(FLEET_BUILD_TESTS)
  FetchContent_Declare(
    doctest
    URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.12.zip
  )
  FetchContent_Declare(
    nanobench
    URL https://github.com/martinus/nanobench/archive/refs/tags/v4.3.11.zip
    SOURCE_SUBDIR cmake-do-not-add
  )
  FetchContent_MakeAvailable(doctest nanobench)

  add_library(nanobench_headers INTERFACE)
  target_include_directories(nanobench_headers INTERFACE "${nanobench_SOURCE_DIR}/src/include")
endif()

if(FLEET_BUILD_PYBIND)
  FetchContent_Declare(
    pybind11
    URL https://github.com/pybind/pybind11/archive/refs/tags/v3.0.4.zip
  )
  FetchContent_MakeAvailable(pybind11)
endif()

if(NOT TARGET asio::asio)
  add_library(asio::asio INTERFACE IMPORTED)
  target_include_directories(asio::asio INTERFACE ${asio_SOURCE_DIR}/asio/include)
  target_compile_definitions(asio::asio INTERFACE ASIO_STANDALONE)
endif()
