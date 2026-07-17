cmake_minimum_required(VERSION 3.16)
project(clean_calib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# Eigen — try system install first, then FetchContent fallback.
# ---------------------------------------------------------------------------
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  message(STATUS "Eigen3 not found via find_package; falling back to FetchContent")
  include(FetchContent)
  FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
  )
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "")
  set(BUILD_TESTING OFF CACHE BOOL "")
  set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "")
  FetchContent_MakeAvailable(Eigen)
endif()

# ---------------------------------------------------------------------------
# stb — header-only, vendored by user under external/stb/
# ---------------------------------------------------------------------------
set(STB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/stb)
if(NOT EXISTS ${STB_DIR}/stb_image.h OR NOT EXISTS ${STB_DIR}/stb_image_write.h)
  message(WARNING
    "stb headers not found in ${STB_DIR}.\n"
    "See external/stb/README.md for download instructions.\n"
    "Image I/O will fail until they are present.")
endif()

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(clean_calib
  src/image/image_io.cpp
  src/image/image_utils.cpp
  src/synthetic/checkerboard.cpp
  src/calib/projection.cpp
  src/synthetic/dataset.cpp
  src/synthetic/dataset_io.cpp
)

target_include_directories(clean_calib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external
)

target_link_libraries(clean_calib PUBLIC Eigen3::Eigen)
target_compile_options(clean_calib PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# CLI executable
# ---------------------------------------------------------------------------
add_executable(clean_calib_cli src/app/main.cpp)
set_target_properties(clean_calib_cli PROPERTIES OUTPUT_NAME clean_calib)
target_link_libraries(clean_calib_cli PRIVATE clean_calib)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
add_executable(clean_calib_tests
  tests/test_main.cpp
  tests/test_checkerboard.cpp
  tests/test_image_io.cpp
  tests/test_camera_types.cpp
  tests/test_image_utils.cpp
  tests/test_projection.cpp
  tests/test_synthetic_dataset.cpp
  tests/test_synthetic_dataset_io.cpp
  tests/test_harness_tests.cpp
)
target_link_libraries(clean_calib_tests PRIVATE clean_calib)
enable_testing()
add_test(NAME unit_tests COMMAND clean_calib_tests)

# ---------------------------------------------------------------------------
# Learning exercises — opt in so unfinished practice never blocks the project.
# ---------------------------------------------------------------------------
option(CLEAN_CALIB_BUILD_PRACTICE "Build standalone learning exercises" OFF)
if(CLEAN_CALIB_BUILD_PRACTICE)
  add_subdirectory(practice)
endif()

# ---------------------------------------------------------------------------
# Visual milestone demos — opt in; generated artifacts go to examples/output.
# ---------------------------------------------------------------------------
option(CLEAN_CALIB_BUILD_DEMOS "Build visual milestone demos" OFF)
if(CLEAN_CALIB_BUILD_DEMOS)
  add_subdirectory(examples/demos)
endif()
