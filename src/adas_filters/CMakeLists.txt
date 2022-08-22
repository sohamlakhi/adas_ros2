cmake_minimum_required(VERSION 2.8.3)
project(adas_filters)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall)
string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type)
if(${build_type} MATCHES RELEASE)
    add_compile_options(-march=native)
endif()

# Build deps
find_package(catkin REQUIRED COMPONENTS
  roscpp
  adas_common
)

# Add any third party dependency

include(ExternalProject)
set(benchmark_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/benchmark)
ExternalProject_Add(
    benchmark-external

    GIT_REPOSITORY "https://github.com/google/benchmark.git"
    GIT_TAG d7aed73677888834d4e8af2b300d01bfb724b70f
    INSTALL_DIR "${benchmark_INSTALL_DIR}"

    UPDATE_COMMAND ""
    PATCH_COMMAND ""

    CMAKE_ARGS -DBENCHMARK_ENABLE_TESTING=OFF -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_BUILD_TYPE=RELEASE

    BUILD_COMMAND ${CMAKE_COMMAND} --build .
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install

    TEST_COMMAND "")

set(benchmark_INCLUDE_DIRS ${benchmark_INSTALL_DIR}/include)
set(benchmark_LINK_DIRECTORIES ${benchmark_INSTALL_DIR}/lib)

link_directories(
  ${benchmark_LINK_DIRECTORIES}
)

include_directories(
  include
  ${benchmark_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS roscpp adas_common
)

add_library(adas_filters STATIC
    src/discrete_filter_helpers.cpp
    src/discrete_system_helpers.cpp
    src/median_filter.cpp
    src/one_euro_filter.cpp)
set_property(TARGET adas_filters PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(adas_filters
  ${catkin_LIBRARIES}
)

add_library(adas_filters_test_scaffold src/test/scaffold.cpp)
set_property(TARGET adas_filters_test_scaffold PROPERTY POSITION_INDEPENDENT_CODE ON)

catkin_add_gtest(basics_test src/test/basics.cpp)
target_link_libraries(basics_test pthread adas_filters)

catkin_add_gtest(helpers_test src/test/helpers.cpp)
target_link_libraries(helpers_test pthread adas_filters_test_scaffold adas_filters)

catkin_add_gtest(median_filter_test src/test/median_filter.cpp)
target_link_libraries(median_filter_test pthread adas_filters_test_scaffold adas_filters)

catkin_add_gtest(one_euro_filter_test src/test/one_euro_filter.cpp)
target_link_libraries(one_euro_filter_test pthread adas_filters_test_scaffold adas_filters)

catkin_add_gtest(calc_shift_test src/test/calc_shift.cpp)
target_link_libraries(calc_shift_test pthread adas_filters_test_scaffold)

add_executable(adas_filters_perf_test src/test/perf.cpp)
target_link_libraries(adas_filters_perf_test
    adas_filters
    adas_filters_test_scaffold
    benchmark
    pthread)
add_dependencies(adas_filters_perf_test benchmark-external)

add_custom_target(run_perf_tests_adas_filters adas_filters_perf_test --benchmark_repetitions=10 --benchmark_counters_tabular=true DEPENDS adas_filters_perf_test)

# Nodelet, if required
