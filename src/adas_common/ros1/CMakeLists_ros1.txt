cmake_minimum_required(VERSION 2.8.3)
project(adas_common)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall)
string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type)
if(${build_type} MATCHES RELEASE)
    add_compile_options(-march=native)
endif()

# Build deps
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  message_generation
)

# Add any third party dependency

catkin_python_setup()

# Message generation, if required
add_message_files(
  FILES
  ProfileTime.msg
  StateWithCovariance.msg
)

generate_messages()

# Runtime dependency
catkin_package(
  INCLUDE_DIRS include
  DEPENDS roscpp nav_msgs geometry_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(adas_common_bytes_helper STATIC src/bytes_helper.cpp)
set_property(TARGET adas_common_bytes_helper PROPERTY POSITION_INDEPENDENT_CODE ON)

catkin_add_gtest(adas_common_bytes_helper_test src/test/bytes_helper_test.cpp)
target_link_libraries(adas_common_bytes_helper_test pthread adas_common_bytes_helper)

add_library(adas_common_controllers STATIC src/pid_controller.cpp src/lateral_controllers.cpp)
set_property(TARGET adas_common_controllers PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(adas_common_controllers
  ${catkin_LIBRARIES}
)

catkin_add_gtest(pid_controller_test src/test/pid_controller_test.cpp)
target_link_libraries(pid_controller_test pthread adas_common_controllers)

catkin_add_gtest(lateral_controllers_test src/test/lateral_controllers_test.cpp)
target_link_libraries(lateral_controllers_test pthread adas_common_controllers)

add_library(adas_common_params STATIC src/params.cpp)
set_property(TARGET adas_common_params PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(adas_common_params
  ${catkin_LIBRARIES}
)

add_library(adas_common_occupancy_grid_utils STATIC src/occupancy_grid_utils.cpp)
set_property(TARGET adas_common_occupancy_grid_utils PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(adas_common_occupancy_grid_utils
  ${catkin_LIBRARIES}
)

add_library(adas_common_profiler STATIC src/profiler.cpp)
set_property(TARGET adas_common_profiler PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(adas_common_profiler
  ${catkin_LIBRARIES}
  adas_common_params
)
add_dependencies(adas_common_profiler adas_common_generate_messages_cpp)

catkin_add_gtest(time_diff_test src/test/time_diff_test.cpp)
target_link_libraries(time_diff_test pthread)

catkin_add_gtest(occupancy_grid_utils_test src/test/occupancy_grid_utils_test.cpp)
target_link_libraries(occupancy_grid_utils_test adas_common_occupancy_grid_utils pthread)

add_library(adas_common_ros_time_diff STATIC src/ros_time_diff.cpp)
set_property(TARGET adas_common_ros_time_diff PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(adas_common_ros_time_diff
  ${catkin_LIBRARIES}
)

add_library(adas_common_checksum STATIC src/checksum.cpp)
set_property(TARGET adas_common_checksum PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(adas_common_checksum
  ${catkin_LIBRARIES}
  adas_common_bytes_helper
)
catkin_add_gtest(checksum_test src/test/checksum_test.cpp)
target_link_libraries(checksum_test adas_common_checksum pthread)

add_library(adas_common_encode_decode STATIC src/encode_decode.cpp)
set_property(TARGET adas_common_encode_decode PROPERTY POSITION_INDEPENDENT_CODE ON)

catkin_add_gtest(encode_decode_test src/test/encode_decode_test.cpp)
target_link_libraries(encode_decode_test adas_common_encode_decode pthread)

add_library(adas_common_serial_protocol STATIC src/serial_protocol.cpp)
set_property(TARGET adas_common_serial_protocol PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(adas_common_serial_protocol
  ${catkin_LIBRARIES}
  adas_common_bytes_helper
  adas_common_checksum
)

catkin_add_gtest(serial_protocol_test src/test/serial_protocol_test.cpp)
target_link_libraries(serial_protocol_test adas_common_serial_protocol pthread)

catkin_add_gtest(lerp_test src/test/lerp_test.cpp)
target_link_libraries(lerp_test pthread)

catkin_add_gtest(span_test src/test/span_test.cpp)
target_link_libraries(span_test pthread)

# Nodelet, if required

