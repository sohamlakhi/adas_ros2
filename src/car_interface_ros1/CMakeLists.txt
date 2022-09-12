cmake_minimum_required(VERSION 2.8.3)
project(car_interface)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall)
string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type)
if(${build_type} MATCHES RELEASE)
    add_compile_options(-march=native)
endif()

# Build deps
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  adas_common
  dynamic_reconfigure
)

# Add any third party dependency

generate_dynamic_reconfigure_options(cfg/Trim.cfg)

# Message generation, if required
add_message_files(
  FILES
  CarCommand.msg
  CarCommandValue.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

# Runtime dependency
catkin_package(
  INCLUDE_DIRS include
  DEPENDS roscpp
  std_msgs
  message_runtime
  adas_common
  dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(car_interface src/car_interface.cpp)
target_link_libraries(car_interface
  ${catkin_LIBRARIES}
  adas_common_params
)
add_dependencies(car_interface car_interface_generate_messages_cpp car_interface_gencfg)

add_executable(car_interface_node src/car_interface_node.cpp)
target_link_libraries(car_interface_node
  ${catkin_LIBRARIES}
  car_interface
)
