cmake_minimum_required(VERSION 2.8.3)
project(car_visualizer)

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
  visualization_msgs
  geometry_msgs
  std_msgs
)

# Add any third party dependency

# Runtime dependency
catkin_package(
  INCLUDE_DIRS include
  DEPENDS roscpp
  adas_common
  visualization_msgs
  geometry_msgs
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(car_bounding_box STATIC src/car_bounding_box.cpp)
target_link_libraries(car_bounding_box
  ${catkin_LIBRARIES}
)

catkin_add_gtest(car_bounding_box_test src/test/car_bounding_box_test.cpp)
target_link_libraries(car_bounding_box_test car_bounding_box pthread)

add_library(car_visualizer STATIC src/car_visualizer.cpp)
target_link_libraries(car_visualizer
  ${catkin_LIBRARIES}
  car_bounding_box
  adas_common_params
)

add_executable(car_visualizer_node src/car_visualizer_node.cpp)
target_link_libraries(car_visualizer_node
  ${catkin_LIBRARIES}
  car_visualizer
)


