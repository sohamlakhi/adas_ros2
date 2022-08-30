cmake_minimum_required(VERSION 3.5)
project(car_visualizer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach(PACKAGE 
  ament_cmake 
  rclcpp
  visualization_msgs
  geometry_msgs
  adas_common)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

add_library(car_bounding_box STATIC src/car_bounding_box.cpp)
ament_target_dependencies(car_bounding_box 
   geometry_msgs rclcpp
) #goemetry_msgs needs to be built before thsi library

target_include_directories(car_bounding_box
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Create Cpp executable
add_executable(car_visualizer_node src/car_visualizer.cpp)
ament_target_dependencies(car_visualizer_node 
  rclcpp visualization_msgs geometry_msgs adas_common
)

target_link_libraries(car_visualizer_node car_bounding_box)

# Install Cpp executables
install(TARGETS
  car_visualizer_node
  car_bounding_box
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
