cmake_minimum_required(VERSION 3.5)
project(adas_common)

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

#find packages that ALL the library code depends on
foreach(PACKAGE 
  ament_cmake
  rclcpp
  nav_msgs
  geometry_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()


#Making libraries STATIC for now
include_directories(include) #adds include property INCLUDE_DIRECTORIES relative to src of current cmakelists.txt. The compiler can now look for these header files

#bytes_helper (contains no header only libraries)
#set(BYTES_HELPER_HEADER_FILES include/${PROJECT_NAME}/bytes_helper.hpp)
add_library(adas_common_bytes_helper STATIC src/bytes_helper.cpp) #${BYTES_HELPER_HEADER_FILES} -> try without compiling headers (find with target_include_directories)
set_property(TARGET adas_common_bytes_helper PROPERTY POSITION_INDEPENDENT_CODE ON)

#specifies which include directories to use while building and installing the target
target_include_directories(adas_common_bytes_helper
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(adas_common_bytes_helper HAS_LIBRARY_TARGET)

install(
  TARGETS adas_common_bytes_helper
  EXPORT adas_common_bytes_helper
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#checksum 
add_library(adas_common_checksum STATIC src/checksum.cpp)
set_property(TARGET adas_common_checksum PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(adas_common_checksum
  adas_common_bytes_helper
)

#specifies which include directories to use while building and installing the target
target_include_directories(adas_common_checksum
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(adas_common_checksum HAS_LIBRARY_TARGET)

install(
  TARGETS adas_common_checksum
  EXPORT adas_common_checksum
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#controllers
add_library(adas_common_controllers STATIC src/pid_controller.cpp src/lateral_controllers.cpp)
set_property(TARGET adas_common_controllers PROPERTY POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(adas_common_controllers
  rclcpp
) #needed for tf2/utils.h

#specifies which include directories to use while building and installing the target
target_include_directories(adas_common_controllers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(adas_common_controllers HAS_LIBRARY_TARGET)

install(
  TARGETS adas_common_controllers
  EXPORT adas_common_controllers
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#occupancy grid utils
add_library(adas_common_occupancy_grid_utils STATIC src/occupancy_grid_utils.cpp)
set_property(TARGET adas_common_occupancy_grid_utils PROPERTY POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(adas_common_occupancy_grid_utils
  rclcpp
)

#specifies which include directories to use while building and installing the target
target_include_directories(adas_common_occupancy_grid_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(adas_common_occupancy_grid_utils HAS_LIBRARY_TARGET)

install(
  TARGETS adas_common_occupancy_grid_utils
  EXPORT adas_common_occupancy_grid_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/adas_common
  DESTINATION include
)

#encode-decode
add_library(adas_common_encode_decode STATIC src/encode_decode.cpp)
set_property(TARGET adas_common_encode_decode PROPERTY POSITION_INDEPENDENT_CODE ON)

#specifies which include directories to use while building and installing the target
target_include_directories(adas_common_encode_decode
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(adas_common_encode_decode HAS_LIBRARY_TARGET)

install(
  TARGETS adas_common_encode_decode
  EXPORT adas_common_encode_decode
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#params
add_library(adas_common_params STATIC src/params.cpp)
set_property(TARGET adas_common_params PROPERTY POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(adas_common_params
  rclcpp geometry_msgs nav_msgs
)

#specifies which include directories to use while building and installing the target
target_include_directories(adas_common_params
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(adas_common_params HAS_LIBRARY_TARGET)

install(
  TARGETS adas_common_params
  EXPORT adas_common_params
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

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
