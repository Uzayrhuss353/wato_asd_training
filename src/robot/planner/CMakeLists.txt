cmake_minimum_required(VERSION 3.10)
project(planner)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Build library from planner source files
add_library(planner_lib
  src/planner_core.cpp
  src/planner_node.cpp
)

# Expose headers to compiler
target_include_directories(planner_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Only include actual ROS2 packages here
ament_target_dependencies(planner_lib
  rclcpp
  nav_msgs
)

# Build node executable
add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node planner_lib)
ament_target_dependencies(planner_node
  rclcpp
  nav_msgs
)

# Install targets
install(TARGETS
  planner_node
  planner_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config files if any
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()