cmake_minimum_required(VERSION 3.8)
project(generate_trajectory)
find_package(stationui_frontend_rviz_plugin_msgs REQUIRED)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(
  include
)

# Define the shared library for the component
add_library(planning_for_leader_component SHARED
  src/plugins/planning_for_leader_core.cpp
)

# Register the shared library as a component


# Link dependencies to the shared library

rclcpp_components_register_node(
  planning_for_leader_component
  PLUGIN "PlanningForLeader"
  EXECUTABLE planning_for_leader_core
)

# Link dependencies to the executable

ament_target_dependencies(
  planning_for_leader_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  px4_msgs
  eigen3_cmake_module
  Eigen3
  stationui_frontend_rviz_plugin_msgs
)

# Install the shared library
install(TARGETS
  planning_for_leader_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the executable
install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
