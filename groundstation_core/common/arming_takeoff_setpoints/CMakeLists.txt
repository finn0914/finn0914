cmake_minimum_required(VERSION 3.8)
project(arming_takeoff_setpoints)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
include_directories(
  include
)

# Define the shared library for the component
add_library(arm_disarm_component SHARED
  src/plugins/arm_disarm_core.cpp
)
add_library(take_off_component SHARED
  src/plugins/take_off_core.cpp
)
add_library(follow_traj_component SHARED
  src/plugins/follow_traj_core.cpp
)
add_library(leader_follower_component SHARED
  src/plugins/leader_follower_core.cpp
)
# Register the shared library as a component


# Link dependencies to the shared library

rclcpp_components_register_node(
  arm_disarm_component
  PLUGIN "ARMDISARM"
  EXECUTABLE arm_disarm_core
)

rclcpp_components_register_node(
  take_off_component
  PLUGIN "TakeOff"
  EXECUTABLE take_off_core
)

rclcpp_components_register_node(
  follow_traj_component
  PLUGIN "FollowTraj"
  EXECUTABLE follow_traj_core
)
rclcpp_components_register_node(
  leader_follower_component
  PLUGIN "LeaderFollower"
  EXECUTABLE leader_follower_core
)
# Define the executable for standalone application
# add_executable(test_simple_planning_node
#   src/test_simple_planning_core.cpp
#   src/test_simple_planning_node.cpp

# )

# Link the executable with the shared library
# target_link_libraries(
#   # test_simple_planning_node
#   arm_disarm_component
# )

# Link dependencies to the executable

ament_target_dependencies(
  arm_disarm_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  px4_msgs
  eigen3_cmake_module
  Eigen3
  
)
ament_target_dependencies(
  take_off_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  px4_msgs
  eigen3_cmake_module
  Eigen3
)
ament_target_dependencies(
  follow_traj_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  px4_msgs
  eigen3_cmake_module
  Eigen3
  std_msgs
)
ament_target_dependencies(
  leader_follower_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  px4_msgs
  eigen3_cmake_module
  Eigen3
  std_msgs
  tf2
  tf2_ros
)
# Install the shared library
install(TARGETS
  arm_disarm_component
  take_off_component
  follow_traj_component
  leader_follower_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the executable
install(TARGETS
  # arming_takeoff_setpoints_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
