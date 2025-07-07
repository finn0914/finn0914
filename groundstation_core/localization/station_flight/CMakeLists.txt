cmake_minimum_required(VERSION 3.8)
project(station_flight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(stationui_frontend_rviz_plugin_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/XyzrpyStamped.msg"
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${station_flight_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}
)
add_executable(pose_to_tf_node src/pose_to_tf_core.cpp
                              src/pose_to_tf_node.cpp)

target_link_libraries(pose_to_tf_node
  ${catkin_LIBRARIES}
  station_flight__rosidl_typesupport_cpp
)

ament_target_dependencies(pose_to_tf_node
  rclcpp 
  # station_flight
  geometry_msgs
  tf2
  tf2_ros
  px4_msgs
  stationui_frontend_rviz_plugin_msgs
)

install(
  TARGETS
  pose_to_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  # scripts/
  DESTINATION
  share/${PROJECT_NAME}/scripts
# lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY
  scripts/
  DESTINATION
  lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
