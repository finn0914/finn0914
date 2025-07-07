cmake_minimum_required(VERSION 3.8)
project(keep_distance_pid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controllers_msg REQUIRED)

include_directories(
  include
)
add_executable(keep_distance_pid
  src/keep_distance_pid_core.cpp
  src/keep_distance_pid_node.cpp
  include/controllers/controllers.cpp
  )
add_dependencies(keep_distance_pid ${PROJECT_NAME})

ament_target_dependencies(
  keep_distance_pid
  controllers_msg
  rclcpp
  sensor_msgs
  geometry_msgs
  px4_msgs
  eigen3_cmake_module
  Eigen3
  )

install(TARGETS
  keep_distance_pid
  DESTINATION lib/${PROJECT_NAME})

# Install configuration files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
