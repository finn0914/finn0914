cmake_minimum_required(VERSION 3.8)
project(tf_tree)

################################################################################
# Compile options
################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(tf2_ros      REQUIRED)
find_package(tf2          REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs     REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

################################################################################
# Targets
################################################################################
add_executable(topics_conversion_main
  src/topics_conversion_main.cpp
)

ament_target_dependencies(topics_conversion_main
  rclcpp
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  px4_msgs
)

################################################################################
# Installation
################################################################################
# Install the executable into <install>/tf_tree/lib/tf_tree/
install(TARGETS 
  topics_conversion_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
ament_package()
