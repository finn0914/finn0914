cmake_minimum_required(VERSION 3.8)
project(transformation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED) 
find_package(stationui_frontend_rviz_plugin_msgs REQUIRED)

add_executable(drone_tf_broadcaster src/drone_tf_broadcaster.cpp)

ament_target_dependencies(
  drone_tf_broadcaster
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  px4_msgs   
  stationui_frontend_rviz_plugin_msgs
)

install(TARGETS
  drone_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
