cmake_minimum_required(VERSION 3.8)
project(station_tcp_connection)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(theora_image_transport REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
add_executable(station_tcp_connection
  src/station_tcp_connection_core.cpp
  src/station_tcp_connection_node.cpp
)
add_dependencies(station_tcp_connection ${PROJECT_NAME})

ament_target_dependencies(
  station_tcp_connection  
  rclcpp 
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  Boost
  theora_image_transport
  )





install(TARGETS
station_tcp_connection
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
launch
# config
DESTINATION share/${PROJECT_NAME}/
)

ament_package(
  
)