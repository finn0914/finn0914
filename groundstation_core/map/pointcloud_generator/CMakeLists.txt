cmake_minimum_required(VERSION 3.8)
project(pointcloud_generator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)



# Add your executable
add_executable(pointcloud_generator_node src/pointcloud_generator.cpp)

# Link dependencies
ament_target_dependencies(pointcloud_generator_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  image_transport
)

# Install the executable
install(TARGETS pointcloud_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
