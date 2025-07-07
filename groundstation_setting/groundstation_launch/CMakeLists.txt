cmake_minimum_required(VERSION 3.8)
project(groundstation_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# add_executable(map_param_uploader src/map_param_uploader_node.cpp)
# ament_target_dependencies(map_param_uploader rclcpp)
# install(TARGETS
#   map_param_uploader
#   DESTINATION lib/${PROJECT_NAME}
# )
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
