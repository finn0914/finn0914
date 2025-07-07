cmake_minimum_required(VERSION 3.8)
project(drone_info_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ————————————— find all the ROS-2 / ament packages you need —————————————————
find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(nav_msgs          REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros           REQUIRED)
find_package(px4_msgs          REQUIRED)
find_package(rviz_visual_tools REQUIRED)   # <--- this brings in the headers+libs
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)


# ————————————— define your node target —————————————————
add_executable(drone_info_rviz
  src/drone_info_rviz.cpp
  src/drone_info_rviz_main.cpp
)

# ————————————— ensure your own headers are found —————————————————
target_include_directories(drone_info_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ————————————— pull in include paths & link libs from each package —————————————————
ament_target_dependencies(drone_info_rviz
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  Eigen3
  px4_msgs
  rviz_visual_tools      # <--- this adds both the include dirs *and* links the right target
)

# ————————————— install rules —————————————————
install(TARGETS
  drone_info_rviz
  DESTINATION lib/${PROJECT_NAME}
)
# Install configuration files
install(DIRECTORY
  launch
  # config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
