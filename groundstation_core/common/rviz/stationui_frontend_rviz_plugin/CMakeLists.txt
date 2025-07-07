cmake_minimum_required(VERSION 3.8)
project(stationui_frontend_rviz_plugin)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rviz_common REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(stationui_frontend_rviz_plugin_msgs REQUIRED)

include_directories(include)

set(CMAKE_AUTOMOC ON)
# it;s necessary to header to add_library-> it back to process of building the code with set(CMAKE_AUTOMOC ON)
add_library(${PROJECT_NAME} SHARED
  src/main_panel.cpp
  src/tf_listener.cpp
  src/drone.cpp
  src/drone_status.cpp
  src/led_indicator.cpp
  src/send_state.cpp
  src/send_armed.cpp
  src/send_setpoint.cpp
  include/main_panel.hpp
  include/tf_listener.hpp
  include/drone.hpp
  include/drone_status.hpp
  include/led_indicator.hpp
  include/send_state.hpp
  include/send_armed.hpp
  include/send_setpoint.hpp
)
target_link_libraries(
    ${PROJECT_NAME} Qt5::Widgets ${catkin_LIBRARIES} # ${QT_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  Qt5
  rviz_common
  px4_msgs
  stationui_frontend_rviz_plugin_msgs
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)
install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}/
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)
ament_package()
