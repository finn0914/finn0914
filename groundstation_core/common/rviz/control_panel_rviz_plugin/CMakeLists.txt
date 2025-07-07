cmake_minimum_required(VERSION 3.8)
project(control_panel_rviz_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)

include_directories(include)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# add_library(${PROJECT_NAME} SHARED
#   src/control_panel.cpp
# )
# it;s necessary to header to add_library-> it back to process of building the code with set(CMAKE_AUTOMOC ON)
add_library(${PROJECT_NAME} SHARED src/control_panel.cpp include/control_panel_rviz_plugin/control_panel.hpp)
target_link_libraries(
    ${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rviz_common
  rviz_default_plugins
  Qt5
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(FILES plugins/plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/
)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_package()


