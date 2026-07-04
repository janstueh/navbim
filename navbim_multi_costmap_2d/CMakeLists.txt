cmake_minimum_required(VERSION 3.8)
project(navbim_multi_costmap_2d)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nav2_costmap_2d REQUIRED)
find_package(navbim_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)

# Use nav2_package() to ensure consistent build settings with nav2
nav2_package()

set(dependencies
  nav2_costmap_2d
  nav2_util
  navbim_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
)

# Build the core library containing multi_costmap_2d_ros
add_library(navbim_multi_costmap_2d_core SHARED
  src/multi_costmap_2d_ros.cpp
  src/multimap_static_layer.cpp
)

target_include_directories(navbim_multi_costmap_2d_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(navbim_multi_costmap_2d_core
  ${dependencies}
)

target_link_libraries(navbim_multi_costmap_2d_core
  nav2_costmap_2d::nav2_costmap_2d_core
)

pluginlib_export_plugin_description_file(nav2_costmap_2d navbim_multi_costmap_2d_plugins.xml)

install(TARGETS
  navbim_multi_costmap_2d_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(navbim_multi_costmap_2d_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  nav2_costmap_2d
  nav2_util
  navbim_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
)

ament_package()
