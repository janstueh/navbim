cmake_minimum_required(VERSION 3.8)
project(navbim_smoother)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(backward_ros REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navbim_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_smoother REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

nav2_package()

set(executable_name smoother_server)
set(library_name navbim_${executable_name}_core)

set(dependencies
  angles
  navbim_msgs
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_smoother
  nav2_util
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  tf2
  tf2_ros
)

# Main library
add_library(${library_name} SHARED
  src/navbim_smoother.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name}
  ${dependencies}
)
target_link_libraries(${library_name}
  Eigen3::Eigen
)

# Main executable
add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(${executable_name} ${library_name})
ament_target_dependencies(${executable_name}
  ${dependencies}
)

# Simple Smoother plugin
add_library(navbim_simple_smoother SHARED
  src/simple_smoother.cpp
)
target_include_directories(navbim_simple_smoother
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(navbim_simple_smoother
  ${dependencies}
)

# Savitzky Golay Smoother plugin
add_library(navbim_savitzky_golay_smoother SHARED
  src/savitzky_golay_smoother.cpp
)
target_include_directories(navbim_savitzky_golay_smoother
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(navbim_savitzky_golay_smoother
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

rclcpp_components_register_nodes(${library_name} "navbim_smoother::SmootherServer")

install(
  TARGETS ${library_name} navbim_simple_smoother navbim_savitzky_golay_smoother
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name} navbim_simple_smoother navbim_savitzky_golay_smoother)
ament_export_dependencies(
  angles
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_smoother
  nav2_util
  nav_2d_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  Eigen3
)
ament_export_targets(${library_name})
ament_package()
