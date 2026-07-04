cmake_minimum_required(VERSION 3.8)
project(navbim_bt_navigator)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navbim_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()

set(executable_name bt_navigator)

set(library_name ${executable_name}_core)

set(dependencies
  ament_index_cpp
  geometry_msgs
  nav2_behavior_tree
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  navbim_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_library(${library_name} SHARED
  src/bt_navigator.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${executable_name} ${library_name})
ament_target_dependencies(${executable_name}
  ${dependencies}
)

add_library(navbim_navigate_to_pose_navigator SHARED
  src/navigators/navigate_to_pose.cpp
)
target_include_directories(navbim_navigate_to_pose_navigator
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(navbim_navigate_to_pose_navigator
  ${dependencies}
)

# Build BT plugin library for path planning node
add_library(navbim_path_planning_bt_nodes SHARED
  src/plugins/action/compute_path_to_pose.cpp
  src/plugins/plugin_registration.cpp
)
target_include_directories(navbim_path_planning_bt_nodes
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(navbim_path_planning_bt_nodes
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)
pluginlib_export_plugin_description_file(behaviortree_cpp navbim_path_planning_bt_nodes.xml)
rclcpp_components_register_nodes(${library_name} "navbim_bt_navigator::BtNavigator")

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name} navbim_navigate_to_pose_navigator navbim_path_planning_bt_nodes
  EXPORT export_${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(geometry_msgs nav2_core nav2_msgs nav2_util nav_msgs pluginlib rclcpp rclcpp_action)
ament_export_targets(export_${library_name})
ament_package()
