cmake_minimum_required(VERSION 3.8)
project(navbim_room_tracker)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navbim_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(library_name room_tracker_core)
set(executable_name room_tracker)

set(dependencies
  geometry_msgs
  navbim_msgs
  nav2_util
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

# Library: room_tracker_core (component library)
add_library(${library_name} SHARED
  src/room_tracker_node.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

# Register component
rclcpp_components_register_nodes(${library_name} "navbim_room_tracker::RoomTracker")

# Executable: room_tracker (standalone)
add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(${executable_name} ${library_name})
ament_target_dependencies(${executable_name}
  ${dependencies}
)

install(TARGETS ${library_name}
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_targets(${library_name})
ament_export_dependencies(
  geometry_msgs
  navbim_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

ament_package()
