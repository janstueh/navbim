cmake_minimum_required(VERSION 3.8)
project(navbim_util)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navbim_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  geometry_msgs
  nav_msgs
  navbim_msgs
  rclcpp
  visualization_msgs
)

# C++ Topological Map Library
set(topological_map_library_name ${PROJECT_NAME}_topological_map)

add_library(${topological_map_library_name} SHARED
  src/topological_map.cpp
)

target_include_directories(${topological_map_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${topological_map_library_name}
  ${dependencies}
)

target_link_libraries(${topological_map_library_name}
  Boost::system
)

# Install C++ libraries
install(TARGETS ${topological_map_library_name}
  EXPORT navbim_util_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts (rename to remove .py extension)
install(PROGRAMS
  ${PROJECT_NAME}/pose_to_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_to_tf_publisher
)

# Export
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${topological_map_library_name})
ament_export_dependencies(
  Boost
  geometry_msgs
  nav_msgs
  navbim_msgs
  rclcpp
  visualization_msgs
)
ament_export_targets(navbim_util_targets)

ament_package()
