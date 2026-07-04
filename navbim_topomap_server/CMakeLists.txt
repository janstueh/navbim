cmake_minimum_required(VERSION 3.8)
project(navbim_topomap_server)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navbim_msgs REQUIRED)
find_package(navbim_util REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system)

nav2_package()

set(executable_name topomap_server)
set(library_name ${executable_name}_core)
set(loader_library_name topological_map_loader)
set(utils_library_name topological_map_utils)

# Library: topological_map_loader
add_library(${loader_library_name} SHARED
  src/topological_map_loader.cpp
)
target_include_directories(${loader_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${loader_library_name}
  navbim_util
)
target_link_libraries(${loader_library_name}
  Boost::system
  navbim_util::navbim_util_topological_map
  nlohmann_json::nlohmann_json
)

# Library: topological_map_utils
add_library(${utils_library_name} SHARED
  src/topological_map_utils.cpp
)
target_include_directories(${utils_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# Disable -Werror for this target due to Boost.Geometry internal warnings
target_compile_options(${utils_library_name} PRIVATE
  -Wno-error=maybe-uninitialized
)
ament_target_dependencies(${utils_library_name}
  navbim_util
)
target_link_libraries(${utils_library_name}
  ${loader_library_name}
  Boost::system
  navbim_util::navbim_util_topological_map
)

# Library: topomap_server_core (component library)
add_library(${library_name} SHARED
  src/topomap_server.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name}
  geometry_msgs
  interactive_markers
  lifecycle_msgs
  navbim_msgs
  nav2_util
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
)
target_link_libraries(${library_name}
  ${loader_library_name}
  ${utils_library_name}
  interactive_markers::interactive_markers
)

# Register component
rclcpp_components_register_nodes(${library_name} "navbim_topomap_server::TopomapServer")

# Executable: topomap_server (standalone)
add_executable(${executable_name}
  src/topomap_server_main.cpp
)
target_link_libraries(${executable_name}
  ${library_name}
)
ament_target_dependencies(${executable_name}
  rclcpp
)

# Install libraries
install(TARGETS
  ${loader_library_name}
  ${utils_library_name}
  ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export information
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${loader_library_name} ${utils_library_name} ${library_name})
ament_export_dependencies(
  geometry_msgs
  interactive_markers
  navbim_msgs
  rclcpp
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
)
ament_export_targets(${library_name})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
