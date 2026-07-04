cmake_minimum_required(VERSION 3.8)
project(navbim_gpp_bim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(backward_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(navbim_multi_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_theta_star_planner REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(navbim_smoother REQUIRED)
find_package(ompl REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navbim_msgs REQUIRED)
find_package(navbim_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

nav2_package()

set(dependencies
  geometry_msgs
  lifecycle_msgs
  nav2_core
  nav2_costmap_2d
  navbim_multi_costmap_2d
  nav2_msgs
  nav2_navfn_planner
  nav2_theta_star_planner
  navbim_smoother
  nav2_util
  nav_msgs
  navbim_msgs
  navbim_util
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  tf2
  tf2_ros
  visualization_msgs
)

# C++ Graph Utils Library
set(graph_utils_library_name graph_utils)

add_library(${graph_utils_library_name} SHARED
  src/graph_utils.cpp
  src/augmented_graph_view.cpp
)
target_include_directories(${graph_utils_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${graph_utils_library_name}
  navbim_util
)
target_link_libraries(${graph_utils_library_name}
  Boost::system
  navbim_util::navbim_util_topological_map
)
# Disable false-positive uninitialized variable warnings in Boost.Geometry buffer algorithm
target_compile_options(${graph_utils_library_name} PRIVATE -Wno-maybe-uninitialized)

# C++ Topomap Planner Library
set(topomap_planner_library_name topomap_planner)

add_library(${topomap_planner_library_name} SHARED
  src/topomap_planner.cpp
)
target_include_directories(${topomap_planner_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${topomap_planner_library_name}
  nav2_util
  navbim_util
  rclcpp
)
target_link_libraries(${topomap_planner_library_name}
  Boost::system
  navbim_util::navbim_util_topological_map
)

# C++ Room Planner Server Component
set(room_planner_executable_name room_planner_server)
set(room_planner_library_name ${room_planner_executable_name}_core)

add_library(${room_planner_library_name} SHARED
  src/room_planner_server.cpp
  src/ompl_planner_wrapper.cpp
)
target_include_directories(${room_planner_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${OMPL_INCLUDE_DIRS}
)
ament_target_dependencies(${room_planner_library_name}
  ${dependencies}
)
target_link_libraries(${room_planner_library_name}
  navbim_multi_costmap_2d::navbim_multi_costmap_2d_core
  navbim_smoother::navbim_simple_smoother
  navbim_smoother::navbim_savitzky_golay_smoother
  ${OMPL_LIBRARIES}
)

add_executable(${room_planner_executable_name}
  src/room_planner_server_main.cpp
)
target_link_libraries(${room_planner_executable_name}
  ${room_planner_library_name}
)

rclcpp_components_register_nodes(${room_planner_library_name} "navbim_gpp_bim::RoomPlannerServer")

# C++ Clearance Server Component
set(clearance_server_executable_name clearance_server)
set(clearance_server_library_name ${clearance_server_executable_name}_core)

add_library(${clearance_server_library_name} SHARED
  src/clearance_server.cpp
)
target_include_directories(${clearance_server_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${clearance_server_library_name}
  geometry_msgs
  nav2_costmap_2d
  navbim_multi_costmap_2d
  navbim_msgs
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
)
target_link_libraries(${clearance_server_library_name}
  nav2_costmap_2d::nav2_costmap_2d_core
  navbim_multi_costmap_2d::navbim_multi_costmap_2d_core
)

add_executable(${clearance_server_executable_name}
  src/clearance_server_main.cpp
)
target_link_libraries(${clearance_server_executable_name}
  ${clearance_server_library_name}
)

rclcpp_components_register_nodes(${clearance_server_library_name} "navbim_gpp_bim::ClearanceServer")

# C++ Gpp Bim Server Component
set(gpp_bim_server_executable_name gpp_bim_server)
set(gpp_bim_server_library_name ${gpp_bim_server_executable_name}_core)

add_library(${gpp_bim_server_library_name} SHARED
  src/gpp_bim_server.cpp
)
target_include_directories(${gpp_bim_server_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${gpp_bim_server_library_name}
  geometry_msgs
  lifecycle_msgs
  nav2_msgs
  navbim_msgs
  navbim_util
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  visualization_msgs
)
target_link_libraries(${gpp_bim_server_library_name}
  ${graph_utils_library_name}
  ${topomap_planner_library_name}
  navbim_util::navbim_util_topological_map
)

add_executable(${gpp_bim_server_executable_name}
  src/gpp_bim_server_main.cpp
)
target_link_libraries(${gpp_bim_server_executable_name}
  ${gpp_bim_server_library_name}
)

rclcpp_components_register_nodes(${gpp_bim_server_library_name} "navbim_gpp_bim::GppBimServer")

# Install C++ targets
install(TARGETS ${graph_utils_library_name}
  EXPORT ${graph_utils_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${topomap_planner_library_name}
  EXPORT ${topomap_planner_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${room_planner_library_name}
  EXPORT ${room_planner_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${room_planner_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${clearance_server_library_name}
  EXPORT ${clearance_server_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${clearance_server_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${gpp_bim_server_library_name}
  EXPORT ${gpp_bim_server_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${gpp_bim_server_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${graph_utils_library_name}
  ${topomap_planner_library_name}
  ${room_planner_library_name}
  ${clearance_server_library_name}
  ${gpp_bim_server_library_name}
)
ament_export_dependencies(
  Boost
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  navbim_multi_costmap_2d
  nav2_msgs
  navbim_msgs
  navbim_util
  nav2_navfn_planner
  nav2_theta_star_planner
  navbim_smoother
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  visualization_msgs
)
ament_export_targets(
  ${graph_utils_library_name}
  ${topomap_planner_library_name}
  ${room_planner_library_name}
  ${clearance_server_library_name}
  ${gpp_bim_server_library_name}
)
ament_package()
