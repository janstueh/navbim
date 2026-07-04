cmake_minimum_required(VERSION 3.8)
project(navbim_multimap_server)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(navbim_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

nav2_package()

set(multimap_server_executable multimap_server)

set(library_name ${multimap_server_executable}_core)

set(map_io_library_name map_io)

set(map_saver_cli_executable map_saver_cli)

set(map_saver_server_executable map_saver_server)

set(costmap_filter_info_server_executable costmap_filter_info_server)

set(dependencies
  lifecycle_msgs
  nav_msgs
  nav2_msgs
  navbim_msgs
  nav2_util
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_srvs
  std_msgs
)

add_library(${map_io_library_name} SHARED
  src/map_mode.cpp
  src/map_io.cpp)
target_include_directories(${map_io_library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  PRIVATE
    ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})
ament_target_dependencies(${map_io_library_name}
  nav_msgs
  nav2_util
)
target_link_libraries(${map_io_library_name}
  ${GRAPHICSMAGICKCPP_LIBRARIES}
  yaml-cpp::yaml-cpp
)

add_library(${library_name} SHARED
  src/multimap_server/multimap_server.cpp
  src/map_saver/map_saver.cpp
  src/costmap_filter_info/costmap_filter_info_server.cpp)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name}
  ${dependencies}
)
target_link_libraries(${library_name}
  ${map_io_library_name}
  yaml-cpp::yaml-cpp
)

add_executable(${multimap_server_executable}
  src/multimap_server/main.cpp)
target_include_directories(${multimap_server_executable}
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${multimap_server_executable}
  ${library_name}
  ${map_io_library_name}
)
ament_target_dependencies(${multimap_server_executable}
  rclcpp
  nav2_util
)

add_executable(${map_saver_cli_executable}
  src/map_saver/main_cli.cpp)
target_include_directories(${map_saver_cli_executable}
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${map_saver_cli_executable}
  ${library_name}
  ${map_io_library_name}
)
ament_target_dependencies(${map_saver_cli_executable}
  nav_msgs
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_lifecycle
)

add_executable(${map_saver_server_executable}
  src/map_saver/main_server.cpp)
target_include_directories(${map_saver_server_executable}
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${map_saver_server_executable}
  ${library_name}
  ${map_io_library_name}
)
ament_target_dependencies(${map_saver_server_executable}
  nav_msgs
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_lifecycle
)

add_executable(${costmap_filter_info_server_executable}
  src/costmap_filter_info/main.cpp)
target_include_directories(${costmap_filter_info_server_executable}
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${costmap_filter_info_server_executable}
  ${library_name}
  ${map_io_library_name}
)
ament_target_dependencies(${costmap_filter_info_server_executable}
  nav_msgs
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_lifecycle
)

rclcpp_components_register_nodes(${library_name} "navbim_multimap_server::CostmapFilterInfoServer")
rclcpp_components_register_nodes(${library_name} "navbim_multimap_server::MapSaver")
rclcpp_components_register_nodes(${library_name} "navbim_multimap_server::MultimapServer")

install(TARGETS
    ${library_name} ${map_io_library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
    ${multimap_server_executable} ${map_saver_cli_executable} ${map_saver_server_executable}
    ${costmap_filter_info_server_executable}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(
  ${library_name}
  ${map_io_library_name}
)
ament_export_dependencies(nav_msgs nav2_msgs nav2_util rclcpp rclcpp_lifecycle std_srvs)
ament_export_targets(${library_name})

ament_package()
