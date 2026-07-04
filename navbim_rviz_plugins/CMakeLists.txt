cmake_minimum_required(VERSION 3.8)
project(navbim_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navbim_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

nav2_package()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.
set(navbim_rviz_plugins_headers_to_moc
  include/navbim_rviz_plugins/bim_panel.hpp
  include/navbim_rviz_plugins/ifc_element_manager.hpp
  include/navbim_rviz_plugins/topomap_panel.hpp
)

foreach(header "${navbim_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(navbim_rviz_plugins_moc_files "${header}")
endforeach()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(library_name ${PROJECT_NAME})

set(dependencies
  geometry_msgs
  navbim_msgs
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rviz_common
  rviz_default_plugins
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  interactive_markers
)

add_library(${library_name} SHARED
  src/bim_panel.cpp
  src/ifc_element_manager.cpp
  src/topomap_panel.cpp
  ${navbim_rviz_plugins_moc_files}
)
target_include_directories(${library_name}
  PUBLIC
    ${Qt5Widgets_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name}
  ${dependencies}
)
target_link_libraries(${library_name}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  interactive_markers::interactive_markers
  yaml-cpp::yaml-cpp
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  navbim_msgs
  lifecycle_msgs
  Qt5
  rclcpp
  rclcpp_action
  rviz_common
  rviz_default_plugins
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  interactive_markers
)

ament_package()
