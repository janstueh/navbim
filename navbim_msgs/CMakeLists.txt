cmake_minimum_required(VERSION 3.8)
project(navbim_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavbimComputePathToPose.action"
  "action/NavbimComputePathToPoseInRoom.action"
  "action/PrePlanEdges.action"
  "msg/BoundingBox.msg"
  "msg/CurrentRoom.msg"
  "msg/Environment.msg"
  "msg/Environments.msg"
  "msg/Polygon.msg"
  "msg/Topomap.msg"
  "msg/TopomapNode.msg"
  "msg/TopomapNodes.msg"
  "msg/TopomapEdge.msg"
  "msg/TopomapEdges.msg"
  "srv/CalculatePathClearance.srv"
  "srv/ClearTopologicalMapPaths.srv"
  "srv/DumpMultiMap.srv"
  "srv/GetAdjacentTransitionNodes.srv"
  "srv/GetElementsByType.srv"
  "srv/GetFloorNodes.srv"
  "srv/GetIfcElementInfo.srv"
  "srv/GetMinZ.srv"
  "srv/GetRoomByCoordinates.srv"
  "srv/GetRoomNeighbors.srv"
  "srv/GetRoomNodes.srv"
  "srv/GetTopologicalMap.srv"
  "srv/IsPointInRoomPolygon.srv"
  "srv/LoadEnvironments.srv"
  "srv/LoadMultiMap.srv"
  "srv/LoadTopomap.srv"
  "srv/SaveTopologicalMap.srv"
  "srv/SmoothPathWithFloorCostmaps.srv"
  "srv/UpdateEdgeData.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs visualization_msgs
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

ament_export_dependencies(rosidl_default_runtime)

ament_package()