// Copyright (c) 2025 NavBIM Contributors
// Licensed under the Apache License, Version 2.0

#ifndef NAVBIM_GPP_BIM__TOPOMAP_PLANNER_HPP_
#define NAVBIM_GPP_BIM__TOPOMAP_PLANNER_HPP_

#include <vector>
#include <tuple>
#include <optional>
#include <functional>

#include "navbim_util/topological_map_types.hpp"
#include "rclcpp/logger.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/service_client.hpp"
#include "navbim_msgs/srv/update_edge_data.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace navbim_gpp_bim
{

/**
 * @brief Result structure for topological map path search
 */
struct TopomapPlannerResult
{
  std::vector<navbim_util::Vertex> coarse_path;  ///< Sequence of vertices in topological map
  std::vector<std::array<double, 3>> detailed_path;  ///< Full path with coordinates
  double total_distance;  ///< Total Euclidean distance
  double total_cost;  ///< Total cost (with penalties)
  bool success;  ///< Whether path was found
};

/**
 * @brief Timing information for profiling
 */
struct TopomapPlannerTimings
{
  double topomap_astar_time = 0.0;  ///< Time spent in A* algorithm
  double second_level_total_time = 0.0;  ///< Total time in second-level planner
  int second_level_count = 0;  ///< Number of second-level calls
};

/**
 * @brief Type for second-level planner callback
 * 
 * Parameters: floor_name, room_name, start_coords, end_coords
 * Returns: Tuple of (optional path, distance in meters, total cost)
 *          Path is vector of 3D coordinates, or nullopt if planning failed
 *          Distance is Euclidean path length
 *          Cost is total accumulated costmap cost (sum of costs at each pose)
 */
using SecondLevelPlannerCallback = std::function<
  std::tuple<std::optional<std::vector<std::array<double, 3>>>, double, double>(
    const std::string& floor_name,
    const std::string& room_name,
    const std::array<double, 3>& start_coords,
    const std::array<double, 3>& end_coords
  )
>;

/**
 * @brief Plan path through topological graph with second-level planning
 * 
 * This implements a two-level path planning approach:
 * 1. First level (A* on topological graph) finds high-level path through rooms/floors
 * 2. Second level (called via callback) plans detailed paths through each space
 * 
 * The algorithm uses a Fibonacci heap for the open set to support efficient
 * decrease-key operations when updating f-scores.
 * 
 * @param graph Topological graph containing rooms, transitions, etc.
 * @param start_vertex Vertex descriptor for start node
 * @param goal_vertex Vertex descriptor for goal node
 * @param second_level_planner Callback for detailed path planning in each space
 * @param update_edge_client Optional nav2 service client to update edge data (always saves)
 * @param resolution Grid resolution for coordinate alignment (default 0.05m)
 * @param penalize_z_movement Penalty factor for vertical movement (default 1.0)
 * @param reuse_paths Whether to reuse previously planned paths (default true)
 * @param logger ROS logger for debug/error messages
 * @param timings Optional output for timing information
 * @return TopomapPlannerResult containing path and metrics
 */
TopomapPlannerResult planTopomapPath(
  navbim_util::TopologicalGraph& graph,
  navbim_util::Vertex start_vertex,
  navbim_util::Vertex goal_vertex,
  SecondLevelPlannerCallback second_level_planner,
  const std::set<navbim_util::Vertex>& disabled_vertices = {},
  std::shared_ptr<nav2_util::ServiceClient<navbim_msgs::srv::UpdateEdgeData, nav2_util::LifecycleNode::SharedPtr>> update_edge_client = nullptr,
  double penalize_z_movement = 1.0,
  bool reuse_paths = true,
  const rclcpp::Logger& logger = rclcpp::get_logger("topomap_planner"),
  TopomapPlannerTimings* timings = nullptr
);

/**
 * @brief Align coordinates to grid resolution
 * 
 * Snaps coordinates to nearest grid cell center. This prevents zigzag artifacts
 * at doors/stairs where path sections are concatenated.
 * 
 * @param coords Input coordinates [x, y, z]
 * @param resolution Grid cell size
 * @return Aligned coordinates
 */
std::array<double, 3> alignCoordsToGrid(
  const std::array<double, 3>& coords,
  double resolution
);

/**
 * @brief Compute total distance and cost of waypoint path
 * 
 * @param waypoints Path as sequence of 3D coordinates
 * @param penalize_z_movement Penalty factor for vertical movement
 * @return Tuple of (total_distance, total_cost)
 */
std::tuple<double, double> computeTotalDistanceAndCost(
  const std::vector<std::array<double, 3>>& waypoints,
  double penalize_z_movement
);

}  // namespace navbim_gpp_bim

#endif  // NAVBIM_GPP_BIM__TOPOMAP_PLANNER_HPP_
