// Copyright (c) 2025 NavBIM Contributors
// Licensed under the Apache License, Version 2.0

#ifndef NAVBIM_GPP_BIM__GPP_BIM_SERVER_HPP_
#define NAVBIM_GPP_BIM__GPP_BIM_SERVER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "navbim_msgs/action/navbim_compute_path_to_pose.hpp"
#include "navbim_msgs/action/navbim_compute_path_to_pose_in_room.hpp"
#include "navbim_msgs/action/pre_plan_edges.hpp"
#include "navbim_msgs/srv/get_topological_map.hpp"
#include "navbim_msgs/srv/get_room_by_coordinates.hpp"
#include "navbim_msgs/srv/update_edge_data.hpp"
#include "navbim_msgs/srv/save_topological_map.hpp"
#include "navbim_msgs/srv/smooth_path_with_floor_costmaps.hpp"
#include "navbim_msgs/srv/calculate_path_clearance.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "navbim_util/topological_map.hpp"
#include "navbim_gpp_bim/graph_utils.hpp"
#include "navbim_gpp_bim/topomap_planner.hpp"

namespace navbim_gpp_bim
{

/**
 * @brief Global Path Planner for BIM environments
 * 
 * This lifecycle node provides topological map-based global path planning
 * using a two-level approach:
 * 1. First level: A* search on topological graph
 * 2. Second level: Detailed grid-based planning through rooms/spaces
 */
class GppBimServer : public nav2_util::LifecycleNode
{
public:
  using ComputePathToPose = navbim_msgs::action::NavbimComputePathToPose;
  using ComputePathToPoseInRoom = navbim_msgs::action::NavbimComputePathToPoseInRoom;
  using PrePlanEdges = navbim_msgs::action::PrePlanEdges;
  using ActionServer = nav2_util::SimpleActionServer<ComputePathToPose>;
  using ActionClient = rclcpp_action::Client<ComputePathToPoseInRoom>;
  using PrePlanActionServer = rclcpp_action::Server<PrePlanEdges>;
  using GetTopologicalMap = navbim_msgs::srv::GetTopologicalMap;
  using GetRoomByCoordinates = navbim_msgs::srv::GetRoomByCoordinates;
  using UpdateEdgeData = navbim_msgs::srv::UpdateEdgeData;
  using SaveTopologicalMap = navbim_msgs::srv::SaveTopologicalMap;

  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit GppBimServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~GppBimServer();



protected:
  /**
   * @brief Configure lifecycle node - load parameters, create services/clients
   * @param state Current lifecycle state
   * @return Transition result
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate lifecycle node - start action server, create bonds
   * @param state Current lifecycle state
   * @return Transition result
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate lifecycle node - stop action server, destroy bonds
   * @param state Current lifecycle state
   * @return Transition result
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup lifecycle node - destroy services/clients
   * @param state Current lifecycle state
   * @return Transition result
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown lifecycle node
   * @param state Current lifecycle state
   * @return Transition result
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Main action callback for path planning
   */
  void computePathToPose();

  /**
   * @brief Action callback for pre-planning all transition edges
   * @param uuid Goal UUID
   * @param goal Goal message with force_replan flag
   * @return Goal response (accept/reject)
   */
  rclcpp_action::GoalResponse handlePrePlanGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PrePlanEdges::Goal> goal);

  /**
   * @brief Handle cancel request for pre-planning action
   * @param goal_handle Goal handle
   * @return Cancel response
   */
  rclcpp_action::CancelResponse handlePrePlanCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PrePlanEdges>> goal_handle);

  /**
   * @brief Execute pre-planning action
   * @param goal_handle Goal handle for publishing feedback and result
   */
  void executePrePlan(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PrePlanEdges>> goal_handle);

  /**
   * @brief Implementation of pre-planning action (runs in separate thread)
   * @param goal_handle Goal handle for publishing feedback and result
   */
  /**
   * @brief Execute pre-planning implementation (called directly or via action)
   * @param result Result object to populate
   * @param force_replan Whether to replan all edges or only unplanned ones
   * @return True if all edges planned successfully
   */
  bool executePrePlanImpl(
    std::shared_ptr<PrePlanEdges::Result> result,
    bool force_replan);

  /**
   * @brief Load topological map from topomap_server via service
   * @return True if successful
   */
  bool loadTopologicalMap();

  /**
   * @brief Second-level planner callback for detailed room planning
   * @param floor_name Name of the floor
   * @param room_name Name of the room
   * @param start_coords Start coordinates [x, y, z]
   * @param end_coords End coordinates [x, y, z]
   * @param is_stair Whether this is a stair connection
   * @return Tuple of (optional path, distance, cost)
   *         Path is vector of 3D coordinates or nullopt if planning failed
   *         Distance is Euclidean path length in meters
   *         Cost is total accumulated costmap cost (sum at each pose)
   */
  std::tuple<std::optional<std::vector<std::array<double, 3>>>, double, double> secondLevelPlanner(
    const std::string & floor_name,
    const std::string & room_name,
    const std::array<double, 3> & start_coords,
    const std::array<double, 3> & end_coords);

  /**
   * @brief Convert waypoints to ROS Path message
   */
  nav_msgs::msg::Path waypointsToPath(
    const std::vector<std::array<double, 3>> & waypoints,
    const std::string & frame_id = "ifc");

  /**
   * @brief Create PoseStamped from coordinates
   */
  geometry_msgs::msg::PoseStamped createPoseStamped(
    const std::array<double, 3> & coords,
    const std::string & frame_id = "ifc");

  /**
   * @brief Publish pruned topological map as Topomap message
   */
  void publishPrunedTopomap(const navbim_util::TopologicalGraph & pruned_graph);

  /**
   * @brief Find the index of the closest waypoint in a path to given coordinates
   * @param path The path to search
   * @param coords Target coordinates [x, y, z]
   * @return Index of the closest waypoint
   */
  size_t findClosestWaypointIndex(
    const nav_msgs::msg::Path & path,
    const std::array<double, 3> & coords);

  // Parameters
  bool reuse_paths_;
  bool pre_plan_paths_;
  bool force_pre_plan_of_planned_paths_;
  std::string second_level_planner_;
  bool timeout_occurred_;  // Track if timeout occurred during second-level planning
  double resolution_;
  double penalize_z_movement_;
  double robot_height_;
  double robot_width_;
  double robot_length_;
  double robot_step_height_;
  bool use_two_level_;
  bool prune_graph_;
  bool visualize_pruned_graph_;
  double pruning_cost_threshold_;  // Skip pruning if estimated cost < threshold
  std::string topomap_file_;
  std::string nav_model_;

  // Topological map
  navbim_util::TopologicalMap topological_map_;
  
  // Mutex to protect topological map during modifications
  std::mutex topological_map_mutex_;
  
  // Flag to track if topological map has been loaded
  std::atomic<bool> topomap_loaded_{false};
  
  // Timer for delayed topological map loading (non-blocking activation)
  rclcpp::TimerBase::SharedPtr topomap_load_timer_;

  // Action server for path planning
  std::unique_ptr<ActionServer> action_server_;

  // Action server for pre-planning edges
  PrePlanActionServer::SharedPtr pre_plan_action_server_;

  // Timer for delayed pre-planning trigger
  rclcpp::TimerBase::SharedPtr pre_plan_trigger_timer_;

  // Action client for room-based planning
  ActionClient::SharedPtr room_planner_client_;

  // Service clients
  using LCNode = nav2_util::LifecycleNode::SharedPtr;
  std::shared_ptr<nav2_util::ServiceClient<GetTopologicalMap, LCNode>> get_topomap_client_;
  std::shared_ptr<nav2_util::ServiceClient<GetRoomByCoordinates, LCNode>> get_room_by_coords_client_;
  std::shared_ptr<nav2_util::ServiceClient<UpdateEdgeData, LCNode>> update_edge_data_client_;
  std::shared_ptr<nav2_util::ServiceClient<SaveTopologicalMap, LCNode>> save_topomap_client_;
  std::shared_ptr<nav2_util::ServiceClient<navbim_msgs::srv::SmoothPathWithFloorCostmaps, LCNode>> floor_smoothing_client_;

  // Parameters for floor smoothing
  bool enable_floor_smoothing_;
  double floor_smoother_max_time_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<navbim_msgs::msg::Topomap>::SharedPtr
    pruned_topomap_pub_;

  // Timings for performance tracking
  struct PlanningTimings
  {
    double total = 0.0;
    double prune_graph = 0.0;        // Includes: prune, visualization, find start/goal vertices
    double topomap_astar = 0.0;
    double second_level_total = 0.0;
    int second_level_count = 0;
    double floor_smoothing = 0.0;           // Floor-level path smoothing time
  };
};

}  // namespace navbim_gpp_bim

#endif  // NAVBIM_GPP_BIM__GPP_BIM_SERVER_HPP_
