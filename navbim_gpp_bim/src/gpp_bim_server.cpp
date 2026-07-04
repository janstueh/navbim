// Copyright (c) 2025
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "navbim_gpp_bim/gpp_bim_server.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <filesystem>

#include "nav2_util/node_utils.hpp"
#include "navbim_msgs/srv/save_topological_map.hpp"
#include "navbim_gpp_bim/augmented_graph_view.hpp"
#include "navbim_gpp_bim/floor_segmentation.hpp"

using namespace std::chrono_literals;

namespace navbim_gpp_bim
{

GppBimServer::GppBimServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("gpp_bim", "", options)
{
}

GppBimServer::~GppBimServer()
{}

nav2_util::CallbackReturn
GppBimServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  auto node = shared_from_this();

  // Declare and get parameters
  declare_parameter("reuse_paths", rclcpp::ParameterValue(true));
  declare_parameter("pre_plan_paths", rclcpp::ParameterValue(false));
  declare_parameter("force_pre_plan_of_planned_paths", rclcpp::ParameterValue(false));
  declare_parameter("second_level_planner", rclcpp::ParameterValue("A*"));
  declare_parameter("resolution", rclcpp::ParameterValue(0.05));
  declare_parameter("penalize_z_movement", rclcpp::ParameterValue(1.0));
  declare_parameter("prune_graph", rclcpp::ParameterValue(true));
  declare_parameter("visualize_pruned_graph", rclcpp::ParameterValue(true));
  declare_parameter("pruning_cost_threshold", rclcpp::ParameterValue(10.0));

  declare_parameter("robot_height", rclcpp::ParameterValue(0.75));
  declare_parameter("robot_width", rclcpp::ParameterValue(0.4));
  declare_parameter("robot_length", rclcpp::ParameterValue(0.5));
  declare_parameter("robot_step_height", rclcpp::ParameterValue(0.0));

  declare_parameter("use_two_level", rclcpp::ParameterValue(true));
  declare_parameter("topomap_file", rclcpp::ParameterValue(std::string("")));
  declare_parameter("nav_model", rclcpp::ParameterValue(std::string("")));
  
  // Floor-wide smoothing parameters
  declare_parameter("enable_floor_smoothing", rclcpp::ParameterValue(false));
  declare_parameter("floor_smoother_max_time", rclcpp::ParameterValue(1.0));

  // Get parameter values
  get_parameter("reuse_paths", reuse_paths_);
  get_parameter("pre_plan_paths", pre_plan_paths_);
  get_parameter("force_pre_plan_of_planned_paths", force_pre_plan_of_planned_paths_);
  get_parameter("second_level_planner", second_level_planner_);
  timeout_occurred_ = false;  // Initialize timeout flag
  get_parameter("resolution", resolution_);
  get_parameter("penalize_z_movement", penalize_z_movement_);
  get_parameter("prune_graph", prune_graph_);
  get_parameter("visualize_pruned_graph", visualize_pruned_graph_);
  get_parameter("pruning_cost_threshold", pruning_cost_threshold_);

  get_parameter("robot_height", robot_height_);
  get_parameter("robot_width", robot_width_);
  get_parameter("robot_length", robot_length_);
  get_parameter("robot_step_height", robot_step_height_);

  get_parameter("use_two_level", use_two_level_);
  get_parameter("topomap_file", topomap_file_);
  get_parameter("nav_model", nav_model_);
  
  // Floor-wide smoothing parameters
  get_parameter("enable_floor_smoothing", enable_floor_smoothing_);
  get_parameter("floor_smoother_max_time", floor_smoother_max_time_);

  RCLCPP_INFO(
    get_logger(), "Configured: planner=%s, use_two_level=%s",
    second_level_planner_.c_str(),
    use_two_level_ ? "true" : "false");

  // Create action server
  action_server_ = std::make_unique<ActionServer>(
    node,
    "navbim_compute_path_to_pose",
    std::bind(&GppBimServer::computePathToPose, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);  // Enable result awareness

  // Create pre-plan action server with reentrant callback group
  auto pre_plan_callback_group = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  
  pre_plan_action_server_ = rclcpp_action::create_server<PrePlanEdges>(
    node,
    "pre_plan_edges",
    std::bind(&GppBimServer::handlePrePlanGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GppBimServer::handlePrePlanCancel, this, std::placeholders::_1),
    std::bind(&GppBimServer::executePrePlan, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    pre_plan_callback_group);

  // Create action client for room-based planning
  room_planner_client_ = rclcpp_action::create_client<ComputePathToPoseInRoom>(
    node,
    "compute_path_to_pose_in_room");

  // Create service clients with internal executor for blocking calls
  auto lc_node = std::static_pointer_cast<nav2_util::LifecycleNode>(shared_from_this());
  get_topomap_client_ = std::make_shared<nav2_util::ServiceClient<GetTopologicalMap, LCNode>>(
    "topomap_server/get_topological_map", lc_node);

  get_room_by_coords_client_ = std::make_shared<nav2_util::ServiceClient<GetRoomByCoordinates, LCNode>>(
    "topomap_server/get_room_by_coordinates", lc_node);

  update_edge_data_client_ = std::make_shared<nav2_util::ServiceClient<UpdateEdgeData, LCNode>>(
    "topomap_server/update_edge_data", lc_node);

  save_topomap_client_ = std::make_shared<nav2_util::ServiceClient<SaveTopologicalMap, LCNode>>(
    "topomap_server/save_topological_map", lc_node);

  // Create service client for floor smoothing
  if (enable_floor_smoothing_) {
    floor_smoothing_client_ = std::make_shared<
      nav2_util::ServiceClient<navbim_msgs::srv::SmoothPathWithFloorCostmaps, LCNode>>(
      "/smooth_path_with_floor_costmaps", lc_node);
    RCLCPP_INFO(get_logger(), "Floor smoothing enabled (max_time: %.2f s)", 
                floor_smoother_max_time_);
  } else {
    RCLCPP_INFO(get_logger(), "Floor smoothing disabled");
  }

  // Publishers
  path_publisher_ = create_publisher<nav_msgs::msg::Path>("planned_path", 10);

  pruned_topomap_pub_ =
    create_publisher<navbim_msgs::msg::Topomap>(
    "~/pruned_topomap", rclcpp::QoS(1).transient_local());

  // Topological map will be loaded from topomap_server during activation

  RCLCPP_INFO(get_logger(), "Configured successfully");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
GppBimServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activate publishers
  path_publisher_->on_activate();
  pruned_topomap_pub_->on_activate();

  // Activate action server
  action_server_->activate();

  // Load topological map asynchronously to avoid blocking activation
  // Use a one-shot timer that fires immediately after activation completes
  topomap_load_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),  // Load shortly after activation
    [this]() {
      topomap_load_timer_->cancel();  // One-shot timer
      
      RCLCPP_DEBUG(get_logger(), "Loading topological map from topomap_server (non-blocking)");
      if (loadTopologicalMap()) {
        topomap_loaded_ = true;
        RCLCPP_INFO(get_logger(), "Topological map loaded successfully");
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to load topological map");
      }
    });

  // Schedule auto-trigger of pre-planning if enabled
  if (pre_plan_paths_) {
    RCLCPP_INFO(get_logger(), "Scheduling edge pre-planning in 15 seconds...");
    
    pre_plan_trigger_timer_ = create_wall_timer(
      std::chrono::seconds(15),
      [this]() {
        RCLCPP_INFO(get_logger(), "Triggering edge pre-planning now...");
        
        // Cancel timer (one-shot)
        pre_plan_trigger_timer_->cancel();
        
        // Directly call pre-planning logic in a separate thread
        // No need to go through action interface for internal trigger
        std::thread([this]() {
          auto result = std::make_shared<PrePlanEdges::Result>();
          bool success = executePrePlanImpl(result, force_pre_plan_of_planned_paths_);
          
          if (success) {
            RCLCPP_INFO(get_logger(), "Auto pre-planning completed successfully: %s", 
                       result->message.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "Auto pre-planning completed with issues: %s", 
                       result->message.c_str());
          }
          
        }).detach();
      });
  }

  // Create bond to lifecycle manager
  createBond();

  RCLCPP_INFO(get_logger(), "Activated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
GppBimServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Destroy bond
  destroyBond();

  // Deactivate action server
  action_server_->deactivate();

  // Deactivate publishers
  path_publisher_->on_deactivate();
  pruned_topomap_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
GppBimServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Clean up action server
  action_server_.reset();

  // Clean up action client
  room_planner_client_.reset();

  // Clean up timer
  if (pre_plan_trigger_timer_) {
    pre_plan_trigger_timer_->cancel();
    pre_plan_trigger_timer_.reset();
  }

  // Clean up service clients
  get_topomap_client_.reset();
  get_room_by_coords_client_.reset();
  update_edge_data_client_.reset();
  save_topomap_client_.reset();
  floor_smoothing_client_.reset();

  // Clean up publishers
  path_publisher_.reset();
  pruned_topomap_pub_.reset();

  // Clear topological map
  topological_map_.clear();

  RCLCPP_INFO(get_logger(), "Cleaned up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
GppBimServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool GppBimServer::loadTopologicalMap()
{
  RCLCPP_INFO(get_logger(), "Loading topological map from topomap_server");

  // Wait for service to be available
  if (!get_topomap_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(
      get_logger(),
      "GetTopologicalMap service not available after waiting");
    return false;
  }

  // Create request
  auto request = std::make_shared<GetTopologicalMap::Request>();

  // Call service - invoke() handles spinning internally without conflicts
  try {
    auto response = get_topomap_client_->invoke(request, 5s);
    
    if (!response) {
      RCLCPP_ERROR(get_logger(), "Failed to call GetTopologicalMap service");
      return false;
    }

    // Load topological map from message
    if (!topological_map_.loadFromMessage(response->topomap)) {
      RCLCPP_ERROR(get_logger(), "Failed to load topological map from message");
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception calling GetTopologicalMap service: %s", e.what());
    return false;
  }

  RCLCPP_INFO(
    get_logger(), "Topological map loaded successfully: %zu vertices, %zu edges",
    topological_map_.numVertices(), topological_map_.numEdges());

  return true;
}

void GppBimServer::computePathToPose()
{
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // Wait for topological map to be loaded (non-blocking activation)
  if (!topomap_loaded_) {
    RCLCPP_WARN(get_logger(), "Topological map not yet loaded, waiting...");
    // Wait up to 5 seconds for map to load
    auto wait_start = std::chrono::steady_clock::now();
    while (!topomap_loaded_) {
      auto elapsed = std::chrono::steady_clock::now() - wait_start;
      if (elapsed > std::chrono::seconds(5)) {
        RCLCPP_ERROR(get_logger(), "Timeout waiting for topological map to load");
        action_server_->terminate_current();
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  
  // Reset timeout flag at start of each planning request
  timeout_occurred_ = false;

  // Publish empty pruned topomap to clear previous visualization
  navbim_msgs::msg::Topomap empty_topomap;
  pruned_topomap_pub_->publish(empty_topomap);

  // Get goal
  auto goal = action_server_->get_current_goal();

  // Create result
  auto result = std::make_shared<ComputePathToPose::Result>();
  result->error_code = ComputePathToPose::Result::UNKNOWN;
  result->error_msg = "";
  result->path = nav_msgs::msg::Path();
  result->node_ids.clear();
  result->min_clearance = -1.0;
  result->avg_clearance = -1.0;

  // Initialize timings
  PlanningTimings timings;

  try {
    // Check if we should use start pose from goal
    if (!goal->use_start) {
      RCLCPP_ERROR(
        get_logger(),
        "use_start=False not supported. Please provide start pose in goal.");
      result->error_code = ComputePathToPose::Result::UNKNOWN;
      result->error_msg = "use_start=False not supported";
      action_server_->terminate_current(result);
      return;
    }

    // Extract start and goal coordinates
    std::array<double, 3> start_coords = {
      std::round(goal->start.pose.position.x * 1000.0) / 1000.0,
      std::round(goal->start.pose.position.y * 1000.0) / 1000.0,
      std::round(goal->start.pose.position.z * 1000.0) / 1000.0
    };

    std::array<double, 3> goal_coords = {
      std::round(goal->goal.pose.position.x * 1000.0) / 1000.0,
      std::round(goal->goal.pose.position.y * 1000.0) / 1000.0,
      std::round(goal->goal.pose.position.z * 1000.0) / 1000.0
    };

    RCLCPP_DEBUG(
      get_logger(), "Planning path from [%.3f, %.3f, %.3f] to [%.3f, %.3f, %.3f]",
      start_coords[0], start_coords[1], start_coords[2],
      goal_coords[0], goal_coords[1], goal_coords[2]);

    // Check for cancellation
    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(get_logger(), "Goal was canceled");
      result->error_code = ComputePathToPose::Result::UNKNOWN;
      result->error_msg = "Goal was canceled";
      action_server_->terminate_current(result);
      return;
    }

    // Find start and goal rooms using local lookup (fast, no service calls)
    // Find start room by coordinates
    auto start_vertex_opt = findRoomByCoordinates(
      topological_map_.getGraph(),
      start_coords[0], start_coords[1], start_coords[2]);

    if (!start_vertex_opt.has_value()) {
      RCLCPP_ERROR(
        get_logger(), 
        "Failed to find start room for coordinates [%.3f, %.3f, %.3f]",
        start_coords[0], start_coords[1], start_coords[2]);
      result->error_code = ComputePathToPose::Result::NO_VALID_PATH;
      result->error_msg = "Start coordinates not in any room";
      action_server_->terminate_current(result);
      return;
    }

    // Find goal room by coordinates
    auto goal_vertex_opt = findRoomByCoordinates(
      topological_map_.getGraph(),
      goal_coords[0], goal_coords[1], goal_coords[2]);

    if (!goal_vertex_opt.has_value()) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to find goal room for coordinates [%.3f, %.3f, %.3f]",
        goal_coords[0], goal_coords[1], goal_coords[2]);
      result->error_code = ComputePathToPose::Result::NO_VALID_PATH;
      result->error_msg = "Goal coordinates not in any room";
      action_server_->terminate_current(result);
      return;
    }

    // Get room information for logging
    const auto & start_room_node = topological_map_.getGraph()[*start_vertex_opt];
    const auto & goal_room_node = topological_map_.getGraph()[*goal_vertex_opt];
    std::string start_room = start_room_node.name;
    std::string start_floor = (start_room_node.floor.has_value() && 
                                     !start_room_node.floor->empty()) 
                                     ? start_room_node.floor->at(0) : "";
    std::string goal_room = goal_room_node.name;
    std::string goal_floor = (goal_room_node.floor.has_value() && 
                                    !goal_room_node.floor->empty()) 
                                    ? goal_room_node.floor->at(0) : "";
    
    RCLCPP_DEBUG(
      get_logger(), 
      "Found start room '%s' (id=%d) and goal room '%s' (id=%d)",
      start_room_node.name.c_str(), start_room_node.id,
      goal_room_node.name.c_str(), goal_room_node.id);

    // Lock the topological map for the duration of planning
    // This prevents concurrent modifications and allows safe in-place graph changes
    std::lock_guard<std::mutex> lock(topological_map_mutex_);

    // Get reference to the base graph (NO COPY - we modify in-place!)
    navbim_util::TopologicalGraph& base_graph = topological_map_.getGraph();
    
    // Create virtual nodes metadata (zero copy!)
    auto virtual_nodes = createVirtualNodes(
      topological_map_,
      start_vertex_opt.value(),
      goal_vertex_opt.value(),
      start_coords,
      goal_coords,
      penalize_z_movement_);
    
    // Use RAII wrapper to ensure START/GOAL vertices are ALWAYS cleaned up
    // even if exceptions occur during planning
    ScopedGraphMaterialization scoped_graph(base_graph, virtual_nodes);
    const auto& materialized = scoped_graph.nodes();

    // Plan path using topomap planner
    if (use_two_level_) {

      // Compute estimated cost between start and goal using costBetweenNodes
      double estimated_cost = navbim_util::costBetweenNodes(
        base_graph, 
        materialized.start_vertex, 
        materialized.goal_vertex, 
        penalize_z_movement_);
      
      RCLCPP_DEBUG(get_logger(), "Estimated cost between start and goal: %.2f (threshold: %.2f)", 
                   estimated_cost, pruning_cost_threshold_);

      // Build disabled vertices set
      // ALWAYS disable floor and room nodes (we only plan between transition nodes)
      auto t0 = std::chrono::high_resolution_clock::now();
      std::set<navbim_util::Vertex> disabled_vertices;
      
      // Add all floor and room vertices to disabled set
      auto vertices = boost::vertices(base_graph);
      for (auto it = vertices.first; it != vertices.second; ++it) {
        const auto& node_type = base_graph[*it].type;
        if (node_type == "floor" || node_type == "room" || 
            node_type == "stair" || node_type == "ramp") {
          disabled_vertices.insert(*it);
        }
      }
      
      RCLCPP_DEBUG(get_logger(), "Disabled %zu floor/room/stair/ramp nodes", disabled_vertices.size());

      // Optionally add pruned vertices (additional filtering beyond floor/room)
      // Skip pruning if estimated cost is below threshold (nearby goals don't need expensive pruning)
      if (prune_graph_ && estimated_cost >= pruning_cost_threshold_) {
        // Get additional vertices to disable from pruning logic
        auto pruned_vertices = pruneGraph(
          base_graph,
          topological_map_.getFloorVertices(),
          start_vertex_opt,
          goal_vertex_opt);
        
        // Add pruned vertices to disabled set (union)
        size_t before_size = disabled_vertices.size();
        disabled_vertices.insert(pruned_vertices.begin(), pruned_vertices.end());
        size_t added = disabled_vertices.size() - before_size;
        
        RCLCPP_DEBUG(get_logger(), "Graph pruning enabled: added %zu pruned vertices", added);
      } else if (prune_graph_) {
        RCLCPP_DEBUG(get_logger(), 
                     "Graph pruning SKIPPED: estimated cost %.2f < threshold %.2f (nearby goal)",
                     estimated_cost, pruning_cost_threshold_);
      } else {
        RCLCPP_DEBUG(get_logger(), "Graph pruning disabled (using only floor/room filtering)");
      }
      
      RCLCPP_DEBUG(get_logger(), "Total disabled vertices: %zu", disabled_vertices.size());

      // Create pruned graph for visualization only (if enabled)
      if (visualize_pruned_graph_) {
        navbim_util::TopologicalGraph pruned_graph = base_graph;
        std::vector<navbim_util::Vertex> vertices_to_remove(
          disabled_vertices.begin(), disabled_vertices.end());
        std::sort(vertices_to_remove.begin(), vertices_to_remove.end(), 
                  std::greater<navbim_util::Vertex>());
        
        for (const auto & v : vertices_to_remove) {
          boost::clear_vertex(v, pruned_graph);
          boost::remove_vertex(v, pruned_graph);
        }

        // Publish pruned topological map (for visualization)
        publishPrunedTopomap(pruned_graph);
      }

      // Create second-level planner callback
      auto second_level_callback = std::bind(
        &GppBimServer::secondLevelPlanner, this,
        std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4);

      // Find START and GOAL vertices in original graph (not pruned!)
      std::optional<navbim_util::Vertex> start_vertex_in_graph, goal_vertex_in_graph;
      vertices = boost::vertices(base_graph);
      for (auto it = vertices.first; it != vertices.second; ++it) {
        if (base_graph[*it].name == "start") {
          start_vertex_in_graph = *it;
        } else if (base_graph[*it].name == "goal") {
          goal_vertex_in_graph = *it;
        }
        if (start_vertex_in_graph.has_value() && goal_vertex_in_graph.has_value()) {
          break;
        }
      }

      if (!start_vertex_in_graph.has_value() || !goal_vertex_in_graph.has_value()) {
        RCLCPP_ERROR(get_logger(), "START or GOAL vertex not found in graph");
        result->error_code = ComputePathToPose::Result::NO_VALID_PATH;
        result->error_msg = "START or GOAL vertex not found";
        // RAII wrapper will automatically clean up START/GOAL vertices
        action_server_->terminate_current(result);
        return;
      }
      
      auto t1 = std::chrono::high_resolution_clock::now();
      timings.prune_graph = std::chrono::duration<double>(t1 - t0).count();

      // Plan topological path with second-level planning
      // Pass disabled_vertices to the planner (uses filtered graph internally)
      TopomapPlannerTimings planner_timings;
      auto plan_result = planTopomapPath(
        base_graph,  // Original graph (NOT pruned_graph!)
        start_vertex_in_graph.value(),
        goal_vertex_in_graph.value(),
        second_level_callback,
        disabled_vertices,  // Pass disabled vertices for filtering
        update_edge_data_client_,
        penalize_z_movement_,
        reuse_paths_,
        get_logger(),
        &planner_timings);

      // Update timings
      timings.topomap_astar = planner_timings.topomap_astar_time;
      timings.second_level_total = planner_timings.second_level_total_time;
      timings.second_level_count = planner_timings.second_level_count;

      if (!plan_result.success) {
        if (timeout_occurred_) {
          RCLCPP_ERROR(get_logger(), "Topological path planning failed due to timeout");
          result->error_code = ComputePathToPose::Result::TIMEOUT;
          result->error_msg = "Timeout waiting for room planner";
        } else {
          RCLCPP_ERROR(get_logger(), "Topological path planning failed");
          result->error_code = ComputePathToPose::Result::NO_VALID_PATH;
          result->error_msg = "No valid path found";
        }
        // RAII wrapper will automatically clean up START/GOAL vertices
        action_server_->terminate_current(result);
        return;
      }

      // Convert to ROS message
      result->path = waypointsToPath(plan_result.detailed_path);
      
      // Calculate floor segments using shared utility function (used for both smoothing and clearance)
      auto floor_segments_exact = calculateFloorSegments(
        plan_result.coarse_path,
        base_graph,
        result->path);
      
      // Apply floor-level smoothing if enabled
      if (enable_floor_smoothing_) {
        auto floor_smoothing_start = std::chrono::high_resolution_clock::now();
        
        // Apply shift for transition zone smoothing (0.2m radius)
        auto floor_segments_shifted = applySegmentShift(floor_segments_exact, result->path, 0.2);
        
        // Log segment information
        for (size_t seg_idx = 0; seg_idx < floor_segments_exact.size(); ++seg_idx) {
          RCLCPP_DEBUG(get_logger(), "Floor segment %zu: floor='%s', indices [%zu, %zu) → shifted [%zu, %zu)",
                      seg_idx,
                      floor_segments_exact[seg_idx].floor_name.c_str(),
                      floor_segments_exact[seg_idx].start_index,
                      floor_segments_exact[seg_idx].end_index,
                      floor_segments_shifted[seg_idx].start_index,
                      floor_segments_shifted[seg_idx].end_index);
        }
        
        if (!floor_segments_shifted.empty()) {
          // Call floor smoothing service with shifted segments
          auto request = std::make_shared<navbim_msgs::srv::SmoothPathWithFloorCostmaps::Request>();
          request->path = result->path;
          request->max_smoothing_time = floor_smoother_max_time_;
          
          // Extract floor names and segment boundaries with shifting applied
          for (const auto & segment : floor_segments_shifted) {
            request->floor_names.push_back(segment.floor_name);
            request->segment_start_indices.push_back(segment.start_index);
            request->segment_end_indices.push_back(segment.end_index);
          }
          
          // Call floor smoothing service with timeout
          auto response = floor_smoothing_client_->invoke(
            request,
            std::chrono::seconds(10));
          
          if (response && response->success) {
            result->path = response->smoothed_path;
            RCLCPP_DEBUG(get_logger(), "Floor smoothing succeeded");
          } else {
            RCLCPP_WARN(get_logger(), "Floor smoothing failed: %s", 
                       response ? response->message.c_str() : "Service call failed or timed out");
          }
        } else {
          RCLCPP_WARN(get_logger(), "No floor segments found for smoothing");
        }
        
        auto floor_smoothing_end = std::chrono::high_resolution_clock::now();
        timings.floor_smoothing = std::chrono::duration<double>(floor_smoothing_end - floor_smoothing_start).count();
      }
      
      // Extract node IDs from coarse path (using original graph)
      for (const auto & vertex : plan_result.coarse_path) {
        result->node_ids.push_back(std::to_string(base_graph[vertex].id));
      }
      
      // Populate floor segments for clearance calculation
      // Reuse the exact floor segments calculated earlier for smoothing
      if (!floor_segments_exact.empty()) {
        for (const auto & segment : floor_segments_exact) {
          result->floor_names.push_back(segment.floor_name);
          result->segment_start_indices.push_back(segment.start_index);
          result->segment_end_indices.push_back(segment.end_index);
        }
        RCLCPP_DEBUG(get_logger(), "Populated %zu floor segments in result", floor_segments_exact.size());
      } else {
        RCLCPP_WARN(get_logger(), "No floor segments calculated for clearance");
      }

      // Set planning time
      auto end_time = std::chrono::high_resolution_clock::now();
      timings.total = std::chrono::duration<double>(end_time - start_time).count();

      auto total_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        end_time - start_time);
      result->planning_time.sec = total_ns.count() / 1000000000;
      result->planning_time.nanosec = total_ns.count() % 1000000000;

      // Log planning success with detailed timing
      RCLCPP_INFO(get_logger(), "Path planning successful!");
      RCLCPP_INFO(get_logger(), "  Start: Floor '%s', Room '%s'", 
                  !start_floor.empty() ? start_floor.c_str() : "N/A",
                  start_room.c_str());
      RCLCPP_INFO(get_logger(), "  Goal: Floor '%s', Room '%s'", 
                  !goal_floor.empty() ? goal_floor.c_str() : "N/A",
                  goal_room.c_str());
      RCLCPP_INFO(get_logger(), "  Total time: %.2f ms", timings.total * 1000.0);
      RCLCPP_INFO(get_logger(), "  Prune graph: %.2f ms", timings.prune_graph * 1000.0);
      RCLCPP_INFO(get_logger(), "  Topomap A*: %.2f ms", timings.topomap_astar * 1000.0);
      RCLCPP_INFO(
        get_logger(), "  Second-level total: %.2f ms (%d calls, avg %.2f ms)",
        timings.second_level_total * 1000.0, timings.second_level_count,
        timings.second_level_count > 0 ? (timings.second_level_total /
        timings.second_level_count) * 1000.0 : 0.0);
      if (enable_floor_smoothing_ && timings.floor_smoothing > 0.0) {
        RCLCPP_INFO(get_logger(), "  Floor smoothing: %.2f ms", timings.floor_smoothing * 1000.0);
      }
      RCLCPP_INFO(get_logger(), "  Path distance: %.4f m", plan_result.total_distance);
      RCLCPP_INFO(get_logger(), "  Path cost: %.4f", plan_result.total_cost);
      RCLCPP_INFO(get_logger(), "  Waypoints: %zu", plan_result.detailed_path.size());
      RCLCPP_INFO(get_logger(), "  Nodes: %zu", result->node_ids.size());

      // Publish path
      path_publisher_->publish(result->path);

      result->error_code = ComputePathToPose::Result::NONE;
      
      // RAII wrapper (scoped_graph) will automatically clean up START/GOAL vertices
      // when it goes out of scope here, even if exceptions occur
      // Lock will be released after cleanup
      action_server_->succeeded_current(result);

    } else {
      // One-level planning: plan directly in the floor costmap
      RCLCPP_DEBUG(get_logger(), "Using one-level planning (direct floor-level planning)");
      
      if (start_floor.empty() || goal_floor.empty()) {
        RCLCPP_ERROR(get_logger(), "Start or goal room has no floor information");
        result->error_code = ComputePathToPose::Result::NO_VALID_PATH;
        result->error_msg = "Start or goal room has no floor information";
        action_server_->terminate_current(result);
        return;
      }
      
      RCLCPP_DEBUG(get_logger(), "Start floor: %s, Goal floor: %s", 
                  start_floor.c_str(), goal_floor.c_str());
      
      // Check if start and goal are on the same floor
      if (start_floor != goal_floor) {
        RCLCPP_ERROR(get_logger(), 
                     "One-level planning requires start and goal to be on the same floor. "
                     "Start is on '%s', goal is on '%s'", 
                     start_floor.c_str(), goal_floor.c_str());
        result->error_code = ComputePathToPose::Result::NO_VALID_PATH;
        result->error_msg = "Start and goal are on different floors (one-level planning only supports same-floor navigation)";
        action_server_->terminate_current(result);
        return;
      }
      
      // Plan using secondLevelPlanner with floor as the "room"
      RCLCPP_DEBUG(get_logger(), "Planning path on floor '%s' using %s planner", 
                  start_floor.c_str(), second_level_planner_.c_str());
      
      auto t0_one_level = std::chrono::high_resolution_clock::now();
      auto [path_opt, path_distance, path_cost] = secondLevelPlanner(
        start_floor,     // floor_name
        start_floor,     // room_name (use floor name as room name for floor-level costmap)
        start_coords,
        goal_coords);
      auto t1_one_level = std::chrono::high_resolution_clock::now();

      double planning_time = std::chrono::duration<double>(t1_one_level - t0_one_level).count();

      if (!path_opt.has_value()) {
        RCLCPP_ERROR(get_logger(), "One-level planning on floor '%s' failed: no path found", 
                     start_floor.c_str());
        result->error_code = ComputePathToPose::Result::NO_VALID_PATH;
        result->error_msg = "No valid path found using one-level planning";
        action_server_->terminate_current(result);
        return;
      }
      
      // Convert waypoints to ROS path
      result->path = waypointsToPath(path_opt.value());

      if (result->path.poses.empty()) {
        RCLCPP_ERROR(get_logger(), "One-level planning returned an empty path on floor '%s'",
                     start_floor.c_str());
        result->error_code = ComputePathToPose::Result::NO_VALID_PATH;
        result->error_msg = "One-level planning returned an empty path";
        action_server_->terminate_current(result);
        return;
      }

      // Populate node_ids with START and GOAL nodes (consistent with two-level planning)
      const auto & start_node = base_graph[materialized.start_vertex];
      const auto & goal_node = base_graph[materialized.goal_vertex];
      result->node_ids = {std::to_string(start_node.id), std::to_string(goal_node.id)};
      
      RCLCPP_DEBUG(get_logger(), "One-level planning: populated node_ids with START [%d] and GOAL [%d] nodes",
                   start_node.id, goal_node.id);
      
      // Populate floor segments (single segment for entire path)
      result->floor_names.push_back(start_floor);
      result->segment_start_indices.push_back(0);
      result->segment_end_indices.push_back(result->path.poses.size() - 1);  // Inclusive end index
      RCLCPP_DEBUG(get_logger(), "One-level planning: created single floor segment [%s, 0-%zu]",
                   start_floor.c_str(), result->path.poses.size() - 1);
      
      // Set planning time in result (for validation statistics)
      auto planning_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        t1_one_level - t0_one_level);
      result->planning_time.sec = planning_ns.count() / 1000000000;
      result->planning_time.nanosec = planning_ns.count() % 1000000000;
      
      // Log results
      RCLCPP_INFO(get_logger(), "One-level planning on floor '%s' successful!", start_floor.c_str());
      RCLCPP_INFO(get_logger(), "  Planning time: %.2f ms", planning_time * 1000.0);
      RCLCPP_INFO(get_logger(), "  Path distance: %.4f m", path_distance);
      RCLCPP_INFO(get_logger(), "  Path cost: %.4f", path_cost);
      RCLCPP_INFO(get_logger(), "  Waypoints: %zu", path_opt.value().size());
      
      // Publish path
      path_publisher_->publish(result->path);
      
      result->error_code = ComputePathToPose::Result::NONE;
      action_server_->succeeded_current(result);
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during path planning: %s", e.what());
    result->error_code = ComputePathToPose::Result::UNKNOWN;
    result->error_msg = std::string("Exception: ") + e.what();
    action_server_->terminate_current(result);
  }
}

size_t GppBimServer::findClosestWaypointIndex(
  const nav_msgs::msg::Path & path,
  const std::array<double, 3> & coords)
{
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - coords[0];
    double dy = path.poses[i].pose.position.y - coords[1];
    double dz = path.poses[i].pose.position.z - coords[2];
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

std::tuple<std::optional<std::vector<std::array<double, 3>>>, double, double>
GppBimServer::secondLevelPlanner(
  const std::string & floor_name,
  const std::string & room_name,
  const std::array<double, 3> & start_coords,
  const std::array<double, 3> & end_coords)
{
  // Wait for action server
  if (!room_planner_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(get_logger(), "Room planner action server not available");
    timeout_occurred_ = true;
    return std::make_tuple(std::nullopt, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  }

  // Create goal
  auto goal_msg = ComputePathToPoseInRoom::Goal();
  goal_msg.start = createPoseStamped(start_coords);
  goal_msg.goal = createPoseStamped(end_coords);
  goal_msg.floor_name = floor_name;
  goal_msg.room_name = room_name;
  goal_msg.planner_id = second_level_planner_;
  goal_msg.use_start = true;

  // Send goal and wait for result
  auto send_goal_options = rclcpp_action::Client<ComputePathToPoseInRoom>::SendGoalOptions();

  auto goal_handle_future = room_planner_client_->async_send_goal(goal_msg, send_goal_options);

  // Wait for goal to be accepted (poll instead of spin to avoid executor conflict)
  auto timeout = std::chrono::seconds(2);
  auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok() && 
         goal_handle_future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() - start_time > timeout) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for room planner to accept goal");
      timeout_occurred_ = true;
      return std::make_tuple(std::nullopt, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    }
  }
  
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(get_logger(), "ROS shutdown during room planner goal submission");
    return std::make_tuple(std::nullopt, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by room planner (likely node unavailable)");
    timeout_occurred_ = true;
    return std::make_tuple(std::nullopt, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  }

  // Wait for result (poll instead of spin)
  auto result_future = room_planner_client_->async_get_result(goal_handle);

  timeout = std::chrono::seconds(10);
  start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok() && 
         result_future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() - start_time > timeout) {
      RCLCPP_ERROR(get_logger(), "Timeout waiting for room planner result");
      timeout_occurred_ = true;
      return std::make_tuple(std::nullopt, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    }
  }
  
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(get_logger(), "ROS shutdown while waiting for room planner result");
    return std::make_tuple(std::nullopt, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  }

  auto wrapped_result = result_future.get();
  
  // Check the action result code to detect system failures
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(get_logger(), "Room planner action failed with code: %d", static_cast<int>(wrapped_result.code));
    timeout_occurred_ = true;
    return std::make_tuple(std::nullopt, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  }
  
  if (!wrapped_result.result) {
    RCLCPP_ERROR(get_logger(), "Room planner returned null result (likely node crashed)");
    timeout_occurred_ = true;
    return std::make_tuple(std::nullopt, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  }

  auto result = wrapped_result.result;

  // Check for success - distinguish between planning failures and system failures
  if (result->error_code != ComputePathToPoseInRoom::Result::NONE) {
    // Check if it's a timeout error from room planner
    if (result->error_code == ComputePathToPoseInRoom::Result::TIMEOUT) {
      RCLCPP_ERROR(get_logger(), "Room planner reported timeout: %s", result->error_msg.c_str());
      timeout_occurred_ = true;
    } else {
      RCLCPP_ERROR(
        get_logger(), "Room planner failed: %s (code %d)",
        result->error_msg.c_str(), result->error_code);
    }
    return std::make_tuple(std::nullopt, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  }

  // Convert path to waypoints
  // NOTE: Some planners only work in 2D, so we need to correct z-coordinates
  // to match the floor height (using start_coords z-value)
  std::vector<std::array<double, 3>> waypoints;
  for (const auto & pose : result->path.poses) {
    waypoints.push_back({
      pose.pose.position.x,
      pose.pose.position.y,
      start_coords[2]  // Use start z-coordinate (floor height) instead of path z
    });
  }

  // Return tuple with path, distance, and cost
  return std::make_tuple(waypoints, result->path_distance, result->path_cost);
}

nav_msgs::msg::Path GppBimServer::waypointsToPath(
  const std::vector<std::array<double, 3>> & waypoints,
  const std::string & frame_id)
{
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = frame_id;

  for (const auto & waypoint : waypoints) {
    path.poses.push_back(createPoseStamped(waypoint, frame_id));
  }

  return path;
}

geometry_msgs::msg::PoseStamped GppBimServer::createPoseStamped(
  const std::array<double, 3> & coords,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = now();
  pose.header.frame_id = frame_id;
  pose.pose.position.x = coords[0];
  pose.pose.position.y = coords[1];
  pose.pose.position.z = coords[2];
  pose.pose.orientation.w = 1.0;  // No rotation
  return pose;
}

void GppBimServer::publishPrunedTopomap(const navbim_util::TopologicalGraph & pruned_graph)
{
  try {
    // Convert TopologicalGraph to Topomap message
    navbim_msgs::msg::Topomap topomap_msg;
    navbim_util::TopologicalMap::convertGraphToMessage(pruned_graph, topomap_msg);
    
    // Publish
    pruned_topomap_pub_->publish(topomap_msg);
    
    RCLCPP_DEBUG(get_logger(), "Published pruned topological map with %zu vertices, %zu edges",
                boost::num_vertices(pruned_graph), boost::num_edges(pruned_graph));
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Failed to publish pruned topological map: %s", e.what());
  }
}

rclcpp_action::GoalResponse
GppBimServer::handlePrePlanGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PrePlanEdges::Goal> /*goal*/)
{
  RCLCPP_INFO(get_logger(), "Received pre-plan edges goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
GppBimServer::handlePrePlanCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<PrePlanEdges>> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel pre-planning");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
GppBimServer::executePrePlan(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<PrePlanEdges>> goal_handle)
{
  // Execute in a separate thread to avoid blocking the executor
  std::thread([this, goal_handle]() {
    RCLCPP_INFO(get_logger(), "Received pre-plan edges goal request");
    
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PrePlanEdges::Result>();
    
    bool success = this->executePrePlanImpl(result, goal->force_replan);
    
    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }).detach();
}

bool
GppBimServer::executePrePlanImpl(
  std::shared_ptr<PrePlanEdges::Result> result,
  bool force_replan)
{
  RCLCPP_INFO(get_logger(), "Executing edge pre-planning (force_replan=%s)", 
             force_replan ? "true" : "false");
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // Get the graph (already loaded in on_activate)
  const auto& graph = topological_map_.getGraph();
  
  if (boost::num_vertices(graph) == 0) {
    result->success = false;
    result->message = "Topological map is empty";
    RCLCPP_ERROR(get_logger(), "Cannot pre-plan: topological map is empty");
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "Pre-planning with graph: %zu vertices, %zu edges",
              boost::num_vertices(graph), boost::num_edges(graph));
  
  // Collect all transition edges
  struct EdgeInfo {
    navbim_util::Vertex source;
    navbim_util::Vertex target;
    int source_id;
    int target_id;
    std::string edge_id;
    double planned_cost;
    std::string subtype;
  };
  
  std::vector<EdgeInfo> edges_to_plan;
  int edges_skipped = 0;

  auto edges = boost::edges(graph);
  for (auto eit = edges.first; eit != edges.second; ++eit) {
    const auto& edge_props = graph[*eit];
    auto source = boost::source(*eit, graph);
    auto target = boost::target(*eit, graph);

    const auto& source_node = graph[source];
    const auto& target_node = graph[target];

    // Only consider transition edges (edges with type="transition")
    if (edge_props.type != "transition") {
      continue;
    }

    // Additionally, ensure both endpoints are transition nodes
    if (source_node.type != "transition" || target_node.type != "transition") {
      continue;
    }

    // Skip stair transitions
    std::string subtype = edge_props.subtype.value_or("");
    if (subtype == "stair") {
      edges_skipped++;
      continue;
    }

    // Check if edge needs planning
    bool needs_planning = edge_props.planned_cost < 0.0;

    if (force_replan || needs_planning) {
      EdgeInfo info;
      info.source = source;
      info.target = target;
      info.source_id = source_node.id;
      info.target_id = target_node.id;
      info.edge_id = std::to_string(source_node.id) + "_" + std::to_string(target_node.id);
      info.planned_cost = edge_props.planned_cost;
      info.subtype = subtype;
      edges_to_plan.push_back(info);
    } else {
      edges_skipped++;
    }
  }
  
  RCLCPP_INFO(get_logger(), "Found %zu edges to plan (force_replan=%s)",
              edges_to_plan.size(), force_replan ? "true" : "false");
  
  result->total_edges = edges_to_plan.size();
  
  // Plan each edge
  int edges_completed = 0;
  int edges_failed = 0;
  
  for (size_t i = 0; i < edges_to_plan.size(); ++i) {
    const auto& edge_info = edges_to_plan[i];
    
    // Get edge data for planning
    const auto& source_node = graph[edge_info.source];
    const auto& target_node = graph[edge_info.target];
    
    // Log progress every 100 edges or for first/last edge
    if (i == 0 || i == edges_to_plan.size() - 1 || (i + 1) % 100 == 0) {
      RCLCPP_INFO(get_logger(), "Planning edge %zu/%zu (%.1f%%)",
                 i + 1, edges_to_plan.size(),
                 (static_cast<double>(i + 1) / edges_to_plan.size()) * 100.0);
    }
    
    // Find floor and room from edge
    auto edge_descriptor = boost::edge(edge_info.source, edge_info.target, graph);
    if (!edge_descriptor.second) {
      RCLCPP_ERROR(get_logger(), "Edge not found: %s", edge_info.edge_id.c_str());
      edges_failed++;
      result->failed_edge_ids.push_back(edge_info.edge_id);
      continue;
    }
    
    const auto& edge_props = graph[edge_descriptor.first];
    
    // Get floor name (from source node's floor list)
    std::string floor_name;
    if (source_node.floor.has_value() && !source_node.floor.value().empty()) {
      floor_name = source_node.floor.value()[0];
    }
    
    // Get room name from room_id
    std::string room_name;
    if (edge_props.room_id.has_value()) {
      int room_id = edge_props.room_id.value();
      // Find room node
      auto vertices = boost::vertices(graph);
      for (auto vit = vertices.first; vit != vertices.second; ++vit) {
        if (graph[*vit].id == room_id) {
          room_name = graph[*vit].name;
          break;
        }
      }
    }
    
    if (floor_name.empty() || room_name.empty()) {
      RCLCPP_WARN(get_logger(), "Edge %s: missing floor or room information", 
                  edge_info.edge_id.c_str());
      edges_failed++;
      result->failed_edge_ids.push_back(edge_info.edge_id);
      continue;
    }
    
    // Get start and end coordinates
    std::array<double, 3> start_coords = {
      source_node.position.x,
      source_node.position.y,
      source_node.position.z
    };
    std::array<double, 3> end_coords = {
      target_node.position.x,
      target_node.position.y,
      target_node.position.z
    };
    
    // Call second-level planner
    auto [path_opt, path_distance, path_cost] = secondLevelPlanner(
      floor_name, room_name, start_coords, end_coords);
    
    if (!path_opt.has_value()) {
      RCLCPP_WARN(get_logger(), "Planning failed for edge %s",
                  edge_info.edge_id.c_str());
      edges_failed++;
      result->failed_edge_ids.push_back(edge_info.edge_id);
      continue;
    }
    
    // Send update to topomap_server
    auto request = std::make_shared<UpdateEdgeData::Request>();
    request->source_id = edge_info.source_id;
    request->target_id = edge_info.target_id;
    request->planned_distance = path_distance;
    request->planned_cost = path_cost;
    request->path.header.frame_id = "ifc";
    
    for (const auto& wp : path_opt.value()) {
      geometry_msgs::msg::PoseStamped ps;
      ps.pose.position.x = wp[0];
      ps.pose.position.y = wp[1];
      ps.pose.position.z = wp[2];
      ps.pose.orientation.w = 1.0;
      request->path.poses.push_back(ps);
    }
    
    // Defer save during batch pre-planning to avoid flooding DDS
    request->defer_save = true;
    
    // Call service (async)
    update_edge_data_client_->invoke(request);
    
    edges_completed++;
  }
  
  // Save the topological map once after all updates
  RCLCPP_INFO(get_logger(), "Pre-planning complete, saving topological map...");
  auto save_request = std::make_shared<SaveTopologicalMap::Request>();
  auto save_response = save_topomap_client_->invoke(save_request, std::chrono::seconds(5));
  
  if (save_response && save_response->success) {
    RCLCPP_INFO(get_logger(), "Topological map saved successfully to: %s", 
                save_response->file_path.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Failed to save topological map: %s", 
                save_response ? save_response->message.c_str() : "timeout");
  }
  
  // Complete
  auto end_time = std::chrono::high_resolution_clock::now();
  result->total_planning_time = std::chrono::duration<double>(end_time - start_time).count();
  result->edges_planned = edges_completed;
  result->edges_failed = edges_failed;
  result->edges_skipped = edges_skipped;
  result->success = (edges_failed == 0);
  
  if (result->success) {
    result->message = "Successfully pre-planned all " + std::to_string(edges_completed) + " transition edges";
  } else {
    result->message = "Pre-planning completed with " + std::to_string(edges_failed) + 
                     " failures out of " + std::to_string(edges_to_plan.size()) + " edges";
  }
  
  RCLCPP_INFO(get_logger(), "Pre-planning complete: %s (%.2f ms)",
              result->message.c_str(), result->total_planning_time * 1000.0);
  
  if (!result->failed_edge_ids.empty()) {
    RCLCPP_WARN(get_logger(), "Failed to plan %zu edges: %s", 
                result->failed_edge_ids.size(),
                std::accumulate(result->failed_edge_ids.begin(), result->failed_edge_ids.end(), std::string(),
                  [](const std::string& a, const std::string& b) {
                    return a.empty() ? b : a + ", " + b;
                  }).c_str());
  }
  
  return result->success;
}

}  // namespace navbim_gpp_bim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(navbim_gpp_bim::GppBimServer)
