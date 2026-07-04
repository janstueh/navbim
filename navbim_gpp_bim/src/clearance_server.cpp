// Copyright (c) 2024 navBIM
//
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

#include "navbim_gpp_bim/clearance_server.hpp"

#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>
#include <limits>
#include <cmath>

#include "lifecycle_msgs/msg/state.hpp"

namespace navbim_gpp_bim
{

ClearanceServer::ClearanceServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("clearance_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating ClearanceServer");
}

ClearanceServer::~ClearanceServer()
{
  RCLCPP_INFO(get_logger(), "Destroying ClearanceServer");
}

nav2_util::CallbackReturn
ClearanceServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Create callback group for clearance service
  clearance_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create costmap as sub-node (following Nav2 pattern from room_planner_server)
  try {
    costmap_ros_ = std::make_shared<navbim_multi_costmap_2d::MultiCostmap2DROS>(
      "clearance_costmap",
      std::string{get_namespace()},
      get_parameter("use_sim_time").as_bool(),
      node_options_);

    // Configure costmap (it will create its own executor thread internally via parent)
    costmap_ros_->configure();

    RCLCPP_INFO(get_logger(), "Costmap configured successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to create costmap: %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Create clearance calculation service
  clearance_service_ = create_service<navbim_msgs::srv::CalculatePathClearance>(
    "/calculate_path_clearance",
    std::bind(
      &ClearanceServer::calculatePathClearance,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3),
    rclcpp::SystemDefaultsQoS(),
    clearance_callback_group_);

  RCLCPP_INFO(get_logger(), "Configured successfully");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ClearanceServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activate costmap
  if (costmap_ros_) {
    const auto costmap_ros_state = costmap_ros_->activate();
    if (costmap_ros_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_ERROR(get_logger(), "Costmap failed to activate");
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  // Create bond connection
  createBond();

  RCLCPP_INFO(get_logger(), "Activated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ClearanceServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Destroy bond connection
  destroyBond();

  // Deactivate costmap
  if (costmap_ros_) {
    costmap_ros_->deactivate();
  }

  RCLCPP_INFO(get_logger(), "Deactivated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ClearanceServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Clear service
  clearance_service_.reset();
  
  // Note: Don't reset callback group here - it will be cleaned up automatically
  // when the node is destroyed. Resetting it while service might still be
  // detaching from waitsets can cause FastDDS mutex assertions.
  
  // Give FastDDS time to finish processing any in-flight messages
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Cleanup costmap (it manages its own executor internally)
  if (costmap_ros_) {
    costmap_ros_->cleanup();
    RCLCPP_INFO(get_logger(), "Costmap cleanup complete");
  }

  RCLCPP_INFO(get_logger(), "Cleaned up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ClearanceServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void ClearanceServer::calculatePathClearance(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::CalculatePathClearance::Request> request,
  std::shared_ptr<navbim_msgs::srv::CalculatePathClearance::Response> response)
{
  try {
    // Validate input
    if (request->path.poses.empty()) {
      response->success = false;
      response->message = "Empty path provided";
      return;
    }
    
    if (request->floor_names.empty()) {
      response->success = false;
      response->message = "No floor segments provided";
      return;
    }
    
    // Validate segment arrays are same size
    if (request->floor_names.size() != request->segment_start_indices.size() ||
        request->floor_names.size() != request->segment_end_indices.size()) {
      response->success = false;
      response->message = "Floor segment arrays have mismatched sizes";
      return;
    }
    
    // Validate segment indices are within path bounds
    size_t path_size = request->path.poses.size();
    for (size_t i = 0; i < request->floor_names.size(); ++i) {
      if (request->segment_start_indices[i] >= path_size ||
          request->segment_end_indices[i] >= path_size) {
        response->success = false;
        response->message = "Segment indices out of bounds: [" + 
                           std::to_string(request->segment_start_indices[i]) + ", " +
                           std::to_string(request->segment_end_indices[i]) + "] for path size " +
                           std::to_string(path_size);
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
      }
      if (request->segment_start_indices[i] > request->segment_end_indices[i]) {
        response->success = false;
        response->message = "Invalid segment: start_index (" + 
                           std::to_string(request->segment_start_indices[i]) + 
                           ") > end_index (" + std::to_string(request->segment_end_indices[i]) + ")";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
      }
    }
    
    RCLCPP_DEBUG(get_logger(), "Calculating clearance for %zu floor segments", 
                 request->floor_names.size());
    std::list<std::tuple<uint32_t, double>> clearances;  // List of (path index, clearance) for valid points
    
    // CRITICAL: Hold shared locks during clearance calculation to prevent data races
    // Floor costmaps require floor_cache_mutex_ for pointer validity
    // NOTE: Floor costmaps are created by timer thread, NOT on-demand
    // If a floor costmap doesn't exist, clearance calculation will fail
    std::shared_lock<std::shared_mutex> costmap_lock(costmap_ros_->getCostmapMutex());
    std::shared_lock<std::shared_mutex> floor_lock(costmap_ros_->getFloorCacheMutex());
    
    // Process each floor segment
    for (size_t i = 0; i < request->floor_names.size(); ++i) {
      const std::string& floor_name = request->floor_names[i];
      uint32_t start_index = request->segment_start_indices[i];
      uint32_t end_index = request->segment_end_indices[i];
      
      // Get costmap for this floor (floor_lock protects pointer)
      auto costmap = costmap_ros_->getCostmapForFloor(floor_name);
      if (!costmap) {
        response->success = false;
        response->message = "Failed to get costmap for floor: " + floor_name;
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
      }
      
      // Calculate segment clearance
      if (!calculateSegmentClearance(
          request->path, start_index, end_index, costmap.get(),
          clearances))
      {
        response->success = false;
        response->message = "Failed to calculate clearance for segment " + std::to_string(i) + 
                           " (floor: " + floor_name + ", indices: " + 
                           std::to_string(start_index) + "-" + 
                           std::to_string(end_index) + ")";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
      }
    }
    
    // Calculate min and average clearance across entire path
    size_t overall_count = clearances.size();
    if (overall_count == 0) {
      response->success = false;
      response->message = "No valid clearances calculated for the path";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    double overall_min = std::accumulate(clearances.begin(), clearances.end(), 
      std::numeric_limits<double>::infinity(), [](double current_min, const auto& t) {
      return std::min(current_min, std::get<1>(t));
    });
    double overall_sum = std::accumulate(clearances.begin(), clearances.end(), 0.0, 
      [](double current_sum, const auto& t) {
      return current_sum + std::get<1>(t);
    });

    if (overall_min > 1.0) {
      RCLCPP_WARN(get_logger(), "High minimum clearance detected: %.2f.", overall_min);
      for (size_t i = 0; i < request->floor_names.size(); ++i) {
        auto costmap = costmap_ros_->getCostmapForFloor(request->floor_names[i]);
        if (!costmap) {
          continue;
        }
        // Print the costmap values (safe because we still hold locks)
        for (unsigned int y = 0; y < costmap->getSizeInCellsY(); ++y) {
          std::string row;
          for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x) {
            unsigned char cost = costmap->getCost(x, y);
            if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
              row += "X";
            } else if (cost == nav2_costmap_2d::NO_INFORMATION) {
              row += "?";
            } else {
              row += ".";
            }
          }
          RCLCPP_WARN(get_logger(), "    %s", row.c_str());
        }
      }
    }
    
    response->min_clearance = overall_min;
    response->avg_clearance = overall_sum / overall_count;
    response->success = true;
    response->message = "Clearance calculation successful";
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in clearance calculation: %s", e.what());
    response->success = false;
    response->message = std::string("Exception during clearance calculation: ") + e.what();
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Unknown exception in clearance calculation");
    response->success = false;
    response->message = "Unknown exception during clearance calculation";
  }
}

bool ClearanceServer::calculateSegmentClearance(
  const nav_msgs::msg::Path & path,
  size_t start_idx,
  size_t end_idx,
  const nav2_costmap_2d::Costmap2D * costmap,
  std::list<std::tuple<uint32_t, double>> & clearances)
{
  if (!costmap) {
    return false;
  }
  
  uint32_t valid_clearance_count = 0;
  for (size_t i = start_idx; i <= end_idx; ++i) {
    const auto & pose = path.poses[i].pose;
    double clearance = calculateClearanceAtPose(
      pose.position.x,
      pose.position.y,
      costmap);
    if (clearance >= 0.0) {  // Valid clearance
      clearances.emplace_back(i, clearance);
      ++valid_clearance_count;
    }
  }
  if (valid_clearance_count == 0) {
    return false;
  }
  return true;
}

double ClearanceServer::calculateClearanceAtPose(
  double x,
  double y,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  // Convert world coordinates to map coordinates
  unsigned int mx, my;
  if (!costmap->worldToMap(x, y, mx, my)) {
    return -1.0;  // Outside map bounds
  }
  
  // Get cost at this location, if in lethal obstacle, clearance is 0
  unsigned char cost = costmap->getCost(mx, my);
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
    return 0.0;
  }
  
  // Search outward in cell rings, up to a radius corresponding to 10 meters
  double resolution = costmap->getResolution();
  int max_search_radius = static_cast<int>(std::ceil(10.0 / resolution));
  double min_dist = std::numeric_limits<double>::infinity();
  
  for (int radius = 1; radius <= max_search_radius; ++radius) {
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        if (std::abs(dx) != radius && std::abs(dy) != radius) {
          continue;
        }
        int cx = mx + dx;
        int cy = my + dy;
        if (cx < 0 || cy < 0 || 
          cx >= static_cast<int>(costmap->getSizeInCellsX()) ||
          cy >= static_cast<int>(costmap->getSizeInCellsY())) {
          // Out of bounds, calculate distance to potential midpoint of cell as worst-case obstacle distance
          double cell_x = (cx + 0.5) * resolution + costmap->getOriginX();
          double cell_y = (cy + 0.5) * resolution + costmap->getOriginY();
          double dist = std::hypot(x - cell_x, y - cell_y);
          min_dist = std::min(min_dist, dist);
        } else if (costmap->getCost(cx, cy) == nav2_costmap_2d::LETHAL_OBSTACLE) {
          // Calculate distance to obstacle cell
          double obs_x, obs_y;
          costmap->mapToWorld(cx, cy, obs_x, obs_y);
          double dist = std::hypot(x - obs_x, y - obs_y);
          min_dist = std::min(min_dist, dist);
        }
      }
    }
    // Safe early-exit: no closer obstacle possible
    if (min_dist < (radius - 1) * resolution) {
      break;
    }
  }
  return std::isfinite(min_dist) ? min_dist : 10.0;
}

}  // namespace navbim_gpp_bim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(navbim_gpp_bim::ClearanceServer)
