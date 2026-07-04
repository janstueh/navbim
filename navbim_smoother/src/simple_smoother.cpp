// Copyright (c) 2022, Samsung Research America
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
// limitations under the License. Reserved.

#include <vector>
#include <memory>
#include "navbim_smoother/simple_smoother.hpp"
#include "nav2_core/smoother_exceptions.hpp"

namespace navbim_smoother
{
using namespace nav2_util::geometry_utils;  // NOLINT
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;
using smoother_utils::PathSegment;

void SimpleSmoother::configure(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> /*costmap_sub*/,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>/*footprint_sub*/)
{
  // Note: costmap_sub and footprint_sub are no longer used
  // Costmap is now passed directly to smooth() method

  auto node = parent.lock();
  logger_ = node->get_logger();

  declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(1e-10));
  declare_parameter_if_not_declared(
    node, name + ".max_its", rclcpp::ParameterValue(1000));
  declare_parameter_if_not_declared(
    node, name + ".w_data", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, name + ".w_smooth", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, name + ".do_refinement", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, name + ".refinement_num", rclcpp::ParameterValue(2));
  declare_parameter_if_not_declared(
    node, name + ".enforce_path_inversion", rclcpp::ParameterValue(true));

  node->get_parameter(name + ".tolerance", tolerance_);
  node->get_parameter(name + ".max_its", max_its_);
  node->get_parameter(name + ".w_data", data_w_);
  node->get_parameter(name + ".w_smooth", smooth_w_);
  node->get_parameter(name + ".do_refinement", do_refinement_);
  node->get_parameter(name + ".refinement_num", refinement_num_);
  node->get_parameter(name + ".enforce_path_inversion", enforce_path_inversion_);
}

// Base class interface method - not used in NavBIM (kept for plugin compatibility)
bool SimpleSmoother::smooth(
  nav_msgs::msg::Path & /*path*/,
  const rclcpp::Duration & /*max_time*/)
{
  throw nav2_core::SmootherException(
    "SimpleSmoother::smooth(path, max_time) is not supported. "
    "Use smooth(path, costmap, max_time) instead with a direct costmap pointer.");
}

// New smooth() method that accepts direct costmap pointer
bool SimpleSmoother::smooth(
  nav_msgs::msg::Path & path,
  nav2_costmap_2d::Costmap2D * costmap,
  const rclcpp::Duration & max_time)
{
  if (!costmap) {
    throw nav2_core::SmootherException("Costmap pointer is null");
  }

  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time.seconds();

  bool reversing_segment;
  nav_msgs::msg::Path curr_path_segment;
  curr_path_segment.header = path.header;

  std::vector<smoother_utils::PathSegment> path_segments{PathSegment{
      0u, static_cast<unsigned int>(path.poses.size() - 1)}};
  if (enforce_path_inversion_) {
    path_segments = smoother_utils::findDirectionalPathSegments(path);
  }

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  bool overall_success = true;
  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > 3) {
      // Populate path segment
      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      // Make sure we're still able to smooth with time remaining
      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time.seconds() - duration_cast<duration<double>>(now - start).count();
      refinement_ctr_ = 0;

      // Attempt to smooth the segment
      // May throw SmootherTimedOut
      bool segment_success = smoothImpl(curr_path_segment, reversing_segment, costmap, time_remaining);
      overall_success = overall_success && segment_success;

      // Assemble the path changes to the main path
      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  return overall_success;
}

bool SimpleSmoother::smoothImpl(
  nav_msgs::msg::Path & path,
  bool & reversing_segment,
  const nav2_costmap_2d::Costmap2D * costmap,
  const double & max_time)
{
  steady_clock::time_point a = steady_clock::now();
  rclcpp::Duration max_dur = rclcpp::Duration::from_seconds(max_time);

  int its = 0;
  double change = tolerance_;
  const unsigned int & path_size = path.poses.size();
  double x_i, y_i, y_m1, y_ip1, y_i_org;
  unsigned int mx, my;

  nav_msgs::msg::Path new_path = path;
  nav_msgs::msg::Path last_path = path;

  while (change >= tolerance_) {
    its += 1;
    change = 0.0;

    // Make sure the smoothing function will converge
    if (its >= max_its_) {
      RCLCPP_WARN(
        logger_,
        "Number of iterations has exceeded limit of %i.", max_its_);
      path = last_path;
      smoother_utils::updateApproximatePathOrientations(path, reversing_segment);
      return false;
    }

    // Make sure still have time left to process
    steady_clock::time_point b = steady_clock::now();
    rclcpp::Duration timespan(duration_cast<duration<double>>(b - a));
    if (timespan > max_dur) {
      RCLCPP_WARN(
        logger_,
        "Smoothing time exceeded allowed duration of %0.2f.", max_time);
      path = last_path;
      smoother_utils::updateApproximatePathOrientations(path, reversing_segment);
      throw nav2_core::SmootherTimedOut("Smoothing time exceed allowed duration");
    }

    unsigned int skipped_waypoints = 0;
    for (unsigned int i = 1; i != path_size - 1; i++) {
      // Store original position before smoothing
      double original_x = getFieldByDim(new_path.poses[i], 0);
      double original_y = getFieldByDim(new_path.poses[i], 1);
      double waypoint_change = 0.0;
      
      // Smooth x, y, and z coordinates (dimensions 0, 1, 2)
      for (unsigned int j = 0; j != 3; j++) {
        x_i = getFieldByDim(path.poses[i], j);
        y_i = getFieldByDim(new_path.poses[i], j);
        y_m1 = getFieldByDim(new_path.poses[i - 1], j);
        y_ip1 = getFieldByDim(new_path.poses[i + 1], j);
        y_i_org = y_i;

        // Smooth based on local 3 point neighborhood and original data locations
        y_i += data_w_ * (x_i - y_i) + smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
        setFieldByDim(new_path.poses[i], j, y_i);
        waypoint_change += abs(y_i - y_i_org);
      }

      // validate update is admissible, only checks cost if a valid costmap pointer is provided
      float cost = 0.0;
      if (costmap) {
        bool in_bounds = costmap->worldToMap(
          getFieldByDim(new_path.poses[i], 0),
          getFieldByDim(new_path.poses[i], 1),
          mx, my);
        if (in_bounds) {
          cost = static_cast<float>(costmap->getCost(mx, my));
        } else {
          // Waypoint outside costmap bounds - treat as obstacle to prevent smoothing outside map
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        }
      }

      if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost != nav2_costmap_2d::NO_INFORMATION) {
        // Revert ONLY this waypoint, continue with others
        // Don't count this change toward convergence metric
        setFieldByDim(new_path.poses[i], 0, original_x);
        setFieldByDim(new_path.poses[i], 1, original_y);
        skipped_waypoints++;
        RCLCPP_DEBUG(
          rclcpp::get_logger("SmacPlannerSmoother"),
          "Waypoint %u resulted in collision (cost=%f), reverting this point and continuing.", i, cost);
      } else {
        // Only count change for successfully smoothed waypoints
        change += waypoint_change;
      }
    }
    
    // If too many waypoints were skipped, consider this a failure
    if (skipped_waypoints > (path_size - 2) / 2) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("SmacPlannerSmoother"),
        "Smoothing skipped %u/%u waypoints due to collisions. Reverting entire path.",
        skipped_waypoints, path_size - 2);
      path = last_path;
      smoother_utils::updateApproximatePathOrientations(path, reversing_segment);
      return false;
    }

    last_path = new_path;
  }

  // Let's do additional refinement, it shouldn't take more than a couple milliseconds
  // but really puts the path quality over the top.
  if (do_refinement_ && refinement_ctr_ < refinement_num_) {
    refinement_ctr_++;
    bool refinement_success = smoothImpl(new_path, reversing_segment, costmap, max_time);
    if (!refinement_success) {
      return false;
    }
  }

  smoother_utils::updateApproximatePathOrientations(new_path, reversing_segment);
  path = new_path;
  return true;
}

double SimpleSmoother::getFieldByDim(
  const geometry_msgs::msg::PoseStamped & msg, const unsigned int & dim)
{
  if (dim == 0) {
    return msg.pose.position.x;
  } else if (dim == 1) {
    return msg.pose.position.y;
  } else {
    return msg.pose.position.z;
  }
}

void SimpleSmoother::setFieldByDim(
  geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
  const double & value)
{
  if (dim == 0) {
    msg.pose.position.x = value;
  } else if (dim == 1) {
    msg.pose.position.y = value;
  } else {
    msg.pose.position.z = value;
  }
}

}  // namespace navbim_smoother

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(navbim_smoother::SimpleSmoother, nav2_core::Smoother)
