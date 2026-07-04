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

#ifndef NAVBIM_GPP_BIM__OMPL_PLANNER_WRAPPER_HPP_
#define NAVBIM_GPP_BIM__OMPL_PLANNER_WRAPPER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

// OMPL includes
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

namespace navbim_gpp_bim
{

/**
 * @class OMPLPlannerWrapper
 * @brief Wrapper around OMPL planners (BIT*, RRT*, etc.) for Nav2 integration
 * 
 * This wrapper allows OMPL sample-based planners to work with Nav2's costmap
 * and supports dynamic costmap switching for multi-room environments.
 */
class OMPLPlannerWrapper : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief Constructor
   */
  OMPLPlannerWrapper() = default;

  /**
   * @brief Destructor
   */
  ~OMPLPlannerWrapper() = default;

  /**
   * @brief Configure the planner
   */
  void configure(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup the planner
   */
  void cleanup() override;

  /**
   * @brief Activate the planner
   */
  void activate() override;

  /**
   * @brief Deactivate the planner
   */
  void deactivate() override;

  /**
   * @brief Create a plan from start to goal
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

  /**
   * @brief Update the costmap pointer used by the planner
   * @param costmap Pointer to the new costmap to use for planning
   */
  void setCostmap(nav2_costmap_2d::Costmap2D * costmap)
  {
    costmap_ = costmap;
  }

  /**
   * @brief Get the current costmap pointer
   * @return Pointer to the current costmap
   */
  nav2_costmap_2d::Costmap2D * getCostmap() const
  {
    return costmap_;
  }

private:
  /**
   * @brief Check if a state is valid (collision-free)
   */
  bool isStateValid(const ompl::base::State * state);

  /**
   * @brief Convert OMPL path to ROS path message
   */
  nav_msgs::msg::Path omplPathToRosPath(
    const ompl::geometric::PathGeometric & path,
    const std::string & frame_id,
    const rclcpp::Time & stamp);

  /**
   * @brief Initialize OMPL SimpleSetup with selected planner
   */
  void setupPlanner();

  // Configuration
  nav2_util::LifecycleNode::WeakPtr node_;
  std::string name_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("OMPLPlannerWrapper")};

  // OMPL components
  std::shared_ptr<ompl::base::RealVectorStateSpace> space_;
  std::shared_ptr<ompl::geometric::SimpleSetup> simple_setup_;

  // Parameters
  std::string planner_type_;  // "BITstar", "RRTstar", "InformedRRTstar", "PRMstar"
  double planning_time_;      // Maximum planning time in seconds
  bool stop_on_first_solution_; // Stop as soon as first solution is found
  bool simplify_solution_;    // Whether to simplify the path
  double range_;              // Maximum edge length for sampling-based planners
  double goal_bias_;          // Probability of sampling the goal
  double interpolation_resolution_; // Resolution for path interpolation
  bool use_informed_sampling_; // Use informed sampling (only for informed planners)
  
  // Cost parameters
  double lethal_cost_threshold_;  // Treat costs above this as lethal
  double inscribed_cost_threshold_; // Inscribed radius cost threshold
  bool allow_unknown_;  // Allow planning through unknown space
};

}  // namespace navbim_gpp_bim

#endif  // NAVBIM_GPP_BIM__OMPL_PLANNER_WRAPPER_HPP_
