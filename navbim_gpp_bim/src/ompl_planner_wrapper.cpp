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

#include "navbim_gpp_bim/ompl_planner_wrapper.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include <ompl/util/Console.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <cmath>

namespace navbim_gpp_bim
{

void OMPLPlannerWrapper::configure(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  logger_ = node->get_logger();
  name_ = name;
  costmap_ = costmap_ros->getCostmap();

  // Set OMPL log level to ERROR to suppress verbose output
  // Options: LOG_DEV2, LOG_DEV1, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_NONE
  ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

  // Declare and get parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".planner_type", rclcpp::ParameterValue("BITstar"));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".planning_time", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".stop_on_first_solution", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".simplify_solution", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".range", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_bias", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_informed_sampling", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lethal_cost_threshold", rclcpp::ParameterValue(253.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".inscribed_cost_threshold", rclcpp::ParameterValue(253.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".planner_type", planner_type_);
  node->get_parameter(name_ + ".planning_time", planning_time_);
  node->get_parameter(name_ + ".stop_on_first_solution", stop_on_first_solution_);
  node->get_parameter(name_ + ".simplify_solution", simplify_solution_);
  node->get_parameter(name_ + ".range", range_);
  node->get_parameter(name_ + ".goal_bias", goal_bias_);
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node->get_parameter(name_ + ".use_informed_sampling", use_informed_sampling_);
  node->get_parameter(name_ + ".lethal_cost_threshold", lethal_cost_threshold_);
  node->get_parameter(name_ + ".inscribed_cost_threshold", inscribed_cost_threshold_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
}

void OMPLPlannerWrapper::cleanup()
{
  simple_setup_.reset();
  space_.reset();
}

void OMPLPlannerWrapper::activate()
{
}

void OMPLPlannerWrapper::deactivate()
{
}

void OMPLPlannerWrapper::setupPlanner()
{
  if (!costmap_) {
    throw std::runtime_error("OMPLPlannerWrapper: costmap not set");
  }

  // Create 2D state space (x, y)
  space_ = std::make_shared<ompl::base::RealVectorStateSpace>(2);

  // Set bounds based on costmap
  ompl::base::RealVectorBounds bounds(2);
  bounds.setLow(0, costmap_->getOriginX());
  bounds.setLow(1, costmap_->getOriginY());
  bounds.setHigh(0, costmap_->getOriginX() + costmap_->getSizeInMetersX());
  bounds.setHigh(1, costmap_->getOriginY() + costmap_->getSizeInMetersY());
  space_->setBounds(bounds);

  // Create SimpleSetup
  simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(space_);

  // Set state validity checker
  simple_setup_->setStateValidityChecker(
    [this](const ompl::base::State * state) {
      return this->isStateValid(state);
    });

  // Set state validity checking resolution to check every 5 cm along edges
  // This ensures paths don't cut through thin obstacles or walls
  // Calculate what fraction of max extent equals 5 cm
  double max_extent = space_->getMaximumExtent();
  double desired_resolution_m = 0.05;  // 5 cm
  double resolution_fraction = desired_resolution_m / max_extent;
  simple_setup_->getSpaceInformation()->setStateValidityCheckingResolution(resolution_fraction);
  
  RCLCPP_DEBUG(logger_, "State validity checking: %.4f m (%.6f of max extent %.2f m)",
               desired_resolution_m, resolution_fraction, max_extent);

  // Create and set the planner based on type
  ompl::base::PlannerPtr planner;
  
  if (planner_type_ == "BITstar") {
    auto bitstar = std::make_shared<ompl::geometric::BITstar>(simple_setup_->getSpaceInformation());
    bitstar->setRewireFactor(1.1);
    if (use_informed_sampling_) {
      bitstar->setUseKNearest(false);  // Use r-disc connection strategy for better performance
    }    
    planner = bitstar;
  } else if (planner_type_ == "RRTstar") {
    auto rrtstar = std::make_shared<ompl::geometric::RRTstar>(simple_setup_->getSpaceInformation());
    rrtstar->setRange(range_);
    rrtstar->setGoalBias(goal_bias_);
    planner = rrtstar;
  } else if (planner_type_ == "InformedRRTstar") {
    auto informed_rrtstar = std::make_shared<ompl::geometric::InformedRRTstar>(
      simple_setup_->getSpaceInformation());
    informed_rrtstar->setRange(range_);
    informed_rrtstar->setGoalBias(goal_bias_);
    planner = informed_rrtstar;
  } else if (planner_type_ == "RRTConnect") {
    auto rrtconnect = std::make_shared<ompl::geometric::RRTConnect>(
      simple_setup_->getSpaceInformation());
    rrtconnect->setRange(range_);
    // RRTConnect does not have setGoalBias
    planner = rrtconnect;
  } else if (planner_type_ == "PRMstar") {
    auto prmstar = std::make_shared<ompl::geometric::PRMstar>(simple_setup_->getSpaceInformation());
    planner = prmstar;
  } else {
    RCLCPP_WARN(
      logger_, "Unknown planner type '%s', defaulting to BITstar",
      planner_type_.c_str());
    planner = std::make_shared<ompl::geometric::BITstar>(simple_setup_->getSpaceInformation());
  }

  simple_setup_->setPlanner(planner);
}

bool OMPLPlannerWrapper::isStateValid(const ompl::base::State * state)
{
  if (!costmap_) {
    return false;
  }

  // Extract x, y from state
  const auto * state2d = state->as<ompl::base::RealVectorStateSpace::StateType>();
  double wx = state2d->values[0];
  double wy = state2d->values[1];

  // Convert world coordinates to map coordinates
  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return false;  // Outside map bounds
  }

  // Get cost at this position
  unsigned char cost = costmap_->getCost(mx, my);

  // Check if cost is acceptable
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  // Reject states with cost above lethal threshold
  if (cost >= static_cast<unsigned char>(lethal_cost_threshold_)) {
    return false;
  }

  // Reject states in inscribed area (within robot's inscribed radius of obstacles)
  if (cost >= static_cast<unsigned char>(inscribed_cost_threshold_)) {
    return false;
  }

  return true;
}

nav_msgs::msg::Path OMPLPlannerWrapper::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header = start.header;

  // Check for cancellation
  if (cancel_checker && cancel_checker()) {
    RCLCPP_INFO(logger_, "Planning cancelled before starting");
    return path;
  }

  if (!costmap_) {
    RCLCPP_ERROR(logger_, "Costmap is not set");
    return path;
  }

  // Setup the planner (recreate each time to ensure clean state)
  setupPlanner();

  // Set start state
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start_state(space_);
  start_state[0] = start.pose.position.x;
  start_state[1] = start.pose.position.y;

  // Set goal state
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal_state(space_);
  goal_state[0] = goal.pose.position.x;
  goal_state[1] = goal.pose.position.y;

  // Check if start and goal are valid
  if (!isStateValid(start_state.get())) {
    RCLCPP_ERROR(logger_, "Start position is not valid (in collision or outside bounds)");
    return path;
  }

  if (!isStateValid(goal_state.get())) {
    RCLCPP_ERROR(logger_, "Goal position is not valid (in collision or outside bounds)");
    return path;
  }

  // Set start and goal
  simple_setup_->setStartAndGoalStates(start_state, goal_state);

  // Attempt to solve
  ompl::base::PlannerStatus solved;
  
  if (stop_on_first_solution_) {
    // Stop as soon as first solution is found (not optimized)
    // NOTE: Optimizing planners (BIT*, RRT*) may not check termination frequently
    // and can still take significant time even with this setting
    simple_setup_->setup();
    auto pdef = simple_setup_->getProblemDefinition();
    // Set objective with infinite cost threshold to avoid optimization
    auto obj = std::make_shared<ompl::base::PathLengthOptimizationObjective>(
      simple_setup_->getSpaceInformation());
    obj->setCostThreshold(ompl::base::Cost(std::numeric_limits<double>::infinity()));
    pdef->setOptimizationObjective(obj);
    // Create termination condition
    auto goal_reached = [pdef]() { return pdef->getSolutionCount() > 0; };
    auto ptc = ompl::base::plannerOrTerminationCondition(
      ompl::base::timedPlannerTerminationCondition(planning_time_),
      ompl::base::PlannerTerminationCondition(goal_reached)
    );
    solved = simple_setup_->getPlanner()->solve(ptc);

  } else {
    // Continue optimizing within time limit
    solved = simple_setup_->solve(planning_time_);
  }

  if (solved) {
    // Get the solution path
    ompl::geometric::PathGeometric & solution_path = simple_setup_->getSolutionPath();
    
    // Validate that the path actually reaches the goal
    if (solution_path.getStateCount() == 0) {
      RCLCPP_ERROR(logger_, "Solution path is empty");
      return path;
    }
    
    const auto * last_state = solution_path.getState(solution_path.getStateCount() - 1)
      ->as<ompl::base::RealVectorStateSpace::StateType>();
    double dx = last_state->values[0] - goal.pose.position.x;
    double dy = last_state->values[1] - goal.pose.position.y;
    double distance_to_goal = std::sqrt(dx * dx + dy * dy);
    
    // Check if path actually reaches goal (within reasonable tolerance)
    const double goal_tolerance = 0.1;  // meters
    if (distance_to_goal > goal_tolerance) {
      RCLCPP_ERROR(logger_, 
        "Path does not reach goal! Distance: %.3f m (tolerance: %.3f m). "
        "Path end: (%.2f, %.2f), Goal: (%.2f, %.2f)",
        distance_to_goal, goal_tolerance,
        last_state->values[0], last_state->values[1],
        goal.pose.position.x, goal.pose.position.y);
      return path;  // Return empty path
    }
    
    // Simplify solution if requested
    if (simplify_solution_) {
      simple_setup_->simplifySolution();
    }

    // Interpolate the path for smoother motion
    solution_path.interpolate(
      static_cast<unsigned int>(solution_path.length() / interpolation_resolution_));

    // Convert to ROS path
    path = omplPathToRosPath(solution_path, start.header.frame_id, start.header.stamp);
  } else {
    RCLCPP_WARN(
      logger_, "OMPL planning failed with status: %s",
      solved.asString().c_str());
  }

  return path;
}

nav_msgs::msg::Path OMPLPlannerWrapper::omplPathToRosPath(
  const ompl::geometric::PathGeometric & ompl_path,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = frame_id;
  ros_path.header.stamp = stamp;

  for (size_t i = 0; i < ompl_path.getStateCount(); ++i) {
    const auto * state = ompl_path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header = ros_path.header;
    pose.pose.position.x = state->values[0];
    pose.pose.position.y = state->values[1];
    pose.pose.position.z = 0.0;

    // Calculate orientation from path direction
    if (i + 1 < ompl_path.getStateCount()) {
      const auto * next_state = 
        ompl_path.getState(i + 1)->as<ompl::base::RealVectorStateSpace::StateType>();
      double dx = next_state->values[0] - state->values[0];
      double dy = next_state->values[1] - state->values[1];
      double yaw = std::atan2(dy, dx);
      
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      pose.pose.orientation.z = std::sin(yaw / 2.0);
    } else {
      // Last pose: use previous orientation
      if (i > 0) {
        pose.pose.orientation = ros_path.poses.back().pose.orientation;
      } else {
        pose.pose.orientation.w = 1.0;
      }
    }

    ros_path.poses.push_back(pose);
  }

  return ros_path;
}

}  // namespace navbim_gpp_bim
