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

#ifndef NAVBIM_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_
#define NAVBIM_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_core/smoother.hpp"
#include "nav2_smoother/smoother_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"
#include "tf2/utils.hpp"

namespace navbim_smoother
{

/**
 * @class navbim_smoother::SavitzkyGolaySmoother
 * @brief A path smoother implementation using Savitzky Golay filters
 */
class SavitzkyGolaySmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief A constructor for nav2_smoother::SavitzkyGolaySmoother
   */
  SavitzkyGolaySmoother() = default;

  /**
   * @brief A destructor for nav2_smoother::SavitzkyGolaySmoother
   */
  ~SavitzkyGolaySmoother() override = default;

  /**
   * @brief Configure the smoother (simplified - no costmap subscriber needed)
   * @param parent Weak pointer to parent lifecycle node
   * @param name Name of the smoother
   * @param tf TF buffer (unused but kept for interface compatibility)
   * @param costmap_sub Costmap subscriber (unused - kept for backward compatibility, can be nullptr)
   * @param footprint_sub Footprint subscriber (unused - kept for backward compatibility, can be nullptr)
   */
  void configure(
    const nav2_util::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override;

  /**
   * @brief Method to cleanup resources.
   */
  void cleanup() override {}

  /**
   * @brief Method to activate smoother and any threads involved in execution.
   */
  void activate() override {}

  /**
   * @brief Method to deactivate smoother and any threads involved in execution.
   */
  void deactivate() override {}

  /**
   * @brief Method to smooth given path (base class interface - not used in NavBIM)
   * 
   * This method is kept for plugin interface compatibility but will throw an exception.
   * Use the overloaded smooth() method with direct costmap pointer instead.
   *
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

  /**
   * @brief Method to smooth given path with direct costmap pointer
   *
   * @param path In-out path to be smoothed
   * @param costmap Pointer to costmap for collision checking
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    nav2_costmap_2d::Costmap2D * costmap,
    const rclcpp::Duration & max_time);

  /**
   * @brief Method to calculate SavitzkyGolay Coefficients
   */
  void calculateCoefficients();

protected:
  /**
   * @brief Smoother method - does the smoothing on a segment
   * @param path Reference to path
   * @param reversing_segment Return if this is a reversing segment
   * @param costmap Pointer to minimal costmap for collision checking
   * @return If smoothing was successful (no collisions)
   */
  bool smoothImpl(
    nav_msgs::msg::Path & path,
    bool & reversing_segment,
    const nav2_costmap_2d::Costmap2D * costmap);

  bool do_refinement_, enforce_path_inversion_;
  int refinement_num_, window_size_, half_window_size_, poly_order_;
  Eigen::VectorXd sg_coeffs_;
  rclcpp::Logger logger_{rclcpp::get_logger("SGSmoother")};
};

}  // namespace navbim_smoother

#endif  // NAVBIM_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_
