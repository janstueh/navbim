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

#ifndef NAVBIM_GPP_BIM__NAVFN_PLANNER_WRAPPER_HPP_
#define NAVBIM_GPP_BIM__NAVFN_PLANNER_WRAPPER_HPP_

#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace navbim_gpp_bim
{

/**
 * @class NavfnPlannerWrapper
 * @brief Wrapper around NavfnPlanner that allows dynamic costmap switching
 * 
 * This wrapper extends nav2_navfn_planner::NavfnPlanner to expose a method
 * for updating the internal costmap pointer. This is needed for multi-room
 * planning where different costmaps need to be used for different rooms.
 */
class NavfnPlannerWrapper : public nav2_navfn_planner::NavfnPlanner
{
public:
  /**
   * @brief Constructor
   */
  NavfnPlannerWrapper() = default;

  /**
   * @brief Destructor
   */
  ~NavfnPlannerWrapper() = default;

  /**
   * @brief Update the costmap pointer used by the planner
   * @param costmap Pointer to the new costmap to use for planning
   * 
   * This method allows switching the costmap that the planner uses without
   * having to reconfigure the entire planner. Useful for multi-room scenarios
   * where each room has its own costmap.
   */
  void setCostmap(nav2_costmap_2d::Costmap2D * costmap)
  {
    costmap_ = costmap;  // Access protected member from parent class
  }

  /**
   * @brief Get the current costmap pointer
   * @return Pointer to the current costmap
   */
  nav2_costmap_2d::Costmap2D * getCostmap() const
  {
    return costmap_;  // Access protected member from parent class
  }
};

}  // namespace navbim_gpp_bim

#endif  // NAVBIM_GPP_BIM__NAVFN_PLANNER_WRAPPER_HPP_
