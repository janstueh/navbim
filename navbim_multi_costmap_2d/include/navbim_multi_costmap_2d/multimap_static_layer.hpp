// Copyright (c) 2024 NavBIM
// Licensed under the Apache License, Version 2.0

#ifndef NAVBIM_MULTI_COSTMAP_2D__MULTIMAP_STATIC_LAYER_HPP_
#define NAVBIM_MULTI_COSTMAP_2D__MULTIMAP_STATIC_LAYER_HPP_

#include "nav2_costmap_2d/static_layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace navbim_multi_costmap_2d
{

/**
 * @class MultimapStaticLayer
 * @brief Custom StaticLayer that properly uses callback_group_ for subscriptions
 * 
 * This extends nav2_costmap_2d::StaticLayer to ensure that map subscriptions
 * are created with the correct callback group, allowing them to be processed
 * by the parent costmap's executor thread.
 */
class MultimapStaticLayer : public nav2_costmap_2d::StaticLayer
{
public:
  MultimapStaticLayer() = default;
  virtual ~MultimapStaticLayer() = default;

  /**
   * @brief Override onInitialize to create subscriptions with callback_group_
   */
  void onInitialize() override;
};

}  // namespace navbim_multi_costmap_2d

#endif  // NAVBIM_MULTI_COSTMAP_2D__MULTIMAP_STATIC_LAYER_HPP_
