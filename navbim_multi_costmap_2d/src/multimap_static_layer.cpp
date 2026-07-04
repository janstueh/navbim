// Copyright (c) 2024 NavBIM
// Licensed under the Apache License, Version 2.0

#include "navbim_multi_costmap_2d/multimap_static_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(navbim_multi_costmap_2d::MultimapStaticLayer, nav2_costmap_2d::Layer)

namespace navbim_multi_costmap_2d
{

void MultimapStaticLayer::onInitialize()
{
  // Replicate essential StaticLayer initialization, but use callback_group_ from the start
  global_frame_ = layered_costmap_->getGlobalFrameID();
  
  // Get parameters (same as parent)
  getParameters();

  // Set up QoS for map subscription
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_qos.reliable();

  RCLCPP_DEBUG(
    logger_,
    "MultimapStaticLayer: Subscribing to map topic (%s) with transient_local durability using callback_group",
    map_topic_.c_str());

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // CRITICAL: Create subscription with callback_group_ from the start
  // This ensures the map is delivered to a subscription that will actually be processed
  rclcpp::SubscriptionOptions map_sub_options;
  map_sub_options.callback_group = callback_group_;
  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_,
    map_qos,
    std::bind(&MultimapStaticLayer::incomingMap, this, std::placeholders::_1),
    map_sub_options);

  if (subscribe_to_updates_) {
    RCLCPP_DEBUG(logger_, "MultimapStaticLayer: Subscribing to updates with callback group");
    rclcpp::SubscriptionOptions update_sub_options;
    update_sub_options.callback_group = callback_group_;
    map_update_sub_ = node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      map_topic_ + "_updates",
      rclcpp::QoS(10),
      std::bind(&MultimapStaticLayer::incomingUpdate, this, std::placeholders::_1),
      update_sub_options);
  }
}

}  // namespace navbim_multi_costmap_2d
