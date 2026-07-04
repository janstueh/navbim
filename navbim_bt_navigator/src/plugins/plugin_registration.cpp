// Copyright (c) 2024 NAVBIM
// Licensed under the Apache License, Version 2.0

#include "behaviortree_cpp/bt_factory.h"

// Forward declare the node types
namespace navbim_bt_navigator
{
class NavbimComputePathToPoseAction;
}

// Include headers to get full class definitions
#include "navbim_bt_navigator/plugins/action/compute_path_to_pose.hpp"

// Plugin registration function expected by BehaviorTree.CPP
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  // NavbimComputePathToPoseAction needs a custom builder because it requires the action name
  BT::NodeBuilder compute_path_builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<navbim_bt_navigator::NavbimComputePathToPoseAction>(
        name, "navbim_compute_path_to_pose", config);
    };
  factory.registerBuilder<navbim_bt_navigator::NavbimComputePathToPoseAction>(
    "NavbimComputePathToPose", compute_path_builder);
}
