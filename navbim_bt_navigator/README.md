# navbim_bt_navigator

The `navbim_bt_navigator` package extends the Nav2 BT Navigator with navbim-specific behavior tree actions and navigators for BIM-based robot navigation. It provides a `NavigateToPoseNavigator` that uses the navbim global path planner (`navbim_gpp_bim`) via custom BT action nodes.

## Overview

The BT Navigator receives a navigation goal and orchestrates the robot to reach it by executing a Behavior Tree. This package adds BT action nodes that interface with navbim-specific services and actions, in particular the room-level path planner. It is based on `nav2_bt_navigator` and inherits its recovery and replanning infrastructure.

See the [nav2_bt_navigator documentation](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html) for general background on BT-based navigation in Nav2.

## Node: `bt_navigator` (executable: `bt_navigator`)

`BtNavigator` is a lifecycle node that loads behavior trees and runs them via `nav2_behavior_tree`. It is based on `nav2_bt_navigator` and inherits its full parameter set. The navbim-specific parameters and their defaults are listed below; see the [nav2_bt_navigator documentation](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html) for the complete parameter reference.

### Actions

| Action | Type | Description |
|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Navigate the robot to a target pose using the configured behavior tree |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `global_frame` | string | `"ifc"` | Global reference frame |
| `robot_base_frame` | string | `"base_link"` | Robot base frame |
| `transform_tolerance` | double | `0.1` | TF lookup timeout in seconds |
| `odom_topic` | string | `"odom"` | Odometry topic for current robot velocity |
| `filter_duration` | double | `0.3` | Duration for filtering velocity commands |
| `plugin_lib_names` | string[] | `[]` | Additional BT plugin libraries to load |
| `navigators` | string[] | `["navigate_to_pose"]` | Navigator plugin instances to load |

## Navigator Plugins

### NavigateToPoseNavigator

**Plugin type:** `navbim_bt_navigator::NavigateToPoseNavigator`

Replaces the standard `NavigateToPoseNavigator` from Nav2 with one that dispatches global path planning requests to `navbim_gpp_bim` via the `NavbimComputePathToPose` BT action node.

## Behavior Tree Action Nodes

### NavbimComputePathToPose

Calls the `navbim_gpp_bim` room planner to compute a multi-floor path to a target pose.

| Port | Direction | Description |
|---|---|---|
| `goal` | input | Target pose |
| `start` | input | Start pose (optional; uses robot pose if unset) |
| `planner_id` | input | Planner plugin ID |
| `path` | output | Computed path |

## Behavior Trees

| File | Description |
|---|---|
| `navbim_plan_to_pose.xml` | Single-shot path planning to a pose; the default behavior tree used by `NavigateToPoseNavigator` |

## Related Packages

- `navbim_gpp_bim`: implements the `NavbimComputePathToPose` action server called by this navigator
- `navbim_msgs`: defines the `NavbimComputePathToPose` action
