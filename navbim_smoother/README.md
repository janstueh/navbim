# navbim_smoother

The `navbim_smoother` package provides a smoother server for the navbim stack, adapted from `nav2_smoother` to work with multi-costmap environments. It hosts smoother plugins that refine paths produced by `navbim_gpp_bim` using the per-room and per-floor costmaps managed by `navbim_multi_costmap_2d`.

## Overview

After the room planner concatenates sub-paths across multiple rooms, the resulting path may contain sharp turns at transition points (door midpoints, stair endpoints) or suboptimal segments near room boundaries. The smoother server refines such paths while respecting obstacle constraints.

The smoother server exposes the `smooth_path` action and loads smoother plugins via `pluginlib`. It is called internally by the `smooth_path_with_floor_costmaps` service of the `room_planner_server` node in `navbim_gpp_bim`, which segments the concatenated path by floor and calls the smoother once per floor segment using the aggregated per-floor costmap. This allows smoothing to move path waypoints slightly away from fixed transition constraints (door midpoints), reducing overall path length while preventing moves into obstacles.

## Node: `smoother_server`

The `SmootherServer` is a `nav2_util::LifecycleNode` that exposes the `SmoothPath` action and loads smoother plugins via `pluginlib`.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `smoother_plugins` | string[] | `["simple_smoother"]` | List of smoother plugin instances to load |
| `costmap_topic` | string | `"global_costmap/costmap_raw"` | Costmap topic for collision checking |
| `footprint_topic` | string | `"global_costmap/published_footprint"` | Footprint topic |
| `robot_base_frame` | string | `"base_link"` | Robot base frame |
| `transform_tolerance` | double | `0.1` | TF lookup timeout in seconds |

### Actions

| Action | Type | Description |
|---|---|---|
| `smooth_path` | `nav2_msgs/action/SmoothPath` | Smooth a given path using the selected smoother plugin |

### Topics Published

| Topic | Type | Description |
|---|---|---|
| `plan_smoothed` | `nav_msgs/msg/Path` | The most recently smoothed path |

## Smoother Plugins

### navbim_simple_smoother

**Plugin type:** `navbim_smoother::SimpleSmoother`

A gradient-descent based path smoother that iteratively minimizes path length and curvature while avoiding obstacles in the costmap. Adapted from `nav2_smoother::SimpleSmoother` to operate on multi-costmap environments.

### navbim_savitzky_golay_smoother

**Plugin type:** `navbim_smoother::SavitzkyGolaySmoother`

A Savitzky-Golay polynomial filter smoother that smooths path coordinates while preserving the general shape of the path. Adapted from `nav2_smoother::SavitzkyGolaySmoother`.

## Related Packages

- `navbim_gpp_bim`: uses the smoother to refine multi-room paths
- `navbim_multi_costmap_2d`: provides the per-room costmaps used for collision-aware smoothing
- `nav2_smoother`: upstream package that this package is based on
