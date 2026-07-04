# navbim_gpp_bim

**GPP-BIM — Global path planning for robot navigation using building information models**

The `navbim_gpp_bim` package implements global path planning for robot navigation using BIM models in multi-floor indoor environments. Using BIM models eliminates the need for prerecording maps and leverages the geometric and semantic information stored within them. 

## Overview

Classical Nav2 planners operate on a single 2D occupancy grid. Multi-floor buildings require navigation across rooms and floors whose spatial relationships are naturally encoded in BIM models. GPP-BIM applies a divide-and-conquer paradigm: rather than searching for a long path in a single large map, it substitutes the problem with multiple short sub-path searches in per-room occupancy maps, guided by a topological map.

The approach has two levels:

1. **Level 1 — Topological map**: An edge-expanding A* algorithm searches the topological map to find the sequence of rooms and transition points (door midpoints, stair waypoints) connecting the start pose to the goal pose.
2. **Level 2 — Occupancy maps**: For each edge in the topological route, a metric planner finds a collision-free path within the room's cost map. The sub-paths are concatenated and smoothed into a single optimal output path.

Sub-paths between fixed transition nodes can be precomputed offline, reducing online planning time by up to 65% for complex multi-floor paths.

## Getting Started

### Start the Devcontainer

The repository ships a VS Code devcontainer that provides a pre-built ROS 2 Jazzy environment with all dependencies.

**Prerequisites:** [VS Code](https://code.visualstudio.com/) with the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension, and Docker.

1. Open the `navbim` repository root in VS Code.
2. When prompted, click **Reopen in Container** — or open the Command Palette (`Ctrl+Shift+P`) and run **Dev Containers: Reopen in Container**.
3. The first launch builds the container image and runs the workspace setup scripts (`on-create-command.sh`, `update-content-command.sh`, `post-create-command.sh`). Subsequent launches reuse the cached image and are fast.

The devcontainer mounts `/tmp/.X11-unix` and forwards `$DISPLAY` so that RViz2 renders on your host display. On Linux this works out of the box; on macOS and Windows an X server (e.g. XQuartz, VcXsrv) must be running on the host.

### Run with the Toy Example

```bash
ros2 launch navbim_bringup bringup_launch.py ifc:=Toy_example
```

`bringup_launch.py` checks whether a navigation model already exists in `navbim_bringup/nav_model/Toy_example/`. If not, it automatically runs the navigation model generator first before starting the navbim stack. The navigation model only needs to be generated once per IFC file; subsequent launches skip this step. To force regeneration, pass `force_nav_model_gen:=True`.

Use the BIM panel in RViz to select a floor and set start and goal poses by clicking on the BIM mesh. Path planning is triggered automatically when a goal pose is set.

To generate the navigation model separately without launching the full stack:

```bash
ros2 launch navbim_bringup nav_model_gen_launch.py ifc:=Toy_example
```

To launch only the path planning components without the full bringup:

```bash
ros2 launch navbim_bringup gpp_bim_launch.py ifc:=Toy_example
```

### Run with your own BIM model and robot

**Step 1: Add your IFC file**

Copy your IFC file to `navbim_bringup/bim/<your_model>.ifc`.

**Step 2: Add a robot description**

Add a URDF/xacro file for your robot to `navbim_description/urdf/<your_robot>.urdf.xacro`.

**Step 3: Adapt the parameters**

Edit `navbim_bringup/params/navbim_params.yaml` to match your robot dimensions and the desired planning resolution:

```yaml
gpp_bim:
  ros__parameters:
    robot_height: 0.75
    robot_width: 0.4
    robot_length: 0.5
    resolution: 0.05
```

**Step 4: Launch the navbim stack**

```bash
ros2 launch navbim_bringup bringup_launch.py ifc:=<your_model> robot_name:=<your_robot>
```

## Architecture

The package provides three lifecycle nodes implemented in C++.

### Node: `gpp_bim` (executable: `gpp_bim_server`)

`GppBimServer` is the top-level planning node. It implements the edge-expanding A* algorithm (level 1), orchestrates the level 2 room-by-room planning via `RoomPlannerServer`, and assembles the final path. The Boost Graph Library is used to manage the topological map.

### Node: `room_planner_server` (executable: `room_planner_server`)

`RoomPlannerServer` handles level 2 planning within a single room. It loads and manages the per-room `MultiCostmap2DROS` costmap and runs the configured planner plugin for each room segment. The following level 2 planner wrappers are available as `pluginlib` plugins of type `nav2_core::GlobalPlanner`:

| Plugin type | Description |
|---|---|
| `navbim_gpp_bim::NavFnPlannerWrapper` | Grid-based Dijkstra/A* from `nav2_navfn_planner`. |
| `navbim_gpp_bim::ThetaStarPlannerWrapper` | Any-angle planner from `nav2_theta_star_planner`. |
| `navbim_gpp_bim::OmplPlannerWrapper` | Sampling-based planners via the Open Motion Planning Library: BIT*, RRT*, Informed-RRT*, RRT-Connect, PRM*. |

### Node: `clearance_server` (executable: `clearance_server`)

`ClearanceServer` is an optional utility node that calculates path clearance metrics (minimum and average distance from obstacles) using the per-room costmaps.

## Actions

### `gpp_bim` node

| Action | Type | Description |
|---|---|---|
| `navbim_compute_path_to_pose` | `navbim_msgs/action/NavbimComputePathToPose` | Plan a globally-consistent path across rooms and floors (main entry point) |
| `pre_plan_edges` | `navbim_msgs/action/PrePlanEdges` | Precompute paths for all transition edges in the topological map |

### `room_planner_server` node

| Action | Type | Description |
|---|---|---|
| `compute_path_to_pose_in_room` | `navbim_msgs/action/NavbimComputePathToPoseInRoom` | Plan a single-room path (called internally by `gpp_bim`) |

## Services

### `room_planner_server` node

| Service | Type | Description |
|---|---|---|
| `smooth_path_with_floor_costmaps` | `navbim_msgs/srv/SmoothPathWithFloorCostmaps` | Smooth a path floor-by-floor using aggregated per-floor cost maps |

### `clearance_server` node

| Service | Type | Description |
|---|---|---|
| `calculate_path_clearance` | `navbim_msgs/srv/CalculatePathClearance` | Calculate minimum and average clearance of a path from obstacles |

## Topics Published

### `gpp_bim` node

| Topic | Type | Description |
|---|---|---|
| `planned_path` | `nav_msgs/msg/Path` | Full planned path |
| `~/pruned_topomap` | `navbim_msgs/msg/Topomap` | Pruned topological map used during the last planning request (transient local QoS) |

### `room_planner_server` node

| Topic | Type | Description |
|---|---|---|
| `plan` | `nav_msgs/msg/Path` | Path planned for the most recent single-room request |

## Parameters

### `gpp_bim` node

| Parameter | Type | Default | Description |
|---|---|---|---|
| `topomap_file` | string | `""` | Path to the topological map YAML file |
| `nav_model` | string | `""` | Path to the navigation model directory (occupancy maps) |
| `second_level_planner` | string | `"A*"` | Level 2 planner to use (`"A*"`, `"Theta*"`, `"RRT-Connect"`, etc.) |
| `use_two_level` | bool | `true` | Enable two-level planning; if false, uses level 2 only (no topological routing) |
| `reuse_paths` | bool | `true` | Reuse precomputed sub-paths stored on topological edges |
| `pre_plan_paths` | bool | `false` | Automatically precompute paths for all edges on activation |
| `force_pre_plan_of_planned_paths` | bool | `false` | Re-plan edges that already have a precomputed path |
| `prune_graph` | bool | `true` | Enable topological map pruning to remove dead ends |
| `pruning_cost_threshold` | double | `10.0` | Euclidean start–goal distance (m) below which pruning is skipped |
| `visualize_pruned_graph` | bool | `true` | Publish the pruned topological map after each planning request |
| `resolution` | double | `0.05` | Occupancy map resolution in meters |
| `penalize_z_movement` | double | `1.0` | Cost multiplier for vertical movement in the heuristic |
| `robot_height` | double | `0.75` | Robot height in meters (used for obstacle filtering) |
| `robot_width` | double | `0.4` | Robot width in meters |
| `robot_length` | double | `0.5` | Robot length in meters |
| `robot_step_height` | double | `0.0` | Maximum step height in meters; obstacles below this are passable |
| `enable_floor_smoothing` | bool | `false` | Smooth the full path floor-by-floor after planning |
| `floor_smoother_max_time` | double | `1.0` | Time budget for floor-level smoothing in seconds |

### `room_planner_server` node

| Parameter | Type | Default | Description |
|---|---|---|---|
| `planner_plugins` | string[] | `["GridBased"]` | List of level 2 planner plugin instances to load |
| `expected_planner_frequency` | double | `1.0` | Expected planning rate in Hz (used for timing warnings) |
| `costmap_update_timeout` | double | `1.0` | Timeout in seconds when waiting for a costmap update |
| `enable_room_smoothing` | bool | `true` | Smooth each room-level sub-path after planning |
| `room_smoother_type` | string | `"simple"` | Smoother to use: `"simple"` (gradient descent) or `"savitzky_golay"` |
| `room_smoother_max_time` | double | `0.5` | Time budget for room-level smoothing in seconds |

## Related Packages

- `navbim_nav_model_gen`: generates the topological map and occupancy maps from the IFC model
- `navbim_multi_costmap_2d`: provides per-room 2D cost maps used by the level 2 planners
- `navbim_topomap_server`: serves the topological map and provides topology queries
- `navbim_room_tracker`: tracks which room the robot is currently in
- `navbim_smoother`: smooths the concatenated multi-room path using floor-level cost maps
- `navbim_bt_navigator`: orchestrates navbim navigation via behavior trees

## Paper

Please cite the GPP-BIM paper if you use any of this code.

Stührenberg, J., Tandon, A. & Smarsly, K., 2026. GPP-BIM — Global path planning for robot navigation using building information models. Advanced Engineering Informatics, Volume 74, 104809.

```
@article{STUHRENBERG2026104809,
  title = {GPP-BIM — Global path planning for robot navigation using building information models},
  journal = {Advanced Engineering Informatics},
  volume = {74},
  pages = {104809},
  year = {2026},
  issn = {1474-0346},
  doi = {https://doi.org/10.1016/j.aei.2026.104809},
  url = {https://www.sciencedirect.com/science/article/pii/S147403462600501X},
  author = {Jan Stührenberg and Aditya Tandon and Kay Smarsly},
  keywords = {Global path planning, Building information modeling (BIM), Industry Foundation Classes (IFC), Robot Operating System (ROS), Topological maps, Occupancy grid maps},
}
```