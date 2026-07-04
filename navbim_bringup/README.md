# navbim_bringup

The `navbim_bringup` package provides launch files, configuration parameters, and IFC-based BIM models for bringing up the full navbim navigation stack.

## Launch Files

| Launch File | Description |
|---|---|
| `bringup_launch.py` | Top-level launch file for the full navbim stack |
| `bim_server_launch.py` | Launches the `navbim_bim_server` node |
| `bim_alignment_launch.py` | Launches nodes for aligning the BIM model to the robot's coordinate frame |
| `gpp_bim_launch.py` | Launches `gpp_bim`, `room_planner_server`, `clearance_server` (all from `navbim_gpp_bim`) and `bt_navigator` |
| `map_infrastructure_launch.py` | Launches the multimap server and multi-costmap nodes |
| `nav_model_gen_launch.py` | Launches the navigation model generator |
| `robot_launch.py` | Launches robot description and state publisher nodes |
| `rviz_launch.py` | Launches RViz with the navbim default configuration |

## Usage

To bring up the full navbim stack with the Toy Example:

```bash
ros2 launch navbim_bringup bringup_launch.py ifc:=Toy_example
```

`bringup_launch.py` uses the `ifc` argument to locate the IFC file at `bim/<ifc>.ifc` and the navigation model at `nav_model/<ifc>/`. If the navigation model does not yet exist it is generated automatically before starting the stack. To specify paths explicitly:

```bash
ros2 launch navbim_bringup bringup_launch.py \
  ifc_file:=/path/to/building.ifc \
  nav_model:=/path/to/nav_model/ \
  robot_name:=<your-robot>
```

## Configuration

The main parameter file is `params/navbim_params.yaml`. It configures all navbim nodes. The `rviz/` directory contains RViz configuration files for visualizing the navbim stack.
