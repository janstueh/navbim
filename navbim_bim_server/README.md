# navbim_bim_server

The `navbim_bim_server` package provides a ROS 2 lifecycle node that loads an IFC (Industry Foundation Classes) BIM file and exposes its geometric and semantic element data as ROS 2 services. It is the primary interface between the BIM model and the navbim navigation stack.

## Overview

BIM models in IFC format contain rich semantic and geometric information about a building's structure and contents, including walls, doors, rooms, slabs, and installed equipment. The `bim_server` node makes this information queryable at runtime via ROS 2 service interfaces.

## Node: `bim_server`

The `bim_server` is implemented as a `rclpy.lifecycle.LifecycleNode`. The IFC file is parsed on `on_configure()` using [ifcopenshell](https://ifcopenshell.org/).

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ifc_file` | string | `""` | Absolute path to the IFC file to load |

### Services

| Service | Type | Description |
|---|---|---|
| `bim_server/get_element_info` | `navbim_msgs/srv/GetIfcElementInfo` | Query an IFC element by its GUID; returns type, name, and pose |
| `bim_server/get_elements_by_type` | `navbim_msgs/srv/GetElementsByType` | Query all IFC elements of a given IFC type (e.g. `IfcDoor`) |

## Usage

The server is launched as part of the navbim bringup. To launch it standalone:

```bash
ros2 launch navbim_bringup bim_server_launch.py ifc_file:=/path/to/model.ifc
```

## Related Packages

- `navbim_nav_model_gen`: consumes the IFC file offline to generate the navigation model
- `navbim_rviz_plugins`: uses `get_element_info` to visualize BIM elements in RViz
- `navbim_msgs`: message and service definitions
