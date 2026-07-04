# navbim_topomap_server

The `navbim_topomap_server` package provides a lifecycle node that loads, serves, and optionally saves a topological map of the building. The topological map is a graph in which nodes represent waypoints and rooms, and edges represent navigable connections between adjacent rooms such as open passages, doors, ramps, and stairways.

## Overview

The topological map provides the coarse, room-level navigation layer used by `navbim_gpp_bim` to find a room-to-room route before metric path planning within each room. It is generated offline from the IFC model by `navbim_nav_model_gen` and loaded at startup.

## Node: `topomap_server`

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `topomap_file` | string | `""` | Path to the topological map YAML file |
| `global_frame` | string | `"ifc"` | Reference frame for node coordinates |
| `save_at_shutdown` | bool | `false` | Whether to write the map back to disk on shutdown |

### Services

| Service | Type | Description |
|---|---|---|
| `topomap_server/get_floor_nodes` | `navbim_msgs/srv/GetFloorNodes` | Get all topological nodes on a given floor |
| `topomap_server/get_room_nodes` | `navbim_msgs/srv/GetRoomNodes` | Get all topological nodes in a given room |
| `topomap_server/get_room_neighbors` | `navbim_msgs/srv/GetRoomNeighbors` | Get rooms adjacent to a given room |
| `topomap_server/get_adjacent_transition_nodes` | `navbim_msgs/srv/GetAdjacentTransitionNodes` | Get transition nodes (e.g. door waypoints) between a room and its neighbors |
| `topomap_server/get_room_by_coordinates` | `navbim_msgs/srv/GetRoomByCoordinates` | Find the room containing a given 3D position |
| `topomap_server/get_topological_map` | `navbim_msgs/srv/GetTopologicalMap` | Get the full topological map |
| `topomap_server/load_topomap` | `navbim_msgs/srv/LoadTopomap` | Load a topological map from file at runtime |
| `topomap_server/save_topological_map` | `navbim_msgs/srv/SaveTopologicalMap` | Save the current topological map to file |
| `topomap_server/update_edge_data` | `navbim_msgs/srv/UpdateEdgeData` | Update data stored on a topological edge (e.g. a pre-planned path) |
| `topomap_server/clear_topological_map_paths` | `navbim_msgs/srv/ClearTopologicalMapPaths` | Clear pre-planned paths from all edges |
| `topomap_server/get_min_z` | `navbim_msgs/srv/GetMinZ` | Get the minimum Z coordinate within the topological map (used for floor height queries) |
| `topomap_server/is_point_in_room_polygon` | `navbim_msgs/srv/IsPointInRoomPolygon` | Check whether a 2D point lies within a room's boundary polygon |

## Map Format

The topological map is stored in YAML format. Three node types and three edge types are used:

**Node types:**

| Type | Description | Source |
|---|---|---|
| Floor node | Represents a building storey | `IfcBuildingStorey` |
| Room node | Represents a room or space | `IfcSpace` |
| Transition node | Represents a passage point (door midpoint, stair/ramp endpoint) | `IfcDoor`, `IfcStair`, `IfcRamp` |

**Edge types:**

| Type | Connects | Description |
|---|---|---|
| Floor edge | Floor node → Room node | Associates rooms with their floor |
| Room edge | Room node → Transition node | Associates transitions with their room |
| Transition edge | Transition node → Transition node | Represents a traversable path between two transitions in the same room |

**Node attributes:** `node_id`, `ifc_guid`, `position` (3D), `polygon` (boundary), `min_z`, `max_z`, `room_id`, `subtype` (e.g. `"door"`, `"stair"`, `"start"`, `"goal"`)

**Edge attributes:** `edge_id`, `room_id`, `distance`, `cost`, `path` (optional precomputed waypoints)

## Related Packages

- `navbim_nav_model_gen`: generates the topological map from the IFC model
- `navbim_room_tracker`: uses `get_room_by_coordinates` to track the robot's current room
- `navbim_gpp_bim`: uses the topological graph for room-level route planning
- `navbim_util`: provides the C++ graph types used to represent the topological map
