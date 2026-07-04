# navbim_room_tracker

The `navbim_room_tracker` package provides a lifecycle node that continuously tracks which room of the building the robot is currently located in. This information is used by `navbim_multi_costmap_2d` to activate the correct per-room costmap and by `navbim_gpp_bim` to determine the starting room for path planning.

## Overview

The room tracker periodically queries the robot's current position via TF and calls the `topomap_server/get_room_by_coordinates` service to determine the enclosing room. The result is published as a `navbim_msgs/msg/CurrentRoom` message with transient-local QoS so that newly joining nodes receive the latest room information immediately.

## Node: `room_tracker`

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_frame` | string | `"base_link"` | TF frame of the robot |
| `global_frame` | string | `"ifc"` | Global reference frame (IFC coordinate frame) |
| `update_frequency` | double | `2.0` | Room detection rate in Hz |
| `topomap_server_timeout` | double | `5.0` | Timeout in seconds when waiting for the topological map server |

### Topics Published

| Topic | Type | Description |
|---|---|---|
| `current_room` | `navbim_msgs/msg/CurrentRoom` | Currently detected room (transient local QoS) |

### Services Used

| Service | Type | Description |
|---|---|---|
| `topomap_server/get_room_by_coordinates` | `navbim_msgs/srv/GetRoomByCoordinates` | Query which room contains a given 3D point |

## Related Packages

- `navbim_topomap_server`: provides the `get_room_by_coordinates` service
- `navbim_multi_costmap_2d`: subscribes to `current_room` to activate the matching costmap
- `navbim_gpp_bim`: uses `current_room` as the planning start room
