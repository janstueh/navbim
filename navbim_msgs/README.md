# navbim_msgs

The `navbim_msgs` package provides the custom messages, services, and actions used across the navbim navigation stack. It complements the standard `nav_msgs` and `nav2_msgs` interfaces with types specific to BIM-based, multi-floor robot navigation.

## Messages

| Message | Description |
|---|---|
| `BoundingBox` | 3D axis-aligned bounding box |
| `CurrentRoom` | Current room ID and name the robot is located in |
| `Environment` | Description of a map environment (floor, map file, origin) |
| `Environments` | Collection of `Environment` entries |
| `Polygon` | 2D polygon in the IFC coordinate frame |
| `Topomap` | Full topological map (nodes and edges) |
| `TopomapEdge` | Edge in the topological map connecting two rooms |
| `TopomapEdges` | Collection of `TopomapEdge` entries |
| `TopomapNode` | Node in the topological map representing a waypoint or room |
| `TopomapNodes` | Collection of `TopomapNode` entries |

## Services

| Service | Description |
|---|---|
| `CalculatePathClearance` | Compute the clearance of a path from obstacles |
| `ClearTopologicalMapPaths` | Clear pre-planned paths from the topological map |
| `DumpMultiMap` | Save the currently loaded multi-floor map to disk |
| `GetAdjacentTransitionNodes` | Get transition nodes (e.g. doors) adjacent to a given room |
| `GetElementsByType` | Query all IFC elements of a given IFC type |
| `GetFloorNodes` | Get all topological map nodes on a given floor |
| `GetIfcElementInfo` | Query an IFC element by GUID; returns type, name, and pose |
| `GetMinZ` | Get the minimum Z coordinate of the current floor |
| `GetRoomByCoordinates` | Find the room containing a given 3D point |
| `GetRoomNeighbors` | Get the rooms adjacent to a given room |
| `GetRoomNodes` | Get all topological map nodes in a given room |
| `GetTopologicalMap` | Get the full topological map |
| `IsPointInRoomPolygon` | Check whether a 2D point lies within a room's polygon |
| `LoadEnvironments` | Load a set of floor environments into the multimap server |
| `LoadMultiMap` | Load a multi-floor map description from file |
| `LoadTopomap` | Load a topological map from file at runtime |
| `SaveTopologicalMap` | Save the current topological map to file |
| `SmoothPathWithFloorCostmaps` | Smooth a path using per-floor costmaps |
| `UpdateEdgeData` | Update the data (e.g. pre-planned path) on a topological edge |

## Actions

| Action | Description |
|---|---|
| `NavbimComputePathToPose` | Compute a multi-floor path to a goal pose |
| `NavbimComputePathToPoseInRoom` | Compute a path given a target room identifier |
| `PrePlanEdges` | Pre-plan metric paths along all edges of the topological map |
