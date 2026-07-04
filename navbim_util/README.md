# navbim_util

The `navbim_util` package provides common utility code shared across the navbim navigation stack, in both C++ and Python.

## C++ Utilities

The C++ headers are installed under `include/navbim_util/`.

### topological_map_types.hpp

Defines the graph types used to represent the topological map throughout the stack:

- `TopologicalGraph`: a Boost Graph Library adjacency list with typed node and edge properties
- `NodeProperties`: position, floor name, room ID, and node type for a topological node
- `EdgeProperties`: connected nodes, edge type, and optional pre-planned path for a topological edge
- Auxiliary types: `Point2D`, `Polygon`, `BoundingBox`, `Position`, `Vertex`, `Edge`, `VertexIterator`, `EdgeIterator`

### topological_map.hpp

Utility functions for constructing and querying `TopologicalGraph` instances.

### node_registry.hpp

Registry utilities for managing composable node instances in the navbim stack.

## Python Utilities

The Python modules are installed as the `navbim_util` Python package.

### network_utils.py

Graph distance and cost functions for the topological map:

- `euclidean_distance_between_nodes()`: 3D Euclidean distance between two graph nodes
- `cost_between_nodes()`: weighted cost with configurable penalty for vertical movement
- Helper functions for querying room and floor information from graph edges

### point_utils.py / polygon_utils.py

2D geometric utility functions for point-in-polygon tests and polygon operations, built on [Shapely](https://shapely.readthedocs.io/).

### pose_to_tf_publisher.py

Utility for publishing a static pose as a TF transform, used during BIM alignment.

## Related Packages

- `navbim_topomap_server`: uses the C++ topological graph types from `topological_map_types.hpp`
- `navbim_nav_model_gen`: uses the Python utilities for geometry processing and graph construction
