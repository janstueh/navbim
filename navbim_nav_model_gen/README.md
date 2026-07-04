# navbim_nav_model_gen

The `navbim_nav_model_gen` package generates the navigation model for the navbim stack offline from an IFC-based BIM file. The navigation model consists of a topological map (level 1) and a set of per-room occupancy grid maps (level 2), which together provide the spatial representation needed by `navbim_gpp_bim` for multi-floor path planning.

## Overview

Standard robot navigation requires pre-recorded occupancy grid maps. In BIM-based navigation, these maps are derived automatically from the building's IFC model, eliminating the need for manual map recording. The generator processes IFC geometry to produce both topological and metric representations of the building.

## Navigation Model Generation

### Step 1: Topological map generation

The topological map is a graph extracted from IFC semantic data. Three node types and three edge types are created:

| Node / Edge type | Source IFC entity |
|---|---|
| Floor node | `IfcBuildingStorey` |
| Room node | `IfcSpace`, `IfcRelSpaceBoundary` |
| Transition node (door) | `IfcDoor` |
| Transition node (stair/ramp) | `IfcStair`, `IfcRamp` |
| Floor edge | connects floor node to its room nodes |
| Room edge | connects room node to its transition nodes |
| Transition edge | connects transition nodes in the same room |

Each node stores a position, room ID, IFC GUID, bounding polygon, and height range. Each edge stores the pair of connected transition nodes and an optional precomputed metric path.

For each floor, the `IfcConvert` tool from IfcOpenShell is used to export individual meshes of `IfcBuiltElement` elements for visualization in the BIM panel.

### Step 2: Occupancy map generation

Per-room occupancy grid maps are generated from the IFC geometry in three sub-steps:

1. **Voxel grid generation**: A 3D voxel grid is built for each room at a configurable resolution δ using the IFC element meshes (via Open3D). Voxel interiors are filled by ray tracing. IFC element types are tracked via their GUIDs for use in step 2. Voxel grids are stored as `.ply` files.
2. **Occupancy map generation**: The voxel grid is collapsed to a 2D occupancy grid map. Cells without a supporting `IfcSlab` or `IfcCovering` element (floor) below them are marked as obstacles. Cells with any occupied voxel between the floor height and the floor height plus the agent height are marked as obstacles. A configurable step height filters out obstacles below that height. Stair and ramp flight endpoints are marked as free to ensure reachability. A breadth-first search from outside the room polygon boundary marks unreachable cells as unknown (preventing cost inflation). Occupancy maps are stored as `.png` + `.yaml` files.
3. **Cost map generation**: Cost maps are inflated versions of the occupancy maps. Cells within half the robot width from obstacles are marked with infinite cost (inscribed zone); cells further away are inflated using an exponential decay function with a configurable cost scaling factor. For path smoothing, per-room cost maps are aggregated into per-floor cost maps.

## Node: `nav_model_gen`

The generator runs as a plain ROS 2 node (not a lifecycle node). Parameters are declared by the `GeneratorManager` class.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ifc_file` | string | `""` | Absolute path to the input IFC file |
| `nav_model` | string | `""` | Output directory for the generated navigation model |
| `resolution` | double | `0.05` | Voxel and occupancy map resolution in meters |
| `robot_height` | double | `0.5` | Robot height in meters; voxels above floor + height are treated as obstacles |
| `robot_step_height` | double | `0.0` | Maximum step height in meters; obstacles below this are passable |
| `2D` | bool | `true` | Restrict occupancy maps to 2D (required for ground robots) |
| `generate_floor_maps` | bool | `true` | Aggregate per-room maps into per-floor maps |
| `delete_previous_maps` | bool | `false` | Delete previously generated maps in the output directory before generating |
| `export_meshes` | bool | `true` | Export IFC element meshes for RViz visualization |
| `mesh_export_format` | string | `".dae"` | Mesh export format (e.g. `".dae"`, `".obj"`) |
| `min_area` | double | `2.0` | Minimum room area in m²; rooms smaller than this are skipped |
| `vertical_tolerance` | double | `0.3` | Vertical tolerance in meters when assigning elements to floors |
| `penalize_z_movement` | double | `3.0` | Cost multiplier for vertical movement in the topological map edge cost |
| `visualize_generation_of_floors` | bool | `false` | Show per-floor voxel generation visualizations (requires display) |
| `visualize_generation_of_rooms` | bool | `false` | Show per-room voxel generation visualizations (requires display) |

## Usage

The generator can be run as a standalone process or launched via `navbim_bringup`:

```bash
ros2 launch navbim_bringup nav_model_gen_launch.py ifc:=Toy_example
```

To specify paths explicitly:

```bash
ros2 launch navbim_bringup nav_model_gen_launch.py \
  ifc_file:=/path/to/building.ifc \
  nav_model:=/path/to/nav_model/
```

## Output

The generator writes the navigation model to the specified output directory:

- `<room_id>.ply`: voxel grid for each room (for visualization and debugging)
- `<room_id>.png` / `<room_id>.yaml`: occupancy grid map for each room
- `topomap.yaml`: topological map loaded by `navbim_topomap_server`
- `environments.yaml`: floor environment descriptions loaded by `navbim_multimap_server`

## Dependencies

The generator uses the following Python packages:

- `ifcopenshell`: IFC parsing and mesh export via IfcConvert
- `open3d`: 3D voxel grid generation and ray tracing
- `shapely`: polygon operations and room boundary computation
- `networkx`: topological graph construction

## Related Packages

- `navbim_bim_server`: serves IFC element data at runtime
- `navbim_topomap_server`: loads and serves the generated topological map
- `navbim_multimap_server`: loads and serves the generated floor maps
- `navbim_multi_costmap_2d`: builds per-room cost maps from the floor maps
