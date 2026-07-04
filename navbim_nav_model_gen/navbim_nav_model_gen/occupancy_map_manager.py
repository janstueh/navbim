from .occupancy_map import OccupancyMap
from .voxel_grid import VoxelGrid
from .ifc import IFCElement, IFCStairRampElement
from .room import Room
from .floor import Floor
from .generation_utils import ResourceMonitor
from navbim_util.polygon_utils import convert_shapely_to_json, convert_json_to_shapely
from shapely.geometry import Point, Polygon
from typing import Dict, List
import os
import yaml
import numpy as np
import math
from collections import deque
import concurrent.futures
from concurrent.futures import ProcessPoolExecutor


# Global worker functions for multiprocessing (must be at module level)
def process_floor_occupancy_worker(floor_data: Dict[str, any]) -> Dict[str, any]:
    """Worker function for parallel floor occupancy map processing."""
    floor_name = floor_data['floor_name']
    navigation_model_path = floor_data['navigation_model_path']
    resolution = floor_data['resolution'] 
    vertical_tolerance = floor_data['vertical_tolerance']
    robot_height = floor_data['robot_height']
    robot_step = floor_data['robot_step']
    
    result = {
        'floor_name': floor_name,
        'success': False,
        'error': None,
        'yaml_path': None
    }
    
    try:
        # Recreate manager instance
        manager = OccupancyMapManager(navigation_model_path=navigation_model_path, 
                                      resolution=resolution, 
                                      vertical_tolerance=vertical_tolerance,
                                      robot_height=robot_height,
                                      robot_step=robot_step)

        # Create floor instance with voxel grid
        voxel_grid_obj = VoxelGrid(save_path=f"{navigation_model_path}/{floor_name}/voxel_grid.ply",
                                    guid_mapping_path=f"{navigation_model_path}/{floor_name}/voxel_guid_mapping.json",
                                    resolution=resolution)
        
        # Convert floor_polygon back from JSON if it exists
        floor_data_converted = convert_json_to_shapely(floor_data)
        floor_polygon = floor_data_converted.get('floor_polygon', None)
        
        floor = Floor(floor_name=floor_name, 
                      min_z=floor_data['floor_min_z'], 
                      max_z=floor_data['floor_max_z'], 
                      voxel_grid=voxel_grid_obj,
                      polygon=floor_polygon)

        # Create IFC elements dict
        elements = manager.create_elements_from_data(floor_data['elements_data'])
        
        # Create output directory
        os.makedirs(f"{navigation_model_path}/{floor_name}", exist_ok=True)

        # Create occupancy map
        occupancy_map = manager.create_floor_occupancy_map(floor, elements)
        floor.occupancy_map = occupancy_map
        occupancy_map.save_map(f"{navigation_model_path}")

        result['yaml_path'] = f"{floor_name}/{floor_name}.yaml"
        result['success'] = True
        
    except Exception as e:
        import traceback
        result['error'] = f"{str(e)}\n{traceback.format_exc()}"
        
    return result


# Global worker functions for multiprocessing (must be at module level)
def process_room_occupancy_worker(room_data: Dict[str, any]) -> Dict[str, any]:
    """Worker function for parallel room occupancy map processing."""
    floor_name = room_data['floor_name']
    room_name = room_data['room_name']
    navigation_model_path = room_data['navigation_model_path']
    resolution = room_data['resolution'] 
    vertical_tolerance = room_data['vertical_tolerance']
    robot_height = room_data['robot_height']
    robot_step = room_data['robot_step']

    result = {
        'floor_name': floor_name,
        'room_name': room_name,
        'success': False,
        'error': None,
        'yaml_path': None
    }

    try:
        # Recreate manager instance
        manager = OccupancyMapManager(navigation_model_path=navigation_model_path, 
                                      resolution=resolution, 
                                      vertical_tolerance=vertical_tolerance,
                                      robot_height=robot_height,
                                      robot_step=robot_step)

        # Create room instance with  voxel grid
        voxel_grid_obj = VoxelGrid(save_path=f"{navigation_model_path}/{floor_name}/{room_name}/voxel_grid.ply",
                                   guid_mapping_path=f"{navigation_model_path}/{floor_name}/{room_name}/voxel_guid_mapping.json",
                                   resolution=resolution)
        room = Room(name=room_name,
                    floor=[floor_name],
                    max_width_of_walls=room_data['max_width_of_walls'],
                    polygon=room_data['polygon'],
                    voxel_grid=voxel_grid_obj)

        # Create floor instance
        floor = Floor(floor_name=floor_name,
                      min_z=room_data['floor_min_z'],
                      max_z=room_data['floor_max_z'])

        # Create IFC elements dict
        elements = manager.create_elements_from_data(room_data['elements_data'])

        # Create output directory
        os.makedirs(f"{navigation_model_path}/{floor_name}/{room_name}", exist_ok=True)

        # Create occupancy map
        room.occupancy_map = manager.create_room_occupancy_map(room=room, 
                                                               elements=elements, 
                                                               floor=floor)
        room.occupancy_map.save_map(f"{navigation_model_path}")

        result['yaml_path'] = f"{floor_name}/{room_name}/{room_name}.yaml"
        result['success'] = True
        
    except Exception as e:
        import traceback
        result['error'] = f"{str(e)}\n{traceback.format_exc()}"
        
    return result


class OccupancyMapManager:
    """Manages occupancy map-related operations."""

    def __init__(self, 
                 navigation_model_path: str, 
                 resolution: float = 0.05, 
                 vertical_tolerance: float = 0.3, 
                 robot_height: float = 0.5, 
                 robot_step: float = 0.0, 
                 generate_floor_maps: bool = True):
        self.navigation_model_path = navigation_model_path
        self.resolution = resolution
        self.vertical_tolerance = vertical_tolerance
        self.robot_height = robot_height
        self.robot_step = robot_step
        self.generate_floor_maps = generate_floor_maps
    

    def create_occupancy_maps(self, 
                              elements: Dict[str, 'IFCElement'], 
                              floors: Dict[str, 'Floor'],
                              rooms: Dict[str, 'Room']) -> Dict[str, OccupancyMap]:
        """Create occupancy maps for all floors and rooms in parallel."""

        # Create directories and prepare files
        ifc_name = os.path.basename(os.path.normpath(self.navigation_model_path))
        occupancy_maps_yaml_file = f"{self.navigation_model_path}/{ifc_name}.yaml"
        yaml_data = {}

        # Prepare yaml data
        for floor_name, floor in floors.items():
            yaml_data[floor_name] = {
                "global_frame": "ifc",
                "maps": {},
            }

        # Generate floor maps if required
        if self.generate_floor_maps:

            # Calculate optimal number of processes for occupancy mapping
            optimal_processes = ResourceMonitor.get_optimal_floor_processes(
                elements=elements, 
                floors=floors,
                resolution=self.resolution
            )

            print(f"Creating occupancy maps for {len(floors)} floors using {optimal_processes} parallel processes...")

            # Prepare floor tasks for parallel processing
            floor_tasks = []
            for floor_name, floor in floors.items():
                # Check if voxel grid exists
                try:
                    voxel_grid = floor.voxel_grid.load_voxel_grid()
                except Exception as e:
                    print(f"Error loading voxel grid file for floor {floor_name}: {e}")
                    continue

                if not voxel_grid or len(voxel_grid.get_voxels()) == 0:
                    print(f"No voxel grid for floor: {floor_name}, skipping...")
                    continue

                # Serialize elements for this floor
                elements_data = []
                for element_id, element in elements.items():
                    if ((hasattr(element, 'floor') and floor_name in element.floor) or
                        (hasattr(element, 'start_floor') and floor_name == element.start_floor) or
                        (hasattr(element, 'end_floor') and floor_name == element.end_floor)):
                        elem_data = {
                            'ifc_guid': element.ifc_guid,
                            'bbox': getattr(element, 'bbox', None),
                            'floor': element.floor,
                            'element_type': element.ifc_description.is_a() if hasattr(element, 'ifc_description') else '',
                            'polygon': element.polygon if hasattr(element, 'polygon') else None
                        }
                        if element.ifc_description.is_a() in ['IfcStair', 'IfcStairFlight', 'IfcRamp', 'IfcRampFlight']:
                            elem_data['start_floor'] = element.start_floor if hasattr(element, 'start_floor') else None
                            elem_data['end_floor'] = element.end_floor if hasattr(element, 'end_floor') else None
                            elem_data['start_polygon'] = element.start_polygon if hasattr(element, 'start_polygon') else None
                            elem_data['end_polygon'] = element.end_polygon if hasattr(element, 'end_polygon') else None
                        # Convert all Shapely geometries to JSON format for serialization
                        elem_data = convert_shapely_to_json(elem_data)
                        elements_data.append(elem_data)

                # Create task data
                task_data = {
                    'floor_name': floor_name,
                    'floor_min_z': floor.min_z,
                    'floor_max_z': floor.max_z,
                    'floor_polygon': floor.polygon,  # Add floor polygon
                    'voxel_grid_path': floor.voxel_grid.save_path,
                    'navigation_model_path': self.navigation_model_path,
                    'resolution': self.resolution,
                    'vertical_tolerance': self.vertical_tolerance,
                    'robot_height': self.robot_height,
                    'robot_step': self.robot_step,
                    'elements_data': elements_data,
                }
                # Convert Shapely geometries to JSON for serialization
                task_data = convert_shapely_to_json(task_data)
                floor_tasks.append(task_data)

            if not floor_tasks:
                print("  No valid floors to process")

            # Execute floor processing in parallel
            with ProcessPoolExecutor(max_workers=optimal_processes) as executor:
                future_to_floor = {executor.submit(process_floor_occupancy_worker, task): task['floor_name'] 
                                for task in floor_tasks}
                
                for future in concurrent.futures.as_completed(future_to_floor):
                    floor_name = future_to_floor[future]
                    result = future.result()
                    if result['success']:
                        # Merge yaml data
                        yaml_data[floor_name]['maps'][floor_name] = result['yaml_path']
                        print(f"  ✓ Created occupancy map for floor: {result['floor_name']}")
                    else:
                        print(f"  ✗ Error processing floor {result['floor_name']}: {result['error']}")

        # Get all rooms except the ones that are contained in combined rooms
        rooms_without_contained = {room_id: room for room_id, room in rooms.items()
                                  if not (hasattr(room, 'contained_in') and room.contained_in is not None)}

        # Calculate optimal number of processes for occupancy mapping
        optimal_processes = ResourceMonitor.get_optimal_room_processes(
            elements=elements, 
            rooms=rooms_without_contained,
            resolution=self.resolution)

        print(f"Creating occupancy maps for {len(rooms_without_contained)} rooms using {optimal_processes} parallel processes...")

        # Prepare room tasks for parallel processing
        room_tasks = []
        for room_id, room in rooms_without_contained.items():
            # Check if voxel grid exists
            try:
                voxel_grid = room.voxel_grid.load_voxel_grid()
            except Exception as e:
                print(f"Error loading voxel grid file for room {room.name}: {e}")
                continue

            if not voxel_grid or len(voxel_grid.get_voxels()) == 0:
                print(f"No voxel grid for room: {room.name}, skipping...")
                continue

            # Serialize elements for this room
            elements_data = []
            for element_id, element in elements.items():
                # Include room-specific elements
                room_element = hasattr(element, 'room') and room_id in element.room
                if room_element:
                    elem_data = {
                        'ifc_guid': element.ifc_guid,
                        'bbox': getattr(element, 'bbox', None),
                        'room': getattr(element, 'room', []),
                        'floor': getattr(element, 'floor', []),
                        'element_type': element.ifc_description.is_a() if hasattr(element, 'ifc_description') else '',
                        'polygon': element.polygon if hasattr(element, 'polygon') else None
                    }
                    if hasattr(element, 'ifc_description') and element.ifc_description.is_a() in ['IfcStair', 'IfcStairFlight', 'IfcRamp', 'IfcRampFlight']:
                        elem_data['start_floor'] = element.start_floor if hasattr(element, 'start_floor') else None
                        elem_data['end_floor'] = element.end_floor if hasattr(element, 'end_floor') else None
                        elem_data['start_polygon'] = element.start_polygon if hasattr(element, 'start_polygon') else None
                        elem_data['end_polygon'] = element.end_polygon if hasattr(element, 'end_polygon') else None
                    # Convert all Shapely geometries to JSON format for serialization
                    elem_data = convert_shapely_to_json(elem_data)
                    elements_data.append(elem_data)

            # Get floor information for the room
            room_floor_name = room.floor[0]
            floor_info = floors.get(room_floor_name)
            if not floor_info:
                print(f"Warning: Floor {room_floor_name} not found for room {room.name}")
                continue
                
            # Create task data
            task_data = {
                'room_id': room_id,
                'room_name': room.name,
                'floor_name': room_floor_name,
                'floor_min_z': floor_info.min_z,
                'floor_max_z': floor_info.max_z,
                'polygon': room.polygon,
                'max_width_of_walls': room.max_width_of_walls,
                'voxel_grid_path': room.voxel_grid.save_path,
                'navigation_model_path': self.navigation_model_path,
                'resolution': self.resolution,
                'vertical_tolerance': self.vertical_tolerance,
                'robot_height': self.robot_height,
                'robot_step': self.robot_step,
                'elements_data': elements_data
            }
            room_tasks.append(task_data)

        if not room_tasks:
            print("  No valid rooms to process")

        # Execute room processing in parallel
        with ProcessPoolExecutor(max_workers=optimal_processes) as executor:
            future_to_room = {executor.submit(process_room_occupancy_worker, task): task['room_id'] 
                            for task in room_tasks}

            for future in concurrent.futures.as_completed(future_to_room):
                room_id = future_to_room[future]
                room_name = rooms[room_id].name
                result = future.result()
                if result['success']:
                    # Merge yaml data
                    floor_name = result['floor_name']
                    yaml_data[floor_name]['maps'][room_name] = result['yaml_path']
                    print(f"  ✓ Created occupancy map for room: {result['room_name']}")
                else:
                    print(f"  ✗ Error processing room {result['room_name']}: {result['error']}")

        # Write the occupancy map YAML file
        with open(occupancy_maps_yaml_file, 'w') as f:
            yaml.dump(yaml_data, f)

        print(f"Occupancy map YAML file saved to {occupancy_maps_yaml_file}")
    

    def create_floor_occupancy_map(self, floor: 'Floor', elements: Dict[str, 'IFCElement']) -> 'OccupancyMap':
        """Creates an occupancy map for a single floor."""

        # Load the floor's voxel grid and guid mapping
        voxel_grid_obj = floor.voxel_grid
        voxel_grid_obj.load_voxel_grid()

        # Get all voxels of the voxel grid
        if voxel_grid_obj.voxel_grid is None or len(voxel_grid_obj.voxel_grid.get_voxels()) == 0:
            print(f"  Warning: No voxels in grid, returning empty occupancy map")
            empty_grid = np.zeros((1, 1), dtype=np.uint8)
            return OccupancyMap(
                occupancy_grid=empty_grid,
                resolution=self.resolution,
                origin=[0.0, 0.0, floor.min_z],
                floor=floor.floor_name
            )

        floor.occupancy_map = self.create_occupancy_map(voxel_grid_obj=voxel_grid_obj, 
                                                        elements=elements, 
                                                        floor=floor)

        return floor.occupancy_map


    def create_room_occupancy_map(self, room: 'Room', elements: Dict[str, 'IFCElement'],
                                  floor: 'Floor') -> 'OccupancyMap':
        """Creates an occupancy map for a room."""

        # Load the room's voxel grid and guid mapping
        voxel_grid_obj = room.voxel_grid
        voxel_grid_obj.load_voxel_grid()

        # Get all voxels of the voxel grid
        if voxel_grid_obj.voxel_grid is None or len(voxel_grid_obj.voxel_grid.get_voxels()) == 0:
            print(f"  Warning: No voxels in grid, returning empty occupancy map")
            empty_grid = np.zeros((1, 1), dtype=np.uint8)
            return OccupancyMap(
                occupancy_grid=empty_grid,
                resolution=self.resolution,
                origin=[0.0, 0.0, floor.min_z],
                floor=floor.floor_name,
                room=room.name
            )

        room.occupancy_map = self.create_occupancy_map(voxel_grid_obj=voxel_grid_obj, 
                                                       elements=elements, 
                                                       floor=floor, 
                                                       room=room)
        room.occupancy_map = self.bfs_dilation(room)

        return room.occupancy_map


    def create_occupancy_map(self, voxel_grid_obj: 'VoxelGrid', elements: Dict[str, 'IFCElement'],
                             floor: 'Floor', room: 'Room' = None) -> 'OccupancyMap':

        # Load and get all voxels
        voxel_grid = voxel_grid_obj.load_voxel_grid()
        voxel_grid_obj.load_voxel_guid_mapping()

        if voxel_grid is None:
            raise ValueError(f"Failed to load voxel grid from {voxel_grid_obj.save_path}")
        
        voxels = voxel_grid.get_voxels()

        # Extract max indices for grid sizing
        voxel_indices = np.array([voxel.grid_index for voxel in voxels])
        max_indices = np.max(voxel_indices, axis=0)

        # Create a 2D grid of the proper size  
        grid_size_x = max_indices[0] + 1
        grid_size_y = max_indices[1] + 1
        
        # Initialize the occupancy grid (0 = occupied by default)
        occupancy_grid = np.zeros((grid_size_y, grid_size_x), dtype=np.uint8)

        def mark_stair_polygon_cells(polygon: Polygon, origin: list, stair_start_end_cells: set) -> None:
            """Mark cells within stair/ramp polygons."""
            if not polygon:
                return
                
            # Get polygon bounds
            minx, miny, maxx, maxy = polygon.bounds
            
            # Convert world coordinates to grid indices
            grid_minx = max(0, int((minx - origin[0]) / self.resolution))
            grid_miny = max(0, int((miny - origin[1]) / self.resolution))
            grid_maxx = min(occupancy_grid.shape[1] - 1, int((maxx - origin[0]) / self.resolution))
            grid_maxy = min(occupancy_grid.shape[0] - 1, int((maxy - origin[1]) / self.resolution))

            # Check each cell in the bounding box
            for y in range(grid_miny, grid_maxy + 1):
                for x in range(grid_minx, grid_maxx + 1):
                    # Calculate the four corners of the cell
                    cell_left = origin[0] + x * self.resolution
                    cell_bottom = origin[1] + y * self.resolution
                    cell_right = origin[0] + (x + 1) * self.resolution
                    cell_top = origin[1] + (y + 1) * self.resolution

                    # Create corner points
                    corners = [
                        Point(cell_left, cell_bottom),    # Bottom-left
                        Point(cell_right, cell_bottom),   # Bottom-right
                        Point(cell_right, cell_top),      # Top-right
                        Point(cell_left, cell_top)        # Top-left
                    ]
                    
                    # Count how many corners are inside the polygon
                    corners_inside = sum(1 for corner in corners if polygon.contains(corner))

                    # Mark in stair_start_end_cells if any corner is inside
                    if corners_inside > 0:
                        stair_start_end_cells.add((x, y))

        slab_cells = set()
        obstacle_cells = set()
        door_cells = set() # Do not include doors in occupancy maps
        stair_ramp_obstacle_cells = set()  # Track stairs/ramps separately
        stair_start_end_cells = set()  # Cells at start/end of stairs/ramps

        # Collect stair/ramp GUIDs for identification
        stair_ramp_guids = set()
        if elements:
            for guid, element in elements.items():
                if isinstance(element, IFCStairRampElement):
                    stair_ramp_guids.add(element.ifc_guid)
        
        # If we are unlucky, doors may be elevated when floor construction is missing in the BIM model
        # We need to check for the min_z of the doors on this floor to avoid blocking them in the OGM
        doors_elevated = 0.0
        for guid, element in elements.items():
            if (element.is_door() and 
                floor.floor_name in element.floor and 
                floor.min_z < element.bbox["min_z"] < floor.min_z + self.vertical_tolerance):
                # Do not include outliers above vertical tolerance
                doors_elevated = max(doors_elevated, element.bbox["min_z"] - floor.min_z)
        if doors_elevated > 0.0:
            # Increase to multiple of resolution
            doors_elevated = math.ceil(doors_elevated / self.resolution) * self.resolution

        # Process all voxels and categorize them using GUID mapping
        for voxel in voxels:
            x = voxel.grid_index[0]
            y = voxel.grid_index[1] 
            z_world = voxel_grid.origin[2] + (voxel.grid_index[2] * voxel_grid.voxel_size)
            
            if (0 <= x < grid_size_x and 
                0 <= y < grid_size_y and 
                floor.min_z - self.vertical_tolerance < z_world < floor.min_z + doors_elevated + self.robot_height):
                # Get GUIDs for this voxel
                guids_at_voxel = voxel_grid_obj.get_guids_at_voxel_index(voxel.grid_index)

                # Check if this voxel contains floor slab or stair/ramp elements
                if not guids_at_voxel:
                    continue
                for guid in guids_at_voxel:
                    if not guid in elements:
                        continue
                    element = elements[guid]
                    # Check if it's a walkable floor
                    if element.is_walkable(floor.min_z):
                        slab_cells.add((x, y))
                    # Exclude doors
                    elif element.is_door():
                        door_cells.add((x, y))
                    # Handle stairs/ramps separately
                    elif guid in stair_ramp_guids and z_world >= floor.min_z + self.robot_step:
                        stair_ramp_obstacle_cells.add((x, y))
                    # Mark as obstacle if above robot step height
                    elif z_world > floor.min_z + doors_elevated + self.robot_step:
                        obstacle_cells.add((x, y))

        # Find start and end stair cells
        # Iterate over stair_ramp_guids and check their polygons
        for guid in stair_ramp_guids:
            stair_element = elements.get(guid)
            if not stair_element:
                continue
            start_floor = getattr(stair_element, 'start_floor', None)
            end_floor = getattr(stair_element, 'end_floor', None)
            start_polygon = getattr(stair_element, 'start_polygon', None)
            end_polygon = getattr(stair_element, 'end_polygon', None)
            if start_floor == floor.floor_name:
                start_polygon = getattr(stair_element, 'start_polygon', None)
                if start_polygon:
                    mark_stair_polygon_cells(start_polygon, voxel_grid.origin, stair_start_end_cells)
            if end_floor == floor.floor_name:
                end_polygon = getattr(stair_element, 'end_polygon', None)
                if end_polygon:
                    mark_stair_polygon_cells(end_polygon, voxel_grid.origin, stair_start_end_cells)
                        
        # Apply occupancy rules: 
        # - Cells with obstacles above robot step height are occupied
        # - Cells with slabs (floors) below and no obstacles are free
        # - Cells without slabs are occupied (no floor to walk on)
        # - Cells with doors are free
        # - Cells at start/end of stairs/ramps are free, when there is no obstacle
        # - Cells outside the floor and room polygon are unknown
        
        # Create combined polygon for boundary checking (if available)
        inside_polygon = None
        if floor.polygon is not None:
            inside_polygon = floor.polygon.union(room.polygon if room and room.polygon else floor.polygon)
        elif room and room.polygon is not None:
            inside_polygon = room.polygon
        
        # Track cell type counts for debugging
        cells_outside_polygon = 0
        cells_no_floor = 0
            
        for x in range(grid_size_x):
            for y in range(grid_size_y):
                # Convert grid coordinates to world coordinates (cell center) for polygon checking
                world_x = voxel_grid.origin[0] + (x + 0.5) * self.resolution
                world_y = voxel_grid.origin[1] + (y + 0.5) * self.resolution
                point = Point(world_x, world_y)
                
                if (x, y) in obstacle_cells:
                    occupancy_grid[y, x] = 0  # Occupied (obstacle)
                elif (x, y) in door_cells:
                   occupancy_grid[y, x] = 255  # Free (door opening)
                elif (x, y) in slab_cells and (x, y) not in obstacle_cells and (x, y) not in stair_ramp_obstacle_cells:
                    occupancy_grid[y, x] = 255  # Free (walkable floor)
                elif (x, y) in stair_start_end_cells and (x, y) not in obstacle_cells:
                    occupancy_grid[y, x] = 255  # Free (stair/ramp)
                elif inside_polygon is not None and not inside_polygon.contains(point):
                    occupancy_grid[y, x] = 100  # Unknown (outside floor/room boundary)
                    cells_outside_polygon += 1
                else:
                    occupancy_grid[y, x] = 0  # Occupied (no floor to walk on)
                    cells_no_floor += 1
        
        origin = [round(float(voxel_grid.origin[0]), 3), 
                  round(float(voxel_grid.origin[1]), 3), 
                  round(float(floor.min_z), 3)]
        
        # Create the occupancy map object
        occupancy_map = OccupancyMap(
            occupancy_grid=occupancy_grid,
            resolution=self.resolution,
            origin=origin,
            floor=floor.floor_name,
            room=room.name if room and hasattr(room, 'name') else None
        )

        return occupancy_map
    

    def bfs_dilation(self, room: 'Room') -> 'OccupancyMap':
        """Perform BFS dilation on the occupancy grid of a room to mark unreachable areas."""
        # Check if polygon exists
        if not hasattr(room, 'polygon') or room.polygon is None:
            print(f"    Warning: No polygon found for room {room.name}, returning original occupancy map")
            return room.occupancy_map
            
        # Mark pixels inside the polygon and create buffered polygon mask
        occupancy_grid = room.occupancy_map.occupancy_grid
        polygon = room.polygon
        marked_grid = np.zeros((occupancy_grid.shape[0], occupancy_grid.shape[1]), dtype=np.uint8)
        inside_buffered_polygon = np.zeros((occupancy_grid.shape[0], occupancy_grid.shape[1]), dtype=bool)
        pixels_inside_polygon = 0
        
        # Buffer the polygon by max wall width to get the full area of interest
        # Max_width should be a multiple of the resolution
        max_width = room.max_width_of_walls
        if max_width % room.occupancy_map.resolution != 0:
            max_width = (max_width // room.occupancy_map.resolution + 1) * room.occupancy_map.resolution
        buffer_distance = max_width
        buffered_polygon = polygon.buffer(buffer_distance, join_style=2)
        
        for y in range(occupancy_grid.shape[0]):
            for x in range(occupancy_grid.shape[1]):
                point = Point(room.occupancy_map.origin[0] + (x + 0.5) * room.occupancy_map.resolution, 
                              room.occupancy_map.origin[1] + (y + 0.5) * room.occupancy_map.resolution)
                
                # Check if inside buffered polygon
                if buffered_polygon.contains(point):
                    inside_buffered_polygon[y, x] = True
                
                # Mark free cells inside original polygon as starting points for BFS
                if polygon.contains(point) and occupancy_grid[y, x] == 255:
                    marked_grid[y, x] = 1
                    pixels_inside_polygon += 1
        
        if pixels_inside_polygon == 0:
            print(f"    Warning: No free pixels found inside polygon for {room.name}")
            return room.occupancy_map

        buffer_pixels = math.ceil(room.max_width_of_walls / room.occupancy_map.resolution) + 1

        # Create distance map for proper dilation
        distance_map = np.full(marked_grid.shape, np.inf, dtype=np.float32)
        
        # Initialize queue with all room pixels at distance 0
        queue = deque()
        for y in range(marked_grid.shape[0]):
            for x in range(marked_grid.shape[1]):
                if marked_grid[y, x] == 1:
                    distance_map[y, x] = 0.0
                    queue.append((x, y, 0.0))

        # Perform BFS dilation with distance tracking
        while queue:
            x, y, current_distance = queue.popleft()
            
            # Skip if we've already found a shorter path to this cell
            if current_distance > distance_map[y, x]:
                continue
                
            # Add neighbors to the queue
            for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                ny, nx = y + dy, x + dx
                if (0 <= ny < marked_grid.shape[0] and 
                    0 <= nx < marked_grid.shape[1]):
                    
                    # Calculate new distance (Manhattan distance for 4-connectivity)
                    new_distance = current_distance + 1.0
                    
                    # Only process if this is a shorter path and within buffer range
                    if (new_distance < distance_map[ny, nx] and 
                        new_distance <= buffer_pixels and
                        occupancy_grid[ny, nx] == 255):  # Only expand into free space
                        
                        distance_map[ny, nx] = new_distance
                        queue.append((nx, ny, new_distance))
        
        # Create grown grid from distance map (this is the buffered polygon)
        grown_grid = (distance_map <= buffer_pixels).astype(np.uint8)
        
        # Initialize everything as unknown (100)
        final_grid = np.full_like(occupancy_grid, 100, dtype=np.uint8)
        
        # Apply rules based on buffered polygon boundary:
        # 1. Outside buffered polygon → unknown (100) - already set
        # 2. Inside buffered polygon + reached by BFS → free (255)
        # 3. Inside buffered polygon + obstacle → real obstacle (0)
        # 4. Inside buffered polygon + not reached by BFS → unknown (100) - behind walls
        
        # Mark reachable free space as free
        final_grid[(grown_grid == 1) & (occupancy_grid == 255)] = 255
        
        # Mark real obstacles inside buffered polygon as occupied
        final_grid[inside_buffered_polygon & (occupancy_grid == 0)] = 0

        room.occupancy_map.occupancy_grid = final_grid

        return room.occupancy_map


    def create_elements_from_data(self, elements_data: List[Dict[str, any]]) -> Dict[str, IFCElement]:
        """Create element dicts with IFCElements or IFCStairRampElements from serialized data."""
        elements = {}
        for elem in elements_data:
            # Convert JSON geometries back to Shapely objects
            elem = convert_json_to_shapely(elem)
            
            if elem.get('element_type', '') in ['IfcStair', 'IfcStairFlight', 'IfcRamp', 'IfcRampFlight']:
                elements[elem['ifc_guid']] = IFCStairRampElement(ifc_guid=elem['ifc_guid'],
                                                                  bbox=elem.get('bbox'),
                                                                  floor=elem.get('floor', []),
                                                                  element_type=elem.get('element_type', ''),
                                                                  polygon=elem.get('polygon'),
                                                                  start_floor=elem.get('start_floor', None),
                                                                  end_floor=elem.get('end_floor', None),
                                                                  start_polygon=elem.get('start_polygon'),
                                                                  end_polygon=elem.get('end_polygon'))
            else:
                elements[elem['ifc_guid']] = IFCElement(ifc_guid=elem['ifc_guid'],
                                                        bbox=elem.get('bbox'),
                                                        floor=elem.get('floor', []),
                                                        element_type=elem.get('element_type', ''),
                                                        polygon=elem.get('polygon'))
        return elements