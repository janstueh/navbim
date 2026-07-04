from typing import Dict, TYPE_CHECKING
import open3d as o3d
from .voxel_grid import VoxelGrid
from .generation_utils import ResourceMonitor
import concurrent.futures
from shapely.geometry import Polygon, Point

if TYPE_CHECKING:
    from ifc import IFCElement
    from floor import Floor
    from room import Room


# Global worker functions for multiprocessing (must be at module level)
def generate_floor_voxel_grid_worker(floor_data: Dict[str, any]) -> Dict[str, any]:
    """Worker function for parallel floor voxel grid generation."""
    floor_info = floor_data['floor_info']
    floor_name = floor_info['name']
    elements_data = floor_data['elements_data']
    navigation_model_path = floor_data['navigation_model_path']
    resolution = floor_data['resolution']
    
    result = {
        'floor_name': floor_name,
        'success': False,
        'error': None,
        'elements_processed': 0
    }
    
    try:
        # Create single VoxelGrid object for this floor
        save_path_floor_voxel_grid = f"{navigation_model_path}/{floor_name}/voxel_grid.ply"
        floor_guid_mapping_path = f"{navigation_model_path}/{floor_name}/voxel_guid_mapping.json"

        floor_voxel_grid_obj = VoxelGrid(save_path=save_path_floor_voxel_grid,
                                         resolution=resolution,
                                         guid_mapping_path=floor_guid_mapping_path)

        # Collect all points from voxel grids for the floor
        floor_points = []
        element_point_mapping = []  # Store (point, element_guid) pairs
        elements_processed = 0
        
        # Process each element
        for element_data in elements_data:
            element_guid = element_data['guid']
            voxel_grid_path = element_data['voxel_grid_path']
            guid_mapping_path = element_data['guid_mapping_path']

            # Load element's voxel grid
            element_voxel_grid_obj = VoxelGrid(save_path=voxel_grid_path, 
                                               resolution=resolution,
                                               guid_mapping_path=guid_mapping_path)
            point_cloud = element_voxel_grid_obj.get_point_cloud_from_voxel_grid()
            
            if point_cloud is not None:
                floor_points.extend(point_cloud.points)
                # Store point-to-GUID mapping for later processing
                for point in point_cloud.points:
                    element_point_mapping.append((point, element_guid))
                elements_processed += 1

        # Create voxel grid from collected points
        if floor_points:
            floor_pcd = o3d.geometry.PointCloud()
            floor_pcd.points = o3d.utility.Vector3dVector(floor_points)
            floor_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(floor_pcd, resolution)
        else:
            # Create empty voxel grid
            empty_pcd = o3d.geometry.PointCloud()
            floor_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(empty_pcd, resolution)

        # Save voxel grid to disk
        floor_voxel_grid_obj.set_voxel_grid(floor_voxel_grid)
        
        # Now map points to voxel centers and associate with GUIDs
        for point, element_guid in element_point_mapping:
            floor_voxel_grid_obj.add_guid_to_voxel_by_coordinates(point, element_guid)
        
        floor_voxel_grid_obj.save_voxel_grid()
        floor_voxel_grid_obj.save_voxel_guid_mapping()

        result['success'] = True
        result['elements_processed'] = elements_processed
        
    except Exception as e:
        result['error'] = str(e)
    
    return result


def generate_room_voxel_grid_worker(room_data: Dict[str, any]) -> Dict[str, any]:
    """Worker function for parallel room voxel grid generation."""
    room_info = room_data['room_info']
    room_id = room_info['id']
    room_name = room_info['name']
    floor_info = room_data['floor_info']
    floor_name = floor_info['name']
    elements_data = room_data['elements_data']
    navigation_model_path = room_data['navigation_model_path']
    resolution = room_data['resolution']
    vertical_tolerance = room_data['vertical_tolerance']

    result = {
        'id': room_id,
        'success': False,
        'error': None,
        'elements_processed': 0
    }
    
    try:
        # Create directory for room
        room_dir = f"{navigation_model_path}/{floor_name}/{room_name}"
        import os
        os.makedirs(room_dir, exist_ok=True)
        
        # Create single VoxelGrid object for this room
        save_path_room_voxel_grid = f"{room_dir}/voxel_grid.ply"
        room_guid_mapping_path = f"{room_dir}/voxel_guid_mapping.json"

        room_voxel_grid_obj = VoxelGrid(save_path=save_path_room_voxel_grid, 
                                        resolution=resolution, 
                                        guid_mapping_path=room_guid_mapping_path)

        # Collect all points from voxel grids for the room
        room_points = []
        element_point_mapping = []  # Store (point, element_guid) pairs
        elements_processed = 0
        
        # Process each element
        for element_data in elements_data:
            element_guid = element_data['guid']
            voxel_grid_path = element_data['voxel_grid_path']
            guid_mapping_path = element_data['guid_mapping_path']
            
            # Load element's voxel grid
            element_voxel_grid_obj = VoxelGrid(save_path=voxel_grid_path, 
                                               resolution=resolution,
                                               guid_mapping_path=guid_mapping_path)
            point_cloud = element_voxel_grid_obj.get_point_cloud_from_voxel_grid()

            if point_cloud is not None:
                # Filter points by buffered room polygon and floor height
                max_width = room_info.get('max_width', 0.2)
                polygon = room_info.get('polygon', None)
                min_z = floor_info.get('min_z', float('-inf'))
                max_z = floor_info.get('max_z', float('inf'))
                filtered_points = []

                if (max_width and polygon):
                    # Buffer based on max_width should be a multiple of the resolution + the resolution itself for safety
                    if max_width % resolution != 0:
                        max_width = (max_width // resolution + 1) * resolution
                    max_width += resolution
                    room_polygon = Polygon(polygon).buffer(max_width, join_style=2)

                    for point in point_cloud.points:
                        if (room_polygon.contains(Point(point[0], point[1])) and
                            min_z - vertical_tolerance < point[2] < max_z + vertical_tolerance):
                            filtered_points.append(point)
                else:
                    # If no polygon, use all points
                    filtered_points = list(point_cloud.points)
                
                room_points.extend(filtered_points)
                # Store point-to-GUID mapping for later processing
                for point in filtered_points:
                    element_point_mapping.append((point, element_guid))
                elements_processed += 1

        # Create voxel grid from collected points
        if room_points:
            room_pcd = o3d.geometry.PointCloud()
            room_pcd.points = o3d.utility.Vector3dVector(room_points)
            room_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(room_pcd, resolution)
        else:
            # Create empty voxel grid
            empty_pcd = o3d.geometry.PointCloud()
            room_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(empty_pcd, resolution)

        # Save voxel grid to disk
        room_voxel_grid_obj.set_voxel_grid(room_voxel_grid)
        
        # Now map points to voxel centers and associate with GUIDs
        for point, element_guid in element_point_mapping:
            room_voxel_grid_obj.add_guid_to_voxel_by_coordinates(point, element_guid)
        
        room_voxel_grid_obj.save_voxel_grid()
        room_voxel_grid_obj.save_voxel_guid_mapping()

        result['success'] = True
        result['elements_processed'] = elements_processed
        
    except Exception as e:
        result['error'] = str(e)
    
    return result


class VoxelGridManager:
    """Manager for voxel grid operations."""

    def __init__(self, resolution: float = 0.05, robot_height: float = 0.5,
                 vertical_tolerance: float = 0.3, navigation_model_path: str = None,
                 generate_floor_maps: bool = False) -> None:
        self.resolution = resolution
        self.robot_height = robot_height
        self.vertical_tolerance = vertical_tolerance
        self.navigation_model_path = navigation_model_path
        self.generate_floor_maps = generate_floor_maps

    
    def generate_voxel_grids(self, elements: Dict[str, 'IFCElement'], 
                             floors: Dict[str, 'Floor'], rooms: Dict[str, 'Room'],) -> None:
        """Generate voxel grids and the GUID mapping for all floors and rooms in parallel."""
        
        if self.generate_floor_maps:
            # Calculate optimal number of processes with floor data
            optimal_processes = ResourceMonitor.get_optimal_floor_processes(
                elements=elements, 
                floors=floors,
                resolution=self.resolution
            )

            print(f"Generating voxel grids and GUID mappings for {len(floors)} floors using {optimal_processes} parallel processes...")
            
            # Prepare floor data for parallel processing
            floor_tasks = []
            for floor_name, floor in floors.items():
                # Collect elements for this floor
                floor_elements_data = []
                for element_id, element in elements.items():
                    if (floor_name in element.floor and 
                        hasattr(element, 'voxel_grid') and element.voxel_grid is not None and
                        not element.ifc_description.is_a("IfcSpace")):
                        floor_elements_data.append({
                            'guid': element.ifc_guid,
                            'voxel_grid_path': element.voxel_grid.save_path,
                            'guid_mapping_path': element.voxel_grid.guid_mapping_path
                        })
                
                floor_tasks.append({
                    'floor_info': {
                        'name': floor_name,
                        'min_z': floor.min_z,
                        'max_z': floor.max_z
                    },
                    'elements_data': floor_elements_data,
                    'navigation_model_path': self.navigation_model_path,
                    'resolution': self.resolution
                })
            
            # Process floors in parallel
            completed_floors = 0
            total_elements_processed = 0
            
            with concurrent.futures.ProcessPoolExecutor(max_workers=optimal_processes) as executor:
                # Submit all floor tasks
                future_to_floor = {
                    executor.submit(generate_floor_voxel_grid_worker, task): task['floor_info']['name']
                    for task in floor_tasks
                }
                
                # Collect results as they complete
                for future in concurrent.futures.as_completed(future_to_floor):
                    floor_name = future_to_floor[future]
                    try:
                        result = future.result()
                        if result['success']:
                            completed_floors += 1
                            total_elements_processed += result['elements_processed']
                            
                            # Recreate VoxelGrid object and attach to floor
                            save_path = f"{self.navigation_model_path}/{floor_name}/voxel_grid.ply"
                            guid_mapping_path = f"{self.navigation_model_path}/{floor_name}/voxel_guid_mapping.json"
                            floor_voxel_grid_obj = VoxelGrid(save_path=save_path, 
                                                            resolution=self.resolution, 
                                                            guid_mapping_path=guid_mapping_path)
                            floors[floor_name].voxel_grid = floor_voxel_grid_obj
                            
                            print(f"  ✓ Completed voxel grid for floor: {floor_name} ({result['elements_processed']} elements)")
                        else:
                            print(f"  ✗ Failed to generate voxel grid for floor {floor_name}: {result['error']}")
                            
                    except Exception as e:
                        print(f"  ✗ Error processing floor {floor_name}: {e}")
        
        rooms_without_contained = {room_id: room for room_id, room in rooms.items()
                                 if not (hasattr(room, 'contained_in') and room.contained_in is not None)}
        # Process rooms only if they exist
        if rooms_without_contained is not None and len(rooms_without_contained) > 0:
            # Calculate optimal number of processes with rooms data
            optimal_processes = ResourceMonitor.get_optimal_room_processes(
                elements=elements, 
                rooms=rooms_without_contained,
                resolution=self.resolution
            )

            print(f"Generating voxel grids and GUID mappings for {len(rooms_without_contained)} rooms using {optimal_processes} parallel processes...")

            # Prepare room data for parallel processing
            room_tasks = []
            for room_id, room in rooms_without_contained.items():
                # Skip rooms that are part of combined rooms
                if hasattr(room, 'contained_in') and room.contained_in is not None:
                    continue
                room_elements_data = []
                for element_id, element in elements.items():
                    if (room_id in element.room and 
                        hasattr(element, 'voxel_grid') and element.voxel_grid is not None and
                        not element.ifc_description.is_a("IfcSpace")):
                        room_elements_data.append({
                            'guid': element.ifc_guid,
                            'voxel_grid_path': element.voxel_grid.save_path,
                            'guid_mapping_path': element.voxel_grid.guid_mapping_path
                        })
                room_tasks.append({
                    'room_info': {
                        'id': room_id,
                        'name': room.name,
                        'polygon': room.polygon,
                        'max_width': max(0.2, room.max_width_of_walls) if room.max_width_of_walls else 0.2
                    },
                    'floor_info': {
                        'name': room.floor[0],
                        'min_z': floors[room.floor[0]].min_z,
                        'max_z': floors[room.floor[0]].max_z,
                        'polygon_coords': room.polygon.exterior.coords[:] if hasattr(room, 'polygon') and room.polygon else None
                    },
                    'elements_data': room_elements_data,
                    'navigation_model_path': self.navigation_model_path,
                    'resolution': self.resolution,
                    'robot_height': self.robot_height,
                    'vertical_tolerance': self.vertical_tolerance
                })

        # Process rooms in parallel
        completed_rooms = 0
        total_elements_processed = 0
        
        with concurrent.futures.ProcessPoolExecutor(max_workers=optimal_processes) as executor:
            # Submit all room tasks
            future_to_room = {
                executor.submit(generate_room_voxel_grid_worker, task): task['room_info']['id']
                for task in room_tasks
            }
            
            # Collect results as they complete
            for future in concurrent.futures.as_completed(future_to_room):
                room_id = future_to_room[future]
                room_name = rooms[room_id].name
                floor_name = rooms[room_id].floor[0]
                try:
                    result = future.result()
                    if result['success']:
                        completed_rooms += 1
                        total_elements_processed += result['elements_processed']

                        # Recreate VoxelGrid object and attach to room
                        save_path = f"{self.navigation_model_path}/{floor_name}/{room_name}/voxel_grid.ply"
                        guid_mapping_path = f"{self.navigation_model_path}/{floor_name}/{room_name}/voxel_guid_mapping.json"
                        room_voxel_grid_obj = VoxelGrid(save_path=save_path, 
                                                        resolution=self.resolution, 
                                                        guid_mapping_path=guid_mapping_path)
                        rooms[room_id].voxel_grid = room_voxel_grid_obj

                        print(f"  ✓ Completed voxel grid for room: {room_name} ({result['elements_processed']} elements)")
                    else:
                        print(f"  ✗ Failed to generate voxel grid for room {room_name}: {result['error']}")

                except Exception as e:
                    print(f"  ✗ Error processing room {room_name}: {e}")