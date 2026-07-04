import open3d as o3d
import numpy as np
from typing import Optional, Dict, List, Tuple, TYPE_CHECKING
import os
import json
import colorsys

if TYPE_CHECKING:
    from gpp_bim.ifc import IFCElement

class VoxelGrid:
    """A wrapper class for Open3D voxel grids with disk-based storage capabilities."""
    
    def __init__(self, 
                 save_path: Optional[str] = None,
                 resolution: float = 0.05,
                 guid_mapping_path: Optional[str] = None) -> None:
        """Initialize a VoxelGrid with metadata for disk storage."""
        self.resolution = resolution
        self.save_path = save_path
        self.guid_mapping_path = guid_mapping_path
        self.voxel_grid: o3d.geometry.VoxelGrid = None
        self.voxel_guid_mapping: Dict[Tuple[int, int, int], List[str]] = {}


    def get_voxel_grid(self) -> Optional[o3d.geometry.VoxelGrid]:
        """Get the in-memory voxel grid."""
        return self.voxel_grid
    

    def set_voxel_grid(self, voxel_grid: o3d.geometry.VoxelGrid) -> None:
        """Set the in-memory voxel grid."""
        self.voxel_grid = voxel_grid
    

    def get_voxel_guid_mapping(self) -> Dict[Tuple[int, int, int], List[str]]:
        """Get the in-memory voxel GUID mapping."""
        return self.voxel_guid_mapping
    

    def set_voxel_guid_mapping(self, mapping: Dict[Tuple[int, int, int], List[str]]) -> None:
        """Set the in-memory voxel GUID mapping."""
        self.voxel_guid_mapping = mapping
    

    def save_voxel_grid(self) -> None:
        """Save the voxel grid to a file if a save path is provided."""
        if self.save_path and self.voxel_grid is not None:
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            try:
                # Save the voxel grid to the specified path
                o3d.io.write_voxel_grid(self.save_path, self.voxel_grid, 
                                        write_ascii=False, compressed=True)
            except Exception as e:
                print(f"Error saving voxel grid: {e}")
        else:
            if not self.save_path:
                print("No voxel grid save path provided.")
            if self.voxel_grid is None:
                print("No voxel grid to save.")


    def save_voxel_guid_mapping(self) -> None:
        """Save the voxel GUID mapping to a JSON file."""
        if not self.guid_mapping_path:
            return
        os.makedirs(os.path.dirname(self.guid_mapping_path), exist_ok=True)
        # Convert tuple keys to strings for JSON serialization
        serializable_mapping = {}
        for voxel_index, guid_list in self.voxel_guid_mapping.items():
            key = f"{voxel_index[0]},{voxel_index[1]},{voxel_index[2]}"
            serializable_mapping[key] = guid_list
        with open(self.guid_mapping_path, 'w') as f:
            json.dump(serializable_mapping, f, indent=2)
    
    
    def load_voxel_grid(self) -> Optional[o3d.geometry.VoxelGrid]:
        """Load the voxel grid (in-memory first, then from file if needed)."""
        if self.voxel_grid is not None:
            return self.voxel_grid
        if not self.save_path:
            print("No voxel grid save path provided.")
            return None
        if not os.path.exists(self.save_path):
            print(f"Voxel grid file does not exist: {self.save_path}")
            return None
        try:
            # Read the voxel grid from the specified path
            voxel_grid = o3d.io.read_voxel_grid(self.save_path)
            # Check if the voxel grid was successfully loaded
            if voxel_grid is None or len(voxel_grid.get_voxels()) == 0:
                print(f"No voxel data found in file {self.save_path}")
                return None
            self.voxel_grid = voxel_grid
            return voxel_grid
        except Exception as e:
            print(f"Error reading voxel grid from {self.save_path}: {e}")
            return None


    def load_voxel_guid_mapping(self) -> Dict[Tuple[int, int, int], List[str]]:
        """Load the voxel GUID mapping (in-memory first, then from file if needed)."""
        if self.voxel_guid_mapping is not None and len(self.voxel_guid_mapping) > 0:
            return self.voxel_guid_mapping
        if not self.guid_mapping_path or not os.path.exists(self.guid_mapping_path):
            return {}
        try:
            with open(self.guid_mapping_path, 'r') as f:
                serializable_mapping = json.load(f)
            # Convert string keys back to tuples of floats (world coordinates)
            voxel_guid_mapping = {}
            for key, guid_list in serializable_mapping.items():
                x, y, z = map(int, key.split(','))
                voxel_index = (x, y, z)
                voxel_guid_mapping[voxel_index] = guid_list
            self.voxel_guid_mapping = voxel_guid_mapping
            return self.voxel_guid_mapping
        except Exception as e:
            print(f"Error loading GUID mapping from {self.guid_mapping_path}: {e}")
            return {}
    

    def free_memory(self) -> None:
        """Free the in-memory voxel grid to save memory."""
        self.voxel_grid = None
        self.voxel_guid_mapping = None


    def get_voxel_center_from_index(self, voxel_index: Tuple[int, int, int]) -> Tuple[float, float, float]:
        """Get the world coordinates of the center of a voxel given its grid index."""
        if self.voxel_grid is None:
            # Load the voxel grid if not in memory
            self.load_voxel_grid()
        
        # Check if voxel grid loading was successful
        if self.voxel_grid is None:
            raise ValueError(f"Failed to load voxel grid from {self.save_path}")
            
        grid_origin = self.voxel_grid.origin
        grid_voxel_size = self.voxel_grid.voxel_size
        voxel_center_x = grid_origin[0] + (voxel_index[0] + 0.5) * grid_voxel_size
        voxel_center_y = grid_origin[1] + (voxel_index[1] + 0.5) * grid_voxel_size
        voxel_center_z = grid_origin[2] + (voxel_index[2] + 0.5) * grid_voxel_size
        return (voxel_center_x, voxel_center_y, voxel_center_z)


    def get_index_from_coordinates(self, point: Tuple[float, float, float]) -> Tuple[int, int, int]:
        """Get the voxel grid index for a given world coordinate point."""
        if self.voxel_grid is None:
            # Load the voxel grid if not in memory
            self.load_voxel_grid()
        if self.voxel_grid is None:
            raise ValueError(f"Failed to load voxel grid from {self.save_path}")
        grid_origin = self.voxel_grid.origin
        grid_voxel_size = self.voxel_grid.voxel_size
        x_idx = int((point[0] - grid_origin[0]) / grid_voxel_size)
        y_idx = int((point[1] - grid_origin[1]) / grid_voxel_size)
        z_idx = int((point[2] - grid_origin[2]) / grid_voxel_size)
        return (x_idx, y_idx, z_idx)

    
    def get_guids_at_coordinate(self, coordinate: Tuple[float, float, float]) -> List[str]:
        """Get all IFC GUIDs associated with a world coordinate."""
        if self.voxel_guid_mapping is None:
            # Load the voxel GUID mapping if not in memory
            self.load_voxel_guid_mapping()
        voxel_index = self.get_index_from_coordinates(coordinate)
        return self.get_guids_at_voxel_index(voxel_index)


    def get_guids_at_voxel_index(self, voxel_index: Tuple[int, int, int]) -> List[str]:
        """Get all IFC GUIDs associated with a voxel index."""
        if self.voxel_guid_mapping is None:
            # Load the voxel GUID mapping if not in memory
            self.load_voxel_guid_mapping()
        return self.voxel_guid_mapping.get(tuple(voxel_index), [])


    def add_guid_to_voxel_by_index(self, voxel_index: Tuple[int, int, int], guid: str) -> None:
        """Add an IFC GUID to a voxel center coordinate, supporting multiple GUIDs per voxel."""
        if self.voxel_guid_mapping is None:
            # Load the voxel GUID mapping if not in memory
            self.load_voxel_guid_mapping()
        if voxel_index not in self.voxel_guid_mapping:
            self.voxel_guid_mapping[voxel_index] = []
        if guid not in self.voxel_guid_mapping[voxel_index]:
            self.voxel_guid_mapping[voxel_index].append(guid)
    

    def add_guid_to_voxel_by_coordinates(self, coordinate: Tuple[float, float, float], guid: str) -> None:
        """Add an IFC GUID to a voxel center coordinate, supporting multiple GUIDs per voxel."""
        voxel_index = self.get_index_from_coordinates(coordinate)
        self.add_guid_to_voxel_by_index(voxel_index, guid)


    def get_point_cloud_from_voxel_grid(self) -> Optional[o3d.geometry.PointCloud]:
        """Get a point cloud with points in the middle of voxels from the voxel grid."""
        # Load voxel grid
        if self.voxel_grid is None:
            self.load_voxel_grid()
        # Check if voxel grid loading was successful
        if self.voxel_grid is None:
            return None
        # Convert voxel grid to point cloud
        points = []
        voxels = self.voxel_grid.get_voxels()
        for voxel in voxels:
            voxel_center = self.get_voxel_center_from_index(voxel.grid_index)
            points.append([voxel_center[0], voxel_center[1], voxel_center[2]])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd
    

    def create_filled_voxel_grid(self, mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.VoxelGrid:
        """
        Creates a filled voxel grid from a mesh.
        Uses ray casting to determine interior points and combines with surface voxels.
        """
        # Handle empty mesh
        if mesh.is_empty():
            empty_pcd = o3d.geometry.PointCloud()
            return o3d.geometry.VoxelGrid.create_from_point_cloud(empty_pcd, self.resolution)

        # Get the bounding box of the mesh
        bbox = mesh.get_axis_aligned_bounding_box()
        min_bound = bbox.min_bound
        max_bound = bbox.max_bound

        # Calculate aligned origin (multiple of resolution)
        aligned_origin = np.array([
            np.floor(min_bound[0] / self.resolution) * self.resolution,
            np.floor(min_bound[1] / self.resolution) * self.resolution,
            np.floor(min_bound[2] / self.resolution) * self.resolution
        ])

        # Create a surface voxel grid from the mesh with the new bounds
        surface_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh_within_bounds(
            mesh, self.resolution, aligned_origin, max_bound)

        # Get the boundary voxels
        voxels = surface_grid.get_voxels()
        if len(voxels) == 0:
            print(f"No surface voxels found in mesh")
            empty_pcd = o3d.geometry.PointCloud()
            return o3d.geometry.VoxelGrid.create_from_point_cloud(empty_pcd, self.resolution)
        
        # Get the bounding box dimensions
        voxel_indices = np.array([voxel.grid_index for voxel in voxels])
        min_indices = np.min(voxel_indices, axis=0)
        max_indices = np.max(voxel_indices, axis=0)
        
        # Create a 3D grid of voxel candidates
        x_range = range(min_indices[0], max_indices[0] + 1)
        y_range = range(min_indices[1], max_indices[1] + 1)
        z_range = range(min_indices[2], max_indices[2] + 1)
        
        # Convert mesh to a ray-castable format
        mesh_for_ray = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(mesh_for_ray)
        
        # Create sample points throughout the bounding box
        sample_points = []
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    # Convert grid index to world coordinates
                    point = [
                        surface_grid.origin[0] + x * self.resolution,
                        surface_grid.origin[1] + y * self.resolution,
                        surface_grid.origin[2] + z * self.resolution
                    ]
                    sample_points.append(point)
        
        # Convert to numpy array for processing
        points = np.array(sample_points, dtype=np.float32)
        
        # Batch ray casting to find interior points
        interior_points = []
        directions = [
            np.array([1.0, 0.0, 0.0], dtype=np.float32),
            np.array([-1.0, 0.0, 0.0], dtype=np.float32),
            np.array([0.0, 1.0, 0.0], dtype=np.float32),
            np.array([0.0, -1.0, 0.0], dtype=np.float32),
            np.array([0.0, 0.0, 1.0], dtype=np.float32),
            np.array([0.0, 0.0, -1.0], dtype=np.float32)
        ]
        
        # Create all rays for all points and all directions in one batch
        num_points = len(points)
        num_directions = len(directions)
        
        # Repeat each point for each direction
        all_points = np.repeat(points, num_directions, axis=0)  # Shape: (num_points * num_directions, 3)
        
        # Tile directions to match the repeated points
        all_directions = np.tile(directions, (num_points, 1))  # Shape: (num_points * num_directions, 3)
        
        # Create all rays at once
        all_rays = np.hstack([all_points, all_directions])  # Shape: (num_points * num_directions, 6)
        
        # Cast all rays in a single batch
        tensor_rays = o3d.core.Tensor(all_rays, dtype=o3d.core.Dtype.Float32)
        hits = scene.cast_rays(tensor_rays)
        
        # Process all hits at once
        all_hit_counts = (hits['t_hit'].numpy() != np.inf).astype(int)
        
        # Reshape hit results back to (num_points, num_directions)
        hit_counts_per_point = all_hit_counts.reshape(num_points, num_directions)
        
        # Count inside hits for each point (odd number of hits means inside)
        inside_count = np.sum(hit_counts_per_point % 2, axis=1)
        
        # Points that are inside for all ray directions are considered interior
        inside_mask = inside_count == num_directions
        interior_points = [tuple(point) for point, is_inside in zip(points, inside_mask) if is_inside]
        
        # Combine interior points with surface voxel centers
        surface_points = []
        for voxel in voxels:
            x = surface_grid.origin[0] + voxel.grid_index[0] * self.resolution
            y = surface_grid.origin[1] + voxel.grid_index[1] * self.resolution
            z = surface_grid.origin[2] + voxel.grid_index[2] * self.resolution
            surface_points.append([x, y, z])
        
        # Create a unified point cloud from both sets of points
        surface_points_array = np.array(surface_points, dtype=np.float32)
        
        if interior_points:
            # Convert interior points to properly shaped array
            interior_points_array = np.array(interior_points, dtype=np.float32)
            all_points = np.vstack([interior_points_array, surface_points_array])
        else:
            # No interior points found, use only surface points
            all_points = surface_points_array
        
        # Remove duplicates by converting to a set of tuples and back
        unique_points = np.array(list(set(map(tuple, all_points))), dtype=np.float32)
        
        # Create the unified point cloud
        combined_pcd = o3d.geometry.PointCloud()
        combined_pcd.points = o3d.utility.Vector3dVector(unique_points)
        
        # Create voxel grid from combined points, preserving the original surface grid origin
        combined_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(combined_pcd, self.resolution)
        combined_grid.origin = aligned_origin

        if self.save_path is not None:
            self.set_voxel_grid(combined_grid)
            self.save_voxel_grid()

        return combined_grid
    

    def create_colored_voxel_grid(self, 
                                  elements: Dict[str, 'IFCElement'] = None,
                                  max_z: float = float('inf')) -> o3d.geometry.VoxelGrid:
        """Create a colored version of a voxel grid for visualization based on IFC element types."""
        
        # Define element type configuration (priority, color)
        # Lower priority number = higher priority for display
        element_config = {
            'IfcDoor': {'priority': 1, 'color': [0.8, 0.6, 0.2]},           # Orange
            'IfcWindow': {'priority': 2, 'color': [0.2, 0.6, 0.9]},         # Light Blue
            'IfcWall': {'priority': 3, 'color': [0.8, 0.4, 0.2]},           # Brown
            'IfcSlab': {'priority': 4, 'color': [0.6, 0.6, 0.6]},           # Gray
            'IfcColumn': {'priority': 5, 'color': [0.4, 0.4, 0.4]},         # Dark Gray
            'IfcStair': {'priority': 6, 'color': [0.5, 0.3, 0.7]},          # Purple
            'IfcStairFlight': {'priority': 7, 'color': [0.5, 0.3, 0.7]},    # Purple
            'IfcRamp': {'priority': 8, 'color': [0.6, 0.3, 0.8]},           # Light Purple
            'IfcRampFlight': {'priority': 9, 'color': [0.6, 0.3, 0.8]},     # Light Purple
            'IfcBeam': {'priority': 10, 'color': [0.7, 0.5, 0.3]},          # Wood Brown
            'IfcRoof': {'priority': 11, 'color': [0.6, 0.2, 0.2]},          # Dark Red
            'IfcRailing': {'priority': 12, 'color': [0.9, 0.2, 0.2]},       # Red
            'IfcFurniture': {'priority': 13, 'color': [0.2, 0.8, 0.2]},     # Green
            'Unknown': {'priority': 999, 'color': [0.9, 0.9, 0.9]}          # White
        }

        # Load voxel grid
        self.load_voxel_grid()
        if self.voxel_grid is None:
            return None
        voxels = self.voxel_grid.get_voxels()
        if len(voxels) == 0:
            return self.voxel_grid

        # Load GUID mapping if not already loaded
        if not self.voxel_guid_mapping:
            self.load_voxel_guid_mapping()

        # Extract voxel positions and collect element types
        points = []
        voxel_element_types = []
        resolution = self.voxel_grid.voxel_size

        # Collect all unique element types from all voxels
        all_element_types = set()

        for voxel in voxels:
            voxel_center = self.get_voxel_center_from_index(voxel.grid_index)
            guids_at_voxel = self.get_guids_at_voxel_index(voxel.grid_index)

            # Determine element type for this voxel using priority system
            element_type = "Unknown"
            highest_priority = 999  # Start with lowest priority
            
            if guids_at_voxel and elements:
                for guid in guids_at_voxel:
                    if guid in elements:
                        element = elements[guid]
                        current_type = element.element_type if hasattr(element, 'element_type') else 'Unknown'

                        # Skip ceiling and roof elements
                        if element.is_ceiling(max_z) or element.physical_part_of_roof():
                            continue
                        
                        # Get priority for this element type (default to 14 for unlisted types)
                        current_priority = element_config.get(current_type, {'priority': 14})['priority']
                        
                        # Use this element type if it has higher priority (lower number)
                        if current_priority < highest_priority:
                            highest_priority = current_priority
                            element_type = current_type
            
            # Only add voxel if we found at least one valid element
            if element_type != "Unknown" or not guids_at_voxel or not elements:
                points.append([voxel_center[0], voxel_center[1], voxel_center[2]])
                voxel_element_types.append(element_type)
                all_element_types.add(element_type)
        
        # Create a color palette using predefined colors and generated colors for others
        color_palette = {}
        
        # First, assign predefined colors from element_config
        for element_type in all_element_types:
            if element_type in element_config:
                color_palette[element_type] = element_config[element_type]['color']
        
        # Generate colors for any remaining element types not in our predefined list
        undefined_types = [t for t in all_element_types if t not in element_config]
        if undefined_types:
            for i, element_type in enumerate(undefined_types):
                # Use HSV to create distinguishable colors for undefined types
                hue = i / max(len(undefined_types), 1)
                saturation = 0.6  # Lower saturation to distinguish from predefined colors
                value = 0.8
                
                # Convert HSV to RGB
                rgb = colorsys.hsv_to_rgb(hue, saturation, value)
                color_palette[element_type] = list(rgb)
        
        # Assign colors to each voxel based on its element type
        colors = np.zeros((len(points), 3))
        for i, element_type in enumerate(voxel_element_types):
            colors[i] = color_palette.get(element_type, [0.5, 0.5, 0.5])  # Default gray
        
        # Create colored point cloud and voxel grid
        colored_pcd = o3d.geometry.PointCloud()
        
        # Check if we have any points to process
        if len(points) == 0:
            print("Warning: No valid points found for colored voxel grid")
            # Return an empty voxel grid
            return o3d.geometry.VoxelGrid()
        
        # Convert to numpy arrays and validate
        points_array = np.array(points, dtype=np.float64)
        colors_array = np.array(colors, dtype=np.float64)
        
        # Validate array shapes
        if points_array.size == 0 or colors_array.size == 0:
            print("Warning: Empty arrays in colored voxel grid creation")
            return o3d.geometry.VoxelGrid()
        
        if points_array.shape[1] != 3:
            print(f"Warning: Invalid points shape {points_array.shape}, expected (N, 3)")
            return o3d.geometry.VoxelGrid()
        
        colored_pcd.points = o3d.utility.Vector3dVector(points_array)
        colored_pcd.colors = o3d.utility.Vector3dVector(colors_array)

        self.free_memory()
        
        return o3d.geometry.VoxelGrid.create_from_point_cloud(colored_pcd, resolution)