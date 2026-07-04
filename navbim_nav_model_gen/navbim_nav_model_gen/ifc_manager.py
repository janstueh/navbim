from .ifc import IFCElement, IFCStairRampElement
from .voxel_grid import VoxelGrid
from .generation_utils import print_progress_bar, ResourceMonitor
import ifcopenshell
import ifcopenshell.geom
import numpy as np
import open3d as o3d
from shapely.geometry import Polygon
from typing import Dict, List, Optional, Any
import concurrent.futures
import multiprocessing
from dataclasses import dataclass


@dataclass
class ProcessingConfig:
    """Configuration for IFC element processing."""
    navigation_model_path: str
    export_meshes: bool
    mesh_export_format: str
    ifc_file_path: str
    resolution: float


class GeometryProcessor:
    """Static methods for processing IFC element geometry without creating full objects."""
    
    @staticmethod
    def process_element_batch(elements_batch: List[Dict[str, str]], 
                            config: ProcessingConfig) -> Dict[str, Dict[str, Any]]:
        """Process a batch of IFC elements and return serializable data."""
        
        # Open IFC file in this worker process
        try:
            ifc_model = ifcopenshell.open(config.ifc_file_path)
        except Exception as e:
            print(f"Error opening IFC file in worker process: {e}")
            return {}
        
        processed_elements_data = {}
        
        for element_data in elements_batch:
            global_id = element_data['global_id']
            element_type = element_data['element_type']
                
            try:
                # Get the actual element from the IFC model
                element = ifc_model.by_guid(global_id)
                if element is None:
                    continue
                
                # Skip certain element types from geometry processing
                if element.is_a("IfcBuildingStorey") or element.is_a("IfcRelSpaceBoundary"):
                    # For these types, just store basic info
                    element_data_out = {
                        'element_type': element_type,
                        'ifc_guid': global_id,
                        'vertices': None,
                        'faces': None,
                        'bbox': None,
                        'polygon': None,
                        'position': None,
                        'floor': [],  # Initialize as empty list for floor assignment
                        'mesh_save_path': GeometryProcessor.get_mesh_path(global_id, config) if element_type != "IfcSpace" else None,
                        'voxel_grid_save_path': GeometryProcessor.get_voxel_path(global_id, config) if element_type != "IfcSpace" else None,
                        'resolution': config.resolution,
                        'ifc_file_path': config.ifc_file_path
                    }
                else:
                    # Process geometry using standalone functions (the heavy work)
                    geometry_data = GeometryProcessor.extract_shape_data(element, element_type)
                    
                    # Create voxel grid if we have valid geometry (even if polygon failed)
                    voxel_grid_path = None
                    if (element_type != "IfcSpace" and
                        geometry_data['vertices'] is not None and
                        geometry_data['faces'] is not None and
                        len(geometry_data['vertices']) > 0 and
                        len(geometry_data['faces']) > 0):
                        voxel_grid_path = GeometryProcessor.get_voxel_path(global_id, config)
                        try:
                            GeometryProcessor.create_voxel_grid(
                                geometry_data['vertices'],
                                geometry_data['faces'],
                                voxel_grid_path,
                                config.resolution
                            )
                        except Exception as e:
                            print(f"  Error creating voxel grid for {global_id}: {e}")
                            voxel_grid_path = None
                    
                    # Prepare final data structure
                    element_data_out = {
                        'element_type': element_type,
                        'ifc_guid': global_id,
                        'vertices': geometry_data['vertices'],
                        'faces': geometry_data['faces'],
                        'bbox': geometry_data['bbox'],
                        'polygon': geometry_data['polygon'],
                        'position': geometry_data['position'],
                        'floor': [],  # Initialize as empty list for floor assignment
                        'mesh_save_path': GeometryProcessor.get_mesh_path(global_id, config) if element_type != "IfcSpace" else None,
                        'voxel_grid_save_path': voxel_grid_path,
                        'resolution': config.resolution,
                        'ifc_file_path': config.ifc_file_path
                    }
                
                processed_elements_data[global_id] = element_data_out
                
            except Exception as e:
                print(f"Error processing element {global_id} of type {element_type}: {e}")
        
        return processed_elements_data
    
    @staticmethod
    def extract_shape_data(ifc_element, element_type: str) -> Dict[str, Any]:
        """Extract geometry data from IFC element - the core heavy computation."""
        result = {
            'vertices': None,
            'faces': None,
            'bbox': None,
            'polygon': None,
            'position': None,
            'error': None
        }
        
        try:
            if ifc_element.Representation is None:
                result['error'] = "No geometry representation"
                return result
            # Extract raw geometry using IfcOpenShell
            settings = ifcopenshell.geom.settings()
            settings.set(settings.USE_WORLD_COORDS, True)
            shape = ifcopenshell.geom.create_shape(settings, ifc_element)
            geometry = shape.geometry
            
            # Convert to numpy arrays
            vertices = np.array(geometry.verts).reshape(-1, 3).astype(np.float64)
            faces = np.array(geometry.faces).reshape(-1, 3).astype(np.int32)
            
            if len(vertices) == 0 or len(faces) == 0:
                if hasattr(geometry, 'has_valid_geometry') and geometry.has_valid_geometry():
                    print(f"Shape could not be extracted for {ifc_element.GlobalId}")
                    result['error'] = "Shape could not be extracted"
                return result
                
            # Extract bounding box
            bbox = GeometryProcessor.extract_bbox(vertices)
            if bbox is None:
                print(f"  Warning: Failed to extract bbox for {ifc_element.GlobalId} ({element_type})")
            
            # Extract 2D polygon
            polygon_data = GeometryProcessor.extract_2d_polygon(vertices, faces)
            if polygon_data is None:
                print(f"  Warning: Failed to extract 2D polygon for {ifc_element.GlobalId} ({element_type}) - "
                      f"Vertices: {len(vertices)}, Faces: {len(faces)}")
            
            # Calculate position (this will be None if polygon_data is None)
            position = GeometryProcessor.calculate_position(bbox, polygon_data, element_type)
            
            # Convert to serializable format
            result.update({
                'vertices': vertices.tolist(),
                'faces': faces.tolist(),
                'bbox': bbox,
                'polygon': polygon_data,
                'position': position
            })
            
        except Exception as e:
            print(f"  Error in extract_shape_data for {ifc_element.GlobalId} ({element_type}): {e}")
            result['error'] = str(e)
            
        return result
    
    @staticmethod
    def extract_bbox(vertices: np.ndarray) -> Optional[Dict[str, float]]:
        """Extract bounding box from vertices."""
        try:
            if len(vertices) == 0:
                return None
            x_coords = vertices[:, 0]
            y_coords = vertices[:, 1]
            z_coords = vertices[:, 2]
            return {
                "min_x": round(float(np.min(x_coords)), 3), 
                "max_x": round(float(np.max(x_coords)), 3),
                "min_y": round(float(np.min(y_coords)), 3),
                "max_y": round(float(np.max(y_coords)), 3),
                "min_z": round(float(np.min(z_coords)), 3),
                "max_z": round(float(np.max(z_coords)), 3)
            }
        except Exception:
            return None
    
    @staticmethod
    def extract_2d_polygon(vertices: np.ndarray, faces: np.ndarray, max_height: float = 0.0) -> Optional[Dict[str, Any]]:
        """Extract 2D polygon by projecting geometry onto XY plane."""
        
        def merge_polygons(multi_polygon, grow: float = 0.3, shrink: float = 0.3):
            """Merge overlapping polygons with buffer operations."""
            from shapely.ops import unary_union
            buffered = multi_polygon.buffer(grow, join_style=2)
            merged = unary_union(buffered)
            return merged.buffer(-shrink, join_style=2)
        
        try:
            if len(vertices) == 0 or len(faces) == 0:
                return None
            
            # Calculate bounding box for height filtering
            z_coords = vertices[:, 2]
            min_z = np.min(z_coords)
            max_z = np.max(z_coords)
            
            # Create triangles in 2D
            triangles_2d = []
            skipped_triangles = 0
            invalid_triangles = 0
            
            for i in range(len(faces)):
                v1, v2, v3 = faces[i]
                
                # Validate face indices
                if v1 >= len(vertices) or v2 >= len(vertices) or v3 >= len(vertices):
                    continue
                if v1 < 0 or v2 < 0 or v3 < 0:
                    continue

                # Check height constraints (only if max_height != 0)
                if max_height > 0:
                    if not (min_z <= vertices[v1, 2] <= min_z + max_height and
                            min_z <= vertices[v2, 2] <= min_z + max_height and
                            min_z <= vertices[v3, 2] <= min_z + max_height):
                        skipped_triangles += 1
                        continue
                elif max_height < 0:
                    if not (max_z + max_height <= vertices[v1, 2] <= max_z and
                            max_z + max_height <= vertices[v2, 2] <= max_z and
                            max_z + max_height <= vertices[v3, 2] <= max_z):
                        skipped_triangles += 1
                        continue
                
                # Create 2D triangle using numpy slicing
                p1 = (float(vertices[v1, 0]), float(vertices[v1, 1]))
                p2 = (float(vertices[v2, 0]), float(vertices[v2, 1]))
                p3 = (float(vertices[v3, 0]), float(vertices[v3, 1]))
                
                # Check for degenerate triangles (collinear points)
                if p1 == p2 or p2 == p3 or p1 == p3:
                    invalid_triangles += 1
                    continue
                
                try:
                    triangle = Polygon([p1, p2, p3])
                    if triangle.is_valid and triangle.area > 1e-10:  # Very small threshold for area
                        triangles_2d.append(triangle)
                    else:
                        invalid_triangles += 1
                except Exception as e:
                    invalid_triangles += 1
                    continue
            
            # Debug information for troubleshooting
            total_faces = len(faces)
            valid_triangles = len(triangles_2d)
            
            # If no valid triangles were found, provide debug info
            if valid_triangles == 0:
                print(f"  Debug: No valid triangles found - Total faces: {total_faces}, "
                      f"Skipped (height): {skipped_triangles}, Invalid: {invalid_triangles}, "
                      f"Height constraint: {max_height}")
                return None
            
            # Compute union of triangles
            if triangles_2d:
                from shapely.ops import unary_union
                
                try:
                    geometry_2d = unary_union(triangles_2d)
                except Exception as e:
                    print(f"  Debug: Union operation failed: {e}")
                    return None
                
                # Handle different geometry types
                if geometry_2d.is_empty:
                    print(f"  Debug: Union resulted in empty geometry")
                    return None
                
                # Simplify and clean
                try:
                    if geometry_2d.geom_type == 'MultiPolygon':
                        geometry_2d = merge_polygons(geometry_2d)
                    
                    # After merge, might still be MultiPolygon if polygons aren't connected
                    # In that case, take the largest polygon
                    if geometry_2d.geom_type == 'MultiPolygon':
                        # Get the largest polygon from the MultiPolygon
                        geometry_2d = max(geometry_2d.geoms, key=lambda p: p.area)
                    
                    # Try without simplification first
                    if geometry_2d.is_valid and geometry_2d.area > 1e-6:
                        # Only simplify if the original is valid
                        simplified = geometry_2d.simplify(0.03, preserve_topology=True)
                        if simplified.is_valid and simplified.area > 1e-6:
                            geometry_2d = simplified
                        # If simplification failed, keep the original
                    
                    if geometry_2d.is_valid and geometry_2d.area > 1e-6:
                        # Return serializable polygon data
                        try:
                            coords = list(geometry_2d.exterior.coords)
                            centroid = geometry_2d.centroid
                            return {
                                'coordinates': coords,
                                'area': geometry_2d.area,
                                'centroid_x': centroid.x,
                                'centroid_y': centroid.y
                            }
                        except Exception as e:
                            print(f"  Debug: Failed to extract polygon data: {e}")
                            return None
                    else:
                        print(f"  Debug: Final geometry invalid or too small - Valid: {geometry_2d.is_valid}, Area: {geometry_2d.area}")
                        return None
                        
                except Exception as e:
                    print(f"  Debug: Geometry processing failed: {e}")
                    return None
            
            return None
            
        except Exception as e:
            print(f"  Debug: extract_2d_polygon failed with exception: {e}")
            return None
    
    @staticmethod
    def calculate_position(bbox: Optional[Dict[str, float]], 
                          polygon_data: Optional[Dict[str, Any]], 
                          element_type: str) -> Optional[Dict[str, float]]:
        """Calculate position for topology graph display."""
        if bbox is None or polygon_data is None:
            return None
        try:
            x = polygon_data['centroid_x']
            y = polygon_data['centroid_y']
            
            # Determine Z coordinate based on element type
            if element_type in ["IfcStair", "IfcStairFlight", "IfcRamp", "IfcRampFlight"]:
                z = (bbox["max_z"] + bbox["min_z"]) / 2
            else:
                z = bbox["min_z"]
                
            return {
                "x": round(x, 3),
                "y": round(y, 3),
                "z": round(z, 3)
            }
        except Exception:
            return None
    
    @staticmethod
    def get_mesh_path(global_id: str, config: ProcessingConfig) -> Optional[str]:
        """Generate mesh save path."""
        return f"{config.navigation_model_path}/meshes/{global_id}{config.mesh_export_format}"
    
    @staticmethod
    def get_voxel_path(global_id: str, config: ProcessingConfig) -> str:
        """Generate voxel grid save path."""
        return f"{config.navigation_model_path}/voxel_grids/{global_id}.ply"
    
    @staticmethod
    def create_voxel_grid(vertices: List[List[float]], faces: List[List[int]], 
                         save_path: str, resolution: float) -> None:
        """Create voxel grid from geometry data."""
        try:
            # Convert back to numpy for voxel grid creation
            vertices_np = np.array(vertices)
            faces_np = np.array(faces)
            
            # Create Open3D mesh
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(vertices_np)
            mesh.triangles = o3d.utility.Vector3iVector(faces_np)
            mesh.compute_vertex_normals()
            mesh.compute_triangle_normals()
            
            # Create voxel grid
            voxel_grid = VoxelGrid(save_path=save_path, resolution=resolution)
            voxel_grid.create_filled_voxel_grid(mesh)
            
        except Exception as e:
            print(f"Error creating voxel grid: {e}")
    

# Global functions needed for multiprocessing (must be at module level)
def process_element_batch_worker(batch_data: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """Worker function for multiprocessing - delegates to GeometryProcessor."""
    elements_batch = batch_data['elements_batch']
    config = batch_data['config']

    return GeometryProcessor.process_element_batch(elements_batch, config)


def save_mesh_worker(mesh_data: Dict[str, Any]) -> Dict[str, Any]:
    """Worker function for parallel mesh saving using IfcConvert."""
    ifc_guid = mesh_data['ifc_guid']
    mesh_save_path = mesh_data['mesh_save_path']
    ifc_file_path = mesh_data['ifc_file_path']
    element_type = mesh_data['element_type']
    threads = mesh_data.get('threads', 1)
    
    result = {
        'ifc_guid': ifc_guid,
        'success': False,
        'error': None,
        'element_type': element_type
    }
    
    try:
        import subprocess
        
        # Use IfcConvert to export this specific element with colors
        cmd = [
            "IfcConvert", "--include", "attribute", "GlobalId", ifc_guid,
            "-y", ifc_file_path, mesh_save_path, "-j", str(threads)
        ]
        
        process_result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        
        if process_result.returncode == 0:
            result['success'] = True
        else:
            result['error'] = f"IfcConvert failed: {process_result.stderr}"
            
    except subprocess.TimeoutExpired:
        result['error'] = "IfcConvert timeout"
    except FileNotFoundError:
        result['error'] = "IfcConvert not found. Please install IfcOpenShell command-line tools."
    except Exception as e:
        result['error'] = f"Error: {str(e)}"
    
    return result


class IfcManager:
    """Manager for IFC file processing."""

    def __init__(self, ifc_file_path: Optional[str] = None,
                 navigation_model_path: Optional[str] = None,
                 export_meshes: bool = True,
                 mesh_export_format: str = ".dae",
                 resolution: float = 0.05) -> None:
        self.ifc_file_path = ifc_file_path
        self.navigation_model_path = navigation_model_path
        self.export_meshes = export_meshes
        self.mesh_export_format = mesh_export_format
        self.resolution = resolution
        self.ifc_model = None
        self.elements = {}  # Dictionary to hold extracted elements
        if ifc_file_path:
            self.load_ifc_file()

    
    def load_ifc_file(self, ifc_file_path: Optional[str] = None) -> Any:
        """Load an IFC file from the specified path."""
        if ifc_file_path:
            self.ifc_file_path = ifc_file_path
            
        try:
            self.ifc_model = ifcopenshell.open(self.ifc_file_path)
            print(f"Loaded IFC model: {self.ifc_file_path}")
            return self.ifc_model
        except Exception as e:
            raise ValueError(f"Failed to load IFC model from {self.ifc_file_path}: {e}")
        
    
    def process_elements(self) -> Dict[str, 'IFCElement']:
        """Extract all relevant elements from the IFC model in parallel using multiprocessing.
        Processing includes shape extraction and voxel grid creation."""
        
        # Get all elements and prepare for processing
        all_elements_data = self.get_all_relevant_elements()
        total_elements = len(all_elements_data)

        print(f"Found {total_elements} IFC elements. Creating voxel grids for each element...")
        
        # Process elements in parallel
        processed_elements = self.process_elements_parallel(all_elements_data, total_elements)
        
        # Print summary
        self.print_element_summary(processed_elements)
        
        # Save meshes sequentially to avoid I/O contention
        if self.export_meshes:
            self.save_meshes(processed_elements)
            
        return processed_elements

    
    def get_all_relevant_elements(self) -> List[Dict[str, str]]:
        """Get all relevant IFC elements with only serializable string data."""
        all_elements_data = []
        
        # Get all elements inheriting from IfcElement (physical elements)
        all_elements = self.ifc_model.by_type("IfcElement")
        for element in all_elements:
            element_type = element.is_a()
            if element_type not in ["IfcOpeningElement", "IfcVirtualElement"]:
                all_elements_data.append({
                    'global_id': str(element.GlobalId),  # Explicitly convert to string
                    'element_type': str(element_type)    # Explicitly convert to string
                })
        
        # Add specific non-physical types
        non_physical_types = ["IfcBuildingStorey", "IfcSpace", "IfcRelSpaceBoundary"]
        for element_type in non_physical_types:
            elements = self.ifc_model.by_type(element_type)
            for element in elements:
                all_elements_data.append({
                    'global_id': str(element.GlobalId),  # Explicitly convert to string
                    'element_type': str(element_type)    # Explicitly convert to string
                })
        
        return all_elements_data


    def process_elements_parallel(self, all_elements_data: List[Dict[str, str]], total_elements: int) -> Dict[str, 'IFCElement']:
        """Process elements in parallel using multiprocessing with cleaner architecture."""
        
        # Create processing configuration
        config = ProcessingConfig(
            navigation_model_path=self.navigation_model_path,
            export_meshes=self.export_meshes,
            mesh_export_format=self.mesh_export_format,
            ifc_file_path=self.ifc_file_path,
            resolution=self.resolution,
        )
        
        # Calculate optimal number of processes and batch size for element geometry processing
        optimal_processes, max_batch_size, num_batches = ResourceMonitor.get_optimal_element_processes_and_batches(
            self.ifc_file_path, total_elements)
        
        # Create batches, ensuring a smaller final batch if needed
        batches = []
        batch_size = max_batch_size
        i = 0
        j = 0
        while i < total_elements:
            # If remaining elements are less than effective_batch_size, create a smaller final batch
            if i + batch_size >= total_elements:
                batches.append(all_elements_data[i:total_elements])
                break
            else:
                batches.append(all_elements_data[i:i+batch_size])
                i += batch_size
                j += 1
                # Check if the batch_size may be lowered to avoid one very small batch in the end
                if (batch_size - 1) * (num_batches - j) >= total_elements - i:
                    batch_size -= 1

        # Prepare batch data for workers
        batch_data_list = [
            {
                'elements_batch': batch,
                'config': config
            }
            for batch in batches
        ]
        
        # Process with multiprocessing
        num_processes = min(optimal_processes, len(batch_data_list))
        if batch_size == max_batch_size:
            print(f"  Using {num_processes} parallel processes for {num_batches} batches with {batch_size} elements per batch")
        else:
            print(f"  Using {num_processes} parallel processes for {num_batches} batches with {batch_size}-{max_batch_size} elements per batch")

        processed_elements = {}
        processed_count = 0
        
        with concurrent.futures.ProcessPoolExecutor(max_workers=num_processes) as executor:
            # Submit all batches
            futures = [
                executor.submit(process_element_batch_worker, batch_data)
                for batch_data in batch_data_list
            ]
            
            # Collect results and reconstruct IFCElement objects
            for future in concurrent.futures.as_completed(futures):
                try:
                    batch_results = future.result()
                    
                    # Reconstruct IFCElement objects from serializable data
                    for global_id, element_data in batch_results.items():
                        ifc_element = self.reconstruct_ifc_element(element_data)
                        processed_elements[global_id] = ifc_element
                    
                    processed_count += len(batch_results)
                    
                except Exception as e:
                    print(f"Error processing batch: {e}")

                print_progress_bar(processed_count, total_elements, prefix='  ', length=30)
        
        return processed_elements
    
    
    def reconstruct_ifc_element(self, element_data: Dict[str, Any]) -> 'IFCElement':
        """Reconstruct IFCElement object from processed data."""
        
        # Re-fetch the original IFC element to restore full functionality
        element = self.ifc_model.by_guid(element_data['ifc_guid'])
        if element is None:
            print(f"Warning: Could not re-fetch element {element_data['ifc_guid']}")
            element = None
        
        # Create IFCElement object with the original element reference
        if element_data['element_type'] in ["IfcStair", "IfcStairFlight", "IfcRamp", "IfcRampFlight"]:
            ifc_element = IFCStairRampElement(
                ifc_description=element,
                element_type=element_data['element_type'],
                ifc_guid=element_data['ifc_guid'],
                mesh_save_path=element_data['mesh_save_path'],
                ifc_file_path=element_data['ifc_file_path'],
                voxel_grid_save_path=element_data['voxel_grid_save_path'],
                resolution=element_data['resolution']
            )
        else:
            ifc_element = IFCElement(
                ifc_description=element,
                element_type=element_data['element_type'],
                ifc_guid=element_data['ifc_guid'],
                mesh_save_path=element_data['mesh_save_path'],
                ifc_file_path=element_data['ifc_file_path'],
                voxel_grid_save_path=element_data['voxel_grid_save_path'],
                resolution=element_data['resolution']
            )
        
        # Restore ALL processed data (avoiding re-computation)
        ifc_element.vertices = np.array(element_data['vertices']) if element_data['vertices'] is not None else None
        ifc_element.faces = np.array(element_data['faces']) if element_data['faces'] is not None else None
        ifc_element.bbox = element_data['bbox']
        ifc_element.position = element_data['position']
        ifc_element.floor = element_data['floor']
        
        # Restore polygon from serialized data
        if element_data['polygon'] is not None:
            try:
                coords = element_data['polygon']['coordinates']
                ifc_element.polygon = Polygon(coords)
            except Exception as e:
                print(f"Error reconstructing polygon for {element_data['ifc_guid']}: {e}")
                ifc_element.polygon = None
        else:
            ifc_element.polygon = None
        
        # Regenerate mesh from processed vertex/face data (lightweight operation)
        if ifc_element.vertices is not None and ifc_element.faces is not None:
            try:
                mesh = o3d.geometry.TriangleMesh()
                mesh.vertices = o3d.utility.Vector3dVector(ifc_element.vertices)
                mesh.triangles = o3d.utility.Vector3iVector(ifc_element.faces)
                mesh.compute_vertex_normals()
                mesh.compute_triangle_normals()
                ifc_element.mesh = mesh
            except Exception as e:
                print(f"Error regenerating mesh for {element_data['ifc_guid']}: {e}")
                ifc_element.mesh = None
        else:
            ifc_element.mesh = None
        
        # Load existing voxel grid if it was created
        if element_data['voxel_grid_save_path'] is not None:
            try:
                ifc_element.voxel_grid = VoxelGrid(
                    save_path=element_data['voxel_grid_save_path'],
                    resolution=element_data['resolution']
                )
                # The voxel grid was already created and saved in the worker
            except Exception as e:
                print(f"Error setting up voxel grid for {element_data['ifc_guid']}: {e}")
                ifc_element.voxel_grid = None
        else:
            ifc_element.voxel_grid = None
        
        # Handle special stair/ramp processing
        if isinstance(ifc_element, IFCStairRampElement) and ifc_element.vertices is not None and ifc_element.faces is not None:
            try:
                # Check if this is a physical part of stair or ramp
                if ifc_element.physical_part_of_stair_or_ramp():
                    # Extract start and end positions and walking line
                    ifc_element.start_position, ifc_element.end_position, ifc_element.start_polygon, \
                    ifc_element.end_polygon, ifc_element.walking_line = ifc_element.get_stair_ramp_start_and_end()
            except Exception as e:
                print(f"Error processing stair/ramp specific data for {element_data['ifc_guid']}: {e}")
                ifc_element.start_polygon = None
                ifc_element.end_polygon = None
                ifc_element.start_position = None
                ifc_element.end_position = None
                ifc_element.walking_line = None
        
        return ifc_element

    
    def print_element_summary(self, processed_elements: Dict[str, 'IFCElement']) -> None:
        """Print summary of processed elements by type."""
        element_type_counts = {}
        for element in processed_elements.values():
            element_type = element.element_type
            element_type_counts[element_type] = element_type_counts.get(element_type, 0) + 1
        
        print("Elements found in IFC file:")
        for element_type, count in sorted(element_type_counts.items()):
            print(f"  {count} {element_type}s")

    
    def save_meshes(self, processed_elements: Dict[str, 'IFCElement']) -> None:
        """Save meshes in parallel using multiple IfcConvert processes with resource optimization."""
        
        # Collect elements that need mesh saving
        elements_to_save = []
        for element in processed_elements.values():
            if (element.mesh_save_path is not None and 
                element.element_type != "IfcSpace" and 
                element.mesh is not None):
                elements_to_save.append(element)
        
        total_meshes = len(elements_to_save)
        if total_meshes == 0:
            print("No meshes to save.")
            return

        # Calculate optimal number of processes based on system resources
        optimal_processes = ResourceMonitor.get_optimal_mesh_processes(
            self.ifc_file_path, total_meshes
        )

        print(f"Saving {total_meshes} meshes using {optimal_processes} parallel IfcConvert processes...")
        
        # Calculate threads per IfcConvert process
        threads_per_process = max(1, multiprocessing.cpu_count() // optimal_processes)
        
        # Prepare mesh data for workers
        mesh_tasks = []
        for element in elements_to_save:
            mesh_tasks.append({
                'ifc_guid': element.ifc_guid,
                'mesh_save_path': element.mesh_save_path,
                'ifc_file_path': element.ifc_file_path,
                'element_type': element.element_type,
                'threads': threads_per_process
            })
        
        # Track progress
        saved_count = 0
        failed_count = 0
        
        # Process meshes in parallel
        with concurrent.futures.ProcessPoolExecutor(max_workers=optimal_processes) as executor:
            # Submit all mesh saving tasks
            futures = [executor.submit(save_mesh_worker, task) for task in mesh_tasks]
            
            # Collect results as they complete
            for future in concurrent.futures.as_completed(futures):
                try:
                    result = future.result()
                    if result['success']:
                        saved_count += 1
                    else:
                        failed_count += 1
                        print(f"  Failed to save mesh for {result['ifc_guid']} ({result['element_type']}): {result['error']}")
                        
                except Exception as e:
                    failed_count += 1
                    print(f"  Unexpected error in mesh saving task: {e}")
                
                # Update progress
                completed = saved_count + failed_count
                print_progress_bar(completed, total_meshes, prefix='  ', length=30)
        
        if failed_count > 0:
            print(f"Warning: {failed_count} meshes failed to save. Check IfcConvert installation and file permissions.")