from .ifc_manager import IfcManager
from .floor_manager import FloorManager
from .room_manager import RoomManager
from .topology_manager import TopologyManager
from .voxel_grid_manager import VoxelGridManager
from .occupancy_map_manager import OccupancyMapManager
import rclpy
import os
from rclpy.node import Node
import open3d as o3d
import matplotlib.pyplot as plt
import time
import networkx as nx
import multiprocessing

# Set multiprocessing start method for better ROS 2 compatibility
if __name__ != '__main__':
    multiprocessing.set_start_method('spawn', force=True)


class GeneratorManager(Node):
    """Central manager for navigation model generation."""
    
    def __init__(self) -> None:
        super().__init__('nav_model_gen')

        self.declare_parameter('ifc_file', '')
        self.declare_parameter('nav_model', '')
        self.declare_parameter('delete_previous_maps', False)
        self.declare_parameter('2D', True)
        self.declare_parameter('generate_floor_maps', True)
        self.declare_parameter('visualize_generation_of_floors', False)
        self.declare_parameter('visualize_generation_of_rooms', False)
        self.declare_parameter('export_meshes', True)
        self.declare_parameter('mesh_export_format', '.dae')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('vertical_tolerance', 0.3)
        self.declare_parameter('penalize_z_movement', 3.0)
        self.declare_parameter('min_area', 2.0)
        self.declare_parameter('robot_height', 0.5)
        self.declare_parameter('robot_step_height', 0.0)

        self.ifc_file_path = self.get_parameter('ifc_file').get_parameter_value().string_value
        self.navigation_model_path = self.get_parameter('nav_model').get_parameter_value().string_value
        self.delete_previous = self.get_parameter('delete_previous_maps').get_parameter_value().bool_value
        self.two_d = self.get_parameter('2D').get_parameter_value().bool_value
        self.generate_floor_maps = self.get_parameter('generate_floor_maps').get_parameter_value().bool_value
        self.visualize_generation_of_floors = self.get_parameter('visualize_generation_of_floors').get_parameter_value().bool_value
        self.visualize_generation_of_rooms = self.get_parameter('visualize_generation_of_rooms').get_parameter_value().bool_value
        self.export_meshes = self.get_parameter('export_meshes').get_parameter_value().bool_value
        self.mesh_export_format = self.get_parameter('mesh_export_format').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.vertical_tolerance = self.get_parameter('vertical_tolerance').get_parameter_value().double_value
        self.penalize_z_movement = self.get_parameter('penalize_z_movement').get_parameter_value().double_value
        self.min_area = self.get_parameter('min_area').get_parameter_value().double_value
        self.robot_height = self.get_parameter('robot_height').get_parameter_value().double_value
        self.robot_step = self.get_parameter('robot_step_height').get_parameter_value().double_value

        # Initialize data structures
        self.elements = {}  # ifc_guid -> IFCElement objects
        self.floors = {}  # floor_name -> Floor objects
        self.rooms = {}  # room_id -> Room objects
        self.topology_graph = nx.Graph()  # For building topology and relationships
        self.topological_map = nx.Graph()  # Final navigation graph

        # Managers for different tasks
        self.ifc_manager = IfcManager(ifc_file_path=self.ifc_file_path,
                                      navigation_model_path=self.navigation_model_path,
                                      export_meshes=self.export_meshes,
                                      mesh_export_format=self.mesh_export_format,
                                      resolution=self.resolution)
        self.floor_manager = FloorManager(vertical_tolerance=self.vertical_tolerance)
        self.room_manager = RoomManager(vertical_tolerance=self.vertical_tolerance, 
                                        min_area=self.min_area)
        self.topology_manager = TopologyManager(vertical_tolerance=self.vertical_tolerance,
                                                penalize_z_movement=self.penalize_z_movement,
                                                resolution=self.resolution)
        self.voxel_grid_manager = VoxelGridManager(resolution=self.resolution, 
                                                   robot_height=self.robot_height,
                                                   vertical_tolerance=self.vertical_tolerance, 
                                                   navigation_model_path=self.navigation_model_path,
                                                   generate_floor_maps=self.generate_floor_maps)
        self.occupancy_map_manager = OccupancyMapManager(self.navigation_model_path,
                                                         resolution=self.resolution, 
                                                         vertical_tolerance=self.vertical_tolerance,
                                                         robot_height=self.robot_height,
                                                         robot_step=self.robot_step,
                                                         generate_floor_maps=self.generate_floor_maps)
        
        self.generate_navigation_model()


    def generate_navigation_model(self) -> None:
        """Generates the navigation model from an IFC file."""
        
        start_time = time.time()
        print(f"Starting navigation model generation with resolution {self.resolution} m...")

        self.prepare_folders()

        # Step 1: Extract IFC elements into structured data
        print("\n1. Processing IFC elements...")
        self.elements = self.ifc_manager.process_elements()

        # Step 2: Extract and assign floors
        print("\n2. Processing floors...")
        self.floors = self.floor_manager.extract_floors(self.elements)
        self.elements, self.floors = self.floor_manager.assign_elements_to_floors(
            self.elements, self.floors, self.navigation_model_path)

        # Step 3: Detect rooms and spaces and assign elements to rooms
        print("\n3. Detecting rooms...")
        self.rooms, merged_walls = self.room_manager.detect_rooms(self.elements, self.floors)
        if self.visualize_generation_of_floors or self.visualize_generation_of_rooms:
            print("Displaying detected rooms...")
            # Visualization of detected rooms for each floor
            for floor_name, floor in self.floors.items():
                # Get rooms for this floor only
                floor_rooms = {room_id: room for room_id, room in self.rooms.items() 
                                if hasattr(room, 'floor') and floor_name in room.floor}
                self.room_manager.visualize_room_detection(floor_name, 
                                                           merged_walls.get(floor_name, None), 
                                                           floor_rooms)

        # Step 4: Build spatial relationships
        print("\n4. Building spatial relationships...")
        self.topology_manager.find_spatial_relationships(self.elements, self.rooms)

        # Step 5: Generate topology graph from structured data
        print("\n5. Building topology graph...")
        self.topology_graph = self.topology_manager.build_topology_graph(
            self.elements, self.floors, self.rooms)

        # Step 6: Generate topological map for navigation
        print("\n6. Generating topological map...")
        self.topological_map = self.topology_manager.generate_topological_map(
            self.topology_graph)
        self.topology_manager.save_topological_map(self.topological_map, self.navigation_model_path)

        # Step 7: Generate voxel grids and GUID mapping
        print("\n7. Generating voxel grids and GUID mappings...")
        self.voxel_grid_manager.generate_voxel_grids(self.elements, self.floors, self.rooms)
        # Visualization of voxel grids after generation
        if self.visualize_generation_of_floors and self.generate_floor_maps:
            print("Displaying voxelized floors...")
            for floor in self.floors.values():
                voxel_grid_obj = floor.voxel_grid
                if voxel_grid_obj is not None:
                    colored_voxel_grid = voxel_grid_obj.create_colored_voxel_grid(
                        elements=self.elements, max_z=floor.max_z)
                    try:
                        o3d.visualization.draw_geometries([colored_voxel_grid], window_name=floor.floor_name)
                    except Exception as e:
                        print(f"Error visualizing floor {floor.floor_name}: {e}")
        if self.visualize_generation_of_rooms:
            print("Displaying voxelized rooms...")
            for room in self.rooms.values():
                voxel_grid_obj = room.voxel_grid
                if voxel_grid_obj is not None:
                    colored_voxel_grid = voxel_grid_obj.create_colored_voxel_grid(
                        elements=self.elements, max_z=room.bbox["max_z"])
                    try:
                        o3d.visualization.draw_geometries([colored_voxel_grid], window_name=room.name)
                    except Exception as e:
                        print(f"Error visualizing room {room.name}: {e}")

        if self.two_d:
            # Step 8: Create occupancy maps
            print("\n8. Creating 2D occupancy maps...")
            self.occupancy_map_manager.create_occupancy_maps(self.elements, self.floors, self.rooms)
            # Visualization of occupancy maps
            if self.visualize_generation_of_floors and self.generate_floor_maps:
                for floor in self.floors.values():
                    if (floor.occupancy_map is not None and
                        floor.occupancy_map.occupancy_grid is not None):
                        plt.imshow(floor.occupancy_map.occupancy_grid, cmap="gray", vmin=0, vmax=255, origin="lower")
                        plt.title(f"Occupancy Grid for {floor.name}")
                        plt.show()
            if self.visualize_generation_of_rooms:
                for room in self.rooms.values():
                    if (room.occupancy_map is not None and
                        room.occupancy_map.occupancy_grid is not None):
                        plt.imshow(room.occupancy_map.occupancy_grid, cmap="gray", vmin=0, vmax=255, origin="lower")
                        plt.title(f"Occupancy Grid for {room.name}")
                        plt.show()

        end_time = time.time()
        execution_time = end_time - start_time
        minutes, seconds = divmod(execution_time, 60)
        print(f"\nNavigation model generation completed in {int(minutes):02d}:{int(seconds):02d}")
    
    
    def prepare_folders(self) -> None:
        """Prepare folders for the navigation model."""
        if self.delete_previous:
            # Remove the previous navigation model if it exists
            if os.path.exists(self.navigation_model_path):
                # Remove directory and all its contents using os
                for root, dirs, files in os.walk(self.navigation_model_path, topdown=False):
                    # Remove all files
                    for file in files:
                        if not file.endswith(".rviz"): # Keep RViz config files
                            os.remove(os.path.join(root, file))
                    # Remove all empty directories
                    for dir in dirs:
                        os.rmdir(os.path.join(root, dir))
                print(f"\nDeleted previous navigation model in {self.navigation_model_path}")
        # Create folder for navigation model
        os.makedirs(self.navigation_model_path, exist_ok=True)
        os.makedirs(os.path.join(self.navigation_model_path, "meshes"), exist_ok=True)
        os.makedirs(os.path.join(self.navigation_model_path, "voxel_grids"), exist_ok=True)
    

def main() -> None:
    rclpy.init()
    GeneratorManager()
    exit(0)

if __name__ == '__main__':
    main()
