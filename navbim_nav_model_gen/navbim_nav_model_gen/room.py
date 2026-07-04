import networkx as nx
from typing import List, Optional, Any, Dict, TYPE_CHECKING

if TYPE_CHECKING:
    from gpp_bim.floor import Floor
    from gpp_bim.voxel_grid import VoxelGrid
    from gpp_bim.occupancy_map import OccupancyMap


class Room:
    """Represents a room in the IFC model."""

    def __init__(self, name: Optional[str] = None, 
                 floor: Optional[List[str]] = None, 
                 bbox: Optional[Dict[str, float]] = None, 
                 polygon: Optional[Any] = None, 
                 position: Optional[Any] = None, 
                 voxel_grid: Optional['VoxelGrid'] = None, 
                 occupancy_map: Optional['OccupancyMap'] = None,
                 max_width_of_walls: Optional[float] = None) -> None:
        self.name = name
        self.floor = floor
        self.bbox = bbox
        self.polygon = polygon
        self.position = position  # Position of the room for visualization
        self.voxel_grid = voxel_grid
        self.occupancy_map = occupancy_map
        self.max_width_of_walls = max_width_of_walls

    
    def calculate_max_width_of_walls(self, topology_graph: nx.Graph) -> None:
        """Calculate the maximum width of walls in a room."""

        def get_width(wall_polygon) -> float:
            """Extracts the width from the boundaries of a polygon."""
            if not wall_polygon:
                return 0.0
            min_rect = wall_polygon.minimum_rotated_rectangle
            # Get coordinates of rectangle vertices
            x, y = min_rect.exterior.coords.xy
            # Calculate edge lengths (this works for any orientation)
            width = float('inf')
            for i in range(len(x)-1):  # -1 because the last point is the same as first
                length = ((x[i] - x[i+1])**2 + (y[i] - y[i+1])**2)**0.5
                if length < width:
                    width = length
            # Width is the minimum of the two edge lengths
            return width if width < float('inf') else 0.0

        max_width = 0.0
        id = self.ifc_guid if hasattr(self, 'ifc_guid') else self.name
        for wall in topology_graph.neighbors(id):
            wall_data = topology_graph.nodes[wall]
            if wall_data.get("type") in ["IfcWall", "IfcWallStandardCase"]:
                width = get_width(wall_data.get("polygon"))
                if width > max_width:
                    max_width = width
        self.max_width_of_walls = max_width


    def add_to_topology_graph(self, topology_graph: nx.Graph) -> None:
        """Add this room to the topology graph."""
        if (self.bbox is None or self.polygon is None or 
            self.position is None or self.floor is None):
            return
        topology_graph.add_node(self.name,
                                name=self.name,
                                type="Room",
                                floor=self.floor,
                                bbox=self.bbox,
                                polygon=self.polygon,
                                position=self.position)

class IfcRoom(Room):
    """Represents a room extracted from an IfcSpace."""
    
    def __init__(self, name: Optional[str] = None, 
                 floor: Optional['Floor'] = None, 
                 bbox: Optional[Dict[str, float]] = None, 
                 polygon: Optional[Any] = None, 
                 position: Optional[Any] = None,
                 voxel_grid: Optional['VoxelGrid'] = None, 
                 occupancy_map: Optional['OccupancyMap'] = None,
                 max_width_of_walls: Optional[float] = None, 
                 ifc_description: Optional[Any] = None,
                 ifc_guid: str = "", 
                 contained_in: Optional['CombinedRoom'] = None) -> None:
        super().__init__(name, floor, bbox, polygon, position, voxel_grid, 
                         occupancy_map, max_width_of_walls)
        self.ifc_description = ifc_description
        self.ifc_guid = ifc_guid
        # Reference to the parent room if this part of combined room
        self.contained_in = contained_in
    
    def add_to_topology_graph(self, topology_graph: nx.Graph) -> None:
        """Add this room to the topology graph."""
        if (self.bbox is None or self.polygon is None or 
            self.position is None or self.floor is None):
            return
        if self.contained_in:
            # If this room is part of a combined room, do not add it separately
            return
        topology_graph.add_node(self.ifc_guid,
                                name=self.name,
                                type="IfcRoom",
                                floor=self.floor,
                                bbox=self.bbox,
                                polygon=self.polygon,
                                position=self.position,
                                ifc_description=self.ifc_description,
                                ifc_guid=self.ifc_guid)


class DetectedRoom(Room):
    """Represents a room detected from the geometry of an IfcSpace."""

    def __init__(self, name: Optional[str] = None, 
                 floor: Optional['Floor'] = None, 
                 bbox: Optional[Dict[str, float]] = None, 
                 polygon: Optional[Any] = None, 
                 position: Optional[Any] = None,
                 voxel_grid: Optional['VoxelGrid'] = None, 
                 occupancy_map: Optional['OccupancyMap'] = None,
                 max_width_of_walls: Optional[float] = None) -> None:
        super().__init__(name, floor, bbox, polygon, position, voxel_grid, 
                         occupancy_map, max_width_of_walls)

    def add_to_topology_graph(self, topology_graph: nx.Graph) -> None:
        """Add this room to the topology graph."""
        if (self.bbox is None or self.polygon is None or 
            self.position is None or self.floor is None):
            return
        topology_graph.add_node(self.name,
                                name=self.name,
                                type="DetectedRoom",
                                floor=self.floor,
                                bbox=self.bbox,
                                polygon=self.polygon,
                                position=self.position)


class CombinedRoom(Room):
    """Represents a combined room that covers multiple IFC rooms."""

    def __init__(self, name: Optional[str] = None, 
                 floor: Optional['Floor'] = None, 
                 bbox: Optional[Dict[str, float]] = None, 
                 polygon: Optional[Any] = None, 
                 position: Optional[Any] = None, 
                 voxel_grid: Optional['VoxelGrid'] = None, 
                 occupancy_map: Optional['OccupancyMap'] = None,
                 max_width_of_walls: Optional[float] = None, 
                 ifc_rooms: List['IfcRoom'] = None) -> None:
        super().__init__(name, floor, bbox, polygon, position, voxel_grid, 
                         occupancy_map, max_width_of_walls)
        self.ifc_rooms = ifc_rooms if ifc_rooms is not None else []
    
    def add_to_topology_graph(self, topology_graph: nx.Graph) -> None:
        """Add this room including the room_list to the topology graph."""
        if (self.bbox is None or self.polygon is None or 
            self.position is None or self.floor is None):
            return
        room_list = []
        for ifc_room in self.ifc_rooms:
            room_data = {
                "name": ifc_room.name,
                "bbox": ifc_room.bbox,
                "polygon": ifc_room.polygon,
                "position": ifc_room.position,
                "ifc_description": ifc_room.ifc_description,
                "ifc_guid": ifc_room.ifc_guid
            }
            room_list.append(room_data)
        topology_graph.add_node(self.name,
                                name=self.name,
                                type="CombinedRoom",
                                floor=self.floor,
                                bbox=self.bbox,
                                polygon=self.polygon,
                                position=self.position)