from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from gpp_bim.occupancy_map import OccupancyMap
    from gpp_bim.voxel_grid import VoxelGrid
    import networkx as nx
    from shapely.geometry import Polygon


class Floor:
    """Represents a floor with its spatial properties."""

    def __init__(self, floor_name: str, min_z: float, max_z: float, 
                 voxel_grid: Optional['VoxelGrid'] = None,
                 occupancy_map: Optional['OccupancyMap'] = None,
                 polygon: Optional['Polygon'] = None,
                 position: Optional[dict] = None) -> None:
        self.floor_name = floor_name
        self.min_z = min_z
        self.max_z = max_z
        self.voxel_grid = voxel_grid
        self.occupancy_map = occupancy_map
        self.polygon = polygon  # Unionized polygon from IfcSlabs
        self.position = position  # {"x": centroid_x, "y": centroid_y, "z": min_z}
        
        # Calculate position from polygon if not provided
        if self.polygon is not None and self.position is None:
            centroid = self.polygon.centroid
            self.position = {
                "x": round(centroid.x, 3),
                "y": round(centroid.y, 3),
                "z": round(self.min_z, 3)
            }
    
    def add_to_topology_graph(self, topology_graph: 'nx.Graph') -> None:
        """Add this floor to the topology graph."""
        topology_graph.add_node(self.floor_name, 
                                name=self.floor_name,
                                type="Floor",
                                min_z=self.min_z, 
                                max_z=self.max_z,
                                polygon=self.polygon,
                                position=self.position)


class StairSlabFloor(Floor):
    """Represents a stair/ramp slab floor."""
    
    def __init__(self, floor_name: str, min_z: float, max_z: float, 
                 voxel_grid: Optional['VoxelGrid'] = None,
                 occupancy_map: Optional['OccupancyMap'] = None,
                 parent_floor: Optional[str] = None,
                 polygon: Optional['Polygon'] = None,
                 position: Optional[dict] = None) -> None:
        super().__init__(floor_name, min_z, max_z, voxel_grid, occupancy_map, polygon, position)
        # Reference to the main floor name this stair/ramp belongs to
        self.parent_floor = parent_floor
    
    def add_to_topology_graph(self, topology_graph: 'nx.Graph') -> None:
        """Add this floor to the topology graph."""
        topology_graph.add_node(self.floor_name, 
                                name=self.floor_name,
                                type="StairSlabFloor",
                                parent_floor=self.parent_floor,
                                min_z=self.min_z, 
                                max_z=self.max_z,
                                polygon=self.polygon,
                                position=self.position)