import os
from .ifc import IFCElement, IFCStairRampElement
from .room import Room, IfcRoom
from .floor import Floor
from navbim_util.polygon_utils import convert_shapely_to_json
from navbim_util.network_utils import euclidean_distance_between_nodes, cost_between_nodes
from typing import Dict, List, Tuple, Any
import numpy as np
import networkx as nx
import copy
from shapely.ops import unary_union
from shapely.affinity import translate
import json


class TopologyManager:
    """Manages the topology graph and topological map generation."""

    def __init__(self, vertical_tolerance: float = 0.3, penalize_z_movement: float = 3.0,
                 resolution: float = 0.05) -> None:
        self.vertical_tolerance = vertical_tolerance
        self.penalize_z_movement = penalize_z_movement
        self.resolution = resolution

    def find_spatial_relationships(self, elements: Dict[str, 'IFCElement'],
                                   rooms: Dict[str, 'Room']) -> List[Tuple[str, str]]:
        """Find spatial relationships between elements and rooms."""

        relationships = []

        # ToDo: Process IfcRelSpaceBoundary relationships first

        # Find spatial relationships between rooms and elements
        for room_id, room_data in rooms.items():

            # Continue if room data is incomplete
            if (room_data.polygon is None or 
                room_data.floor is None or 
                len(room_data.floor) == 0):
                continue
            # Continue if IfcRoom is contained in combined room
            if (isinstance(room_data, IfcRoom) and
                room_data.contained_in is not None):
                continue

            # Iterate through elements
            for element_id, element in elements.items():
                # Omit elements without a shape
                if element.polygon is None:
                    continue
                # At least one item in both floor lists should match
                floors_match = any(floor in room_data.floor for floor in element.floor)
                # For stairs/ramps, also check if start_floor or end_floor matches
                if (hasattr(element, 'start_floor') or hasattr(element, 'end_floor')):
                    start_floor_matches = (hasattr(element, 'start_floor') and 
                                         element.start_floor in room_data.floor)
                    end_floor_matches = (hasattr(element, 'end_floor') and 
                                       element.end_floor in room_data.floor)
                    # Skip only if no floors match at all (regular, start, or end)
                    if not (floors_match or start_floor_matches or end_floor_matches):
                        continue
                # For regular elements, skip if no regular floors match
                elif not floors_match:
                    continue
                
                # Check adjacency
                # For stairs/ramps, check adjacency also for start and end points
                if isinstance(element, IFCStairRampElement):
                    if self.check_adjacency(room_data.polygon, element.polygon):
                        relationships.append((room_id, element_id))
                    if (hasattr(element, 'start_polygon') and 
                        hasattr(element, 'start_floor')):
                        if (self.check_adjacency(room_data.polygon, element.start_polygon) and
                            element.start_floor in room_data.floor):
                            relationships.append((room_id, element_id + "_start"))
                    if (hasattr(element, 'end_polygon') and
                        hasattr(element, 'end_floor')):
                        if (self.check_adjacency(room_data.polygon, element.end_polygon) and
                            element.end_floor in room_data.floor):
                            relationships.append((room_id, element_id + "_end"))
                        
                # For doors and windows, check perpendicular adjacency
                elif element.element_type in ["IfcDoor", "IfcWindow"]:
                    if self.check_adjacency_perpendicular(room_data.polygon, element.polygon):
                        relationships.append((room_id, element_id))

                # For other elements, check standard adjacency
                elif self.check_adjacency(room_data.polygon, element.polygon):
                    relationships.append((room_id, element_id))

        # Add relationships to ifc element data
        for room_id, element_id in relationships:
            if element_id in elements:
                if not room_id in elements[element_id].room:
                    elements[element_id].room.append(room_id)
            elif (element_id.endswith("_start") and
                  element_id[:-6] in elements):
                elements[element_id[:-6]].start_room = room_id
            elif (element_id.endswith("_end") and
                  element_id[:-4] in elements):
                elements[element_id[:-4]].end_room = room_id

        print(f"Found {len(relationships)} spatial relationships")


    def check_adjacency(self, polygon1: Any, polygon2: Any, buffer: float = 0.05) -> bool:
        """Check if two polygons are spatially adjacent."""
        try:
            buffered_polygon2 = polygon2.buffer(buffer)
            if (polygon1.intersects(buffered_polygon2) or
                polygon1.contains(polygon2) or
                polygon2.contains(polygon1)):
                return True
            return False
        except Exception:
            return False
    

    def check_adjacency_perpendicular(self, polygon1: Any, polygon2: Any, buffer: float = 0.5) -> bool:
        """Check if two polygons are spatially adjacent in a perpendicular manner."""
        try:
            # Get the minimum bounding rectangle of polygon2
            min_rotated_rect = polygon2.minimum_rotated_rectangle
            rect_coords = list(min_rotated_rect.exterior.coords[:-1])
            rect_array = np.array(rect_coords)
            # Calculate edge lengths to determine which is width vs height
            edge1_length = np.linalg.norm(rect_array[1] - rect_array[0])
            edge2_length = np.linalg.norm(rect_array[2] - rect_array[1])
            # Determine the secondary direction (shorter edge for doors/windows)
            if edge1_length < edge2_length:
                secondary_direction = rect_array[1] - rect_array[0]
                secondary_direction = secondary_direction / np.linalg.norm(secondary_direction)
            else:
                secondary_direction = rect_array[2] - rect_array[1]
                secondary_direction = secondary_direction / np.linalg.norm(secondary_direction)
            # Create expanded polygon
            expansion_vector = secondary_direction * buffer
            left_polygon = translate(polygon2, xoff=expansion_vector[0], yoff=expansion_vector[1])
            right_polygon = translate(polygon2, xoff=-expansion_vector[0], yoff=-expansion_vector[1])
            expanded_polygon = unary_union([polygon2, left_polygon, right_polygon])
            return polygon1.intersects(expanded_polygon)
        except Exception:
            return False


    def build_topology_graph(self, elements: Dict[str, 'IFCElement'], 
                             floors: Dict[str, 'Floor'], 
                             rooms: Dict[str, 'Room']) -> nx.Graph:
        """Build the topology graph from elements, floors, and rooms."""

        topology_graph = nx.Graph()

        # Different types of nodes and edges in the topology graph:
        # - Floor nodes
        # - Room nodes
        # - Element nodes
        # - Floor edges (connects rooms and elements to their respective floors)
        # - Room edges (connects elements to rooms based on spatial relationships)

        # Add floor nodes for each floor
        for floor in floors.values():
            floor.add_to_topology_graph(topology_graph)

        # Add room nodes for each room
        for room in rooms.values():
            room.add_to_topology_graph(topology_graph)

        # Add nodes for each element
        for element in elements.values():
            element.add_to_topology_graph(topology_graph)

        # Add floor edges to connect all rooms and elements to the floor they belong to
        for room_id, room in rooms.items():
            if not room_id in topology_graph:
                continue
            for floor_name in room.floor:
                if floor_name in topology_graph:
                    topology_graph.add_edge(floor_name, room_id, type="floor")
        for element_id, element in elements.items():
            if not element_id in topology_graph:
                continue
            for floor_name in element.floor:
                if floor_name in topology_graph:
                    topology_graph.add_edge(floor_name, element_id, type="floor")
            # Add room edges for stair/ramp start and end nodes
            if isinstance(element, IFCStairRampElement) and element.ifc_guid in topology_graph:
                # Between start and stair/ramp
                if element.ifc_guid + "_start" in topology_graph.nodes:
                    topology_graph.add_edge(element.ifc_guid + "_start", element_id, type="room")
                # Between end and stair/ramp
                if element.ifc_guid + "_end" in topology_graph.nodes:
                    topology_graph.add_edge(element.ifc_guid + "_end", element_id, type="room")
                # Between start and room
                if (element.ifc_guid + "_start" in topology_graph.nodes and
                    element.start_room in topology_graph):
                    topology_graph.add_edge(element.ifc_guid + "_start", element.start_room, type="room")
                # Between end and room
                if (element.ifc_guid + "_end" in topology_graph.nodes and
                    element.end_room in topology_graph):
                    topology_graph.add_edge(element.ifc_guid + "_end", element.end_room, type="room")
            # Add room edges to connect elements to their respective rooms
            else:
                for room_id in element.room:
                    if (room_id in topology_graph and
                        room_id != element_id and 
                        element.element_type != "IfcSpace"):
                        topology_graph.add_edge(room_id, element_id, type="room")

        # Calculate maximum wall widths for each room to be used later to crop rooms
        for room_id, room in rooms.items():
            if room_id in topology_graph:
                room.calculate_max_width_of_walls(topology_graph)

        print(f"Topology graph built with {topology_graph.number_of_nodes()} nodes and {topology_graph.number_of_edges()} edges")

        return topology_graph


    def generate_topological_map(self, topology_graph: nx.Graph) -> nx.Graph:
        """Generates the topological map from the topology graph."""

        # Remove all elements and attributes that are not necessary for navigation
        topological_map = self.remove_unnecessary_nodes_and_attributes(topology_graph)

        # Convert to integer node IDs for ROS compatibility
        topological_map = self.convert_to_integer_ids(topological_map)

        # Add transition edges between rooms
        self.add_transition_edges(topological_map)

        print(f"Topological map generated with {topological_map.number_of_nodes()} nodes and {topological_map.number_of_edges()} edges")
        return topological_map


    def remove_unnecessary_nodes_and_attributes(self, topology_graph: nx.Graph) -> nx.Graph:
        """Remove nodes and attributes that are not necessary for navigation."""

        topological_map = topology_graph.copy()

        nodes_to_remove = []
        for node_id, node_data in list(topological_map.nodes(data=True)):
            node_type = node_data.get("type", "")
            # Only keep floors, rooms, doors, stairs, and ramp nodes
            if (node_type in ["Floor", "StairSlabFloor", 
                              "Room", "IfcRoom", "DetectedRoom", "CombinedRoom",
                              "IfcDoor", "IfcStair", "IfcStairFlight", "IfcRamp", "IfcRampFlight"]):
                # Remove unnecessary attributes from these elements
                attributes_to_remove = []
                for attr in node_data.keys():
                    if attr in ["vertices", "faces", "mesh", "ifc_description"]:
                        attributes_to_remove.append(attr)
                for attr in attributes_to_remove:
                    topological_map.nodes[node_id].pop(attr, None)
            else:
                nodes_to_remove.append(node_id)
        for node_id in nodes_to_remove:
            topological_map.remove_node(node_id)
        
        return topological_map


    def convert_to_integer_ids(self, topological_map: nx.Graph) -> nx.Graph:
        """Convert string node IDs to integers for ROS compatibility."""
        new_node_id = 0
        new_edge_id = 0
        id_mapping = {}
        
        # Create new graph with integer IDs
        new_graph = nx.Graph()
        
        # Convert nodes
        for old_id, node_data in topological_map.nodes(data=True):
            node_type = node_data.get("type")

            if node_type in ["Floor", "StairSlabFloor"]:
                new_data = copy.deepcopy(node_data)
                new_data["type"] = "floor"
                # Use existing position if available, otherwise default to (0, 0, min_z)
                if "position" not in new_data or new_data["position"] is None:
                    new_data["position"] = {"x": 0.0, "y": 0.0, "z": new_data["min_z"]}
                new_graph.add_node(new_node_id, **new_data)
                id_mapping[old_id] = new_node_id
                new_node_id += 1
            
            elif node_type in ["Room", "IfcRoom", "DetectedRoom", "CombinedRoom"]:
                new_data = copy.deepcopy(node_data)
                new_data["type"] = "room"
                new_graph.add_node(new_node_id, **new_data)
                id_mapping[old_id] = new_node_id
                new_node_id += 1
                
            elif node_type == "IfcDoor":
                new_data = copy.deepcopy(node_data)
                new_data["type"] = "transition"
                new_data["subtype"] = "door"
                # Align position to grid
                #new_data["position"] = self.align_position_to_grid(new_data["position"])
                new_graph.add_node(new_node_id, **new_data)
                id_mapping[old_id] = new_node_id
                new_node_id += 1
                
            elif node_type in ["IfcStair", "IfcStairFlight"]:
                new_data = copy.deepcopy(node_data)
                if old_id.endswith("_start"):
                    new_data["type"] = "transition"
                    new_data["subtype"] = "stair_start"
                    new_data["position"]["z"] = topological_map.nodes[new_data["floor"]]["min_z"]
                    # Align position to grid
                    #new_data["position"] = self.align_position_to_grid(new_data["position"])
                elif old_id.endswith("_end"):
                    new_data["type"] = "transition"
                    new_data["subtype"] = "stair_end"
                    new_data["position"]["z"] = topological_map.nodes[new_data["floor"]]["min_z"]
                    # Align position to grid
                    #new_data["position"] = self.align_position_to_grid(new_data["position"])
                else:
                    new_data["type"] = "stair"
                new_graph.add_node(new_node_id, **new_data)
                id_mapping[old_id] = new_node_id
                new_node_id += 1
            
            elif node_type in ["IfcRamp", "IfcRampFlight"]:
                new_data = copy.deepcopy(node_data)
                new_data["type"] = "transition"
                if old_id.endswith("_start"):
                    new_data["type"] = "transition"
                    new_data["subtype"] = "ramp_start"
                    new_data["position"]["z"] = topological_map.nodes[new_data["floor"]]["min_z"]
                    # Align position to grid
                    #new_data["position"] = self.align_position_to_grid(new_data["position"])
                elif old_id.endswith("_end"):
                    new_data["type"] = "transition"
                    new_data["subtype"] = "ramp_end"
                    new_data["position"]["z"] = topological_map.nodes[new_data["floor"]]["min_z"]
                    # Align position to grid
                    #new_data["position"] = self.align_position_to_grid(new_data["position"])
                else:
                    new_data["type"] = "ramp"
                new_graph.add_node(new_node_id, **new_data)
                id_mapping[old_id] = new_node_id
                new_node_id += 1
        
        # Convert edges
        for u, v, edge_data in topological_map.edges(data=True):
            if u in id_mapping and v in id_mapping:
                new_u = id_mapping[u]
                new_v = id_mapping[v]
                edge_data["id"] = new_edge_id
                new_graph.add_edge(new_u, new_v, **edge_data)
                new_edge_id += 1
                
        return new_graph


    def add_transition_edges(self, topological_map: nx.Graph) -> None:
        """Add transition edges between transition nodes corresponding to the same room."""

        def interpolate_walking_line_3d(walking_line_2d: List[Tuple[float, float]], 
                                        start_z: float, 
                                        end_z: float) -> List[List[float]]:
            """
            Convert a 2D walking line to a 3D path with linearly interpolated z-coordinates.
            Also aligns the xy-coordinates to multiples of the resolution and densifies the path
            so that consecutive points are approximately resolution distance apart.
            """
            if not walking_line_2d or len(walking_line_2d) < 2:
                return []
            
            # Align xy-coordinates to grid
            """ aligned_line_2d = [
                (round(x / self.resolution) * self.resolution,
                 round(y / self.resolution) * self.resolution)
                for x, y in walking_line_2d
            ] """
            aligned_line_2d = walking_line_2d  # Skip alignment for now to preserve original path shape

            # Calculate cumulative distances along the 2D path
            distances = [0.0]
            for i in range(1, len(aligned_line_2d)):
                dx = aligned_line_2d[i][0] - aligned_line_2d[i-1][0]
                dy = aligned_line_2d[i][1] - aligned_line_2d[i-1][1]
                dist = np.sqrt(dx**2 + dy**2)
                distances.append(distances[-1] + dist)
            
            total_distance = distances[-1]
            if total_distance == 0:
                return []
            
            # Interpolate z-coordinates based on distance ratio
            path_3d = []
            for i, (x, y) in enumerate(aligned_line_2d):
                ratio = distances[i] / total_distance
                z = start_z + (end_z - start_z) * ratio
                path_3d.append([round(x, 3), round(y, 3), round(z, 3)])
            
            # Densify the path by inserting intermediate points
            densified_path = []
            for i in range(len(path_3d) - 1):
                p1 = path_3d[i]
                p2 = path_3d[i + 1]
                
                # Always add the first point
                densified_path.append(p1)
                
                # Calculate 3D distance between consecutive points
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                dz = p2[2] - p1[2]
                segment_distance = np.sqrt(dx**2 + dy**2 + dz**2)
                
                # If distance is greater than resolution, insert intermediate points
                if segment_distance > self.resolution:
                    # Calculate number of intermediate points needed
                    num_points = int(np.ceil(segment_distance / self.resolution))
                    
                    # Insert intermediate points
                    for j in range(1, num_points):
                        t = j / num_points
                        interp_x = p1[0] + t * dx
                        interp_y = p1[1] + t * dy
                        interp_z = p1[2] + t * dz
                        densified_path.append([round(interp_x, 3), round(interp_y, 3), round(interp_z, 3)])
            
            # Add the last point
            if path_3d:
                densified_path.append(path_3d[-1])
            
            return densified_path

        for room_node, room_data in topological_map.nodes(data=True):
            if room_data.get("type") in ["room", "stair", "ramp"]:
                transition_neighbors = [n for n in topological_map.neighbors(room_node)
                                      if topological_map.nodes[n].get("type") == "transition"]
                # Add edges between all pairs of transition neighbors
                for i in range(len(transition_neighbors)):
                    for j in range(i + 1, len(transition_neighbors)):
                        tn1, tn2 = transition_neighbors[i], transition_neighbors[j]
                        if not topological_map.has_edge(tn1, tn2):
                            # Calculate distance
                            distance = euclidean_distance_between_nodes(topological_map, tn1, tn2)
                            cost = cost_between_nodes(topological_map, tn1, tn2)
                            edge_data = {
                                "type": "transition",
                                "estimated_distance": distance,
                                "planned_distance": -1.0,
                                "estimated_cost": cost,
                                "planned_cost": -1.0,
                                "path": [],
                                "id": len(list(topological_map.edges())),
                                "room_id": room_node
                            }
                            if room_data.get("type") in ["stair", "ramp"]:
                                edge_data["subtype"] = room_data["type"]
                                # Pre-plan path for stairs/ramps based on walking_line
                                walking_line_2d = room_data.get("walking_line")
                                if walking_line_2d:
                                    start_z = topological_map.nodes[tn1]["position"]["z"]
                                    end_z = topological_map.nodes[tn2]["position"]["z"]
                                    path_3d = interpolate_walking_line_3d(walking_line_2d, start_z, end_z)
                                    if path_3d:
                                        edge_data["path"] = path_3d
                                        # Calculate actual path distance in 3D
                                        planned_distance = 0.0
                                        planned_cost = 0.0
                                        for k in range(1, len(path_3d)):
                                            dx = path_3d[k][0] - path_3d[k-1][0]
                                            dy = path_3d[k][1] - path_3d[k-1][1]
                                            dz = path_3d[k][2] - path_3d[k-1][2]
                                            planned_distance += np.sqrt(dx**2 + dy**2 + dz**2)
                                            planned_cost += np.sqrt(dx**2 + dy**2 + self.penalize_z_movement * dz**2)
                                        edge_data["planned_cost"] = planned_cost
                                        edge_data["planned_distance"] = planned_distance

                            topological_map.add_edge(tn1, tn2, **edge_data)


    def save_topological_map(self, topological_map: nx.Graph, navigation_model_path: str) -> None:
        """Save the topological map to a JSON file."""       
        try:
            # Convert to node-link format
            data = nx.node_link_data(topological_map)
            # Convert Shapely geometries to JSON-serializable format
            data = convert_shapely_to_json(data)
            # Define file path
            model_name = os.path.basename(navigation_model_path.rstrip('/'))
            file_path = f"{navigation_model_path}/{model_name}.json"
            # Save to file
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"Topological map saved to {file_path}")
        except Exception as e:
            print(f"Error saving topological map: {e}")


    def align_position_to_grid(self, position: Dict[str, float]) -> Dict[str, float]:
        """Align a position dictionary to grid resolution."""
        return {
            "x": round(round(position["x"] / self.resolution) * self.resolution, 3),
            "y": round(round(position["y"] / self.resolution) * self.resolution, 3),
            "z": round(position["z"], 3)
        }