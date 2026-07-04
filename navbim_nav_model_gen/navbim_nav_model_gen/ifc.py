import networkx as nx
import numpy as np
import open3d as o3d
from shapely import LineString, Point
from shapely.geometry import Polygon
from shapely.ops import split
from typing import Dict, List, Any, Optional, Tuple, TYPE_CHECKING
import subprocess
from ladybug_geometry_polyskel.polyskel import _skeletonize

if TYPE_CHECKING:
    from gpp_bim.voxel_grid import VoxelGrid


class IFCElement:
    """Represents an IFC element with its properties and geometry."""
    
    def __init__(self, 
                 ifc_description: Any = None, 
                 element_type: str = None, 
                 ifc_guid: str = None, 
                 vertices: Optional[np.ndarray] = None, 
                 faces: Optional[np.ndarray] = None, 
                 bbox: Optional[Dict[str, float]] = None, 
                 polygon: Optional[Polygon] = None,
                 mesh: Optional[o3d.geometry.TriangleMesh] = None, 
                 voxel_grid: Optional['VoxelGrid'] = None, 
                 position: Optional[Dict[str, float]] = None, 
                 floor: List[str] = None,
                 room: List[str] = None,
                 mesh_save_path: Optional[str] = None,
                 ifc_file_path: Optional[str] = None,
                 voxel_grid_save_path: Optional[str] = None,
                 resolution: float = 0.05) -> None:
        self.ifc_description = ifc_description
        self.element_type = element_type
        self.ifc_guid = ifc_guid
        self.vertices = vertices
        self.faces = faces
        self.bbox = bbox
        self.polygon = polygon
        self.mesh = mesh
        self.voxel_grid = voxel_grid
        self.position = position
        self.floor = floor if floor is not None else []  # List to hold floor assignments
        self.room = room if room is not None else []  # List to hold room assignments
        self.mesh_save_path = mesh_save_path
        self.ifc_file_path = ifc_file_path  # Path to original IFC file for IfcConvert
        self.voxel_grid_save_path = voxel_grid_save_path
        self.resolution = resolution

    
    def save_mesh(self, threads: int = 1) -> None:
        """Save the mesh using IfcConvert to preserve IFC materials and colors."""
        if self.element_type == "IfcSpace":
            return
        try:
            # Use IfcConvert to export this specific element with colors
            if not hasattr(self, 'ifc_file_path') or not self.ifc_file_path:
                raise ValueError("IFC file path not available for IfcConvert")
            # Create the include filter for this specific element by GlobalId
            cmd = [
                "IfcConvert", "--include", "attribute", "GlobalId", self.ifc_guid,
                "-y", self.ifc_file_path, self.mesh_save_path, "-j", str(threads)
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            if result.returncode != 0:
                print(f"IfcConvert failed for {self.ifc_guid}: {result.stderr}")
                raise RuntimeError(f"IfcConvert failed: {result.stderr}")
        except subprocess.TimeoutExpired:
            print(f"IfcConvert timeout for {self.ifc_guid}")
        except FileNotFoundError:
            print("IfcConvert not found. Please install IfcOpenShell command-line tools.")
        except Exception as e:
            print(f"Error saving mesh with IfcConvert for {self.element_type} {self.ifc_guid}: {e}")

    
    def check_assembly(self) -> List[Any]:
        """ Check whether the IFCElement is an assembly and return its parts. """
        aggregated_parts = []
        # Check composition relationships (IfcRelAggregates)
        if hasattr(self.ifc_description, 'IsDecomposedBy') and self.ifc_description.IsDecomposedBy:
            for rel in self.ifc_description.IsDecomposedBy:
                if rel.is_a('IfcRelAggregates'):
                    for related_object in rel.RelatedObjects:
                        aggregated_parts.append(related_object)
        return aggregated_parts


    def check_part_of_assembly(self) -> Optional[Any]:
        """ Check whether the IFCElement is part of an assembly and returns the parent element. """
        parent_element = None
        # Check if the child element has a "Decomposes" relationship
        if hasattr(self.ifc_description, 'Decomposes') and self.ifc_description.Decomposes:
            for rel in self.ifc_description.Decomposes:
                if rel.is_a('IfcRelAggregates'):
                    parent_element = rel.RelatingObject
                    break  # Typically only one parent per child
        return parent_element
    

    def is_door(self) -> bool:
        """Check if the element is a door."""
        if self.ifc_description:
            return self.ifc_description.is_a("IfcDoor")
        if self.element_type == "IfcDoor":
            return True
        return False

    def is_walkable(self, height: float, tolerance: float = 0.1) -> bool:
        """Check if you could walk on the element at a given height with small tolerance."""
        if self.ifc_description:
            if (self.ifc_description.is_a("IfcSlab") or
                self.ifc_description.is_a("IfcCovering") or
                self.ifc_description.is_a("IfcWall") or
                self.ifc_description.is_a("IfcWallStandardCase") or
                self.ifc_description.is_a("IfcBeam")):
                if height - tolerance <= self.bbox["max_z"] <= height + tolerance:
                    return True
        if self.element_type in ["IfcSlab", "IfcCovering", "IfcWall", "IfcWallStandardCase", "IfcBeam"]:
            if self.bbox and height - tolerance <= self.bbox["max_z"] <= height + tolerance:
                return True
        return False

    
    def is_ceiling(self, max_z: float) -> bool:
        """Check if the element is part of the ceiling for the given max_z of the floor."""
        if self.ifc_description:
            if self.ifc_description.is_a("IfcSlab") or self.ifc_description.is_a("IfcCovering"):
                if self.bbox["min_z"] >= max_z - 1.0:
                    return True
        if self.element_type in ["IfcSlab", "IfcCovering"]:
            if self.bbox and self.bbox["min_z"] >= max_z - 1.0:
                return True
        return False

   
    def physical_part_of_roof(self) -> bool:
        """Check if the element is a physical part of a roof assembly."""
        if self.ifc_description.is_a('IfcRoof'):
            if self.check_assembly():
                return False
            return True
        parent_element = self.check_part_of_assembly()
        if parent_element and parent_element.is_a('IfcRoof'):
            return True
        return False
    

    def physical_part_of_stair_or_ramp(self) -> bool:
        """ Check if the element is a physical part of a stair or ramp assembly. """
        if self.ifc_description.is_a('IfcStair') or self.ifc_description.is_a('IfcRamp'):
            if self.check_assembly():
                return False
            return True
        parent_element = self.check_part_of_assembly()
        if parent_element and (parent_element.is_a('IfcStair') or parent_element.is_a('IfcRamp')):
            return True
        return False


    def add_to_topology_graph(self, graph: nx.Graph) -> None:
        """Add the element as a node in the given graph."""
        if self.element_type == "IfcSpace":
            return
        if (self.bbox is None or self.polygon is None or 
            self.mesh is None or self.position is None):
            return
        graph.add_node(self.ifc_guid,
                       ifc_guid=self.ifc_guid,
                       ifc_description=self.ifc_description,
                       type=self.element_type,
                       vertices=self.vertices,
                       faces=self.faces,
                       bbox=self.bbox,
                       polygon=self.polygon,
                       mesh=self.mesh,
                       position=self.position,
                       floor=self.floor)


class IFCStairRampElement(IFCElement):
    """
    Represents a stair or ramp element in the IFC model
    (IfcStair, IfcStairFlight, IfcRamp, IfcRampFlight).
    Adds start and end attributes to the IFCElement properties.
    """

    def __init__(self, ifc_description: Any = None, 
                 element_type: str = None, 
                 ifc_guid: str = None, 
                 vertices: Optional[np.ndarray] = None, 
                 faces: Optional[np.ndarray] = None, 
                 bbox: Optional[Dict[str, float]] = None, 
                 polygon: Optional[Any] = None,
                 mesh: Optional[Any] = None, 
                 position: Optional[Dict[str, float]] = None, 
                 floor: List[str] = None,
                 room: List[str] = None,
                 start_floor: str = None, 
                 end_floor: str = None,
                 start_room: str = None, 
                 end_room: str = None,
                 start_position: Optional[Dict[str, float]] = None, 
                 end_position: Optional[Dict[str, float]] = None,
                 start_polygon: Optional[Any] = None, 
                 end_polygon: Optional[Any] = None,
                 walking_line: Optional[List[Tuple[float, float]]] = None,
                 mesh_save_path: Optional[str] = None,
                 ifc_file_path: Optional[str] = None,
                 voxel_grid_save_path: Optional[str] = None,
                 resolution: float = 0.05) -> None:
        super().__init__(ifc_description,
                         element_type,
                         ifc_guid,
                         vertices=vertices,
                         faces=faces,
                         bbox=bbox,
                         polygon=polygon,
                         mesh=mesh,
                         position=position,
                         floor=floor,
                         room=room,
                         mesh_save_path=mesh_save_path,
                         ifc_file_path=ifc_file_path,
                         voxel_grid_save_path=voxel_grid_save_path,
                         resolution=resolution)
        self.start_floor = start_floor
        self.end_floor = end_floor
        self.start_room = start_room
        self.end_room = end_room
        self.start_position = start_position
        self.end_position = end_position
        self.start_polygon = start_polygon
        self.end_polygon = end_polygon
        self.walking_line = walking_line
    

    def get_stair_ramp_start_and_end(self) -> Tuple[Optional[Dict[str, float]], Optional[Dict[str, float]], 
        Optional[Polygon], Optional[Polygon], Optional[List[Tuple[float, float]]]]:
        """Extract start and end points for stairs/ramps and the walking line."""
        
        def get_centerline_of_polygon(polygon: Polygon) -> LineString:
            """Get the centerline of a polygon using polyskel and find the longest connected path."""
            
            try:
                # Convert Shapely polygon to polyskel format (list of lists for coordinates)
                exterior_coords = [[float(x), float(y)] for x, y in polygon.exterior.coords[:-1]]  # Remove duplicate last point                
                # Compute straight skeleton (only takes polygon parameter, no holes)
                skeleton = _skeletonize(exterior_coords)
                # Extract skeleton edges (arcs)
                skeleton_lines = []
                for arc in skeleton:
                    # Each arc has source and sinks
                    source = arc.source
                    for sink in arc.sinks:
                        # Create line from source to sink
                        line = LineString([(source.x, source.y), (sink.x, sink.y)])
                        if line.length > 0.01:  # Filter out very short segments
                            skeleton_lines.append(line)
                if not skeleton_lines:
                    raise ValueError("No skeleton segments found")
                # Build a graph from skeleton lines
                G = nx.Graph()
                for line in skeleton_lines:
                    start_pt = (line.coords[0][0], line.coords[0][1])
                    end_pt = (line.coords[1][0], line.coords[1][1])
                    G.add_edge(start_pt, end_pt, weight=line.length, geometry=line)
                
                if G.number_of_nodes() < 2:
                    raise ValueError(f"Insufficient nodes in skeleton graph: {G.number_of_nodes()}")
                
                # Identify endpoints (nodes with degree 1) and inner points (nodes with degree >= 2)
                endpoints = [node for node, degree in G.degree() if degree == 1]
                inner_points = [node for node, degree in G.degree() if degree >= 2]
                num_nodes = G.number_of_nodes()
                max_dist = 0.0
                centerline = None

                # Ensure we have at least 2 endpoints
                if len(endpoints) < 2:
                    raise ValueError(f"Insufficient endpoints: only {len(endpoints)} found in skeleton graph")

                # If we have fewer than 2 inner points, use any two furthest nodes
                elif len(inner_points) < 2 and num_nodes >= 2:
                    # Use all nodes as potential endpoints
                    all_nodes = list(G.nodes())
                    # Find the longest path between any two nodes in the graph
                    for i in range(len(all_nodes)):
                        for j in range(i + 1, len(all_nodes)):
                            try:
                                path = nx.shortest_path(G, source=all_nodes[i], target=all_nodes[j], weight='weight')
                                path_length = sum(G.get_edge_data(path[k], path[k + 1])['weight'] for k in range(len(path) - 1))
                                if path_length > max_dist:
                                    max_dist = path_length
                                    # Build LineString for the centerline
                                    centerline_coords = [(pt[0], pt[1]) for pt in path]
                                    centerline = LineString(centerline_coords)
                            except nx.NetworkXNoPath:
                                continue
                    if max_dist == 0.0:
                        # Graph is completely disconnected, just use first two nodes
                        if len(all_nodes) >= 2:
                            centerline_coords = [(all_nodes[0], all_nodes[1])]
                            centerline = LineString(centerline_coords)
                
                # If we have at least 2 inner points, remove endpoints
                if len(inner_points) >= 2:
                    G.remove_nodes_from(endpoints)
                    # Re-identify endpoints after removal
                    endpoints = [node for node, degree in G.degree() if degree == 1]
                    # Find the longest path between any two endpoints in the graph
                    for i in range(len(endpoints)):
                        for j in range(i + 1, len(endpoints)):
                            try:
                                path = nx.shortest_path(G, source=endpoints[i], target=endpoints[j], weight='weight')
                                # Compute path length
                                path_length = 0.0
                                for k in range(len(path) - 1):
                                    edge_data = G.get_edge_data(path[k], path[k + 1])
                                    path_length += edge_data['weight']
                                if path_length > max_dist:
                                    max_dist = path_length
                                    # Build LineString for the centerline
                                    centerline_coords = [(pt[0], pt[1]) for pt in path]
                                    centerline = LineString(centerline_coords)
                            except nx.NetworkXNoPath:
                                continue
                
                # Fallback: if no path found, use the longest skeleton line
                if centerline is None:
                    if skeleton_lines:
                        centerline = max(skeleton_lines, key=lambda line: line.length)
                    else:
                        raise ValueError("No valid centerline path found")
                
                return centerline
                
            except Exception as e:
                print(f"  Centerline extraction failed: {e}")
                return None
        
        def extend_centerline_to_boundary(line: LineString, polygon, length_factor=3.0):
            """Extend a line in both directions until it hits the polygon boundary."""

            def find_segment_midpoint_for_point(pt: Point, polygon):
                """
                Given a Point 'pt' that lies on (or near) polygon.boundary, find the polygon
                boundary segment (pair of vertices) that was intersected, and return the
                midpoint of that segment. Searches exterior and interiors. Returns None if no segment is found.
                """

                # Build a list of linear rings to search (exterior first, then interiors)
                rings = [polygon.exterior] + list(polygon.interiors)

                # tolerance: small fraction of polygon size
                minx, miny, maxx, maxy = polygon.bounds
                diag = np.hypot(maxx - minx, maxy - miny)
                tol = diag * 1e-6 if diag > 0 else 1e-6

                best_seg = None
                best_dist = float("inf")

                for ring in rings:
                    coords = list(ring.coords)  # closed ring (first==last)
                    # iterate over segments as consecutive coordinate pairs
                    for i in range(len(coords) - 1):
                        a = coords[i]
                        b = coords[i + 1]
                        seg = LineString([a, b])

                        # If point lies exactly on segment, distance will be 0 (ideal)
                        d = seg.distance(pt)
                        if d < best_dist:
                            best_dist = d
                            best_seg = (a, b)

                # If the best distance is reasonable, return midpoint
                if best_seg is not None and best_dist <= tol:
                    a, b = best_seg
                    return (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0

                return None

            coords = np.array(line.coords)
            start, end = coords[0], coords[-1]

            # Compute direction vectors for start and end segments
            dir_start = start - coords[1]         # backward direction
            dir_end = end - coords[-2]            # forward direction

            # Normalize
            dir_start /= np.linalg.norm(dir_start)
            dir_end /= np.linalg.norm(dir_end)

            # Create extension points far along those directions
            ext_start_pt = Point(start + dir_start * length_factor)
            ext_end_pt = Point(end + dir_end * length_factor)

            # Create extended line for intersection check
            ext_line = LineString([ext_start_pt, *coords, ext_end_pt])

            # Intersect with polygon boundary
            inters = ext_line.intersection(polygon.boundary)

            # Collect intersection points
            if inters.is_empty:
                print(f"No intersection found for {self.element_type} {self.ifc_guid}")
                return line  # nothing found, keep original

            if inters.geom_type == "MultiPoint":
                inter_points = list(inters.geoms)
            elif inters.geom_type == "Point":
                inter_points = [inters]
            else:
                # Rare: if intersection yields line segments
                inter_points = [p for p in inters.geoms if isinstance(p, Point)]

            if len(inter_points) < 2:
                print(f"Not enough intersections found for {self.element_type} {self.ifc_guid}")
                return line

            # Get start and end intersection points nearest to original line ends
            start_point = min(inter_points, key=lambda p: p.distance(Point(start)))
            end_point = min(inter_points, key=lambda p: p.distance(Point(end)))
            start_mid_point = find_segment_midpoint_for_point(start_point, polygon)
            end_mid_point = find_segment_midpoint_for_point(end_point, polygon)

            if start_mid_point is None or end_mid_point is None:
                return line

            # Construct extended line with midpoints
            extended = LineString([start_mid_point] + list(coords) + [end_mid_point])

            return extended

        def cut_polygon_at_distance(polygon: Polygon, centerline: LineString,
                                    distance: float, from_start=True, length_factor=2.0):
            """
            Cut a polygon perpendicular to a centerline at a specified distance (in meters)
            from either the start or end of the line.
            """

            length = centerline.length
            if length == 0:
                raise ValueError("Centerline has zero length.")

            # Clamp distance to line length
            distance = np.clip(distance, 0, length)

            # Choose where to cut
            if from_start:
                cut_point = centerline.interpolate(distance)
                ref_point = Point(centerline.coords[0])
                neighbor_point = centerline.interpolate(distance + 0.01)
            else:
                cut_point = centerline.interpolate(length - distance)
                ref_point = Point(centerline.coords[-1])
                neighbor_point = centerline.interpolate(length - distance - 0.01)

            # Compute direction vector of the line at the cut point
            v = np.array([neighbor_point.x - cut_point.x,
                        neighbor_point.y - cut_point.y])
            norm = np.linalg.norm(v)
            if norm == 0:
                return None
            v /= norm

            # Rotate 90° to get perpendicular direction
            perp = np.array([-v[1], v[0]])

            # Create a long cutting line through the polygon
            p1 = cut_point.coords[0] + length_factor * perp
            p2 = cut_point.coords[0] - length_factor * perp
            cutting_line = LineString([p1, p2])

            # Split polygon
            result = split(polygon, cutting_line)

            if len(result.geoms) < 2:
                # If splitting failed, return original polygon
                return polygon

            # Choose the half containing the reference point
            for geom in result.geoms:
                if geom.contains(ref_point):
                    return geom

            # Fallback: if no geom contains ref_point, return the smallest half
            return min(result.geoms, key=lambda g: g.area)


        ifc_type = self.ifc_description.is_a()
        if not (self.physical_part_of_stair_or_ramp() and 
                ifc_type in ["IfcStair", "IfcStairFlight", "IfcRamp", "IfcRampFlight"]):
            return None, None, None, None, None

        try:
            # Check if we have the necessary geometry data
            if self.vertices is None or len(self.vertices) == 0:
                print(f"No vertices available for {self.element_type} {self.ifc_guid}")
                return None, None, None, None, None

            # Use centerline extraction from polygon to get the main direction
            centerline = get_centerline_of_polygon(self.polygon)
            if centerline is None:
                print(f"Failed to extract centerline for {self.element_type} {self.ifc_guid}")
                return None, None, None, None, None

            # Extend the centerline to the polygon boundary
            extended_line = extend_centerline_to_boundary(centerline, self.polygon)

            # Simplify the walking line
            simplified_line = extended_line.simplify(tolerance=0.05, preserve_topology=True)

            # Get the start and end polygons by cutting the main polygon
            start_polygon = cut_polygon_at_distance(self.polygon, simplified_line, distance=0.4, from_start=True)
            end_polygon = cut_polygon_at_distance(self.polygon, simplified_line, distance=0.4, from_start=False)

            # Analyze z coordinates of vertices in the start and end polygons
            points_3d = self.vertices
            vertices_in_start_polygon = []
            vertices_in_end_polygon = []
            buffered_start_polygon = start_polygon.buffer(0.01)
            buffered_end_polygon = end_polygon.buffer(0.01)
            for point in points_3d:
                if buffered_start_polygon.contains(Point(point[0], point[1])):
                    vertices_in_start_polygon.append(point)
                if buffered_end_polygon.contains(Point(point[0], point[1])):
                    vertices_in_end_polygon.append(point)
            # Get mean, min and max z-values for start and end points
            if len(vertices_in_start_polygon) == 0 or len(vertices_in_end_polygon) == 0:
                print(f"No vertices found in start or end polygon for {self.element_type} {self.ifc_guid}")
                return None, None, None, None, None
            start_z_mean = np.mean([v[2] for v in vertices_in_start_polygon])
            end_z_mean = np.mean([v[2] for v in vertices_in_end_polygon])
            start_z_min = np.min([v[2] for v in vertices_in_start_polygon])
            start_z_max = np.max([v[2] for v in vertices_in_start_polygon])
            end_z_min = np.min([v[2] for v in vertices_in_end_polygon])
            end_z_max = np.max([v[2] for v in vertices_in_end_polygon])

            # Compare the mean z-values to determine which end is start and which is end
            if start_z_mean <= end_z_mean:
                start_point = {"x": round(simplified_line.coords[0][0], 3),
                               "y": round(simplified_line.coords[0][1], 3),
                               "z": round(start_z_min, 3)}
                end_point = {"x": round(simplified_line.coords[-1][0], 3),
                             "y": round(simplified_line.coords[-1][1], 3),
                             "z": round(end_z_max, 3)}
                # Extract and round coords in walking line
                simplified_line = [(round(c[0], 3), round(c[1], 3)) for c in simplified_line.coords]
                #self._visualize_stair_ramp(start_point, end_point, start_polygon, end_polygon, simplified_line)
                return start_point, end_point, start_polygon, end_polygon, simplified_line
            else:
                # Swap start and end
                start_point = {"x": round(simplified_line.coords[-1][0], 3),
                               "y": round(simplified_line.coords[-1][1], 3),
                               "z": round(end_z_min, 3)}
                end_point = {"x": round(simplified_line.coords[0][0], 3),
                             "y": round(simplified_line.coords[0][1], 3),
                             "z": round(start_z_max, 3)}
                # Reverse and round coords in the walking line
                simplified_line = [(round(c[0], 3), round(c[1], 3)) for c in simplified_line.coords[::-1]]
                #self._visualize_stair_ramp(start_point, end_point, end_polygon, start_polygon, simplified_line)
                return start_point, end_point, end_polygon, start_polygon, simplified_line

        except Exception as e:
            print(f"Error extracting start/end points for {self.element_type} {self.ifc_guid}: {e}")
            return None, None, None, None, None
    
    def _visualize_stair_ramp(self, start_point, end_point, start_polygon, end_polygon, simplified_line) -> None:
        """Visualize the stair/ramp geometry in 2D using matplotlib."""
        import matplotlib
        matplotlib.use('Agg')  # Use non-interactive backend
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(12, 10))
        
        # Helper function to plot a polygon
        def plot_polygon(poly, color, alpha, label):
            if poly is not None and not poly.is_empty:
                x, y = poly.exterior.xy
                ax.fill(x, y, color=color, alpha=alpha, label=label)
                ax.plot(x, y, color=color, linewidth=2)

        # Plot main stair/ramp polygon
        plot_polygon(self.polygon, 'blue', 0.3, 'Main Polygon')
        # Plot start polygon
        plot_polygon(start_polygon, 'green', 0.5, 'Start Polygon')
        # Plot end polygon
        plot_polygon(end_polygon, 'red', 0.5, 'End Polygon')
        # Plot start point
        if start_point:
            start_x = start_point['x']
            start_y = start_point['y']
            ax.plot(start_x, start_y, 'go', markersize=12, label='Start Point', zorder=5)
            ax.annotate('START', (start_x, start_y), xytext=(10, 10), 
                       textcoords='offset points', fontsize=10, fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.5', facecolor='green', alpha=0.7))
        # Plot end point
        if end_point:
            end_x = end_point['x']
            end_y = end_point['y']
            ax.plot(end_x, end_y, 'ro', markersize=12, label='End Point', zorder=5)
            ax.annotate('END', (end_x, end_y), xytext=(10, -20), 
                       textcoords='offset points', fontsize=10, fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.5', facecolor='red', alpha=0.7))
        # Plot walking line
        if simplified_line:
            walking_x = [coord[0] for coord in simplified_line]
            walking_y = [coord[1] for coord in simplified_line]
            ax.plot(walking_x, walking_y, 'k-', linewidth=2, label='Walking Line', zorder=4)
            ax.plot(walking_x, walking_y, 'ko', markersize=4, zorder=4)
        # Set labels and title
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.set_title(f'{self.element_type} Geometry - {self.ifc_guid}', fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='datalim')
        
        plt.tight_layout()
        plt.savefig(f'/tmp/stair_ramp_{self.ifc_guid}.png', dpi=150, bbox_inches='tight')
        print(f"  Visualization saved to /tmp/stair_ramp_{self.ifc_guid}.png")
        plt.close(fig)


    def add_to_topology_graph(self, graph: nx.Graph) -> None:
        """Stairs and ramps are added as three nodes in the graph."""
        if (self.bbox is None or self.polygon is None or 
            self.mesh is None or self.position is None or
            self.start_polygon is None or self.start_position is None or self.start_floor is None or
            self.end_polygon is None or self.end_position is None or self.end_floor is None):
            return
        graph.add_node(self.ifc_guid,
                       ifc_guid=self.ifc_guid,
                       ifc_description=self.ifc_description,
                       type=self.element_type,
                       vertices=self.vertices,
                       faces=self.faces,
                       bbox=self.bbox,
                       polygon=self.polygon,
                       mesh=self.mesh,
                       position=self.position,
                       walking_line=self.walking_line)
        graph.add_node(self.ifc_guid + "_start",
                       ifc_guid=self.ifc_guid,
                       ifc_description=self.ifc_description,
                       type=self.element_type,
                       polygon=self.start_polygon,
                       position=self.start_position,
                       floor=self.start_floor)
        graph.add_node(self.ifc_guid + "_end",
                       ifc_guid=self.ifc_guid,
                       ifc_description=self.ifc_description,
                       type=self.element_type,
                       polygon=self.end_polygon,
                       position=self.end_position,
                       floor=self.end_floor)


