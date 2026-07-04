from .room import Room, IfcRoom, DetectedRoom, CombinedRoom
from .ifc import IFCElement
from .floor import Floor, StairSlabFloor
from .generation_utils import valid_name
from typing import List, Dict, Any, Tuple
from shapely.geometry import Polygon, box
from shapely.ops import unary_union, orient
from matplotlib import pyplot as plt

class RoomManager:
    """Manages room-related operations."""

    def __init__(self, vertical_tolerance: float = 0.3, min_area: float = 2.0) -> None:
        self.rooms = {}
        self.vertical_tolerance = vertical_tolerance
        self.min_area = min_area


    def detect_rooms(self, elements: Dict[str, 'IFCElement'], floors: Dict[str, 'Floor']) \
        -> Tuple[Dict[str, 'Room'], Dict[str, Any]]:
        """Detect rooms from IFC spaces and geometric analysis."""

        # First detect rooms from IFC spaces
        self.detect_ifc_spaces(elements, floors)
        # Detect rooms from wall enclosures
        merged_walls = self.detect_rooms_from_walls(elements, floors)
        # Create rooms for stair/ramp slabs
        self.create_rooms_for_stair_slabs(elements, floors)

        return self.rooms, merged_walls


    def detect_ifc_spaces(self, elements: Dict[str, 'IFCElement'], floors: Dict[str, 'Floor']) -> Dict[str, 'IfcRoom']:
        """Creates rooms based on IfcSpace elements."""

        ifc_spaces = [element for element in elements.values() 
                     if element.element_type == "IfcSpace"]
        print(f"Found {len(ifc_spaces)} IfcSpace elements")
        
        for space in ifc_spaces:
            # Use floor's min_z and max_z for consistent z-coordinates
            # IFC spaces may have their bbox at different heights (e.g. 0.15m above floor)
            position_z = space.position["z"] if space.position else 0.0
            bbox = space.bbox.copy() if space.bbox else {}
            
            # Get the floor object to use its min_z and max_z
            if space.floor and len(space.floor) > 0:
                floor_name = space.floor[0]  # Use first floor if multi-floor
                if floor_name in floors:
                    floor_obj = floors[floor_name]
                    position_z = floor_obj.min_z
                    # Also update bbox to use floor's z-coordinates
                    if bbox:
                        bbox["min_z"] = round(floor_obj.min_z, 3)
                        bbox["max_z"] = round(floor_obj.max_z, 3)
            
            position = {
                "x": round(space.position["x"], 3) if space.position else 0.0,
                "y": round(space.position["y"], 3) if space.position else 0.0,
                "z": round(position_z, 3)
            }

            # Check if name has already been used, append number if so
            room_name = valid_name(space.ifc_description.LongName)
            for existing_rooms in self.rooms.values():
                if existing_rooms.name == room_name:
                    counter = 1
                    while f"{room_name}_{counter}" in [r.name for r in self.rooms.values()]:
                        counter += 1
                    room_name = f"{room_name}_{counter}"
                    break

            room = IfcRoom(name=room_name, 
                           floor=space.floor, 
                           bbox=bbox,  # Use corrected bbox with floor's min_z and max_z
                           polygon=space.polygon, 
                           position=position,  # Use corrected position with floor's min_z
                           ifc_description=space.ifc_description, 
                           ifc_guid=space.ifc_guid)
            self.rooms.update({room.ifc_guid: room})


    def detect_rooms_from_walls(self, elements: Dict[str, 'IFCElement'], 
                                floors: Dict[str, 'Floor'], 
                                proximity_threshold: float = 0.05) -> Dict[str, Any]:
        """Detect rooms based on wall enclosures for floors with missing IfcSpaces."""
        
        print("Detecting rooms from wall enclosures...")

        wall_types = ["IfcWall", "IfcWallStandardCase", "IfcColumn"]
        merged_walls_dict = {}
        for floor_name, floor in floors.items():
            if isinstance(floor, StairSlabFloor):
                # Skip stair/ramp slab floors, they are handled separately
                continue
            print(f"Processing floor: {floor_name}")

            # Get all wall polygons for this floor
            wall_polygons = []
            for element_guid, element in elements.items():
                # Ignore walls that start too far above the floor level
                if (element.element_type in wall_types and 
                    hasattr(element, 'polygon') and element.polygon and
                    hasattr(element, 'floor') and floor_name in element.floor and
                    hasattr(element, 'bbox') and element.bbox["min_z"] <= floor.min_z + 2.0):
                    wall_polygons.append((element_guid, element.polygon))
            if not wall_polygons:
                print(f"  No walls detected on {floor_name}")
                continue
                
            # Merge wall geometries
            merged_walls = self.merge_elements(wall_polygons, proximity_threshold)
            merged_walls_dict[floor_name] = merged_walls # For visualization

            # Find enclosed spaces
            enclosures = self.find_enclosed_spaces(merged_walls)
            if not enclosures:
                print(f"  No enclosures detected on {floor_name}")
                continue
            print(f"  Detected {len(enclosures)} enclosures on {floor_name}")

            # Initialize room counter for unique names
            detected_room_counter = 0
            combined_room_counter = 0
            # Process each enclosure
            for enclosure in enclosures:
                # Check overlap with existing IFC spaces on the same floor
                covering_spaces = []
                for room_id, room in self.rooms.items():
                    if (isinstance(room, IfcRoom) and hasattr(room, 'polygon') and room.polygon and
                        hasattr(room, 'floor') and floor_name in room.floor):
                        # Check if this IfcRoom overlaps with the current enclosure
                        space_covered, enclosure_covered, similarity = self.calculate_overlap_percentage(
                            room.polygon, enclosure)
                        if similarity >= 0.9:
                            # Almost identical - skip this enclosure
                            covering_spaces.append(room)
                            break
                        elif space_covered >= 0.9:
                            # Space is covered by enclosure
                            covering_spaces.append(room)
                        elif enclosure_covered >= 0.9:
                            # Enclosure is covered by space - skip this enclosure
                            covering_spaces.append(room)
                            break

                # No covering spaces found --> Detected room
                if len(covering_spaces) == 0:
                    # Determine attributes
                    room_id = f"{floor_name}_DetectedRoom_{detected_room_counter}"
                    floor_obj = floors[floor_name]
                    room_bbox = self.get_bbox_from_polygon(enclosure, floor_obj)
                    position = {
                        "x": round(enclosure.centroid.x, 3),
                        "y": round(enclosure.centroid.y, 3),
                        "z": round(floor_obj.min_z, 3)
                    }
                    # Create DetectedRoom object
                    detected_room = DetectedRoom(name=room_id, 
                                                 floor=[floor_name], 
                                                 bbox=room_bbox, 
                                                 polygon=enclosure,
                                                 position=position)
                    self.rooms.update({room_id: detected_room})
                    detected_room_counter += 1
                    print(f"    Detected room {room_id} with area {enclosure.area:.2f} m²")
                
                # Multiple spaces covered --> Combined room
                elif len(covering_spaces) > 1:
                    room_id = f"{floor_name}_CombinedRoom_{combined_room_counter}"
                    # Union all space polygons with enclosure
                    new_boundaries = enclosure
                    ifc_rooms_list = []
                    for ifc_room in covering_spaces:
                        if hasattr(ifc_room, 'polygon') and ifc_room.polygon:
                            new_boundaries = new_boundaries.union(ifc_room.polygon)
                            ifc_rooms_list.append(ifc_room)
                            # Set contained_in attribute of IfcRoom
                            ifc_room.contained_in = room_id
                    new_boundaries = new_boundaries.simplify(0.03, preserve_topology=True)

                    # Determine the other attributes
                    floor_obj = floors[floor_name]
                    room_bbox = self.get_bbox_from_polygon(new_boundaries, floor_obj)
                    position = {
                        "x": round(new_boundaries.centroid.x, 3),
                        "y": round(new_boundaries.centroid.y, 3),
                        "z": round(floor_obj.min_z, 3)
                    }
                    # Create CombinedRoom object
                    combined_room = CombinedRoom(name=room_id, 
                                                 floor=[floor_name], 
                                                 bbox=room_bbox, 
                                                 polygon=new_boundaries,
                                                 position=position,
                                                 ifc_rooms=ifc_rooms_list)
                    self.rooms[room_id] = combined_room
                    combined_room_counter += 1
                    print(f"    Detected combined room {room_id} with area {new_boundaries.area:.2f} m² enclosing:")
                    for ifc_room in ifc_rooms_list:
                        print(f"      - {ifc_room.name} with guid {ifc_room.ifc_guid} and area {ifc_room.polygon.area:.2f} m²")

                # For single space case, we'll leave existing IFC spaces as-is
                else:
                    print(f"    Allocated room to IfcSpace {room.name} with guid {room.ifc_guid} and area {room.polygon.area:.2f} m²")

        return merged_walls_dict

    
    def merge_elements(self, elements: List[Tuple[str, Any]], proximity_threshold: float = 0.05) -> Any:
        """Merge elements based on proximity."""
        if not elements:
            return None
        # Buffer elements slightly to connect nearby segments
        buffered_elements = [element[1].buffer(proximity_threshold, join_style=2) for element in elements]
        # Merge overlapping elements
        merged_elements = unary_union(buffered_elements)
        # Remove buffer and roundings using mitre join style
        return merged_elements.buffer(-proximity_threshold, join_style=2)

    
    def find_enclosed_spaces(self, merged_walls: Any) -> List[Any]:
        """Find enclosed spaces directly from wall lines or merged walls."""
        
        # Create bounding box with larger padding
        minx, miny, maxx, maxy = merged_walls.bounds
        padding = 10  # Increase padding to better exclude outer walls
        floor_bounds = box(minx-padding, miny-padding, maxx+padding, maxy+padding)
        
        # The difference between the floor bounds and the walls gives us the rooms
        inverse_geometry = floor_bounds.difference(merged_walls)
        
        # Extract valid polygons with holes
        valid_rooms = []
        
        if inverse_geometry.geom_type == 'Polygon':
            # Single polygon case, remove small holes from e.g. columns
            inverse_geometry = self.remove_small_holes(inverse_geometry, self.min_area)
            if inverse_geometry.area > self.min_area and not inverse_geometry.equals(floor_bounds):
                # Add a strict check that polygon doesn't touch the boundary
                if not inverse_geometry.touches(floor_bounds):
                    valid_rooms.append(inverse_geometry)
        elif inverse_geometry.geom_type == 'MultiPolygon':
            # Extract all valid polygons first
            valid_polys = []
            for poly in inverse_geometry.geoms:
                # More strict filtering: must not touch or be near boundary
                if (poly.area > self.min_area and 
                    not poly.equals(floor_bounds) and 
                    not poly.touches(floor_bounds) and
                    # Add distance check to ensure rooms are well inside boundary
                    poly.distance(floor_bounds.boundary) > 1.0):
                    # Remove small holes from e.g. columns
                    cleaned_poly = self.remove_small_holes(poly, self.min_area)
                    valid_polys.append(cleaned_poly)
                    
            # Sort by area (largest first)
            valid_polys.sort(key=lambda x: x.area, reverse=True)
            
            # Keep track of which polygons we've processed
            processed = set()
            
            # Check for containment relationships
            for i, poly in enumerate(valid_polys):
                if i in processed:
                    continue
                    
                processed.add(i)
                
                # Find all polygons that this one contains
                holes = []
                for j, potential_hole in enumerate(valid_polys):
                    if i == j or j in processed:
                        continue
                        
                    # Use covers instead of contains for better handling of shared boundaries
                    if poly.covers(potential_hole) and not poly.equals(potential_hole):
                        holes.append(potential_hole)
                        processed.add(j)
                
                if holes:
                    # Create a new polygon with holes, ensuring proper orientation
                    try:                        
                        # Ensure exterior ring is counter-clockwise
                        exterior_coords = list(orient(poly).exterior.coords)
                        
                        # Ensure interior rings (holes) are clockwise
                        interior_coords = []
                        for hole in holes:
                            oriented_hole = orient(hole, sign=-1.0)  # Clockwise orientation
                            interior_coords.append(list(oriented_hole.exterior.coords))
                        
                        # Create the polygon with holes
                        new_poly = Polygon(exterior_coords, interior_coords)
                        
                        # Validate before adding
                        if new_poly.is_valid:
                            valid_rooms.append(new_poly)
                        else:
                            print(f"Warning: Created invalid polygon with holes, using original")
                            valid_rooms.append(poly)
                    except Exception as e:
                        print(f"Error creating polygon with holes: {e}")
                        valid_rooms.append(poly)
                else:
                    valid_rooms.append(poly)
        
        return valid_rooms

    
    def remove_small_holes(self, polygon: Any, min_hole_area: float = 1.0) -> Any:
        """Remove holes (interior rings) from a polygon if their area is below threshold."""
        if not polygon.interiors:  # No holes to process
            return polygon
            
        # Get the exterior coordinates
        exterior_coords = list(polygon.exterior.coords)
        
        # Filter interior rings based on area
        kept_interiors = []
        for interior in polygon.interiors:
            # Create a polygon from this hole to calculate its area
            hole_poly = Polygon(interior)
            if hole_poly.area >= min_hole_area:
                kept_interiors.append(list(interior.coords))
                
        # Create new polygon with filtered holes
        if kept_interiors:
            return Polygon(exterior_coords, kept_interiors)
        else:
            return Polygon(exterior_coords)  # No holes remain

    
    def calculate_overlap_percentage(self, polygon1: Any, polygon2: Any) -> List[float]:
        """Calculate overlap percentage between two polygons."""
        # Ensure polygons are valid
        if not polygon1.is_valid:
            polygon1 = polygon1.buffer(0)
        if not polygon2.is_valid:
            polygon2 = polygon2.buffer(0)
            
        # Calculate areas
        area1 = polygon1.area
        area2 = polygon2.area
        
        # Calculate intersection
        intersection = polygon1.intersection(polygon2)
        intersection_area = intersection.area
        
        # Calculate union
        union = polygon1.union(polygon2)
        union_area = union.area
        
        # Calculate percentages
        polygon1_covered = (intersection_area / area1) if area1 > 0 else 0
        polygon2_covered = (intersection_area / area2) if area2 > 0 else 0
        jaccard_index = (intersection_area / union_area) if union_area > 0 else 0
        
        return [polygon1_covered, polygon2_covered, jaccard_index]

    
    def get_bbox_from_polygon(self, polygon: Any, floor_obj: Any) -> Dict[str, float]:
        """Get bounding box from polygon for a specific floor."""
        min_x, min_y, max_x, max_y = polygon.bounds
        
        return {
            "min_x": round(min_x, 3),
            "max_x": round(max_x, 3),
            "min_y": round(min_y, 3),
            "max_y": round(max_y, 3),
            "min_z": round(floor_obj.min_z, 3),
            "max_z": round(floor_obj.max_z, 3)
        }
    

    def create_rooms_for_stair_slabs(self, elements: Dict[str, 'IFCElement'], 
                                     floors: Dict[str, 'Floor']) -> None:
        """Create rooms for stair/ramp slabs."""
        for floor_name, floor in floors.items():
            if not isinstance(floor, StairSlabFloor):
                continue
            slab_nr = 0
            for element_guid, element in elements.items():
                if (element.element_type == "IfcSlab" and 
                    element.physical_part_of_stair_or_ramp() and
                    hasattr(element, 'polygon') and element.polygon and
                    hasattr(element, 'floor') and floor_name in element.floor):
                    # Create a room for this slab
                    room_id = f"{floor_name}_Slab_{slab_nr}"
                    room_bbox = self.get_bbox_from_polygon(element.polygon, floor)
                    position = {
                        "x": round(element.polygon.centroid.x, 3),
                        "y": round(element.polygon.centroid.y, 3),
                        "z": round(floor.min_z, 3)
                    }
                    # Create DetectedRoom object
                    stair_slab_room = DetectedRoom(name=room_id, 
                                                   floor=[floor_name], 
                                                   bbox=room_bbox, 
                                                   polygon=element.polygon,
                                                   position=position)
                    self.rooms.update({room_id: stair_slab_room})
                    slab_nr += 1


    def visualize_room_detection(self, floor_name: str, merged_walls: Any, 
                                floor_rooms: Dict[str, Any]) -> None:
        """Visualize room detection results for a specific floor."""
        
        # Create figure
        fig, ax = plt.subplots(figsize=(15, 12))
        
        # Plot walls in black (if available)
        if merged_walls is not None:
            if merged_walls.geom_type == 'Polygon':
                x, y = merged_walls.exterior.xy
                ax.fill(x, y, color='black', alpha=1.0)
            elif merged_walls.geom_type == 'MultiPolygon':
                for poly in merged_walls.geoms:
                    x, y = poly.exterior.xy
                    ax.fill(x, y, color='black', alpha=1.0)

        # Plot rooms for this floor
        room_items = [(room_id, room) for room_id, room in floor_rooms.items() 
                      if hasattr(room, 'floor') and floor_name in room.floor]
        
        # Sort rooms by area for consistent coloring
        room_items.sort(key=lambda x: x[1].polygon.area if hasattr(x[1], 'polygon') and x[1].polygon else 0, reverse=True)
        
        # Plot rooms
        combined_room_labels = []  # Store combined room labels to draw last
        
        for i, (room_id, room) in enumerate(room_items):
            if not hasattr(room, 'polygon') or not room.polygon:
                continue
                
            color = plt.cm.tab10(i % 10)  # Use tab10 colormap for colors
            
            boundaries = room.polygon
            room_type = type(room).__name__  # Get the class name (IfcRoom, DetectedRoom, etc.)
            
            # Plot the exterior boundary
            x, y = boundaries.exterior.xy
            ax.fill(x, y, color=color, alpha=0.6)
            
            # Plot holes if any exist
            if hasattr(boundaries, 'interiors') and boundaries.interiors:
                for interior in boundaries.interiors:
                    x_hole, y_hole = zip(*interior.coords)
                    ax.fill(x_hole, y_hole, color='white', alpha=0.7)
                    ax.plot(x_hole, y_hole, 'k--', linewidth=1)
            
            # Special handling for CombinedRooms - show enclosed IfcRooms
            if room_type == 'CombinedRoom' and hasattr(room, 'ifc_rooms'):
                for j, ifc_room in enumerate(room.ifc_rooms):
                    if hasattr(ifc_room, 'polygon') and ifc_room.polygon:
                        # Plot individual IfcRoom polygon with dotted outline
                        ifc_x, ifc_y = ifc_room.polygon.exterior.xy
                        ax.plot(ifc_x, ifc_y, '--', color=color, linewidth=2, alpha=0.8)
                
                # Store combined room label to draw last (on top)
                if hasattr(room, 'position') and room.position:
                    position = room.position
                else:
                    centroid = boundaries.centroid
                    position = {
                        "x": round(centroid.x, 3),
                        "y": round(centroid.y, 3),
                        "z": 0.0  # Default for visualization
                    }
                
                # Use room color for label background
                label_color = [color[0], color[1], color[2], 0.8]  # Same color with alpha
                combined_room_labels.append((position, room_id, label_color))
            
            else:
                # Add room label for non-combined rooms
                if hasattr(room, 'position') and room.position:
                    position = room.position
                else:
                    # Fallback to polygon centroid
                    centroid = boundaries.centroid
                    position = {
                        "x": round(centroid.x, 3),
                        "y": round(centroid.y, 3),
                        "z": 0.0  # Default for visualization
                    }
                
                # Create appropriate label based on room type
                if room_type == 'IfcRoom':
                    # For IfcRooms, show name above ifc_guid
                    ifc_name = getattr(room, 'name', 'Unknown')
                    ifc_guid = getattr(room, 'ifc_guid', room_id)
                    label_text = f"{ifc_name}\n{ifc_guid}"
                else:
                    # For DetectedRoom, show room_id
                    label_text = room_id
                
                # Use room color for label background
                label_color = [color[0], color[1], color[2], 0.8]  # Same color with alpha
                ax.text(position["x"], position["y"], label_text, 
                       ha='center', va='center', fontsize=10, 
                       bbox=dict(boxstyle="round,pad=0.3", facecolor=label_color, alpha=0.8))
        
        # Draw combined room labels last so they appear on top
        for position, room_id, label_color in combined_room_labels:
            ax.text(position["x"], position["y"], room_id, 
                   ha='center', va='center', fontsize=12, weight='bold',
                   bbox=dict(boxstyle="round,pad=0.4", facecolor=label_color, alpha=0.9))
        
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f'Room Detection Results on {floor_name}')
        plt.tight_layout()
        plt.show()
