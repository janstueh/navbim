from .floor import Floor, StairSlabFloor
from .generation_utils import valid_name
from typing import Dict, Any, List, Tuple, Optional, TYPE_CHECKING
import collections
import math
import yaml
import os

if TYPE_CHECKING:
    from .ifc import IFCElement


class FloorManager:
    """Manages floor-related operations."""

    def __init__(self, vertical_tolerance: float = 0.3) -> None:
        self.vertical_tolerance = vertical_tolerance  # Tolerance for elevation differences


    def extract_floors(self, elements: Dict[str, 'IFCElement']) -> Dict[str, 'Floor']:
        """Extract floors based on the surface area and elevation of IfcSlabs and IfcBuildingStorey."""
        
        # Get building storeys from IFC model first
        ifc_storeys = collections.OrderedDict() # Store storeys by elevation
        for guid, element in elements.items():
            if element.element_type == "IfcBuildingStorey":
                storey = element.ifc_description
                elevation = storey.Elevation if hasattr(storey, 'Elevation') else 0.0
                ifc_storeys[elevation] = storey
        # Sort the dictionary by elevation
        ifc_storeys = collections.OrderedDict(sorted(ifc_storeys.items()))

        # Get the highest point of the building
        highest_point = -math.inf
        for element in elements.values():
            if hasattr(element, 'bbox') and element.bbox:
                max_z = element.bbox["max_z"]
                if max_z > highest_point:
                    highest_point = max_z

        # Analyze slab elevations for both regular and stair slab floors
        regular_floors, stair_slab_data = self.analyze_slab_elevations(elements, highest_point)

        # Combine IFC and analyzed elevations for regular floors, and map polygons
        final_floors = self.combine_floor_elevations(
            ifc_storeys, regular_floors)

        # Create floors
        floors = {}
        print(f"Detected floors:")

        # Create regular Floor objects
        for i, (elevation, floor_polygon) in enumerate(final_floors.items()):
            # Use IFC name if available, otherwise generate a name
            floor_name = f"Floor_{i}"
            for ifc_elevation in ifc_storeys.keys():
                if 0.0 <= elevation - ifc_elevation < self.vertical_tolerance:
                    floor_name = valid_name(ifc_storeys[ifc_elevation].Name)
                    break
            
            # Determine min and max elevations
            min_elevation = elevation
            final_elevations_list = list(final_floors.keys())
            if i < len(final_elevations_list) - 1:
                max_elevation = final_elevations_list[i + 1]
            else:
                max_elevation = highest_point
            
            # Create regular Floor object
            floor = Floor(
                floor_name=floor_name,
                min_z=min_elevation,
                max_z=max_elevation,
                polygon=floor_polygon
            )
            
            floors[floor_name] = floor
            print(f"  {floor_name} at {elevation:.2f} m (range: {min_elevation:.2f} - {max_elevation:.2f})")
        
        # Create StairSlabFloor objects
        stair_floors = self.create_stair_slab_floors(stair_slab_data, floors)
        floors.update(stair_floors)
        
        print(f"Total floors detected: {len(floors)}")
        return floors


    def analyze_slab_elevations(self, elements: Dict[str, 'IFCElement'], 
                                highest_point: float) -> Tuple[collections.OrderedDict, collections.OrderedDict]:
        """Analyze slab elevations to detect floor levels for both regular and stair floors."""
        
        # Extract elevation and polygon of floor and stair/ramp slabs in parallel
        floor_slab_data = collections.OrderedDict()  # elevation -> polygon
        stair_slab_data = collections.OrderedDict()  # elevation -> {'elements': List, 'polygon': Polygon}
        
        for guid, element in elements.items():
            if element.element_type == "IfcSlab":
                # Ignore roof slabs
                if hasattr(element.ifc_description, "PredefinedType") and element.ifc_description.PredefinedType == "ROOF":
                    continue
                if hasattr(element, 'bbox') and element.bbox and hasattr(element, 'polygon') and element.polygon:
                    elevation = element.bbox["max_z"]
                    polygon = element.polygon
                    
                    # Check if it's part of stairs/ramps or regular floors
                    if element.physical_part_of_stair_or_ramp():
                        # Store stair/ramp slabs with their elements for later processing
                        if elevation not in stair_slab_data:
                            stair_slab_data[elevation] = {'elements': [element], 'polygon': polygon}
                        else:
                            # Union of polygons
                            stair_slab_data[elevation]['polygon'] = stair_slab_data[elevation]['polygon'].buffer(0).union(polygon.buffer(0))
                            stair_slab_data[elevation]['elements'].append(element)
                        
                    else:
                        # Process regular floor slabs
                        if elevation not in floor_slab_data:
                            floor_slab_data[elevation] = polygon
                        else:
                            # Union of polygons
                            floor_slab_data[elevation] = floor_slab_data[elevation].buffer(0).union(polygon.buffer(0))    

        # Extract regular floor elevations based on area analysis
        # Also consolidate polygons when merging close elevations
        regular_floors = collections.OrderedDict()  # elevation -> polygon
        
        if floor_slab_data:
            floor_slab_data = collections.OrderedDict(
                sorted(floor_slab_data.items()))
            
            previous_elevation = None
            for elevation, current_polygon in floor_slab_data.items():
                if elevation + self.vertical_tolerance > highest_point:
                    break  # Floors need a minimum height
                
                if previous_elevation is None:
                    # First floor
                    previous_elevation = elevation
                    if current_polygon:
                        regular_floors[elevation] = current_polygon
                        
                elif elevation - previous_elevation < self.vertical_tolerance:
                    # Close elevations - check if area is significant
                    last_elevation = list(regular_floors.keys())[-1] if regular_floors else None
                    if last_elevation and current_polygon.area / floor_slab_data[last_elevation].area > 0.1:
                        # Update to higher elevation and union the polygons
                        old_elevation = last_elevation
                        old_polygon = regular_floors.pop(old_elevation)  # Remove old elevation
                        
                        # Union polygons from old and new elevation
                        if current_polygon and old_polygon:
                            try:
                                regular_floors[elevation] = old_polygon.buffer(0).union(current_polygon.buffer(0))
                            except Exception as e:
                                print(f"  Warning: Failed to union polygons at elevations {old_elevation:.2f} and {elevation:.2f}: {e}")
                                regular_floors[elevation] = current_polygon
                        elif current_polygon:
                            regular_floors[elevation] = current_polygon
                        elif old_polygon:
                            regular_floors[elevation] = old_polygon
                    previous_elevation = elevation
                    
                else:
                    # New distinct floor
                    previous_elevation = elevation
                    if current_polygon:
                        regular_floors[elevation] = current_polygon
        
        # Sort stair slab data for consistent processing
        if stair_slab_data:
            stair_slab_data = collections.OrderedDict(
                sorted(stair_slab_data.items()))

        return regular_floors, stair_slab_data


    def combine_floor_elevations(self, ifc_elevations: collections.OrderedDict, 
                                 regular_floors: collections.OrderedDict) -> collections.OrderedDict:
        """Combine elevations from IFC and analysis, preferring IFC names.
        Also maps consolidated polygons from analyzed elevations to final elevations.
        """
        
        ifc_elevation_list = list(ifc_elevations.keys())
        
        if not ifc_elevation_list:
            return regular_floors
        
        # Map analyzed elevations to closest IFC elevations and track polygons
        # Prefer analyzed elevation if it's higher than IFC elevation within tolerance
        mapped_elevations = {}
        final_floors = collections.OrderedDict()
        
        for analyzed_elevation, polygon in regular_floors.items():
            for ifc_elevation in ifc_elevation_list:
                if abs(analyzed_elevation - ifc_elevation) <= self.vertical_tolerance:
                    # Use analyzed elevation if it's higher, otherwise use IFC elevation
                    if analyzed_elevation > ifc_elevation:
                        # Keep the analyzed elevation (don't map to IFC)
                        if analyzed_elevation not in mapped_elevations:
                            mapped_elevations[analyzed_elevation] = analyzed_elevation
                            final_floors[analyzed_elevation] = polygon
                    else:
                        # Use IFC elevation if analyzed is lower or equal
                        if (analyzed_elevation not in mapped_elevations or
                            abs(analyzed_elevation - ifc_elevation) < 
                            abs(analyzed_elevation - mapped_elevations[analyzed_elevation])):
                            mapped_elevations[analyzed_elevation] = ifc_elevation
                            final_floors[ifc_elevation] = polygon
                    break
        
        # Handle unmapped analyzed elevations
        for analyzed_elevation, polygon in regular_floors.items():
            if analyzed_elevation not in mapped_elevations:
                # Not mapped to any IFC elevation, keep as-is
                final_floors[analyzed_elevation] = polygon
        
        # Add IFC elevations not close to any analyzed elevation (no polygon)
        for ifc_elevation in ifc_elevation_list:
            if not any(abs(analyzed_elevation - ifc_elevation) <= self.vertical_tolerance 
                      for analyzed_elevation in regular_floors.keys()):
                if ifc_elevation not in final_floors:
                    print(f"  Warning: IFC floor at elevation {ifc_elevation:.2f}m has no matching slab data - polygon will be None")
                    final_floors[ifc_elevation] = None
        
        # Sort by elevation and return
        return collections.OrderedDict(sorted(final_floors.items()))


    def create_stair_slab_floors(self, stair_slab_data: collections.OrderedDict,
                                   main_floors: Dict[str, 'Floor']) -> Dict[str, 'StairSlabFloor']:
        """Create StairSlabFloor objects with unionized polygons from stair/ramp slabs.
        
        Args:
            stair_slab_data: OrderedDict mapping elevations to {'elements': List, 'polygon': Polygon}
            main_floors: Dictionary of main Floor objects for parent floor association
            
        Returns:
            Dictionary of StairSlabFloor objects keyed by floor name
        """
        
        stair_floors = {}
        floor_counters = {}
        
        # Get all regular floor elevations for filtering
        regular_floor_elevations = [floor.min_z for floor in main_floors.values()]
        
        # Process all stair slab elevations
        for elevation, data in stair_slab_data.items():
            # Filter out stair slab elevations that are too close to regular floor elevations
            too_close_to_regular_floor = False
            for regular_elevation in regular_floor_elevations:
                if abs(elevation - regular_elevation) <= self.vertical_tolerance:
                    too_close_to_regular_floor = True
                    break
            if too_close_to_regular_floor:
                continue
            
            # Use pre-computed unionized polygon from analyze_slab_elevations
            stair_polygon = data['polygon']
                
            # Find the main floor this elevation belongs to
            parent_floor = None
            parent_floor_name = None
            for floor_name, floor in main_floors.items():
                if floor.min_z <= elevation < floor.max_z:
                    parent_floor = floor
                    parent_floor_name = floor_name
                    break
            if parent_floor:
                # Create unique stair/ramp floor name
                if parent_floor_name not in floor_counters:
                    floor_counters[parent_floor_name] = 0
                stair_floor_name = f"{parent_floor_name}_Stair_{floor_counters[parent_floor_name]}"
                floor_counters[parent_floor_name] += 1
                
                # Create StairSlabFloor object with parent floor name and polygon
                stair_floor = StairSlabFloor(
                    floor_name=stair_floor_name,
                    min_z=elevation,
                    max_z=parent_floor.max_z,
                    parent_floor=parent_floor_name,
                    polygon=stair_polygon
                )
                
                stair_floors[stair_floor_name] = stair_floor
                print(f"  {stair_floor_name} at {elevation:.2f} m (stair/ramp floor, parent: {parent_floor_name})")
        
        return stair_floors


    def assign_elements_to_floors(self, elements: Dict[str, 'IFCElement'], 
                                  floors: Dict[str, 'Floor'], 
                                  navigation_model_path: str = None) \
                                  -> Tuple[Dict[str, 'IFCElement'], Dict[str, 'Floor']]:
        """
        Assign elements to floors by setting the floor attribute of each IFCElement.
        Elements can be assigned to multiple floors. 
        Stairs and ramps are assigned start and end floors specifically.
        Also creates mesh reference YAML files for each floor for visualization.
        """

        def calculate_overlap(element_min_z: float, element_max_z: float, floor_min_z: float, floor_max_z: float) -> float:
            """Calculate the vertical overlap between an element and a floor."""
            overlap = 0.0
            if element_min_z <= floor_min_z:
                # Element is below the floor
                if element_max_z <= floor_min_z:
                    overlap = 0.0
                # Element overlaps the whole floor
                elif element_max_z >= floor_max_z:
                    overlap = floor_max_z - floor_min_z
                # Partial overlap
                elif element_max_z > floor_min_z:
                    overlap = element_max_z - floor_min_z
            else:
                # Element is above the floor
                if element_min_z >= floor_max_z:
                    overlap = 0.0
                # Floor overlaps the whole element
                elif element_max_z <= floor_max_z:
                    overlap = element_max_z - element_min_z
                # Partial overlap
                elif element_max_z > floor_min_z:
                    overlap = floor_max_z - element_min_z
            return overlap
        
        def find_nearest_floor(height: float) -> Optional[str]:
            """
            Find the nearest floor based on min_z to a given height.
            """
            smallest_diff = float('inf')
            nearest_floor = None
            for name, floor in floors.items():
                diff = abs(floor.min_z - height)
                if diff < smallest_diff:
                    smallest_diff = diff
                    nearest_floor = name
            return nearest_floor

        # Create sorted list of regular floors and stair floors separately
        regular_floors = [(name, floor) for name, floor in floors.items() 
                         if not isinstance(floor, StairSlabFloor)]
        stair_floors = [(name, floor) for name, floor in floors.items() 
                       if isinstance(floor, StairSlabFloor)]
        regular_floors.sort(key=lambda x: x[1].min_z)
        stair_floors.sort(key=lambda x: x[1].min_z)

        # Initialize dict for mesh references
        mesh_references = {}
        for floor_name in floors.keys():
            mesh_references[floor_name] = []
        
        # Assign elements to regular floors based on the bounding box
        for guid, element in elements.items():
            if hasattr(element, 'bbox') and element.bbox:
                element_min_z = element.bbox["min_z"]
                element_max_z = element.bbox["max_z"]
                for floor_name, floor in regular_floors:
                    # Calculate the overlap of the element with the floor
                    overlap = calculate_overlap(element_min_z, element_max_z, floor.min_z, floor.max_z)
                    # Slabs should be also assigned to the floor above using the vertical tolerance
                    if (element.element_type == "IfcSlab" and
                        (overlap > 0.05 or # At least 5cm overlap
                         floor.min_z - self.vertical_tolerance <= element_max_z < floor.max_z)):
                        element.floor.append(floor_name)
                        if not element.is_ceiling(floor.max_z):
                            # Put the mesh in the mesh references
                            mesh_references[floor_name].append(element.mesh_save_path)
                    # Other elements need at least 5 cm overlap
                    elif overlap > 0.05:
                        # Ignore elements that start within the vertical tolerance around max_z
                        if element_min_z >= floor.max_z - self.vertical_tolerance:
                            continue
                        element.floor.append(floor_name)
                        if not element.physical_part_of_roof() and not element.element_type == "IfcSpace":
                            # Put the mesh in the mesh references
                            mesh_references[floor_name].append(element.mesh_save_path)
                    # Elements below the first floor (foundations etc.) are not assigned

                # Check if element is part of a stair assembly
                if element.physical_part_of_stair_or_ramp():
                    # Also assign to stair slab floors
                    for stair_floor_name, stair_floor in stair_floors:
                        if (stair_floor.min_z <= element_min_z < stair_floor.max_z or
                            stair_floor.min_z <= element_max_z < stair_floor.max_z or
                            (element_min_z <= stair_floor.min_z and element_max_z >= stair_floor.max_z)):
                            element.floor.append(stair_floor_name)
                    # Assign the start_floor and end_floor attributes
                    if element.element_type in ["IfcStair", "IfcStairFlight", "IfcRamp", "IfcRampFlight"]:
                        element.start_floor = find_nearest_floor(element_min_z)
                        element.end_floor = find_nearest_floor(element_max_z)
                        
            elif (element.check_assembly() or
                  element.element_type in ["IfcBuildingStorey", "IfcRelSpaceBoundary"]):
                continue
            
            else:
                print(f"  Element {guid} ({element.element_type}) has no bounding box, skipping floor assignment.")

        # Check if floors are unreachable by any stairs or ramps for multistory buildings
        # and remove them if so
        if len(regular_floors) > 1:
            reachable_floors = set()
            for element in elements.values():
                if element.element_type in ["IfcStair", "IfcStairFlight", "IfcRamp", "IfcRampFlight"]:
                    if element.start_floor:
                        reachable_floors.add(element.start_floor)
                    if element.end_floor:
                        reachable_floors.add(element.end_floor)
            
            # Identify floors to remove and process them
            floors_to_remove = []
            for floor_name, floor in list(floors.items()):
                if floor_name not in reachable_floors:
                    floors_to_remove.append((floor_name, floor))
            
            # Process each floor to be removed
            for floor_name, removed_floor in floors_to_remove:
                print(f"{floor_name} is unreachable by any stairs or ramps and will be removed.")
                
                # Find the floor below this one (if any)
                floor_below_name = None
                floor_below = None
                for candidate_name, candidate_floor in floors.items():
                    if (candidate_name != floor_name and 
                        candidate_name not in [f[0] for f in floors_to_remove] and
                        candidate_floor.min_z < removed_floor.min_z):
                        if floor_below is None or candidate_floor.min_z > floor_below.min_z:
                            floor_below_name = candidate_name
                            floor_below = candidate_floor
                
                # Shift elements from removed floor to floor below
                if floor_below_name:
                    print(f"  Shifting elements from {floor_name} to floor below: {floor_below_name}")
                    
                    for element in elements.values():
                        if floor_name in element.floor:
                            # Remove from the removed floor
                            element.floor.remove(floor_name)
                            
                            # Add to floor below if not already there
                            if floor_below_name not in element.floor:
                                element.floor.append(floor_below_name)
                                
                                # Update mesh references for floor below
                                if (hasattr(element, 'mesh_save_path') and 
                                    element.mesh_save_path and
                                    not element.physical_part_of_roof() and 
                                    element.element_type != "IfcSpace"):
                                    mesh_references[floor_below_name].append(element.mesh_save_path)
                    
                    floor_below.max_z = max(floor_below.max_z, removed_floor.max_z)                    
                    print(f"  New {floor_below_name} range: {floor_below.min_z:.2f} - {floor_below.max_z:.2f}")
                
                else:
                    # Remove elements from the removed floor (they become orphaned)
                    for element in elements.values():
                        if floor_name in element.floor:
                            element.floor.remove(floor_name)
                
                # Remove the floor and its mesh references
                del floors[floor_name]
                del mesh_references[floor_name]

        # Create YAML files with mesh references
        for floor_name in floors:
            floor_dir = os.path.join(navigation_model_path, floor_name)
            os.makedirs(floor_dir, exist_ok=True)
            yaml_path = os.path.join(floor_dir, "meshes.yaml")
            with open(yaml_path, 'w') as f:
                yaml.dump({'floor_name': floor_name, 'meshes': mesh_references[floor_name]}, f)
        
        return elements, floors


    
    