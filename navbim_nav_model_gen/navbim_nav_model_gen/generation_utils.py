import re
import psutil
import os
import multiprocessing
import math
from typing import Tuple


class ResourceMonitor:
    """Monitor system resources to optimize parallel processing."""
    
    @staticmethod
    def estimate_floor_memory_usage(elements: dict, 
                                    floors: dict, 
                                    resolution: float = 0.05) -> dict:
        """Estimate memory usage per floor based on element data and voxel calculations."""
        
        floor_memory_estimates = {}
        
        for floor_name, floor in floors.items():
            # Get all elements on this floor
            floor_elements = []
            for element_id, element in elements.items():
                if (hasattr(element, 'floor') and floor_name in element.floor and 
                    hasattr(element, 'bbox') and element.bbox is not None and
                    not (hasattr(element, 'ifc_description') and element.ifc_description.is_a("IfcSpace"))):
                    floor_elements.append(element)
            
            if not floor_elements:
                floor_memory_estimates[floor_name] = 0.1  # Minimal memory for empty floor
                continue
            
            total_voxels = 0
            
            for element in floor_elements:
                bbox = element.bbox
                
                # Calculate bounding box dimensions
                size_x = bbox['max_x'] - bbox['min_x']
                size_y = bbox['max_y'] - bbox['min_y']
                size_z = bbox['max_z'] - bbox['min_z']
                
                # Calculate number of voxels for this element
                voxels_x = max(1, int(size_x / resolution))
                voxels_y = max(1, int(size_y / resolution))
                voxels_z = max(1, int(size_z / resolution))
                
                element_voxels = voxels_x * voxels_y * voxels_z
                total_voxels += element_voxels
            
            bytes_per_voxel = 48
            overhead_factor = 2.5  # Account for intermediate data structures
            
            # Calculate memory in GB
            memory_bytes = total_voxels * bytes_per_voxel * overhead_factor
            memory_gb = max(0.1, memory_bytes / (1024**3))  # Minimum 100MB
            
            floor_memory_estimates[floor_name] = memory_gb
        
        return floor_memory_estimates
    
    @staticmethod
    def get_optimal_floor_processes(elements: dict = None, 
                                    floors: dict = None, 
                                    resolution: float = 0.05) -> int:
        """Calculate optimal number of parallel floor processes based on system resources and actual floor data."""
        
        # Get system specs
        cpu_count = multiprocessing.cpu_count()
        available_memory_gb = psutil.virtual_memory().available / (1024**3)
        
        # Calculate memory requirements based on actual floor data
        if elements is not None and floors is not None:
            floor_memory_estimates = ResourceMonitor.estimate_floor_memory_usage(
                elements=elements, floors=floors, resolution=resolution)

            # Calculate memory statistics
            total_estimated_memory = sum(floor_memory_estimates.values())
            max_floor_memory = max(floor_memory_estimates.values()) if floor_memory_estimates else 0.5
            avg_floor_memory = total_estimated_memory / len(floor_memory_estimates) if floor_memory_estimates else 0.5
            
            # Conservative approach: use max floor memory for process calculation
            memory_per_floor_gb = max_floor_memory
            
            # Also check if we can fit multiple average floors
            avg_processes_by_memory = max(1, int(available_memory_gb * 0.7 / avg_floor_memory))
            max_processes_by_memory = max(1, int(available_memory_gb * 0.7 / memory_per_floor_gb))
            
            # Use the more conservative estimate
            final_memory_processes = min(avg_processes_by_memory, max_processes_by_memory)
            
        else:
            memory_per_floor_gb = 3.0
            
            final_memory_processes = max(1, int(available_memory_gb * 0.7 / memory_per_floor_gb))
            
        # CPU constraint: Do not create more processes than CPU cores
        max_processes_by_cpu = max(1, cpu_count)

        # Floor constraint: Don't create more processes than floors
        max_processes_by_floors = max(1, min(len(floors), cpu_count))

        # Take the most restrictive constraint
        optimal_processes = min(
            final_memory_processes,
            max_processes_by_cpu,
            max_processes_by_floors
        )
        
        return optimal_processes
    
    @staticmethod
    def get_optimal_room_processes(elements: dict = None, 
                                   rooms: dict = None, 
                                   resolution: float = 0.05) -> int:
        """Calculate optimal number of parallel room processes based on system resources and actual room data."""
        
        # Get system specs
        cpu_count = multiprocessing.cpu_count()
        available_memory_gb = psutil.virtual_memory().available / (1024**3)
        
        # Calculate memory requirements based on actual room data
        if elements is not None and rooms is not None:
            room_memory_estimates = {}
            
            for room_id, room in rooms.items():
                # Get all elements in this room
                room_elements = []
                for element_id, element in elements.items():
                    # Check if element is in room
                    if (hasattr(element, 'room') and room_id in element.room and
                        hasattr(element, 'bbox') and element.bbox is not None and
                        not (hasattr(element, 'ifc_description') and element.ifc_description.is_a("IfcSpace"))):
                            room_elements.append(element)
                
                if not room_elements:
                    room_memory_estimates[room_id] = 0.05  # Minimal memory for empty room
                    continue
                
                total_voxels = 0
                
                for element in room_elements:
                    bbox = element.bbox
                    
                    # Calculate bounding box dimensions
                    size_x = bbox['max_x'] - bbox['min_x']
                    size_y = bbox['max_y'] - bbox['min_y']
                    size_z = bbox['max_z'] - bbox['min_z']
                    
                    # Calculate number of voxels for this element
                    voxels_x = max(1, int(size_x / resolution))
                    voxels_y = max(1, int(size_y / resolution))
                    voxels_z = max(1, int(size_z / resolution))
                    
                    element_voxels = voxels_x * voxels_y * voxels_z
                    total_voxels += element_voxels
                
                bytes_per_voxel = 32
                overhead_factor = 2.0
                
                # Calculate memory in GB
                memory_bytes = total_voxels * bytes_per_voxel * overhead_factor
                memory_gb = max(0.05, memory_bytes / (1024**3))  # Minimum 50MB
                
                room_memory_estimates[room_id] = memory_gb
            
            # Calculate memory statistics
            if room_memory_estimates:
                total_estimated_memory = sum(room_memory_estimates.values())
                max_room_memory = max(room_memory_estimates.values())
                avg_room_memory = total_estimated_memory / len(room_memory_estimates)
                
                # Conservative approach: use max room memory for process calculation
                memory_per_room_gb = max_room_memory
                
                # Also check if we can fit multiple average rooms
                avg_processes_by_memory = max(1, int(available_memory_gb * 0.8 / avg_room_memory))
                max_processes_by_memory = max(1, int(available_memory_gb * 0.8 / memory_per_room_gb))
                
                # Use the more conservative estimate
                final_memory_processes = max(1, min(avg_processes_by_memory, max_processes_by_memory))
            else:
                final_memory_processes = max(1, cpu_count // 2)
                
        else:
            memory_per_room_gb = 0.3  # Rooms typically smaller than floors
            
            final_memory_processes = max(1, int(available_memory_gb * 0.8 / memory_per_room_gb))

        # CPU constraint: Do not create more processes than CPU cores
        max_processes_by_cpu = max(1, cpu_count)

        # Room constraint: Don't create more processes than rooms, but allow more than floor processing
        max_processes_by_rooms = max(1, min(len(rooms), cpu_count))

        # Take the most restrictive constraint
        optimal_processes = min(
            final_memory_processes,
            max_processes_by_cpu,
            max_processes_by_rooms
        )
        
        return optimal_processes
    
    @staticmethod
    def get_optimal_element_processes_and_batches(ifc_file_path: str, 
                                                  total_elements: int) \
                                                  -> Tuple[int, int, int]:
        """Calculate optimal number of parallel processes and batch size for geometry processing of elements."""
        # Get system specs
        cpu_count = multiprocessing.cpu_count()
        memory_gb = psutil.virtual_memory().available / (1024**3)
        
        # Get IFC file size
        try:
            ifc_file_size_mb = os.path.getsize(ifc_file_path) / (1024**2)
        except:
            ifc_file_size_mb = 100  # Default assumption
        
        # Estimate memory usage per element (geometry extraction is heavy)
        # Conservative: 32MB per element, 2x file size overhead
        bytes_per_element = 32 * 1024 * 1024  # 32MB
        memory_factor = 2.0 if ifc_file_size_mb > 100 else 1.5
        available_bytes = memory_gb * (1024**3)
        max_processes_by_memory = max(1, int(available_bytes / (bytes_per_element * memory_factor)))
        
        # Performance constraint: Don't create more processes than elements or cpu cores
        max_processes_by_elements = min(total_elements, cpu_count)
        max_processes_by_cpu = max(1, cpu_count)
        
        # Take the most restrictive constraint
        optimal_processes = min(max_processes_by_memory, max_processes_by_elements, max_processes_by_cpu)
        optimal_processes = max(1, optimal_processes)
        
        # Determine batch size
        min_batch_size = 10
        max_batch_size = 50
        batch_size = max(min_batch_size, min(max_batch_size, total_elements // (2*optimal_processes) if optimal_processes > 0 else total_elements))
        # Ensure enough batches for all processes
        batches = math.ceil(total_elements / batch_size) if batch_size > 0 else 1
        if batches < optimal_processes:
            batch_size = max(1, total_elements // optimal_processes)
        batch_size = max(1, batch_size)
        
        return optimal_processes, batch_size, batches

    @staticmethod
    def get_optimal_mesh_processes(ifc_file_path: str, total_elements: int) -> int:
        """Calculate optimal number of parallel IfcConvert processes based on system resources."""
        
        # Get system specs
        cpu_count = multiprocessing.cpu_count()
        memory_gb = psutil.virtual_memory().total / (1024**3)
        
        # Get IFC file size
        try:
            ifc_file_size_mb = os.path.getsize(ifc_file_path) / (1024**2)
        except:
            ifc_file_size_mb = 100  # Default assumption

        # Memory constraint: Each IfcConvert process may load the full IFC file
        # Conservative estimate: each process uses 2-4x file size in RAM
        memory_factor = 3.0 if ifc_file_size_mb > 100 else 2.0
        max_processes_by_memory = max(1, int((memory_gb * 0.7 * 1024) / (ifc_file_size_mb * memory_factor)))
      
        # Performance constraint: Don't create more processes than elements or cpu cores
        max_processes_by_elements = min(total_elements, cpu_count)
        max_processes_by_load = max(1, cpu_count)

        # Take the most restrictive constraint
        optimal_processes = min(
            max_processes_by_memory,
            max_processes_by_elements,
            max_processes_by_load
        )
        
        return optimal_processes


def valid_name(name: str) -> str:
    """Creates a valid name which must be alphanumeric, can contain underscores, 
    and does not start with a number."""
    def replace_umlaut(match):
        umlaut_map = {
            'ü': 'ue', 'ö': 'oe', 'ä': 'ae',
            'Ü': 'Ue', 'Ö': 'Oe', 'Ä': 'Ae',
            'ß': 'ss'
        }
        return umlaut_map[match.group(0)]
    # First replace umlauts
    name = re.sub(r'[üöäÜÖÄß]', replace_umlaut, name)
    # Then replace other non-alphanumeric characters
    name = re.sub(r'[^a-zA-Z0-9_]', '_', name)
    if name and name[0].isdigit():
        name = '_' + name
    return name


def print_progress_bar(iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """Call in a loop to create a terminal progress bar."""
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix}|{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()
