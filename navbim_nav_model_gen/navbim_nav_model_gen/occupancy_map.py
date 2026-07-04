import matplotlib.pyplot as plt
import yaml


class OccupancyMap:
    """Represents a 2D occupancy map for a room or floor."""
    
    def __init__(self, occupancy_grid=None, 
                 resolution=None, 
                 origin=None, 
                 floor=None, 
                 room=None):
        self.occupancy_grid = occupancy_grid
        self.resolution = resolution
        self.origin = origin
        self.floor = floor
        self.room = room
    
    def save_map(self, nav_model_path):
        """Save the occupancy map as two files (.png + .yaml)."""
        name = self.room if self.room else self.floor
        if self.occupancy_grid is not None:
            # Validate occupancy grid before saving
            if self.occupancy_grid.size == 0:
                print(f"Warning: Empty occupancy grid for {name}, skipping save")
                return
            
            if self.occupancy_grid.shape[0] == 0 or self.occupancy_grid.shape[1] == 0:
                print(f"Warning: Invalid occupancy grid dimensions {self.occupancy_grid.shape} for {name}, skipping save")
                return
            
            # Ensure the array is uint8 and has valid values
            grid_to_save = self.occupancy_grid.astype('uint8')
            
            # Replace any NaN or invalid values with 0
            import numpy as np
            if np.any(np.isnan(grid_to_save)) or np.any(np.isinf(grid_to_save)):
                print(f"Warning: Found NaN or inf values in occupancy grid for {name}, replacing with 0")
                grid_to_save = np.nan_to_num(grid_to_save, nan=0, posinf=0, neginf=0).astype('uint8')
            
            # Ensure values are in valid range [0, 255]
            grid_to_save = np.clip(grid_to_save, 0, 255)

            if self.room is None:
                path = f"{nav_model_path}/{self.floor}/{self.floor}"
            else:
                path = f"{nav_model_path}/{self.floor}/{self.room}/{self.room}"

            try:
                plt.imsave(f"{path}.png", grid_to_save, cmap="gray", vmin=0, vmax=255, origin="lower", format="png")
                yaml_data = {
                    "image": f"{name}.png",
                    "resolution": self.resolution,
                    "origin": [self.origin[0], self.origin[1], self.origin[2], 0.0],  # Add yaw
                    "negate": 0,
                    "occupied_thresh": 0.65,
                    "free_thresh": 0.196,
                    "mode": "trinary"
                }
                with open(f"{path}.yaml", "w") as yaml_file:
                    yaml.dump(yaml_data, yaml_file, default_flow_style=False, sort_keys=False)
            except Exception as e:
                print(f"Error saving occupancy map for {name}: {e}")
                print(f"Grid shape: {grid_to_save.shape}, dtype: {grid_to_save.dtype}, min: {np.min(grid_to_save)}, max: {np.max(grid_to_save)}")
        else:
            raise ValueError("Occupancy grid is not set.")