import numpy as np

class PathNavigator:
    def __init__(self, path_file='path_data_with_speeds.npy', window_size=500):
        """Initialize with path data and search window size."""
        try:
            self.path_data = np.load(path_file)
            if self.path_data.shape[1] != 6:
                raise ValueError("Expected 6 columns (x,y,curvature,heading,distance,speed_limit)")
        except Exception as e:
            print(f"Error loading path data: {str(e)}")
            raise
        
        self.current_idx = 0
        self.window_size = window_size
        self.path_length = len(self.path_data)
        
    def update_position(self, robot_x, robot_y):
        """
        Update robot position and return navigation info.
        Returns dict with:
        - distance_from_start
        - heading
        - speed_limit  
        - path_offset (signed distance from path)
        - is_last_point
        """
        # Find nearest point in window around current position
        search_start = max(0, self.current_idx - self.window_size//2)
        search_end = min(self.path_length, self.current_idx + self.window_size//2)
        
        window = self.path_data[search_start:search_end]
        if len(window) == 0:
            return None
            
        # Calculate distances to all points in window
        dx = window[:,0] - robot_x
        dy = window[:,1] - robot_y
        distances = dx**2 + dy**2
        
        # Update current_idx to nearest point
        nearest_in_window = np.argmin(distances)
        self.current_idx = search_start + nearest_in_window
        point = self.path_data[self.current_idx]
        
        # Calculate signed offset from path
        path_heading = np.radians(point[3])
        dx = robot_x - point[0]
        dy = robot_y - point[1]
        
        # Rotate robot position to path frame
        offset = -dx * np.sin(path_heading) + dy * np.cos(path_heading)
        
        return {
            'distance_from_start': point[4],
            'heading': point[3],
            'speed_limit': point[5],
            'path_offset': offset,
            'is_last_point': self.current_idx == self.path_length - 1
        }
        
    def get_current_point(self):
        """Get the current closest point's full data."""
        if 0 <= self.current_idx < self.path_length:
            return self.path_data[self.current_idx]
        return None
        
    def reset(self):
        """Reset navigator to start of path."""
