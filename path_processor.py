import math
import cv2
import numpy as np

# Global configuration
CELL_SIZE = 5  # cm per grid cell

class PathWalker:
    def __init__(self, segments):
        self.segments = segments
        self.current_segment_idx = 0
        self.distance_traveled = 0.0
        self.current_position = segments[0]['start'] if segments else (0, 0)
    
    def get_distance_traveled(self):
        """Return total distance traveled along path in cm"""
        return self.distance_traveled
    
    def get_current_position(self):
        """Return current (x, y) position in cm"""
        return tuple(map(float, self.current_position))
    
    def move(self, distance):
        """Move along path by specified distance (cm)"""
        remaining_distance = distance
        
        while remaining_distance > 0 and self.current_segment_idx < len(self.segments):
            segment = self.segments[self.current_segment_idx]
            segment_length = segment['length']
            segment_remaining = segment_length - (self.distance_traveled - segment['distance_from_start'] if self.current_segment_idx > 0 else self.distance_traveled)
            
            if remaining_distance <= segment_remaining:
                # Move within current segment
                ratio = (self.distance_traveled - segment['distance_from_start'] + remaining_distance) / segment_length
                start = np.array(segment['start'])
                end = np.array(segment['end'])
                self.current_position = tuple(start + (end - start) * ratio)
                self.distance_traveled += remaining_distance
                remaining_distance = 0
            else:
                # Move to end of current segment
                self.current_position = segment['end']
                self.distance_traveled += segment_remaining
                remaining_distance -= segment_remaining
                self.current_segment_idx += 1
        
        return self.current_position

def read_waypoints(file_path):
    with open(file_path, 'r') as f:
        content = f.read().strip()
        waypoints = eval(content)
        return [(x*CELL_SIZE, y*CELL_SIZE) for (x,y) in waypoints]

def calculate_distance(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

def process_path(waypoints):
    segments = []
    cumulative_distance = 0.0
    
    for i in range(len(waypoints)-1):
        start = waypoints[i]
        end = waypoints[i+1]
        length = calculate_distance(start, end)
        
        segment = {
            'start': start,
            'end': end,
            'length': length,
            'distance_from_start': cumulative_distance
        }
        segments.append(segment)
        cumulative_distance += length
    
    return segments

def plot_path(segments, output_path='path_visualization.png'):
    try:
        img = cv2.imread('ocupancy.png')
        if img is None:
            raise FileNotFoundError("Could not load ocupancy.png")
        
        # Draw path lines
        for segment in segments:
            start = (int(segment['start'][1]/CELL_SIZE), int(segment['start'][0]/CELL_SIZE))
            end = (int(segment['end'][1]/CELL_SIZE), int(segment['end'][0]/CELL_SIZE))
            cv2.line(img, start, end, (0, 0, 255), 2)  # Red lines
        
        # Create walker and plot positions every 500cm
        walker = PathWalker(segments)
        total_length = segments[-1]['distance_from_start'] + segments[-1]['length']
        
        for distance in range(0, int(total_length)+500, 500):
            # Reset walker for accurate incremental movement
            walker = PathWalker(segments)
            walker.move(distance)
            pos = walker.get_current_position()
            img_pos = (int(pos[1]/CELL_SIZE), int(pos[0]/CELL_SIZE))
            print(f"Plotting dot at {distance}cm: position {pos} -> image pos {img_pos}")
            cv2.circle(img, img_pos, 5, (0, 255, 0), -1)  # Green dots
        
        cv2.imwrite(output_path, img)
        return True
    
    except Exception as e:
        print(f"Error during visualization: {e}")
        return False

def main():
    waypoints = read_waypoints('path.txt')
    segments = process_path(waypoints)
    
    # Example usage of PathWalker
    walker = PathWalker(segments)
    print("Initial position:", walker.get_current_position())
    
    # Move walker and print status
    walker.move(1000)
    print("After moving 1000cm:")
    print("Position:", walker.get_current_position())
    print("Distance traveled:", walker.get_distance_traveled())
    
    if plot_path(segments):
        print(f"Successfully saved path visualization to path_visualization.png (using CELL_SIZE={CELL_SIZE}cm)")
    else:
        print("Failed to create path visualization")

if __name__ == "__main__":
    main()