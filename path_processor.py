import math
import cv2
import numpy as np

# Global configuration
CELL_SIZE = 5  # cm per grid cell

def read_waypoints(file_path):
    with open(file_path, 'r') as f:
        content = f.read().strip()
        waypoints = eval(content)
        # Convert to cm by multiplying by CELL_SIZE
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
        
        for segment in segments:
            # Convert back to grid coordinates and transpose (swap x and y)
            start = (int(segment['start'][1]/CELL_SIZE), int(segment['start'][0]/CELL_SIZE))
            end = (int(segment['end'][1]/CELL_SIZE), int(segment['end'][0]/CELL_SIZE))
            cv2.line(img, start, end, (0, 0, 255), 2)
        
        cv2.imwrite(output_path, img)
        return True
    
    except Exception as e:
        print(f"Error during visualization: {e}")
        return False

def main():
    waypoints = read_waypoints('path.txt')
    segments = process_path(waypoints)
    
    print("Generated Segments:")
    for i, segment in enumerate(segments, 1):
        print(f"Segment {i}:")
        print(f"  Start: {segment['start']} cm")
        print(f"  End: {segment['end']} cm")
        print(f"  Length: {segment['length']:.2f} cm")
        print(f"  Distance from start: {segment['distance_from_start']:.2f} cm")
        print()
    
    if plot_path(segments):
        print(f"Successfully saved path visualization to path_visualization.png (using CELL_SIZE={CELL_SIZE}cm)")
    else:
        print("Failed to create path visualization")

if __name__ == "__main__":
    main()