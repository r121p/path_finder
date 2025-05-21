import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
from math import atan2, degrees

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

def smooth_path(segments, distance=50):
    """Generate smoothed path by averaging positions of two walkers"""
    smoothed = []
    lead = PathWalker(segments)
    trail = PathWalker(segments)
    
    # Move leading walker ahead by specified distance
    lead.move(distance)
    
    while lead.get_distance_traveled() < (segments[-1]['distance_from_start'] + segments[-1]['length']):
        # Get current positions
        lead_pos = np.array(lead.get_current_position())
        trail_pos = np.array(trail.get_current_position())
        
        # Calculate and store median point
        median = tuple((lead_pos + trail_pos) / 2)
        smoothed.append(median)
        
        # Move both walkers forward
        lead.move(1)
        trail.move(1)
    
    return smoothed

def calculate_curvature(points):
    """Calculate curvature for each point (except endpoints)"""
    curvatures = []
    distances = []
    cumulative_distance = 0.0
    
    # Calculate segment distances and cumulative distance
    for i in range(1, len(points)):
        dist = np.linalg.norm(np.array(points[i]) - np.array(points[i-1]))
        distances.append(dist)
        cumulative_distance += dist
    
    # Calculate curvature for each point (except first and last)
    for i in range(1, len(points)-1):
        # Get vectors to adjacent points
        prev_point = np.array(points[i-1])
        curr_point = np.array(points[i])
        next_point = np.array(points[i+1])
        
        # Calculate headings
        vec1 = curr_point - prev_point
        vec2 = next_point - curr_point
        heading1 = atan2(vec1[1], vec1[0])
        heading2 = atan2(vec2[1], vec2[0])
        
        # Calculate angle change (curvature)
        angle_change = degrees(heading2 - heading1) % 360
        if angle_change > 180:
            angle_change -= 360
        
        # Normalize by segment lengths
        avg_dist = (distances[i-1] + distances[i]) / 2
        curvature = angle_change / avg_dist if avg_dist > 0 else 0
        curvatures.append(curvature)
    
    return curvatures, distances

def plot_curvature_analysis(points, output_path='curvature_analysis.png'):
    """Plot curvature vs distance along path"""
    curvatures, segment_distances = calculate_curvature(points)
    
    # Calculate cumulative distances for each point (excluding first and last)
    cumulative_distances = []
    current_dist = 0.0
    for i in range(1, len(points)-1):
        current_dist += segment_distances[i-1]
        cumulative_distances.append(current_dist)
    
    plt.figure(figsize=(10, 6))
    plt.plot(cumulative_distances, curvatures, 'b-')
    plt.xlabel('Distance from start (cm)')
    plt.ylabel('Curvature (degrees/cm)')
    plt.title('Path Curvature Analysis')
    plt.grid(True)
    plt.savefig(output_path)
    plt.close()

def plot_path(segments, output_path='path_visualization.png'):
    try:
        img = cv2.imread('ocupancy.png')
        if img is None:
            raise FileNotFoundError("Could not load ocupancy.png")
        
        # Draw original path lines
        for segment in segments:
            start = (int(segment['start'][1]/CELL_SIZE), int(segment['start'][0]/CELL_SIZE))
            end = (int(segment['end'][1]/CELL_SIZE), int(segment['end'][0]/CELL_SIZE))
            cv2.line(img, start, end, (0, 0, 255), 2)  # Red lines
        
        # Draw smoothed path
        smoothed = smooth_path(segments, 50)
        for i in range(len(smoothed)-1):
            start = (int(smoothed[i][1]/CELL_SIZE), int(smoothed[i][0]/CELL_SIZE))
            end = (int(smoothed[i+1][1]/CELL_SIZE), int(smoothed[i+1][0]/CELL_SIZE))
            cv2.line(img, start, end, (255, 0, 0), 2)  # Blue lines
        
        # Create walker and plot positions every 500cm
        walker = PathWalker(segments)
        total_length = segments[-1]['distance_from_start'] + segments[-1]['length']
        
        for distance in range(0, int(total_length)+500, 500):
            # Plot verification dots on original path
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
        
        # Perform curvature analysis on smoothed path
        smoothed = smooth_path(segments, 50)
        plot_curvature_analysis(smoothed, 'curvature_analysis.png')
        print("Successfully saved curvature analysis to curvature_analysis.png")
    else:
        print("Failed to create path visualization")

if __name__ == "__main__":
    main()