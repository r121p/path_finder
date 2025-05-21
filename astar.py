import math
import heapq
import pathfinder  # Our C++ module
import cv2

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic estimate to end
        self.f = 0  # Total cost (g + h)
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __repr__(self):
        return f"Node({self.position}, g={self.g}, h={self.h}, f={self.f})"

def image_to_grid(image_path, threshold=200):
    """Convert an image to a grid where pixels < threshold are obstacles"""
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not read image at {image_path}")
    
    # Normalize and threshold
    grid = []
    for row in img:
        grid_row = []
        for pixel in row:
            grid_row.append(0 if pixel >= threshold else 1)
        grid.append(grid_row)
    
    return grid

def draw_path_on_image(image_path, original_path, optimized_path=None, output_path="path_comparison.png"):
    """Draw paths on the original image and save to new file"""
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Could not read image at {image_path}")
    
    # Draw original path as red line with yellow waypoints
    for i in range(len(original_path)-1):
        cv2.line(img, original_path[i][::-1], original_path[i+1][::-1], (0, 0, 255), 2)
        cv2.circle(img, original_path[i][::-1], 3, (0, 255, 255), -1)  # Yellow waypoint
    
    # Draw final waypoint for original path
    cv2.circle(img, original_path[-1][::-1], 3, (0, 255, 255), -1)
    
    # Draw optimized path as blue line with cyan waypoints if provided
    if optimized_path:
        for i in range(len(optimized_path)-1):
            cv2.line(img, optimized_path[i][::-1], optimized_path[i+1][::-1], (255, 0, 0), 2)
            cv2.circle(img, optimized_path[i][::-1], 3, (255, 255, 0), -1)  # Cyan waypoint
        
        # Draw final waypoint for optimized path
        cv2.circle(img, optimized_path[-1][::-1], 3, (255, 255, 0), -1)
    
    # Draw start (green) and end (blue) points using original path
    cv2.circle(img, original_path[0][::-1], 5, (0, 255, 0), -1)
    cv2.circle(img, original_path[-1][::-1], 5, (255, 0, 0), -1)
    
    cv2.imwrite(output_path, img)
    print(f"Saved result to {output_path}")

if __name__ == "__main__":
    # Load grid from cost_map.png
    try:
        input_image = "cost_map.png"
        grid = image_to_grid(input_image)
    except FileNotFoundError:
        print("Error: cost_map.png not found. Using example grid instead.")
        grid = [
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ]
        input_image = None
    
    # Example start and end positions
    start = (0, 0)
    end = (len(grid)-1, len(grid[0])-1)  # Bottom-right corner
    
    # Run Theta* pathfinding using C++ implementation
    print("=== Running Theta* Pathfinding ===")
    path = pathfinder.find_path(grid, start, end)
    
    if path:
        # Optimize path using C++ implementation
        final_path = pathfinder.multi_pass_optimize(grid, path)
        
        # Visualize both paths if using image input
        if input_image:
            draw_path_on_image(input_image, path, final_path, "thetastar_comparison.png")
