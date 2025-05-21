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

def line_of_sight(grid, a, b):
    """Check if there's a clear path between two points without obstacles
    
    Args:
        grid: 2D grid representation
        a: (x1, y1) tuple
        b: (x2, y2) tuple
    
    Returns:
        True if line of sight exists, False otherwise
    """
    x1, y1 = a
    x2, y2 = b
    
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x = x1
    y = y1
    n = 1 + dx + dy
    x_inc = 1 if x2 > x1 else -1
    y_inc = 1 if y2 > y1 else -1
    error = dx - dy
    dx *= 2
    dy *= 2
    
    for i in range(n):
        # Check grid bounds
        if x < 0 or x >= len(grid) or y < 0 or y >= len(grid[0]):
            return False
            
        # Check if current cell is blocked
        if grid[x][y] != 0:
            return False
            
        if error > 0:
            x += x_inc
            error -= dy
        elif error < 0:
            y += y_inc
            error += dx
        else:  # Exactly diagonal
            x += x_inc
            y += y_inc
            error -= dy
            error += dx
            n -= 1
    
    return True

def optimize_path(grid, path):
    """Optimize path by removing unnecessary waypoints when direct line exists"""
    if not path or len(path) < 3:
        return path
        
    optimized = [path[0]]
    current_index = 0
    
    while current_index < len(path) - 1:
        # Start checking from furthest point first
        for next_index in range(len(path)-1, current_index, -1):
            if line_of_sight(grid, path[current_index], path[next_index]):
                optimized.append(path[next_index])
                current_index = next_index
                break
        else:
            # No direct path found, move to next point
            current_index += 1
            optimized.append(path[current_index])
    
    # Final check for direct path from start to end
    if len(optimized) > 2 and line_of_sight(grid, optimized[0], optimized[-1]):
        return [optimized[0], optimized[-1]]
    
    return optimized

def split_long_segments(path, max_length=10):
    """Split path segments longer than max_length into smaller segments"""
    if not path or len(path) < 2:
        return path
        
    new_path = [path[0]]
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        
        if distance > max_length:
            segments = int(distance / max_length) + 1
            for s in range(1, segments):
                ratio = s / segments
                new_x = int(x1 + (x2 - x1) * ratio)
                new_y = int(y1 + (y2 - y1) * ratio)
                new_path.append((new_x, new_y))
        new_path.append(path[i])
    
    return new_path

def reverse_optimize_path(grid, path):
    """Optimize path from end to start"""
    if not path or len(path) < 3:
        return path
        
    optimized = [path[-1]]
    current_index = len(path) - 1
    
    while current_index > 0:
        # Start checking from start point first
        for next_index in range(0, current_index):
            if line_of_sight(grid, path[next_index], path[current_index]):
                optimized.append(path[next_index])
                current_index = next_index
                break
        else:
            # No direct path found, move to previous point
            current_index -= 1
            optimized.append(path[current_index])
    
    # Reverse to maintain start-to-end order
    return optimized[::-1]

def multi_pass_optimize(grid, path, passes=5):
    """Perform alternating forward/reverse optimization passes"""
    if not path or len(path) < 3:
        return path
        
    optimized = path
    for i in range(passes):
        # Forward pass
        optimized = split_long_segments(optimized)
        optimized = optimize_path(grid, optimized)
        # Reverse pass
        optimized = split_long_segments(optimized)
        optimized = reverse_optimize_path(grid, optimized)
        
    return optimized

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

    
    # Draw optimized path as blue line with cyan waypoints if provided
    if optimized_path:
        for i in range(len(optimized_path)-1):
            cv2.line(img, optimized_path[i][::-1], optimized_path[i+1][::-1], (255, 0, 0), 2)
            cv2.circle(img, optimized_path[i][::-1], 3, (255, 255, 0), -1)  # Cyan waypoint
        
        # Draw final waypoint for optimized path
        cv2.circle(img, optimized_path[-1][::-1], 3, (255, 255, 0), -1)
    
    
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
        print("Optimizing path")
        final_path = multi_pass_optimize(grid, path)
        with open('path.txt','w') as f:
            f.write(str(final_path))
        # Visualize both paths if using image input
        if input_image:
            draw_path_on_image(input_image, path, final_path, "thetastar_comparison.png")
