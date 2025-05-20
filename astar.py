import heapq
import math

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

def heuristic(a, b):
    """Euclidean distance heuristic"""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

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

def astar(grid, start, end, theta=False):
    """Pathfinding algorithm implementation (A* or Theta*)
    
    Args:
        grid: 2D list representing the grid (0=walkable, 1=obstacle)
        start: (x, y) tuple for start position
        end: (x, y) tuple for end position
    
    Returns:
        List of (x, y) tuples representing the path from start to end
        None if no path found
    """
    # Create start and end nodes
    start_node = Node(start)
    end_node = Node(end)
    
    # Initialize open and closed lists
    open_list = []
    closed_list = set()
    
    # Add start node to open list
    heapq.heappush(open_list, start_node)
    
    # Possible movement directions (4-way movement)
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)
        
        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path
        
        # Generate children
        children = []
        for direction in directions:
            # Get node position
            node_position = (
                current_node.position[0] + direction[0],
                current_node.position[1] + direction[1]
            )
            
            # Make sure within range
            if (node_position[0] > (len(grid) - 1) or 
                node_position[0] < 0 or 
                node_position[1] > (len(grid[0]) - 1) or 
                node_position[1] < 0):
                continue
                
            # Make sure walkable terrain
            if grid[node_position[0]][node_position[1]] != 0:
                continue
                
            # Create new node
            new_node = Node(node_position, current_node)
            children.append(new_node)
        
        # Loop through children
        for child in children:
            # Child is on the closed list
            if child.position in closed_list:
                continue
                
            # Create the f, g, and h values
            if theta and current_node.parent:
                # Theta*: try to connect to grandparent if line of sight exists
                if line_of_sight(grid, current_node.parent.position, child.position):
                    new_g = current_node.parent.g + heuristic(current_node.parent.position, child.position)
                    if new_g < child.g:
                        child.parent = current_node.parent
                        child.g = new_g
                else:
                    child.g = current_node.g + 1
            else:
                # Regular A*
                child.g = current_node.g + 1
            
            child.h = heuristic(child.position, end_node.position)
            child.f = child.g + child.h
            
            # Check if child is already in open list with better g
            found = False
            for open_node in open_list:
                if child == open_node and child.g >= open_node.g:
                    found = True
                    break
            
            if not found:
                heapq.heappush(open_list, child)
    
    # No path found
    return None

def image_to_grid(image_path, threshold=200):
    """Convert an image to a grid where pixels < threshold are obstacles
    
    Args:
        image_path: Path to the input image
        threshold: Pixel value threshold (values below are obstacles)
    
    Returns:
        2D list representing the grid (0=walkable, 1=obstacle)
    """
    import cv2
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

def draw_path_on_image(image_path, path, output_path="path_result.png"):
    """Draw the found path on the original image and save to new file
    
    Args:
        image_path: Path to original image
        path: List of (x,y) tuples representing the path
        output_path: Path to save the result image
    """
    import cv2
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Could not read image at {image_path}")
    
    # Draw path as red line
    for i in range(len(path)-1):
        cv2.line(img, path[i][::-1], path[i+1][::-1], (0, 0, 255), 2)
    
    # Draw start (green) and end (blue) points
    cv2.circle(img, path[0][::-1], 5, (0, 255, 0), -1)
    cv2.circle(img, path[-1][::-1], 5, (255, 0, 0), -1)
    
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
    
    # Run Theta* pathfinding
    print("=== Running Theta* Pathfinding ===")
    path = astar(grid, start, end, theta=True)
    
    # Print detailed path information
    print("\nTheta* Path Details:")
    print(f"Path length: {len(path)} steps")
    print(f"Start: {start}")
    print(f"End: {end}")
    print("Path coordinates:")
    for i, coord in enumerate(path):
        print(f"Step {i+1}: {coord}")
    
    # Visualize path if using image input
    if input_image and path:
        draw_path_on_image(input_image, path, "thetastar_result.png")