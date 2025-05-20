import heapq

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
    """Manhattan distance heuristic"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, end):
    """A* pathfinding algorithm implementation
    
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
            child.g = current_node.g + 1
            child.h = heuristic(child.position, end_node.position)
            child.f = child.g + child.h
            
            # Child is already in the open list and has lower g value
            found = False
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    found = True
                    break
            
            if found:
                continue
                
            # Add the child to the open list
            heapq.heappush(open_list, child)
    
    # No path found
    return None

if __name__ == "__main__":
    # Example usage
    # 0 = walkable, 1 = obstacle
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
    
    start = (0, 0)
    end = (7, 6)
    
    path = astar(grid, start, end)
    print("Path found:", path)