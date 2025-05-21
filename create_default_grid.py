import cv2
import numpy as np

def create_default_grid(output_path='ocupancy.png', width=500, height=500):
    """Create a default occupancy grid with some obstacles"""
    # Create a white background
    grid = np.ones((height, width), dtype=np.uint8) * 255
    
    # Add some obstacles (black pixels)
    
    # Vertical wall with a gap
    for i in range(50, 450):
        if 200 <= i <= 300:  # Gap in the wall
            continue
        grid[i, 250] = 0
    
    # Horizontal wall with a gap
    for i in range(50, 450):
        if 200 <= i <= 300:  # Gap in the wall
            continue
        grid[250, i] = 0
    
    # Add some random obstacles
    np.random.seed(42)  # For reproducibility
    for _ in range(20):
        x = np.random.randint(50, width-50)
        y = np.random.randint(50, height-50)
        size = np.random.randint(10, 30)
        
        # Don't place obstacles near the center gaps
        if (200 <= x <= 300 and 200 <= y <= 300):
            continue
            
        # Create small obstacle clusters
        cv2.circle(grid, (x, y), size // 2, 0, -1)
    
    # Add a maze-like structure in the bottom-right quadrant
    for i in range(300, 450, 30):
        cv2.line(grid, (300, i), (450, i), 0, 2)
    
    for i in range(300, 450, 30):
        # Skip some lines to create paths
        if i % 60 == 0:
            continue
        cv2.line(grid, (i, 300), (i, 450), 0, 2)
    
    # Save the grid
    cv2.imwrite(output_path, grid)
    print(f"Default occupancy grid saved to {output_path}")
    
    return grid

if __name__ == "__main__":
    create_default_grid()