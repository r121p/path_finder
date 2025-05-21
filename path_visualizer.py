import numpy as np
import pygame
import sys
from path_navigator import PathNavigator

def main():
    # Initialize pygame
    pygame.init()
    
    # Load path data
    try:
        path_data = np.load('path_data_with_speeds.npy')
    except FileNotFoundError:
        print("Error: path_data_with_speeds.npy not found")
        sys.exit(1)
        
    # Create a window
    width, height = 800, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Path Visualizer")
    
    # Colors
    BACKGROUND = (0, 0, 0)
    PATH_COLOR = (0, 255, 0)
    POINT_COLOR = (255, 0, 0)
    TEXT_COLOR = (255, 255, 255)
    
    # Scale factors to fit path in window
    min_x, max_x = np.min(path_data[:, 0]), np.max(path_data[:, 0])
    min_y, max_y = np.min(path_data[:, 1]), np.max(path_data[:, 1])
    
    scale_x = (width - 100) / (max_x - min_x) if max_x != min_x else 1
    scale_y = (height - 100) / (max_y - min_y) if max_y != min_y else 1
    scale = min(scale_x, scale_y)
    
    offset_x = (width - (max_x - min_x) * scale) / 2 - min_x * scale
    offset_y = (height - (max_y - min_y) * scale) / 2 - min_y * scale
    
    # Font for displaying info
    font = pygame.font.SysFont('Arial', 16)
    
    # Initialize path navigator
    navigator = PathNavigator()
    
    # Main loop
    running = True
    selected_point = None
    nav_info = None
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Find closest point to mouse click
                mouse_x, mouse_y = pygame.mouse.get_pos()
                distances = []
                for point in path_data:
                    px = point[0] * scale + offset_x
                    py = point[1] * scale + offset_y
                    distances.append((mouse_x - px)**2 + (mouse_y - py)**2)
                selected_point = np.argmin(distances)
                # Convert mouse coords to path coords before testing navigator
                path_x = (mouse_x - offset_x) / scale
                path_y = (mouse_y - offset_y) / scale
                nav_info = navigator.update_position(path_x, path_y)
        
        # Clear screen
        screen.fill(BACKGROUND)
        
        # Verify path data shape
        if path_data.shape[1] != 6:
            print(f"Error: Expected 6 columns in path_data, got {path_data.shape[1]}")
            pygame.quit()
            sys.exit(1)
            
        # Draw path
        points = []
        for point in path_data:
            x = int(point[0] * scale + offset_x)
            y = int(point[1] * scale + offset_y)
            points.append((x, y))
        
        if len(points) > 1:
            pygame.draw.lines(screen, PATH_COLOR, False, points, 2)
        
        # Draw points and highlight selected one
        for i, point in enumerate(path_data):
            x = point[0] * scale + offset_x
            y = point[1] * scale + offset_y
            color = POINT_COLOR if i != selected_point else (0, 0, 255)
            pygame.draw.circle(screen, color, (int(x), int(y)), 5)
        
        # Display info for selected point
        if selected_point is not None:
            point = path_data[selected_point]
            info = [
                f"Point {selected_point}",
                f"Position: ({point[0]:.2f}, {point[1]:.2f})",
                f"Curvature: {point[2]:.4f}",
                f"Heading: {point[3]:.2f}°",
                f"Distance from start: {point[4]:.2f}",
                f"Speed limit: {point[5]:.2f}",
                "",
                "Path Navigator Info:",
                f"Distance from start: {nav_info['distance_from_start']:.2f}" if nav_info else "N/A",
                f"Heading: {nav_info['heading']:.2f}°" if nav_info else "N/A",
                f"Speed limit: {nav_info['speed_limit']:.2f}" if nav_info else "N/A",
                f"Path offset: {nav_info['path_offset']:.2f}" if nav_info else "N/A",
                f"Is last point: {nav_info['is_last_point']}" if nav_info else "N/A"
            ]
            
            y_offset = 10
            for line in info:
                text = font.render(line, True, TEXT_COLOR)
                screen.blit(text, (10, y_offset))
                y_offset += 20
        
        pygame.display.flip()
    
    pygame.quit()

if __name__ == "__main__":
    main()