import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt

def create_cost_map(input_path, output_path, buffer_distance=5, buffer_color=(127, 127, 127)):
    # Read the input image
    img = cv2.imread(input_path)
    if img is None:
        raise FileNotFoundError(f"Could not read image at {input_path}")
    
    # Convert to grayscale and threshold to get binary occupancy
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY_INV)
    
    # Calculate distance transform from occupied cells
    distance = distance_transform_edt(binary == 0)
    
    # Create buffer mask (cells within buffer_distance of occupied cells)
    buffer_mask = distance <= buffer_distance
    
    # Create output image (copy of original)
    output = img.copy()
    
    # Apply buffer color to buffered cells
    output[buffer_mask] = buffer_color
    
    # Save the result
    cv2.imwrite(output_path, output)
    print(f"Cost map saved to {output_path}")

if __name__ == "__main__":
    input_image = "ocupancy.png"
    output_image = "cost_map.png"
    create_cost_map(input_image, output_image)