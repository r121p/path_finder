import numpy as np
import matplotlib.pyplot as plt

def calculate_speed_limits(path_data, max_speed, min_speed, max_turning_speed_rad_s, window_size_cm=50):
    """
    Calculate speed limits for each point in path data.
    
    Args:
        path_data: numpy array of shape (n,5) containing [x,y,curvature,heading,distance]
        max_speed: maximum allowed speed (cm/s)
        min_speed: minimum allowed speed (cm/s)
        max_turning_speed_rad_s: maximum angular speed (radians/second)
        window_size_cm: size of moving average window in cm (default: 50)
        
    Returns:
        numpy array with added speed limit column (shape n,6)
    """
    # Extract distances and curvature values
    distances = path_data[:, 4]
    curvatures = path_data[:, 2]
    
    # Smooth curvature with weighted moving average based on distance window
    smoothed_curvatures = np.zeros_like(curvatures)
    for i in range(len(curvatures)):
        # Calculate weights using Gaussian-like distribution
        dist_from_center = np.abs(distances - distances[i])
        weights = np.exp(-(dist_from_center**2)/(2*(window_size_cm/4)**2))
        weights[dist_from_center > window_size_cm/2] = 0  # Zero weights outside window
        
        # Normalize weights and calculate weighted average
        normalized_weights = weights / np.sum(weights)
        smoothed_curvatures[i] = np.sum(curvatures * normalized_weights)
    
    # Convert smoothed curvature to radians/cm
    curvatures_rad_cm = np.radians(smoothed_curvatures)
    
    # Calculate speed limit: v = ω/κ (where ω is max angular speed in rad/s, κ is curvature in rad/cm)
    # This gives v in cm/s
    speed_limits = max_turning_speed_rad_s / (np.abs(curvatures_rad_cm) + 1e-6)  # Avoid division by zero
    
    # Clip speeds between min and max
    speed_limits = np.clip(speed_limits, min_speed, max_speed)
    
    # Add speed limit as new column
    return np.column_stack((path_data, speed_limits))

def plot_speed_profile(path_data_with_speeds, filename="speed_profile.png"):
    """Plot speed limit vs distance and save to file"""
    distances = path_data_with_speeds[:, 4]
    speeds = path_data_with_speeds[:, 5]
    
    plt.figure(figsize=(10, 5))
    plt.plot(distances, speeds, label='Speed Limit')
    plt.xlabel('Distance from start (cm)')
    plt.ylabel('Speed limit (cm/s)')
    plt.title('Speed Limit Profile')
    plt.grid(True)
    plt.legend()
    plt.savefig(filename)
    plt.close()

if __name__ == "__main__":
    # Example usage
    path_data = np.load("path_data.npy")  # Load path data
    
    # Parameters (adjust these as needed)
    MAX_SPEED = 100  # cm/s
    MIN_SPEED = 20   # cm/s
    MAX_TURNING_SPEED_RAD_S = 1.0  # rad/s
    
    # Calculate and add speed limits
    path_data_with_speeds = calculate_speed_limits(path_data, MAX_SPEED, MIN_SPEED, MAX_TURNING_SPEED_RAD_S, 50)
    
    # Save the enhanced data
    np.save("path_data_with_speeds.npy", path_data_with_speeds)
    
    # Plot and save the results
    plot_speed_profile(path_data_with_speeds, "speed_profile.png")