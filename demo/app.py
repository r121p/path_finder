import os
import sys
import cv2
import numpy as np
from flask import Flask, render_template, request, redirect, url_for, flash, send_from_directory
from werkzeug.utils import secure_filename

# Add parent directory to path to import modules from parent directory
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import pathfinding modules
from astar import astar, optimize_path, multi_pass_optimize, image_to_grid, draw_path_on_image
from process_grid import create_cost_map
from path_processor import process_path, read_waypoints, plot_path

# Create Flask app
app = Flask(__name__)
app.secret_key = 'pathfinder_secret_key'

# Configure upload folder
UPLOAD_FOLDER = 'static/uploads'
RESULTS_FOLDER = 'static/results'
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg'}

app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['RESULTS_FOLDER'] = RESULTS_FOLDER

# Create folders if they don't exist
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
os.makedirs(RESULTS_FOLDER, exist_ok=True)

def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/upload', methods=['POST'])
def upload_file():
    # Check if a file was uploaded
    if 'file' not in request.files:
        flash('No file part')
        return redirect(request.url)
    
    file = request.files['file']
    
    # If user doesn't select file, browser also submits an empty part
    if file.filename == '':
        flash('No selected file')
        return redirect(request.url)
    
    if file and allowed_file(file.filename):
        filename = secure_filename(file.filename)
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        file.save(filepath)
        
        # Process the uploaded image to create cost map
        cost_map_path = os.path.join(app.config['RESULTS_FOLDER'], 'cost_map.png')
        buffer_distance = int(request.form.get('buffer_distance', 5))
        create_cost_map(filepath, cost_map_path, buffer_distance)
        
        # Get image dimensions for the UI
        img = cv2.imread(filepath)
        height, width = img.shape[:2]
        
        return render_template('pathfinder.html', 
                              original_image=os.path.basename(filepath),
                              cost_map='cost_map.png',
                              img_width=width,
                              img_height=height)
    
    flash('Invalid file type. Please upload a PNG or JPG image.')
    return redirect(url_for('index'))

@app.route('/find_path', methods=['POST'])
def find_path():
    # Get parameters from form
    start_x = int(request.form.get('start_x', 0))
    start_y = int(request.form.get('start_y', 0))
    end_x = int(request.form.get('end_x', 100))
    end_y = int(request.form.get('end_y', 100))
    use_theta = 'use_theta' in request.form
    optimize_passes = int(request.form.get('optimize_passes', 5))
    
    # Get the cost map path
    cost_map_path = os.path.join(app.config['RESULTS_FOLDER'], 'cost_map.png')
    original_image = request.form.get('original_image')
    original_path = os.path.join(app.config['UPLOAD_FOLDER'], original_image)
    
    # Load grid from cost map
    grid = image_to_grid(cost_map_path)
    
    # Run pathfinding
    start = (start_y, start_x)  # Note: grid is indexed as [y][x]
    end = (end_y, end_x)
    
    path = astar(grid, start, end, theta=use_theta)
    
    if not path:
        flash('No path found! Try different start/end points or reduce buffer distance.')
        return redirect(url_for('index'))
    
    # Optimize path if requested
    if optimize_passes > 0:
        optimized_path = multi_pass_optimize(grid, path, passes=optimize_passes)
    else:
        optimized_path = path
    
    # Save path to file
    path_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'path.txt')
    with open(path_file, 'w') as f:
        f.write(str(optimized_path))
    
    # Draw paths on image
    result_image = 'path_result.png'
    result_path = os.path.join(app.config['RESULTS_FOLDER'], result_image)
    draw_path_on_image(original_path, path, optimized_path, result_path)
    
    # Process path for visualization
    waypoints = read_waypoints(path_file)
    segments = process_path(waypoints)
    
    # Plot path with smoothing
    smooth_result = 'smooth_path.png'
    smooth_path = os.path.join(app.config['RESULTS_FOLDER'], smooth_result)
    plot_path(segments, smooth_path)
    
    # Calculate path statistics
    original_length = sum(
        np.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
        for i in range(len(path)-1)
    )
    
    optimized_length = sum(
        np.sqrt((optimized_path[i+1][0] - optimized_path[i][0])**2 + 
                (optimized_path[i+1][1] - optimized_path[i][1])**2)
        for i in range(len(optimized_path)-1)
    )
    
    waypoint_reduction = (1 - len(optimized_path) / len(path)) * 100 if len(path) > 0 else 0
    length_reduction = (1 - optimized_length / original_length) * 100 if original_length > 0 else 0
    
    return render_template('results.html',
                          original_image=original_image,
                          cost_map='cost_map.png',
                          path_result=result_image,
                          smooth_path=smooth_result,
                          original_length=round(original_length, 2),
                          optimized_length=round(optimized_length, 2),
                          original_waypoints=len(path),
                          optimized_waypoints=len(optimized_path),
                          waypoint_reduction=round(waypoint_reduction, 2),
                          length_reduction=round(length_reduction, 2),
                          algorithm="Theta*" if use_theta else "A*",
                          optimize_passes=optimize_passes)

@app.route('/static/<path:filename>')
def serve_static(filename):
    return send_from_directory('static', filename)

@app.route('/static/uploads/<path:filename>')
def serve_uploads(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename)

@app.route('/static/results/<path:filename>')
def serve_results(filename):
    return send_from_directory(app.config['RESULTS_FOLDER'], filename)

@app.route('/use_default')
def use_default():
    # Copy default image to uploads folder
    default_image = 'ocupancy.png'
    default_image_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), default_image)
    
    if not os.path.exists(default_image_path):
        # Import the create_default_grid function
        from create_default_grid import create_default_grid
        create_default_grid(default_image_path)
    
    upload_path = os.path.join(app.config['UPLOAD_FOLDER'], default_image)
    os.makedirs(os.path.dirname(upload_path), exist_ok=True)
    
    # Copy or create the default image
    if os.path.exists(default_image_path):
        img = cv2.imread(default_image_path)
        cv2.imwrite(upload_path, img)
    else:
        # Create a simple default grid
        grid = np.zeros((100, 100, 3), dtype=np.uint8)
        grid.fill(255)  # White background
        
        # Add some obstacles
        for i in range(20, 80):
            grid[i, 50] = [0, 0, 0]  # Vertical line
            grid[50, i] = [0, 0, 0]  # Horizontal line
        
        cv2.imwrite(upload_path, grid)
    
    # Process the default image to create cost map
    cost_map_path = os.path.join(app.config['RESULTS_FOLDER'], 'cost_map.png')
    buffer_distance = 5  # Default buffer distance
    create_cost_map(upload_path, cost_map_path, buffer_distance)
    
    # Get image dimensions for the UI
    img = cv2.imread(upload_path)
    height, width = img.shape[:2]
    
    return render_template('pathfinder.html', 
                          original_image=default_image,
                          cost_map='cost_map.png',
                          img_width=width,
                          img_height=height)

if __name__ == '__main__':
    app.run(debug=True)