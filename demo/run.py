"""
Simple script to run the Pathfinder Demo application.
"""
import os
import sys
from app import app

if __name__ == "__main__":
    # Make sure we're running from the demo directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    # Add parent directory to path to import modules from parent directory
    sys.path.append(os.path.dirname(script_dir))
    
    # Run the Flask app
    app.run(debug=True, host='0.0.0.0', port=5000)