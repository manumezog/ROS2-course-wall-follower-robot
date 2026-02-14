import pandas as pd
import matplotlib
# CRITICAL: Switch backend to 'Agg' before importing pyplot
# This prevents the script from trying to open a window
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import os
from datetime import datetime

def plot_robot_path(csv_file):
    try:
        # 1. Load the data
        if not os.path.exists(csv_file):
            print(f"Error: {csv_file} not found. Did the robot finish the lap?")
            return

        data = pd.read_csv(csv_file)
        
        # 2. Setup Plot
        plt.figure(figsize=(10, 8))
        
        # Plot trajectory
        plt.plot(data['X'], data['Y'], label='Robot Path', color='blue', linewidth=2, alpha=0.7)
        
        # Highlight Start and End
        plt.scatter(data['X'].iloc[0], data['Y'].iloc[0], color='green', s=100, label='Start', zorder=5)
        plt.scatter(data['X'].iloc[-1], data['Y'].iloc[-1], color='red', s=100, label='End (Lap Complete)', zorder=5)
        
        # Formatting
        plt.title('Robot Odometry: Lap Visualization', fontsize=15)
        plt.xlabel('X Position (meters)', fontsize=12)
        plt.ylabel('Y Position (meters)', fontsize=12)
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.axis('equal') 
        
        # 3. Generate Timestamped Filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_name = f'robot_lap_{timestamp}.png'
        
        # 4. Save Image (No plt.show())
        plt.savefig(image_name, dpi=300, bbox_inches='tight')
        
        # 5. Clear memory (Good practice in headless scripts)
        plt.close()
        
        print(f"--- Visualization Success ---")
        print(f"Image saved as: {os.path.abspath(image_name)}")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Ensure this matches the filename saved by odom_recorder.py
    plot_robot_path('robot_lap_data.csv')