from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    
    # 1. Define the Wall Follower Node (Logic)
    wall_follower_node = Node(
        package='wall_follower',
        executable='wall_follower_executable',
        output='screen'
    )

    # 2. Define the Plotter Process (Triggers after finish)
    # Assumes plot_lap.py is in your workspace root. 
    # If it fails, check the path!
    plotter_process = ExecuteProcess(
        cmd=['python3', 'plot_lap.py'],
        output='screen'
    )

    return LaunchDescription([
        # NODE A: Service Server (Finder)
        Node(
            package='wall_follower',
            executable='wall_finder_executable',
            output='screen'
        ),
        
        # NODE B: Action Server (Recorder) - CRITICAL! MISSING BEFORE!
        Node(
            package='wall_follower',
            executable='odom_recorder_executable',
            output='screen'
        ),

        # NODE C: Main Logic (Follower)
        wall_follower_node,

        # AUTOMATION: Trigger Plotter when Follower finishes
        RegisterEventHandler(
            OnProcessExit(
                target_action=wall_follower_node,
                on_exit=[plotter_process],
            )
        )
    ])