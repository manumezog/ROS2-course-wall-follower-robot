#!/usr/bin/env python3

"""
-------------------------------------------------------------------------------
HEADER DOCUMENTATION
-------------------------------------------------------------------------------
Project: Mars Rover - Real-World Wall Follower (P-Controller + Recovery)
Author: Manuel Mezo
Description: 
    Optimized for real-circuit performance with slow, controlled speeds.
    Maintains 0.30m from right wall using P-Control (Kp=3).
    Includes a timed recovery maneuver for frontal deadlocks.
-------------------------------------------------------------------------------
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # --- Publishers ---
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.meas_publisher = self.create_publisher(String, '/dist_meas', 10)

        # --- Subscriber ---
        self.subscriber_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # --- TUNED PARAMETERS (Based on real-life testing) ---
        self.setpoint = 0.30       # Desired distance from right wall
        self.kp = 3.0              # Proportional Gain
        self.cruise_speed = 0.1    # Linear speed during normal following
        self.corner_speed = 0.15   # Linear speed during cornering
        self.reverse_speed = -0.08 # Gentle backing up speed
        
        # --- RECOVERY / STUCK DETECTION ---
        self.stuck_start_time = None
        self.is_recovering = False
        self.recovery_start_time = 0.0
        self.stuck_timeout = 1.8   # Wait 1.8s of blocking before reversing

        self.get_logger().info(f"Node Started. Target: {self.setpoint}m | Speed: {self.cruise_speed}m/s")

    def laserscan_callback(self, msg):
        # 1. RANGE SENSOR INDEXING
        num_rays = len(msg.ranges)
        front_idx = 0
        right_idx = int(num_rays * 3 / 4) 

        dist_front = msg.ranges[front_idx]
        dist_right = msg.ranges[right_idx]

        # Filter 'inf' and noise
        if math.isinf(dist_front) or dist_front < 0.01: dist_front = 10.0
        if math.isinf(dist_right) or dist_right < 0.01: dist_right = 10.0

        # 2. DIAGNOSTICS
        diag_msg = String()
        mode = "RECOVERING" if self.is_recovering else "NORMAL"
        diag_msg.data = f"[{mode}] F: {dist_front:.2f} R: {dist_right:.2f}"
        self.meas_publisher.publish(diag_msg)

        # 3. TIME-BASED LOGIC
        current_time = self.get_clock().now().nanoseconds / 1e9
        action = Twist()

        # --- STATE A: RECOVERY MANEUVER (If robot is stuck) ---
        if self.is_recovering:
            elapsed = current_time - self.recovery_start_time
            
            if elapsed < 1.5:     # Phase 1: Reverse away from the front wall
                action.linear.x = self.reverse_speed
                action.angular.z = 0.0
                self.get_logger().info("STUCK: Backing up...")
            elif elapsed < 2.5:   # Phase 2: Pivot Left to reset heading
                action.linear.x = 0.0
                action.angular.z = 1.2
                self.get_logger().info("STUCK: Re-orienting...")
            else:                # Phase 3: Transition back
                self.is_recovering = False
                self.stuck_start_time = None
                self.get_logger().info("Recovery done. Resuming.")
            
            self.vel_publisher.publish(action)
            return

        # --- STATE B: NORMAL NAVIGATION ---
        
        # Priority 1: Front Wall Detection
        if dist_front < 0.5:
            if self.stuck_start_time is None:
                self.stuck_start_time = current_time
            
            # Check for Deadlock
            if (current_time - self.stuck_start_time) > self.stuck_timeout:
                self.get_logger().error("DEADLOCK! Initiating reverse.")
                self.is_recovering = True
                self.recovery_start_time = current_time
            else:
                # Normal corner avoidance (Slow linear, fast angular)
                action.linear.x = self.corner_speed
                action.angular.z = 1.5 
                self.get_logger().warn("Corner detected.")
        else:
            # Path clear: Reset stuck timer and use P-Controller
            self.stuck_start_time = None
            
            # P-Control: (Setpoint - Actual) * Gain
            error = self.setpoint - dist_right
            action.linear.x = self.cruise_speed
            action.angular.z = error * self.kp
            
            # Limited logging to avoid flooding terminal
            if int(current_time * 2) % 2 == 0: 
                self.get_logger().info(f"Error: {error:.3f} | Turn: {action.angular.z:.3f}")

        # 4. PUBLISH VELOCITY
        self.vel_publisher.publish(action)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send stop message on CTRL+C
        node.vel_publisher.publish(Twist()) 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()