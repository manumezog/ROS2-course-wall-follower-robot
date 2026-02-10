#!/usr/bin/env python3

"""
-------------------------------------------------------------------------------
HEADER DOCUMENTATION
-------------------------------------------------------------------------------
Project: Wall Follower with Diagnostics
Author: Manuel Mezo
Description: 
    Follows the right wall at 0.2m-0.3m while publishing raw distance 
    diagnostics to /dist_meas for monitoring
-------------------------------------------------------------------------------
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math # Necessary for isinf check


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Velocity Publisher
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # DIAGNOSTICS: Your requested distance monitor
        self.meas_publisher = self.create_publisher(String, '/dist_meas', 10)

        # Range Sensor Subscriber
        self.subscriber_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.get_logger().info("Wall Follower Active. Monitor /dist_meas for live data.")

    def laserscan_callback(self, msg):
        # 1. SENSOR INDEXING
        # Assuming a 360-degree LIDAR with 0 as front. 
        # Check these indices if the robot behaves weirdly!
        num_rays = len(msg.ranges)
        front_idx = 0
        right_idx = int(num_rays * 3 / 4) # 270 degrees (90 degrees to the right)

        dist_front = msg.ranges[front_idx]
        dist_right = msg.ranges[right_idx]

        # Clean "inf" or 0.0 values to avoid math errors
        #if dist_front < 0.05 or math.isinf(dist_front): dist_front = 10.0
        #if dist_right < 0.05 or math.isinf(dist_right): dist_right = 10.0
        if math.isinf(dist_front): dist_front = 10.0
        if math.isinf(dist_right): dist_right = 10.0

        # 2. PUBLISH DIAGNOSTICS
        diag_msg = String()
        diag_msg.data = f"Front: {dist_front:.2f} | Right: {dist_right:.2f}"
        self.meas_publisher.publish(diag_msg)

        # 3. CONTROL LOGIC (Nested Priority)
        action = Twist()
        
        # --- THE FIX: SAFETY FIRST ---
        # If the front wall is close, we MUST turn left regardless of what the right side says.
        if dist_front < 0.5:
            self.get_logger().warn("FRONT WALL! Turning Fast Left.")
            action.linear.x = 0.05   # Slow forward to keep momentum but avoid collision
            action.angular.z = 0.6   # Fast turn
        
        # --- WALL FOLLOWING LOGIC (Only runs if front is clear) ---
        elif dist_right > 0.3:
            # Too far: Approach the wall
            self.get_logger().info("Far from wall: Veering Right.")
            action.linear.x = 0.15
            action.angular.z = -0.25 # Turn right
            
        elif dist_right < 0.2:
            # Too close: Move away from wall
            self.get_logger().info("Too close: Veering Left.")
            action.linear.x = 0.15
            action.angular.z = 0.25  # Turn left
            
        else:
            # Sweet spot: Stay the course
            action.linear.x = 0.2
            action.angular.z = 0.0

        self.vel_publisher.publish(action)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.vel_publisher.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()