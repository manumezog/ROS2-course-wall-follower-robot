#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile, HistoryPolicy
import math
import time
import sys

from wall_follower_interfaces.srv import FindWall
from wall_follower_interfaces.action import OdomRecord

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=10
        )
        
        # --- CLIENTS & PUBLISHERS ---
        self.srv_client = self.create_client(FindWall, 'find_wall')
        self.action_client = ActionClient(self, OdomRecord, 'record_odom')
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.subscriber_laser = self.create_subscription(
            LaserScan, '/scan', self.laserscan_callback, self.qos
        )
        
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # --- METRICS ---
        self.current_total_dist = 0.0
        self.dist_to_origin = 0.0
        self.start_x = None
        self.start_y = None
        
        # --- TUNING PARAMETERS ---
        self.setpoint = 0.30       
        self.kp = 1.2              
        self.kd = 6.0              
        self.prev_error = 0.0      
        self.last_valid_dist = 0.30 
        self.cruise_speed = 0.10   
        self.max_turn_rate = 1.2   
        
        self.find_wall_complete = False  
        self.get_logger().info("Wall Follower (ANTI-GHOST PATCH) Ready.")

    # --- ODOMETRY ---
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
        self.dist_to_origin = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)

    # --- SERVICE & ACTION ---
    def call_find_wall_service(self):
        while not self.srv_client.wait_for_service(timeout_sec=1.0):
            pass
        self.future = self.srv_client.call_async(FindWall.Request())
        self.future.add_done_callback(self.find_wall_done_cb)

    def find_wall_done_cb(self, future):
        self.send_odom_record_goal()

    def send_odom_record_goal(self):
        self.action_client.wait_for_server()
        goal_msg = OdomRecord.Goal()
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def feedback_callback(self, feedback_msg):
        self.current_total_dist = feedback_msg.feedback.current_total

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_cb)
        self.find_wall_complete = True

    def get_result_cb(self, future):
        self.vel_publisher.publish(Twist()) 
        time.sleep(1.0)
        sys.exit(0) 

    # --- CONTROL LOOP ---
    def laserscan_callback(self, msg):
        if not self.find_wall_complete: return

        num_rays = len(msg.ranges)
        
        # 1. FRONT SENSOR
        front_min = min(msg.ranges[0:20] + msg.ranges[-20:])
        if math.isinf(front_min) or front_min == 0.0: front_min = 10.0

        # 2. RIGHT SENSOR (WITH GHOST FIX)
        right_idx = int(num_rays * 0.75)
        right_cone = msg.ranges[right_idx-15 : right_idx+15]
        
        # FIX: Filter out 0.0 values which cause the "Death Loop"
        right_cone = [r for r in right_cone if r < 5.0 and r > 0.05]
        
        if len(right_cone) > 0:
            dist_right = min(right_cone)
        else:
            dist_right = self.last_valid_dist 

        # 3. ANTI-KNOT LOGIC
        # Only apply gap protection if we are NOT in a corner (Front is clear)
        # This prevents the robot from driving straight into a wall while turning.
        if front_min > 0.6 and dist_right > (self.last_valid_dist + 0.6):
            dist_right = self.setpoint 
        else:
            self.last_valid_dist = dist_right 

        self.get_logger().info(
            f"Wall: {dist_right:.2f}m | Front: {front_min:.2f}m | "
            f"Run: {self.current_total_dist:.2f}m",
            throttle_duration_sec=0.5
        )

        action = Twist()

        if front_min < 0.18:
            action.linear.x = -0.1
            action.angular.z = 0.0 
        elif front_min < 0.40:
            action.linear.x = 0.04 
            action.angular.z = 0.9 
            self.prev_error = 0.0
        else:
            error = self.setpoint - dist_right
            derivative = error - self.prev_error
            turn = (error * self.kp) + (derivative * self.kd)
            turn = max(min(turn, self.max_turn_rate), -self.max_turn_rate)
            
            if front_min < 0.6 and dist_right < 0.35:
                action.linear.x = 0.05 
            else:
                action.linear.x = self.cruise_speed
                
            action.angular.z = turn
            self.prev_error = error

        self.vel_publisher.publish(action)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    node.call_find_wall_service()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.shutdown()
    except KeyboardInterrupt:
        node.vel_publisher.publish(Twist()) 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()