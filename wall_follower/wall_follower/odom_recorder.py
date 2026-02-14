import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker 
from wall_follower_interfaces.action import OdomRecord
import math
import time
import csv
import os

class OdomRecorderNode(Node):
    def __init__(self):
        super().__init__('odom_recorder_node')
        self.callback_group = ReentrantCallbackGroup()
        
        self.last_odom = Point()
        self.odom_record = []
        self.first_odom = Point()
        self.total_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.is_recording = False # Flag to control recording

        # Publisher for RViz
        self.marker_pub = self.create_publisher(Marker, '/path_marker', 10)

        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self.callback_group
        )

        self.action_server = ActionServer(
            self, OdomRecord, '/record_odom', self.execute_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info("Recorder Ready. Will save on CTRL+C.")

    def odom_callback(self, msg):
        self.last_odom.x = msg.pose.pose.position.x
        self.last_odom.y = msg.pose.pose.position.y

    def publish_path_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.id = 0
        marker.scale.x = 0.05 
        marker.color.r = 0.0
        marker.color.g = 1.0 
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = self.odom_record
        self.marker_pub.publish(marker)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('--- RECORDING STARTED ---')
        
        # Reset Logic
        self.first_odom.x = self.last_odom.x
        self.first_odom.y = self.last_odom.y
        self.last_x = self.last_odom.x
        self.last_y = self.last_odom.y
        self.total_distance = 0.0
        self.odom_record = []
        self.is_recording = True

        feedback_msg = OdomRecord.Feedback()

        while rclpy.ok():
            if not self.is_recording: break

            current_x = self.last_odom.x
            current_y = self.last_odom.y
            
            p = Point(x=current_x, y=current_y)
            self.odom_record.append(p)
            self.publish_path_marker()

            dx = current_x - self.last_x
            dy = current_y - self.last_y
            dist_step = math.sqrt(dx*dx + dy*dy)

            if dist_step > 0.005: # Record small movements too
                self.total_distance += dist_step
                self.last_x = current_x
                self.last_y = current_y

            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)

            dist_to_start = math.sqrt((current_x - self.first_odom.x)**2 + (current_y - self.first_odom.y)**2)
            
            # LAP COMPLETE CHECK
            if self.total_distance > 3.0 and dist_to_start < 0.35:
                self.get_logger().info("LAP COMPLETED. Saving Data...")
                self.save_to_csv()
                goal_handle.succeed()
                result = OdomRecord.Result()
                result.list_of_odoms = self.odom_record
                return result
            
            time.sleep(0.5)

    def save_to_csv(self):
        if not self.odom_record:
            self.get_logger().warn("No data to save!")
            return

        filename = 'robot_lap_data.csv'
        try:
            # We open in 'w' mode to overwrite previous run
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['X', 'Y'])
                for p in self.odom_record:
                    writer.writerow([p.x, p.y])
            self.get_logger().info(f"--- DATA SAVED: {os.path.abspath(filename)} ---")
        except Exception as e:
            self.get_logger().error(f"CSV Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomRecorderNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("CTRL+C Detected! Saving data before exit...")
    finally:
        # --- CRITICAL: SAVE ON EXIT ---
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()