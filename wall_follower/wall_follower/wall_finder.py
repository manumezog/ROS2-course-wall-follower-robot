#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower_interfaces.srv import FindWall
import math
import time

class WallFinder(Node):
    def __init__(self):
        super().__init__('wall_finder_node')

        # Threading Setup
        self.service_group = ReentrantCallbackGroup()
        self.sensor_group = MutuallyExclusiveCallbackGroup()

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan, 'scan', self.laser_cb, 10, callback_group=self.sensor_group
        )
        self.srv = self.create_service(
            FindWall, 'find_wall', self.finder_callback, callback_group=self.service_group
        )

        self.last_laser = None
        self.new_data = False
        self.get_logger().info("Wall Finder (HEAVY FILTER + TIMEOUTS) Ready.")

    def laser_cb(self, msg):
        self.last_laser = msg
        self.new_data = True

    def stop_robot(self):
        self.vel_pub.publish(Twist())

    def get_filtered_ranges(self):
        if self.last_laser is None: return None
        # AGGRESSIVE FILTER: Ignore anything < 0.20m (Dust/Floor noise)
        # Replace INF with 10.0
        return [r if (r > 0.20 and not math.isinf(r)) else 10.0 for r in self.last_laser.ranges]

    def finder_callback(self, request, response):
        self.get_logger().info("--- STARTING ALIGNMENT ---")

        # Wait for data
        wait_start = time.time()
        while self.last_laser is None:
            if time.time() - wait_start > 5.0:
                self.get_logger().error("No Laser Data! Aborting.")
                return response
            time.sleep(0.1)

        # ==========================================
        # STEP 1: ORIENT TO CLOSEST OBJECT (FRONT)
        # ==========================================
        self.get_logger().info("Step 1: Finding closest wall (Max 15s)...")
        move = Twist()
        start_time = time.time()
        
        while rclpy.ok():
            # TIMEOUT GUARD
            if time.time() - start_time > 15.0:
                self.get_logger().warn("Step 1 Timeout! Forcing next step.")
                self.stop_robot()
                break

            if not self.new_data:
                time.sleep(0.01)
                continue
            self.new_data = False

            ranges = self.get_filtered_ranges()
            # Find minimum distance in the CLEANED data
            shortest_ray = min(ranges)
            shortest_idx = ranges.index(shortest_ray)
            num_rays = len(ranges)
            
            # DEBUG: See what the robot sees!
            print(f"S1: MinIdx: {shortest_idx} | Dist: {shortest_ray:.2f}   ", end='\r')

            # Tolerance: +/- 15 degrees (~30 indices)
            front_tolerance = 30 

            # If shortest ray is NOT in the front cone
            if shortest_idx > front_tolerance and shortest_idx < (num_rays - front_tolerance):
                # Rotate towards it
                move.angular.z = 0.4 if shortest_idx < (num_rays/2) else -0.4
                self.vel_pub.publish(move)
            else:
                self.stop_robot()
                self.get_logger().info(f"Step 1 Done. Facing Index {shortest_idx}")
                break

        time.sleep(0.5)

        # ==========================================
        # STEP 2: APPROACH WALL
        # ==========================================
        self.get_logger().info("Step 2: Approach...")
        while rclpy.ok():
            if not self.new_data:
                time.sleep(0.01)
                continue
            self.new_data = False

            ranges = self.get_filtered_ranges()
            front_dist = ranges[0] 
            
            if front_dist > 0.45: # Stop at 45cm
                move.linear.x = 0.1
                move.angular.z = 0.0
                self.vel_pub.publish(move)
            else:
                self.stop_robot()
                break

        time.sleep(0.5)

        # ==========================================
        # STEP 3: ALIGN TO RIGHT (INDEX 540)
        # ==========================================
        num_rays = len(self.last_laser.ranges)
        target_index = int(num_rays * 0.75) # 540
        tolerance = 20 # Wide tolerance to prevent infinite hunting
        
        self.get_logger().info(f"Step 3: Aligning to Index {target_index} (Right) (Max 15s)...")
        start_time = time.time()
        
        aligned_count = 0
        while rclpy.ok():
            # TIMEOUT GUARD
            if time.time() - start_time > 15.0:
                self.get_logger().warn("Step 3 Timeout! Force Starting.")
                self.stop_robot()
                break

            if not self.new_data:
                time.sleep(0.01)
                continue
            self.new_data = False

            ranges = self.get_filtered_ranges()
            min_val = min(ranges)
            min_idx = ranges.index(min_val)
            
            print(f"S3: Target: {target_index} | Curr: {min_idx}   ", end='\r')

            # Check alignment
            if min_idx < (target_index - tolerance) or min_idx > (target_index + tolerance):
                move.linear.x = 0.0
                move.angular.z = 0.3 # Rotate Left
                self.vel_pub.publish(move)
                aligned_count = 0
            else:
                aligned_count += 1
                self.stop_robot()
                if aligned_count > 3: # Shorter stability check
                    self.get_logger().info("SUCCESS: Aligned to Right.")
                    break

        response.wallfound = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WallFinder()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()