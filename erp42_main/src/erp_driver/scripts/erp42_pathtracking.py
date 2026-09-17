#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry

# Adjust the import according to your package structure
from erp_interfaces.msg import ErpCmdMsg  # Replace if your ROS2 msg is different

from tf_transformations import euler_from_quaternion

class DynamicWaypointNavigator(Node):
    def __init__(self):
        super().__init__('dynamic_waypoint_navigator')

        # Parameters
        self.lookahead_distance = 3
        self.max_linear_speed = 50  # ERP42 speed in KPH * 10
        self.max_steer_angle = -2000  # ERP42 max steering command
        self.max_steer_rad = math.radians(28)  # 28 degrees to radians
        self.angular_tolerance = 0.25
        self.prev_steer = 0

        # State
        self.current_pose = None
        self.path = []
        self.current_goal_index = None
        self.goal_reached = False
        self.path_initialized = False
        self.current_lidar = None

        # Publishers and Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_ekf',
            self.odom_callback,
            10
        )
        self.path_sub = self.create_subscription(
            Path,
            '/waypoints_path1',
            self.path_callback,
            10
        )
        self.lidar_sub = self.create_subscription(
            ErpCmdMsg,
            '/erp42_ctrl_cmd/lidar',
            self.lidar_callback,
            10
        )
        self.erp_cmd_pub = self.create_publisher(
            ErpCmdMsg,
            '/erp42_ctrl_cmd',
            10
        )

    def odom_callback(self, odom_msg):
        self.current_pose = odom_msg.pose.pose
        if self.path and self.current_goal_index is None:
            self.current_goal_index = self.find_nearest_waypoint()
            self.get_logger().info(f"Odom updated. Starting at nearest waypoint index: {self.current_goal_index}")
        if self.current_pose and self.path and not self.goal_reached:
            self.navigate_to_waypoints()

    def path_callback(self, path_msg):
        if not self.path_initialized:
            self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
            self.get_logger().info(f"Received {len(self.path)} waypoints.")
            if self.current_pose:
                self.current_goal_index = self.find_nearest_waypoint()
                self.get_logger().info(f"Starting at waypoint {self.current_goal_index}")
            else:
                self.current_goal_index = 0
            self.path_initialized = True

    def lidar_callback(self, msg):
        self.current_lidar = msg

    def compute_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def find_nearest_waypoint(self):
        if not self.current_pose or not self.path:
            return 0
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Get yaw from orientation
        _, _, robot_yaw = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])

        distances = [self.compute_distance(robot_x, robot_y, wp_x, wp_y) for wp_x, wp_y in self.path]
        nearest_index = distances.index(min(distances))

        window_size = 5
        start_index = nearest_index
        end_index = min(nearest_index + window_size, len(self.path))
        best_index = nearest_index
        best_cost = float('inf')
        for i in range(start_index, end_index):
            wp_x, wp_y = self.path[i]
            distance = distances[i]
            desired_yaw = math.atan2(wp_y - robot_y, wp_x - robot_x)
            heading_error = abs(math.atan2(math.sin(desired_yaw - robot_yaw), math.cos(desired_yaw - robot_yaw)))
            if heading_error > 1.0:
                continue
            cost = distance * 0.7 + heading_error * 0.5
            if cost < best_cost:
                best_cost = cost
                best_index = i

        if self.current_goal_index is not None:
            return max(self.current_goal_index, best_index)
        return best_index

    def compute_heading_error(self, target_x, target_y, robot_yaw):
        desired_yaw = math.atan2(target_y - self.current_pose.position.y,
                                 target_x - self.current_pose.position.x)
        return math.atan2(math.sin(desired_yaw - robot_yaw),
                          math.cos(desired_yaw - robot_yaw))

    def get_lookahead_waypoint(self):
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        for i in range(self.current_goal_index, len(self.path)):
            wp_x, wp_y = self.path[i]
            distance = self.compute_distance(robot_x, robot_y, wp_x, wp_y)
            if distance >= self.lookahead_distance:
                return i
        return len(self.path) - 1

    def navigate_to_waypoints(self):
        if not self.current_pose or not self.path or self.goal_reached:
            return

        if self.current_lidar is not None and self.current_lidar.brake == 2:
            new_index = self.find_nearest_waypoint()
            if new_index > self.current_goal_index and new_index < self.current_goal_index + 8:
                self.get_logger().info(f"LiDAR control active: updating waypoint index from {self.current_goal_index} to {new_index}")
                self.current_goal_index = new_index

        if self.current_goal_index is None:
            self.current_goal_index = self.find_nearest_waypoint()
            self.get_logger().info(f"Initializing starting waypoint to {self.current_goal_index}")

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        _, _, robot_yaw = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])

        lookahead_index = self.get_lookahead_waypoint()
        if lookahead_index < self.current_goal_index:
            lookahead_index = self.current_goal_index
        self.current_goal_index = lookahead_index

        target_x, target_y = self.path[self.current_goal_index]
        distance_to_target = self.compute_distance(robot_x, robot_y, target_x, target_y)

        if self.current_goal_index == len(self.path) - 1 and distance_to_target < 1.5:
            self.get_logger().info("Final waypoint reached. Stopping robot.")
            self.stop_robot()
            self.goal_reached = True
            return

        heading_error = self.compute_heading_error(target_x, target_y, robot_yaw)
        if abs(heading_error) > self.max_steer_rad:
            heading_error = math.copysign(self.max_steer_rad, heading_error)

        erp_cmd = ErpCmdMsg()
        erp_cmd.e_stop = False
        erp_cmd.gear = 0  # Forward
        erp_cmd.speed = self.max_linear_speed

        alpha = 0.65
        steer_value = int((heading_error / self.max_steer_rad) * self.max_steer_angle)
        erp_cmd.steer = int(alpha * steer_value + (1 - alpha) * self.prev_steer)
        self.prev_steer = erp_cmd.steer

        erp_cmd.brake = 1  # No braking
        self.erp_cmd_pub.publish(erp_cmd)
        self.get_logger().info(
            f"Navigating: Waypoint={self.current_goal_index}, Distance={distance_to_target:.2f}, Heading Error={heading_error:.2f}"
        )

    def stop_robot(self):
        stop_cmd = ErpCmdMsg()
        stop_cmd.e_stop = False
        stop_cmd.gear = 0
        stop_cmd.speed = 0
        stop_cmd.steer = 0
        stop_cmd.brake = 155
        self.erp_cmd_pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    navigator = DynamicWaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

