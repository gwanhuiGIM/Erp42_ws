#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from erp_interfaces.msg import ErpCmdMsg
import time

class FusionControllerNode(Node):
    def __init__(self):
        super().__init__('fusion_controller_node')
        self.path_cmd = None
        self.lane_cmd = None
        self.last_path_time = 0
        self.last_lane_time = 0

        self.path_sub = self.create_subscription(
            ErpCmdMsg, '/erp42_ctrl_cmd/path', self.path_callback, 10)
        self.lane_sub = self.create_subscription(
            ErpCmdMsg, '/erp42_ctrl_cmd/lane', self.lane_callback, 10)
        self.cmd_pub = self.create_publisher(
            ErpCmdMsg, '/erp42_ctrl_cmd', 10)

        self.publish_rate = 0.05  # 20Hz
        self.timer = self.create_timer(self.publish_rate, self.publish_cmd)

        self.lane_cmd_timeout = 0.2  # seconds (hold last lane cmd for 0.2s)
        self.path_cmd_timeout = 0.2  # seconds (hold last path cmd for 0.2s)

    def path_callback(self, msg):
        self.path_cmd = msg
        self.last_path_time = time.time()

    def lane_callback(self, msg):
        self.lane_cmd = msg
        self.last_lane_time = time.time()

    def publish_cmd(self):
        now = time.time()
        lane_valid = self.lane_cmd and (now - self.last_lane_time) < self.lane_cmd_timeout and self.lane_cmd.brake == 3
        path_valid = self.path_cmd and (now - self.last_path_time) < self.path_cmd_timeout and self.path_cmd.brake == 2

        if lane_valid:
            cmd = self.lane_cmd
            # self.get_logger().info("Publishing LANE command")
        elif path_valid:
            cmd = self.path_cmd
            # self.get_logger().info("Publishing PATH command")
        else:
            # No valid command, stop vehicle
            cmd = ErpCmdMsg()
            cmd.e_stop = False
            cmd.gear = 0
            cmd.speed = 0
            cmd.steer = 0
            cmd.brake = 155  # Full brake
            # self.get_logger().warn("No valid command, stopping.")

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FusionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
