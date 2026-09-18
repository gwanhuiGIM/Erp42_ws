#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from erp_driver.msg import erpCmdMsg
from sensor_msgs.msg import LaserScan
from math import pi

class EStopNode(Node):
    def __init__(self):
        super().__init__('wego_estop_node')
        self.cmd_pub = self.create_publisher(erpCmdMsg, '/erp42_ctrl_cmd', 1)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.cmd_msg = erpCmdMsg()
        self.timer = self.create_timer(0.1, self.e_stop)
        self.lidar_msg = None

    def lidar_cb(self, msg):
        self.lidar_msg = msg

    def e_stop(self):
        if not self.lidar_msg:
            return

        obs = []
        cs = 0; cad = 0
        d_min = self.lidar_msg.angle_min * 180/pi
        d_inc = self.lidar_msg.angle_increment * 180/pi
        degrees = [d_min + i*d_inc for i in range(len(self.lidar_msg.ranges))]

        for idx, dist in enumerate(self.lidar_msg.ranges):
            deg = degrees[idx]
            if 0 < dist < 0.75 and abs(deg) < 60:
                obs.append(deg)
                if len(obs) > 1 and obs[-1] - obs[-2] > 5:
                    cs = obs[-1] - obs[-2]
                    cad = obs[-2] + cs/2

        avg_deg = 0
        if obs:
            left = 60 - obs[-1]
            right = 60 + obs[0]
            if max(left, right, cs) == left:
                avg_deg = 60 - left/2
                self.get_logger().info('Going left')
            elif max(left, right, cs) == right:
                avg_deg = -60 + right/2
                self.get_logger().info('Going right')
            else:
                avg_deg = cad
                self.get_logger().info('Going center')
            avg_deg = -avg_deg * 0.5

        steer = int(avg_deg * 71)
        self.cmd_msg.steer = steer
        self.cmd_pub.publish(self.cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
