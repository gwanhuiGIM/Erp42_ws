#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from erp_interfaces.msg import ErpStatusMsg, ErpCmdMsg
import sensor_msgs_py.point_cloud2 as pc2
from math import atan2, degrees

class EStop3DNode(Node):
    def __init__(self):
        super().__init__('wego_estop_3d')
        self.pub = self.create_publisher(ErpCmdMsg, '/erp42_ctrl_cmd/lidar', 10)
        self.sub = self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_cb, 10)
        self.cmd = ErpCmdMsg()
        self.timer = self.create_timer(0.04, self.control_loop)
        self.obstacle_detected = False

    def lidar_cb(self, msg):
        self.points = pc2.read_points(msg, field_names=("x","y","z","intensity"), skip_nans=True)

    def control_loop(self):
        self.obstacle_detected = False
        for x, y, z, intensity in self.points:
            dist = (x**2 + y**2)**0.5
            ang = degrees(atan2(y, x))
            if -15 <= ang <= 15 and 0.4 < dist < 1.5:
                self.obstacle_detected = True
                break

        if self.obstacle_detected:
            self.get_logger().info("Obstacle ahead — stopping/braking")
            self.cmd.steer = 0
            self.cmd.speed = 0
            self.cmd.brake = 155
        else:
            self.get_logger().info("Clear path — moving forward")
            self.cmd.steer = 0
            self.cmd.speed = 1
            self.cmd.brake = 0

        self.pub.publish(self.cmd)

def main():
    rclpy.init()
    node = EStop3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
