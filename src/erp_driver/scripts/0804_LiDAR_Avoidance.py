#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from erp_interfaces.msg import ErpStatusMsg, ErpCmdMsg
import sensor_msgs_py.point_cloud2 as pc2
from math import atan2, degrees, pi

class FlatProjectionAvoidanceNode(Node):
    def __init__(self):
        super().__init__('flat_projection_avoidance_node')
        self.sub = self.create_subscription(PointCloud2, '/velo dyne_points', self.lidar_cb, 10)
        self.pub = self.create_publisher(ErpCmdMsg, '/erp42_ctrl_cmd/lidar', 10)
        self.cmd = ErpCmdMsg()
        self.timer = self.create_timer(0.04, self.control_loop)
        self.points = []

    def lidar_cb(self, msg):
        # PointCloud2를 (x, y, z, intensity) 포맷으로 파싱
        raw_points = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)

        # 전처리: 지면 투영 (z 필터링하여 평면 유지)
        self.points = []
        for x, y, z, _ in raw_points:
            if abs(z) < 0.5:  # ground 또는 가까운 물체 중심층 필터
                self.points.append((x, y))

    def control_loop(self):
        obstacle_angles = []

        for x, y in self.points:
            dist = (x**2 + y**2)**0.5
            angle = degrees(atan2(y, x))
            if 0.4 < dist < 1.5 and -60 <= angle <= 60:
                obstacle_angles.append(angle)

        if not obstacle_angles:
            avg_degree = 0
            self.get_logger().info("No obstacles — going straight")
        else:
            obstacle_angles.sort()
            left_space = 60 - obstacle_angles[-1]
            right_space = 60 + obstacle_angles[0]

            self.get_logger().info(f"Obstacle angles: {obstacle_angles}")
            self.get_logger().info(f"Left space: {left_space:.1f}°, Right space: {right_space:.1f}°")

            if max(left_space, right_space) == left_space:
                avg_degree = 60 - (left_space / 2)
                self.get_logger().info(f"Going left → steer to {avg_degree:.1f}°")
            else:
                avg_degree = -60 + (right_space / 2)
                self.get_logger().info(f"Going right → steer to {avg_degree:.1f}°")

            avg_degree = int(-avg_degree)  # 차량 좌표계 기준 반전

        # 차량 제어 명령 생성
        self.cmd.steer = avg_degree * 71  # 차량 모델에 맞는 비율 적용
        self.cmd.speed = 1
        self.cmd.brake = 0 if avg_degree == 0 else 0
        self.pub.publish(self.cmd)

def main():
    rclpy.init()
    node = FlatProjectionAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
