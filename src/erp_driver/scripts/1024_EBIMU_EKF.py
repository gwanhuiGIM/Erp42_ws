#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from erp_interfaces.msg import ErpStatusMsg
from erp_interfaces.srv import SetOrigin
import numpy as np
import pymap3d as pm
import math

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class EkfGlobal(Node):
    def __init__(self):
        super().__init__('ekf_global')

        # === Parameters ===
        self.declare_parameter('wheel_base', 1.040)
        self.declare_parameter('wheel_radius', 0.265)
        self.declare_parameter('encoder_ticks_per_rev', 100.0)
        self.declare_parameter('process_noise_xy', 0.5)
        self.declare_parameter('process_noise_theta_deg', 5.0)
        self.declare_parameter('process_noise_xy_min', 0.5)
        self.declare_parameter('process_noise_xy_max', 4.0)
        self.declare_parameter('process_noise_theta_deg_min', 2.0)
        self.declare_parameter('process_noise_theta_deg_max', 25.0)

        # === Load params ===
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.proc_xy = self.get_parameter('process_noise_xy').value
        self.proc_th = math.radians(self.get_parameter('process_noise_theta_deg').value)
        self.Q_xy_min = self.get_parameter('process_noise_xy_min').value
        self.Q_xy_max = self.get_parameter('process_noise_xy_max').value
        self.Q_th_min = math.radians(self.get_parameter('process_noise_theta_deg_min').value)
        self.Q_th_max = math.radians(self.get_parameter('process_noise_theta_deg_max').value)

        # === EKF Variables ===
        self.x = np.zeros(3)
        self.P = np.eye(3)
        self.Q = np.diag([self.proc_xy**2, self.proc_xy**2, self.proc_th**2])

        self.origin_set = False
        self.lat0 = None
        self.lon0 = None
        self.last_encoder = None
        self.last_twist_linear = 0.0

        # === IMU variables ===
        self.imu_heading = None
        self.imu_offset = 0.0
        self.imu_offset_alpha = 0.1

        # === ROS Interfaces ===
        self.create_subscription(ErpStatusMsg, '/erp42_status', self.cb_status, 10)
        self.create_subscription(String, '/ebimu_data', self.cb_ebimu, 50)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.cb_gps, 5)
        self.odom_pub = self.create_publisher(Odometry, '/odom_ekf', 10)

        # ✅ /set_origin service server 추가 (WaypointPublisher와 동일 구조)
        self.set_origin_srv = self.create_service(SetOrigin, '/set_origin', self.handle_set_origin_service)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.last_timer_time = self.get_clock().now()

        self.get_logger().info('EKF Global node with EBIMU + /set_origin server started.')

    # === /set_origin 서비스 콜백 ===
    def handle_set_origin_service(self, request, response):
        self.lon0 = request.longitude
        self.lat0 = request.latitude
        self.origin_set = True
        response.success = True
        self.get_logger().info(f'✅ Origin manually set: lat={self.lat0}, lon={self.lon0}')
        return response

    # === EBIMU 데이터 파싱 ===
    def cb_ebimu(self, msg: String):
        try:
            raw = msg.data.strip().replace('\r', '').replace('\n', '')
            parts = raw.split(',')
            if len(parts) < 9:
                return

            # 예시: "100-0,-0.8650,-0.0092,0.0007,0.5015,0.0,0.0,0.0,94"
            qw = float(parts[4])
            qx = float(parts[1])
            qy = float(parts[2])
            qz = float(parts[3])

            roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
            self.imu_heading = wrap_angle(yaw - np.deg2rad(90))

        except Exception as e:
            self.get_logger().warn(f"IMU parse error: {e}")

    # === ERP 상태 콜백 ===
    def cb_status(self, msg: ErpStatusMsg):
        now = self.get_clock().now()
        ticks = msg.encoder
        if self.last_encoder is None:
            self.last_encoder = ticks
            return

        delta_ticks = ticks - self.last_encoder
        self.last_encoder = ticks
        distance = (delta_ticks / self.ticks_per_rev) * 2 * np.pi * self.wheel_radius
        steer_rad = math.radians(msg.steer / -71.0)
        v = distance / 0.05
        self.predict(0.05, v, steer_rad)
        self.last_twist_linear = v

    # === GPS 콜백 ===
    def cb_gps(self, msg: NavSatFix):
        if not self.origin_set:
            self.get_logger().warn("Origin not set. Waiting for /set_origin service call...")
            return

        x_enu, y_enu, _ = pm.geodetic2enu(msg.latitude, msg.longitude, 0.0,
                                          self.lat0, self.lon0, 0.0)
        z = np.array([x_enu, y_enu])
        cov = msg.position_covariance
        R = np.diag([cov[0] if cov[0] > 0 else 1.0,
                     cov[4] if cov[4] > 0 else 1.0])

        H = np.array([[1, 0, 0], [0, 1, 0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

    # === EKF 예측 ===
    def predict(self, dt, v, delta):
        if dt <= 0:
            return
        theta = self.x[2]
        dx = v * dt * np.cos(theta)
        dy = v * dt * np.sin(theta)
        dtheta = (v * dt / self.wheel_base) * np.tan(delta)
        self.x += np.array([dx, dy, dtheta])
        F = np.array([
            [1.0, 0.0, -v * dt * np.sin(theta)],
            [0.0, 1.0,  v * dt * np.cos(theta)],
            [0.0, 0.0, 1.0]
        ])
        self.P = F @ self.P @ F.T + self.Q

    # === 타이머 콜백 ===
    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds * 1e-9
        self.last_timer_time = now
        if dt > 0:
            self.predict(dt, 0.0, 0.0)

        corrected_heading = float(self.x[2])
        if self.imu_heading is not None:
            corrected_heading = wrap_angle(self.imu_heading + self.imu_offset)

        self.publish_odom(corrected_heading=corrected_heading)

    # === Odom 퍼블리시 ===
    def publish_odom(self, corrected_heading=None):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])
        heading = corrected_heading if corrected_heading is not None else float(self.x[2])
        quat = quaternion_from_euler(0, 0, heading)
        odom.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        odom.twist.twist.linear.x = self.last_twist_linear
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = EkfGlobal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
