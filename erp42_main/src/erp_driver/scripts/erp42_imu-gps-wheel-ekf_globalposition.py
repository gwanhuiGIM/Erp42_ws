#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from erp_interfaces.msg import ErpStatusMsg, ErpCmdMsg
from erp_interfaces.srv import SetOrigin
import numpy as np
import pymap3d as pm
import math

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class EkfGlobal(Node):
    def __init__(self):
        super().__init__('ekf_global')
        self.declare_parameter('wheel_base', 1.040)
        self.declare_parameter('wheel_radius', 0.265)
        self.declare_parameter('encoder_ticks_per_rev', 100.0)
        self.declare_parameter('process_noise_xy', 0.5)
        self.declare_parameter('process_noise_theta_deg', 5.0)
        self.declare_parameter('process_noise_xy_min', 0.5)
        self.declare_parameter('process_noise_xy_max', 4.0)
        self.declare_parameter('process_noise_theta_deg_min', 2.0)
        self.declare_parameter('process_noise_theta_deg_max', 25.0)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value

        self.proc_xy = self.get_parameter('process_noise_xy').value
        self.proc_th = math.radians(self.get_parameter('process_noise_theta_deg').value)
        self.Q_xy_min = self.get_parameter('process_noise_xy_min').value
        self.Q_xy_max = self.get_parameter('process_noise_xy_max').value
        self.Q_th_min = math.radians(self.get_parameter('process_noise_theta_deg_min').value)
        self.Q_th_max = math.radians(self.get_parameter('process_noise_theta_deg_max').value)

        self.x = np.zeros(3)
        self.P = np.eye(3) * 1.0
        self.Q = np.diag([self.proc_xy**2, self.proc_xy**2, self.proc_th**2])

        self.origin_set = False
        self.lat0 = None
        self.lon0 = None

        self.last_time = self.get_clock().now()
        self.last_encoder = None

        self.last_gps = None
        self.last_gps_time = None
        self.is_stopped = False

        # Command/feedback for adaptive Q
        self.last_cmd_speed = 0.0
        self.last_cmd_steer = 0.0
        self.last_twist_linear = 0.0

        # IMU heading correction offset
        self.imu_offset = 0.0
        self.imu_offset_alpha = 0.1   # Smoothing factor
        self.imu_heading = None       # Most recent IMU yaw (radians)
        self.last_gps_heading = None  # Last computed GPS heading (radians)

        self.create_subscription(ErpCmdMsg, '/erp42_ctrl_cmd', self.cb_cmd, 10)
        self.create_subscription(ErpStatusMsg, '/erp42_status', self.cb_status, 10)
        self.create_subscription(Imu, '/vectornav/imu', self.cb_imu, 50)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.cb_gps, 5)
        self.odom_pub = self.create_publisher(Odometry, '/odom_ekf', 10)
        self.set_origin_client = self.create_client(SetOrigin, '/set_origin')

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.last_timer_time = self.get_clock().now()

    def call_set_origin(self, lat, lon):
        if not self.set_origin_client.service_is_ready():
            self.get_logger().warn('SetOrigin service not available yet, will retry next GPS fix.')
            return False
        req = SetOrigin.Request()
        req.latitude = float(lat)
        req.longitude = float(lon)
        future = self.set_origin_client.call_async(req)
        future.add_done_callback(self.handle_set_origin_response)
        return True

    def handle_set_origin_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.origin_set = True
                self.get_logger().info('SetOrigin service call successful and origin_set=True')
            else:
                self.origin_set = False
                self.get_logger().error('SetOrigin service call returned failure!')
        except Exception as e:
            self.origin_set = False
            self.get_logger().error(f'SetOrigin service call failed: {e}')

    def cb_cmd(self, msg: ErpCmdMsg):
        # Predict with /erp42_ctrl_cmd
        dt = 0.05  # Or use msg header time difference if available
        v_kph = msg.speed / 10.0
        v_mps = v_kph * (1000.0 / 3600.0)
        steer_rad = math.radians(msg.steer / -71.0)
        dist = v_mps * dt
        self.predict(dt, v_mps, steer_rad)
        self.last_cmd_speed = v_kph
        self.last_cmd_steer = steer_rad

    def cb_status(self, msg: ErpStatusMsg):
        # Use encoder as odometry measurement to correct x, y
        now = self.get_clock().now()
        ticks = msg.encoder
        if self.last_encoder is None:
            self.last_encoder = ticks
            return
        delta_ticks = ticks - self.last_encoder
        self.last_encoder = ticks
        distance = (delta_ticks / self.ticks_per_rev) * 2 * np.pi * self.wheel_radius
        steer_rad = math.radians(msg.steer / -71.0)
        v = distance / 0.05 if 0.05 > 0 else 0.0
        # Adaptive Q based on error between commanded and feedback speed/steer
        v_feedback = v * 3.6   # m/s to kph
        speed_error = abs(self.last_cmd_speed - v_feedback)
        steer_error = abs(self.last_cmd_steer - steer_rad)
        speed_norm = min(speed_error / 10.0, 1.0)
        steer_norm = min(steer_error / 0.2, 1.0)
        error_level = (speed_norm + steer_norm) / 2.0
        Q_xy = self.Q_xy_min + (self.Q_xy_max - self.Q_xy_min) * error_level
        Q_th = self.Q_th_min + (self.Q_th_max - self.Q_th_min) * error_level
        self.Q = np.diag([Q_xy**2, Q_xy**2, Q_th**2])

        # Correction: measurement update for x/y (as in classic EKF)
        theta = self.x[2]
        pred_x = self.x[0] + distance * np.cos(theta)
        pred_y = self.x[1] + distance * np.sin(theta)
        z = np.array([pred_x, pred_y])
        R_odom = np.diag([0.01**2, 0.01**2])  # Feedback measurement noise
        H = np.array([[1, 0, 0], [0, 1, 0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + R_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P
        self.last_twist_linear = v

    def cb_imu(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_heading = wrap_angle(yaw - np.deg2rad(90))

    def cb_gps(self, msg: NavSatFix):
        if not self.origin_set:
            self.lat0 = msg.latitude
            self.lon0 = msg.longitude
            ok = self.call_set_origin(self.lat0, self.lon0)
            if ok:
                self.get_logger().info(f"Calling /set_origin with lat={self.lat0}, lon={self.lon0}")
            return
        x_enu, y_enu, _ = pm.geodetic2enu(msg.latitude, msg.longitude, 0.0,
                                          self.lat0, self.lon0, 0.0)
        z = np.array([x_enu, y_enu])
        cov = msg.position_covariance
        R = np.diag([cov[0], cov[4]])
        H = np.array([[1, 0, 0], [0, 1, 0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

        # GPS-based heading update (for offset, not filter!)
        if self.last_gps is not None:
            dx = x_enu - self.last_gps[0]
            dy = y_enu - self.last_gps[1]
            moving = np.hypot(dx, dy) > 0.2
            ok_cov = ((msg.status.status == 2) or
                      (msg.status.status == 0 and
                       msg.position_covariance[0] < 0.5 and
                       msg.position_covariance[4] < 0.5))
            if moving and ok_cov and self.imu_heading is not None:
                gps_heading = np.arctan2(dy, dx)
                heading_err = wrap_angle(gps_heading - (self.imu_heading + self.imu_offset))
                self.imu_offset = wrap_angle(self.imu_offset + self.imu_offset_alpha * heading_err)
        self.last_gps = (x_enu, y_enu)
        self.last_gps_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds * 1e-9
        self.last_timer_time = now
        if dt > 0:
            self.predict(dt, 0.0, 0.0)
        # Use IMU + offset as final heading if IMU is available
        if self.imu_heading is not None:
            corrected_heading = wrap_angle(self.imu_heading + self.imu_offset)
        else:
            corrected_heading = float(self.x[2])  # fallback to filter value if no IMU yet
        self.publish_odom(corrected_heading=corrected_heading)

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
    node.get_logger().info('EKF Global node started: ctrl_cmd predict, status measurement, IMU-GPS heading offset.')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
