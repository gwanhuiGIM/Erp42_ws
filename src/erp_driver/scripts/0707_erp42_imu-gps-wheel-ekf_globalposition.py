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
    """ Helper function to wrap angles to [-pi, pi] """
    return (angle + np.pi) % (2 * np.pi) - np.pi

class EkfGlobal(Node):
    def __init__(self):
        super().__init__('ekf_global')
        # Parameters
        self.declare_parameter('wheel_base', 1.040)
        self.declare_parameter('wheel_radius', 0.265)
        self.declare_parameter('encoder_ticks_per_rev', 100.0)
        self.declare_parameter('process_noise_xy', 25.0)
        self.declare_parameter('process_noise_theta_deg', 5.0)
        self.declare_parameter('process_noise_xy_min', 1.0)
        self.declare_parameter('process_noise_xy_max', 30.0)
        self.declare_parameter('process_noise_theta_deg_min', 2.0)
        self.declare_parameter('process_noise_theta_deg_max', 25.0)

        # Load parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value

        # Process/measurement noise
        self.proc_xy = self.get_parameter('process_noise_xy').value
        self.proc_th = math.radians(self.get_parameter('process_noise_theta_deg').value)
        self.Q_xy_min = self.get_parameter('process_noise_xy_min').value
        self.Q_xy_max = self.get_parameter('process_noise_xy_max').value
        self.Q_th_min = math.radians(self.get_parameter('process_noise_theta_deg_min').value)
        self.Q_th_max = math.radians(self.get_parameter('process_noise_theta_deg_max').value)

        # State: [x, y, heading]
        self.x = np.zeros(3)
        self.P = np.eye(3) * 1.0
        self.Q = np.diag([self.proc_xy**2, self.proc_xy**2, self.proc_th**2])

        # Origin for ENU
        self.origin_set = False
        self.lat0 = None
        self.lon0 = None

        # Last measurements
        self.last_time = self.get_clock().now()
        self.last_encoder = None
        self.last_gps = None
        self.last_gps_time = None

        # Command/feedback for adaptive Q
        self.last_cmd_speed = 0.0
        self.last_cmd_steer = 0.0
        self.last_twist_linear = 0.0

        # **[수정]** 주행 상태 변수 추가
        self.is_stopped = True
        self.is_reversing = False

        # IMU heading correction
        self.imu_heading = None        # Raw IMU yaw (ENU-aligned)
        self.imu_offset = 0.0          # Offset between IMU and GPS heading
        self.imu_offset_alpha = 0.1   # Blend rate for offset calibration
        self.imu_heading_measurement_R = math.radians(5.0) ** 2

        # Subscriptions and publications
        self.create_service(SetOrigin, '/set_origin', self.set_origin_callback)
        self.create_subscription(ErpCmdMsg, '/erp42_ctrl_cmd', self.cb_cmd, 10)
        self.create_subscription(ErpStatusMsg, '/erp42_status', self.cb_status, 10)
        self.create_subscription(Imu, '/vectornav/imu', self.cb_imu, 50)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.cb_gps, 5)
        self.odom_pub = self.create_publisher(Odometry, '/odom_ekf_global', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.last_timer_time = self.get_clock().now()

        self.get_logger().info('EKF Global node started: Fusing /erp42_ctrl_cmd, /erp42_status, and IMU for robust heading.')

    def set_origin_callback(self, request, response):
        self.lat0 = request.latitude
        self.lon0 = request.longitude
        self.origin_set = True
        self.get_logger().info(
            f"ENU origin set: lat={self.lat0}, lon={self.lon0}, alt=0.0"
        )
        response.success = True
        return response

    def cb_cmd(self, msg: ErpStatusMsg):
    # Predict using encoder (instead of unreliable msg.speed)
        dt = 0.05  # Timer interval

        if self.last_encoder is None:
            self.last_encoder = msg.encoder
            return

        delta_ticks = msg.encoder - self.last_encoder
        self.last_encoder = msg.encoder

    # Calculate distance and velocity from encoder
        distance = (delta_ticks / self.ticks_per_rev) * 2 * np.pi * self.wheel_radius
        v_mps = distance / dt  # Velocity in m/s

        steer_rad = math.radians(msg.steer / -71.0)
        self.predict(dt, v_mps, steer_rad)

        self.last_cmd_speed = v_mps * 3.6  # for logging if needed
        self.last_cmd_steer = steer_rad

    def cb_status(self, msg: ErpStatusMsg):
        ticks = msg.encoder
        if self.last_encoder is None:
            self.last_encoder = ticks
            return
        delta_ticks = ticks - self.last_encoder
        self.last_encoder = ticks
        distance = (delta_ticks / self.ticks_per_rev) * 2 * np.pi * self.wheel_radius
        steer_rad = math.radians(msg.steer / -71.0)
        v = distance / 0.05 if 0.05 > 0 else 0.0
        
        # **[수정]** 주행 상태 판단 로직
        if abs(v) < 0.02 and abs(self.last_cmd_speed) < 0.1:
            self.is_stopped = True
        else:
            self.is_stopped = False
        
        self.is_reversing = v < -0.1

        # Adaptive Q
        v_feedback = v * 3.6
        speed_error = abs(self.last_cmd_speed - v_feedback)
        steer_error = abs(self.last_cmd_steer - steer_rad)
        speed_norm = min(speed_error / 10.0, 1.0)
        steer_norm = min(steer_error / 0.2, 1.0)
        error_level = (speed_norm + steer_norm) / 2.0
        Q_xy = self.Q_xy_min + (self.Q_xy_max - self.Q_xy_min) * error_level
        Q_th = self.Q_th_min + (self.Q_th_max - self.Q_th_min) * error_level
        self.Q = np.diag([Q_xy**2, Q_xy**2, Q_th**2])

        # Measurement update for x/y (odometry)
        theta = self.x[2]
        pred_x = self.x[0] + distance * np.cos(theta)
        pred_y = self.x[1] + distance * np.sin(theta)
        z = np.array([pred_x, pred_y])
        R_odom = np.diag([0.01**2, 0.01**2])
        H = np.array([[1, 0, 0], [0, 1, 0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + R_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P
        self.last_twist_linear = v

    def cb_imu(self, msg: Imu):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # Align IMU yaw: ENU (East=0, North=pi/2)
        self.imu_heading = wrap_angle(yaw - np.deg2rad(-90))
        msg.angular_velocity
        msg.linear_acceleration
    def cb_gps(self, msg: NavSatFix):
        if not self.origin_set:
            return

        # **[수정]** 정지 상태일 때 GPS 업데이트 건너뛰기
        if self.is_stopped:
            self.get_logger().debug('Vehicle is stopped, skipping GPS update.')
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

        # Calibrate IMU heading offset if GPS is moving and confident
        if self.last_gps is not None:
            dx = x_enu - self.last_gps[0]
            dy = y_enu - self.last_gps[1]
            moving = np.hypot(dx, dy) > 0.1
            ok_cov = ((msg.status.status == 2) or
                      (msg.status.status == 0 and
                       msg.position_covariance[0] < 0.01 and
                       msg.position_covariance[4] < 0.01))
            
            if moving and ok_cov and self.imu_heading is not None:
                gps_heading = np.arctan2(dy, dx)
                
                # **[수정]** 후진 시 GPS 헤딩 방향 보정
                if self.is_reversing:
                    gps_heading = wrap_angle(gps_heading + np.pi)

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

        # FUSE IMU+OFFSET AS HEADING (theta) MEASUREMENT
        if self.imu_heading is not None:
            imu_heading_corrected = wrap_angle(self.imu_heading + self.imu_offset)
            z = np.array([imu_heading_corrected])
            R = np.array([[self.imu_heading_measurement_R]])
            H = np.array([[0, 0, 1]])
            y = z - H @ self.x
            y[0] = wrap_angle(y[0])
            S = H @ self.P @ H.T + R
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x += (K @ y).flatten()
            self.x[2] = wrap_angle(self.x[2])
            self.P = (np.eye(3) - K @ H) @ self.P

        self.publish_odom(corrected_heading=float(self.x[2]))

    def predict(self, dt, v, delta):
        if dt <= 0:
            return
        theta = self.x[2]
        dx = v * dt * np.cos(theta)
        dy = v * dt * np.sin(theta)
        dtheta = (v * dt / self.wheel_base) * np.tan(delta)
        self.x += np.array([dx, dy, dtheta])
        self.x[2] = wrap_angle(self.x[2])
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()