#!/usr/bin/env python3
# ROS 2 Humble / Python 3.10
#
# EKF position (x,y,yaw) using:
#  - ERP42 status (encoder + steer) for kinematics
#  - GPS (NavSatFix) for global XY in ENU
#  - EBIMU string for orientation yaw (and we align it to GPS heading)
#
# Plus:
#  - /set_origin service (this node is the server)
#  - /ebimu_pose publisher (PoseStamped) to inspect IMU quaternion live
#  - /odom_ekf_global publisher (Odometry) for fused global odom
#
# CHANGE IN THIS VERSION:
#   Vehicle motion now uses IMU heading as the driving heading.
#   So when IMU says you're facing 90°, forward motion in odom goes along 90°,
#   not along the old internal yaw.
#

import math
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseStamped
from std_msgs.msg import String

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from erp_interfaces.msg import ErpStatusMsg
from erp_interfaces.srv import SetOrigin

import pymap3d as pm


def wrap_angle(rad: float) -> float:
    return (rad + np.pi) % (2.0 * np.pi) - np.pi


class EkfGlobal(Node):
    def __init__(self):
        super().__init__('ekf_global')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('wheel_base', 1.040)               # [m]
        self.declare_parameter('wheel_radius', 0.265)             # [m]
        self.declare_parameter('encoder_ticks_per_rev', 100.0)

        # process noise base / bounds (for adaptive Q)
        self.declare_parameter('process_noise_xy', 0.5)
        self.declare_parameter('process_noise_theta_deg', 5.0)
        self.declare_parameter('process_noise_xy_min', 0.5)
        self.declare_parameter('process_noise_xy_max', 4.0)
        self.declare_parameter('process_noise_theta_deg_min', 2.0)
        self.declare_parameter('process_noise_theta_deg_max', 25.0)

        # IMU mounting yaw offset (deg): vehicle_forward = imu_yaw - imu_mount_yaw_deg
        self.declare_parameter('imu_mount_yaw_deg', 90.0)

        # Auto-set origin from first GPS fix (if True)
        self.declare_parameter('auto_set_origin_from_first_fix', True)

        # Pull param values
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.ticks_per_rev = float(self.get_parameter('encoder_ticks_per_rev').value)

        proc_xy = float(self.get_parameter('process_noise_xy').value)
        proc_th = math.radians(float(self.get_parameter('process_noise_theta_deg').value))

        self.Q_xy_min = float(self.get_parameter('process_noise_xy_min').value)
        self.Q_xy_max = float(self.get_parameter('process_noise_xy_max').value)
        self.Q_th_min = math.radians(float(self.get_parameter('process_noise_theta_deg_min').value))
        self.Q_th_max = math.radians(float(self.get_parameter('process_noise_theta_deg_max').value))

        self.imu_mount_yaw = math.radians(float(self.get_parameter('imu_mount_yaw_deg').value))
        self.auto_origin = bool(self.get_parameter('auto_set_origin_from_first_fix').value)

        # -----------------------------
        # EKF state: [x, y, yaw]
        # -----------------------------
        self.x = np.zeros(3, dtype=float)
        self.P = np.eye(3, dtype=float) * 1.0
        self.Q = np.diag([proc_xy**2, proc_xy**2, proc_th**2])

        # ENU origin (global frame)
        self.origin_set = False
        self.lat0 = None
        she_lon0 = None  # We'll define properly below
        self.lon0 = None

        # Timing / odom bookkeeping
        self.last_time = None          # rclpy time of last ERP42 status
        self.last_encoder = None       # last encoder tick count
        self.last_twist_linear = 0.0   # m/s

        # For adaptive Q (compare commanded vs feedback)
        self.last_cmd_speed_kph = 0.0
        self.last_cmd_steer = 0.0  # [rad]

        # GPS info
        self.last_gps_xy = None
        self.last_gps_time = None

        # IMU heading + GPS alignment offset
        # imu_heading        : yaw from IMU (already rotated into vehicle forward frame)
        # imu_offset         : slowly-learned correction to align IMU heading to GPS heading
        self.imu_heading = None
        self.imu_offset = 0.0
        self.imu_offset_alpha = 0.5
        self.last_gps_heading = None

        # Store last full quaternion from EBIMU for debug/pose publisher
        self.last_imu_qx = 0.0
        self.last_imu_qy = 0.0
        self.last_imu_qz = 0.0
        self.last_imu_qw = 1.0

        # -----------------------------
        # ROS Interfaces
        # -----------------------------
        # Subscriptions
        self.create_subscription(ErpStatusMsg, '/erp42_status', self.cb_status, 60)
        self.create_subscription(String, '/ebimu_data', self.cb_ebimu, 100)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.cb_gps, 5)

        # Publications
        self.odom_pub = self.create_publisher(Odometry, '/odom_ekf_global', 10)

        # PoseStamped publisher for IMU orientation debug
        self.ebimu_pose_pub = self.create_publisher(PoseStamped, '/ebimu_pose', 10)

        # Service SERVER: /set_origin
        self.origin_srv = self.create_service(SetOrigin, '/set_origin', self.on_set_origin)
        self.get_logger().info("Service /set_origin is available in this node.")

        # Timer to publish regularly
        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info(
            'EKF Global started (IMU-driven heading for motion): '
            'ERP42 kinematics + encoder XY correction + GPS XY + EBIMU yaw '
            '(GPS-aligned), plus /ebimu_pose publisher.'
        )

    # =============================
    # Helper: current fused heading
    # =============================
    def _current_heading_for_motion(self):
        """
        Return the yaw we should trust RIGHT NOW for vehicle forward direction.
        Priority:
          1. imu_heading + imu_offset (IMU corrected by GPS alignment)
          2. fallback self.x[2] if IMU not available yet
        """
        if self.imu_heading is not None:
            return wrap_angle(self.imu_heading + self.imu_offset)
        else:
            return float(self.x[2])

    # =============================
    # Service: /set_origin
    # =============================
    def on_set_origin(self, request: SetOrigin.Request, response: SetOrigin.Response):
        """
        Allows external callers to set/override the ENU origin (lat/lon).
        If origin already exists, rebase current (x,y) so continuity is kept.
        """
        new_lat = float(request.latitude)
        new_lon = float(request.longitude)

        if self.origin_set:
            # Rebase state if origin changes after we're already running
            try:
                vx, vy, _ = pm.geodetic2enu(
                    self.lat0, self.lon0, 0.0,
                    new_lat, new_lon, 0.0
                )
                self.x[0] = float(self.x[0] + vx)
                self.x[1] = float(self.x[1] + vy)
                self.get_logger().info(
                    f"Rebased state to new origin by (+{vx:.3f}, +{vy:.3f}) m."
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to rebase coordinates to new origin: {e}"
                )

        self.lat0 = new_lat
        self.lon0 = new_lon
        self.origin_set = True
        response.success = True
        self.get_logger().info(
            f"SetOrigin accepted: lat={self.lat0:.8f}, lon={self.lon0:.8f} (origin_set=True)"
        )
        return response

    # =============================
    # ERP42 status callback
    # =============================
    def cb_status(self, msg: ErpStatusMsg):
        """
        Steps:
        - Latch first encoder + time if not ready yet.
        - Compute dt and distance from encoder ticks.
        - Predict EKF using ackermann kinematics, BUT using IMU heading for forward dir.
        - Adaptive Q and small XY correction.
        """
        now = self.get_clock().now()

        # --- Bootstrap guard ---
        if (self.last_time is None) or (self.last_encoder is None):
            self.last_time = now
            self.last_encoder = msg.encoder

            # initialize yaw to IMU heading if available
            init_heading = self._current_heading_for_motion()
            self.x[2] = init_heading

            # store "commanded" snapshot
            self.last_cmd_speed_kph = msg.speed / 10.0
            self.last_cmd_steer = math.radians(msg.steer / -71.0)
            return

        # --- Normal operation ---
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3
        self.last_time = now

        # encoder delta -> distance
        delta_ticks = msg.encoder - self.last_encoder
        self.last_encoder = msg.encoder

        distance = (
            (delta_ticks / self.ticks_per_rev)
            * 2.0
            * math.pi
            * self.wheel_radius
        )
        v_mps = distance / dt if dt > 0 else 0.0
        self.last_twist_linear = v_mps

        steer_rad = math.radians(msg.steer / -71.0)

        # pick heading for this motion step
        heading_used = self._current_heading_for_motion()

        # 1) Predict
        self._predict(dt, v_mps, steer_rad, heading_override=heading_used)

        # 2) Adaptive Q
        v_feedback_kph = v_mps * 3.6
        speed_err = abs(self.last_cmd_speed_kph - v_feedback_kph)
        steer_err = abs(self.last_cmd_steer - steer_rad)
        speed_norm = min(speed_err / 10.0, 1.0)
        steer_norm = min(steer_err / 0.2, 1.0)
        error_level = 0.5 * (speed_norm + steer_norm)

        Q_xy = self.Q_xy_min + (self.Q_xy_max - self.Q_xy_min) * error_level
        Q_th = self.Q_th_min + (self.Q_th_max - self.Q_th_min) * error_level
        self.Q = np.diag([Q_xy**2, Q_xy**2, Q_th**2])

        # 3) Mini XY correction (treat predicted pos as a noisy measurement)
        z_xy = np.array([self.x[0], self.x[1]], dtype=float)
        H = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ], dtype=float)
        R_odom = np.diag([0.01**2, 0.01**2])
        self._ekf_update(z_xy, H, R_odom)

        # Update "commanded" snapshot for next iteration
        self.last_cmd_speed_kph = msg.speed / 10.0
        self.last_cmd_steer = steer_rad

    # =============================
    # EBIMU callback (String)
    # =============================
    def cb_ebimu(self, msg: String):
        """
        Example line:
        "100-0,-0.6881,0.0331,-0.0180,0.7246,-14.1,7.9,13.1,-0.113,-0.013,0.964,57"
          idx0   idx1     idx2     idx3     idx4    ...
        Quaternion fields are [z, y, x, w] at [1,2,3,4].
        We reorder them to (x,y,z,w) for ROS.
        1) store imu_heading (yaw in vehicle frame)
        2) publish /ebimu_pose = PoseStamped with raw quaternion
        """
        try:
            raw = msg.data.strip().replace('\r', '').replace('\n', '')
            parts = raw.split(',')
            if len(parts) < 5:
                return

            # EBIMU gives: z,y,x,w
            qz = float(parts[1])
            qy = float(parts[2])
            qx = float(parts[3])
            qw = float(parts[4])

            # normalize quaternion
            qn = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if qn == 0.0 or not math.isfinite(qn):
                self.get_logger().warn("EBIMU quaternion invalid (norm==0 or NaN).")
                return
            qx /= qn
            qy /= qn
            qz /= qn
            qw /= qn

            # stash for debug publisher
            self.last_imu_qx = qx
            self.last_imu_qy = qy
            self.last_imu_qz = qz
            self.last_imu_qw = qw

            # get yaw
            roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

            # yaw -> vehicle forward frame
            heading_vehicle = wrap_angle(yaw - self.imu_mount_yaw)
            self.imu_heading = heading_vehicle

            # Publish PoseStamped so you can see IMU orientation live
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "imu_link"
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation = Quaternion(
                x=qx, y=qy, z=qz, w=qw
            )
            self.ebimu_pose_pub.publish(pose_msg)

        except Exception as e:
            self.get_logger().warn(f"EBIMU parse error: {e}")

    # =============================
    # GPS callback
    # =============================
    def cb_gps(self, msg: NavSatFix):
        """
        - Optionally auto-set origin from first fix.
        - EKF XY update using ENU from GPS.
        - Learn imu_offset so (IMU yaw + offset) matches GPS track heading.
        """
        if (not self.origin_set) and self.auto_origin:
            self.lat0 = float(msg.latitude)
            self.lon0 = float(msg.longitude)
            self.origin_set = True
            self.get_logger().info(
                f"Origin auto-set from first GPS: lat={self.lat0:.8f}, lon={self.lon0:.8f}"
            )
            # fall through

        if not self.origin_set:
            # still no origin (auto disabled and no /set_origin yet)
            return

        # Convert GPS to ENU
        x_enu, y_enu, _ = pm.geodetic2enu(
            msg.latitude, msg.longitude, 0.0,
            self.lat0, self.lon0, 0.0
        )

        # EKF XY update with GPS
        z_gps = np.array([x_enu, y_enu], dtype=float)
        cov = msg.position_covariance
        R_gps = np.diag([
            max(cov[0], 1e-3),
            max(cov[4], 1e-3)
        ])
        H = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ], dtype=float)
        self._ekf_update(z_gps, H, R_gps)

        # Use GPS displacement direction to adapt imu_offset (heading alignment)
        if self.last_gps_xy is not None:
            dx = x_enu - self.last_gps_xy[0]
            dy = y_enu - self.last_gps_xy[1]
            dist = math.hypot(dx, dy)
            moving_ok = dist > 0.2  # movement threshold for reliable heading

            good_cov = (
                (msg.status.status == 2) or
                (
                    msg.status.status == 0 and
                    cov[0] < 0.5 and
                    cov[4] < 0.5
                )
            )

            if moving_ok and good_cov and self.imu_heading is not None:
                gps_heading = math.atan2(dy, dx)
                err = wrap_angle(gps_heading - (self.imu_heading + self.imu_offset))
                self.imu_offset = wrap_angle(
                    self.imu_offset + self.imu_offset_alpha * err
                )
                self.last_gps_heading = gps_heading

        self.last_gps_xy = (x_enu, y_enu)
        self.last_gps_time = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )

    # =============================
    # Timer: publish odom
    # =============================
    def on_timer(self):
        now = self.get_clock().now()

        # keep yaw in state synced to IMU even if /erp42_status slows down
        if self.last_time is None:
            self.last_time = now
        else:
            dt_idle = (now - self.last_time).nanoseconds * 1e-9
            if dt_idle > 0.0:
                # zero-speed predict but still snap yaw toward IMU heading
                heading_used = self._current_heading_for_motion()
                self._predict(dt_idle, 0.0, 0.0, heading_override=heading_used)
                self.last_time = now

        # heading for odom orientation
        heading_out = self._current_heading_for_motion()
        self._publish_odom(heading_out)

    # =============================
    # EKF internals
    # =============================
    def _predict(self, dt, v, steer_rad, heading_override=None):
        """
        Ackermann bicycle model prediction.
        We integrate using heading_override (IMU heading) if provided.
        Then we force self.x[2] to stay consistent with that heading.
        """
        if dt <= 0.0:
            return

        # decide which heading to use for forward direction
        if heading_override is None:
            theta = float(self.x[2])
        else:
            theta = float(heading_override)

        # translation
        dx = v * dt * math.cos(theta)
        dy = v * dt * math.sin(theta)

        # yaw change from steering
        if abs(math.cos(steer_rad)) > 1e-6:
            dtheta = (v * dt / self.wheel_base) * math.tan(steer_rad)
        else:
            dtheta = 0.0

        new_theta = wrap_angle(theta + dtheta)

        # update state
        self.x[0] += dx
        self.x[1] += dy
        self.x[2] = new_theta  # force EKF yaw to track IMU+offset + kinematics

        # build Jacobian around theta we actually used
        F = np.array([
            [1.0, 0.0, -v * dt * math.sin(theta)],
            [0.0, 1.0,  v * dt * math.cos(theta)],
            [0.0, 0.0, 1.0]
        ], dtype=float)

        self.P = F @ self.P @ F.T + self.Q

    def _ekf_update(self, z, H, R):
        """Linear EKF measurement update."""
        y = z - (H @ self.x)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])
        I_KH = np.eye(3) - (K @ H)
        # numerically stable covariance (Joseph-ish)
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def _publish_odom(self, heading):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])

        q = quaternion_from_euler(0.0, 0.0, heading)
        odom.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        odom.twist.twist.linear.x = float(self.last_twist_linear)

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = EkfGlobal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
