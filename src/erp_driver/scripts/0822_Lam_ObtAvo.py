#!/usr/bin/env python3
# file: lidar_track_follower.py
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from sklearn.cluster import DBSCAN

from erp_interfaces.msg import ErpCmdMsg  # adjust if your msg name differs


class LidarTrackFollower(Node):
    """
    Assumptions:
      - LiDAR frame: +X forward, +Y left, +Z up (common for vehicle_base or LiDAR frame)
      - Cones create two roughly parallel rows (left/right). We take midpoints to form centerline.
      - ERP42 steer command range ~ [-2000, 2000] mapping to ~ ±28 deg.
      - ERP42 speed is typically kph*10 (check your interface). Defaults chosen conservatively.
    """

    def __init__(self):
        super().__init__('lidar_track_follower')

        # ---------------- Parameters (can be overridden via ROS params) ----------------
        self.declare_parameter('lookahead_distance', 3.0)
        self.declare_parameter('wheelbase', 1.04)                # ERP42 ~1.04 m
        self.declare_parameter('max_steer_deg', 28.0)
        self.declare_parameter('max_steer_cmd', 2000)

        # LiDAR filtering
        self.declare_parameter('z_min', -1.0)                    # keep near ground
        self.declare_parameter('z_max', 0.5)
        self.declare_parameter('x_min', 0.5)                     # ignore very close/behind
        self.declare_parameter('x_max', 25.0)
        self.declare_parameter('r_max', 30.0)

        # DBSCAN
        self.declare_parameter('dbscan_eps', 0.5)
        self.declare_parameter('dbscan_min_samples', 6)
        self.declare_parameter('y_side_threshold', 0.20)         # points with |y|<th are ignored for side split

        # Speed policy (kph*10 if using ERP42 default)
        self.declare_parameter('speed_straight', 30)
        self.declare_parameter('speed_medium', 22)
        self.declare_parameter('speed_slow', 15)
        self.declare_parameter('curv_thresh_medium', 0.20)       # rad of heading change (~11 deg)
        self.declare_parameter('curv_thresh_sharp', 0.35)        # (~20 deg)
        self.declare_parameter('speed_smoothing_alpha', 0.6)     # 0..1 (higher = smoother/slower to change)

        # ---------------- Load params ----------------
        self.Ld = float(self.get_parameter('lookahead_distance').value)
        self.L  = float(self.get_parameter('wheelbase').value)
        self.max_steer_deg  = float(self.get_parameter('max_steer_deg').value)
        self.max_steer_cmd  = int(self.get_parameter('max_steer_cmd').value)

        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.r_max = float(self.get_parameter('r_max').value)

        self.db_eps = float(self.get_parameter('dbscan_eps').value)
        self.db_min = int(self.get_parameter('dbscan_min_samples').value)
        self.y_side_th = float(self.get_parameter('y_side_threshold').value)

        self.speed_straight = int(self.get_parameter('speed_straight').value)
        self.speed_medium   = int(self.get_parameter('speed_medium').value)
        self.speed_slow     = int(self.get_parameter('speed_slow').value)
        self.curv_med       = float(self.get_parameter('curv_thresh_medium').value)
        self.curv_sharp     = float(self.get_parameter('curv_thresh_sharp').value)
        self.speed_alpha    = float(self.get_parameter('speed_smoothing_alpha').value)

        self.max_steer_rad = math.radians(self.max_steer_deg)

        # Smoothed speed state
        self._speed_filtered = self.speed_straight

        # ---------------- ROS I/O ----------------
        self.sub_cloud = self.create_subscription(
            PointCloud2, '/velodyne_points', self.lidar_callback, 10
        )
        self.pub_cmd = self.create_publisher(
            ErpCmdMsg, '/erp42_ctrl_cmd', 10
        )

        self.get_logger().info('LidarTrackFollower initialized.')

    # --------------------------------------------------------------------------
    # Utilities
    # --------------------------------------------------------------------------
    @staticmethod
    def _angle_diff(a, b):
        """smallest signed angle a-b in [-pi, pi]."""
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def _estimate_curvature_heading(self, midpoints: np.ndarray, target_idx: int) -> float:
        """
        Estimate curvature as heading change between the path start and around the target index.
        Returns absolute heading difference in radians.
        """
        # start heading (between first two valid points)
        i1 = 0
        i2 = min(1, midpoints.shape[0] - 1)
        if i2 == 0:
            return 0.0
        h_start = math.atan2(midpoints[i2, 1] - midpoints[i1, 1],
                             midpoints[i2, 0] - midpoints[i1, 0])

        # heading at/near target
        j2 = max(1, target_idx)
        j1 = j2 - 1
        h_target = math.atan2(midpoints[j2, 1] - midpoints[j1, 1],
                              midpoints[j2, 0] - midpoints[j1, 0])

        return abs(self._angle_diff(h_target, h_start))

    def _pure_pursuit(self, target_xy: np.ndarray) -> float:
        """
        Pure pursuit steering (radians). target_xy is [x, y] in vehicle frame.
        """
        Ld = np.linalg.norm(target_xy)
        if Ld < 1e-3:
            return 0.0

        alpha = math.atan2(target_xy[1], target_xy[0])  # angle to target
        # kappa = 2*sin(alpha)/Ld  -> steer = atan(L * kappa)
        steer_rad = math.atan(self.L * (2.0 * math.sin(alpha) / Ld))
        # clamp to physical limits
        steer_rad = max(-self.max_steer_rad, min(self.max_steer_rad, steer_rad))
        return steer_rad

    # --------------------------------------------------------------------------
    # Main callback
    # --------------------------------------------------------------------------
    def lidar_callback(self, cloud: PointCloud2):
        # 1) Convert to Nx3 array and filter
        pts = []
        for x, y, z in pc2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True):
            r = math.hypot(x, y)
            if (self.z_min <= z <= self.z_max and
                self.x_min <= x <= self.x_max and
                r <= self.r_max):
                pts.append((x, y, z))

        if len(pts) < 30:
            self.get_logger().debug('Too few points after filtering.')
            return

        pts = np.asarray(pts, dtype=np.float32)

        # 2) Cluster on XY
        try:
            db = DBSCAN(eps=self.db_eps, min_samples=self.db_min)
            labels = db.fit_predict(pts[:, :2])
        except Exception as e:
            self.get_logger().warn(f'DBSCAN failed: {e}')
            return

        # 3) Collect cluster centroids
        unique_labels = [l for l in set(labels) if l != -1]
        if len(unique_labels) < 2:
            self.get_logger().debug('Not enough clusters.')
            return

        centroids = []
        for lbl in unique_labels:
            cluster_pts = pts[labels == lbl]
            if cluster_pts.shape[0] == 0:
                continue
            c = cluster_pts.mean(axis=0)  # (x, y, z)
            centroids.append(c)

        if len(centroids) < 2:
            self.get_logger().debug('Not enough centroids.')
            return

        centroids = np.asarray(centroids, dtype=np.float32)

        # 4) Split into left/right by y
        left  = centroids[centroids[:, 1] >  self.y_side_th]
        right = centroids[centroids[:, 1] < -self.y_side_th]

        if left.shape[0] < 1 or right.shape[0] < 1:
            self.get_logger().debug('Missing left or right cone set.')
            return

        # 5) Sort along X (front to back) and pair by index
        left  = left[left[:, 0].argsort()]
        right = right[right[:, 0].argsort()]
        n = min(left.shape[0], right.shape[0])
        if n < 2:
            self.get_logger().debug('Too few pairs to form a path.')
            return

        midpoints = (left[:n, :2] + right[:n, :2]) * 0.5  # shape (n, 2)
        # keep only points ahead and reasonably spaced
        midpoints = midpoints[midpoints[:, 0] > 0.0]
        if midpoints.shape[0] < 2:
            self.get_logger().debug('Not enough forward midpoints.')
            return

        # 6) Choose a lookahead target along centerline
        dists = np.linalg.norm(midpoints, axis=1)
        idx_candidates = np.where(dists >= self.Ld)[0]
        target_idx = int(idx_candidates[0]) if idx_candidates.size > 0 else (midpoints.shape[0] - 1)
        target_xy = midpoints[target_idx]

        # 7) Estimate curvature via heading change and set speed
        heading_diff = self._estimate_curvature_heading(midpoints, target_idx)

        if heading_diff > self.curv_sharp:
            speed_cmd = self.speed_slow
        elif heading_diff > self.curv_med:
            speed_cmd = self.speed_medium
        else:
            speed_cmd = self.speed_straight

        # Smooth the speed to avoid oscillations
        self._speed_filtered = int(
            self.speed_alpha * self._speed_filtered + (1.0 - self.speed_alpha) * speed_cmd
        )

        # 8) Pure Pursuit steering
        steer_rad = self._pure_pursuit(target_xy)
        steer_cmd = int((steer_rad / self.max_steer_rad) * self.max_steer_cmd)
        steer_cmd = max(-self.max_steer_cmd, min(self.max_steer_cmd, steer_cmd))

        # 9) Publish ERP42 command
        cmd = ErpCmdMsg()
        cmd.e_stop = False
        cmd.gear = 0                 # forward
        cmd.speed = self._speed_filtered
        cmd.steer = -steer_cmd
        cmd.brake = 2                # no brake (adjust to your interface)

        self.pub_cmd.publish(cmd)

        self.get_logger().info(
            f"midN={midpoints.shape[0]} target=({target_xy[0]:.1f},{target_xy[1]:.1f}) "
            f"headingΔ={heading_diff:.2f} rad speed={cmd.speed} steer={cmd.steer}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarTrackFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




