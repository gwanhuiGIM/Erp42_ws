#!/usr/bin/env python3
# 출처: Google Drive에서 발견, 원본 파일명 "trafficlight_stop_ros2.py" (2026-08-21).
# 원본 파일명은 오해 소지가 있음 -- 신호등 로직은 전혀 없고, 실제 내용은
# LiDAR /scan 섹터 클리어런스 기반 좌/우 회피 + waypoints_path pathtracking을
# Controller Node 없이 단일 노드로 통합한 아키텍처(DynamicWaypointNavigator).
# 어느 launch에도 배선된 근거 없음 -- 실행/실기 검증 안 됨(미검증).
"""
DynamicWaypointNavigator (ROS 2 Humble / Python 3)


- Path following from /waypoints_path (nav_msgs/Path) + /odometry/filtered/global (nav_msgs/Odometry)
- Obstacle avoidance from /scan (sensor_msgs/LaserScan) using LEFT/CENTER/RIGHT sectors
- Dynamic LEFT/RIGHT with hysteresis and a path-aware tiebreaker when L/R are similar
- Cooldown before resuming path
- Tunables:
    * center_block_dist, side_clear_min
    * similarity_ratio (ratio closeness for "similar")
    * side_delta_min_m (absolute distance gap needed to treat one side as clearly better)
    * sector_quantile (robust clearance quantile, default 0.20)
"""


import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, List, Tuple


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
# Adjust to your actual ERP message type name:
# from erp_driver.msg import ErpCmdMsg as erpCmdMsg
from erp_driver.msg import erpCmdMsg
from tf_transformations import euler_from_quaternion




class Mode(Enum):
    PATH_FOLLOW    = 0
    AVOID_TURN     = 1
    AVOID_COOLDOWN = 2




class Side(Enum):
    LEFT  = +1
    RIGHT = -1
    NONE  = 0




@dataclass
class SectorClearance:
    left: float = 0.0
    center: float = 0.0
    right: float = 0.0
    stamp_ns: int = 0




class DynamicWaypointNavigator(Node):
    def __init__(self):
        super().__init__('dynamic_waypoint_navigator')


        # ---- Path-follow params ----
        self.declare_parameter('lookahead_distance', 3.25)
        self.declare_parameter('max_linear_speed', 70)      # ERP-42 KPH*10
        self.declare_parameter('max_steer_angle', 2000)     # servo units magnitude
        self.declare_parameter('max_steer_deg', 28.0)
        self.declare_parameter('angular_tolerance', 0.1)
        self.declare_parameter('steer_smooth_alpha', 0.5)
        self.declare_parameter('control_hz', 20.0)


        # ---- Avoidance / sectors ----
        self.declare_parameter('center_span_deg', 40.0)     # +/- around 0°
        self.declare_parameter('side_start_deg', 30.0)      # start offset from center
        self.declare_parameter('side_span_deg', 50.0)       # width per side sector
        self.declare_parameter('center_block_dist', 6.0)    # m
        self.declare_parameter('side_clear_min', 4.0)       # m (eligibility)
        self.declare_parameter('similarity_ratio', 1.10)    # max/min ≤ this ⇒ similar
        self.declare_parameter('side_delta_min_m', 0.0)     # ABS gap needed along with ratio
        self.declare_parameter('sector_quantile', 0.20)     # 0..1 (robust clearance)


        # Hysteresis & timings
        self.declare_parameter('min_turn_s', 1.0)
        self.declare_parameter('flip_ratio', 1.25)
        self.declare_parameter('side_bias', 0.0)            # + favors LEFT, - favors RIGHT
        self.declare_parameter('clear_hold_s', 0.3)
        self.declare_parameter('scan_timeout_s', 0.5)


        # Turn/drive during avoidance
        self.declare_parameter('avoid_speed', 20)           # KPH*10
        self.declare_parameter('avoid_brake_cmd', 1)        # 155 to stop-turn
        self.declare_parameter('steer_left_cmd', +1200)
        self.declare_parameter('steer_right_cmd', -1200)


        # Optional external brake override (disabled by default)
        self.declare_parameter('use_brake_override', False)


        gp = self.get_parameter
        self.lookahead_distance = float(gp('lookahead_distance').value)
        self.max_linear_speed   = int(gp('max_linear_speed').value)
        self.max_steer_angle    = int(gp('max_steer_angle').value)
        self.max_steer_rad      = float(gp('max_steer_deg').value) * math.pi/180.0
        self.angular_tolerance  = float(gp('angular_tolerance').value)
        self.steer_smooth_alpha = float(gp('steer_smooth_alpha').value)
        self.control_hz         = float(gp('control_hz').value)


        self.center_span_deg    = float(gp('center_span_deg').value)
        self.side_start_deg     = float(gp('side_start_deg').value)
        self.side_span_deg      = float(gp('side_span_deg').value)
        self.center_block_dist  = float(gp('center_block_dist').value)
        self.side_clear_min     = float(gp('side_clear_min').value)
        self.similarity_ratio   = float(gp('similarity_ratio').value)
        self.side_delta_min_m   = float(gp('side_delta_min_m').value)
        self.sector_quantile    = float(gp('sector_quantile').value)


        self.min_turn_s         = float(gp('min_turn_s').value)
        self.flip_ratio         = float(gp('flip_ratio').value)
        self.side_bias          = float(gp('side_bias').value)
        self.clear_hold_s       = float(gp('clear_hold_s').value)
        self.scan_timeout_s     = float(gp('scan_timeout_s').value)


        self.avoid_speed        = int(gp('avoid_speed').value)
        self.avoid_brake_cmd    = int(gp('avoid_brake_cmd').value)
        self.steer_left_cmd     = int(gp('steer_left_cmd').value)
        self.steer_right_cmd    = int(gp('steer_right_cmd').value)


        self.use_brake_override = bool(gp('use_brake_override').value)


        # QoS
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_default = QoSProfile(depth=20)


        # Subs / pubs
        self.create_subscription(Odometry, '/odometry/filtered/global', self.odom_cb, qos_default)
        self.create_subscription(Path, '/waypoints_path', self.path_cb, qos_default)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_sensor)


        self.cmd_pub = self.create_publisher(erpCmdMsg, '/erp42_ctrl_cmd/path', qos_default)


        # State
        self.current_pose: Optional[Pose] = None
        self.path: List[Tuple[float, float]] = []
        self.current_goal_index: Optional[int] = None
        self.goal_reached = False
        self.path_initialized = False
        self.prev_steer = 0


        self.mode = Mode.PATH_FOLLOW
        self.clearance = SectorClearance()
        self.turn_side = Side.NONE
        self.turn_side_since_ns = 0
        self.resume_time_ns = 0
        self.last_info_log_ns = 0


        self.create_timer(1.0 / self.control_hz, self.on_timer)
        self.get_logger().info("DynamicWaypointNavigator (ROS2) started with distance-driven side selection.")


    # ---------------------------- Utilities ----------------------------
    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds


    def _publish_erp(self, speed: int, steer: int, brake: int, gear: int = 0, e_stop: bool = False):
        m = erpCmdMsg()
        m.e_stop = bool(e_stop)
        m.gear   = int(gear)
        m.speed  = int(speed)
        m.steer  = int(steer)
        m.brake  = int(brake)
        self.cmd_pub.publish(m)


    # ---------------------------- Callbacks ----------------------------
    def odom_cb(self, odom: Odometry):
        self.current_pose = odom.pose.pose
        if self.path and self.current_goal_index is None:
            self.current_goal_index = self.find_nearest_waypoint()
            self.get_logger().info(f"Odom init → start at waypoint {self.current_goal_index}")


    def path_cb(self, msg: Path):
        if not self.path_initialized:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.current_goal_index = self.find_nearest_waypoint() if self.current_pose else 0
            self.path_initialized = True
            self.get_logger().info(f"Loaded {len(self.path)} waypoints.")


    def scan_cb(self, scan: LaserScan):
        def percentile_dist(start_deg, end_deg, q: float):
            start_rad = math.radians(start_deg)
            end_rad   = math.radians(end_deg)
            a0, inc = scan.angle_min, scan.angle_increment
            n = len(scan.ranges)


            i0 = max(0, int((start_rad - a0) / inc))
            i1 = min(n - 1, int(math.ceil((end_rad - a0) / inc)))
            if i1 <= i0:
                return 0.0
            vals = []
            r = scan.ranges
            for i in range(i0, i1):
                v = r[i]
                if math.isfinite(v) and v > 0.01:
                    vals.append(v)
            if not vals:
                return 0.0
            vals.sort()
            k = max(0, min(len(vals) - 1, int(q * (len(vals) - 1))))
            return vals[k]


        q = max(0.0, min(1.0, self.sector_quantile))
        c_half = self.center_span_deg * 0.5
        center = percentile_dist(-c_half, +c_half, q)
        right  = percentile_dist(-(self.side_start_deg + self.side_span_deg), -self.side_start_deg, q)
        left   = percentile_dist(+self.side_start_deg,  self.side_start_deg + self.side_span_deg, q)


        self.clearance = SectorClearance(left=left, center=center, right=right, stamp_ns=self.now_ns())


    # ---------------------------- Core logic ----------------------------
    def _scan_fresh(self) -> bool:
        if self.clearance.stamp_ns == 0:
            return False
        return (self.now_ns() - self.clearance.stamp_ns) * 1e-9 <= self.scan_timeout_s


    def _center_blocked(self) -> bool:
        if not self._scan_fresh():
            return False
        c = self.clearance.center
        return (c > 0.0) and (c < self.center_block_dist)


    def compute_distance(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)


    def find_nearest_waypoint(self) -> int:
        if not self.current_pose or not self.path:
            return 0
        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        _, _, ryaw = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])


        dists = [self.compute_distance(rx, ry, px, py) for (px, py) in self.path]
        nearest_idx = int(dists.index(min(dists)))


        window = 5
        start = nearest_idx
        end = min(nearest_idx + window, len(self.path))
        best = nearest_idx
        best_cost = float('inf')
        for i in range(start, end):
            px, py = self.path[i]
            dist = dists[i]
            desired_yaw = math.atan2(py - ry, px - rx)
            h = abs(math.atan2(math.sin(desired_yaw - ryaw), math.cos(desired_yaw - ryaw)))
            if h > 1.0:
                continue
            cost = dist + h
            if cost < best_cost:
                best_cost = cost
                best = i


        if self.current_goal_index is not None:
            return max(self.current_goal_index, best)
        return best


    def _get_lookahead_index(self) -> int:
        if self.current_goal_index is None:
            return 0
        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        for i in range(self.current_goal_index, len(self.path)):
            px, py = self.path[i]
            if self.compute_distance(rx, ry, px, py) >= self.lookahead_distance:
                return i
        return len(self.path) - 1


    def _path_turn_preference(self) -> Side:
        if not self.current_pose or not self.path:
            return Side.NONE


        if self.current_goal_index is None:
            look_i = self.find_nearest_waypoint()
        else:
            look_i = self._get_lookahead_index()
            if look_i < self.current_goal_index:
                look_i = self.current_goal_index


        tx, ty = self.path[min(look_i, len(self.path)-1)]
        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        _, _, ryaw = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])


        desired = math.atan2(ty - ry, tx - rx)
        herr = math.atan2(math.sin(desired - ryaw), math.cos(desired - ryaw))
        if herr > 0.0:
            return Side.LEFT
        elif herr < 0.0:
            return Side.RIGHT
        else:
            return Side.NONE


    def _choose_side(self) -> Side:
        L = self.clearance.left
        R = self.clearance.right


        # apply constant bias (+ to LEFT, - to RIGHT)
        Lb = L + max(0.0, self.side_bias)
        Rb = R + max(0.0, -self.side_bias)


        left_ok  = (Lb >= self.side_clear_min)
        right_ok = (Rb >= self.side_clear_min)
        if not left_ok and not right_ok:
            return Side.NONE
        if left_ok and not right_ok:
            return Side.LEFT
        if right_ok and not left_ok:
            return Side.RIGHT


        bigger  = max(Lb, Rb)
        smaller = max(1e-6, min(Lb, Rb))
        ratio   = bigger / smaller
        delta   = abs(Lb - Rb)


        # "Clearly better" requires BOTH ratio and absolute delta
        if (ratio > self.similarity_ratio) and (delta >= self.side_delta_min_m):
            # mild stickiness to current side handled by min_turn_s/flip logic outside
            return Side.LEFT if Lb > Rb else Side.RIGHT


        # Similar → prefer side that turns toward the path
        pref = self._path_turn_preference()
        if pref == Side.LEFT and left_ok:
            return Side.LEFT
        if pref == Side.RIGHT and right_ok:
            return Side.RIGHT


        # fallback: keep current if any, else pick larger
        if self.turn_side in (Side.LEFT, Side.RIGHT):
            return self.turn_side
        return Side.LEFT if Lb >= Rb else Side.RIGHT


    # ---------------------------- Timer loop ----------------------------
    def on_timer(self):
        if not self.current_pose or not self.path or self.goal_reached:
            return


        center_blocked = self._center_blocked()


        if self.mode == Mode.PATH_FOLLOW:
            if center_blocked:
                self.mode = Mode.AVOID_TURN
                self.turn_side = Side.NONE
                self.turn_side_since_ns = self.now_ns()
                new_idx = self.find_nearest_waypoint()
                if self.current_goal_index is None or new_idx > self.current_goal_index:
                    self.current_goal_index = new_idx
            else:
                self._step_path_follow()
                return


        if self.mode == Mode.AVOID_TURN:
            now_ns = self.now_ns()
            if self.turn_side == Side.NONE:
                s = self._choose_side()
                if s != Side.NONE:
                    self.turn_side = s
                    self.turn_side_since_ns = now_ns
            else:
                elapsed = (now_ns - self.turn_side_since_ns) * 1e-9
                if elapsed >= self.min_turn_s:
                    s = self._choose_side()
                    # only flip if other side is clearly better (handled by ratio+delta gate)
                    if s != Side.NONE and s != self.turn_side:
                        self.turn_side = s
                        self.turn_side_since_ns = now_ns


            if self.turn_side == Side.NONE:
                self._publish_erp(0, 0, 155)  # both sides bad → stop
            else:
                steer = self.steer_left_cmd if self.turn_side == Side.LEFT else self.steer_right_cmd
                self._publish_erp(self.avoid_speed, steer, self.avoid_brake_cmd)


            # resume when center is clear again
            if self._scan_fresh() and self.clearance.center >= self.center_block_dist:
                self.mode = Mode.AVOID_COOLDOWN
                self.resume_time_ns = self.now_ns() + int(self.clear_hold_s * 1e9)
                new_idx = self.find_nearest_waypoint()
                if self.current_goal_index is None or new_idx > self.current_goal_index:
                    self.current_goal_index = new_idx
                self._publish_erp(0, 0, 155)
                return


        if self.mode == Mode.AVOID_COOLDOWN:
            if self._center_blocked():
                self.mode = Mode.AVOID_TURN
                return
            if self.now_ns() < self.resume_time_ns:
                self._publish_erp(0, 0, 155)
                return
            self.mode = Mode.PATH_FOLLOW
            self.turn_side = Side.NONE
            self._step_path_follow()


    # ---------------------------- Path following ----------------------------
    def _step_path_follow(self):
        if self.current_goal_index is None:
            self.current_goal_index = self.find_nearest_waypoint()


        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        _, _, ryaw = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])


        look_i = self._get_lookahead_index()
        if look_i < self.current_goal_index:
            look_i = self.current_goal_index
        self.current_goal_index = look_i


        tx, ty = self.path[self.current_goal_index]
        dist = self.compute_distance(rx, ry, tx, ty)


        if self.current_goal_index == len(self.path) - 1 and dist < self.angular_tolerance:
            self.get_logger().info("Final waypoint reached. Stopping.")
            self._publish_erp(0, 0, 155)
            self.goal_reached = True
            return


        desired = math.atan2(ty - ry, tx - rx)
        herr = math.atan2(math.sin(desired - ryaw), math.cos(desired - ryaw))
        herr = max(-self.max_steer_rad, min(self.max_steer_rad, herr))
        steer_cmd = int((herr / self.max_steer_rad) * self.max_steer_angle)
        steer_cmd = int(self.steer_smooth_alpha * steer_cmd + (1.0 - self.steer_smooth_alpha) * self.prev_steer)
        self.prev_steer = steer_cmd


        self._publish_erp(self.max_linear_speed, steer_cmd, 1)


        now_ns = self.now_ns()
        if (now_ns - self.last_info_log_ns) * 1e-9 > 1.0:
            self.get_logger().info(f"PATH: wp={self.current_goal_index} dist={dist:.2f} herr={herr:.2f} steer={steer_cmd}")
            self.last_info_log_ns = now_ns




def main():
    rclpy.init()
    node = DynamicWaypointNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()




if __name__ == '__main__':
    main()