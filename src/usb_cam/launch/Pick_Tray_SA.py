# cuRobo v0.7.8 (v1, warp 1.8.x) — Isaac warp 1.8.2와 호환 같은 프로세스 OK.

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.robot.surface_gripper")
simulation_app.update()


from pathlib import Path
import sys
import time

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualSphere
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import String

_THIS_DIR = Path(__file__).resolve().parent

RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from dual_surface_gripper_adapter import DualSurfaceGripperAdapter
from m0609_pick_place_controller_surface import PickPlaceController
from m0609_rmpflow_controller import RMPFlowController

from m0609_curobo_controller import M0609CuroboController

# 팀원 RMPFlow move controller (PLACING 좌표 이동)
from m0609_move_controller import M0609MoveController


# ============================================================
# A. 에셋 경로
# ============================================================
FULL_SCENE_USD = "/home/rokey/cobot3_ws/isaacpjt/vaccum_gripper/Collected_full_scene/full_scene.usda"
TRAY_USD_PATH = str(
    _THIS_DIR
    / "Collected_model_redtray_scaled_for_180mm_pads"
    / "model_redtray_scaled_for_180mm_pads.usda"
)

# ── 수술 도구 8종 ──
TOOL_DIR = "/home/rokey/Downloads/Asset_Tools-main/SurgicalInstruments_A/Model"
TOOL_USDS = [
    f"{TOOL_DIR}/sm_bipolardissectingscissors_a01_01.usd",  # tray 0
    f"{TOOL_DIR}/sm_caliper_a01_01.usd",                    # tray 1
    f"{TOOL_DIR}/sm_clamps_a01_01.usd",                     # tray 2
    f"{TOOL_DIR}/sm_forceps_a01_01.usd",                    # tray 3
    f"{TOOL_DIR}/sm_handsaws_a01_01.usd",                   # tray 4
    f"{TOOL_DIR}/sm_knife_a01_01.usd",                      # tray 5
    f"{TOOL_DIR}/sm_ligatureneedle_a01_01.usd",             # tray 6
    f"{TOOL_DIR}/sm_mallet_a01_01.usd",                     # tray 7
    f"{TOOL_DIR}/sm_needle_a01_01.usd",                     # tray 8 (9번째 도구)
]
TOOL_DROP_HEIGHT = 0.05


ROBOT1_PRIM = "/World/m0609"
ROBOT2_PRIM = "/World/m0609_01"
EE_LINK_NAME = "link_6"

CUROBO_ROBOT_CONFIG = "/home/rokey/m0609_v1.yml"


def suction_paths(robot_prim):
    base = f"{robot_prim}/onrobot_rg2ft/gripper_body/dual_suction_tool"
    return [
        f"{base}/suction_contact_left/SurfaceGripper_left",
        f"{base}/suction_contact_right/SurfaceGripper_right",
    ]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8


# ============================================================
# B. 좌표
# ============================================================
M0609_URDF_PATH = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

TABLE_HEIGHT = 1.0

ROBOT1_BASE = np.array([0.5, 0.2, 1.0])
ROBOT2_BASE = np.array([-0.5, 0.2, 1.0])
MAX_REACH = 0.85

TRAY_TOP_Z = 0.0186
SUCTION_HEIGHT_DIFF = 0.184

TRAY_Z = 1.05
TRAY_ORIENT = np.array([0.7071, 0.0, 0.0, 0.7071])

# ── 트레이 9개 (3 x 3) ──
# 카메라 시야 내 배치를 위해 간격 0.20 m로 축소
# 인덱스 배치도 (행=y 방향, 열=x 방향):
#   col:  0       1      2
#   row0: [0]    [1]    [2]   ← y=0.55 (앞열)
#   row1: [3]    [4]    [5]   ← y=0.75 (중간열)
#   row2: [6]    [7]    [8]   ← y=0.95 (뒷열)
_tray_xs = [-0.20, 0.00, 0.20]   # x 간격 0.20 m
_tray_ys = [0.55, 0.75, 0.95]    # y 간격 0.20 m
TRAY_POSITIONS = []
for _y in _tray_ys:              # row-major: 직관적 인덱스 순서
    for _x in _tray_xs:
        TRAY_POSITIONS.append(np.array([_x, _y, TRAY_Z]))

# (row, col) → 인덱스 변환 헬퍼
def tray_index(row: int, col: int) -> int:
    """3x3 그리드에서 (row, col)로 트레이 인덱스 반환 (0-based)."""
    assert 0 <= row < 3 and 0 <= col < 3, f"row/col 범위 초과: ({row},{col})"
    return row * 3 + col

# Pick 명령 좌표 인덱스 명시 (예시)
PICK_INDEX = {
    "앞-왼쪽":   tray_index(0, 0),  # 0
    "앞-중앙":   tray_index(0, 1),  # 1
    "앞-오른쪽": tray_index(0, 2),  # 2
    "중-왼쪽":   tray_index(1, 0),  # 3
    "중-중앙":   tray_index(1, 1),  # 4
    "중-오른쪽": tray_index(1, 2),  # 5
    "뒤-왼쪽":   tray_index(2, 0),  # 6
    "뒤-중앙":   tray_index(2, 1),  # 7
    "뒤-오른쪽": tray_index(2, 2),  # 8
}

HAND_CENTER = np.array([0.0, 0.75, TABLE_HEIGHT])  # 3x3 중심열 기준

TARGET_TRAY_INDEX = tray_index(2, 2)   # 기본 타겟: 뒤-오른쪽(8)

# ★ 멀티 트레이 시퀀스 (순서대로 집고-추종-놓기): 9개 전체
TRAY_SEQUENCE = [
    tray_index(2, 2),  # 8 뒤-오른쪽
    tray_index(2, 1),  # 7 뒤-중앙
    tray_index(2, 0),  # 6 뒤-왼쪽
    tray_index(1, 2),  # 5 중-오른쪽
    tray_index(1, 1),  # 4 중-중앙
    tray_index(1, 0),  # 3 중-왼쪽
    tray_index(0, 2),  # 2 앞-오른쪽
    tray_index(0, 1),  # 1 앞-중앙
    tray_index(0, 0),  # 0 앞-왼쪽
]

DEFAULT_EE_OFFSET = np.array([0.0, 0.0, 0.20])

FOLLOW_MIN_Z = 1.18

# ── cuRobo FOLLOWING 속도 제한 (관성 떨굼 방지) ──
MAX_JOINT_STEP = 0.02   # 한 프레임 최대 관절 변화 (rad). 작을수록 느림/안정

# ── PLACING 파라미터 ──
# 좌표 접근(트레이 고정좌표) → 고정 관절각 정밀 안착
PLACE_LINK6_ABOVE_TRAY = 0.136  # link_6와 트레이표면 z차이 (좌표 접근용)
PLACE_HIGH_OFFSET = 0.10        # 먼저 그 위 몇 m로 이동
PLACE_APPROACH_GAP = 0.05       # MOVE_DOWN은 정밀안착 5cm 위까지만 (좌표)
PLACE_MOVE_TOL = 0.04           # 좌표 move 도착 판정
PLACE_JOINT_TOL = 0.02          # 관절 정밀안착 판정
PLACE_JOINT_STEP = 0.015        # 관절 안착 시 한 프레임 변화 (부드럽게)
PLACE_SETTLE = 10               # 안착 후 안정화 프레임

# ── 수평 가능 영역 clamp (z별 반경) ──
FOLLOW_Z_MIN = 1.10
FOLLOW_Z_MAX = 1.55


def reach_max_at_z(z):
    r = 0.78 - 0.467 * (z - 1.10)
    return max(r * 0.9, 0.1)


def clamp_to_reachable(target, base_pos):
    t = np.array(target, dtype=float)
    t[2] = np.clip(t[2], FOLLOW_Z_MIN, FOLLOW_Z_MAX)
    rmax = reach_max_at_z(t[2])
    dxy = t[:2] - base_pos[:2]
    dist = np.linalg.norm(dxy)
    if dist > rmax:
        t[:2] = base_pos[:2] + dxy / dist * rmax
    return t


def limit_joint_step(cur_q, next_q, max_step):
    """관절 변화량을 max_step으로 제한 (속도 제한). 관성 떨굼 방지."""
    cur_q = np.asarray(cur_q, dtype=float)
    next_q = np.asarray(next_q, dtype=float)
    delta = next_q - cur_q
    delta = np.clip(delta, -max_step, max_step)
    return cur_q + delta


EVENTS_DT = [
    0.008, 0.005, 0.02, 0.15, 0.0025,
    0.01, 0.0025, 1.0, 0.008, 0.08,
]


def which_robots_can_reach(tray_pos):
    d1 = np.linalg.norm(tray_pos[:2] - ROBOT1_BASE[:2])
    d2 = np.linalg.norm(tray_pos[:2] - ROBOT2_BASE[:2])
    can = []
    if d1 <= MAX_REACH: can.append(1)
    if d2 <= MAX_REACH: can.append(2)
    return can


# ============================================================
# 그리퍼 방향
# ============================================================
def downward_tool_orientation_yaw(yaw_rad):
    half = yaw_rad * 0.5
    return np.array([0.0, np.cos(half), np.sin(half), 0.0], dtype=np.float64)


def tool_orientation_for_tray(tray_orient_quat):
    w, x, y, z = tray_orient_quat
    tray_yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    ee_yaw = tray_yaw + np.pi / 2
    return downward_tool_orientation_yaw(ee_yaw)


TOOL_ORIENTATION = tool_orientation_for_tray(TRAY_ORIENT)


# ============================================================
# 손 추종 ROS2 구독
# ============================================================
class HandSubscriber(Node):
    def __init__(self):
        super().__init__("m0609_hand_subscriber")
        self.ee_target = None
        self.raw_pos = None
        self.hand_mode = "TRACKING"

        sq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        rq = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Point, "/hand_raw", self._raw_cb, sq)
        self.create_subscription(Point, "/hand_xyz", self._xyz_cb, sq)
        self.create_subscription(String, "/hand_mode", self._mode_cb, rq)

    def _raw_cb(self, msg): self.raw_pos = np.array([msg.x, msg.y, msg.z])
    def _xyz_cb(self, msg): self.ee_target = np.array([msg.x, msg.y, msg.z])
    def _mode_cb(self, msg): self.hand_mode = msg.data


def find_prim_path_by_name(root_path, name):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def initialize_robot(robot, world):
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    robot.gripper.open()


class SurgeryTask(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._trays = []
        self._tools = []

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._disable_cameras()
        self._discover_and_register(scene)
        self._setup_physics()
        self._create_trays(scene)
        self._create_markers(scene)
        print("\n[완료] 로봇1 흡착 + 트레이 9개(3x3) + 도구 9개!\n")

    def _disable_cameras(self):
        stage = omni.usd.get_context().get_stage()
        cnt = 0
        for robot_prim in (ROBOT1_PRIM, ROBOT2_PRIM):
            root = stage.GetPrimAtPath(robot_prim)
            if not root.IsValid():
                continue
            for prim in Usd.PrimRange(root):
                n = prim.GetName().lower()
                if any(k in n for k in ("realsense", "d455", "rsd455", "camera")):
                    prim.SetActive(False)
                    cnt += 1
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            n = prim.GetName().lower()
            if "camera_graph" in n or "renderproduct" in n:
                prim.SetActive(False)
                cnt += 1
        print(f"  [OK] 카메라/그래프 비활성화: {cnt}개")

    def _discover_and_register(self, scene):
        ee1 = find_prim_path_by_name(ROBOT1_PRIM, EE_LINK_NAME)
        if ee1 is None:
            raise RuntimeError(f"로봇1 {EE_LINK_NAME} 없음")
        self._ee1_path = ee1

        stage = omni.usd.get_context().get_stage()
        for p in suction_paths(ROBOT1_PRIM):
            if not stage.GetPrimAtPath(p).IsValid():
                raise RuntimeError(f"흡착 없음: {p}")

        gripper1 = DualSurfaceGripperAdapter(
            end_effector_prim_path=ee1,
            surface_gripper_prim_paths=suction_paths(ROBOT1_PRIM),
            write_status_to_usd=True,
        )
        self._robot1 = scene.add(
            SingleManipulator(
                prim_path=ROBOT1_PRIM, name="robot1",
                end_effector_prim_path=ee1, gripper=gripper1,
            )
        )
        print(f"  [OK] 로봇1 등록 EE={ee1}")

    def _setup_physics(self):
        stage = omni.usd.get_context().get_stage()
        cnt = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT1_PRIM)):
            for dt in ("angular", "linear"):
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    cnt += 1
        print(f"  [OK] drive: {cnt}")

    def _create_trays(self, scene):
        from pxr import UsdPhysics
        stage = omni.usd.get_context().get_stage()
        for i, pos in enumerate(TRAY_POSITIONS):
            ref = f"/World/tray_{i}"
            add_reference_to_stage(usd_path=TRAY_USD_PATH, prim_path=ref)
            for _ in range(10):
                simulation_app.update()
            tray = scene.add(
                SingleRigidPrim(
                    prim_path=f"{ref}/E_redtray_28",
                    name=f"tray_{i}",
                    position=pos,
                    orientation=TRAY_ORIENT,
                )
            )
            self._trays.append(tray)

            tool_ref = f"/World/tool_{i}"
            add_reference_to_stage(usd_path=TOOL_USDS[i], prim_path=tool_ref)
            for _ in range(5):
                simulation_app.update()
            tool_prim = stage.GetPrimAtPath(tool_ref)
            if not tool_prim.IsValid():
                print("[warn] tool", i, "invalid")
                continue

            xform = UsdGeom.Xformable(tool_prim)
            xform.ClearXformOpOrder()
            tool_pos = pos + np.array([0.0, 0.0, TOOL_DROP_HEIGHT])
            xform.AddTranslateOp().Set(Gf.Vec3d(float(tool_pos[0]), float(tool_pos[1]), float(tool_pos[2])))

            if not tool_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI.Apply(tool_prim)
            UsdPhysics.MassAPI.Apply(tool_prim)
            UsdPhysics.MassAPI(tool_prim).GetMassAttr().Set(0.001)  # 거의 0 (무게중심 영향 최소)

            for child in Usd.PrimRange(tool_prim):
                if child.GetTypeName() == "Mesh":
                    if not child.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI.Apply(child)
                    mc = UsdPhysics.MeshCollisionAPI.Apply(child)
                    mc.GetApproximationAttr().Set("convexHull")

            self._tools.append(tool_ref)

        print("[OK] trays", len(self._trays), "tools", len(self._tools))

    def _create_markers(self, scene):
        scene.add(VisualSphere(
            prim_path="/World/HandMarker", name="hand_marker",
            position=HAND_CENTER, radius=0.03,
            color=np.array([0.1, 0.3, 1.0]),
        ))

    def get_observations(self):
        tray = self._trays[TARGET_TRAY_INDEX]
        tpos, _ = tray.get_world_pose()
        return {
            "robot1": {"joint_positions": self._robot1.get_joint_positions()},
            "target_tray": {
                "position": tpos,
                "picking_position": tpos + np.array([0.0, 0.0, TRAY_TOP_Z]),
            },
        }

    def get_trays(self):
        return self._trays

    def pre_step(self, control_index, simulation_time):
        pass

    def post_reset(self):
        if hasattr(self, "_robot1"):
            self._robot1.gripper.post_reset()


def main():
    rclpy.init()
    hand_node = HandSubscriber()

    omni.usd.get_context().open_stage(FULL_SCENE_USD)
    for _ in range(80):
        simulation_app.update()
    print("[OK] full_scene stage 열기")

    print(f"[타겟] tray{TARGET_TRAY_INDEX} @ {TRAY_POSITIONS[TARGET_TRAY_INDEX]} "
          f"→ 닿는 로봇{which_robots_can_reach(TRAY_POSITIONS[TARGET_TRAY_INDEX])}")

    my_world = World(stage_units_in_meters=1.0)
    task = SurgeryTask(name="surgery_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("robot1")
    hand_marker = my_world.scene.get_object("hand_marker")
    initialize_robot(robot, my_world)

    for _ in range(30):
        my_world.step(render=True)

    # ★ 초기위치(상승된 채 시작하는 그 자세) 관절 저장 — MOVE_UP 목표
    initial_joints = robot.get_joint_positions().copy()
    print(f"[초기위치 관절 저장] {np.array(initial_joints).round(3)}")

    ee_offset = DEFAULT_EE_OFFSET.copy()

    pick_place = PickPlaceController(
        name="pp1", gripper=robot.gripper, robot_articulation=robot,
        end_effector_initial_height=TABLE_HEIGHT + 0.20,
        events_dt=EVENTS_DT,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )
    rmpflow = RMPFlowController(
        name="rmp1", robot_articulation=robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )

    print("\n[cuRobo] 초기화 중...")
    curobo = M0609CuroboController(
        robot_config_path=CUROBO_ROBOT_CONFIG,
        base_position=tuple(ROBOT1_BASE),
        base_yaw_deg=90.0,
        use_mpc=True,
    )

    # 팀원 move controller (PLACING 좌표 접근). tcp_offset=0 → link_6 직접.
    move_ctrl = M0609MoveController(
        name="move1", robot_articulation=robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
        tcp_offset_local=np.zeros(3),
        position_tolerance=PLACE_MOVE_TOL,
    )

    # ★ 트레이별 PLACING 좌표 계산 함수 (멀티 트레이)
    def place_targets_for(tray_index):
        h = TRAY_POSITIONS[tray_index]
        base = np.array([h[0], h[1], h[2] + PLACE_LINK6_ABOVE_TRAY])
        high = base + np.array([0.0, 0.0, PLACE_HIGH_OFFSET])
        down = base + np.array([0.0, 0.0, PLACE_APPROACH_GAP])
        lift = base + np.array([0.0, 0.0, 0.05 + PLACE_APPROACH_GAP])
        return base, high, down, lift

    # 현재 타겟 트레이 추적용 (task가 보는 트레이도 갱신)
    def set_target_tray(tray_index):
        task.__dict__  # noop
        # SurgeryTask.get_observations가 TARGET_TRAY_INDEX 모듈 전역을 봄 → 갱신
        globals()['TARGET_TRAY_INDEX'] = tray_index

    curobo_joint_idx = [None]
    dof_names_cache = [None]

    def get_curobo_joints():
        dof_names = list(robot.dof_names)
        if curobo_joint_idx[0] is None:
            curobo_joint_idx[0] = [dof_names.index(j) for j in curobo.joint_names]
            dof_names_cache[0] = dof_names
            print(f"[cuRobo] 관절 매핑: {curobo.joint_names} -> 인덱스 {curobo_joint_idx[0]}")
        full = robot.get_joint_positions()
        return np.array([full[i] for i in curobo_joint_idx[0]], dtype=float)

    def apply_curobo_joints(q6):
        action = ArticulationAction(
            joint_positions=np.asarray(q6, dtype=float),
            joint_indices=np.asarray(curobo_joint_idx[0]),
        )
        robot.apply_action(action)

    print(f"\n[수술실 - 멀티트레이 {TRAY_SEQUENCE}, FOLLOWING=cuRobo(속도제한), PLACING=좌표+관절안착]\n")

    was_playing = False
    state = "PICKING"
    seq_idx = 0                          # ★ TRAY_SEQUENCE 진행 인덱스
    cur_tray = TRAY_SEQUENCE[0]          # 현재 작업 트레이
    set_target_tray(cur_tray)
    pick_joints_per_tray = {}            # ★ 트레이별 흡착 관절 저장
    place_phase = "MOVE_HIGH"
    place_settle = 0
    dbg_counter = [0]

    while simulation_app.is_running():
        my_world.step(render=True)
        rclpy.spin_once(hand_node, timeout_sec=0)
        time.sleep(0.01)

        if hand_node.raw_pos is not None:
            hand_marker.set_world_pose(position=hand_node.raw_pos)

        is_playing = my_world.is_playing()

        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)
            pick_place.reset()
            rmpflow.reset()
            robot.gripper.open()
            state = "PICKING"
            place_phase = "MOVE_HIGH"
            place_settle = 0
            print("[STATE] -> PICKING")

        if not is_playing:
            was_playing = is_playing
            continue

        obs = task.get_observations()
        tray_obs = obs["target_tray"]
        current_joints = obs["robot1"]["joint_positions"]
        ee_pos, _ = robot.end_effector.get_world_pose()

        # ── PICKING (RMPFlow) ──
        if state == "PICKING":
            event = pick_place.get_current_event()
            cur_off = ee_offset.copy()
            if event in (1, 2, 3):
                cur_off[2] -= 0.042
            actions = pick_place.forward(
                picking_position=tray_obs["picking_position"],
                placing_position=tray_obs["picking_position"],
                current_joint_positions=current_joints,
                end_effector_offset=cur_off,
                end_effector_orientation=TOOL_ORIENTATION,
            )
            robot.apply_action(actions)
            if event >= 4:
                # ★ 현재 트레이의 흡착 관절 저장 (트레이별)
                if cur_tray not in pick_joints_per_tray:
                    pick_joints_per_tray[cur_tray] = robot.get_joint_positions().copy()
                    print(f"[안착관절 저장] tray{cur_tray} = {np.array(pick_joints_per_tray[cur_tray]).round(3)}")
                state = "FOLLOWING"
                print(f"[STATE] PICKING(tray{cur_tray})->FOLLOWING (cuRobo)")

        # ── FOLLOWING (cuRobo MPC + clamp + 속도제한) ──
        elif state == "FOLLOWING":
            target = hand_node.ee_target if hand_node.ee_target is not None else ee_pos
            target = clamp_to_reachable(target, ROBOT1_BASE)

            cur_q = get_curobo_joints()
            curobo.mpc_set_goal(target, TOOL_ORIENTATION, cur_q)
            next_q = curobo.mpc_step(cur_q)
            # ★ 속도 제한 (관성 떨굼 방지) — 한 프레임 관절변화 제한
            next_q = limit_joint_step(cur_q, next_q, MAX_JOINT_STEP)
            apply_curobo_joints(next_q)

            dbg_counter[0] += 1
            if dbg_counter[0] % 30 == 0:
                ee_world, _ = robot.end_effector.get_world_pose()
                print(f"[CHECK] 손목표(clamp)={np.array(target).round(3)}  "
                      f"Isaac_EE={np.array(ee_world).round(3)}")

            if hand_node.hand_mode == "HOME":
                state = "PLACING"
                place_phase = "MOVE_HIGH"
                p_base, p_high, p_down, p_lift = place_targets_for(cur_tray)
                move_ctrl.reset()
                move_ctrl.set_target(position=p_high, orientation=TOOL_ORIENTATION)
                print(f"[STATE] FOLLOWING->PLACING(tray{cur_tray})  MOVE_HIGH={p_high.round(3)}")

        # ── PLACING: 좌표접근(HIGH→DOWN) → 관절정밀안착 → open → LIFT → MOVE_UP ──
        elif state == "PLACING":
            p_base, p_high, p_down, p_lift = place_targets_for(cur_tray)
            if place_phase == "MOVE_HIGH":
                robot.apply_action(move_ctrl.forward())
                dbg_counter[0] += 1
                if dbg_counter[0] % 30 == 0:
                    print(f"[MOVE_HIGH] 오차={move_ctrl.get_position_error():.4f}")
                if move_ctrl.is_done():
                    move_ctrl.reset()
                    move_ctrl.set_target(position=p_down, orientation=TOOL_ORIENTATION)
                    place_phase = "MOVE_DOWN"
                    print(f"[PLACING] MOVE_HIGH 완료 -> MOVE_DOWN={p_down.round(3)}")

            elif place_phase == "MOVE_DOWN":
                robot.apply_action(move_ctrl.forward())
                dbg_counter[0] += 1
                if dbg_counter[0] % 30 == 0:
                    print(f"[MOVE_DOWN] 오차={move_ctrl.get_position_error():.4f}")
                if move_ctrl.is_done():
                    move_ctrl.reset()
                    place_phase = "JOINT_PLACE"
                    print("[PLACING] MOVE_DOWN 완료 -> JOINT_PLACE (트레이별 관절 안착)")

            elif place_phase == "JOINT_PLACE":
                # ★ 현재 트레이의 흡착 관절로 정밀 안착
                tgt = pick_joints_per_tray.get(cur_tray)
                if tgt is None:
                    place_phase = "RELEASE"   # 안전
                else:
                    cur = robot.get_joint_positions()
                    stepped = limit_joint_step(cur, tgt, PLACE_JOINT_STEP)
                    robot.apply_action(ArticulationAction(joint_positions=stepped))
                    err = float(np.linalg.norm(np.asarray(cur) - tgt))
                    dbg_counter[0] += 1
                    if dbg_counter[0] % 30 == 0:
                        print(f"[JOINT_PLACE] 관절오차={err:.4f}")
                    if err < PLACE_JOINT_TOL:
                        place_settle += 1
                        if place_settle >= PLACE_SETTLE:
                            place_phase = "RELEASE"
                            place_settle = 0
                            print("[PLACING] 정밀안착 완료 -> RELEASE")
                    else:
                        place_settle = 0

            elif place_phase == "RELEASE":
                robot.gripper.open()
                # 놓은 자리에서 위로 5cm 먼저 빠지기
                move_ctrl.reset()
                move_ctrl.set_target(position=p_lift, orientation=TOOL_ORIENTATION)
                place_phase = "LIFT"
                place_settle = 0
                print(f"[PLACING] 놓음(open) -> LIFT (위 5cm)={p_lift.round(3)}")

            elif place_phase == "LIFT":
                robot.apply_action(move_ctrl.forward())
                dbg_counter[0] += 1
                if dbg_counter[0] % 30 == 0:
                    print(f"[LIFT] 오차={move_ctrl.get_position_error():.4f}")
                if move_ctrl.is_done():
                    move_ctrl.reset()
                    place_phase = "MOVE_UP"
                    print("[PLACING] LIFT 완료 -> MOVE_UP (초기위치로)")

            elif place_phase == "MOVE_UP":
                cur = robot.get_joint_positions()
                stepped = limit_joint_step(cur, initial_joints, PLACE_JOINT_STEP)
                robot.apply_action(ArticulationAction(joint_positions=stepped))
                err = float(np.linalg.norm(np.asarray(cur) - initial_joints))
                dbg_counter[0] += 1
                if dbg_counter[0] % 30 == 0:
                    print(f"[MOVE_UP] 초기위치 오차={err:.4f}")
                if err < 0.05:
                    # ★ 다음 트레이로 진행
                    seq_idx += 1
                    if seq_idx < len(TRAY_SEQUENCE):
                        cur_tray = TRAY_SEQUENCE[seq_idx]
                        set_target_tray(cur_tray)
                        print(f"[STATE] PLACING 완료 -> IDLE  (다음 트레이 {cur_tray})")
                    else:
                        print(f"[STATE] PLACING 완료 -> IDLE  (시퀀스 끝, 모든 트레이 완료!)")
                    state = "IDLE"

        # ── IDLE (초기위치 유지) ──
        elif state == "IDLE":
            # 초기위치 관절 유지 (RMPFlow 대신 관절 직접 — 안정)
            cur = robot.get_joint_positions()
            stepped = limit_joint_step(cur, initial_joints, PLACE_JOINT_STEP)
            robot.apply_action(ArticulationAction(joint_positions=stepped))
            if hand_node.hand_mode == "TRACKING":
                pick_place.reset()
                state = "PICKING"
                print("[STATE] IDLE->PICKING")

        was_playing = is_playing

    simulation_app.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()