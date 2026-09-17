# ERP42 포트폴리오 요소 추출 계획 (Codex 독립 스캔)

## 1. 목적과 판정 원칙

이 문서는 `src/`(개인 작업본)와 `erp42_main/src/`(팀 정리본)의 소스 코드와 ROS interface/configuration을 직접 스캔해, 취업·이력서용 포트폴리오 소재를 추출하기 위한 계획이다.

- **확인됨(정적 근거)**: 이번 세션의 `grep`/`read`로 실제 코드와 `파일:라인`을 확인한 사실이다.
- **미검증(실행)**: 코드가 있어도 build, launch, rosbag replay, 차량 실기로 동작을 확인하지 않은 상태다.
- **사용자 확인 필요**: 코드로 알 수 없는 대회 결과, 팀 역할, 실제 사용 버전, 본인 기여 범위다.
- 성능 수치(정확도, FPS, 성공률, 오차, 완주 시간)는 로그나 측정 원본이 없으면 쓰지 않는다.
- 외부 패키지가 포함된 사실과 본인이 구현·수정한 사실을 구분한다. upstream package 자체를 본인 구현으로 주장하지 않는다.
- 최종 문구는 `문제/미션 → 내가 한 설계·구현 → 검증 방법 → 확인된 결과 → 한계` 순서로 만든다.

### 이번 독립 조사 범위

- 읽음: `src/`, `erp42_main/src/` 아래 코드, interface, package/launch/configuration
- 읽지 않음: `docs/portfolio/*`, `docs/plans/2026-08-20-portfolio-extraction.md`, `docs/state.md`
- 실행하지 않음: `colcon build`, ROS node/launch, rosbag replay, simulation, ERP42 실차 구동

따라서 아래의 구현 내용은 모두 **정적 코드 확인**이며, 동작·성능은 별도 표기가 없는 한 **미검증**이다.

## 2. 소스 트리 역할과 증거 우선순위

### 2.1 두 트리의 활용 방식

1. `src/`: 날짜별 실험 코드, LiDAR clustering/avoidance, EBIMU, localization package까지 포함된 개인 작업본으로 취급한다.
2. `erp42_main/src/`: ERP driver, localization, lane/YOLO, GNSS/NTRIP, camera/IMU package가 정리된 팀 작업본으로 취급한다.
3. 같은 기능이 양쪽에 있으면 `diff`로 동일성·변경점을 확인한 뒤, 실제 대회에 사용한 파일을 사용자에게 확인한다.
4. `ublox`, `usb_cam`, `vectornav`, `robot_localization`, `velodyne`, `ntrip_client`, `yolo_ros` 등 외부 프로젝트로 보이는 패키지는 license/maintainer/upstream 이력을 확인하고 **도입·통합·설정 경험**과 **직접 구현**을 분리한다.

### 2.2 코드상 확인된 경계

- `src/erp_driver/scripts/`에만 날짜별 LiDAR 및 controller 실험 파일이 다수 존재한다. 예: `0702_erp42_controller.py`, `0724_erp42_ObstacleAvoidance.py`, `0822_Lam_ObtAvo.py`, `erp42_pathtracking_lidar_integrated.py` (**확인됨**).
- `erp42_main/src/erp_driver/scripts/`에는 serial, EKF, waypoint publisher, path tracking, lane detection 계열이 정리되어 있다 (**확인됨**).
- `erp42_main/src/yolo_ros/yolo_ros/setup.py:15-18`의 maintainer/license와 `yolo_node.py:1-14`의 copyright를 보면 YOLO ROS core는 외부 코드로 보인다. 본인 주장은 custom model·topic integration·mission logic에 한정해야 한다 (**확인됨(정적 근거), 실제 수정 범위는 사용자 확인 필요**).

## 3. 1차 미션-코드 매핑

| 미션/기능 후보 | 정적 코드 근거 | 현재 판정 | 포트폴리오 추출 방향 |
|---|---|---|---|
| GNSS waypoint 기반 주행 | `erp42_main/src/erp_driver/scripts/erp42_pubwaypointscnuservice_pymap3d.py:19-21,41-63,76-82`; `erp42_pathtracking.py:35-56,84-120,128-190` | 코드 경로 확인됨, 실행 미검증 | geodetic→ENU, `map` frame Path 생성, nearest/lookahead waypoint, heading-error steering을 한 pipeline으로 설명 |
| GNSS/IMU/wheel localization | `erp42_main/src/erp_driver/scripts/erp42_imu-gps-wheel-ekf_globalposition.py:41-74,101-172,190-230` | 선형화된 3-state EKF 형태 코드 확인됨, 정확도·GPS 음영 통과 미검증 | command prediction, encoder/GNSS correction, IMU heading offset, adaptive process noise의 계산식과 data flow 추출 |
| Camera lane following | `erp42_main/src/erp_driver/scripts/erp42_lanedetect.py:53-84,247-301,324-376` | HSV/LDA/Canny/Hough/temporal smoothing 코드 확인됨, 주행 성능 미검증 | illumination 대응 전처리, histogram 기반 후보 제한, polynomial fit, missing-lane hold, P-control을 단계별로 추출 |
| YOLO lane 후보 | `erp42_main/src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py:49-72,114-154,327-385`; `erp42_main/src/erp_driver/scripts/erp42_lanedetect_yolo.py:17-88` | framework와 adapter 코드 존재, 현재 adapter 정적 오류 및 model 배선 미검증 | 외부 YOLO ROS 도입과 custom lane adapter를 분리. 실제 사용 model, 학습 데이터, 사용자 수정분 확인 후에만 채택 |
| LiDAR ground/wall 제거 및 clustering | `src/cluster_bev/src/cluster_bev_node.cpp:25-49,68-157,160-198`; `src/pcl_clustering_py/pcl_clustering_py/euclidean_cluster_node.py:15-89` | C++ pipeline 확인됨. Python 버전은 정적 결함 존재. build/run 미검증 | VoxelGrid→RANSAC plane removal→PassThrough→Euclidean clustering→BEV projection의 설계 근거 추출 |
| LiDAR cone/track following | `src/erp_driver/scripts/0822_Lam_ObtAvo.py:29-89,140-241` | DBSCAN, centroid pairing, centerline, Pure Pursuit 코드 확인됨, 실기 미검증 | point filtering→DBSCAN→좌/우 cluster→midpoint centerline→curvature speed policy→Pure Pursuit로 정리 |
| 정적 장애물 회피 + path tracking 통합 | `src/erp_driver/scripts/erp42_pathtracking_lidar_integrated.py:45-66,76-188,223-269,362-474,477-518` | FSM/sector-clearance/hysteresis 코드 확인됨. message import와 launch 배선은 미검증 | `PATH_FOLLOW / AVOID_TURN / AVOID_COOLDOWN` 상태전이, stale scan 검사, 양측 막힘 정지, 복귀 cooldown을 안전·상태관리 사례로 추출 |
| command arbitration/fail-safe | `src/erp_driver/scripts/0702_erp42_controller.py:8-58` | lane/path timeout과 fallback brake 코드 확인됨, producer contract 불일치 가능 | stale command 0.2 s timeout, lane 우선순위, no-valid-command full brake를 architecture 소재로 검증 |
| ERP42 serial protocol | `erp42_main/src/erp_driver/scripts/erp42_serial.py:9-27,30-74`; `erp42_main/src/erp_interfaces/msg/ErpCmdMsg.msg:1-5`; `ErpStatusMsg.msg:1-8` | 40 Hz packet I/O 및 custom msg 확인됨, hardware 통신 미검증 | ROS command/status ↔ 18-byte packet, alive counter, encoder feedback을 통신·embedded integration 소재로 추출 |
| RTK/NTRIP 연동 | `erp42_main/src/ntrip_client/scripts/ntrip_ros.py:15-88`; `erp42_main/src/ntrip_client/src/ntrip_client/ntrip_client.py:25-65,67-137,189-238` | 외부 client package 포함 및 parameterized connection 코드 확인됨, caster/RTK fix 미검증 | 직접 구현 주장이 아니라 u-blox RTK correction pipeline 도입·설정·운영 증거로 제한 |
| 배달·교차로·신호등·주차 mission logic | 두 트리 전체 keyword scan에서 의미 있는 구현이 발견되지 않음. `erp42_pathtracking_lidar_integrated.py:2-6`도 원 파일명과 달리 신호등 로직이 없다고 명시 | **미검증/현재 코드 근거 없음** | 별도 branch, 유실 파일, launch, 대회 영상·로그가 제공되기 전에는 구현 주장 제외 |

## 4. 기술 스택 추출 항목

### 4.1 직접 코드 근거가 있는 stack

- ROS 2 Python/C++ node: `rclpy`, `rclcpp`, publisher/subscriber/service/timer, custom interface (**확인됨**).
- Data/interface: `sensor_msgs/{Image,PointCloud2,LaserScan,NavSatFix,Imu}`, `nav_msgs/{Path,Odometry}`, `erp_interfaces/{ErpCmdMsg,ErpStatusMsg,SetOrigin}` (**확인됨**).
- Localization/math: NumPy matrix EKF, bicycle kinematics, quaternion/yaw, `pymap3d.geodetic2enu` (**확인됨**).
- Perception: OpenCV/cv_bridge, HSV/Canny/Hough/polynomial fitting, scikit-learn LDA/DBSCAN, PCL VoxelGrid/RANSAC/KdTree/Euclidean clustering (**확인됨**).
- Control: lookahead waypoint heading control, Pure Pursuit 형태 steering, curvature-dependent speed, smoothing, command arbitration/FSM (**확인됨(코드 형태), 실차 성능 미검증**).
- Hardware/data link: pyserial/`struct` ERP42 packet, u-blox/NTRIP/RTCM, USB camera, VectorNav/EBIMU, Velodyne package inclusion (**확인됨(포함·호출), 실제 장착/사용은 사용자 확인 필요**).
- ML integration: Ultralytics/PyTorch 기반 lifecycle node 및 YOLO detection message pipeline (**확인됨(외부 package 포함), 실제 대회 사용 model·학습은 미검증**).

### 4.2 stack별 최종 산출물

각 stack은 다음 5개 필드로 카드화한다.

1. 해결한 문제와 입력 조건
2. 계산/algorithm pipeline
3. ROS topic·service·frame·unit contract
4. 본인 기여 코드와 외부 dependency 경계
5. 검증 증거와 남은 한계

## 5. 아키텍처 추출 계획

### 5.1 우선 복원할 data flow

```text
u-blox NavSatFix ----┐
VectorNav/IMU -------+--> EKF/global localization --> /odom_ekf --┐
ERP42 status/encoder -┘                                           |
                                                                  +--> waypoint tracker --> command
Excel GNSS waypoint --> SetOrigin + geodetic2enu --> Path --------┘

USB camera --> lane perception ------------------> lane command --┐
LiDAR --> clustering / avoidance / track follow -> lidar/path cmd -+--> controller 후보 --> /erp42_ctrl_cmd
                                                                  |
/erp42_ctrl_cmd --> serial packet --> ERP42 --> status/encoder ----┘
```

이 그림은 **후보 아키텍처**다. topic 이름과 producer/consumer가 실제로 맞물리는지는 다음 단계에서 모두 대조한다.

### 5.2 반드시 검증할 interface contract

- `/waypoints_path1` 대 `/waypoints_path`, `/odom_ekf` 대 `/odometry/filtered/global`처럼 버전별 topic 차이를 표로 만든다.
- `/erp42_ctrl_cmd`, `/erp42_ctrl_cmd/path`, `/erp42_ctrl_cmd/lane`, `/erp42_ctrl_cmd/lidar`의 publisher/consumer를 전수 매핑한다.
- `brake` 값을 단순 제동량과 source-selector flag로 동시에 사용하는지 확인한다.
- `map`, `base_link`, LiDAR frame의 transform 존재 여부와 좌표축·단위·steering sign을 확인한다.
- sensor stream QoS와 command QoS가 publisher/subscriber 사이에서 호환되는지 확인한다.
- launch/CMake/setup entry point에 실제 node가 설치·배선되는지 확인한다.

## 6. 주장별 검증 계획

### 단계 A — 정적 provenance와 소유권

1. 두 트리의 동일 파일을 `diff --no-index`로 비교한다.
2. package metadata, copyright, maintainer, `.git` 유무로 upstream code를 분리한다.
3. 날짜별 실험본 → 팀 정리본의 계보를 사용자와 함께 확정한다.
4. 각 주장에 최소 1개의 핵심 `파일:라인`과 caller/consumer `파일:라인`을 붙인다.

**통과 기준:** “내가 구현”과 “팀/외부 package를 도입·통합”이 섞이지 않는다.

### 단계 B — static integrity

1. Python은 `py_compile` 또는 import 가능한 범위에서 syntax/name 오류를 확인한다.
2. C++/Python package의 `package.xml`, CMake/setup, entry point, launch wiring을 대조한다.
3. topic, message type, frame, unit, QoS, timer rate, stale-data handling을 producer/consumer 쌍으로 확인한다.
4. dead branch와 실행 불가능한 prototype은 portfolio main flow에서 제외한다.

현재 발견되어 우선 확인할 risk:

- `erp42_pathtracking_lidar_integrated.py:37-40`은 실제 `erp_interfaces.msg.ErpCmdMsg`와 다른 import/type 이름을 사용한다.
- `erp42_lanedetect_yolo.py:8-9,20-22`는 `DetectionArray`를 import하지만 subscription type으로 정의되지 않은 `Detection2DArray`를 사용한다.
- `pcl_clustering_py/euclidean_cluster_node.py:95-101,108-118,125-128`은 list 대신 문자열을 초기화한 뒤 `append`하고, PointCloud field 구성도 비어 있어 실행 가능 주장을 보류한다.
- `0702_erp42_controller.py:39-40`은 `brake==3/2`를 source-valid flag로 보지만, 팀 lane publisher `erp42_lanedetect.py:373-376`은 `steer`만 채워 default brake를 발행한다.
- `erp_driver/CMakeLists.txt:14-20`은 serial 계열 3개만 설치한다. EKF/path/lane/controller/LiDAR node의 실행 entry point와 launch wiring은 확인되지 않았다.
- `best_lane_yolov11.pt` 파일은 존재하지만, code/launch keyword scan에서 이 model을 선택하는 명시적 wiring은 발견되지 않았다.

**통과 기준:** 정적 오류가 있는 코드는 “prototype/실험”으로만 표시하고 완성 시스템 증거로 쓰지 않는다.

### 단계 C — deterministic software validation

실행 전에 package별로 범위를 나눠 검증한다.

1. `colcon list`로 실제 package 인식을 확인한다.
2. custom package(`erp_interfaces`, `erp_driver`, `cluster_bev`, `pcl_clustering_py`, 필요한 localization package)를 package-select build한다.
3. pure logic은 작은 fixture로 검증한다.
   - ERP packet pack/unpack round trip
   - geodetic→ENU 기준점/축 방향
   - EKF predict/update matrix shape와 covariance 유한성
   - waypoint nearest/lookahead와 steering saturation
   - sector quantile, FSM transition, stale scan, both-sides-blocked stop
   - cone centroid pairing과 Pure Pursuit sign
4. launch description과 executable 존재를 `ros2 pkg executables`, `ros2 launch ... --show-args` 등으로 확인한다.

**현재 상태:** 전부 **미검증**. 이 문서 작성 세션에서는 실행하지 않았다.

### 단계 D — data/replay validation

1. 실제 사용 rosbag과 waypoint 파일을 사용자에게 받아 topic/type/frame을 inventory한다.
2. rosbag replay로 perception/localization/control node를 vehicle output과 분리해 실행한다.
3. 결과를 숫자로 쓸 경우 측정 정의를 먼저 고정한다.
   - localization: 기준 trajectory와 lateral/longitudinal error
   - lane: annotated frame 기준 detection/center offset
   - obstacle: detection range, false stop, minimum clearance
   - control: cross-track error, heading error, command latency/staleness
4. 동일 bag·config·commit으로 재현되는 표와 plot만 포트폴리오에 넣는다.

**현재 상태:** rosbag/log/ground truth를 확인하지 않아 **미검증**.

### 단계 E — simulation 및 실차 Human Gate

- Simulation 검증과 실차 검증을 별도로 기록한다.
- steering sign, brake semantics, serial port, wheelbase/encoder scale, sensor extrinsic, `map→base_link`를 먼저 확인한다.
- ERP42 actuation은 사용자 승인과 현장 안전 인원·E-stop 확인 후에만 수행한다.
- 실차 결과에는 코스, 날씨/조도, GNSS 상태, speed limit, 실패·recovery 사례를 함께 남긴다.

**현재 상태:** simulation/실차 모두 **미검증**.

## 7. 최종 포트폴리오에서 추출할 산출물

1. **프로젝트 한 줄 요약**: 차량·대회·미션·본인 역할을 과장 없이 1문장으로 정리
2. **미션 카드**: waypoint 주행, localization, lane, LiDAR perception/avoidance, vehicle interface별 문제-설계-검증-결과
3. **시스템 아키텍처**: sensor→localization/perception→control arbitration→serial I/O의 topic graph
4. **핵심 algorithm 카드**:
   - adaptive-Q 3-state EKF
   - geodetic→ENU waypoint pipeline
   - camera lane pipeline와 temporal smoothing
   - PCL filtering/clustering/BEV
   - DBSCAN cone centerline + Pure Pursuit
   - avoidance FSM + stale input/fail-safe
5. **Engineering decision 기록**: Python prototype 대 C++ PCL, 분리형 controller 대 통합 node, GNSS/IMU heading 보정 등의 선택과 trade-off
6. **검증 표**: static/build/unit/replay/simulation/hardware를 분리한 evidence matrix
7. **Contribution boundary**: 본인/팀원/upstream code와 실제 대회 사용 여부를 명시
8. **한계와 개선**: 미배선 node, hard-coded path/parameter, interface mismatch, test 부재를 다음 개선으로 정리

## 8. 사용자 확인이 필요한 항목

### 최우선: 결과와 실제 사용본

- 대회명·연도·예선/본선 진출·수상/순위·공식 완주 결과는 무엇인가?
- 실제 대회 차량에서 실행한 branch/file/launch 조합은 무엇인가? `src/`와 `erp42_main/src/` 중 어느 쪽이 deployment source였는가?
- 실제 주행 rosbag, terminal log, 영상, waypoint 파일, 측정 표가 남아 있는가?

### 담당 범위와 contribution

- 본인이 직접 설계·작성·debug한 파일은 무엇이며, 팀원 작성 또는 외부에서 가져온 파일은 무엇인가?
- EKF, lane/YOLO, LiDAR, controller, serial, NTRIP 중 본인 책임 범위와 팀원 책임 범위는 어디까지인가?
- 외부 package에서 직접 수정한 부분이 있다면 upstream 대비 diff나 commit이 있는가?

### 시스템·센서 사실

- 실차 센서 구성과 최종 사용 세대(카메라, LiDAR, GNSS, VectorNav/EBIMU)는 무엇인가?
- LiDAR의 static obstacle/cone track code가 어떤 미션에서 실제 사용되었는가?
- GPS 음영구간은 어떤 sensor fusion/fallback으로 통과했으며, 이를 입증할 log/영상이 있는가?
- YOLO model의 class, dataset 규모/출처, 학습 담당, 실제 사용 weight는 무엇인가?

### 수치와 문제 해결 사례

- 재현 가능한 성능 수치의 원본(측정 방법·횟수·조건 포함)이 있는가?
- 대표적인 현장 실패 1~3개와 원인, 변경 전후 evidence가 있는가?
- 심사위원/팀 feedback 또는 공식 기술보고서 중 공개 가능한 근거가 있는가?

## 9. 완료 조건

포트폴리오 요소 추출은 다음이 모두 충족될 때 완료로 본다.

- 모든 기술 주장에 code anchor와 contribution owner가 있다.
- mission별 실제 사용 여부가 `사용/실험/미사용/모름`으로 구분된다.
- build, unit, replay, simulation, hardware evidence가 서로 섞이지 않는다.
- 성능 수치는 원본 evidence와 측정 조건이 있을 때만 포함한다.
- upstream 도입과 직접 구현이 구분된다.
- 배달·교차로·신호등·주차처럼 현재 코드 근거가 없는 항목은 추가 증거 전까지 제외한다.
- 이력서용 bullet, 면접용 STAR, 상세 portfolio architecture가 같은 사실 집합을 사용한다.

## 10. 다음 실행 순서

1. 사용자에게 실제 대회 사용본·담당 범위·결과를 확인한다.
2. 두 트리 custom code의 provenance/diff 표를 만든다.
3. topic/service/frame/unit/entry point 기반 실제 architecture를 복원한다.
4. 정적 결함을 분류해 portfolio 채택 후보를 확정한다.
5. package build와 pure-logic test를 수행한다.
6. rosbag/log/영상이 있으면 replay와 정량 검증을 수행한다.
7. 검증된 주장만 mission card, architecture, STAR, 이력서 bullet로 변환한다.
