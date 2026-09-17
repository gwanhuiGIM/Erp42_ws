# ERP42 포트폴리오 교차관심사 맵 (PORTFOLIO_MAP)

> 이 문서는 `src/`(개인 작업본)와 `erp42_main/src/`(팀 정리본) 두 워크스페이스를 병합한 소스 전수 스캔 중간 산출물이다. `PORTFOLIO_EXTRACTION_GUIDE.md`의 (B0) 단계 산출물이며, `PORTFOLIO_SOURCE.md`·`SKILL_INVENTORY.md`가 이 문서에서 파생된다.
>
> **작성 경위**: 두 ws를 한 번에 스캔하는 단일 codex exec 작업이 22분간 로그 정지(정지 의심)를 일으켜, `src/`용과 `erp42_main/src/`용으로 나눠 병렬 재실행한 뒤 이 문서로 병합했다. 각 절의 원본은 `docs/portfolio/PORTFOLIO_MAP_src.md`, `docs/portfolio/PORTFOLIO_MAP_erp42_main.md`에 그대로 보존돼 있다(병합 시 내용 변경 없음, 배치만 통합).
>
> 판정은 **정적 source/config 확인** 기준이다. 파일 존재는 한 번의 대회 run에 통합·실행됐다는 증거가 아니며, 확인할 수 없는 사항은 `[확인 필요]`로 남긴다. 참고 문서: `docs/portfolio/mission-cards.md`, `docs/portfolio/tech-stack.md`, `docs/portfolio/architecture.md`.

---

## 1. 버전 고정 헤더

| 항목 | `src/`(개인 작업본) | `erp42_main/src/`(팀 정리본) |
|---|---|---|
| 스캔 경로 | `/home/kimkh/colcon_ws/src` | `/home/kimkh/colcon_ws/erp42_main/src` |
| 저장소 상태 | git repo 아님(HEAD 없음) | git repo 아님(HEAD 없음) |
| 최신 Python mtime | `1782262910` — `src/usb_cam/launch/Pick_Tray_SA.py` | `1754633350` (2025-08-08 15:09:10) — `erp42_main/src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py` |

**하드웨어 토폴로지(공통, 사용자 제공)**: 카메라 설정 4대(camera1~4) 중 실배선 1~2대, Velodyne LiDAR(src만), GPS+RTK(NTRIP/u-blox), IMU(vectornav→ebimu 세대교체 — vectornav는 양쪽에, ebimu는 src에만), 휠 인코더, ERP42 차량.

- `src/`: `camera1`+`params_1.yaml`(`src/usb_cam/launch/camera.launch.py:52-53`), `camera2`+`params_2.yaml`(`src/usb_cam/launch/2camera.launch.py:52-53`).
- `erp42_main/src/`: `params_1.yaml`~`params_4.yaml` 네 개 존재하지만 launch는 camera1/camera2만 참조. 어느 physical camera가 어느 미션을 담당했는지는 소스로 복원 불가 → `[확인 필요]`.

---

## 2. 전 패키지 LOC 체크리스트

### 2-A. `src/` (17개 최상위 디렉터리)

| 최상위 디렉터리 | `.py` 파일 수 | Python LOC |
|---|---:|---:|
| `cluster_bev` | 0 | 0 |
| `ebimu_pkg` | 7 | 219 |
| `erp_driver` | 30 | 4,131 |
| `erp_interfaces` | 3 | 26 |
| `fast_gicp` | 2 | 176 |
| `hdl_global_localization` | 0 | 0 |
| `hdl_localization` | 3 | 197 |
| `ndt_omp` | 0 | 0 |
| `ntrip_client` | 12 | 1,156 |
| `pcl_clustering_py` | 6 | 241 |
| `pcl_ros` | 5 | 255 |
| `robot_localization` | 16 | 975 |
| `ublox` | 2 | 128 |
| `usb_cam` | 6 | 1,053 |
| `vectornav` | 2 | 73 |
| `velodyne` | 22 | 1,674 |
| `Yolo_pt` | 0 | 0 |
| **합계** | **116** | **10,304** |

### 2-B. `erp42_main/src/` (13개 최상위 디렉터리 + src root 도구 1개)

| 최상위 디렉터리/범위 | Python 파일 수 | Python LOC |
|---|---:|---:|
| `erp_driver` | 16 | 2,239 |
| `erp_interfaces` | 3 | 26 |
| `ntrip_client` | 12 | 1,156 |
| `rosbag_convert_clean_py` | 1 | 45 |
| `ublox` | 0 | 0 |
| `ublox_gps` | 2 | 128 |
| `ublox_msgs` | 0 | 0 |
| `ublox_serialization` | 0 | 0 |
| `usb_cam` | 5 | 322 |
| `vectornav` | 2 | 73 |
| `vectornav_msgs` | 0 | 0 |
| `waypoint` | 0 | 0 |
| `yolo_ros` | 17 | 2,272 |
| `[src root]`(`rosbag2csv.py`) | 1 | 116 |
| **합계** | **59** | **6,377** |

`best_lane_yolov11.pt`도 `[src root]`에 있으나 Python LOC에는 미포함.

**두 ws 합산: 30개 최상위 디렉터리, 175개 `.py` 파일, 16,681 LOC.**

---

## 3. 파일 × 11축 교차관심사 매트릭스

범례: `●` 중심, `○` 접점, 빈칸/`—`은 해당없음. `[시뮬]`은 venv/conda/ABI bridging/Docker 격리를 포함.

### 3-A. `src/`

| 패키지 | 주요 파일 | 시뮬 | 비전 | 좌표계 | 통신 | 노드설계 | 모션플래닝 | 상태관리 | 안전 | 인프라 | 데이터 | HRI |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| `cluster_bev` | `src/cluster_bev/src/cluster_bev_node.cpp` |  |  |  | ● |  |  |  |  | ○ | ○ |  |
| `ebimu_pkg` | `src/ebimu_pkg/ebimu_pkg/ebimu_publisher.py` |  |  | ○ | ● |  |  |  |  |  | ○ |  |
| `erp_driver` | `src/erp_driver/scripts/1024_EBIMU_EKF.py` |  |  | ● | ○ |  |  | ○ |  |  | ○ |  |
| `erp_driver` | `src/erp_driver/scripts/erp42_pathtracking.py` |  |  | ○ | ○ |  | ● | ○ | ● |  |  |  |
| `erp_driver` | `src/erp_driver/scripts/0702_erp42_controller.py` |  |  |  | ○ |  |  | ● | ● |  |  |  |
| `erp_driver` | `src/erp_driver/scripts/0702_erp42_lanedetect.py` |  | ● |  | ○ |  | ○ |  |  |  |  |  |
| `erp_driver` | `src/erp_driver/scripts/0822_Obstacle_3d.py` |  |  |  | ○ |  | ○ |  | ● |  | ○ |  |
| `erp_driver` | `src/erp_driver/scripts/erp42_serial.py` |  |  |  | ● | ○ |  | ○ | ○ |  |  |  |
| `erp_driver` | `src/erp_driver/scripts/erp42_pubwaypointscnuservice_pymap3d.py` |  |  | ○ | ○ |  | ○ |  |  |  | ● |  |
| `erp_interfaces` | `src/erp_interfaces/msg/ErpCmdMsg.msg` |  |  |  | ● |  |  | ○ | ○ |  |  |  |
| `fast_gicp` | `src/fast_gicp/CMakeLists.txt` | ● |  |  |  | ● |  |  |  | ○ |  |  |
| `hdl_global_localization` | `src/hdl_global_localization/docker/noetic/Dockerfile` | ● |  | ○ |  |  |  |  |  | ○ |  |  |
| `hdl_localization` | `src/hdl_localization/launch/hdl_localization.launch.py` |  |  | ○ | ○ |  |  |  |  | ● |  |  |
| `ndt_omp` | `src/ndt_omp/docker/foxy/Dockerfile` | ● |  | ○ |  |  |  |  |  | ○ |  |  |
| `ntrip_client` | `src/ntrip_client/scripts/ntrip_ros_base.py` |  |  | ○ | ● |  |  |  |  | ○ | ○ |  |
| `pcl_clustering_py` | `src/pcl_clustering_py/pcl_clustering_py/euclidean_cluster_node.py` |  |  |  | ● |  |  |  |  | ○ | ○ |  |
| `pcl_ros` | `src/pcl_ros/src/transforms.cpp` |  |  | ● | ○ |  |  |  |  |  |  |  |
| `robot_localization` | `src/robot_localization/launch/ekf.launch.py` |  |  | ○ | ○ |  |  |  |  | ● |  |  |
| `robot_localization` | `src/robot_localization/test/test_ekf_localization_node_bag1.launch.py` |  |  | ○ |  |  |  |  |  | ○ | ● |  |
| `ublox` | `src/ublox/ublox_gps/launch/ublox_gps_node-launch.py` |  |  | ○ | ○ |  |  |  |  | ● |  |  |
| `usb_cam` | `src/usb_cam/launch/camera_config.py` |  | ○ |  | ○ |  |  |  |  | ● |  |  |
| `usb_cam` | `src/usb_cam/launch/Pick_Tray_SA.py`(**주의: ERP42와 무관한 별개 Isaac Sim 앱 — §3.1 참고**) | ● | ○ | ○ | ○ |  | ○ | ○ | ○ |  |  | ● |
| `vectornav` | `src/vectornav/vectornav/launch/vectornav.launch.py` |  |  | ○ | ○ |  |  |  |  | ● |  |  |
| `velodyne` | `src/velodyne/velodyne/launch/velodyne-all-nodes-VLP16-launch.py` |  |  | ○ | ○ |  |  |  |  | ● | ○ |  |
| `Yolo_pt` | `src/Yolo_pt/best.pt` |  | ○ |  |  |  |  |  |  |  | ○ |  |

**3-A.1 축별 정직한 공백 해석**
- HRI는 ERP42 주행 stack에서 확인되지 않음. 유일한 접점은 별도 Isaac Sim 앱 `Pick_Tray_SA.py`의 `/hand_raw`, `/hand_xyz`, `/hand_mode` — **이 프로젝트(ERP42 대회)와 무관한 다른 작업의 파일로 판단**, 포트폴리오 성과에 포함하지 않는다.
- `Executor`/`CallbackGroup`/Python `Lock`은 선택 ERP42 노드에서 미확인. 노드설계의 중심 근거는 `fast_gicp` thread 설정과 `erp42_serial.py`의 40 Hz timer뿐.
- `venv`/`conda`는 주요 경로에서 미확인. 격리·ABI 축은 Docker와 pybind11/CUDA build option이 중심 근거.
- `/erp42_ctrl_cmd/camera`는 `src/`에 endpoint 없음. `/erp42_ctrl_cmd/lidar`의 `brake==2` 조건은 publisher 값과 안 맞는 dead branch(mission-cards.md 재확인).

### 3-B. `erp42_main/src/`

| 패키지 | 주요 파일 | 시뮬 | 비전 | 좌표계 | 통신 | 노드설계 | 모션플래닝 | 상태관리 | 안전 | 인프라 | 데이터 | HRI |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| `erp_driver` | `scripts/erp42_imu-gps-wheel-ekf_globalposition.py` | — | — | ● | ● | ○ | ○ | ● | ○ | ○ | ○ | — |
| `erp_driver` | `scripts/global_rotate.py` | — | — | ● | ● | ○ | — | — | — | — | — | — |
| `erp_driver` | `scripts/erp42_pubwaypointscnuservice_pymap3d.py` | — | — | ● | ● | ○ | ○ | ○ | — | — | ● | — |
| `erp_driver` | `scripts/erp42_pathtracking.py` | — | — | ○ | ● | ○ | ● | ● | ● | — | ○ | — |
| `erp_driver` | `scripts/erp42_serial.py` | — | — | — | ● | ○ | — | ● | ● | ○ | ○ | — |
| `erp_driver` | `scripts/erp42_lanedetect.py` | — | ● | — | ○ | ○ | ○ | ○ | ○ | — | — | — |
| `erp_driver` | `scripts/erp42_lanedetect_yolo.py`(**NameError로 노드 기동 실패 — §4-B 참고**) | — | ● | — | ○ | ○ | ○ | ○ | ● | — | — | — |
| `erp_driver` | `launch/erp42_base.launch.py` | — | — | — | ○ | — | — | — | ○ | ● | — | — |
| `erp_interfaces` | `msg/ErpCmdMsg.msg`, `msg/ErpStatusMsg.msg` | — | — | — | ● | — | — | ○ | ○ | ○ | ○ | — |
| `ntrip_client` | `scripts/ntrip_ros_base.py` | — | — | ○ | ● | ○ | — | ○ | — | ○ | ○ | — |
| `ntrip_client` | `launch/ntrip_client_launch.py` | — | — | ○ | ○ | — | — | ○ | — | ● | — | — |
| `rosbag_convert_clean_py` | `gps_convert.py` | — | — | ○ | — | — | — | — | — | — | ● | — |
| `ublox` | `package.xml` | — | — | — | ○ | — | — | — | — | ○ | — | — |
| `ublox_gps` | `src/node.cpp` | — | — | ○ | ● | ○ | — | ○ | — | ○ | ● | — |
| `ublox_msgs` | `msg/NavPVT.msg` | — | — | ○ | ○ | — | — | ○ | — | — | ● | — |
| `ublox_serialization` | `package.xml` | — | — | — | ○ | — | — | — | — | ○ | ○ | — |
| `usb_cam` | `launch/camera_config.py` | — | ○ | — | ○ | — | — | — | — | ● | — | — |
| `usb_cam` | `src/usb_cam_node.cpp` | — | ● | — | ● | ○ | — | ○ | — | ○ | ○ | — |
| `vectornav` | `src/vn_sensor_msgs.cc` | — | — | ○ | ● | ○ | — | ○ | — | ○ | ● | — |
| `vectornav` | `launch/vectornav.launch.py` | — | — | ○ | ○ | — | — | — | — | ● | — | — |
| `vectornav_msgs` | `msg/InsGroup.msg` | — | — | ○ | ○ | — | — | ○ | — | — | ● | — |
| `waypoint` | `waypoints_real_w2toe4_gps.xls` | — | — | ○ | — | — | ○ | — | — | — | ○ | — |
| `yolo_ros` | `Dockerfile` | ● | — | — | — | — | — | — | — | ● | — | — |
| `yolo_ros` | `yolo_ros/yolo_ros/yolo_node.py` | ○ | ● | — | ● | ● | — | ● | — | ○ | ○ | — |
| `yolo_ros` | `yolo_bringup/launch/yolo.launch.py` | — | ○ | — | ● | ○ | — | ○ | — | ● | — | — |
| `[src root]` | `rosbag2csv.py` | ● | — | — | — | — | — | — | — | — | ● | — |

**3-B 참고**: HRI는 `erp42_main/src/` 전 범위에서 확인 안 됨(빈칸 처리, 억지로 채우지 않음).

---

## 4. 파일별 × 축 상세

전체 상세(파일:라인 근거, §0-1 기여경계 한 줄)는 원본 문서에 보존돼 있다. 요약과 핵심 발견만 아래에 옮긴다 — **전체를 인용해야 할 때는 원본을 직접 참조할 것**: `docs/portfolio/PORTFOLIO_MAP_src.md` §4, `docs/portfolio/PORTFOLIO_MAP_erp42_main.md` §4.

### 4-A. `src/` 핵심 발견 (전체는 PORTFOLIO_MAP_src.md 참고)
- `1024_EBIMU_EKF.py`: `pymap3d.geodetic2enu`(`:126`)로 ENU 변환, 고정 선형 관측행렬 `H`(`:133`) — sigma-point 생성 근거 없어 **UKF 아님, 선형 EKF**로 서술.
- `erp42_pathtracking.py`: nearest/lookahead waypoint(`:84,128,161`) + heading error 비례제어 + `alpha=0.65` blending(`:184-186`, PID의 I/D 항 없음). `brake=155` 정지(`:195-201`) vs `brake==2` LiDAR 분기는 index 갱신뿐인 dead branch(`:142-145`).
- `Pick_Tray_SA.py`: Isaac Sim 앱, `/hand_raw`/`/hand_xyz`/`/hand_mode` 구독(`:251-266`) — **ERP42 대회와 무관한 별도 프로젝트 파일로 판단, 포트폴리오 성과 주장에서 제외**.
- `fast_gicp`, `hdl_global_localization`, `ndt_omp`, `pcl_ros`, `robot_localization` 다수 파일: **bundled upstream 소스로 보이며 개인 기여 범위는 `[확인 필요]`** — git 이력 없어 확정 불가.
- `Yolo_pt/best.pt`, `last.pt`: binary weight만 존재, loader/model declaration 없음 — YOLO 버전·클래스·정확도·최종 채택 여부 전부 `[확인 필요]`.

### 4-B. `erp42_main/src/` 핵심 발견 (전체는 PORTFOLIO_MAP_erp42_main.md 참고)
- `erp42_imu-gps-wheel-ekf_globalposition.py`: quaternion→yaw+90°보정(`:149-152`), geodetic→ENU(`:154-164`), GPS covariance 기반 update(`:154-188`) — vectornav 버전 EKF.
- `erp42_pathtracking.py`: `/erp42_ctrl_cmd`를 **직접** 발행(`:35-57`) — src판과 달리 Controller Node를 거치지 않음. `brake==2` LiDAR 분기는 이 ws에 해당 publisher가 없어 dead branch(mission-cards.md 재확인).
- `erp42_lanedetect_yolo.py`: import는 `DetectionArray`인데 subscription은 미정의 `Detection2DArray` 사용 → **`NameError`로 노드 기동 자체가 실패**(`:8-9, :20-24`). 정적 경로상 dead — "완성"으로 서술 금지.
- `yolo_node.py`: `LifecycleNode` + configure/activate 전이(`:49-52,74-128,130-159`), CUDA cache 정리(`:161-167`) — Ultralytics/PyTorch가 inference 제공, ROS wrapper는 lifecycle·QoS·서비스 구성.
- `ntrip_client_launch.py`: **인증정보가 소스에 평문으로 들어 있음**(`username`/`password`, `:18-19`) — 이 문서엔 값을 옮기지 않음, 포트폴리오 공개 시 반드시 마스킹 필요.
- `usb_cam/launch/camera_config.py`: camera1/2 launch만 있고 params_3/4를 참조하는 launch 없음(mission-cards.md 판정 재확인).

---

## 5. 기반 기술 요소 (§3-A 기준)

### 5-A. `src/`

| 요소 | 근거 | 판정 |
|---|---|---|
| 센서 전력·대역폭 | 카메라 MJPEG 30fps 640×640/640×360(`params_1/2.yaml:3-10`), Velodyne VLP16 600rpm 고정IP(`VLP16-velodyne_driver_node-params.yaml:3,10-12`) | 대역폭 접점 있음. 전력 budget·USB 분산은 **코드로 확인 안 됨** |
| 레이턴시 최적화 | `SensorDataQoS`(cluster_bev_node.cpp:43-49), 40Hz timer(erp42_serial.py:27) | 접점 있음. end-to-end 측정·profiling은 **코드로 확인 안 됨** |
| CPU·GPU 최적화 | fast_gicp thread 수 노출, CUDA build option(CMakeLists.txt:4,31-34) | 접점 확인됨. 실제 GPU 사용률·벤치마크는 `[확인 필요]` |
| 하드웨어 매뉴얼 기반 환경구성 | camera/Velodyne/EBIMU device·baudrate 명시 | 환경구성 확인됨. vendor manual 항목 매핑은 **코드로 확인 안 됨** |
| 언어·런타임 최적화 | C++(vendor stack)/Python(orchestration) 분리, pybind11 ABI bridge | 역할 분리 확인. 현재 Humble에서 실사용 근거 없어 **접점 낮음** |

### 5-B. `erp42_main/src/`

| 요소 | 근거 | 판정 |
|---|---|---|
| 센서 전력·대역폭 | camera1 640×480 30Hz YUV422(`params_1.yaml:3-22`), VectorNav 필요 field만 enable(`InsGroup.msg:1-5`), 20Hz(`vectornav.yaml:7-37`) | 대역폭 접점 있음. 전력/USB hub 실측은 **코드로 확인 안 됨** |
| 레이턴시 최적화 | ERP serial 40Hz(`erp42_serial.py:27`), EKF 20Hz(`:74`), YOLO QoS depth 1(`yolo_node.py:115-120`) | 접점 있음. end-to-end 측정은 **코드로 확인 안 됨** |
| CPU·GPU 최적화 | YOLO 기본 `cuda:0`, optional half precision(`yolo_node.py:55-70,335-348`), CUDA cache 정리(`:161-167`) | GPU 접점 있음. profiling/TensorRT는 **코드로 확인 안 됨** |
| 하드웨어 매뉴얼 기반 환경구성 | ERP serial/u-blox/VectorNav device·baudrate·protocol 명시(`erp42_base.launch.py:6-13`, `zed_f9p.yaml:1-17`, `vectornav.yaml:1-26`) | 환경구성 근거 있음. device path 충돌은 runtime `[확인 필요]` |
| 언어·런타임 최적화 | C++(driver)/Python(orchestration), `numpy<2`+Ultralytics 8.3.91 고정(`requirements.txt:1-4`) | 호환성 접점 있음. 성능 벤치마크는 **접점 낮음** |

---

## 6. 자체 대조 (누락 대조)

### 6-A. `src/`
- §2→§3 커버리지: **PASS — 17/17** 패키지 전부 매트릭스에 최소 1행.
- §3 ●→§4 앵커: **PASS — 28/28** (시뮬/격리4, 비전1, 좌표계2, 통신6, 노드설계1, 모션플래닝1, 상태관리1, 안전3, 인프라7, 데이터1, HRI1).

### 6-B. `erp42_main/src/`
- §2→§3 커버리지: **PASS — 13/13 + src root 1/1**.
- §3 ●→§4 앵커: **PASS** — dead topic/branch 판정(`/erp42_ctrl_cmd/lidar` publisher 0건, `/erp42_ctrl_cmd/lane` subscriber 0건, `/erp42_ctrl_cmd/camera` endpoint 0건)은 mission-cards.md/architecture.md와 재확인 일치.

### 6-C. 검증 수준 (공통)
- **정적 source/config**: PASS — 두 ws 모두 inventory·LOC·mtime·line anchor·dead-path grep을 현재 checkout에서 확인.
- **build/test**: 수행하지 않음 — 문서 작성 작업, 코드 변경 없음.
- **simulation/runtime topic graph**: 수행하지 않음.
- **hardware**: 수행하지 않음 — 실제 센서 연결, QoS 호환성, 차량 actuation은 `[확인 필요]`.
- 두 ws의 기존 검증 사실(`mission-cards.md`, `architecture.md`, `tech-stack.md`)을 재사용했고, 개별 variant 존재를 최종 대회 run 통합으로 승격하지 않았다.

### 6-D. 병합 시 발견한 주의사항 (Phase 3 `PORTFOLIO_SOURCE.md` 작성 시 반드시 반영)
1. **`src/usb_cam/launch/Pick_Tray_SA.py`는 ERP42 대회와 무관한 별개 프로젝트(Isaac Sim pick-and-place) 파일** — 포트폴리오 성과·LOC·11축 커버리지 주장에서 제외할 것. [시뮬]/[HRI] 축에 이 파일 근거로 부풀리지 않는다.
2. **`ntrip_client/launch/ntrip_client_launch.py`(erp42_main)에 인증정보 평문 포함**(`username`/`password`, `:18-19`) — 포트폴리오 공개 시(GitHub 링크/스니펫) 반드시 마스킹.
3. **erp42_main판 `erp42_lanedetect_yolo.py`는 `NameError`로 노드 기동 자체가 실패** — "완성"으로 서술 금지, "미완성/미검증"으로 톤 낮출 것(mission-cards.md 기존 판정과 일치).
4. **다수 파일이 bundled upstream/vendor 소스로 추정**(fast_gicp, hdl_global_localization, ndt_omp, pcl_ros, robot_localization 일부, ublox_gps/msgs/serialization, vectornav 등) — git 이력이 없어 개인 기여 범위를 소스만으로 확정 불가. §0-1 기여경계 작성 시 "라이브러리/upstream이 제공 / 통합·설정·연결이 코드의 몫"으로 보수적으로 서술.
5. **`rosbag_convert_clean_py`의 실제 목적 확인(사용자 확인, 2026-08-20)**: `gps_convert.py`/`odom_convert`/`plot_waypoint_csv`에 하드코딩된 팀원 PC 경로(`/home/mrlam/colcon_ws/bagfiles_ros2/...`)로 볼 때, 이 패키지는 범용 로그 정제 도구가 아니라 **NTRIP/RTK 보정 GPS(`/ublox_gps_node/fix`)를 rosbag로 기록→CSV 변환→정제해서 waypoint 파일(`waypoints_real_e4tow2_gps.xls` 등, 경로명 `e4tow2` 일치)을 생성하는 파이프라인**이었다. 실제 rosbag 파일은 두 ws 어디에도 없음(`robot_localization`의 vendor 테스트용 `test1~3.bag` 3개만 존재). 부가로 `odom_ukf_ctrl11.csv`/`test3_UKF_Pathtrack` 등 경로명이 UKF 실험 흔적을 시사함(mission-cards.md UKF/EKF 판정에 반영됨).
