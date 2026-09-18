# `src/` 포트폴리오 교차관심사 맵

> 범위는 개인 작업본 `src/`뿐이다. `erp42_main/`은 별도 작업 중이므로 이 문서의 스캔·LOC·매트릭스·상세 근거에서 제외했다.  
> 판정은 정적 source/config 확인 기준이다. 파일 존재는 한 번의 대회 run에 통합·실행됐다는 증거가 아니며, 확인할 수 없는 사항은 `[확인 필요]`로 남긴다.

## 1. 버전 고정 헤더

- 스캔 경로: `/home/kimkh/colcon_ws/src`
- 저장소 상태: `/home/kimkh/colcon_ws` 자체는 git repository가 아니므로 고정할 HEAD가 없다.
- Python 최신 mtime 산출 명령: `find src -name '*.py' -printf '%T@ %p\n' | sort -n | tail -1`
- Python 최신 mtime 결과: `1782262910.1972822240 src/usb_cam/launch/Pick_Tray_SA.py`
- 하드웨어 토폴로지(사용자 제공): 카메라 설정 4대 중 실배선 1~2대, Velodyne LiDAR, GPS+RTK, IMU(`vectornav`에서 `ebimu`로 변경), 휠 인코더, ERP42.
- source로 추가 확인되는 카메라 launch: `camera1`+`params_1.yaml`(`src/usb_cam/launch/camera.launch.py:52-53`)과 `camera2`+`params_2.yaml`(`src/usb_cam/launch/2camera.launch.py:52-53`). 실제 동시 배선 수와 physical 역할은 source만으로 확정할 수 없어 `[확인 필요]`이다.
- 참고 사실 재사용: `docs/portfolio/mission-cards.md`, `docs/portfolio/tech-stack.md`, `docs/portfolio/architecture.md`. 특히 구현 존재와 최종 통합을 구분하고, custom sensor fusion은 UKF가 아니라 선형 EKF로 서술한다.

## 2. `src/` 전 패키지 Python LOC 체크리스트

집계 명령은 `find src -name '*.py' -exec wc -l {} +`이며, 패키지는 `src/` 바로 아래 최상위 디렉터리 단위로 합산했다. Python 파일이 없는 디렉터리도 0 LOC로 남겼다.

| 확인 | 최상위 디렉터리 | `.py` 파일 수 | Python LOC |
|---|---|---:|---:|
| ☑ | `cluster_bev` | 0 | 0 |
| ☑ | `ebimu_pkg` | 7 | 219 |
| ☑ | `erp_driver` | 30 | 4,131 |
| ☑ | `erp_interfaces` | 3 | 26 |
| ☑ | `fast_gicp` | 2 | 176 |
| ☑ | `hdl_global_localization` | 0 | 0 |
| ☑ | `hdl_localization` | 3 | 197 |
| ☑ | `ndt_omp` | 0 | 0 |
| ☑ | `ntrip_client` | 12 | 1,156 |
| ☑ | `pcl_clustering_py` | 6 | 241 |
| ☑ | `pcl_ros` | 5 | 255 |
| ☑ | `robot_localization` | 16 | 975 |
| ☑ | `ublox` | 2 | 128 |
| ☑ | `usb_cam` | 6 | 1,053 |
| ☑ | `vectornav` | 2 | 73 |
| ☑ | `velodyne` | 22 | 1,674 |
| ☑ | `Yolo_pt` | 0 | 0 |
| **합계** | **17개 최상위 디렉터리** | **116** | **10,304** |

## 3. 파일 × 11축 교차관심사 매트릭스

범례: `●` 중심, `○` 접점, 빈칸은 **해당없음**이다. 이 문서의 `[시뮬]`은 요청 범위에 따라 simulation뿐 아니라 venv/conda/ABI bridging/Docker 격리도 포함한다. `Yolo_pt`는 executable source가 아닌 binary weight 보관 디렉터리라 중심(`●`) 판정을 하지 않았다.

| 패키지 | 주요 파일 | 시뮬 | 비전 | 좌표계 | 통신 | 노드설계 | 모션플래닝 | 상태관리 | 안전 | 인프라 | 데이터 | HRI |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| `cluster_bev` | `src/cluster_bev/src/cluster_bev_node.cpp` |  |  |  | ● |  |  |  |  | ○ | ○ |  |
| `ebimu_pkg` | `src/ebimu_pkg/ebimu_pkg/ebimu_publisher.py` |  |  | ○ | ● |  |  |  |  |  | ○ |  |
| `erp_driver` | `src/erp_driver/scripts/erp42_ebimu_ekf_globalposition.py` |  |  | ● | ○ |  |  | ○ |  |  | ○ |  |
| `erp_driver` | `src/erp_driver/scripts/erp42_pathtracking.py` |  |  | ○ | ○ |  | ● | ○ | ● |  |  |  |
| `erp_driver` | `src/erp_driver/scripts/erp42_controller.py` |  |  |  | ○ |  |  | ● | ● |  |  |  |
| `erp_driver` | `src/erp_driver/scripts/archive/0702_erp42_lanedetect.py` |  | ● |  | ○ |  | ○ |  |  |  |  |  |
| `erp_driver` | `src/erp_driver/scripts/archive/0822_Obstacle_3d.py` |  |  |  | ○ |  | ○ |  | ● |  | ○ |  |
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
| `usb_cam` | `src/usb_cam/launch/Pick_Tray_SA.py` | ● | ○ | ○ | ○ |  | ○ | ○ | ○ |  |  | ● |
| `vectornav` | `src/vectornav/vectornav/launch/vectornav.launch.py` |  |  | ○ | ○ |  |  |  |  | ● |  |  |
| `velodyne` | `src/velodyne/velodyne/launch/velodyne-all-nodes-VLP16-launch.py` |  |  | ○ | ○ |  |  |  |  | ● | ○ |  |
| `Yolo_pt` | `src/Yolo_pt/best.pt` |  | ○ |  |  |  |  |  |  |  | ○ |  |

### 3.1 축별 정직한 공백 해석

- HRI는 ERP42 주행 stack에서 확인되지 않았다. 유일한 명시적 접점은 별도 Isaac Sim application인 `Pick_Tray_SA.py`의 `/hand_raw`, `/hand_xyz`, `/hand_mode` 입력이다.
- `Executor`, `CallbackGroup`, Python `Lock` 사용은 선택한 ERP42 Python node에서 확인되지 않았다. 노드설계의 중심 근거는 `fast_gicp`의 thread 수 설정과 ERP serial의 40 Hz timer 접점이다.
- `venv`/`conda` 구성은 주요 실행 경로에서 확인되지 않았다. 격리·ABI 축의 중심 근거는 Docker와 pybind11/CUDA build option이다.
- `/erp42_ctrl_cmd/camera`는 `src/`에서 endpoint가 확인되지 않는다. `/erp42_ctrl_cmd/lidar`의 `brake==2` 조건은 publisher 값과 맞지 않는 dead branch다. 이는 `mission-cards.md`의 검증 결과를 재사용했다.

## 4. 파일별 × 축 상세

### 4.1 `cluster_bev`

#### `src/cluster_bev/src/cluster_bev_node.cpp`

- **● 통신:** `input_topic`/`output_topic_bev` parameter를 선언하고(`:26-27`), Velodyne stream에 `rclcpp::SensorDataQoS()`를 적용해 subscribe한 뒤 PointCloud2를 publish한다(`:43-49`).
- **§0-1 기여경계:** 라이브러리 `rclcpp`와 PCL이 ROS 통신·point-cloud 처리를 제공 / 코드는 voxel·RANSAC·clustering parameter와 `/clusters_bev` 변환 node를 구성한다. 개인 수정 범위는 git provenance만으로 분리되지 않아 `[확인 필요]`.

### 4.2 `ebimu_pkg`

#### `src/ebimu_pkg/ebimu_pkg/ebimu_publisher.py`

- **● 통신:** pyserial로 입력받을 장치를 115200 baud로 열고(`:13-16`), `ebimu_data` publisher를 생성해(`:30`) `readline()` 결과를 전달한다(`:37`).
- **§0-1 기여경계:** 라이브러리 pyserial/rclpy가 serial I/O와 ROS publisher를 제공 / 코드는 EBIMU port 선택, line read, String topic 변환을 연결한다.

### 4.3 `erp_driver`

#### `src/erp_driver/scripts/erp42_ebimu_ekf_globalposition.py`

- **● 좌표계:** `tf_transformations` quaternion↔Euler를 사용하고(`:8,98,179`), GPS 위경도를 `pymap3d.geodetic2enu`로 ENU 변환한다(`:126`). GPS update는 고정 선형 관측행렬 `H`를 사용한다(`:133`); sigma-point 생성 근거가 없으므로 UKF로 쓰지 않는다.
- **§0-1 기여경계:** 라이브러리 NumPy/pymap3d/tf_transformations가 행렬·ENU·quaternion 연산을 제공 / 코드는 EBIMU·GPS·wheel status subscription, 선형 EKF update/predict, `/odom_ekf` 출력을 조합한다(`:61-64,121,141`).

#### `src/erp_driver/scripts/erp42_pathtracking.py`

- **● 모션플래닝:** nearest/lookahead waypoint를 선택하고(`:84,128,161`), heading error를 계산해(`:122,175`) 비례 steering에 `alpha=0.65` 이전값 blending을 적용한다(`:184-186`). PID의 I/D 항 근거는 없다.
- **● 안전:** final waypoint에서 `stop_robot()`을 호출하고(`:170-171`), 정지 명령은 `brake=155`다(`:195-201`). 반면 LiDAR 조건 `brake==2`는 waypoint index만 갱신하며(`:142-145`), `mission-cards.md`에서 확인한 것처럼 `/lidar` publisher 중 값 2를 내는 node가 없어 dead branch다.
- **§0-1 기여경계:** 라이브러리 rclpy/tf_transformations가 ROS 통신과 yaw 추출을 제공 / 코드는 odometry·path·LiDAR command를 결합한 waypoint navigator와 ERP42 command 생성을 구현한다.

#### `src/erp_driver/scripts/erp42_controller.py`

- **● 상태관리:** lane command를 path command보다 먼저 검사하는 `if/elif` selector와 freshness timeout을 둔다(`:37-46`). 이는 두 채널 prototype이며 보고서의 camera/lidar/path 3-tier 전체 구현은 아니다.
- **● 안전:** 어느 입력도 valid하지 않으면 `brake=155` full-brake command를 publish한다(`:48-58`).
- **§0-1 기여경계:** 라이브러리 rclpy가 timer/pub-sub를 제공 / 코드는 lane/path command의 유효시간, brake sentinel, 우선순위와 fallback brake state를 정의한다(`:16-24,39-55`).

#### `src/erp_driver/scripts/archive/0702_erp42_lanedetect.py`

- **● 비전:** OpenCV로 HSV white/yellow mask를 만들고(`:58-63,111-117`), Canny/Hough line을 계산한다(`:129-134`). `/usb_cam_0/image_raw`을 받아 `/erp42_ctrl_cmd/lane`을 publish한다(`:366-368`). camera launch output과의 topic mismatch 가능성은 `architecture.md` 근거상 남아 있다.
- **§0-1 기여경계:** 라이브러리 OpenCV/cv_bridge가 image 변환·HSV·edge/Hough 연산을 제공 / 코드는 ROI, lane candidate, steering command node를 구성한다.

#### `src/erp_driver/scripts/archive/0822_Obstacle_3d.py`

- **● 안전:** `/velodyne_points`를 구독해 `/erp42_ctrl_cmd/lidar`를 발행한다(`:14,17`); 장애물 조건에서 `brake=200`, 그 외 `brake=0`을 낸다(`:69,117`). 따라서 pathtracking이 요구하는 `brake==2`와 맞지 않아 최종 selector 연동은 dead branch이며, standalone safety experiment로만 서술한다.
- **§0-1 기여경계:** 라이브러리 rclpy/sensor_msgs가 PointCloud2 pub-sub를 제공 / 코드는 ROI 기반 장애물 판단과 ERP42 brake command 실험을 구성한다. 실제 제동 검증은 수행하지 않았다.

#### `src/erp_driver/scripts/erp42_serial.py`

- **● 통신:** `/erp42_ctrl_cmd` subscriber와 `/erp42_status` publisher를 만들고(`:24-25`), 40 Hz timer로 serial packet을 송신한다(`:27,42-44`). 수신 packet에서 brake와 encoder를 복원한다(`:31,56-57`).
- **§0-1 기여경계:** 라이브러리 pyserial/rclpy가 UART와 ROS I/O를 제공 / 코드는 ERP42 packet encode/decode, command 송신, status/encoder publish를 연결한다.

#### `src/erp_driver/scripts/erp42_pubwaypointscnuservice_pymap3d.py`

- **● 데이터:** Excel waypoint를 pandas로 읽고(`:41`), 각 위경도를 ENU로 변환해(`:51`) `/waypoints_path1` Path로 publish한다(`:19`). 입력 경로가 `/home/mrlam/...xls`로 hard-coded되어 있어(`:86`) 현재 machine 재현에는 `[확인 필요]`가 남는다.
- **§0-1 기여경계:** 라이브러리 pandas/pymap3d가 Excel parsing과 geodetic→ENU 변환을 제공 / 코드는 waypoint file을 ROS `Path` message로 구성·발행한다.

### 4.4 `erp_interfaces`

#### `src/erp_interfaces/msg/ErpCmdMsg.msg`

- **● 통신:** ERP42 command contract를 `e_stop`, `gear`, `speed`, `steer`, `brake` field로 정의한다(`:1-5`). Status 쪽은 encoder를 포함한다(`src/erp_interfaces/msg/ErpStatusMsg.msg:1-8`).
- **§0-1 기여경계:** ROSIDL이 message code generation을 제공 / 코드는 ERP42 command/status의 wire-level field contract를 정의한다.

### 4.5 `fast_gicp`

#### `src/fast_gicp/CMakeLists.txt`

- **● 시뮬/격리:** Python ABI bridge를 위한 `BUILD_PYTHON_BINDINGS` option과 pybind11 module을 정의한다(`:7,99-101`); GPU VGICP를 위한 CUDA option도 분리돼 있다(`:4,31-34,114-117`).
- **● 노드설계:** native implementation의 thread 수는 Python binding에서 `setNumThreads`로 노출된다(`src/fast_gicp/src/python/main.cpp:100,107,186`). 이는 ROS Executor 설계가 아니라 point-cloud registration 내부 parallelism이다.
- **§0-1 기여경계:** 라이브러리 fast_gicp/PCL/CUDA/pybind11이 registration, parallelism, Python ABI를 제공 / 이 디렉터리는 bundled upstream source로 보이며 개인 구현·수정 범위는 `[확인 필요]`.

### 4.6 `hdl_global_localization`

#### `src/hdl_global_localization/docker/noetic/Dockerfile`

- **● 시뮬/격리:** `ros:noetic` image에서 catkin workspace를 만들고 package를 복사해 `catkin_make`한다(`:1,9-17`). 현재 ROS 2 Humble workspace와 같은 runtime이라고 볼 수 없으며 별도 legacy container recipe다.
- **§0-1 기여경계:** Docker/ROS Noetic/catkin이 legacy build environment를 제공 / 파일은 upstream localization package의 재현 recipe이며 개인 기여 여부는 `[확인 필요]`.

### 4.7 `hdl_localization`

#### `src/hdl_localization/launch/hdl_localization.launch.py`

- **● 인프라:** parameter file과 global map을 받아 GlobalmapServer·HdlLocalization composable node를 container에 배선한다(`:36-38,70-91`), 별도 Node action도 구성한다(`:63,99-100`).
- **§0-1 기여경계:** ROS 2 launch/components와 HDL localization library가 lifecycle·registration 기능을 제공 / launch 코드는 component container, parameter, map wiring을 선언한다. 대회 ERP42 path와의 통합은 `[확인 필요]`.

### 4.8 `ndt_omp`

#### `src/ndt_omp/docker/foxy/Dockerfile`

- **● 시뮬/격리:** `ros:foxy` image에서 `/root/colcon_ws`를 만들고 package를 복사한 뒤 `colcon build`한다(`:1,10-19`). Humble 실행 증거가 아니라 Foxy build recipe다.
- **§0-1 기여경계:** Docker/ROS Foxy/OpenMP NDT가 isolated build와 registration을 제공 / 파일은 bundled upstream package의 environment recipe이며 개인 수정 범위는 `[확인 필요]`.

### 4.9 `ntrip_client`

#### `src/ntrip_client/scripts/ntrip_ros_base.py`

- **● 통신:** RTCM message type을 선택해 `rtcm` publisher를 만들고(`:61-78`), NMEA와 u-blox fix를 구독한다(`:100-101`). 0.1 s timer가 NTRIP client에서 받은 RTCM을 publish한다(`:103-104,163-182`).
- **§0-1 기여경계:** 라이브러리 rclpy와 NTRIP client가 ROS transport와 correction stream 처리를 제공 / 코드는 NMEA/GPS 입력, RTCM type adapter, timer publication을 연결한다. server credential·실제 RTK fix는 이 정적 조사에서 확인하지 않았다.

### 4.10 `pcl_clustering_py`

#### `src/pcl_clustering_py/pcl_clustering_py/euclidean_cluster_node.py`

- **● 통신:** PointCloud2를 subscribe하고 `/clustered_points` publisher를 만든다(`:32-37`); ROS↔Python-PCL 변환 후 RANSAC·Euclidean clustering 결과를 publish한다(`:41-44,56-84,93-123`).
- **§0-1 기여경계:** Python-PCL/NumPy/rclpy가 point-cloud container·segmentation·message transport를 제공 / 코드는 conversion, plane removal, clustering, colorized output node를 구성한다. downstream decision/control consumer는 확인되지 않았다.

### 4.11 `pcl_ros`

#### `src/pcl_ros/src/transforms.cpp`

- **● 좌표계:** source/target frame과 timestamp로 TF를 lookup하고(`:65-80`), point cloud를 변환해 output `frame_id`를 target으로 갱신한다(`:93-95`). tf2 Transform/Eigen 변환도 제공한다(`:120-126,234-255`).
- **§0-1 기여경계:** 라이브러리 PCL/tf2/Eigen이 cloud·TF·matrix primitives를 제공 / 이 파일은 upstream `pcl_ros` transform implementation이며 개인 기여로 귀속하지 않는다.

### 4.12 `robot_localization`

#### `src/robot_localization/launch/ekf.launch.py`

- **● 인프라:** `robot_localization`의 `ekf_node`를 package parameter `params/ekf.yaml`과 함께 launch한다(`:18,28-33`). 이 upstream EKF package가 존재한다는 사실은 대회용 custom EKF의 구현 기여와 분리한다.
- **§0-1 기여경계:** 라이브러리 `robot_localization`이 generic EKF node를 제공 / launch 파일은 parameter file과 executable wiring을 제공하는 upstream example이다.

#### `src/robot_localization/test/test_ekf_localization_node_bag1.launch.py`

- **● 데이터:** bag1 test용 YAML 경로와 output file argument를 만들고(`:25-34`), EKF와 pose tester node를 함께 launch한다(`:42-60`). `test1.bag` 등 bag file은 존재하지만 이 launch에서 playback command는 확인되지 않았고 CMake의 bag tests도 주석 처리돼 있어(`src/robot_localization/CMakeLists.txt:239-255`) 실행 상태는 `[확인 필요]`다.
- **§0-1 기여경계:** `launch_testing`/robot_localization test executable이 validation harness를 제공 / 이 파일은 bundled upstream test asset이며 개인 대회 데이터 처리 기여로 귀속하지 않는다.

### 4.13 `ublox`

#### `src/ublox/ublox_gps/launch/ublox_gps_node-launch.py`

- **● 인프라:** `ublox_gps_node` executable을 parameter와 함께 구성한다(`:47-50`)며, node 종료를 launch shutdown으로 연결한다(`:52-56`).
- **§0-1 기여경계:** u-blox ROS 2 driver가 GNSS protocol/device 처리를 제공 / launch 코드는 driver parameter wiring과 process termination policy를 제공하는 upstream package asset이다.

### 4.14 `usb_cam`

#### `src/usb_cam/launch/camera_config.py`

- **● 인프라:** Pydantic model로 camera node의 `remappings`와 `namespace`를 검증하고(`:34,42-43`), name이 있으면 image/camera_info remapping을 자동 생성한다(`:54-64`). 실제 device/framerate/resolution은 `params_1.yaml:3-10`, `params_2.yaml:3-10`에 분리돼 있다.
- **§0-1 기여경계:** usb_cam/Pydantic/ROS launch가 camera capture와 config validation을 제공 / 코드는 parameter/remapping schema를 구성한다. camera1/2 physical 역할은 `[확인 필요]`.

#### `src/usb_cam/launch/Pick_Tray_SA.py`

- **● 시뮬/격리:** Isaac Sim `SimulationApp`을 열고 ROS 2 bridge와 surface-gripper extension을 활성화한다(`:3-10`), USD stage/world API를 사용한다(`:19,22-28`). ERP42 주행과는 별도 application이다.
- **● HRI:** `HandSubscriber`가 `/hand_raw`, `/hand_xyz`, `/hand_mode`를 구독하고(`:251-266`), hand marker/target과 `HOME`·`TRACKING` mode를 simulation loop에 반영한다(`:413,442,554-558,606,622,720`).
- **§0-1 기여경계:** Isaac Sim/Omniverse/rclpy가 simulation·USD·ROS bridge를 제공 / 코드는 hand-tracking input을 M0609 scene의 target/mode로 연결하는 application logic을 구성한다. 실제 hand tracker publisher는 이 파일 밖이라 `[확인 필요]`.

### 4.15 `vectornav`

#### `src/vectornav/vectornav/launch/vectornav.launch.py`

- **● 인프라:** 같은 `vectornav.yaml`을 사용해 raw driver와 sensor_msgs 변환 node를 함께 launch한다(`:10-23,28-29`). 사용자 제공 토폴로지상 현재 IMU가 EBIMU로 변경됐으므로, 이 package의 현행 실배선 사용 여부는 `[확인 필요]`다.
- **§0-1 기여경계:** VectorNav vendor driver가 serial sensor protocol과 message 변환을 제공 / launch 코드는 driver와 sensor_msgs node 및 config를 배선하는 upstream asset이다.

### 4.16 `velodyne`

#### `src/velodyne/velodyne/launch/velodyne-all-nodes-VLP16-launch.py`

- **● 인프라:** VLP16 driver, pointcloud transform, laserscan node의 config를 읽어 세 node를 한 launch에 구성한다(`:43-70`). Driver parameter는 `device_ip=192.168.1.201`, `frame_id=velodyne`, `model=VLP16`, `rpm=600`이다(`src/velodyne/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml:3,10-12`).
- **§0-1 기여경계:** Velodyne ROS 2 stack이 UDP packet decode, PointCloud2, LaserScan 변환을 제공 / launch 코드는 VLP16별 driver-transform-scan pipeline을 구성하는 upstream asset이다.

### 4.17 `Yolo_pt`

#### `src/Yolo_pt/best.pt`

- **● 없음:** `best.pt`와 `last.pt` binary weight가 존재하므로 비전/데이터 접점(`○`)은 있으나, line anchor가 있는 loader·model declaration·ROS publisher가 이 디렉터리에 없다. filename만으로 YOLO major version, class set, accuracy, 최종 채택 여부를 정할 수 없어 모두 `[확인 필요]`다.
- **§0-1 기여경계:** 추정 가능한 YOLO library가 학습·inference 형식을 제공 / 이 디렉터리에는 weight artifact만 있고 학습 코드·dataset provenance·개인 기여 경계가 없어 확정하지 않는다.

## 5. 기반 기술 요소 (§3-A, `src/`만)

| 요소 | 코드 근거 | 판정 |
|---|---|---|
| 센서 전력·대역폭 | 카메라는 MJPEG, 30 fps, 640×640/640×360으로 설정됨(`src/usb_cam/config/params_1.yaml:3-10`, `params_2.yaml:3-10`). Velodyne는 VLP16, 600 rpm, 고정 device IP를 사용(`src/velodyne/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml:3,10-12`). | 해상도·fps·encoding·rpm이라는 대역폭 접점은 있음. 전력 budget, USB controller 분산, NIC throughput 계산은 **코드로 확인 안 됨**. |
| 레이턴시 최적화 | cluster node가 sensor stream에 `SensorDataQoS`를 사용(`src/cluster_bev/src/cluster_bev_node.cpp:43-49`); ERP serial은 40 Hz timer(`src/erp_driver/scripts/erp42_serial.py:27`). | low-latency를 의도할 수 있는 접점은 있으나 end-to-end latency 측정, deadline/liveliness tuning, profiling 결과는 **코드로 확인 안 됨**. |
| CPU·GPU 최적화 | fast_gicp가 thread 수를 노출(`src/fast_gicp/src/python/main.cpp:100,107,186`)하고 CUDA build를 option으로 둠(`src/fast_gicp/CMakeLists.txt:4,31-34`). cluster node는 voxel leaf size parameter를 둠(`src/cluster_bev/src/cluster_bev_node.cpp:28,69`). | CPU parallelism·GPU compile path·downsampling 접점은 확인됨. 실제 build option, GPU 사용률, benchmark는 `[확인 필요]`; 대회 경로 통합도 확인 안 됨. |
| 하드웨어 매뉴얼 기반 환경구성 | camera device path/format/resolution, Velodyne model/IP/rpm, EBIMU baudrate가 config/code에 명시됨. | device-specific 환경구성은 확인됨. 이 값이 어떤 제조사 manual 항목에서 왔는지는 **코드로 확인 안 됨**. |
| 언어·런타임 최적화 | sensor/point-cloud vendor stack은 C++, orchestration/실험 node는 Python; fast_gicp는 pybind11 ABI bridge와 optional CUDA module을 제공(`src/fast_gicp/CMakeLists.txt:99-117`). Docker recipe는 ROS Noetic/Foxy를 분리한다. | C++/Python 역할 분리와 ABI/container 접점은 있음. 현재 Humble에서 Docker image나 Python binding을 실제 사용했다는 근거는 없어 **접점 낮음**. |

## 6. 자체 대조

### 6.1 §2 패키지 → §3 행 커버리지

- 결과: **PASS — 17/17**.
- 확인 목록: `cluster_bev`, `ebimu_pkg`, `erp_driver`, `erp_interfaces`, `fast_gicp`, `hdl_global_localization`, `hdl_localization`, `ndt_omp`, `ntrip_client`, `pcl_clustering_py`, `pcl_ros`, `robot_localization`, `ublox`, `usb_cam`, `vectornav`, `velodyne`, `Yolo_pt`가 §3에 각각 최소 1행 있다.
- `Yolo_pt`는 source가 아닌 binary weight만 있으므로 행은 유지하되 `●`를 부여하지 않았다.

### 6.2 §3 `●` → §4 파일:라인 앵커

- 결과: **PASS — 28/28 `●` 설명 완료**.
- 축별 개수: 시뮬/격리 4, 비전 1, 좌표계 2, 통신 6, 노드설계 1, 모션플래닝 1, 상태관리 1, 안전 3, 인프라 7, 데이터 1, HRI 1.
- `●`마다 §4에 최소 1개 `파일:라인` 근거를 붙였다. binary `Yolo_pt/best.pt`는 line anchor를 만들 수 없어 `○`로만 분류했다.

### 6.3 검증 수준과 남은 `[확인 필요]`

- 정적 source/config/LOC: 확인 완료.
- build/test: 문서 작성 작업이며 코드 변경이 없어 실행하지 않음.
- simulation/runtime topic graph: 실행하지 않음.
- hardware: camera physical 역할·동시 배선 수, VectorNav→EBIMU 전환 상태, RTK fix, LiDAR/ERP42 actuation은 실행·검증하지 않음.
- 통합 판정: `mission-cards.md`와 `architecture.md`의 기존 검증 사실을 재사용했다. 개별 variant의 존재를 최종 run 통합으로 승격하지 않았다.
