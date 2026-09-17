# ERP42 팀 정리본 포트폴리오 맵

> 이 문서는 팀 정리본 `erp42_main/src/`만 다룬다. 개인 작업본 `src/`는 다른 작업의 변경 대상이므로 조사 결과와 성과를 이 문서에 합치지 않았다. 정적 source/config 분석 문서이며 runtime, simulation, 실차 actuation은 수행하지 않았다. 확인할 수 없는 항목은 `[확인 필요]`로 표시한다.

## 1. 버전 고정 헤더

| 항목 | 고정값 |
|---|---|
| 스캔 경로 | `/home/kimkh/colcon_ws/erp42_main/src` |
| 버전 기준 | 이 디렉토리는 git repository가 아니므로 HEAD 없음 |
| 최신 Python mtime 조사 명령 | `find erp42_main/src -name '*.py' -printf '%T@ %p\n' \| sort -n \| tail -1` |
| 최신 Python mtime | epoch `1754633350.0000000000`, `2025-08-08 15:09:10 +0900` |
| 최신 Python 파일 | `erp42_main/src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py` |
| 하드웨어 토폴로지 | 카메라 4대 구성 중 실배선 1~2대(코드상 launch 배선은 camera1/camera2), GPS + RTK(NTRIP/u-blox), IMU(VectorNav), ERP42 휠 인코더, ERP42 차량 |

하드웨어 토폴로지는 사용자 제공 사실로 고정했다. camera 설정은 `params_1.yaml`~`params_4.yaml` 네 개지만 launch는 camera1/camera2만 참조한다. 어느 physical camera가 어느 미션을 담당했는지는 source로 복원되지 않아 `[확인 필요]`다.

## 2. `erp42_main/src/` 전 패키지 LOC 체크리스트

집계는 요청된 `find erp42_main/src -name '*.py' -exec wc -l {} +` 결과를 최상위 디렉토리별로 합산했다. Python 파일이 없는 디렉토리도 `find ... -mindepth 1 -maxdepth 1 -type d` 결과에서 제외하지 않았다.

| 최상위 디렉토리/범위 | Python 파일 수 | Python LOC | 체크 |
|---|---:|---:|---|
| `erp_driver` | 16 | 2,239 | ✓ |
| `erp_interfaces` | 3 | 26 | ✓ |
| `ntrip_client` | 12 | 1,156 | ✓ |
| `rosbag_convert_clean_py` | 1 | 45 | ✓ |
| `ublox` | 0 | 0 | ✓ |
| `ublox_gps` | 2 | 128 | ✓ |
| `ublox_msgs` | 0 | 0 | ✓ |
| `ublox_serialization` | 0 | 0 | ✓ |
| `usb_cam` | 5 | 322 | ✓ |
| `vectornav` | 2 | 73 | ✓ |
| `vectornav_msgs` | 0 | 0 | ✓ |
| `waypoint` | 0 | 0 | ✓ |
| `yolo_ros` | 17 | 2,272 | ✓ |
| `[src root]` (`rosbag2csv.py`) | 1 | 116 | package 밖 최상위 도구 |
| **합계** | **59** | **6,377** | ✓ |

`best_lane_yolov11.pt`도 `[src root]`에 있으나 Python LOC에는 포함되지 않는다.

## 3. 파일 × 11축 교차관심사 매트릭스

범례: **● 중심 구현**, **○ 접점**, `—` 직접 근거 없음. `시뮬` 축은 요청 정의에 따라 venv/conda/ABI bridging/Docker를 포함한다. HRI는 전 범위에서 사용자 상호작용/의도 입력/대화형 interface 구현을 찾지 못했으므로 **해당없음**이며 모든 행을 `—`로 둔다.

| 패키지 | 주요 파일 | 시뮬 | 비전 | 좌표계 | 통신 | 노드설계 | 모션플래닝 | 상태관리 | 안전 | 인프라 | 데이터 | HRI |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| `erp_driver` | `scripts/erp42_imu-gps-wheel-ekf_globalposition.py` | — | — | ● | ● | ○ | ○ | ● | ○ | ○ | ○ | — |
| `erp_driver` | `scripts/global_rotate.py` | — | — | ● | ● | ○ | — | — | — | — | — | — |
| `erp_driver` | `scripts/erp42_pubwaypointscnuservice_pymap3d.py` | — | — | ● | ● | ○ | ○ | ○ | — | — | ● | — |
| `erp_driver` | `scripts/erp42_pathtracking.py` | — | — | ○ | ● | ○ | ● | ● | ● | — | ○ | — |
| `erp_driver` | `scripts/erp42_serial.py` | — | — | — | ● | ○ | — | ● | ● | ○ | ○ | — |
| `erp_driver` | `scripts/erp42_lanedetect.py` | — | ● | — | ○ | ○ | ○ | ○ | ○ | — | — | — |
| `erp_driver` | `scripts/erp42_lanedetect_yolo.py` | — | ● | — | ○ | ○ | ○ | ○ | ● | — | — | — |
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

## 4. 파일별 × 축 상세

각 항목의 `§0-1 기여경계`는 library/upstream 기능과 이 checkout에 놓인 연결·알고리즘 코드를 분리한다. 저자 개인 기여 여부는 git 이력이 없어 확정하지 않는다.

### 4.1 `erp_driver`

#### `scripts/erp42_imu-gps-wheel-ekf_globalposition.py`

- **§0-1 기여경계:** NumPy가 행렬 연산, `pymap3d`가 geodetic→ENU, `tf_transformations`가 quaternion 변환을 제공 / 코드는 ERP42 command·encoder, VectorNav IMU, u-blox GPS를 묶어 custom 선형 EKF와 `/odom_ekf`를 구성한다.
- **좌표계 ●:** quaternion→yaw와 90° 보정(`:149-152`), geodetic→ENU(`:154-164`), `map`→`base_link` odometry와 quaternion 출력(`:218-228`).
- **통신 ●:** command/status/IMU/GPS subscriptions, odometry publisher, `/set_origin` client, 20 Hz timer(`:67-75`); service는 async 호출(`:77-85`).
- **상태관리 ●:** filter state/covariance/origin/정지·최근 센서 상태(`:41-65`), GPS covariance 기반 update와 IMU offset 상태 갱신(`:154-188`).

#### `scripts/global_rotate.py`

- **§0-1 기여경계:** `tf_transformations`가 quaternion 생성·곱셈을 제공 / 코드는 `/odometry/local` orientation에 고정 yaw offset을 적용해 `/odometry/local1`로 재발행한다.
- **좌표계 ●:** 원 quaternion과 `-π/2` Z-axis offset quaternion을 곱해 orientation을 갱신(`:32-52`).
- **통신 ●:** odometry subscription/publisher(`:12-18`)와 변환 메시지 publish(`:57-58`). upstream/downstream 전체 연결은 현재 범위에서 확인되지 않는다.

#### `scripts/erp42_pubwaypointscnuservice_pymap3d.py`

- **§0-1 기여경계:** pandas가 Excel 입력, `pymap3d`가 geodetic→ENU를 제공 / 코드는 `/set_origin`을 기준으로 waypoint를 `map` frame `Path`로 만든다.
- **좌표계 ●:** 위경도 waypoint를 ENU로 변환하고 `map` frame pose로 구성(`:41-62`).
- **통신 ●:** `/waypoints_path1` publisher, `/set_origin` service, 1 s timer(`:19-21`), Path publish(`:68-82`).
- **데이터 ●:** Excel의 `Longitude`/`Latitude` columns를 읽어 waypoint list로 변환(`:40-63`). 단 입력 경로가 `/home/mrlam/...`으로 고정돼 있어 이 checkout에서의 재현은 `[확인 필요]`(`:84-87`).

#### `scripts/erp42_pathtracking.py`

- **§0-1 기여경계:** `tf_transformations`가 odometry quaternion에서 yaw를 추출 / 코드는 waypoint 선택, heading error P 제어, steering alpha blending, ERP42 정지 command를 구현한다.
- **통신 ●:** `/odom_ekf`, `/waypoints_path1`, `/erp42_ctrl_cmd/lidar` subscriptions와 `/erp42_ctrl_cmd` publisher(`:35-57`), command publish(`:189-190`).
- **모션플래닝 ●:** 거리+heading cost의 nearest waypoint 탐색(`:84-120`), lookahead 선택(`:122-136`), heading error 비례 steering과 `alpha=0.65` blending(`:175-190`). PID의 I/D 항은 없다.
- **상태관리 ●:** pose/path/current index/goal/LiDAR state(`:26-32`)와 callback 기반 초기화·진행(`:59-79`, `:138-173`). 별도 priority selector node가 아니라 pathtracking이 `/erp42_ctrl_cmd`를 직접 발행한다.
- **안전 ●:** 마지막 waypoint에서 `stop_robot()` 호출(`:169-173`), speed 0·steer 0·brake 155 발행(`:195-202`). 반면 `brake==2` LiDAR 분기는 정지가 아니라 index 갱신만 수행(`:142-146`)하며, `erp42_main/src/`에는 해당 LiDAR command publisher가 없어 dead branch라는 `mission-cards.md` 판정을 재사용한다.

#### `scripts/erp42_serial.py`

- **§0-1 기여경계:** pyserial이 serial I/O, `struct`가 packet encode/decode를 제공 / 코드는 ROS command/status와 ERP42 byte protocol을 40 Hz로 연결한다.
- **통신 ●:** `/erp42_status` publisher, `/erp42_ctrl_cmd` subscriber, 40 Hz timer(`:24-28`), 18-byte 수신·publish와 command write(`:30-46`).
- **상태관리 ●:** 최신 command packet과 8-bit alive counter를 보존·순환(`:22-28`, `:39-45`).
- **안전 ●:** 수신 packet에서 e-stop/brake를 복원(`:48-59`)하고 송신 packet에 e-stop/brake를 포함(`:61-74`). 이것은 전달 경로 근거이며 fail-safe 동작 검증은 아니다.

#### `scripts/erp42_lanedetect.py`

- **§0-1 기여경계:** OpenCV가 HSV, morphology, CLAHE, Canny, Hough primitives를, scikit-learn이 LDA를 제공 / 코드는 ROI·시간축 buffer·lane fitting·steering 연결을 구성한다.
- **비전 ●:** HSV yellow/white mask와 morphology/CLAHE(`:115-124`), adaptive Canny(`:129-136`), HoughLinesP(`:138-145`). image input은 `/usb_cam_0/image_raw`, output은 `/erp42_ctrl_cmd/lane`(`:324-342`). `camera1/2` launch topic과 mismatch이고 lane command subscriber도 없다는 참고 문서 판정을 유지한다.

#### `scripts/erp42_lanedetect_yolo.py`

- **§0-1 기여경계:** `yolo_msgs`가 detection message contract를 제공 / 코드는 좌·우 lane bbox 중심을 ERP42 steering/stop command로 변환하려 한다.
- **비전 ●:** detection class/bbox 중심으로 좌·우 lane을 선택하고 P steering을 계산(`:29-88`). 그러나 import는 `DetectionArray`인데 subscription은 미정의 `Detection2DArray`를 사용해 node 생성 시 `NameError`가 난다(`:8-9`, `:20-24`). 따라서 완성된 vision 경로로 주장하지 않는다.
- **안전 ●:** lane 미검출 시 stop branch(`:63-73`)와 speed 0·brake 155 command(`:90-97`)가 있으나, 위 startup error와 `/erp42_ctrl_cmd/lane` subscriber 부재 때문에 runtime 도달 여부는 `[확인 필요]`가 아니라 현재 정적 경로상 dead다.

#### `launch/erp42_base.launch.py`

- **§0-1 기여경계:** ROS 2 launch가 process orchestration을 제공 / 코드는 `erp42_serial.py`의 device와 baudrate를 배선한다.
- **인프라 ●:** `erp_driver/erp42_serial.py`를 `/dev/ttyUSB0`, 115200 baud로 launch(`:4-15`). Python node 자체 default `/dev/ttyUSB1`보다 launch parameter가 우선한다.

### 4.2 interfaces, RTK/GPS, 데이터 도구

#### `erp_interfaces/msg/ErpCmdMsg.msg`, `msg/ErpStatusMsg.msg`

- **§0-1 기여경계:** ROSIDL이 language별 type support를 생성 / interface files는 ERP42 command/status wire fields를 정의한다.
- **통신 ●:** command는 e-stop/gear/speed/steer/brake(`ErpCmdMsg.msg:1-5`), status는 여기에 control mode/encoder/alive를 포함(`ErpStatusMsg.msg:1-8`).

#### `ntrip_client/scripts/ntrip_ros_base.py`

- **§0-1 기여경계:** NTRIP client core와 ROS message packages가 caster/RTCM payload 처리를 제공 / wrapper 코드는 ROS params, GPS/NMEA input, RTCM publisher와 reconnect 상태를 묶는다.
- **통신 ●:** RTCM publisher(`:60-78`), NMEA·fix subscriptions와 0.1 s timer(`:94-105`), caster 수신 RTCM publish(`:163-165`).

#### `ntrip_client/launch/ntrip_client_launch.py`

- **§0-1 기여경계:** ROS 2 launch가 argument/parameter 전달을 제공 / launch file은 namespace, caster connection, RTCM type, reconnect/timeout 설정을 node에 주입한다.
- **인프라 ●:** namespace/node/debug arguments(`:10-17`, `:24-27`), node 생성과 connection parameters(`:32-47`), frame/message/reconnect/timeout settings(`:64-79`). 인증정보가 source에 평문으로 들어 있으나 이 문서에는 값을 전재하지 않는다.

#### `[src root]/rosbag2csv.py`

- **§0-1 기여경계:** `rosbag2_py`가 bag storage/reader를, ROSIDL runtime이 dynamic message lookup/deserialization을 제공 / 코드는 topic별 CSV flattening과 상대 timestamp 출력을 수행한다.
- **시뮬 ● (ABI bridging):** clang/libc++ Python extension을 위한 `RTLD_GLOBAL | RTLD_LAZY` opt-in bridge(`:26-33`). 일반 simulation 실행 근거는 아니다.
- **데이터 ●:** rosbag options/SequentialReader(`:36-69`), topic별 message flattening과 CSV 기록(`:71-109`).

#### `rosbag_convert_clean_py/gps_convert.py`

- **§0-1 기여경계:** pandas가 CSV selection/filter/write를 제공 / 코드는 GPS columns의 결측·유효범위 정리 규칙을 적용한다.
- **데이터 ●:** latitude/longitude/altitude typed load(`:9-28`), 결측 제거·좌표 유효범위 filter·CSV 저장(`:30-42`). 입력/출력 절대경로의 현 checkout 재현은 `[확인 필요]`다(`:4-7`).

#### `ublox/package.xml`

- **§0-1 기여경계:** upstream `ublox` metapackage가 u-blox GPS driver/messages/UBX serialization package 집합을 제공 / 이 파일은 세 runtime package dependency를 묶는다(`:3-15`). 11축의 중심 구현보다는 통신·인프라 접점이다.

#### `ublox_gps/src/node.cpp`

- **§0-1 기여경계:** upstream u-blox C++ driver가 receiver protocol과 ROS publishers를 제공 / 이 checkout은 NTRIP `/rtcm` correction을 u-blox node에 연결하는 driver source를 포함한다.
- **통신 ●:** 설정에 따른 NAV/NMEA publishers(`:474-499`)와 RTK correction `/rtcm` subscription(`:501-502`). 실제 GPS fix publisher는 firmware-specific source에 분산되어 있다.
- **데이터 ●:** NAV status/pose/covariance/clock과 NMEA sentence를 parameter에 따라 선택적으로 발행하는 sensor data surface(`:474-499`).

#### `ublox_msgs/msg/NavPVT.msg`

- **§0-1 기여경계:** upstream u-blox message package가 UBX NAV-PVT wire schema를 제공 / 이 file은 시간, fix, RTK carrier phase, 위치·속도·heading fields를 선언한다.
- **데이터 ●:** time/fix validity(`:13-45`), DGPS·power-save·carrier phase state(`:44-62`), LLH accuracy와 NED velocity/heading(`:70-90`).

#### `ublox_serialization/package.xml`

- **§0-1 기여경계:** upstream serialization headers가 ROS message↔u-blox binary format 변환을 제공 / package metadata는 header-only serialization package와 `ament_cmake` build type을 선언한다(`:3-18`). 11축의 중심 구현보다는 통신·인프라·데이터 접점이다.

### 4.3 camera, IMU, waypoint data

#### `usb_cam/launch/camera_config.py`

- **§0-1 기여경계:** Pydantic가 launch config validation을 제공 / 코드는 camera name으로 image/camera_info remapping을 생성한다.
- **인프라 ●:** package config path와 validation(`:30-49`), camera별 image/compressed/camera_info remapping 생성(`:51-65`). camera1/2 launch만 있고 params_3/4를 참조하는 launch가 없다는 참고 문서 결과를 유지한다.

#### `usb_cam/src/usb_cam_node.cpp`

- **§0-1 기여경계:** upstream `usb_cam`/image_transport가 V4L2 capture와 ROS image transport를 제공 / node source는 camera publisher, QoS queue, capture service와 parameters를 구성한다.
- **비전 ●:** `usb_cam` instance와 image/camera_info message를 소유하는 camera node 구성(`:42-53`).
- **통신 ●:** `image_raw` camera publisher with QoS 100과 `set_capture` service(`:37-62`).

#### `vectornav/src/vn_sensor_msgs.cc`

- **§0-1 기여경계:** upstream VectorNav driver/library가 sensor packet access를 제공 / ROS adapter는 time, IMU, GNSS, magnetic, pressure, velocity, pose topics를 발행한다.
- **통신 ●:** VectorNav time/IMU/GNSS 및 보조 sensor publishers(`:39-60`); EKF가 사용하는 것은 `vectornav/imu`(`:49`).
- **데이터 ●:** time reference, compensated/uncompensated IMU, GNSS, magnetic field, temperature, pressure, body velocity와 pose message set을 구성(`:43-60`).

#### `vectornav/launch/vectornav.launch.py`

- **§0-1 기여경계:** ROS 2 launch가 process orchestration을 제공 / launch file은 raw driver와 sensor_msgs adapter에 같은 YAML을 전달한다.
- **인프라 ●:** `vectornav`와 `vn_sensor_msgs` executables, shared `vectornav.yaml` parameters(`:10-29`).

#### `vectornav_msgs/msg/InsGroup.msg`

- **§0-1 기여경계:** ROSIDL이 generated types를 제공 / message file은 VectorNav INS composite fields와 enable bits를 정의한다.
- **데이터 ●:** 필요한 field만 enable해 bandwidth/update rate를 조절한다는 contract(`:1-5`), INS group bit mask와 LLA/ECEF/NED/body fields(`:12-39`).

#### `waypoint/waypoints_real_w2toe4_gps.xls`

- **§0-1 기여경계:** Excel/pandas가 tabular storage/read를 제공 / binary `.xls`는 GPS waypoint dataset이다. text line anchor를 만들 수 없어 matrix는 데이터·좌표계·motion planning **접점(○)**으로만 표시했다. dataset 내용과 실제 사용 run은 `[확인 필요]`다.

### 4.4 `yolo_ros`

#### `Dockerfile`

- **§0-1 기여경계:** Docker와 official ROS image가 isolated runtime/build base를 제공 / file은 source copy, rosdep/pip dependencies, colcon Release build를 재현한다.
- **시뮬 ●:** ROS distro build argument, `ros:${ROS_DISTRO}` base, isolated workspace copy(`:1-7`). 이는 container 환경 근거이며 simulator 자체 근거는 아니다.
- **인프라 ●:** apt/rosdep/pip 설치(`:9-18`)와 `CMAKE_BUILD_TYPE=Release` colcon build(`:20-26`).

#### `yolo_ros/yolo_ros/yolo_node.py`

- **§0-1 기여경계:** Ultralytics/PyTorch가 YOLO inference를, OpenCV/cv_bridge가 image 변환을 제공 / ROS wrapper는 lifecycle, parameters, QoS, services, DetectionArray 변환을 구성한다.
- **비전 ●:** YOLO/YOLOWorld model binding(`:30-35`, `:72`), model load/fuse(`:130-143`), image→prediction with threshold/IoU/image size/half/device controls(`:327-349`).
- **통신 ●:** depth-1 configurable image QoS와 lifecycle detection publisher(`:108-123`), image subscription(`:145-154`).
- **노드설계 ●:** `LifecycleNode`와 configure/activate transitions(`:49-52`, `:74-128`, `:130-159`). Executor/CallbackGroup/Lock/Thread의 project-level custom 설계 근거는 찾지 못했다.
- **상태관리 ●:** enable/model/device/inference parameters(`:54-70`, `:77-112`)와 lifecycle activation/deactivation의 model/service/subscription resource 상태(`:130-179`).

#### `yolo_bringup/launch/yolo.launch.py`

- **§0-1 기여경계:** ROS 2 launch가 namespace/parameters/remapping/conditional node orchestration을 제공 / launch file은 YOLO pipeline의 image topic, QoS, model settings와 optional tracking/3D/debug wiring을 노출한다.
- **통신 ●:** input image topic/QoS arguments(`:137-150`), `yolo` namespace와 image remapping(`:203-207`, `:229-254`). 기본 `/usb_cam_0/image_raw`는 usb_cam launch의 `/camera1/image_raw` 또는 `/camera2/image_raw`와 mismatch다.
- **인프라 ●:** model/device/inference parameters를 node에 전달하고 input remapping을 적용(`:229-254`), optional tracking node를 조건부 구성(`:256-264`).

## 5. 기반 기술 요소 (§3-A, `erp42_main/src/`만)

| 요소 | 코드 근거 | 판정 |
|---|---|---|
| 센서 전력·대역폭 | camera1은 640×480, 30 Hz, YUV422, manual exposure/white balance(`usb_cam/config/params_1.yaml:3-22`). VectorNav composite message는 필요한 field만 enable해 bandwidth를 보존하고 update rate를 높인다고 명시(`vectornav_msgs/msg/InsGroup.msg:1-5`); config는 20 Hz와 binary group mask를 둔다(`vectornav/config/vectornav.yaml:7-37`). | **대역폭 접점 있음.** 전력 budget, USB hub 전력, 카메라 1~2대 동시 실측 traffic은 **코드로 확인 안 됨**. |
| 레이턴시 최적화 | ERP serial 40 Hz timer(`erp_driver/scripts/erp42_serial.py:27`), EKF 20 Hz timer(`erp_driver/scripts/erp42_imu-gps-wheel-ekf_globalposition.py:74`), YOLO image QoS depth 1(`yolo_ros/yolo_ros/yolo_ros/yolo_node.py:115-120`), model fuse attempt(`:139-143`)가 있다. | **접점 있음.** end-to-end latency 측정, deadline, callback scheduling 검증은 **코드로 확인 안 됨**. |
| CPU·GPU 최적화 | YOLO device 기본 `cuda:0`, optional half precision, image size/max detections knobs(`yolo_node.py:55-70`)와 inference parameter 전달(`:335-348`), deactivation 때 CUDA cache 정리(`:161-167`). | **GPU 접점 있음.** `half` 기본은 false이며 CPU profiling, batching, TensorRT/ONNX 최적화는 **코드로 확인 안 됨**. |
| 하드웨어 매뉴얼 기반 환경구성 | ERP serial device/115200(`erp_driver/launch/erp42_base.launch.py:6-13`), u-blox `/dev/ttyACM0`/115200와 Survey-In parameters(`ublox_gps/config/zed_f9p.yaml:1-17`), VectorNav `/dev/ttyUSB0`/115200와 protocol settings(`vectornav/config/vectornav.yaml:1-26`), camera device/framerate/pixel format(`usb_cam/config/params_1.yaml:1-12`). | **환경구성 근거 있음.** 어떤 vendor manual/section을 근거로 값을 정했는지는 **코드로 확인 안 됨**. device path 충돌 여부는 runtime `[확인 필요]`. |
| 언어·런타임 최적화 | C++ sensor drivers(usb_cam/u-blox/VectorNav)와 Python orchestration/perception/control이 공존한다. Docker는 Release colcon build(`yolo_ros/Dockerfile:20-23`), requirements는 `numpy<2`, Ultralytics 8.3.91을 고정(`yolo_ros/requirements.txt:1-4`), `rosbag2csv.py`는 optional ABI bridge를 둔다(`:26-33`). | **런타임 호환성 접점 있음.** 언어 선택의 benchmark나 Python GIL/Executor 최적화 근거는 없어 성능 최적화 주장에는 **접점 낮음**. |

## 6. 자체 대조

### 6.1 §2 패키지 → §3 matrix coverage

- §2에서 확인한 최상위 디렉토리 13개를 §3에 모두 최소 1행 배치했다: `erp_driver`, `erp_interfaces`, `ntrip_client`, `rosbag_convert_clean_py`, `ublox`, `ublox_gps`, `ublox_msgs`, `ublox_serialization`, `usb_cam`, `vectornav`, `vectornav_msgs`, `waypoint`, `yolo_ros`.
- package 밖 top-level `rosbag2csv.py`도 `[src root]` 행으로 별도 포함했다.
- 결과: **PASS (13/13 + src root 1/1)**.

### 6.2 §3 ● → §4 line anchor coverage

- §3의 모든 ● cell을 §4에서 동일 파일의 `file:line`으로 anchor했다.
- binary waypoint `.xls`와 metapackage/serialization metadata처럼 line 기반 중심 구현을 주장할 수 없는 행은 ○만 사용했다.
- dead topic/branch의 음성 근거는 `mission-cards.md`와 `architecture.md`의 이미 검증된 판정을 재사용했고, 현재 범위에서도 `/erp42_ctrl_cmd/lidar` publisher 0건, `/erp42_ctrl_cmd/lane` subscriber 0건, `/erp42_ctrl_cmd/camera` endpoint 0건을 재확인했다.
- 결과: **PASS**.

### 6.3 검증 수준

- **정적 source/config:** PASS — inventory, LOC, mtime, line anchor, topic/dead-path grep을 현재 checkout에서 확인.
- **build/test:** 수행하지 않음 — source 수정이 아닌 문서 작성이며, 이 문서는 동작 성공을 주장하지 않는다.
- **simulation:** 수행하지 않음 — Docker/ABI 항목은 파일 근거만 기록.
- **hardware:** 수행하지 않음 — 실제 sensor 연결, QoS compatibility, topic graph, 차량 actuation은 `[확인 필요]`.
