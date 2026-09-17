# ERP42 Skill Inventory

`PORTFOLIO_MAP.md` §3의 `●`(중심)·`○`(접점) 셀을 §4 상세의 설명과 line anchor로 재배열한 표다. `src/`와 `erp42_main/src/`는 `ws`로 구분하며, 태그는 요청된 11축 순서로 정렬했다.

| 태그 | ws | 구체적 요소 | 코드 근거 | 심화 학습 여지 |
|---|---|---|---|---|
| [시뮬] | src | `fast_gicp` ● Python ABI bridge용 pybind11·CUDA build option | `src/fast_gicp/CMakeLists.txt:4,7,31-34,99-101,114-117` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: Docker/ABI 격리 이론(컨테이너 네임스페이스, ROS distro 간 ABI 비호환 원인) |
| [시뮬] | src | `hdl_global_localization` ● ROS Noetic catkin Docker 격리 build | `src/hdl_global_localization/docker/noetic/Dockerfile:1,9-17` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: Docker/ABI 격리 이론(컨테이너 네임스페이스, ROS distro 간 ABI 비호환 원인) |
| [시뮬] | src | `ndt_omp` ● ROS Foxy colcon Docker 격리 build | `src/ndt_omp/docker/foxy/Dockerfile:1,10-19` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: Docker/ABI 격리 이론(컨테이너 네임스페이스, ROS distro 간 ABI 비호환 원인) |
| [시뮬] | src | `usb_cam` ● Isaac Sim `SimulationApp`·ROS 2 bridge·surface-gripper extension | `src/usb_cam/launch/Pick_Tray_SA.py:3-10,19,22-28` | ERP42 프로젝트와 무관한 별개 Isaac Sim 앱(PORTFOLIO_MAP.md §6-D) — 포트폴리오 학습 로드맵 대상 아님 |
| [시뮬] | erp42_main | `[src root]` ● clang/libc++ extension용 `RTLD_GLOBAL`/`RTLD_LAZY` ABI bridge | `rosbag2csv.py:26-33` | Docker/ABI 격리 이론(컨테이너 네임스페이스, ROS distro 간 ABI 비호환 원인) |
| [시뮬] | erp42_main | `yolo_ros` ● official ROS image 기반 isolated Docker workspace | `Dockerfile:1-7` | Docker/ABI 격리 이론(컨테이너 네임스페이스, ROS distro 간 ABI 비호환 원인) |
| [시뮬] | erp42_main | `yolo_ros` ○ device·model resource를 lifecycle로 구성·해제하는 inference runtime | `yolo_ros/yolo_ros/yolo_node.py:54-70,130-179` | Docker/ABI 격리 이론(컨테이너 네임스페이스, ROS distro 간 ABI 비호환 원인) |
| [비전] | src | `erp_driver` ● HSV mask + Canny/Hough lane 검출 | `src/erp_driver/scripts/0702_erp42_lanedetect.py:58-63,111-117,129-134` | 고전 vision(HSV/Hough) vs 딥러닝(YOLO) 트레이드오프, 카메라 캘리브레이션 |
| [비전] | src | `usb_cam` ○ Isaac Sim scene의 hand marker/target 반영 | `src/usb_cam/launch/Pick_Tray_SA.py:413,442,554-558,606,622,720` | ERP42 프로젝트와 무관한 별개 Isaac Sim 앱(PORTFOLIO_MAP.md §6-D) — 포트폴리오 학습 로드맵 대상 아님 |
| [비전] | src | `usb_cam` ○ camera image/camera_info remapping schema | `src/usb_cam/launch/camera_config.py:34,42-43,54-64` | 고전 vision(HSV/Hough) vs 딥러닝(YOLO) 트레이드오프, 카메라 캘리브레이션 |
| [비전] | src | `Yolo_pt` ○ YOLO binary weight artifact(`best.pt`, `last.pt`) 보관 | `src/Yolo_pt/best.pt` — line anchor 없음(§4 명시) | 고전 vision(HSV/Hough) vs 딥러닝(YOLO) 트레이드오프, 카메라 캘리브레이션 |
| [비전] | erp42_main | `erp_driver` ● HSV·morphology·CLAHE·Canny·Hough lane 검출 | `scripts/erp42_lanedetect.py:115-124,129-136,138-145,324-342` | 고전 vision(HSV/Hough) vs 딥러닝(YOLO) 트레이드오프, 카메라 캘리브레이션 |
| [비전] | erp42_main | `erp_driver` ● detection bbox 기반 lane 선택·P steering(미정의 message type으로 startup 실패) | `scripts/erp42_lanedetect_yolo.py:8-9,20-24,29-88` | 고전 vision(HSV/Hough) vs 딥러닝(YOLO) 트레이드오프, 카메라 캘리브레이션 |
| [비전] | erp42_main | `usb_cam` ○ camera별 image/compressed/camera_info remapping | `launch/camera_config.py:30-49,51-65` | 고전 vision(HSV/Hough) vs 딥러닝(YOLO) 트레이드오프, 카메라 캘리브레이션 |
| [비전] | erp42_main | `usb_cam` ● V4L2 camera image/camera_info node 구성 | `src/usb_cam_node.cpp:42-53` | 고전 vision(HSV/Hough) vs 딥러닝(YOLO) 트레이드오프, 카메라 캘리브레이션 |
| [비전] | erp42_main | `yolo_ros` ● YOLO/YOLOWorld load·fuse·parameterized inference | `yolo_ros/yolo_ros/yolo_node.py:30-35,72,130-143,327-349` | 고전 vision(HSV/Hough) vs 딥러닝(YOLO) 트레이드오프, 카메라 캘리브레이션 |
| [비전] | erp42_main | `yolo_ros` ○ YOLO input image remapping과 optional tracking/3D/debug wiring | `yolo_bringup/launch/yolo.launch.py:137-150,203-207,229-254,256-264` | 고전 vision(HSV/Hough) vs 딥러닝(YOLO) 트레이드오프, 카메라 캘리브레이션 |
| [좌표계] | src | `ebimu_pkg` ○ EBIMU serial line을 ROS message로 변환 | `src/ebimu_pkg/ebimu_pkg/ebimu_publisher.py:13-16,30,37` | 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `erp_driver` ● quaternion↔Euler + geodetic→ENU 선형 EKF | `src/erp_driver/scripts/1024_EBIMU_EKF.py:8,98,126,133,179` | 칼만필터 이론(선형 EKF vs UKF sigma-point) — 현재 UKF가 아닌 선형 EKF임을 mission-cards.md에서 이미 확인, sigma-point 확장을 공부하면 보고서 주장과 코드를 맞출 수 있음 |
| [좌표계] | src | `erp_driver` ○ odometry yaw 기반 heading error 계산 | `src/erp_driver/scripts/erp42_pathtracking.py:122,175` | 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `erp_driver` ○ GPS waypoint geodetic→ENU 변환 | `src/erp_driver/scripts/erp42_pubwaypointscnuservice_pymap3d.py:41,51` | 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `hdl_global_localization` ○ localization용 legacy Noetic container recipe | `src/hdl_global_localization/docker/noetic/Dockerfile:1,9-17` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `hdl_localization` ○ global map과 localization component 배선 | `src/hdl_localization/launch/hdl_localization.launch.py:36-38,63,70-91,99-100` | 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `ndt_omp` ○ NDT registration용 Foxy container recipe | `src/ndt_omp/docker/foxy/Dockerfile:1,10-19` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `ntrip_client` ○ NMEA·u-blox fix 입력을 NTRIP correction stream과 연결 | `src/ntrip_client/scripts/ntrip_ros_base.py:61-78,100-104,163-182` | 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `pcl_ros` ● timestamp 기반 TF lookup·point cloud target-frame 변환 | `src/pcl_ros/src/transforms.cpp:65-80,93-95,120-126,234-255` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `robot_localization` ○ parameterized generic EKF node wiring | `src/robot_localization/launch/ekf.launch.py:18,28-33` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `robot_localization` ○ bag1 pose validation용 EKF/tester wiring | `src/robot_localization/test/test_ekf_localization_node_bag1.launch.py:25-34,42-60` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `ublox` ○ GNSS driver parameter wiring | `src/ublox/ublox_gps/launch/ublox_gps_node-launch.py:47-56` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `usb_cam` ○ Isaac Sim hand target를 M0609 scene에 반영 | `src/usb_cam/launch/Pick_Tray_SA.py:413,442,554-558,606,622,720` | ERP42 프로젝트와 무관한 별개 Isaac Sim 앱(PORTFOLIO_MAP.md §6-D) — 포트폴리오 학습 로드맵 대상 아님 |
| [좌표계] | src | `vectornav` ○ raw driver와 sensor_msgs adapter의 shared config | `src/vectornav/vectornav/launch/vectornav.launch.py:10-23,28-29` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | src | `velodyne` ○ `frame_id=velodyne`인 VLP16 pointcloud pipeline | `src/velodyne/velodyne/launch/velodyne-all-nodes-VLP16-launch.py:43-70`; `src/velodyne/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml:3,10-12` | LiDAR 드라이버 설정(rpm/해상도 트레이드오프)은 vendor 담당이나, point cloud 활용은 본인 몫 — dead branch를 실제로 연결하는 것이 다음 학습 단계 |
| [좌표계] | erp42_main | `erp_driver` ● quaternion→yaw+90° 보정, geodetic→ENU, `map`→`base_link` odometry | `scripts/erp42_imu-gps-wheel-ekf_globalposition.py:149-152,154-164,218-228` | 칼만필터 이론(공분산 가중 보정의 수학적 근거) — 팀 동료 담당 구간이라 코드 읽기 수준의 이해 |
| [좌표계] | erp42_main | `erp_driver` ● odometry orientation에 `-π/2` yaw offset 적용 | `scripts/global_rotate.py:32-52` | 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | erp42_main | `erp_driver` ● GPS waypoint를 ENU·`map` frame Path로 변환 | `scripts/erp42_pubwaypointscnuservice_pymap3d.py:41-62` | 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | erp42_main | `erp_driver` ○ odometry quaternion에서 yaw를 추출해 heading error 계산 | `scripts/erp42_pathtracking.py:84-120,122-136,175-190` | P control 이론과 pure-pursuit/Stanley 비교 — 곡률 기반 조향법 학습 여지 |
| [좌표계] | erp42_main | `ntrip_client` ○ NMEA·GPS fix 기반 RTK correction 연결 | `scripts/ntrip_ros_base.py:60-78,94-105,163-165` | 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | erp42_main | `ntrip_client` ○ correction node의 frame/message 설정 주입 | `launch/ntrip_client_launch.py:64-79` | 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | erp42_main | `rosbag_convert_clean_py` ○ GPS latitude/longitude/altitude column 정리 | `gps_convert.py:9-28,30-42` | 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | erp42_main | `ublox_gps` ○ NAV pose/covariance와 NMEA sensor surface | `src/node.cpp:474-502` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | erp42_main | `ublox_msgs` ○ UBX NAV-PVT LLH·NED·heading schema | `msg/NavPVT.msg:70-90` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | erp42_main | `vectornav` ○ GNSS·velocity·pose sensor topic adapter | `src/vn_sensor_msgs.cc:39-60` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | erp42_main | `vectornav` ○ raw driver와 sensor_msgs adapter의 shared YAML wiring | `launch/vectornav.launch.py:10-29` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | erp42_main | `vectornav_msgs` ○ INS LLA/ECEF/NED/body field contract | `msg/InsGroup.msg:12-39` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 좌표변환 수학(quaternion, geodetic↔ENU 변환 유도) — 대부분 pymap3d/tf_transformations가 대신 계산 |
| [좌표계] | erp42_main | `waypoint` ○ GPS waypoint dataset | `waypoints_real_w2toe4_gps.xls` — text line anchor 없음(§4 명시) | binary 데이터 — 좌표계 학습 여지는 이를 소비하는 pymap3d 변환 코드 쪽에서 다룸 |
| [통신] | src | `cluster_bev` ● SensorDataQoS PointCloud2 subscribe/publish | `src/cluster_bev/src/cluster_bev_node.cpp:26-27,43-49` | point cloud 다운샘플링 이론(voxel grid)과 RANSAC 평면 적합 수학 — PCL이 대신 계산하는 부분을 직접 유도해보면 심화 |
| [통신] | src | `ebimu_pkg` ● pyserial 115200 baud→`ebimu_data` publisher | `src/ebimu_pkg/ebimu_pkg/ebimu_publisher.py:13-16,30,37` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `erp_driver` ○ EBIMU·GPS·wheel subscriptions→`/odom_ekf` output | `src/erp_driver/scripts/1024_EBIMU_EKF.py:61-64,121,141` | 칼만필터 이론(선형 EKF vs UKF sigma-point) — 현재 UKF가 아닌 선형 EKF임을 mission-cards.md에서 이미 확인, sigma-point 확장을 공부하면 보고서 주장과 코드를 맞출 수 있음 |
| [통신] | src | `erp_driver` ○ odometry·Path·LiDAR command 결합 | `src/erp_driver/scripts/erp42_pathtracking.py:142-145,170-171,195-201` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `erp_driver` ○ lane/path command timer selector | `src/erp_driver/scripts/0702_erp42_controller.py:16-24,37-55` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `erp_driver` ○ `/usb_cam_0/image_raw`→`/erp42_ctrl_cmd/lane` | `src/erp_driver/scripts/0702_erp42_lanedetect.py:366-368` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `erp_driver` ○ `/velodyne_points`→`/erp42_ctrl_cmd/lidar` | `src/erp_driver/scripts/0822_Obstacle_3d.py:14,17` | 센서 데이터 흐름 설계(ROI 필터링→판단 파이프라인) |
| [통신] | src | `erp_driver` ● `/erp42_ctrl_cmd`↔serial↔`/erp42_status` 40 Hz bridge | `src/erp_driver/scripts/erp42_serial.py:24-27,31,42-44,56-57` | 시리얼 프로토콜 설계(체크섬, 프레이밍) — 현재 패킷 포맷의 오류검출 여부 확인 필요 |
| [통신] | src | `erp_driver` ○ Excel waypoint→`/waypoints_path1` Path publish | `src/erp_driver/scripts/erp42_pubwaypointscnuservice_pymap3d.py:19,41,51` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `erp_interfaces` ● ERP42 command/status wire fields | `src/erp_interfaces/msg/ErpCmdMsg.msg:1-5`; `src/erp_interfaces/msg/ErpStatusMsg.msg:1-8` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `hdl_localization` ○ GlobalmapServer·HdlLocalization component wiring | `src/hdl_localization/launch/hdl_localization.launch.py:36-38,63,70-91,99-100` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `ntrip_client` ● NMEA/fix subscriptions→RTCM timer publisher | `src/ntrip_client/scripts/ntrip_ros_base.py:61-78,100-104,163-182` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `pcl_clustering_py` ● PointCloud2 subscribe→clustered cloud publish | `src/pcl_clustering_py/pcl_clustering_py/euclidean_cluster_node.py:32-37,41-44,56-84,93-123` | Euclidean clustering의 kd-tree 최근접 탐색 복잡도, DBSCAN 등 대안 클러스터링 비교 |
| [통신] | src | `pcl_ros` ○ TF lookup 후 transformed point cloud output | `src/pcl_ros/src/transforms.cpp:65-80,93-95` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `robot_localization` ○ EKF executable·parameter wiring | `src/robot_localization/launch/ekf.launch.py:18,28-33` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `ublox` ○ u-blox driver process·parameter wiring | `src/ublox/ublox_gps/launch/ublox_gps_node-launch.py:47-56` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `usb_cam` ○ camera image/camera_info remapping | `src/usb_cam/launch/camera_config.py:34,42-43,54-64` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `usb_cam` ○ `/hand_raw`·`/hand_xyz`·`/hand_mode` subscriptions | `src/usb_cam/launch/Pick_Tray_SA.py:251-266` | ERP42 프로젝트와 무관한 별개 Isaac Sim 앱(PORTFOLIO_MAP.md §6-D) — 포트폴리오 학습 로드맵 대상 아님 |
| [통신] | src | `vectornav` ○ raw driver + sensor_msgs conversion node launch | `src/vectornav/vectornav/launch/vectornav.launch.py:10-23,28-29` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | src | `velodyne` ○ VLP16 driver→pointcloud transform→laserscan pipeline | `src/velodyne/velodyne/launch/velodyne-all-nodes-VLP16-launch.py:43-70` | LiDAR 드라이버 설정(rpm/해상도 트레이드오프)은 vendor 담당이나, point cloud 활용은 본인 몫 — dead branch를 실제로 연결하는 것이 다음 학습 단계 |
| [통신] | erp42_main | `erp_driver` ● command/status/IMU/GPS subscriptions + odometry publisher + async service | `scripts/erp42_imu-gps-wheel-ekf_globalposition.py:67-85` | 칼만필터 이론(공분산 가중 보정의 수학적 근거) — 팀 동료 담당 구간이라 코드 읽기 수준의 이해 |
| [통신] | erp42_main | `erp_driver` ● odometry orientation 변환·재발행 | `scripts/global_rotate.py:12-18,57-58` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `erp_driver` ● `/set_origin` service + waypoint Path publisher | `scripts/erp42_pubwaypointscnuservice_pymap3d.py:19-21,68-82` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `erp_driver` ● odom/path/LiDAR subscriptions→직접 ERP42 command publish | `scripts/erp42_pathtracking.py:35-57,189-190` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `erp_driver` ● ROS command/status↔ERP42 serial 40 Hz bridge | `scripts/erp42_serial.py:24-28,30-46` | 시리얼 프로토콜 설계와 40Hz 주기의 실시간성 보장 기법 |
| [통신] | erp42_main | `erp_driver` ○ camera image input→lane command output | `scripts/erp42_lanedetect.py:324-342` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `erp_driver` ○ detection message subscription 시도(NameError로 기동 실패) | `scripts/erp42_lanedetect_yolo.py:8-9,20-24` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `erp_driver` ○ serial node device·baudrate launch wiring | `launch/erp42_base.launch.py:4-15` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `erp_interfaces` ● ERP42 command/status ROSIDL contract | `msg/ErpCmdMsg.msg:1-5`; `msg/ErpStatusMsg.msg:1-8` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `ntrip_client` ● NMEA/fix subscriptions→RTCM publisher | `scripts/ntrip_ros_base.py:60-78,94-105,163-165` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `ntrip_client` ○ caster·RTCM connection parameters launch wiring | `launch/ntrip_client_launch.py:10-17,24-27,32-47,64-79` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `ublox` ○ u-blox GPS/messages/serialization runtime dependency bundle | `package.xml:3-15` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `ublox_gps` ● NAV/NMEA publishers + `/rtcm` correction subscription | `src/node.cpp:474-502` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `ublox_msgs` ○ UBX NAV-PVT wire schema | `msg/NavPVT.msg:13-45,44-62,70-90` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `ublox_serialization` ○ ROS message↔u-blox binary serialization package | `package.xml:3-18` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `usb_cam` ○ camera topic remapping generation | `launch/camera_config.py:30-49,51-65` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `usb_cam` ● `image_raw` publisher + `set_capture` service | `src/usb_cam_node.cpp:37-62` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `vectornav` ● time/IMU/GNSS·보조 sensor publishers | `src/vn_sensor_msgs.cc:39-60` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `vectornav` ○ two-node launch orchestration | `launch/vectornav.launch.py:10-29` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `vectornav_msgs` ○ INS composite ROSIDL contract | `msg/InsGroup.msg:1-5,12-39` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `yolo_ros` ● configurable depth-1 image QoS + lifecycle publisher/subscription | `yolo_ros/yolo_ros/yolo_node.py:108-123,145-154` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [통신] | erp42_main | `yolo_ros` ● input image topic/QoS arguments + remapping | `yolo_bringup/launch/yolo.launch.py:137-150,203-207,229-254` | ROS2 QoS 프로파일 이론(Reliable/BestEffort, durability)과 이 프로젝트의 topic mismatch 사례 비교 |
| [노드설계] | src | `erp_driver` ○ 40 Hz timer 기반 serial bridge | `src/erp_driver/scripts/erp42_serial.py:24-27,42-44` | 실시간 주기 보장(RT 스케줄링, jitter 관리) — 40Hz timer의 지터/지연 실측이 없음 |
| [노드설계] | src | `fast_gicp` ● point-cloud registration thread 수의 Python binding 노출 | `src/fast_gicp/CMakeLists.txt:99-101`; `src/fast_gicp/src/python/main.cpp:100,107,186` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `erp_driver` ○ 20 Hz timer + async `/set_origin` service client | `scripts/erp42_imu-gps-wheel-ekf_globalposition.py:67-85` | 칼만필터 이론(공분산 가중 보정의 수학적 근거) — 팀 동료 담당 구간이라 코드 읽기 수준의 이해 |
| [노드설계] | erp42_main | `erp_driver` ○ odometry transform subscriber/publisher node | `scripts/global_rotate.py:12-18,57-58` | ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `erp_driver` ○ service·publisher·1 s timer waypoint node | `scripts/erp42_pubwaypointscnuservice_pymap3d.py:19-21,68-82` | ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `erp_driver` ○ callback 기반 waypoint navigator | `scripts/erp42_pathtracking.py:26-32,35-79,138-173` | ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `erp_driver` ○ 40 Hz serial timer node | `scripts/erp42_serial.py:24-28,30-46` | 시리얼 프로토콜 설계와 40Hz 주기의 실시간성 보장 기법 |
| [노드설계] | erp42_main | `erp_driver` ○ ROI·시간축 buffer·lane fitting node | `scripts/erp42_lanedetect.py:115-124,129-136,138-145,324-342` | ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `erp_driver` ○ detection subscription node 구조(미정의 type으로 startup 실패) | `scripts/erp42_lanedetect_yolo.py:8-9,20-24` | ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `ntrip_client` ○ 0.1 s timer + reconnect state wrapper | `scripts/ntrip_ros_base.py:60-78,94-105,163-165` | ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `ublox_gps` ○ firmware 설정별 publisher·RTCM subscriber 구성 | `src/node.cpp:474-502` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `usb_cam` ○ camera publisher·QoS queue·capture service node | `src/usb_cam_node.cpp:37-62` | ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `vectornav` ○ raw packet→복수 ROS sensor publisher adapter | `src/vn_sensor_msgs.cc:39-60` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `yolo_ros` ● `LifecycleNode` configure/activate transitions | `yolo_ros/yolo_ros/yolo_node.py:49-52,74-128,130-159` | ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [노드설계] | erp42_main | `yolo_ros` ○ conditional YOLO pipeline node orchestration | `yolo_bringup/launch/yolo.launch.py:203-207,229-254,256-264` | ROS2 Executor/CallbackGroup 동시성 모델 — 이 구간엔 미적용, 학습 여지 큼 |
| [모션플래닝] | src | `erp_driver` ● nearest/lookahead waypoint + heading-error P control + alpha blending | `src/erp_driver/scripts/erp42_pathtracking.py:84,122,128,161,175,184-186` | P control 이론(정상상태 오차, 이득 튜닝)과 pure-pursuit/Stanley 등 곡률 기반 조향법 비교 — 지금은 heading error P + 저역통과뿐, 곡률/속도 연동 없음 |
| [모션플래닝] | src | `erp_driver` ○ lane candidate 기반 steering command | `src/erp_driver/scripts/0702_erp42_lanedetect.py:58-63,111-117,129-134,366-368` | 경로추종 알고리즘 비교(pure-pursuit, Stanley, MPC) — 현재 P제어+블렌딩뿐 |
| [모션플래닝] | src | `erp_driver` ○ ROI obstacle 판단 기반 brake command experiment | `src/erp_driver/scripts/0822_Obstacle_3d.py:14,17,69,117` | 장애물 회피 알고리즘 비교(potential field, DWA) — 지금은 ROI 임계값 기반 단순 정지/서행뿐 |
| [모션플래닝] | src | `erp_driver` ○ waypoint list를 ENU `Path`로 구성 | `src/erp_driver/scripts/erp42_pubwaypointscnuservice_pymap3d.py:19,41,51` | 경로추종 알고리즘 비교(pure-pursuit, Stanley, MPC) — 현재 P제어+블렌딩뿐 |
| [모션플래닝] | src | `usb_cam` ○ hand target/mode를 simulation loop의 M0609 target에 반영 | `src/usb_cam/launch/Pick_Tray_SA.py:413,442,554-558,606,622,720` | ERP42 프로젝트와 무관한 별개 Isaac Sim 앱(PORTFOLIO_MAP.md §6-D) — 포트폴리오 학습 로드맵 대상 아님 |
| [모션플래닝] | erp42_main | `erp_driver` ○ fused odometry를 waypoint navigation에 제공 | `scripts/erp42_imu-gps-wheel-ekf_globalposition.py:218-228` | 칼만필터 이론(공분산 가중 보정의 수학적 근거) — 팀 동료 담당 구간이라 코드 읽기 수준의 이해 |
| [모션플래닝] | erp42_main | `erp_driver` ○ waypoint Path 생성 | `scripts/erp42_pubwaypointscnuservice_pymap3d.py:41-62,68-82` | 경로추종 알고리즘 비교(pure-pursuit, Stanley, MPC) — 현재 P제어+블렌딩뿐 |
| [모션플래닝] | erp42_main | `erp_driver` ● nearest/lookahead + heading-error P control + alpha blending | `scripts/erp42_pathtracking.py:84-120,122-136,175-190` | P control 이론과 pure-pursuit/Stanley 비교 — 곡률 기반 조향법 학습 여지 |
| [모션플래닝] | erp42_main | `erp_driver` ○ lane fitting 결과를 steering에 연결 | `scripts/erp42_lanedetect.py:115-124,129-136,138-145,324-342` | 경로추종 알고리즘 비교(pure-pursuit, Stanley, MPC) — 현재 P제어+블렌딩뿐 |
| [모션플래닝] | erp42_main | `erp_driver` ○ bbox 중심 기반 P steering(기동 실패 경로) | `scripts/erp42_lanedetect_yolo.py:20-24,29-88` | 경로추종 알고리즘 비교(pure-pursuit, Stanley, MPC) — 현재 P제어+블렌딩뿐 |
| [모션플래닝] | erp42_main | `waypoint` ○ GPS waypoint dataset | `waypoints_real_w2toe4_gps.xls` — text line anchor 없음(§4 명시) | binary 데이터 — 좌표계 학습 여지는 이를 소비하는 pymap3d 변환 코드 쪽에서 다룸 |
| [상태관리] | src | `erp_driver` ○ EKF filter state/covariance·sensor state 갱신 | `src/erp_driver/scripts/1024_EBIMU_EKF.py:61-64,121,133,141` | 칼만필터 이론(선형 EKF vs UKF sigma-point) — 현재 UKF가 아닌 선형 EKF임을 mission-cards.md에서 이미 확인, sigma-point 확장을 공부하면 보고서 주장과 코드를 맞출 수 있음 |
| [상태관리] | src | `erp_driver` ○ current waypoint index·goal 진행 상태 | `src/erp_driver/scripts/erp42_pathtracking.py:84,128,142-145,161,170-171` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | src | `erp_driver` ● lane 우선 selector + freshness timeout | `src/erp_driver/scripts/0702_erp42_controller.py:37-46` | 상태머신 설계 이론(전이 매트릭스, 가드 조건 명시화) — 지금은 if/elif 2단계뿐, FSM 라이브러리(SMACH 등) 비교 학습 여지 |
| [상태관리] | src | `erp_driver` ○ 최신 serial command·status 처리 | `src/erp_driver/scripts/erp42_serial.py:24-31,42-44,56-57` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | src | `erp_interfaces` ○ e-stop·gear·speed·steer·brake/encoder state contract | `src/erp_interfaces/msg/ErpCmdMsg.msg:1-5`; `src/erp_interfaces/msg/ErpStatusMsg.msg:1-8` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | src | `usb_cam` ○ `HOME`·`TRACKING` mode 반영 | `src/usb_cam/launch/Pick_Tray_SA.py:251-266,554-558,606,622,720` | ERP42 프로젝트와 무관한 별개 Isaac Sim 앱(PORTFOLIO_MAP.md §6-D) — 포트폴리오 학습 로드맵 대상 아님 |
| [상태관리] | erp42_main | `erp_driver` ● filter state/covariance/origin·sensor state + GPS covariance update | `scripts/erp42_imu-gps-wheel-ekf_globalposition.py:41-65,154-188` | 칼만필터 이론(공분산 가중 보정의 수학적 근거) — 팀 동료 담당 구간이라 코드 읽기 수준의 이해 |
| [상태관리] | erp42_main | `erp_driver` ○ origin·waypoint list의 service/timer lifecycle | `scripts/erp42_pubwaypointscnuservice_pymap3d.py:19-21,40-82` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `erp_driver` ● pose/path/index/goal/LiDAR callback state | `scripts/erp42_pathtracking.py:26-32,59-79,138-173` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `erp_driver` ● latest command packet + 8-bit alive counter | `scripts/erp42_serial.py:22-28,39-45` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `erp_driver` ○ 시간축 lane buffer·lane fitting state | `scripts/erp42_lanedetect.py:115-124,129-136,138-145,324-342` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `erp_driver` ○ lane detection·stop branch state(기동 실패 경로) | `scripts/erp42_lanedetect_yolo.py:20-24,29-97` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `erp_interfaces` ○ command/status state fields | `msg/ErpCmdMsg.msg:1-5`; `msg/ErpStatusMsg.msg:1-8` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `ntrip_client` ○ GPS/NMEA input·reconnect state | `scripts/ntrip_ros_base.py:60-78,94-105,163-165` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `ntrip_client` ○ reconnect/timeout settings | `launch/ntrip_client_launch.py:64-79` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `ublox_gps` ○ firmware setting에 따른 NAV/NMEA publisher state | `src/node.cpp:474-502` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `ublox_msgs` ○ fix validity·carrier phase state schema | `msg/NavPVT.msg:13-62` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `usb_cam` ○ capture service와 camera publisher state | `src/usb_cam_node.cpp:37-62` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `vectornav` ○ enabled sensor packet별 publisher state | `src/vn_sensor_msgs.cc:39-60` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `vectornav_msgs` ○ enable bits로 INS field·bandwidth state 선택 | `msg/InsGroup.msg:1-5,12-39` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `yolo_ros` ● enable/model/device/inference params + lifecycle resource state | `yolo_ros/yolo_ros/yolo_node.py:54-70,77-112,130-179` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [상태관리] | erp42_main | `yolo_ros` ○ optional tracking node 조건부 구성 | `yolo_bringup/launch/yolo.launch.py:229-254,256-264` | 명시적 FSM 설계(전이 매트릭스, 가드) — 현재 if/elif 수준 |
| [안전] | src | `erp_driver` ● final waypoint `brake=155` stop + dead LiDAR branch | `src/erp_driver/scripts/erp42_pathtracking.py:142-145,170-171,195-201` | fail-safe/워치독 설계, ISO 26262류 안전 등급 개념 |
| [안전] | src | `erp_driver` ● invalid inputs fallback `brake=155` full brake | `src/erp_driver/scripts/0702_erp42_controller.py:48-58` | fail-safe 설계 패턴(다중 방어선, 워치독) — 지금은 단일 fallback뿐 |
| [안전] | src | `erp_driver` ● obstacle branch `brake=200`/clear `brake=0` | `src/erp_driver/scripts/0822_Obstacle_3d.py:14,17,69,117` | 장애물 회피 알고리즘 비교(potential field, DWA) — 지금은 ROI 임계값 기반 단순 정지/서행뿐 |
| [안전] | src | `erp_driver` ○ serial packet의 brake field encode/decode | `src/erp_driver/scripts/erp42_serial.py:31,56-57` | 안전 인터록 설계(e-stop/brake 신호의 이중화·워치독) |
| [안전] | src | `erp_interfaces` ○ `e_stop`·`brake` command/status fields | `src/erp_interfaces/msg/ErpCmdMsg.msg:1-5`; `src/erp_interfaces/msg/ErpStatusMsg.msg:1-8` | fail-safe/워치독 설계, ISO 26262류 안전 등급 개념 |
| [안전] | src | `usb_cam` ○ simulation mode/target control 접점(ERP42와 별도 앱) | `src/usb_cam/launch/Pick_Tray_SA.py:251-266,554-558,606,622,720` | ERP42 프로젝트와 무관한 별개 Isaac Sim 앱(PORTFOLIO_MAP.md §6-D) — 포트폴리오 학습 로드맵 대상 아님 |
| [안전] | erp42_main | `erp_driver` ○ 정지·최근 sensor/filter state 보존 | `scripts/erp42_imu-gps-wheel-ekf_globalposition.py:41-65,154-188` | 칼만필터 이론(공분산 가중 보정의 수학적 근거) — 팀 동료 담당 구간이라 코드 읽기 수준의 이해 |
| [안전] | erp42_main | `erp_driver` ● final waypoint speed 0·steer 0·brake 155 | `scripts/erp42_pathtracking.py:142-146,169-173,195-202` | fail-safe/워치독 설계, ISO 26262류 안전 등급 개념 |
| [안전] | erp42_main | `erp_driver` ● e-stop/brake serial packet 전달 경로 | `scripts/erp42_serial.py:48-74` | 안전 인터록 설계(e-stop/brake 신호의 이중화·워치독) |
| [안전] | erp42_main | `erp_driver` ○ lane command path의 steering 접점 | `scripts/erp42_lanedetect.py:115-124,129-136,138-145,324-342` | fail-safe/워치독 설계, ISO 26262류 안전 등급 개념 |
| [안전] | erp42_main | `erp_driver` ● lane 미검출 speed 0·brake 155(기동 실패로 dead) | `scripts/erp42_lanedetect_yolo.py:8-9,20-24,63-73,90-97` | fail-safe/워치독 설계, ISO 26262류 안전 등급 개념 |
| [안전] | erp42_main | `erp_driver` ○ serial node device·baudrate launch configuration | `launch/erp42_base.launch.py:4-15` | fail-safe/워치독 설계, ISO 26262류 안전 등급 개념 |
| [안전] | erp42_main | `erp_interfaces` ○ e-stop·brake wire contract | `msg/ErpCmdMsg.msg:1-5`; `msg/ErpStatusMsg.msg:1-8` | fail-safe/워치독 설계, ISO 26262류 안전 등급 개념 |
| [인프라] | src | `cluster_bev` ○ parameterized SensorDataQoS cloud node wiring | `src/cluster_bev/src/cluster_bev_node.cpp:26-27,43-49` | point cloud 다운샘플링 이론(voxel grid)과 RANSAC 평면 적합 수학 — PCL이 대신 계산하는 부분을 직접 유도해보면 심화 |
| [인프라] | src | `fast_gicp` ○ CUDA·pybind11 conditional build configuration | `src/fast_gicp/CMakeLists.txt:4,7,31-34,99-101,114-117` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | src | `hdl_global_localization` ○ Noetic catkin Docker build recipe | `src/hdl_global_localization/docker/noetic/Dockerfile:1,9-17` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | src | `hdl_localization` ● component container·parameter·global-map wiring | `src/hdl_localization/launch/hdl_localization.launch.py:36-38,63,70-91,99-100` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | src | `ndt_omp` ○ Foxy colcon Docker build recipe | `src/ndt_omp/docker/foxy/Dockerfile:1,10-19` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | src | `ntrip_client` ○ NMEA/GPS·RTCM type adapter timer wiring | `src/ntrip_client/scripts/ntrip_ros_base.py:61-78,100-104,163-182` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | src | `pcl_clustering_py` ○ Python-PCL conversion·RANSAC·clustering node pipeline | `src/pcl_clustering_py/pcl_clustering_py/euclidean_cluster_node.py:32-37,41-44,56-84,93-123` | Euclidean clustering의 kd-tree 최근접 탐색 복잡도, DBSCAN 등 대안 클러스터링 비교 |
| [인프라] | src | `robot_localization` ● EKF executable + YAML launch wiring | `src/robot_localization/launch/ekf.launch.py:18,28-33` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | src | `robot_localization` ○ bag1 EKF/tester launch harness | `src/robot_localization/test/test_ekf_localization_node_bag1.launch.py:25-34,42-60` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | src | `ublox` ● driver parameter wiring + exit shutdown policy | `src/ublox/ublox_gps/launch/ublox_gps_node-launch.py:47-56` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | src | `usb_cam` ● Pydantic camera remapping·namespace validation | `src/usb_cam/launch/camera_config.py:34,42-43,54-64` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | src | `vectornav` ● raw driver + sensor_msgs node shared-YAML launch | `src/vectornav/vectornav/launch/vectornav.launch.py:10-23,28-29` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | src | `velodyne` ● VLP16 driver-transform-laserscan launch pipeline | `src/velodyne/velodyne/launch/velodyne-all-nodes-VLP16-launch.py:43-70` | LiDAR 드라이버 설정(rpm/해상도 트레이드오프)은 vendor 담당이나, point cloud 활용은 본인 몫 — dead branch를 실제로 연결하는 것이 다음 학습 단계 |
| [인프라] | erp42_main | `erp_driver` ○ EKF 20 Hz timer·service/pub-sub wiring | `scripts/erp42_imu-gps-wheel-ekf_globalposition.py:67-85` | 칼만필터 이론(공분산 가중 보정의 수학적 근거) — 팀 동료 담당 구간이라 코드 읽기 수준의 이해 |
| [인프라] | erp42_main | `erp_driver` ○ 40 Hz serial protocol bridge | `scripts/erp42_serial.py:24-28,30-46` | 시리얼 프로토콜 설계와 40Hz 주기의 실시간성 보장 기법 |
| [인프라] | erp42_main | `erp_driver` ● serial executable device·baudrate launch wiring | `launch/erp42_base.launch.py:4-15` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `erp_interfaces` ○ ROSIDL command/status interface package | `msg/ErpCmdMsg.msg:1-5`; `msg/ErpStatusMsg.msg:1-8` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `ntrip_client` ○ ROS params·timer·caster wrapper | `scripts/ntrip_ros_base.py:60-78,94-105,163-165` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `ntrip_client` ● namespace·connection·reconnect/timeout launch configuration | `launch/ntrip_client_launch.py:10-17,24-27,32-47,64-79` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `ublox` ○ GPS driver/messages/serialization metapackage | `package.xml:3-15` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `ublox_gps` ○ firmware-configured GPS driver source | `src/node.cpp:474-502` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `ublox_serialization` ○ header-only serialization + `ament_cmake` metadata | `package.xml:3-18` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `usb_cam` ● Pydantic config validation + camera remapping | `launch/camera_config.py:30-49,51-65` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `usb_cam` ○ image_transport camera publisher·capture service | `src/usb_cam_node.cpp:37-62` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `vectornav` ○ multi-sensor ROS adapter | `src/vn_sensor_msgs.cc:39-60` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `vectornav` ● two executables + shared YAML launch wiring | `launch/vectornav.launch.py:10-29` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `yolo_ros` ● dependency install + Release colcon Docker build | `Dockerfile:9-18,20-26` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `yolo_ros` ○ lifecycle·QoS·service ROS wrapper | `yolo_ros/yolo_ros/yolo_node.py:49-52,74-128,130-179` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [인프라] | erp42_main | `yolo_ros` ● model/device params·remapping·optional tracking launch wiring | `yolo_bringup/launch/yolo.launch.py:229-254,256-264` | launch 시스템 설계(파라미터 오버라이드, composable node) |
| [데이터] | src | `cluster_bev` ○ PointCloud2 voxel·RANSAC·clustering output | `src/cluster_bev/src/cluster_bev_node.cpp:26-27,43-49` | point cloud 다운샘플링 이론(voxel grid)과 RANSAC 평면 적합 수학 — PCL이 대신 계산하는 부분을 직접 유도해보면 심화 |
| [데이터] | src | `ebimu_pkg` ○ serial `readline()`→String sensor data | `src/ebimu_pkg/ebimu_pkg/ebimu_publisher.py:13-16,30,37` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | src | `erp_driver` ○ GPS/IMU/wheel input의 선형 EKF state output | `src/erp_driver/scripts/1024_EBIMU_EKF.py:61-64,121,126,133,141` | 칼만필터 이론(선형 EKF vs UKF sigma-point) — 현재 UKF가 아닌 선형 EKF임을 mission-cards.md에서 이미 확인, sigma-point 확장을 공부하면 보고서 주장과 코드를 맞출 수 있음 |
| [데이터] | src | `erp_driver` ○ PointCloud2 ROI obstacle 판단 | `src/erp_driver/scripts/0822_Obstacle_3d.py:14,17,69,117` | 장애물 회피 알고리즘 비교(potential field, DWA) — 지금은 ROI 임계값 기반 단순 정지/서행뿐 |
| [데이터] | src | `erp_driver` ● Excel waypoint parse→ENU Path | `src/erp_driver/scripts/erp42_pubwaypointscnuservice_pymap3d.py:19,41,51,86` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | src | `ntrip_client` ○ NMEA/GPS→RTCM correction stream | `src/ntrip_client/scripts/ntrip_ros_base.py:61-78,100-104,163-182` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | src | `pcl_clustering_py` ○ PointCloud2 RANSAC·Euclidean cluster colorized output | `src/pcl_clustering_py/pcl_clustering_py/euclidean_cluster_node.py:41-44,56-84,93-123` | 클러스터링 결과를 실제 장애물 리스트로 변환해 pathtracking에 연결하는 인지-판단 통합 설계(현재 미연결 — §9 Future Work) |
| [데이터] | src | `robot_localization` ● bag1 YAML·output argument + EKF pose test | `src/robot_localization/test/test_ekf_localization_node_bag1.launch.py:25-34,42-60`; `src/robot_localization/CMakeLists.txt:239-255` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | src | `velodyne` ○ VLP16 packet→PointCloud2/LaserScan pipeline | `src/velodyne/velodyne/launch/velodyne-all-nodes-VLP16-launch.py:43-70` | LiDAR 드라이버 설정(rpm/해상도 트레이드오프)은 vendor 담당이나, point cloud 활용은 본인 몫 — dead branch를 실제로 연결하는 것이 다음 학습 단계 |
| [데이터] | src | `Yolo_pt` ○ YOLO binary weight artifact | `src/Yolo_pt/best.pt` — line anchor 없음(§4 명시) | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `[src root]` ● rosbag dynamic deserialize·flatten→topic별 CSV | `rosbag2csv.py:36-69,71-109` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `erp_driver` ○ GPS covariance 기반 EKF update | `scripts/erp42_imu-gps-wheel-ekf_globalposition.py:154-188` | 칼만필터 이론(공분산 가중 보정의 수학적 근거) — 팀 동료 담당 구간이라 코드 읽기 수준의 이해 |
| [데이터] | erp42_main | `erp_driver` ● Excel `Longitude`/`Latitude`→waypoint list | `scripts/erp42_pubwaypointscnuservice_pymap3d.py:40-63,84-87` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `erp_driver` ○ pose/path/LiDAR state의 waypoint command 산출 | `scripts/erp42_pathtracking.py:26-32,59-79,138-173,175-190` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `erp_driver` ○ 18-byte status decode + command packet encode | `scripts/erp42_serial.py:30-46,48-74` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `erp_interfaces` ○ ERP42 command/status wire fields | `msg/ErpCmdMsg.msg:1-5`; `msg/ErpStatusMsg.msg:1-8` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `ntrip_client` ○ NMEA/fix input + RTCM payload output | `scripts/ntrip_ros_base.py:60-78,94-105,163-165` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `rosbag_convert_clean_py` ● GPS typed load·결측/유효범위 filter·CSV 저장 | `gps_convert.py:4-7,9-28,30-42` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `ublox_gps` ● NAV pose/covariance/clock·NMEA sensor data surface | `src/node.cpp:474-499` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `ublox_msgs` ● time/fix/RTK/LLH/NED/heading schema | `msg/NavPVT.msg:13-45,44-62,70-90` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `ublox_serialization` ○ ROS message↔u-blox binary serialization metadata | `package.xml:3-18` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `usb_cam` ○ image + camera_info message surface | `src/usb_cam_node.cpp:37-62` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `vectornav` ● time/IMU/GNSS/magnetic/pressure/velocity/pose message set | `src/vn_sensor_msgs.cc:43-60` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `vectornav_msgs` ● configurable INS composite fields | `msg/InsGroup.msg:1-5,12-39` | vendor/upstream 패키지 — 개인 기여 아님(§0-1 경계). 학습 여지: 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [데이터] | erp42_main | `waypoint` ○ GPS waypoint `.xls` dataset | `waypoints_real_w2toe4_gps.xls` — text line anchor 없음(§4 명시) | binary 데이터 — 좌표계 학습 여지는 이를 소비하는 pymap3d 변환 코드 쪽에서 다룸 |
| [데이터] | erp42_main | `yolo_ros` ○ image→DetectionArray inference data path | `yolo_ros/yolo_ros/yolo_node.py:108-123,145-154,327-349` | 데이터 파이프라인 재현성(로깅 스키마, 테스트 커버리지) |
| [HRI] | src | `usb_cam` ● `/hand_raw`·`/hand_xyz`·`/hand_mode`를 hand marker/target·`HOME`/`TRACKING` mode로 반영(ERP42와 별도 앱) | `src/usb_cam/launch/Pick_Tray_SA.py:251-266,413,442,554-558,606,622,720` | ERP42 프로젝트와 무관한 별개 Isaac Sim 앱(PORTFOLIO_MAP.md §6-D) — 포트폴리오 학습 로드맵 대상 아님 |

## 커버리지 자체 대조

- 대조 기준: `PORTFOLIO_MAP.md` §3-A·§3-B의 `●` 또는 `○` 셀을 `(패키지, 파일, 축)` 단위로 계산했다.
- 원문 표시 셀: 총 204개 — `src/` 86개, `erp42_main/src/` 118개.
- 태그별 대조: 시뮬 7, 비전 10, 좌표계 28, 통신 42, 노드설계 15, 모션플래닝 11, 상태관리 22, 안전 13, 인프라 29, 데이터 26, HRI 1.
- 본 표 행: 총 204개. 원문 대비 누락 0개, 추가 0개.
- line anchor 예외: §4가 text line anchor 부재를 명시한 binary artifact `src/Yolo_pt/best.pt` 2셀과 `waypoints_real_w2toe4_gps.xls` 3셀은 새 근거를 만들지 않고 해당 사실을 그대로 기록했다.
