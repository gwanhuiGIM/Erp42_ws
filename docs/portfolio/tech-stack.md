# ERP42 ROS 2 기술 스택 — 코드·메타데이터 교차검증

> 대상: 개인 작업본 `src/`와 팀 정리본 `erp42_main/src/`  
> 판정 원칙: `docs/portfolio/mission-cards.md`와 동일하게, 실제 source import 또는 repository 안의 실행 설정으로 재현되는 항목만 **코드로 확인됨**으로 표시한다. 보고서의 기술명은 별도 행으로 분리한다.  
> 버전은 repository에 명시된 값만 적었다. 설치된 runtime 버전은 조사하지 않았으며, 명시가 없으면 `미기재`로 둔다.

## 판정 범례

- **코드로 확인됨**: `.py` import, 실행 설정, 또는 해당 구현 source가 존재한다.
- **코드로 확인됨 (metadata 미선언)**: import는 있지만 그 파일을 소유한 package의 `package.xml`/`setup.py`에는 dependency가 없다.
- **package.xml에만 선언되고 미사용**: 이번 절차의 `.py` import 기준으로 사용이 확인되지 않았다. C++ source 사용 여부까지 “미사용”이라는 뜻은 아니다.
- **보고서에만 있고 코드엔 없음**: 보고서/참고 문서의 주장과 대회용 ERP42 구현 source가 일치하지 않는다. repository에 vendor package가 존재하는 것만으로 대회 경로에 통합됐다고 보지 않는다.
- `setup.py`의 `setuptools`와 test-only lint dependency는 portfolio 기술 스택에서 제외하되, 아래 metadata 수집표에는 선언군으로 남겼다.

## 기술 스택과 불일치

| 라이브러리/프레임워크 | 버전(확인 가능한 경우, 없으면 '미기재') | 사용 파일:라인(최소 1개 예시) | src 또는 erp42_main 중 어디 있는지 | 확인 상태(코드로 확인됨 / package.xml에만 선언되고 미사용 / 보고서에만 있고 코드엔 없음) |
|---|---|---|---|---|
| ROS 2 Python client (`rclpy`) | 미기재 | `src/erp_driver/scripts/1024_EBIMU_EKF.py:2`; `erp42_main/src/erp_driver/scripts/erp42_pathtracking.py:3` | 양쪽 | **코드로 확인됨 (metadata 미선언)** — `erp_driver/package.xml`은 `rclcpp`만 선언하고 `rclpy`는 선언하지 않음 |
| ROS 2 C++ client (`rclcpp`) | 미기재 | `src/erp_driver/package.xml:8`; `erp42_main/src/erp_driver/package.xml:8` | 양쪽 | **package.xml에만 선언되고 미사용** (`.py` import 기준; C++ package dependency) |
| ROS 2 interface/message stack (`sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`) | 미기재 | `src/erp_driver/scripts/1024_EBIMU_EKF.py:4`; `erp42_main/src/erp_driver/scripts/erp42_pathtracking.py:6` | 양쪽 | **코드로 확인됨** — 단 `erp_driver/package.xml`에는 실제 import 중 `geometry_msgs`, `nav_msgs`가 미선언 |
| `erp_interfaces` | 0.0.1 | `src/erp_driver/scripts/1024_EBIMU_EKF.py:9`; `erp42_main/src/erp_driver/package.xml:11` | 양쪽 | **코드로 확인됨** |
| NumPy | `<2` (`yolo_ros/requirements.txt` 기준) | `src/erp_driver/scripts/1024_EBIMU_EKF.py:11`; `erp42_main/src/erp_driver/scripts/ByteHandler.py:4` | 양쪽 | **코드로 확인됨 (metadata 미선언)** — `erp_driver`와 `pcl_clustering_py`의 `package.xml/setup.py`에는 없음; `<2` 제약은 팀본 `yolo_ros`에만 별도 기재 |
| OpenCV (`cv2`) | `>=4.8.1.78` (`yolo_ros/requirements.txt` 기준) | `src/erp_driver/scripts/0702_erp42_lanedetect.py:6`; `erp42_main/src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py:17` | 양쪽 | **코드로 확인됨 (metadata 일부 미선언)** — `erp_driver`에는 미선언, `yolo_ros`는 별도 requirements에만 `opencv-python` 기재 |
| `cv_bridge` | 미기재 | `src/erp_driver/scripts/0702_erp42_lanedetect.py:8`; `erp42_main/src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py:19` | 양쪽 | **코드로 확인됨 (metadata 일부 미선언)** — `usb_cam`/`yolo_ros`에는 선언됐지만 `erp_driver`에는 없음 |
| scikit-learn (`LinearDiscriminantAnalysis`, `DBSCAN`) | 미기재 | `src/erp_driver/scripts/0822_Lam_ObtAvo.py:12`; `erp42_main/src/erp_driver/scripts/claude_lanedetect.py:3` | 양쪽 | **코드로 확인됨 (metadata 미선언)** |
| Matplotlib | 미기재 | `src/erp_driver/scripts/0702_erp42_lanedetect.py:12` | src | **코드로 확인됨 (metadata 미선언)** |
| pandas | 미기재 | `src/erp_driver/scripts/erp42_pubwaypointscnuservice_pymap3d.py:5`; `erp42_main/src/rosbag_convert_clean_py/gps_convert.py:2` | 양쪽 | **코드로 확인됨 (metadata 미선언)** |
| pymap3d | 미기재 | `src/erp_driver/scripts/1024_EBIMU_EKF.py:12`; `erp42_main/src/erp_driver/scripts/erp42_imu-gps-wheel-ekf_globalposition.py:11` | 양쪽 | **코드로 확인됨 (metadata 미선언)** |
| pyserial (`serial`; ROS dependency `python3-serial`) | 미기재 | `src/ebimu_pkg/ebimu_pkg/ebimu_publisher.py:10`; `erp42_main/src/ntrip_client/src/ntrip_client/ntrip_serial_device.py:5` | 양쪽 | **코드로 확인됨 (metadata 일부 미선언)** — `ntrip_client`에는 선언됐지만 `erp_driver`와 `ebimu_pkg`에는 없음 |
| SciPy | 미기재 | `src/hdl_localization/scripts/plot_status.py:7` | src | **코드로 확인됨 (metadata 미선언)** |
| Python-PCL (`pcl`) | 미기재 | `src/pcl_clustering_py/pcl_clustering_py/euclidean_cluster_node.py:8` | src | **코드로 확인됨 (metadata 미선언)** — package에는 `rclpy`, `sensor_msgs`만 runtime 선언 |
| PCL / `pcl_ros` | `pcl_ros` 2.6.2 | `src/pcl_ros/package.xml:5`; `src/pcl_ros/package.xml:30` | src | **package.xml에만 선언되고 미사용** (`.py` import 기준; `libpcl-*`, Eigen, ROS C++ dependency) |
| fast_gicp / PyGICP (`pygicp`) | **0.0.0 (`package.xml`) / 0.0.1 (`setup.py`)** | `src/fast_gicp/src/kitti.py:6`; `src/fast_gicp/setup.py:108` | src | **코드로 확인됨** — 두 metadata의 version 불일치 |
| NDT-OMP (`ndt_omp`) | 0.0.0 | `src/hdl_localization/package.xml:28`; `src/ndt_omp/package.xml:5` | src | **package.xml에만 선언되고 미사용** (`.py` import 기준; localization C++ dependency) |
| HDL localization (`hdl_localization`, `hdl_global_localization`) | 0.0.0 | `src/hdl_localization/scripts/plot_status.py:10`; `src/hdl_localization/package.xml:29` | src | **코드로 확인됨** |
| `robot_localization` | 3.5.3 | `src/robot_localization/package.xml:5`; `src/robot_localization/launch/ekf.launch.py:29` | src | **코드로 확인됨** — upstream EKF/UKF package와 launch/config가 repository에 존재함; 대회용 custom EKF와는 별도 |
| NTRIP client | 1.4.0 | `src/ntrip_client/scripts/ntrip_ros_base.py:8`; `erp42_main/src/ntrip_client/setup.py:9` | 양쪽 | **코드로 확인됨** |
| u-blox ROS 2 driver | 2.3.0 | `src/ublox/ublox_gps/package.xml:4`; `erp42_main/src/ublox_gps/package.xml:4` | 양쪽 | **package.xml에만 선언되고 미사용** (`.py` import 기준; C++ driver) |
| VectorNav ROS 2 driver | 3.0.0 | `src/vectornav/vectornav/package.xml:5`; `erp42_main/src/vectornav/package.xml:5` | 양쪽 | **package.xml에만 선언되고 미사용** (`.py` import 기준; C++ driver) |
| Velodyne ROS 2 stack | 2.5.1 | `src/velodyne/velodyne_driver/package.xml:5`; `src/velodyne/velodyne/package.xml:18` | src | **package.xml에만 선언되고 미사용** (`.py` import 기준; C++ driver/pointcloud stack) |
| `usb_cam` | 0.8.1 | `src/usb_cam/scripts/show_image.py:32`; `erp42_main/src/usb_cam/package.xml:5` | 양쪽 | **코드로 확인됨** — Python viewer/config도 존재하고 camera node 본체는 C++ package |
| Pydantic | 미기재 | `src/usb_cam/launch/camera_config.py:34`; `erp42_main/src/usb_cam/package.xml:40` | 양쪽 | **코드로 확인됨** (`python3-pydantic` 선언과 import 일치) |
| PyYAML (`yaml`) | 미기재 | `src/robot_localization/launch/ekf.launch.py:20`; `erp42_main/src/ublox_gps/launch/ublox_gps_node-composed-launch.py:43` | 양쪽 | **코드로 확인됨 (metadata 미선언)** — `yaml-cpp`/`yaml_cpp_vendor`는 Python `yaml`과 다른 dependency |
| rosbag2 Python API (`rosbag2_py`) | 미기재 | `erp42_main/src/rosbag2csv.py:33` | erp42_main | **코드로 확인됨 (metadata 미선언)** — 이 root-level script를 소유하는 package metadata가 없음 |
| `tf_transformations` | 미기재 | `src/erp_driver/scripts/1024_EBIMU_EKF.py:8`; `erp42_main/src/erp_driver/scripts/erp42_imu-gps-wheel-ekf_globalposition.py:7` | 양쪽 | **코드로 확인됨 (metadata 미선언)** |
| Ultralytics | 8.3.91 | `erp42_main/src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py:31`; `erp42_main/src/yolo_ros/requirements.txt:4` | erp42_main | **코드로 확인됨 (package.xml/setup.py 미선언)** — 별도 requirements에만 pin |
| PyTorch (`torch`) | 미기재 | `erp42_main/src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py:30` | erp42_main | **코드로 확인됨 (metadata 미선언)** — `package.xml`, `setup.py`, 별도 requirements 모두 직접 선언 없음 |
| YOLO ROS 2 wrapper (`yolo_ros`, `yolo_msgs`, `yolo_bringup`) | 4.2.0 | `erp42_main/src/yolo_ros/yolo_ros/package.xml:5`; `erp42_main/src/yolo_ros/yolo_ros/setup.py:22` | erp42_main | **코드로 확인됨** |
| YOLOv11 model family | `yolo11m.pt` / `best_lane_yolov11.pt` (정확한 trained architecture는 미확인) | `erp42_main/src/yolo_ros/yolo_bringup/launch/yolov11.launch.py:38`; `erp42_main/src/best_lane_yolov11.pt` | erp42_main | **코드로 확인됨** — launcher default와 weight filename까지만 확인; weight 내부 metadata는 미확인 |
| YOLOv8l | 미기재 | `docs/portfolio/mission-cards.md:24` | 보고서 주장(두 코드 범위 밖) | **보고서에만 있고 코드엔 없음** — YOLOv8 launcher default는 `yolov8m.pt`이며 `v8l` 근거가 아님 |
| `message_filters` | 미기재 | `erp42_main/src/yolo_ros/yolo_ros/yolo_ros/tracking_node.py:28` | erp42_main | **코드로 확인됨 (yolo_ros metadata 미선언)** — 다른 `src/robot_localization` package의 선언으로 대체되지 않음 |
| `tf2_ros` | 미기재 | `erp42_main/src/yolo_ros/yolo_ros/yolo_ros/detect_3d_node.py:32` | erp42_main | **코드로 확인됨 (yolo_ros metadata 미선언)** |
| Isaac Sim (`isaacsim`, `omni`, `pxr`) | 미기재 | `src/usb_cam/launch/Pick_Tray_SA.py:3`; `src/usb_cam/launch/Pick_Tray_SA.py:19` | src | **코드로 확인됨 (usb_cam metadata 미선언)** — ERP42 camera package 아래에 있는 별도 simulation script로 확인됨 |
| ROS 1 `dynamic_reconfigure` compatibility script | 미기재 | `src/pcl_ros/cfg/common.py:3` | src | **코드로 확인됨 (metadata 미선언)** — ROS 2 `pcl_ros` metadata에는 해당 Python dependency가 없음 |
| custom 선형 EKF (NumPy 구현) | 해당 없음 | `src/erp_driver/scripts/1024_EBIMU_EKF.py:141`; `erp42_main/src/erp_driver/scripts/erp42_imu-gps-wheel-ekf_globalposition.py:203` | 양쪽 | **코드로 확인됨** — GPS update의 고정 선형 관측행렬 예: 개인본 `:133`, 팀본 `:167` |
| 대회 센서융합 구현으로서 UKF (7 sigma points) | 미기재 | `docs/portfolio/mission-cards.md:26` | 보고서 주장(두 ERP42 구현과 불일치) | **보고서에만 있고 코드엔 없음** — custom ERP42 EKF 두 파일에는 sigma-point 생성이 없음; repository의 upstream `robot_localization` UKF는 별개 |

### import는 있으나 package metadata가 빠진 핵심 항목 요약

| 라이브러리/프레임워크 | 버전(확인 가능한 경우, 없으면 '미기재') | 사용 파일:라인(최소 1개 예시) | src 또는 erp42_main 중 어디 있는지 | 확인 상태(코드로 확인됨 / package.xml에만 선언되고 미사용 / 보고서에만 있고 코드엔 없음) |
|---|---|---|---|---|
| `erp_driver` Python runtime: `rclpy`, NumPy, pyserial, pymap3d, pandas, OpenCV, `cv_bridge`, scikit-learn, Matplotlib, `geometry_msgs`, `nav_msgs`, `tf_transformations`, `yolo_msgs` | 미기재 | `src/erp_driver/package.xml:8`; `src/erp_driver/setup.py:16`; `erp42_main/src/erp_driver/setup.py:16` | 양쪽 | **코드로 확인됨 (metadata 미선언/일부 미선언)** — metadata는 `rclcpp`, `std_msgs`, `sensor_msgs`, `erp_interfaces`, `setuptools`만 선언 |
| `pcl_clustering_py` runtime: NumPy, Python-PCL | 미기재 | `src/pcl_clustering_py/pcl_clustering_py/euclidean_cluster_node.py:7`; `src/pcl_clustering_py/setup.py:14` | src | **코드로 확인됨 (metadata 미선언)** |
| `ebimu_pkg` runtime: pyserial | 미기재 | `src/ebimu_pkg/ebimu_pkg/ebimu_publisher.py:10`; `src/ebimu_pkg/package.xml:10` | src | **코드로 확인됨 (metadata 미선언)** |
| `yolo_ros` runtime: `rclpy`, OpenCV, NumPy, PyTorch, Ultralytics, `message_filters`, `tf2_ros`, `geometry_msgs`, `visualization_msgs` | Ultralytics 8.3.91; OpenCV >=4.8.1.78; NumPy <2 | `erp42_main/src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py:17`; `erp42_main/src/yolo_ros/yolo_ros/package.xml:15` | erp42_main | **코드로 확인됨 (package.xml/setup.py 미선언/일부 미선언)** — package metadata는 `cv_bridge`, `std_srvs`, `sensor_msgs`, `yolo_msgs`만 runtime 선언 |

## package.xml / setup.py 선언 수집표

아래는 조사한 package별 선언을 기능군으로 묶은 것이다. “미사용”은 오직 `.py import` 대조 결과이며 CMake/C++ include나 launch의 package 실행은 별도다.

| 라이브러리/프레임워크 | 버전(확인 가능한 경우, 없으면 '미기재') | 사용 파일:라인(최소 1개 예시) | src 또는 erp42_main 중 어디 있는지 | 확인 상태(코드로 확인됨 / package.xml에만 선언되고 미사용 / 보고서에만 있고 코드엔 없음) |
|---|---|---|---|---|
| `erp_driver` 선언군: `rclcpp`, `std_msgs`, `sensor_msgs`, `erp_interfaces`; setup: `setuptools` | 0.0.1 | `src/erp_driver/package.xml:8`; `src/erp_driver/setup.py:16` | 양쪽 | `std_msgs`, `sensor_msgs`, `erp_interfaces`는 **코드로 확인됨**; `rclcpp`는 **package.xml에만 선언되고 미사용** (`.py` 기준) |
| `erp_interfaces` 선언군: `ament_cmake`, `rosidl_default_generators`, `rosidl_default_runtime` | 0.0.1 | `src/erp_interfaces/package.xml:9` | 양쪽 | **package.xml에만 선언되고 미사용** (`.py` import 기준; interface build/runtime 선언) |
| `ntrip_client` 선언군: `rclpy`, `std_msgs`, `rtcm_msgs`, `nmea_msgs`, `sensor_msgs`, `python3-serial`; setup: `setuptools` | 1.4.0 | `src/ntrip_client/package.xml:10`; `erp42_main/src/ntrip_client/setup.py:9` | 양쪽 | **코드로 확인됨** |
| `usb_cam` 선언군: `rclcpp`, `rclcpp_components`, `cv_bridge`, `std_msgs`, `std_srvs`, `sensor_msgs`, `camera_info_manager`, `builtin_interfaces`, `image_transport`, `image_transport_plugins`, `v4l-utils`, `ffmpeg`, `python3-pydantic`, ROSIDL/ament build·test stack | 0.8.1 | `src/usb_cam/package.xml:23`; `erp42_main/src/usb_cam/package.xml:40` | 양쪽 | Python의 `cv_bridge`, messages, Pydantic는 **코드로 확인됨**; 나머지는 **package.xml에만 선언되고 미사용** (`.py` 기준) |
| `ublox*` 선언군: `asio`, diagnostics, `geometry_msgs`, `nmea_msgs`, `rcl_interfaces`, `rclcpp*`, `rtcm_msgs`, `sensor_msgs`, `std_msgs`, `tf2`, `ublox_msgs`, `ublox_serialization`, ROSIDL/ament stack | 2.3.0 | `src/ublox/ublox_gps/package.xml:17`; `erp42_main/src/ublox_msgs/package.xml:18` | 양쪽 | **package.xml에만 선언되고 미사용** (`.py` import 기준) |
| `vectornav*` 선언군: `rclcpp`, `rclcpp_action`, `rclcpp_components`, `geometry_msgs`, `sensor_msgs`, `vectornav_msgs`, `tf2_geometry_msgs`, ROSIDL/ament stack | 3.0.0 | `src/vectornav/vectornav/package.xml:12`; `erp42_main/src/vectornav_msgs/package.xml:10` | 양쪽 | **package.xml에만 선언되고 미사용** (`.py` import 기준) |
| `velodyne*` 선언군: `rclcpp*`, diagnostics, `libpcap`, `sensor_msgs`, `tf2_ros`, `velodyne_msgs`, PCL/Eigen/message_filters/yaml-cpp, ROSIDL/ament stack | 2.5.1 | `src/velodyne/velodyne_driver/package.xml:17`; `src/velodyne/velodyne_pointcloud/package.xml:20` | src | **package.xml에만 선언되고 미사용** (`.py` import 기준) |
| localization 선언군: `hdl_*`, `fast_gicp`, `ndt_omp`, `pcl_ros`, PCL/Eigen, `rclcpp*`, `rclpy`, ROS messages/services, `tf2*`, diagnostics, GeographicLib, Boost, message_filters, YAML vendor, ROSIDL/ament/launch stack | 각 package 행 참조 | `src/hdl_localization/package.xml:15`; `src/robot_localization/package.xml:23`; `src/pcl_ros/package.xml:34` | src | 일부 Python import는 **코드로 확인됨**; C++/build/test 전용 항목은 **package.xml에만 선언되고 미사용** (`.py` 기준) |
| `yolo_ros` 선언군: `cv_bridge`, `std_srvs`, `sensor_msgs`, `yolo_msgs`; `yolo_msgs`의 `std_msgs`, `geometry_msgs`, ROSIDL stack; `yolo_bringup`의 `yolo_ros`; setup: `setuptools` | 4.2.0 | `erp42_main/src/yolo_ros/yolo_ros/package.xml:15`; `erp42_main/src/yolo_ros/yolo_msgs/package.xml:16` | erp42_main | 선언 항목은 **코드로 확인됨**; 위 표의 추가 runtime import는 metadata 미선언 |
| `cluster_bev` 선언군: `rclcpp`, `sensor_msgs`, `pcl_conversions`, `ament_cmake` | 0.0.1 | `src/cluster_bev/package.xml:8` | src | **package.xml에만 선언되고 미사용** (`.py` import 기준; C++ package) |

## 미확인 또는 근거 없음

- `filterpy`: 두 범위의 `.py`, `package.xml`, `setup.py`에서 import/선언이 모두 검색되지 않았다. 따라서 기술 스택 행으로 확정하지 않는다.
- `src/Yolo_pt/best.pt`, `src/Yolo_pt/last.pt`: 파일은 존재하지만 filename만으로 YOLO major version을 정하지 않았다.
- `erp42_main/src/best_lane_yolov11.pt`: filename은 YOLOv11을 가리키지만 weight 내부 metadata는 읽지 않았으므로 정확한 architecture/version은 `미확인`이다.
- 설치된 Python/ROS package version과 실제 node 실행 성공 여부는 이번 정적 source 검증 범위 밖이다.

## 검증 방법

실제로 실행한 read/grep 명령(작업 디렉터리: `/home/kimkh/colcon_ws`):

```bash
sed -n '1,280p' docs/portfolio/mission-cards.md
find src erp42_main/src -type f -name package.xml -print | sort
find src erp42_main/src -type f -name setup.py -print | sort
rg -n --glob 'package.xml' '<(depend|build_depend|buildtool_depend|exec_depend|test_depend|build_export_depend|member_of_group)>[^<]+' src erp42_main/src
rg -n --glob 'package.xml' '<version>[^<]+' src erp42_main/src
rg -n --glob 'setup.py' 'install_requires|entry_points|console_scripts|version=' src erp42_main/src
rg -n --glob '*.py' '^\s*(from\s+[A-Za-z_][A-Za-z0-9_.]*\s+import|import\s+[A-Za-z_][A-Za-z0-9_., ]*)' src erp42_main/src
rg -n --glob '*.py' '^\s*(from\s+(numpy|cv2|ultralytics|pymap3d|filterpy|scipy|sklearn|matplotlib|serial|pandas|torch|yaml)(\.|\s)|import\s+(numpy|cv2|ultralytics|pymap3d|filterpy|scipy|sklearn|matplotlib|serial|pandas|torch|yaml)(\.|\s|,|$))' src erp42_main/src
rg -n --glob '*.py' '^\s*(from\s+filterpy(\.|\s)|import\s+filterpy(\.|\s|,|$))' src erp42_main/src
rg -n 'def (predict|cb_gps)|H\s*=|sigma|Merwe|Unscented|Kalman' src/erp_driver/scripts/1024_EBIMU_EKF.py erp42_main/src/erp_driver/scripts/erp42_imu-gps-wheel-ekf_globalposition.py
rg -n -i 'YOLOv8|YOLOv11|UKF|Unscented|filterpy|YOLO' docs/portfolio/mission-cards.md src erp42_main/src --glob '*.py' --glob 'package.xml' --glob 'setup.py' --glob '*.launch.py' --glob '*.yaml'
find src erp42_main/src -type f \( -iname '*.pt' -o -iname '*yolo*' \) -print | sort
```
