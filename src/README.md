# colcon_ws/src — 공개 기준 워크스페이스

ERP42 자율주행 개발 워크스페이스이며, 이 저장소의 공개 기준이다. `erp42_main/`은 협업 이력 참고용으로 남긴 과거 정리본이고, 차이 요약은 [상위 README](../README.md)의 "저장소 구성" 절과 [`erp42_main/README.md`](../erp42_main/README.md)에 있다.

## 핵심 파일 & 코드 흐름

`erp_driver/scripts/`가 실질적인 진입점이다.

| 역할 | 파일 |
|---|---|
| Localization(EKF) | `erp_driver/scripts/erp42_ebimu_ekf_globalposition.py` |
| Pathtracking | `erp_driver/scripts/erp42_pathtracking.py` |
| Command 중재(Controller) | `erp_driver/scripts/erp42_controller.py` |
| Actuation | `erp_driver/scripts/erp42_serial.py` |
| 차선 인식(실험) | `erp_driver/scripts/erp42_lanedetect.py`, `erp42_lanedetect_yolo.py`(+`yolo_ros/`) |
| LiDAR 장애물인식(실험) | `pcl_clustering_py/`, `cluster_bev/`, `erp_driver/scripts/archive/0724_*`~`0822_*`, `erp42_pathtracking_lidar_integrated.py` |

코드 흐름(요약): `GPS/IMU/encoder → erp42_ebimu_ekf_globalposition → erp42_pathtracking → erp42_controller(lane/path 중재) → erp42_serial`. 각 노드의 동작은 [상위 README](../README.md)의 "핵심 기능"·"시스템 구조"에 있다.

## 패키지 구성

### ERP42 드라이버 스택
| 패키지 | 역할 |
|---|---|
| `erp_driver` | ERP42 시리얼 드라이버 + 제어/경로추종/차선인식/Controller 스크립트 (`scripts/`) |
| `erp_interfaces` | ERP42 메시지 인터페이스 |
| `ntrip_client` | RTK 보정 NTRIP 클라이언트(mountpoint는 main 실차값 반영) |
| `ublox` (`ublox`/`ublox_gps`/`ublox_msgs`/`ublox_serialization` 서브패키지) | u-blox GPS 드라이버(config/launch는 main 실차값 반영) |
| `usb_cam` | USB 카메라 드라이버(`params_1~4.yaml` 4대 설정, main 기준 반영) |
| `vectornav` (`vectornav`/`vectornav_msgs` 서브패키지) | VectorNav IMU/INS/GNSS 드라이버 — EKF는 EBIMU로 대체됐지만 드라이버 패키지 자체는 유지(port는 main 실차값) |
| `ebimu_pkg` | EBIMU IMU 드라이버 — 현재 EKF가 실제로 쓰는 IMU 소스 |
| `waypoint` | waypoint 데이터(`.xls`) — `package.xml`/`setup.py` 없음, colcon 패키지 아님(main에서 편입) |
| `yolo_ros` | YOLO 추론 ROS2 래퍼(main에서 편입) |
| `rosbag_convert_clean_py`, `rosbag2csv.py` | rosbag 후처리 도구(main에서 편입, `package.xml` 없음) |

### LiDAR 실험 스택 (erp42_main에는 없는 영역)
| 패키지 | 역할 |
|---|---|
| `velodyne` | Velodyne LiDAR 드라이버 |
| `pcl_clustering_py`, `cluster_bev` | PCL 기반 포인트클라우드 클러스터링 |
| `Yolo_pt` | YOLO 가중치 로컬 보관 위치(`best.pt`, `last.pt`, `best_lane_yolov11.pt`) — `*.pt`는 gitignore라 공개 저장소에는 없다 |

어느 launch·코드도 참조하지 않는 upstream 사본 6개(`hdl_localization`, `hdl_global_localization`, `ndt_omp`, `fast_gicp`, `pcl_ros`, `robot_localization`)는 루트 `third_party/`로 옮겼다(2026-10-06). 현재 측위는 `robot_localization`이 아니라 custom 선형 EKF(`erp42_ebimu_ekf_globalposition.py`)다.

## `erp_driver/scripts` 구성
날짜 prefix가 없는 파일이 현재 경로다: `erp42_ebimu_ekf_globalposition.py`, `erp42_pathtracking.py`, `erp42_controller.py`, `erp42_serial.py`(+waypoint 발행 `erp42_pubwaypointscnuservice_pymap3d.py`). `erp42_lanedetect.py`, `erp42_lanedetect_yolo.py`도 prefix가 없지만 실험 단계다. 이 구분은 파일명(날짜 prefix 유무) 기준이다.

날짜 prefix 실험 스크립트 15개(LiDAR 회피 8개, EKF 2개, 차선인식 3개, pathtracking 변형 1개, 기타 1개)는 `archive/`에 이력으로 보존했다. `lanedetect.py`, `claude_lanedetect.py`는 ROS 노드가 아닌 독립 영상 실험본(로컬 mp4 대상 LDA 기반 차선 검출 variant)이다.

측위·경로·Controller 스크립트는 `ros2 run`으로 설치되지 않는다(`erp_driver/CMakeLists.txt:15`가 시리얼 스크립트 3개만 등록). `python3 <경로>`로 직접 실행한다.

## 대회 뒤 수정 내역
lane 노드를 Controller 구조에 편입하면서 고친 배선:
- `erp42_lanedetect.py`: `publish_steering()`이 `brake`를 정하지 않아 Controller의 lane 채택 조건(`brake==3`, `erp42_controller.py:39`)을 못 만족하던 문제 → `e_stop=False`, `gear=0`, `speed=15`, `brake=3`을 채워 Controller가 lane 명령을 채택하도록 수정.
- `erp42_lanedetect_yolo.py`: 정의되지 않은 `Detection2DArray` 참조로 노드 생성 시 `NameError`가 나던 문제(erp42_main 시절부터 있던 버그) → import된 `DetectionArray`로 교체. 같은 파일의 `brake=1`도 `brake=3`으로 수정.

## 검증
- 빌드(2026-10-06, `third_party/` 이동 후): `colcon build --symlink-install --continue-on-error --base-paths src --cmake-args -DBUILD_TESTING=OFF`로 src 패키지 21개 중 19개 ✅(`erp_driver`, `erp_interfaces`, `ebimu_pkg`, `ntrip_client`, `ublox*`, `pcl_clustering_py`, `cluster_bev`, `usb_cam`, `vectornav*`, `yolo_*`, `velodyne_msgs`/`_pointcloud`/`_laserscan`). `velodyne_driver`❌(`libpcap-dev` 미설치로 `pcap.h` 없음), `velodyne`(메타패키지) 미처리. `BUILD_TESTING`을 켜면 `ament_lint_auto` 미설치로 실패한다. 노드 실행은 하지 않았다.
- 위 수정 2건은 `erp_driver` 빌드 PASS + `ast.parse` 구문 확인까지 했다. `yolo_msgs.DetectionArray` import는 노드 기동 시점에만 확인 가능하다.
- `waypoint`/`yolo_ros`/`rosbag_convert_clean_py`는 빌드·실행 미검증.
- 실기 성능 검증은 하지 않았다(위 수정분 포함).

## 한계 · 미완성
1. **실기 정지 순서**: 시리얼 노드는 timeout 없이 마지막 명령을 재송신한다 — 끄는 순서는 [상위 README](../README.md) "정지 절차"를 따른다.
2. **차선 경로 미연결**: 카메라 토픽 불일치(`/usb_cam_0/image_raw` 구독 ↔ `usb_cam` launch의 `camera1/image_raw`). 대회 주행 사용 여부는 작성자 기억 기준(확인 자료 없음) path 추종만 쓴 것으로 추정하며, lane 최종 채택 여부는 미확인이다(07-04 실도로 동시 구동 테스트 기록은 있음).
3. **LiDAR 미연결**: 클러스터링 결과를 받는 Controller 입력이 없고, pathtracking의 LiDAR 분기는 발행자가 없다. `erp42_pathtracking_lidar_integrated.py`는 없는 `erp_driver.msg`를 import하고 `/odometry/filtered/global`·`/waypoints_path`를 구독해 현재 토픽과 맞지 않는다.
4. **저장소에 없는 자산**: YOLO 가중치(`Yolo_pt/*.pt`).

<details><summary>세부 사항</summary>

- pathtracking 속도는 하드코딩 `15`다(erp42_main은 `self.max_linear_speed`로 파라미터화, 값 50). 토픽(`/erp42_ctrl_cmd/path`)·`brake=2`는 Controller 연동 때문에 유지해야 하지만 속도 파라미터화는 아직 반영하지 않았다.
- `lanedetect.py`·`claude_lanedetect.py`는 `erp42_lanedetect.py`와 같은 클래스명(`GradientEnhancedLaneDetector`)을 갖지만 서로 import하지 않는 별개 파일이다.
</details>
