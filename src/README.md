# colcon_ws/src — 단일 캐노니컬 워크스페이스 (공개 기준)

ERP42 자율주행 개발 워크스페이스. 2026-09-18 병합으로 이 `src/`가 **공개 기준 단일 ws**가 됐다 — `erp42_main/`(팀 정리본)과의 차이 요약은 [상위 README](../README.md)의 "저장소 구성" 절과 [`erp42_main/README.md`](../erp42_main/README.md)에 있다. `erp42_main/`은 삭제하지 않고 협업 이력 참고용으로만 남아 있다(더 이상 "최종본"이 아님).

## 핵심 파일 & 코드 흐름

`erp_driver/scripts/`가 실질적인 진입점이다.

| 역할 | 파일 |
|---|---|
| Localization(EKF) | `erp_driver/scripts/erp42_ebimu_ekf_globalposition.py` |
| 차선 인식(실험 — 본인 기억상 대회 주행은 path 추종만 쓴 것으로 추정 — lane 최종 채택 여부는 미확인(07-04 실도로 동시 구동 테스트 기록은 있음)) | `erp_driver/scripts/erp42_lanedetect.py`, `erp42_lanedetect_yolo.py`(+`yolo_ros/`) |
| LiDAR 장애물인식(실험) | `pcl_clustering_py/`, `cluster_bev/`, `erp_driver/scripts/archive/0724_*`~`0822_*`(dead branch, 실험 이력), `erp42_pathtracking_lidar_integrated.py`(미배선 — 없는 `erp_driver.msg`를 import하고 `/odometry/filtered/global`·`/waypoints_path`를 구독해 현재 토픽과 불일치) |
| Pathtracking | `erp_driver/scripts/erp42_pathtracking.py` |
| Command 중재(Controller) | `erp_driver/scripts/erp42_controller.py` |
| Actuation | `erp_driver/scripts/erp42_serial.py` |

코드 흐름(요약): `GPS/IMU/encoder → erp42_ebimu_ekf_globalposition → erp42_pathtracking → erp42_controller(lane/path 중재) → erp42_serial`. camera→차선인식, LiDAR→pathtracking 경로는 topic 불일치/미발행 조건으로 정적으로는 dead branch다.

### ⚠️ 병합 후 알려진 정합성 문제 (2026-09-18, codex 교차검증)
main에만 있던 `erp42_lanedetect.py`를 이 ws의 Controller 아키텍처에 새로 편입하면서 드러난 배선 문제:
- ✅ **수정 완료 (2026-09-18)**: `erp42_lanedetect.py`의 `publish_steering()`이 `steer`만 설정하고 `brake`를 안 정해서 `erp42_controller.py:39`의 lane 채택 조건(`brake==3`)을 못 만족하던 문제 — `e_stop=False`, `gear=0`, `speed=15`, `brake=3`을 추가해 Controller가 lane 명령을 채택하도록 수정. `colcon build`(erp_driver) PASS + `ast.parse` 구문 검증 완료.
- ✅ **수정 완료 (2026-09-18)**: `erp42_lanedetect_yolo.py:21`이 정의되지 않은 `Detection2DArray`를 참조해 `NameError`로 노드 생성 자체가 실패하던 문제(main 시절부터 있던 버그) — import된 `DetectionArray`로 교체. 같은 파일의 `brake=1`(구독 조건 `brake==3` 불충족)도 함께 `brake=3`으로 수정. `colcon build`(erp_driver) PASS + `ast.parse` 구문 검증 완료(`yolo_msgs.DetectionArray` 실제 임포트 성공 여부는 노드 기동 시점에만 확인 가능).
- `erp_driver/CMakeLists.txt:15`는 serial 관련 스크립트 3개만 설치 대상으로 등록 — `colcon build` PASS는 lane/Controller/EKF 스크립트가 `ros2 run`으로 설치·실행된다는 뜻이 아니라 `python3 <경로>` 직접 실행 전용이라는 의미다.
- pathtracking 속도값: main은 `self.max_linear_speed`로 파라미터화(값 50)했는데 이 ws는 여전히 하드코딩 `15`다. 토픽(`/erp42_ctrl_cmd/path`)·`brake=2`는 Controller 연동 때문에 유지해야 하지만, 속도 파라미터화 자체는 포팅할 가치가 있는 개선점 — 아직 반영 안 함.

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

### LiDAR / Localization 실험 스택 (erp42_main에는 없던 개인 전용 영역)
| 패키지 | 역할 |
|---|---|
| `velodyne` | Velodyne LiDAR 드라이버 |
| `hdl_localization` | NDT/GICP 기반 3D localization |
| `hdl_global_localization` | 전역 초기 위치 추정 |
| `ndt_omp` | OpenMP 가속 NDT scan matching |
| `fast_gicp` | GPU/CPU 가속 GICP scan matching |
| `pcl_clustering_py`, `cluster_bev` | PCL 기반 포인트클라우드 클러스터링 |
| `robot_localization` | upstream EKF/UKF 패키지 사본 — 현재 경로의 측위는 이 패키지가 아니라 custom 선형 EKF(`erp42_ebimu_ekf_globalposition.py`)이고, 이 출력(`/odometry/filtered/global`)은 미배선 실험본만 구독 |
| `pcl_ros` | upstream PCL ROS 패키지 사본 |
| `Yolo_pt` | YOLO 가중치 로컬 보관 위치(`best.pt`, `last.pt`, `best_lane_yolov11.pt`) — `*.pt`는 gitignore라 **공개 저장소에는 없다** |

## `erp_driver/scripts` 정리 상태 (2026-09-18 병합 후)
날짜 prefix 실험 스크립트 15개(LiDAR 회피 8개, EKF 2개, 차선인식 3개, pathtracking 변형 1개, 기타 1개)는 `erp_driver/scripts/archive/`로 이동했다 — 삭제 아님, "무엇을 시도했는지" 이력 보존 목적. 날짜 prefix를 뗀 파일이 현재 경로다: `erp42_ebimu_ekf_globalposition.py`, `erp42_pathtracking.py`, `erp42_controller.py`, `erp42_serial.py`(+waypoint 발행 `erp42_pubwaypointscnuservice_pymap3d.py`). `erp42_lanedetect.py`, `erp42_lanedetect_yolo.py`도 prefix가 없지만 실험 단계다. 본인 기억상 대회 주행은 path 추종만 쓴 것으로 추정 — lane 최종 채택 여부는 미확인(07-04 실도로 동시 구동 테스트 기록은 있음).

`lanedetect.py`, `claude_lanedetect.py`는 ROS 노드가 아닌 독립 영상 실험본(로컬 mp4 대상 LDA 기반 차선 검출 알고리즘 variant)으로, `erp42_lanedetect.py`(ROS 노드, canonical)와 겹치는 클래스명(`GradientEnhancedLaneDetector`)을 갖지만 서로 import하지 않는 별개 파일이다(codex 교차검증, 2026-09-18) — 실험 이력으로만 의미 있음, archive 이동 검토 대상.

## 빌드 검증 (2026-09-18)
- `colcon build --base-paths src --packages-select <pkg>` 개별 실행: `erp_driver`✅ `erp_interfaces`✅ `ntrip_client`✅ `ublox_serialization`✅ `ublox_msgs`✅ `vectornav_msgs`✅ `ublox_gps`✅ / `usb_cam`❌ `vectornav`❌(둘 다 `ament_lint_auto` 미설치, 병합과 무관한 환경 문제)
- `colcon build --base-paths src`(패키지 지정 없이 전체)는 `ament_lint_auto`/`ament_cmake_cppcheck` 미설치와 `ndt_omp` 자체 문제로 일부 패키지가 실패한다(2026-09-18 기록, 실패 패키지 전체 목록은 남아 있지 않음). 개별 PASS가 전체 동시 빌드 PASS를 보장하지 않는다.
- `waypoint`/`yolo_ros`/`rosbag_convert_clean_py`는 빌드·실행 미검증(⚠️, yolo_ros는 무거운 의존성이라 스킵).

## ⚠️ 확인 필요 (미검증)
- `erp_driver/scripts` 중 어떤 스크립트가 실제로 실차에서 마지막으로 검증됐는지는 파일명(날짜)만으로 추정한 것이며 git log/실행 이력으로 확인하지 않음.
- 실기 성능 검증은 하지 않았다(위 "병합 후 알려진 정합성 문제"의 lane 배선 수정 포함). 카메라 토픽 불일치(`/usb_cam_0/image_raw` 구독 ↔ `usb_cam` launch의 `camera1/image_raw`)는 남아 있다.
