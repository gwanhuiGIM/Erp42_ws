# ERP42 자율주행 워크스페이스

2025 대학생 창작자동차 경진대회(팀 MTP) 출품작 — ERP42(Wego Robotics 4륜 전기차) 기반 자율주행 스택. ROS2 Humble.

## 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src --packages-select <패키지명>
```

⚠️ 패키지 지정 없이 `src` 전체를 한 번에 빌드하면 `ament_lint_auto`/`ament_cmake_cppcheck` 미설치, `ndt_omp` 자체 버그로 일부 패키지가 실패한다 — 위처럼 `--packages-select`로 개별 빌드할 것.

## 패키지 구성

### ERP42 드라이버 스택
| 패키지 | 역할 |
|---|---|
| `erp_driver` | ERP42 시리얼 드라이버 + 제어/경로추종/차선인식/Controller 스크립트(`scripts/`) |
| `erp_interfaces` | ERP42 메시지 인터페이스 |
| `ntrip_client` | RTK 보정 NTRIP 클라이언트 |
| `ublox` | u-blox GPS 드라이버(`ublox_gps`/`ublox_msgs`/`ublox_serialization` 서브패키지) |
| `usb_cam` | USB 카메라 드라이버 |
| `vectornav` | VectorNav IMU/INS/GNSS 드라이버 |
| `ebimu_pkg` | EBIMU IMU 드라이버 — 현재 EKF가 쓰는 IMU 소스 |
| `waypoint` | waypoint 데이터(`.xls`) |
| `yolo_ros` | YOLO 추론 ROS2 래퍼 |
| `rosbag_convert_clean_py` | rosbag 후처리 도구 |

### LiDAR / Localization 스택
| 패키지 | 역할 |
|---|---|
| `velodyne` | Velodyne LiDAR 드라이버 |
| `hdl_localization`, `hdl_global_localization` | NDT/GICP 기반 3D localization |
| `ndt_omp`, `fast_gicp` | scan matching 가속 |
| `pcl_clustering_py`, `cluster_bev` | 포인트클라우드 클러스터링 |
| `robot_localization` | EKF/UKF 센서 퓨전 |
| `Yolo_pt` | YOLO 가중치 저장소 |

## 아키텍처 (요약)

```
GPS/IMU/encoder → EKF(erp42_ebimu_ekf_globalposition.py)
                → pathtracking(erp42_pathtracking.py)
                → Controller(erp42_controller.py, lane/path 중재)
                → serial(erp42_serial.py) → 액추에이터
```

topic 단위 배선 근거·dead branch는 패키지별 스크립트에서 확인할 수 있다. 실차 미검증·알려진 배선 이슈는 [`src/README.md`](src/README.md)에 있다.

## 문서

- `src/` 상세(패키지별 역할, 진입점, 알려진 이슈): [`src/README.md`](src/README.md)
- `erp42_main/`: 팀 협업 이력 참고용 자료(과거 팀 정리본) — 포트폴리오 기준 소스 아님
