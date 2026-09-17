# colcon_ws (개인 작업 ws)

ERP42 자율주행 + LiDAR/localization 실험 스택을 함께 담은 개인 작업 워크스페이스.
최종본은 [`erp42_main/`](../erp42_main) — 두 ws 간 차이·병렬 관리 이유는 [상위 README](../README.md) 참고.

## 담당 범위 & 핵심 참조 파일

이 ws의 `erp_driver/scripts/`가 실질적인 진입점이다. 담당 축(구현/실차 튜닝 구분)은 [`../docs/portfolio/PORTFOLIO_SOURCE.md`](../docs/portfolio/PORTFOLIO_SOURCE.md) §2가 단일 출처 — 요약:

| 역할 | 파일 | 담당 |
|---|---|---|
| Localization(EKF) | `erp_driver/scripts/1024_EBIMU_EKF.py` | 팀 동료(구현) / 본인(실차 튜닝) |
| 차선 인식 | `erp_driver/scripts/0702_erp42_lanedetect.py` | 팀 동료(구현) / 본인(실차 튜닝) |
| LiDAR 장애물인식 | `pcl_clustering_py/`, `cluster_bev/`, `erp_driver/scripts/0724_*`~`0822_*` | **본인** |
| Pathtracking | `erp_driver/scripts/erp42_pathtracking.py` | **본인** |
| Command 중재(Controller) | `erp_driver/scripts/0702_erp42_controller.py` | **본인** |
| Actuation | `erp_driver/scripts/erp42_serial.py` | **본인** |

코드 흐름(이 ws 한정, 요약): `GPS/IMU/encoder → 1024_EBIMU_EKF → erp42_pathtracking → 0702_erp42_controller(lane/path 중재) → erp42_serial`. camera→차선인식, LiDAR→pathtracking 경로는 topic 불일치/미발행 조건으로 정적으로는 dead — 상세 근거는 [`../docs/portfolio/architecture.md`](../docs/portfolio/architecture.md).

## 패키지 구성

### ERP42 드라이버 스택
| 패키지 | 역할 |
|---|---|
| `erp_driver` | ERP42 시리얼 드라이버 + 제어/경로추종/차선인식 스크립트 (`scripts/`) |
| `erp_interfaces` | ERP42 메시지 인터페이스 |
| `ntrip_client` | RTK 보정 NTRIP 클라이언트 |
| `ublox` (`ublox`/`ublox_gps`/`ublox_msgs`/`ublox_serialization` 서브패키지) | u-blox GPS 드라이버 |
| `usb_cam` | USB 카메라 드라이버 (params_1/2 = 카메라 2대 설정) |
| `vectornav` (`vectornav`/`vectornav_msgs` 서브패키지) | VectorNav IMU/INS/GNSS 드라이버 (upstream repo 원본 구조) |
| `ebimu_pkg` | EBIMU IMU 드라이버 |

### LiDAR / Localization 실험 스택 (erp42_main에는 없음)
| 패키지 | 역할 |
|---|---|
| `velodyne` | Velodyne LiDAR 드라이버 |
| `hdl_localization` | NDT/GICP 기반 3D localization |
| `hdl_global_localization` | 전역 초기 위치 추정 |
| `ndt_omp` | OpenMP 가속 NDT scan matching |
| `fast_gicp` | GPU/CPU 가속 GICP scan matching |
| `pcl_clustering_py`, `cluster_bev` | PCL 기반 포인트클라우드 클러스터링 |
| `robot_localization` | EKF/UKF 센서 퓨전 |
| `Yolo_pt` | YOLO 가중치 저장소(`best.pt`, `last.pt`) |

## `erp_driver/scripts` 정리 상태
날짜 prefix가 붙은 반복 실험 스크립트가 다수 누적되어 있음(`0610_...`, `0702_...`, `0724_...`, `0730_...`, `0804_...`, `0822_...`, `1024_...`). 이 중 최종 채택되어 `erp42_main`으로 넘어간 것은 날짜 prefix 없는 버전(`erp42_imu-gps-wheel-ekf_globalposition.py`, `erp42_lanedetect.py`, `erp42_lanedetect_yolo.py`, `erp42_pathtracking.py`) 뿐이며, 나머지 LiDAR 장애물회피 계열(`0724_*`, `0730_*`, `0804_*`, `0822_*`)은 이 ws에만 남은 미채택 실험본.

## ⚠️ 확인 필요 (미검증)
- `vectornav`가 upstream repo 그대로(`vectornav`+`vectornav_msgs` 서브패키지, `.git` 포함) 들어와 있음 — 실제로 이 ws에서 `colcon build`가 통과하는지 미검증.
- `erp_driver/scripts` 중 어떤 실험본이 실제로 실차에서 마지막으로 검증됐는지는 파일명(날짜)만으로 추정한 것이며 git log/실행 이력으로 확인하지 않음.
