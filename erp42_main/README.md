# erp42_main (최종본 ws)

CNU M.T.P 팀 ERP42 자율주행 대회용 최종 코드. 개인 작업 ws는 [`../src`](../src/README.md) —
두 ws 간 차이는 [상위 README](../README.md) 참고.

> 참고: 최상위 `README.md`가 원래 `ublox` 패키지 README(upstream 문서)를 그대로 담고 있었음.
> 그 내용은 [`src/ublox/README.md`](src/ublox/README.md)로 옮기고 여기는 프로젝트 개요로 새로 작성함.

## 핵심 파일 & 코드 흐름

이 ws는 팀이 GitHub에 공유용으로 정리해 올린 부분집합이다(왜 `src/`와 병렬로 관리되는지는 [상위 README](../README.md) 참고).

| 역할 | 파일 |
|---|---|
| Localization(EKF) | `src/erp_driver/scripts/erp42_imu-gps-wheel-ekf_globalposition.py` |
| 차선 인식 | `src/erp_driver/scripts/erp42_lanedetect.py`, `erp42_lanedetect_yolo.py`, `src/yolo_ros/` |
| LiDAR 장애물인식 | *(패키지 자체 없음 — `src/`(개인 ws) 전용 영역, 여긴 미포함)* |
| Pathtracking | `src/erp_driver/scripts/erp42_pathtracking.py` |
| Command 중재(Controller) | *(파일 자체 없음 — pathtracking이 `/erp42_ctrl_cmd`를 직접 발행, `src/`에만 존재)* |
| Actuation | `src/erp_driver/scripts/erp42_serial.py` |

코드 흐름(이 ws 한정, 요약): `GPS/IMU/encoder → erp42_imu-gps-wheel-ekf_globalposition → erp42_pathtracking → erp42_serial`(Controller 없이 직접 발행 — `src/`와 가장 큰 아키텍처 차이). camera→차선인식 경로는 topic 불일치로 정적으로는 dead, LiDAR→pathtracking 경로는 이 ws에 publisher 자체가 없어 dead branch다.

## 패키지 구성 (모두 `src/` 하위, colcon 표준 레이아웃)

| 패키지 | 역할 |
|---|---|
| `erp_driver` | ERP42 시리얼 드라이버 + 최종 채택된 제어/경로추종/차선인식 스크립트 |
| `erp_interfaces` | ERP42 메시지 인터페이스 |
| `waypoint` | Waypoint GPS 데이터(`.xls`) — `erp_driver`에서 분리됨 |
| `ntrip_client` | RTK 보정 NTRIP 클라이언트 |
| `ublox`, `ublox_gps`, `ublox_msgs`, `ublox_serialization` | u-blox GPS 드라이버 (개인 ws의 `ublox/` 서브패키지들이 최상위로 분리 배치됨, 내용 동일) |
| `usb_cam` | USB 카메라 드라이버 (`params_1`~`4` — 카메라 최대 4대) |
| `vectornav` | VectorNav IMU/INS/GNSS 드라이버 |
| `vectornav_msgs` | `vectornav`가 의존하는 커스텀 메시지 패키지 — 원래 이 ws에 누락되어 있던 것을 `../src/vectornav/vectornav_msgs`에서 복사해 추가함 |
| `yolo_ros` | YOLO 추론 ROS2 래퍼 (+ `best_lane_yolov11.pt` 가중치) |
| `rosbag_convert_clean_py`, `rosbag2csv.py` | rosbag → GPS/odom/waypoint CSV 후처리 도구 |

## 개인 ws 대비 최종본에서 정리된 것
- `erp_driver/scripts`의 날짜 prefix 실험 스크립트(`0610_...`~`1024_...`) 중 미채택분 삭제, 채택분만 이름 정리.
- `erp42_pathtracking.py`: 퍼블리시 토픽 `/erp42_ctrl_cmd`로 통일, 속도 하드코딩 제거(`self.max_linear_speed`로 파라미터화), `brake` 값 조정(2→1).
- `usb_cam`: 카메라 장치·해상도·노출 설정을 실차 값으로 교체, 카메라 2대→4대 설정 추가.
- `ntrip_client`: mountpoint 기본값을 `RTK-RTCM31`로 변경.
- `vectornav`: `port`를 `/dev/ttyUSB0`으로 교체.
- LiDAR/localization 실험 스택(hdl_localization, fast_gicp, velodyne 등)은 미포함 — 최종본 범위 밖.

## 이번에 수정한 것 (2026-08-20)
- 🔧 `vectornav_msgs` 패키지 추가: `../src/vectornav/vectornav_msgs` → `src/vectornav_msgs`로 복사. `vectornav/CMakeLists.txt`의 `find_package(vectornav_msgs REQUIRED)`가 해석 안 되던 문제 해소.
- 🔧 `src/` 래퍼 추가: 이전엔 모든 패키지가 `erp42_main/` 루트에 바로 있어 colcon 기본 스캔 경로(`<ws>/src`)와 어긋났음. 전 패키지를 `erp42_main/src/` 하위로 이동.
- ⚠️ 미검증: `colcon build`를 실제로 돌려서 확인하지 않았음 — 위 두 조치는 정적 분석(코드/구조) 기준 수정이며, 이 외의 빌드 에러(다른 의존성 누락 등)가 남아있을 가능성은 배제 못 함.
