# erp42_main (협업 이력 참고용)

팀이 GitHub 공유용으로 정리했던 ERP42 코드의 과거 시점 사본이다. 최종본도, 실차 배포 스냅샷도 아니다. **이 저장소의 공개 기준은 [`../src`](../src/README.md)** 이고, 두 ws의 차이 요약은 [상위 README](../README.md)의 "저장소 구성" 절에 있다.

## 핵심 파일 & 코드 흐름

| 역할 | 파일 |
|---|---|
| Localization(EKF) | `src/erp_driver/scripts/erp42_imu-gps-wheel-ekf_globalposition.py` |
| Pathtracking | `src/erp_driver/scripts/erp42_pathtracking.py` |
| Actuation | `src/erp_driver/scripts/erp42_serial.py` |
| 차선 인식 | `src/erp_driver/scripts/erp42_lanedetect.py`, `erp42_lanedetect_yolo.py`, `src/yolo_ros/` |
| Command 중재(Controller) | *(없음 — pathtracking이 `/erp42_ctrl_cmd`를 직접 발행, `../src`에만 존재)* |
| LiDAR 장애물인식 | *(없음 — `../src` 전용 영역)* |

코드 흐름(요약): `GPS/IMU/encoder → erp42_imu-gps-wheel-ekf_globalposition → erp42_pathtracking → erp42_serial`. Controller 없이 pathtracking이 직접 발행하는 것이 `../src`와 가장 큰 구조 차이다.

## 패키지 구성 (모두 `src/` 하위, colcon 표준 레이아웃)

| 패키지 | 역할 |
|---|---|
| `erp_driver` | ERP42 시리얼 드라이버 + 정리 시점에 남긴 제어/경로추종/차선인식 스크립트 |
| `erp_interfaces` | ERP42 메시지 인터페이스 |
| `waypoint` | Waypoint GPS 데이터(`.xls`) — `erp_driver`에서 분리됨 |
| `ntrip_client` | RTK 보정 NTRIP 클라이언트 |
| `ublox`, `ublox_gps`, `ublox_msgs`, `ublox_serialization` | u-blox GPS 드라이버 (`../src`의 `ublox/` 서브패키지들이 최상위로 분리 배치됨, 내용 동일) |
| `usb_cam` | USB 카메라 드라이버 (`params_1`~`4` — 카메라 최대 4대) |
| `vectornav` | VectorNav IMU/INS/GNSS 드라이버 |
| `vectornav_msgs` | `vectornav`가 의존하는 커스텀 메시지 패키지(`../src/vectornav/vectornav_msgs`에서 복사) |
| `yolo_ros` | YOLO 추론 ROS2 래퍼 (가중치 `*.pt`는 gitignore라 저장소에 없음) |
| `rosbag_convert_clean_py`, `rosbag2csv.py` | rosbag → GPS/odom/waypoint CSV 후처리 도구 |

## `src/` 대비 이 정리본에서 달라진 것
- `erp_driver/scripts`의 날짜 prefix 실험 스크립트(`0610_...`~`1024_...`) 중 미채택분 삭제, 채택분만 이름 정리.
- `erp42_pathtracking.py`: 퍼블리시 토픽 `/erp42_ctrl_cmd`로 통일, 속도 하드코딩 제거(`self.max_linear_speed`로 파라미터화), `brake` 값 조정(2→1).
- `usb_cam`: 카메라 장치·해상도·노출 설정을 실차 값으로 교체, 카메라 2대→4대 설정 추가.
- `ntrip_client`: mountpoint 기본값을 `RTK-RTCM31`로 변경.
- `vectornav`: `port`를 `/dev/ttyUSB0`으로 교체.
- LiDAR/localization 실험 스택(hdl_localization, fast_gicp, velodyne 등)은 이 정리본의 범위 밖.

## 한계 · 미완성
1. **빌드 미검증**: 이 ws는 `colcon build`로 확인하지 않았다.
2. **차선 경로 미연결**: camera→차선인식 경로는 topic 불일치로 동작하지 않는다(정적 분석 기준).
3. **LiDAR 분기 미연결**: pathtracking의 LiDAR 분기는 이 ws에 발행자가 없다.

<details><summary>정리 이력</summary>

- `vectornav_msgs`를 `../src/vectornav/vectornav_msgs`에서 복사해 추가 — `vectornav/CMakeLists.txt`의 `find_package(vectornav_msgs REQUIRED)` 해석용.
- 전 패키지를 `erp42_main/src/` 하위로 이동 — colcon 기본 스캔 경로(`<ws>/src`)에 맞춤.
- 두 조치 모두 정적 분석(코드/구조) 기준이며, 그 밖의 빌드 에러가 남아 있을 가능성은 배제하지 못한다.
</details>
