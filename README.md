# ERP42 자율주행 워크스페이스

2025 대학생 창작자동차 경진대회 출품용 ERP42(Wego Robotics 4륜 전기차) 자율주행 스택이다. GPS·IMU·엔코더로 위치를 추정하고, 미리 기록한 GPS waypoint 경로를 추종하도록 ERP42에 조향·속도 명령을 보낸다. ROS 2 Humble, 대부분 Python(rclpy).

> 출처: 팀 MTP(충남대) 팀 프로젝트(실개발 2~3인)의 대회 코드. 대회 뒤 이 저장소에서 바뀐 것은 README 정리와 `src/` 배선 버그 일부 수정이다(내용은 [`src/README.md`](src/README.md)).

> **핵심 설계**: 주행 명령을 내는 노드(경로추종·차선)가 시리얼로 직접 쓰지 않는다. 각자 `/erp42_ctrl_cmd/<출처>`로 발행하고, `erp42_controller.py` 한 곳이 최근 0.2초 안에 들어온 유효 명령 하나를 골라 `/erp42_ctrl_cmd`로 넘긴다. 유효 명령이 없으면 Controller가 brake=155 정지 명령을 낸다. 출처를 바꿔 끼워도 시리얼 노드는 코드상 그대로 쓰도록 나뉘어 있다.

```
ublox GPS ─ /ublox_gps_node/fix ─┐
EBIMU ───── /ebimu_data ─────────┼─▶ erp42_ebimu_ekf_globalposition ─ /odom_ekf ─┐
erp42_serial ─ /erp42_status ────┘          (선형 EKF, ENU)                     │
                                                                                ▼
waypoint xls ─▶ erp42_pubwaypointscnuservice_pymap3d ─ /waypoints_path1 ─▶ erp42_pathtracking
                                                                                │ /erp42_ctrl_cmd/path (brake=2)
(실험) camera ─▶ erp42_lanedetect ─ /erp42_ctrl_cmd/lane (brake=3) ┐            ▼
                                                                   └─▶ erp42_controller ─ /erp42_ctrl_cmd ─▶ erp42_serial ─▶ ERP42
```

차선 인식(lane)은 실험 단계 코드다. 본인 기억상 대회 주행은 path 추종만 쓴 것으로 추정 — lane 최종 채택 여부는 미확인(07-04 실도로 동시 구동 테스트 기록은 있음). LiDAR 장애물 회피는 실험 이력(`archive/`)으로만 남아 있고 현재 경로에는 연결돼 있지 않다.

## 무엇을 할 수 있나
| 사용자가 하는 일 | 시스템이 하는 일 |
|---|---|
| 주행할 경로의 GPS 좌표를 xls(`Longitude`, `Latitude` 열)로 준비 | 원점 기준 ENU 좌표로 바꿔 `nav_msgs/Path` 발행 |
| `/set_origin` 서비스로 원점(위경도) 지정 | EKF와 waypoint 노드가 그 원점 기준으로 ENU 변환(두 노드 모두 같은 원점이 필요 — 서비스 이름 중복 문제는 아래 "실행"·"한계") |
| 노드를 차례로 띄움 | lookahead 3 m 지점을 목표로 조향을 계산해 ERP42에 40 Hz로 송신하고, 마지막 waypoint 1.5 m 안에 들어오면 정지 명령 |

## 시스템 구조
현재 쓰는 경로는 `src/erp_driver/scripts/`에서 날짜 prefix가 없는 파일들이다.

- **측위** — `erp42_ebimu_ekf_globalposition.py`: 상태 `[x, y, θ]` 3개짜리 선형 EKF(NumPy로 직접 구현). sigma-point가 없으니 UKF는 아니다. 엔코더 속도와 조향(`/erp42_status`)으로 예측하고, GPS 위치(pymap3d `geodetic2enu`, x·y만)로 갱신한다. 발행하는 heading은 EKF의 θ가 아니라 EBIMU yaw(−90° 보정 + offset)로 바꿔 넣는다. 결과는 `/odom_ekf`(frame `map`→`base_link`)로 20 Hz 발행. 차량 상수(`wheel_base` 1.040, `wheel_radius` 0.265, `encoder_ticks_per_rev` 100)와 노이즈는 `declare_parameter`로 노출돼 있다.
- **경로** — `erp42_pubwaypointscnuservice_pymap3d.py`: xls → `/waypoints_path1`. `erp42_pathtracking.py`: 가장 가까운 waypoint에서 lookahead 지점을 고르고, heading 오차로 조향을 계산한다(최대 28°). 속도는 `speed=15`로 고정이고, `brake=2`를 붙여 `/erp42_ctrl_cmd/path`로 발행한다.
- **중재** — `erp42_controller.py`(20 Hz): ① lane 명령(brake==3, 0.2 s 이내) ② path 명령(brake==2, 0.2 s 이내) ③ 둘 다 아니면 정지(speed 0, brake 155). `brake` 값이 출처 표시 역할을 겸하므로 새 출처를 붙일 때도 이 값을 지켜야 Controller가 받아들인다.
- **구동** — `erp42_serial.py`: `/erp42_ctrl_cmd`를 ERP42 패킷(`STX` … `\r\n`)으로 40 Hz 송신하고, 18바이트 응답을 `/erp42_status`(speed·steer·brake·encoder)로 발행한다.
- **차선(실험)** — `erp42_lanedetect.py`(HSV/gradient + LDA, `/usb_cam_0/image_raw` 구독), `erp42_lanedetect_yolo.py`(`/yolo/detections` 구독, `yolo_ros` 필요).

| 구분 | 파일·패키지 |
|---|---|
| 현재 경로 | `erp42_ebimu_ekf_globalposition.py`, `erp42_pubwaypointscnuservice_pymap3d.py`, `erp42_pathtracking.py`, `erp42_controller.py`, `erp42_serial.py`, `ebimu_pkg` |
| 실험(lane) | `erp42_lanedetect.py`, `erp42_lanedetect_yolo.py`, `yolo_ros` |
| 실험·이력 | `scripts/archive/` 15개(LiDAR 회피·EKF·차선 이전 버전), `erp42_pathtracking_lidar_integrated.py`(미배선), `pcl_clustering_py`·`cluster_bev`(클러스터링 결과만 발행하고 Controller엔 lidar 입력이 없음), `lanedetect.py`·`claude_lanedetect.py`·`test*.py`(ROS 노드 아닌 영상 실험) |
| upstream 사본 | `velodyne`, `pcl_ros`, `hdl_localization`, `hdl_global_localization`, `ndt_omp`, `fast_gicp`, `robot_localization`(현재 경로에서는 쓰지 않음), `ublox`, `ntrip_client`, `usb_cam`, `vectornav`, `yolo_ros` |

## 환경 · 장비
- Ubuntu 22.04 + ROS 2 Humble, Python 3. GPU는 YOLO(실험 경로)에만 필요.
- 장비(코드 기본값 기준, 실제 장치 번호는 연결 순서에 따라 바뀜):

| 장비 | 드라이버 | 설정 |
|---|---|---|
| ERP42 | `erp42_serial.py` | `port` 파라미터 — `erp42_base.launch.py`로 띄우면 `/dev/ttyUSB0`(코드 기본값 `/dev/ttyUSB1`은 launch 없이 실행할 때만). 실제 장치명에 맞게 launch를 고친다. 115200 |
| u-blox GPS + NTRIP RTK | `ublox_gps`, `ntrip_client` | NTRIP 계정은 환경변수 `NTRIP_USERNAME`/`NTRIP_PASSWORD`(`.env.example`) |
| EBIMU | `ebimu_pkg/ebimu_publisher.py` | 포트를 실행 시 `input()` 프롬프트로 입력 |
| 카메라(실험) | `usb_cam` | `params_1~4.yaml` |
| Velodyne VLP-16(실험) | `velodyne` | — |

## 저장소 구성
```
colcon_ws/
├── src/            # 공개 기준 ws — 위 흐름은 모두 여기 기준 (상세: src/README.md)
│   ├── erp_driver/      # 시리얼 드라이버 + 측위·경로·Controller 스크립트(scripts/)
│   ├── erp_interfaces/  # ErpCmdMsg, ErpStatusMsg, SetOrigin.srv
│   ├── ebimu_pkg/       # EBIMU → /ebimu_data
│   ├── waypoint/        # waypoint xls 2개
│   └── …                # 센서 드라이버·LiDAR 실험·upstream 사본
├── erp42_main/     # 팀 협업 이력 참고용 정리본(과거 시점). 공개 기준 아님
├── scripts/        # 루트에 남은 단독 실험 스크립트(EKF·신호등)
└── .env.example    # NTRIP 인증정보 템플릿
```
저장소에 없는 것: YOLO 가중치(`src/Yolo_pt/*.pt`, 파일당 131 MB라 gitignore — 필요하면 따로 받아야 함), `.env`(직접 만듦), 주행 rosbag.

`erp42_main/`과의 차이(협업 이력 참고): `erp42_main`에는 Controller가 없어서 pathtracking이 `/erp42_ctrl_cmd`로 바로 발행하고(brake=1, `max_linear_speed` 사용), LiDAR·localization 실험 스택이 없다. 자세한 내용은 [`erp42_main/README.md`](erp42_main/README.md).

## 설치
```bash
source /opt/ros/humble/setup.bash
# Python 의존(실행 PC에서 공급자 확인 필요): pyserial, numpy(<2.0), pymap3d, pandas+xlrd(.xls), tf_transformations, opencv(apt python3-opencv), scikit-learn
colcon build --symlink-install --base-paths src --packages-select erp_interfaces erp_driver ebimu_pkg
```
- `--base-paths src`: 루트에 `erp42_main/src`도 있어서, 경로를 주지 않으면 같은 이름의 패키지가 두 번 잡힌다.
- `--packages-select`로 나눠 빌드한다. `src` 전체를 한꺼번에 빌드하면 `ament_lint_auto`/`ament_cmake_cppcheck` 미설치와 `ndt_omp` 자체 문제로 일부 패키지가 실패한다(2026-09-18 기록).
- numpy 2.x는 Humble cv_bridge와 맞지 않는다. opencv는 pip가 아니라 apt로 설치한다.

## 실행
⚠️ 미검증 — 아래 순서는 코드의 토픽·서비스 연결에서 읽어낸 것이다. 대회 당시의 실제 실행 절차 기록은 아니다.

```bash
# 터미널마다: source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch erp_driver erp42_base.launch.py                  # ERP42 시리얼 (/erp42_status 발행 시작)
ros2 launch ublox_gps ublox_gps_node-launch.py               # GPS → /ublox_gps_node/fix
set -a; source .env; set +a; ros2 launch ntrip_client ntrip_client_launch.py   # RTK 보정
ros2 run ebimu_pkg ebimu_publisher                           # 프롬프트에 포트 입력(예: USB2)
python3 src/erp_driver/scripts/erp42_ebimu_ekf_globalposition.py   # → /odom_ekf
python3 src/erp_driver/scripts/erp42_pubwaypointscnuservice_pymap3d.py  # xls 경로는 파일 안 excel_path를 고쳐서 지정
# 두 노드(EKF·waypoint)에 같은 원점이 필요하지만 /set_origin 서버가 두 노드에 같은 이름으로 있어,
# 한 번 호출로 양쪽 설정이 끝났는지 확인할 수 없다 — 두 노드 로그에 원점 설정 메시지가 모두 찍히는지 볼 것
ros2 service call /set_origin erp_interfaces/srv/SetOrigin "{longitude: <lon>, latitude: <lat>}"
python3 src/erp_driver/scripts/erp42_pathtracking.py         # → /erp42_ctrl_cmd/path
python3 src/erp_driver/scripts/erp42_controller.py           # → /erp42_ctrl_cmd (이때부터 차량에 명령이 나감)
```
- `erp_driver`는 시리얼 스크립트 3개만 설치한다(`CMakeLists.txt:15`). 그래서 측위·경로·Controller는 `ros2 run`이 아니라 `python3 <경로>`로 실행한다.

**실기 위험·정지**
- `erp42_serial.py`는 마지막으로 받은 `/erp42_ctrl_cmd`를 새 명령이 올 때까지 40 Hz로 계속 보낸다. **Controller가 죽거나 멈추면 직전 명령이 그대로 유지된다** — 코드상 시리얼 쪽 timeout은 없다.
- 상위 노드가 끊긴 경우는 Controller가 0.2 s 뒤 정지 명령(brake 155)으로 바꾼다. 이 동작은 Controller가 살아 있을 때만 해당한다.
- 끄는 순서 — **path 추종만 실행한 경우**: ① `erp42_pathtracking` 먼저 종료 → Controller가 0.2 s 뒤 정지 명령으로 전환 ② `ros2 topic echo /erp42_status`에서 speed 0, brake 155가 보이는지 확인 ③ Controller 종료 ④ 시리얼 종료. 실험 lane 노드(`erp42_lanedetect*.py`)도 띄웠다면 Controller가 lane 명령을 path보다 먼저 채택하므로, ①에서 **모든 명령 발행원**(pathtracking·lane)을 끈 뒤 ②로 정지를 확인한다. **Controller를 먼저 끄면 시리얼이 직전 주행 명령을 계속 보낸다.** 비상 시에는 차체 비상정지/수동 전환을 쓴다.

## 검증
- 빌드(2026-09-18 기록, `--packages-select` 개별): `erp_driver` `erp_interfaces` `ntrip_client` `ublox_serialization` `ublox_msgs` `vectornav_msgs` `ublox_gps` PASS / `usb_cam` `vectornav` FAIL(`ament_lint_auto` 미설치). 이번 README 정리 때는 다시 돌리지 않았다.
- 자동 로직 테스트는 없다(`ebimu_pkg`·`pcl_clustering_py`의 `test/`는 copyright·flake8·pep257 린트뿐).
- 실기 성능 검증은 하지 않았다(대회 뒤 수정분 포함). 대회 주행 성능 수치도 이 저장소에 남아 있지 않다.

## 한계 · 미완성
- 두 노드(EKF·waypoint)에 같은 원점이 필요하지만 `/set_origin` 서비스 서버가 **두 노드에 같은 이름으로** 있어, 한 번 호출로 양쪽 설정 완료를 확인할 수 없다(ROS 2에서 같은 이름의 서버가 둘이면 요청이 어느 쪽으로 가는지 정해져 있지 않다).
- waypoint xls 경로가 작성자 PC 절대경로로 하드코딩돼 있다(`erp42_pubwaypointscnuservice_pymap3d.py:86`). 실행 전에 고쳐야 한다.
- pathtracking 속도는 `speed=15`로 고정이다. `max_linear_speed=50`은 선언만 있고 쓰이지 않는다.
- LiDAR 연동 분기는 동작하지 않는 dead branch다. pathtracking이 `/erp42_ctrl_cmd/lidar`(Controller가 아니라 pathtracking이 구독하는 별도 토픽)에서 brake==2를 받으면 waypoint index를 앞당기도록 돼 있지만, 현재 경로에는 이 토픽의 발행자가 없고 `archive/`에만 있다. Controller의 brake==2 판정은 `/erp42_ctrl_cmd/path` 토픽에만 적용된다.
- 차선 경로는 카메라 토픽이 맞지 않는다. 노드는 `/usb_cam_0/image_raw`를 구독하는데 `usb_cam` launch는 `camera1/image_raw`로 remap한다. 실험 경로이며, 본인 기억상 대회 주행은 path 추종만 쓴 것으로 추정 — lane 최종 채택 여부는 미확인(07-04 실도로 동시 구동 테스트 기록은 있음).
- 파라미터가 하드코딩된 곳(pathtracking 상수, waypoint 경로, EBIMU 포트 입력)이 남아 있다. QoS도 기본 depth 값을 쓴다.

## License
루트 LICENSE는 없다. 각 패키지의 `package.xml` license 항목과 upstream 패키지의 `LICENSE`를 따른다. 팀 코드는 `erp_driver`·`erp_interfaces`·`cluster_bev`가 Apache-2.0으로 선언돼 있고, `ebimu_pkg`·`pcl_clustering_py`는 미지정(TODO)이다.

## 더 읽을 문서
| 문서 | 내용 | 지위 |
|---|---|---|
| [`src/README.md`](src/README.md) | 패키지별 역할, 대회 뒤 수정 내역, 빌드 기록, 알려진 문제 | 정본 |
| [`erp42_main/README.md`](erp42_main/README.md) | 과거 팀 정리본의 구성과 차이 | 참고 이력 |
