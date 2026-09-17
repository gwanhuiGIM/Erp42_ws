# 미션 시나리오별 구현 상태 — 미션공지 vs 기술보고서 vs 실제 코드

> 근거 문서:
> - 미션공지: `2025 대학생 창작자동차 경진대회 미션공지_무인모빌리티_0812_ver2.pdf`
> - 기술보고서(2025-09-27 제출): `무인_충남대_MTP.pdf` (팀명 MTP, 충남대)
> - 코드: `src/`(개인 ws) + `erp42_main/src/`(팀 정리본) — [비교 README](../../README.md), [포트폴리오 계획](../plans/2026-08-20-portfolio-extraction.md) 참고
>
> 대회는 **10분(예선 Highway) / 15분(본선) 내 한 번의 자율주행 run**으로 채점되므로, 개별 스크립트가 "존재"하는 것과 "그 10~15분 run에 실제로 통합돼 동작"하는 것은 다른 문제다. 이 문서는 그 둘을 구분해서 표시한다.

## 범례
- ✅ **구현+통합됨**: 코드 존재 + 최종 pathtracking/controller에 실제로 연결되어 있음 (dead topic 아님)
- ⚠️ **구현만 됨(미통합/불완전)**: 코드는 있지만 최종 실행 경로에 연결 안 됨, 또는 일부만 됨
- ❌ **미시도**: 대회에서 시도 자체를 안 함(사용자 확인)
- 🗑️ **구현했으나 유실**: 사용자 기억상 구현·시도했으나 코드/가중치가 현재 두 ws에 안 남아있음(사용자 확인)
- 📄 **보고서 주장 ≠ 코드**: 기술보고서 서술과 실제 코드 구조가 다름 (포트폴리오에서 절대 그대로 베끼면 안 되는 항목)

---

## 0. 센서/아키텍처 (보고서 §1~§2 vs 코드)

| 항목 | 보고서 주장 | 코드 확인 | 상태 |
|---|---|---|---|
| 카메라 3대 용도 분리 | ① 상향고정=신호등(빨강/초록/좌우회전 인식) ② 하향=차선감지 ③ 우향고정=표지판 인식 | `usb_cam` 4개 설정(camera1~4) 중 실제 launch 배선된 건 camera1/camera2뿐(camera3/4는 launch 없이 고아 설정) — 카메라 3대라는 보고서 수치와 대체로 부합하나, 어느 physical 카메라가 신호등/차선/표지판 중 무엇인지는 **여전히 소스코드로 재구성 불가**(CLI 배선, 이전 조사에서 확인) | ⚠️ 보고서로 "의도"는 확인됐지만 "실제 배선"은 코드로 재현 불가 |
| YOLOv8l로 신호등+표지판+차선 통합 인식 (Roboflow 데이터셋, Google Colab 학습) | `2.1.a`, Fig 26~28 | 코드베이스엔 `best_lane_yolov11.pt`(차선용, YOLOv**11**) 하나만 있고 YOLOv8l 가중치 파일 자체가 안 보임. 다만 개인 ws `src/Yolo_pt/best.pt`,`last.pt`가 이 Colab 학습 산출물(`runs/detect/train/weights/{best,last}.pt`)일 가능성이 높음(파일명 패턴 일치) | 📄 **보고서(YOLOv8l) ≠ 저장소의 명명(YOLOv11)** — 버전이 안 맞음. 최종 채택 버전이 무엇인지 확인 필요 |
| LiDAR ROI + PointCloud2 클러스터링으로 장애물 인식 | `2.1.b`, Fig 29 | 개인 ws의 `pcl_clustering_py`, `cluster_bev`, `0730_LiDAR_ObjectDetect.py` 등과 정확히 일치하는 접근. 다만 이 로직들은 **erp42_main(팀 최종본)엔 없고 개인 ws에만 있음** | ⚠️ 설계 방향은 일치하나 "팀 최종 채택본"으로 명시된 단일 구현이 없음(여러 실험본 중 무엇이 최종인지 미확정) |
| UKF(Unscented Kalman Filter, 7 시그마포인트)로 GPS+IMU+ERP42 속도/조향 융합 | `2.2`, Table 3 — sigma point 생성→운동학 모델 전파→예측 측정치↔GPS 비교로 상태 보정 | 실제 코드(`1024_EBIMU_EKF.py`, `EkfGlobal` 클래스)는 **시그마포인트 생성이 없는 선형 EKF**: `predict()`는 결정론적 운동학 모델 1개 경로로만 전파, `cb_gps()`는 고정 `H=[[1,0,0],[0,1,0]]` 선형 관측모델로 칼만이득 계산. UKF의 핵심인 "시그마포인트 7개 생성→비선형 전파→가중평균 복원" 단계가 코드에 없음. **다만 사용자 확인(2026-08-20) 후 재조사한 정황 증거로, UKF를 실제로 실험은 했던 것으로 보인다**: `erp42_main/src/rosbag_convert_clean_py/odom_convert`가 참조하는 하드코딩 경로가 `.../bagfiles_ros2/test5/feedback2_analyzed/odom_ukf_ctrl11.csv`(파일명에 "ukf"), `plot_waypoint_csv`가 참조하는 경로는 `.../bagfiles_ros2/test3_UKF_Pathtrack/spinning_far/waypoints_path.csv`(디렉토리명이 "UKF_Pathtrack")다. 두 경로 다 팀원 PC(`/home/mrlam/...`)를 가리켜 실제 bag 파일은 이 저장소에 없지만, **`robot_localization` 패키지(vendor, `ukf_node` executable 보유)로 UKF 기반 pathtracking을 실험했다가 최종 통합본에서는 커스텀 선형 EKF로 대체된 것으로 추정된다.** | 📄 **보고서(UKF) ≠ 최종 통합 코드(선형 EKF)** — 포트폴리오에 "UKF 구현"이라고 쓰면 안 됨. "GPS 공분산 가중 EKF, UKF는 실험 단계에서 시도 후 최종엔 미채택"이 코드 근거로 쓸 수 있는 정확한 표현 |
| Controller Node(우선순위 selector: ①신호/표지판 ②장애물회피 ③경로추종) | `2.3`, Table 4 — `/erp42_ctrl_cmd/{lidar,camera,path}` 세 채널을 별도 Controller Node가 중재 | 이 설계와 정확히 일치하는 프로토타입이 `0702_erp42_controller.py`(개인 ws, 7/2일자)로 존재 — `/erp42_ctrl_cmd/path`+`/erp42_ctrl_cmd/lane` 구독, `/erp42_ctrl_cmd` 발행. **개인 ws판 `erp42_pathtracking.py`는 실제로 이 Controller Node에 물리도록 배선돼 있었다**(최종 명령을 `/erp42_ctrl_cmd/path`로 발행 L55, `brake=2` L189 — 이게 정확히 `0702_erp42_controller.py`의 `path_valid` 조건과 맞물림). **그런데 erp42_main(팀 최종본)의 동일 파일은 `/erp42_ctrl_cmd`로 직접 발행(L55)+`brake=1`(L189)로 바뀌어 있어 Controller Node를 완전히 우회한다** — 즉 두 ws의 `erp42_pathtracking.py`는 겉보기엔 같은 파일이지만 이 지점에서 아키텍처 세대가 다르다. `erp42_main`판이 유일하게 검사하는 `current_lidar.brake==2` 분기(L142)도 실행 내용은 "정지"가 아니라 **가장 가까운 waypoint 인덱스로 건너뛰는 것**(`current_goal_index` 갱신)뿐이고, `stop_robot()`/`speed=0` 호출이 없다. 게다가 이 `brake==2`를 실제로 발행하는 LiDAR 노드가 저장소에 하나도 없다(`0804_LiDAR_Avoidance.py`는 brake=0 고정, `0724_erp42_3DObstacle_Lam.py`는 155/0, `0822_Obstacle_3d.py`는 200/0 — 셋 다 2를 쓰지 않음) — **이 조건은 현재 코드로는 절대 참이 될 수 없는 죽은 분기다.** `/erp42_ctrl_cmd/camera` 토픽은 전체 저장소에 발행/구독 **0건**, `/erp42_ctrl_cmd/lane`은 발행자(`erp42_lanedetect_yolo.py`,`erp42_lanedetect.py`)만 있고 **구독자가 없는 dead topic** | 📄+⚠️ **보고서의 3단계 우선순위 중 "신호/표지판" 티어가 최종 통합본에 빠져 있고, "장애물회피" 티어도 팀 최종본에서는 트리거 자체가 안 걸리는 죽은 코드다.** Controller Node 통합 시도는 개인 ws 한 시점엔 실제로 존재했으나 팀 최종본에서 버려짐 |
| (신규 발견, 2026-08-21) Controller Node 우회형 회피 통합 시도 | 보고서엔 없는 별도 후보 | Google Drive에서 원본 파일명 `trafficlight_stop_ros2.py`로 발견된 파일을 실제로 읽어보니 신호등 로직은 0줄이고, `DynamicWaypointNavigator` 클래스로 `/scan`(LaserScan) 섹터 클리어런스 기반 좌/우 회피 + `/waypoints_path`+`/odometry/filtered/global` pathtracking을 **Controller Node 없이 단일 노드로 통합**한 구조. Controller Node 우회 자체는 팀 최종본 `erp42_pathtracking.py`(위 행 참고, `/erp42_ctrl_cmd` 직접 발행)와 같은 방향이지만, 이 노드는 `brake==2` 죽은 분기 대신 LiDAR sector 회피를 아예 노드 내부에 직접 구현해 문제를 다른 방식으로 우회하려던 것으로 보임. `src/erp_driver/scripts/erp42_pathtracking_lidar_integrated.py`로 정직한 이름으로 배치함(원본은 `scripts/trafficlight_stop_ros2.py`). **launch 파일(`erp_driver/launch/`) grep 결과 이 노드를 참조하는 곳 없음 — 최종 통합본에 실제로 배선됐다는 근거 없음.** `python3 -m py_compile` 문법 검증만 통과(✅), 실행/실기 검증은 안 됨 | ⚠️ 미검증 — 존재는 확인되나 실제 채택 여부 불명. 향후 grep 기반 조사 시 "신호등" 키워드로 파일명을 신뢰하면 오탐 유발한다는 반면교사 사례로 남김(이 문서의 "정지선/신호등 키워드 grep 0건" 서술은 여전히 유효 — 이 파일은 배치 시 신호등 키워드를 포함하지 않도록 리네임했음) |
| "차선 감지" 하향 카메라(보고서 Fig 18) 역할 | `2.1.a` — "차선 감지(차선인식 주행, 정지선 감지, 주차 차선 감지 등)"이라고 명시(하나의 카메라로 차선+정지선+주차선을 겸함) | ⚠️ **사용자 정정(2026-08-20)**: 이 카메라의 실제 용도는 코드에 남은 것처럼(`LaneCenteringNode`, HSV Hough) "차선 중앙유지 steering"이 아니라 **정지선 인식** 쪽이었던 것으로 기억함. 그런데 두 ws 어디에도 정지선/횡단보도 관련 키워드(`stopline`, `crosswalk`, 가로선 폭/종횡비 판별 등)를 가진 코드가 없음(grep 0건) — `erp42_lanedetect_yolo.py`/`erp42_lanedetect.py`/`lanedetect.py`/`claude_lanedetect.py` 4개 파일 모두 좌우 차선 중앙유지 로직으로만 구성됨 | 🗑️+📄 **정지선 인식 코드 자체가 유실된 것으로 추정.** 현재 남아있는 lanedetect 계열 코드는 보고서가 말하는 "정지선 감지" 기능이 아니라 별개의(혹은 더 이른 시점의) 차선 중앙유지 시도로 보임 — 포트폴리오에 "정지선 인식 구현"을 쓰려면 코드 근거 없이 사용자 기억에만 의존해야 함 |

## 1. 예선 (Track Drive + Highway 주행)

| # | 미션 (미션공지 기준) | 상태 | 근거 |
|---|---|---|---|
| Track Drive | 라바콘 코스 반복 주행(직선+곡선), 라바콘 접촉 시 수동 재정렬 | ⚠️ | waypoint pathtracking(`erp42_pathtracking.py`)으로 경로 자체는 주행 가능하나, "라바콘 접촉 감지" 전용 로직은 없음 — 접촉은 사람이 육안으로 판단해 수동 개입하는 대회 규정이라 코드 불필요할 수도 있음(미검증) |
| 사선주차(6칸 중 3칸) | ❌ 미시도(사용자 확인) | 사선주차 전용 코드 없음. waypoint 좌표만 사선 주차칸에 정확히 찍으면 pathtracking으로 진입 가능하다는 전제인데, "정차 3초 유지" 같은 세부 조건을 처리하는 로직은 안 보임 |
| Highway 진입 | ✅(경로추종 범위) | waypoint pathtracking으로 커버 가능한 범주. 별도 로직 불필요 |
| U-Turn(공사구간 표지판 인식 후 라바콘 유도경로 U턴) | ❌ 미시도(사용자 확인, 표지판 인식 범주) | "공사중" 표지판 인식 코드 없음. waypoint로 U턴 경로 자체는 그릴 수 있으나, **표지판 인식이 전제조건인 미션이라 인식 없이는 조건 미충족** |
| 톨게이트 통과 | ⚠️ | 표지판 인식 없이 waypoint로 물리적 통과는 가능하나, 미션 자체가 "표지판 인식 후" 조건이 걸려 있어 정식 충족은 불확실 |
| GPS 음영구간(120m±α) 통과 | ✅ | `1024_EBIMU_EKF.py`(개인 ws, EBIMU 기반)의 GPS 공분산 가중 보정이 대응 메커니즘(위 표 참고) — 코드로 확인됨. **erp42_main(팀 최종본)에도 구조적으로 동일한 EKF가 있다**(`erp42_imu-gps-wheel-ekf_globalposition.py` — 같은 `predict()`/`H=[[1,0,0],[0,1,0]]` GPS 보정, encoder odometry 보정과 adaptive Q까지 추가된 더 완성된 버전. 단 IMU 소스는 `/ebimu_data` 문자열 파싱이 아니라 `/vectornav/imu`, L69) — 이전 조사에서 "erp42_main엔 EKF가 없다"고 적은 건 EBIMU 파일명만 grep하고 vectornav 버전을 놓친 오조사였음(크로스 리뷰로 확인, 2026-08-20) |
| (신규, 2026-08-21) 실 주행 rosbag(`rosbag2_2025_10_25-16_34_43`)으로 EKF replay 검증 | — | 사용자가 제공한 실 주행 rosbag(용량 때문에 일부 센서만 녹화됨, `.db3` 2.5GB)을 python sqlite3로 직접 열어 토픽 확인(이 환경엔 `ros2 bag`/`ros2bag`/`rosbag2_py`가 설치돼 있지 않아 CLI info 불가, `dpkg`로도 `ros-humble-rosbag2-cpp`/`-storage`만 있고 `ros-humble-ros2bag`은 없음을 확인). bag엔 `/vectornav/imu`(2117), `/vectornav/gnss`, `/ublox_gps_node/fix`(104), `/ebimu_data`(std_msgs/String, 1053) 등은 있으나 **`/erp42_status`(휠 인코더), `/erp42_ctrl_cmd`, `/scan`, `/waypoints_path`, `/odometry/filtered/global`은 전혀 없음**(sqlite topics 테이블 전수 확인). `colcon build --symlink-install --base-paths src --packages-select erp_interfaces erp_driver`는 PASS(7.69s, `docs/src`가 `../src` 심볼릭 링크라 base-paths 미지정 시 중복 패키지 에러 발생 — 참고용으로 남김). 팀 최종본 `erp42_imu-gps-wheel-ekf_globalposition.py` 코드를 직접 읽어 확인한 결과: `/vectornav/imu`+`/ublox_gps_node/fix`만으로도 `timer_callback`(20Hz)이 `/odom_ekf`를 계속 발행하긴 하나(publish_odom이 origin_set 여부와 무관하게 매 틱 호출됨, L190-201), `/erp42_status`가 없으면 `predict()`가 항상 `v=0`으로만 불려(L195) **위치(x,y)가 원점에서 전혀 전파되지 않는다** — 즉 이 bag만으로는 "정지 상태로 GPS만 흡수하는" 것 이상은 검증 불가. 추가로 GPS 절대보정 경로 자체가 `/set_origin` 서비스 서버(별도 노드, 이 bag엔 당연히 없음 — 서비스는 recording되지 않음)가 응답해야 `origin_set=True`가 돼 작동한다는 **기존 문서에 없던 숨은 의존성**도 코드로 확인함(L154-161). **실제 replay 실행은 이 환경에 `ros2 bag play`(또는 `rosbag2_py`) 재생 도구가 없어 시도하지 못함 — ⚠️ 미검증.** 원인 확정(2026-08-21 재조사): `apt-cache search`로 `ros-humble-ros2bag`(0.15.16) 패키지 자체는 apt 저장소에 존재함이 확인됨(=미설치이지 unavailable이 아님). `dpkg -l`로 재확인한 결과 `ros-humble-rosbag2-cpp`/`ros-humble-rosbag2-storage`(C++ 라이브러리, 노드가 링크하는 것)만 깔려있고, CLI 진입점인 `ros-humble-ros2bag`과 그 의존 패키지 `ros-humble-rosbag2-py`(Python 바인딩), `ros-humble-rosbag2-storage-default-plugins`(sqlite3 스토리지 플러그인)는 **셋 다 미설치** — 이래서 `ros2 bag` verb 자체가 CLI에 등록 안 됨. `sudo apt install`을 시도했으나 이 세션이 비대화형이라 `sudo: a password is required`로 실패(사용자가 터미널에서 직접 설치해야 함) — ⚠️ 설치 자체도 미완료. 설치 명령: `sudo apt install ros-humble-ros2bag ros-humble-rosbag2-py ros-humble-rosbag2-storage-default-plugins` 후 재시도하면 최소한 "크래시 없이 발행되는지"는 확인 가능할 것으로 보이나 위치 전파 자체는 이 bag으로는 원천적으로 검증 불가(휠 인코더/제어명령 토픽 부재) |
| GPS 음영구간 내 장애물 인식 후 통과 | ❌ | 팀 최종본(`erp42_main`)의 `erp42_pathtracking.py`가 검사하는 `current_lidar.brake==2` 분기는 (a) 실행 내용이 "정지"가 아니라 waypoint 인덱스 스킵이고 (b) 이 값을 실제로 발행하는 LiDAR 노드가 저장소에 없어 **현재 코드로는 트리거될 수 없는 죽은 조건**이다(위 0번 표 Controller Node 행 참고). "정지는 가능"이라던 이전 판정을 정정함 |
| GPS 음영구간 내 정적장애물 | ❌ | 위와 동일 사유로 정정 |

## 2. 본선 (배달 + 교차로 신호/비신호 + 정적장애물 + 주차)

| # | 미션 | 상태 | 근거 |
|---|---|---|---|
| 배달A(Pick-up, A1/A2/A3 표지판 번호 인식 + 5초 정지) | ❌ 미시도(사용자 확인) | 표지판 숫자(A1/A2/A3) 인식 코드 없음. 배달 관련 코드는 `rosbag_convert_clean_py`의 `gps_convert.py`/`plot_waypoint_csv` 등 **후처리 도구뿐**, 실시간 인식·정지 로직 없음 |
| 교차로 직진/좌회전/우회전(신호 有) | 🗑️ 구현했으나 유실(사용자 확인) | 사용자 기억상 신호등(빨강/초록) 인식은 **실제로 구현·시도했음** — 다만 그 학습 가중치(.pt)가 유실된 것으로 추정. 코드베이스엔 신호등 인식 관련 스크립트·가중치가 전혀 안 남아있어(grep 0건) 어떤 방식(YOLO 클래스 분류 vs 색상 HSV)이었는지, Controller Node의 camera 채널과 연결됐었는지는 코드로 재구성 불가 |
| 교차로 우회전(비신호, 서행 후 횡단보도 3초 정지) | 🗑️/❌ 불명확 | 위 "정지선 인식" 논의와 동일 사안(0번 표 참고) — 사용자는 정지선 인식 로직이 있었다고 기억하나 코드가 안 남아있어 이 미션에 실제 적용됐는지는 미확인 |
| 정적 장애물(대형/소형) | ❌ | 위 GPS 음영구간 장애물 항목과 동일 사유(`brake==2` 죽은 조건) — 정정됨 |
| 배달B(Delivery) | ❌ 미시도(사용자 확인) | 배달A와 동일 사유 |
| 수평주차(3초 정지 후 탈출) | ❌ 미시도(사용자 확인) | 전용 코드 없음(사선주차와 동일 사유 — waypoint로 물리 이동만 가능, 정지시간·정렬 조건 로직 미확인) |

---

## 종합 판정

### 센서퓨전을 통한 상호보완 — 특정 트리거 부재에도 완주 (사용자 확인, 2026-08-20)
신호등/정지선처럼 특정 인지 트리거가 없거나 유실된 상황에서도, **실제 주행에서는 정상 플로우가 끊기지 않고 완주까지 이어졌다**(사용자 확인). 이건 우연이 아니라 아키텍처 설계 자체가 뒷받침하는 결과로 코드에서 근거를 찾을 수 있다:

- 보고서 `2.3` Controller Node 설계(Table 4)의 세 번째 분기가 원래 **"If no events, follow waypoints - lane detection"** — 즉 신호/장애물 같은 특정 트리거가 없으면 시스템이 실패하는 게 아니라 **기본값으로 waypoint 경로추종에 안전하게 폴백(fallback)**하도록 설계돼 있음. 신호등 인식 가중치가 유실된 상태로 달렸어도, "이벤트 없음→경로추종 지속"이라는 이 폴백 경로 자체는 실제 채택된 `erp42_pathtracking.py`에도 그대로 살아있어(경로추종이 항상 기본 동작), 인지 실패가 곧바로 정지/이탈로 이어지지 않는 구조였다.
- 여기에 **GPS+IMU+휠오도메트리 EKF(`1024_EBIMU_EKF.py`)의 공분산 가중 보정**이 위치추정 쪽을 받쳐줘서, 카메라 인지가 하나 빠지더라도 "지금 어디에 있는가"는 계속 안정적으로 계산됨 — 즉 **인지(perception) 한 채널이 비어도, 측위(localization) 채널이 무너지지 않아 차량이 경로를 잃지 않았다.**
- LiDAR 장애물 대응(`lidar_callback`, `brake==2`)은 팀 최종본 기준 **실제로는 트리거될 수 없는 죽은 조건**이라는 게 크로스 리뷰로 확인됐다(위 0번 표 참고) — 따라서 이 부분을 "장애물 인지가 없어도 경로추종이 계속 작동한 안전망"으로 서술하는 건 부정확하다. 정확히는: **장애물 회피 코드가 애초에 걸리지 않는 상태였는데도 경로추종 자체(waypoint following)는 독립적으로 계속 돌아갔기 때문에** 완주가 가능했다 — "장애물 대응이 보완했다"가 아니라 "장애물 대응이 아예 개입 안 했는데도 경로추종만으로 버텼다"는 게 더 정확한 서술이다.

**즉 "인지(신호등/정지선/장애물회피) 모듈이 각자 100% 완성되지 않았거나 아예 안 걸렸어도, 측위(EKF)가 무너지지 않고 waypoint 경로추종이 기본값으로 계속 동작해서 전체 시스템이 완주할 수 있었다"**는 것이 포트폴리오에 정직하게 쓸 수 있는 서술이다. "여러 모듈이 서로 보완했다"기보다는 **"측위+경로추종이라는 최소 기반이 튼튼해서, 그 위의 인지 모듈들이 빠지거나 안 걸려도 시스템 전체가 깨지지 않았다"**에 가깝다 — 이건 "신호등 인식이 있었어도 없었어도 상관없다"는 뜻이 아니라, **미완성/미작동 모듈이 있어도 시스템이 우아하게 성능 저하(graceful degradation)하도록 설계됐다**는 의미로 서술해야 정확하다. 신호 미션 자체의 정식 통과 여부(패널티 부과 등)는 별개 문제로 남는다.

**"코드로 확실히 완성"이라고 포트폴리오에 쓸 수 있는 것:**
1. GPS+IMU+휠오도메트리 센서퓨전(EKF, GPS 음영구간 대응) — 단 "UKF"가 아니라 "EKF" + "공분산 가중 보정"으로 정확히 서술. 팀 최종본(vectornav 버전)·개인 ws(EBIMU 버전) 둘 다 같은 알고리즘으로 존재
2. Waypoint 기반 경로추종 — 단 "PID"가 아니라 **"heading error 비례(P) 제어 + 이전 스텝과의 저역통과 블렌딩(`alpha=0.65`)"**으로 정확히 서술(적분/미분 항 없음, `Kp/Ki/Kd` grep 0건). pymap3d 좌표변환은 확인됨
3. 차선 인식 **시도** — HSV 기반(`lanedetect.py` 등) + YOLO 기반(`erp42_lanedetect_yolo.py`) 두 접근. 단 팀 최종본의 YOLO판은 `Detection2DArray`를 import 없이 참조하는 `NameError`로 **노드 기동 자체가 실패하는 상태**라(L21) "완성"이 아니라 "미완성/미검증"으로 톤을 낮출 것
4. 인지-측위 간 graceful degradation — 단 "LiDAR 애드온이 보완했다"가 아니라 "장애물 대응 코드가 걸리지 않았는데도 경로추종만으로 완주했다"로 정정 서술(위 상세 참고)

**"구현·시도했으나 코드/가중치가 유실된 것" (사용자 기억 근거, 코드 재현 불가 — 포트폴리오엔 "구현 경험 있음, 현재 코드 미보존"으로 정직하게 쓸 것):**
1. 신호등 색상 인식(빨강/초록/좌우회전 판단)
2. 정지선 인식(교차로 우회전 시 횡단보도 정지 등에 쓰였을 것으로 추정, 사용자 기억)

**"설계/보고서엔 있지만 최종 통합 코드에서 빠져 있거나 확인 안 되는 것":**
1. Controller Node의 3단계 우선순위 중 "신호/표지판" 티어 (`/erp42_ctrl_cmd/camera` 토픽 0건)
2. UKF(보고서 주장) — 실제론 선형 EKF

**"미시도" (사용자 확인 — 아예 대회에서 시도하지 않음):**
1. 표지판 숫자 인식(배달 A/B, U-turn 공사표지판)
2. 배달 미션(Pick-up/Delivery)
3. 사선주차·수평주차

## Future Work (사용자 확인, 2026-08-20)
표지판 인식·배달 미션·주차 미션은 이번 대회에서 시도하지 않은 영역으로, **포트폴리오에는 "다음 확장 과제(Future Work)"로 명시**한다 — 이미 있던 인프라(YOLO 파이프라인, Controller Node 설계, waypoint pathtracking)를 재사용하면 구현 여지가 있는 시나리오임:
- 표지판 숫자 인식(YOLO 커스텀 클래스 추가로 확장 가능 — 기존 Roboflow+Colab 학습 파이프라인 재사용)
- 배달 A/B Pick-up/Delivery(표지판 인식 + 정차 시퀀스 결합)
- 사선/수평 주차(정렬 판정 + 정차시간 로직)

사용자는 추후 별도 에이전트(Codex 등)와 함께 이 시나리오들의 알고리즘 설계·구현을 진행할 예정 — 이 문서의 "미시도" 항목이 그 착수 지점이 된다. (운영 규칙 출처: `~/vault/ai/MULTI_AGENT_POLICY.md` — 여러 ws가 공유하는 단일 출처로 전역 CLAUDE.md에 반영됨)

## 크로스 리뷰 이력 (2026-08-20)
독립 에이전트(cross-review)가 문서 전체를 소스코드와 재대조해서 5건을 정정함(모두 직접 재확인 완료):
1. `brake==2` LiDAR 정지 조건이 실제로는 발행자가 없는 죽은 분기임을 발견 → "LiDAR 장애물 정지 통합"을 ✅/⚠️에서 ❌로 하향
2. "erp42_main엔 EKF가 없다"는 서술이 오조사(EBIMU 파일명만 grep, vectornav 버전 EKF를 놓침)였음을 발견 → GPS 음영구간 대응을 더 강하게(양쪽 ws 확인) 재서술
3. "PID"라고 쓴 게 실제론 P제어+저역통과 블렌딩뿐임을 발견 → 서술 정정
4. 팀 최종본 `erp42_lanedetect_yolo.py`가 import 누락으로 즉시 크래시하는 상태임을 발견 → "완성" 판정 하향
5. "erp42_pathtracking.py 양쪽 ws 공통"이라는 서술이 두 ws 간 실질적 아키텍처 차이(Controller Node 배선 여부)를 지웠음을 발견 → 차이를 명시하도록 재서술

## 확신도
확신도: 검증됨(grep·코드 직독, 크로스 리뷰로 재검증) + 사용자 확인(신호등 구현 경험/가중치 유실, 차선중앙유지 아닌 정지선 인식이었을 것, 표지판·배달·주차 미시도) — 다만 정지선 인식이 정확히 어떤 미션 시나리오에 쓰였는지, 신호등 인식이 Controller Node와 실제로 연결됐었는지는 코드가 안 남아있어 여전히 미확인.
내가 채워넣은 가정:
1. `src/Yolo_pt/best.pt`/`last.pt`가 보고서의 YOLOv8l Colab 학습 산출물이라고 추정(파일명 패턴 근거, 실제 모델 아키텍처 미확인)
2. 현재 남은 lanedetect 계열 코드(차선 중앙유지)와 사용자가 기억하는 "정지선 인식"이 서로 다른 시점/다른 파일이라고 추정 — 정지선 코드는 유실됐다고 가정
확인 요청: 없음 (표지판/배달/주차는 Future Work로 명확히 확정됨)
