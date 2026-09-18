# ERP42 자율주행 (2025 대학생 창작자동차 경진대회, 팀 MTP·충남대) — Portfolio Source

> **한 줄 요약**: 예선 10분·본선 15분 단일 run 안에 차선유지·표지판/신호·정적장애물·GPS 음영구간·교차로·주차를 통과해야 하는 대회 미션 하에, 본인은 LiDAR 장애물 클러스터링(`pcl_clustering_py`, `cluster_bev`)과 Waypoint pathtracking/Controller Node(우선순위 selector)를 담당.

> **하드웨어**: ERP42 차량 플랫폼 + Velodyne LiDAR + 카메라(usb_cam) + GPS/IMU(ublox_gps·ntrip_client·vectornav 드라이버 통합, RTK).

> 버전 고정: `src/`(개인 작업본, `/home/kimkh/colcon_ws/src`) + `erp42_main/src/`(팀 정리본, `/home/kimkh/colcon_ws/erp42_main/src`). 둘 다 git repo 아님(HEAD 없음). 근거 문서: `docs/portfolio/mission-cards.md`, `docs/portfolio/tech-stack.md`, `docs/portfolio/architecture.md`, `docs/portfolio/PORTFOLIO_MAP.md`(+ `PORTFOLIO_MAP_src.md`/`PORTFOLIO_MAP_erp42_main.md`) — 전부 cross-review로 재검증됨.
>
> 검증 라벨: ✅ 검증됨(실기·실측) / 🔵 추론(근거 있으나 미확인) / ⚠️ 미검증(계산·정적분석만).

---

## 1. 문제 정의 (Why)

2025 대학생 창작자동차 경진대회(무인 모빌리티 부문)는 예선 10분·본선 15분의 **단일 자율주행 run**으로 채점된다. 실도로 미션(차선 유지, 표지판/신호 인식, 정적 장애물, GPS 음영구간 통과, 교차로, 주차)을 한 번에 통과해야 하므로, 개별 인지 모듈이 "존재"하는 것과 그 짧은 run 안에서 실제로 통합·동작하는 것은 다른 문제다.

이 프로젝트가 다룬 핵심 현장 제약:
- **GPS 음영구간(120m± 통과)**: 단일 GPS만으로는 도심/터널형 구간에서 위치추정이 끊긴다 — IMU·휠 오도메트리 융합이 없으면 경로를 잃는다.
- **다중 인지 소스의 우선순위 충돌**: 차선, 장애물, 표지판 인식이 동시에 명령을 내려는 상황에서 어느 것을 따를지 정해야 한다(대회 보고서는 3단계 selector를 제안).
- **인지 실패에도 완주해야 함**: 짧은 run 안에 카메라 인식이 실패하거나 안 걸려도 차량이 멈추거나 이탈하면 안 된다.

## 2. 담당 역할 & 기여 범위

- **팀 규모**: 2~3인(팀 MTP, 충남대).
- **대회 결과(순위, 통과 미션 수)는 사용자 결정에 따라 이 문서에 기재하지 않는다.**

담당 범위(사용자 확인, 2026-08-20 / 2026-09-17 갱신). "구현"(알고리즘·핵심 로직을 직접 작성)과 "실기 테스트·튜닝"(실차에 올려 검증하고 파라미터를 맞추는 작업)은 서로 다른 축이라 나눠 표기한다 — **실기 테스트·튜닝은 시스템 전체(EKF/비전 포함)를 본인이 담당**했고, 이 문서에서 "본인 담당"이라고 쓸 때는 아래 "구현" 열 기준이다. 팀 동료가 구현한 축은 여전히 구현 성과로 주장하지 않되, 실기 튜닝 기여와 프로젝트 전체 이해는 §3-2에 정리한다.

| 축 | 구현(알고리즘/코드) | 실기 테스트·파라미터 튜닝 | 근거 파일 |
|---|---|---|---|
| LiDAR 장애물인식/클러스터링 | **본인** | **본인** | `pcl_clustering_py/euclidean_cluster_node.py`, `cluster_bev/src/cluster_bev_node.cpp`, `erp_driver/scripts/archive/0724_*`~`0822_*`(병합 후 archive로 이동, 2026-09-18) |
| Pathtracking/제어/Controller Node | **본인** | **본인** | `erp42_pathtracking.py`, `erp42_controller.py`, `erp42_serial.py` |
| EKF 센서퓨전(localization) | 팀 동료 | **본인**(실차 파라미터 조정) | `erp42_ebimu_ekf_globalposition.py` / `erp42_imu-gps-wheel-ekf_globalposition.py` — §3-2 |
| 비전(차선 인식/YOLO) | 팀 동료 | **본인**(실차 파라미터 조정) | `0702_erp42_lanedetect*.py`, `erp42_lanedetect_yolo.py`, `yolo_ros` — §3-2 |
| RTK/GPS/IMU 드라이버 통합 | 팀 공용 인프라 | **본인**(포트/mountpoint 등 실차 설정값) | vendor 패키지 통합 수준(`ublox_gps`, `ntrip_client`, `vectornav`) |

> 🔵 추론(사용자 진술 근거, 2026-09-17): "실기 테스트는 내가 담당해서 진행했던거라 코드 이해 및 파라미터 튜닝도 진행" — `README.md`에 기록된 실차 튜닝값 변경들(카메라 device/해상도, `vectornav` port, `ntrip_client` mountpoint, `erp42_pathtracking.py`의 속도/brake 값 등, `src/` → `erp42_main/`)은 이 실기 테스트·튜닝 작업의 산출물로 본다. 알고리즘 자체의 저자성과는 별개다.

## 3. 시스템 아키텍처

전체 그래프는 `docs/portfolio/architecture.md` 참고(mermaid, 엣지별 파일:라인 근거 표 포함). 본인 담당 구간 중심 요약:

**LiDAR → 장애물 판단 경로 (두 가지 설계가 병행 실험됨)**
- *채널 방식*: `Velodyne → 0724_erp42_3DObstacle_Lam.py/0822_Obstacle_3d.py → /erp42_ctrl_cmd/lidar → erp42_pathtracking.py(brake==2 조건 검사)`. 그러나 이 세 발행 파일 어느 것도 `brake=2`를 실제로 내지 않아(각각 155/0, 200/0) **dead branch**다(`architecture.md` Edge evidence 재확인).
- *직접제어 방식*: `Velodyne → 0822_Lam_ObtAvo.py 등 → /erp42_ctrl_cmd → erp42_serial.py`(Controller/pathtracking 우회, 실선 — 살아있는 대안 경로).
- `pcl_clustering_py`/`cluster_bev`는 `/clustered_points`, `/clusters_bev`를 발행하지만 이를 구독하는 판단/제어 노드가 저장소에 없다(`architecture.md` "그래프 밖으로 분리한 topic") — **클러스터링 결과가 최종 의사결정에 연결된 근거는 현재 코드에 없다.**

**Pathtracking → 제어 경로 (src와 erp42_main이 아키텍처 세대가 다름)**
- `src/erp42_pathtracking.py`: waypoint 추종 결과를 `/erp42_ctrl_cmd/path`로 발행(`:53-56`), `erp42_controller.py`가 `lane`과 `path` 두 채널을 selector로 중재해 `/erp42_ctrl_cmd`를 최종 발행.
- `erp42_main/erp42_pathtracking.py`: Controller Node를 거치지 않고 `/erp42_ctrl_cmd`를 **직접** 발행 — 팀 최종본에서는 selector 단계가 통째로 빠졌다.

## 3-1. 결합 지점 (하나 바꾸면 같이 바뀌어야 하는 것)

| 결합 지점 | 관련 파일 | 무엇이 깨지는가 |
|---|---|---|
| `brake` 정수 sentinel 규약 | `erp42_serial.py`(해석) ↔ `erp42_pathtracking.py`(`brake==2` 검사) ↔ LiDAR variant들(`brake=155/0/200`) | 값 규약이 문서화 안 돼 있어 LiDAR variant들이 서로 다른 정지값을 쓰고, pathtracking이 기다리는 `2`는 누구도 발행 안 함 — **이 불일치를 문서화한 것 자체가 시스템 결합도 이해의 증거** |
| 카메라 topic 이름 | `usb_cam` 발행(`/camera1`,`/camera2/image_raw`) ↔ lane/YOLO 구독(`/usb_cam_0/image_raw`) | launch 리매핑과 노드 구독부가 따로 진화해 mismatch — 카메라 쪽을 바꾸면 인지 노드도 같이 바꿔야 하는데 안 됨 |
| Controller Node 유무 | `src/erp42_pathtracking.py`(Controller 경유) vs `erp42_main/erp42_pathtracking.py`(직접 발행) | 같은 파일명이지만 두 ws가 이 지점에서 아키텍처 세대가 갈림 — 병합·팀 통합 시 반드시 명시해야 하는 지점 |

## 3-2. 팀 프로젝트 기술 이해 (알고리즘 구현은 팀 동료, 실기 테스트·튜닝은 본인)

> §2 표 기준 **알고리즘/코드 구현**은 본인 담당 밖이다. 아래는 "내가 이 알고리즘을 설계했다"가 아니라 **프로젝트 전체를 이해하고 실차에서 튜닝하기 위해 코드를 읽고 파악한 내용**이며, 알고리즘 설계 성과로는 제시하지 않는다. 다만 §2 표에 정리했듯 이 모듈들의 **실기 테스트·파라미터 조정은 본인이 진행**했다(🔵 추론 — 사용자 진술 근거). 상세 근거는 `architecture.md`(topic 그래프)·`SKILL_INVENTORY.md`(§[좌표계]·§[비전] 태그 전체)·`tech-stack.md`(보고서 vs 코드 스택 대조)가 원본이다.

**EKF 센서퓨전(localization)**
- `src/`: `erp42_ebimu_ekf_globalposition.py` — GPS(`/ublox_gps_node/fix`)·IMU(`/ebimu_data`)·wheel encoder(`/erp42_status`)를 구독해 `/odom_ekf` 발행. `pymap3d.geodetic2enu`로 ENU 변환(`:126`), 고정 선형 관측행렬 `H`(`:133`) — sigma-point 생성 근거가 없어 **UKF가 아니라 선형 EKF**다.
- `erp42_main/`: `erp42_imu-gps-wheel-ekf_globalposition.py` — VectorNav IMU(`/vectornav/imu`)로 센서만 교체된 동형 구현. quaternion→yaw +90° 보정, geodetic→ENU, `map`→`base_link` odometry 발행(`:149-152,154-164,218-228`), GPS covariance 기반 update(`:154-188`). pathtracking이 발행하는 `/erp42_ctrl_cmd`를 다시 구독해 prediction feedback으로 쓰는 구조(`architecture.md` M_PATH→M_EKF edge)라는 점은 `src/`판(Controller 경유)과 다른 아키텍처 세대다.
- 실 주행 rosbag(`rosbag2_2025_10_25-16_34_43`)에 `/erp42_status`가 없어 `predict()`가 `v=0`으로만 불린다는 숨은 의존성을 코드로 확인함(2026-08-21, §6 원본 요약) — replay 실행 자체는 도구 미설치로 미시도.

**비전 — 차선 인식 / YOLO**
- `src/`: `0702_erp42_lanedetect.py` — HSV mask + Canny/Hough 고전 vision lane 검출(`:58-63,111-117,129-134`), `/erp42_ctrl_cmd/lane` 발행. 카메라 발행 topic(`/camera1,2/image_raw`)과 구독 topic(`/usb_cam_0/image_raw`)이 불일치해 입력 자체가 dead(`architecture.md` edge evidence).
- `erp42_main/`: `erp42_lanedetect.py`(HSV·morphology·CLAHE·Canny·Hough, `:115-145,324-342`) + `yolo_ros`(Ultralytics/PyTorch 기반, `LifecycleNode` configure/activate 전이, `:49-52,74-128,130-159`). `erp42_lanedetect_yolo.py`는 `Detection2DArray`(미정의) 구독 vs `DetectionArray`(실제 import) 불일치로 **`NameError` 확정** — 노드 기동 자체가 실패한다.
- 대회 기술보고서는 YOLOv8l을 주장하지만 실제 가중치 파일명(`best_lane_yolov11.pt`)과 launcher default는 YOLOv11/YOLOv8m 계열이다(`tech-stack.md` 재검증) — 이 불일치는 팀 동료 구현의 사실관계 확인이며 본인 판단이 개입된 게 아니다.

## 4. 핵심 기술 의사결정

### [모션플래닝] Waypoint pathtracking — nearest+lookahead + P제어 + 저역통과 blending
- **구현**: nearest/lookahead waypoint 선택(`erp42_pathtracking.py:84-120,128-136`, 5-waypoint window + heading gate + goal index 비감소 제약 `:118-120,161-164` 포함) + heading error 비례(P) steering(`:175`) + 이전 스텝과 `alpha=0.65` 블렌딩(`:184-187`)이 코드로 확인됨. 적분/미분 항 근거 없음(`Kp/Ki/Kd`/`PID` grep 0건 — `tech-stack.md` 확인). "pure-pursuit/PID 대신 채택"은 코드가 보여주는 구현 형태일 뿐, 당시 대안을 비교해 결정했다는 기록은 아니라 "결정"이 아닌 "구현"으로 표기한다(2026-08-29 독립 codex 재검증, `docs/plans/2026-08-29-star-review-codex.md`).
- **§0-1 기여경계**: 라이브러리 `tf_transformations`가 odometry quaternion→yaw 추출만 제공 / waypoint 탐색, heading error 계산, blending, 정지 sentinel(`brake=155`) 로직은 직접 구현.

### [안전][상태관리] Controller Node 우선순위 selector
- **결정**: `erp42_controller.py`에서 lane 채널을 path 채널보다 먼저 검사하는 `if/elif` selector + freshness timeout(`:37-46`) + 어느 입력도 유효하지 않으면 `brake=155` full-brake로 폴백(`:48-58`) 구현. 대회 보고서가 주장하는 camera/lidar/path 3단계 selector 전체는 아니고, lane/path 2채널 prototype이다.
- **§0-1 기여경계**: 라이브러리 `rclpy`가 timer/pub-sub만 제공 / 채널 유효시간, brake sentinel, 우선순위, fallback 상태 정의는 직접 설계.
- **한계(정직하게)**: 이 selector는 `src/erp42_pathtracking.py:53-57,179-190`가 발행하는 `/erp42_ctrl_cmd/path`·`brake=2`와 Controller의 `path_valid` 조건이 **topic·값 계약 수준에서는 일치**한다. 다만 두 파일 모두 `src/erp_driver/CMakeLists.txt:14-20`의 설치 대상도, 유일한 launch 파일의 실행 대상도 아니어서 "실제로 배선돼 살아있는 경로"가 아니라 **"소스 레벨 계약만 맞는 prototype"**이다 — 실행 로그·rosbag은 이번 검토 범위에 없어 확인 불가(2026-08-29 독립 codex 재검증). 팀 최종본(`erp42_main/`)에서는 Controller 파일 자체가 없어 이 selector가 통째로 빠져 있다는 것은 정확함.

### [데이터][안전] LiDAR 장애물 클러스터링/실험 variant 다수
- **결정**: `pcl_clustering_py`(Python-PCL 기반 Euclidean clustering)와 `cluster_bev`(C++, voxel+RANSAC 평면제거+clustering)를 병행 구현. 여기에 더해 `0724_*`~`0822_*` 날짜 prefix로 **직접 `/erp42_ctrl_cmd`에 발행하는 직접제어형 variant 4개**(`0724_erp42_ObstacleAvoidance.py:11`, `0822_Lam_ObtAvo.py:87-89`, `0822_Ob_bangbang3d.py:14`, `0822_ObstackeaAv_2d.py:13`)를 시도 — 기존에 "최소 5개"로 썼던 것은 숫자 인용 오류였음(2026-08-29 독립 codex 재검증으로 정정). 이 중 `0822_Lam_ObtAvo.py:29-89,140-241`는 DBSCAN clustering + centroid pairing + centerline 추출 + Pure Pursuit 형태 steering까지 갖춘, 다른 variant보다 완성도 높은 구현이다.
- **§0-1 기여경계**: Python-PCL/PCL이 point-cloud container·segmentation·clustering 알고리즘 제공 / 파라미터 튜닝, ROI 정의, brake 값 실험, ROS 노드 wiring은 직접 구현.
- **한계(정직하게)**: 여러 실험 variant 중 **팀 최종 채택본으로 명시된 단일 구현이 없다.** 클러스터링 결과(`/clustered_points`, `/clusters_bev`)를 구독하는 판단 노드도 저장소에 없어, "클러스터링→의사결정" 연결은 코드로 확인되지 않는다. 추가로 `pcl_clustering_py/euclidean_cluster_node.py:95-101,108-118`는 리스트를 빈 문자열(`''`)로 초기화한 뒤 `.append()`를 호출해 정상 입력에서도 runtime error가 나는 상태이고(`py_compile` 통과는 이 오류를 못 잡음), `0804_LiDAR_Avoidance.py:12`는 구독 topic명에 오타(`'/velo dyne_points'`), `0822_Obstacle_3d.py:93`은 list+int 연산으로 `TypeError`가 있어 — 이 variant들은 "병행 구현"보다 **"실험 소스 작성" 수준**으로 표기한다(2026-08-29 독립 codex 재검증).

### [모션플래닝] LiDAR cone-following의 bicycle 운동학 모델 기반 Pure Pursuit (2026-08-29 추가)
- **결정**: `0822_Lam_ObtAvo.py:31,57`에서 `wheelbase=1.04m`(ERP42 실측 제원)를 파라미터로 선언하고, `:130-135`에서 목표점 각도 `alpha`와 lookahead 거리 `Ld`로 표준 Pure Pursuit 조향각 공식 `steer_rad = atan(L * 2*sin(alpha) / Ld)`을 직접 구현했다 — 2-wheel bicycle 운동학 모델을 실제로 코드에 반영한 유일한 지점이다(다른 pathtracking 파일들은 heading-error 비례 steering만 사용, bicycle model 수식 없음). `:101-117`의 `_estimate_curvature_heading`으로 진행 방향 heading 변화를 곡률 추정치로 써서 속도 정책에도 연결한다.
- **§0-1 기여경계**: `numpy`/`math`가 삼각함수 연산만 제공 / wheelbase 파라미터화, Pure Pursuit 조향각 도출, curvature 추정, `max_steer_rad` 포화(saturation) 처리는 직접 구현.
- **한계(정직하게)**: 이 bicycle model은 LiDAR cone-following variant 1개에서만 쓰이고, 대회 waypoint pathtracking 본선 코드(`erp42_pathtracking.py`)는 여전히 heading-error P제어 방식이라 서로 다른 두 조향 방식이 공존한다. 이 variant가 launch/CMake에 배선돼 실행됐는지는 §4-3 한계와 동일하게 미확인.

### [통신] ERP42 command 다중 채널 설계 (`/erp42_ctrl_cmd/{lane,path,lidar,camera}`)
- **결정**: 보고서의 3-tier selector를 반영해 채널을 4개(`lane`/`path`/`lidar`/`camera`)로 **의도 설계**. 다만 네 채널을 하나의 구현에서 동시에 선언·중재하는 코드는 없고, 실제 Controller prototype도 `lane`/`path` 두 채널만 처리한다(`erp42_controller.py:16-21`) — "채널을 4개로 분리 설계"는 코드로 구현된 사실이 아니라 **보고서/의도 수준**으로 한정한다(2026-08-29 독립 codex 재검증).
- **§0-1 기여경계**: `rclpy` pub/sub가 전송 계층 제공 / 채널 분리·의미 부여는 직접 설계.
- **한계(정직하게)**: `camera` 채널은 전체 저장소에서 발행/구독 **0건**(설계만 있고 구현 안 됨), `lidar` 채널은 앞서 설명한 dead branch, `lane` 채널은 `src/`에서만 살아있고 `erp42_main/`에서는 구독자가 없는 dead topic.

### [노드설계] Controller·pathtracking·serial 역할 분리와 timer 기반 I/O
- **결정**: 경로추종은 `/erp42_ctrl_cmd/path`를 발행하고(`erp42_pathtracking.py:53-57`), Controller Node가 lane/path 입력을 구독해 20Hz timer에서 `/erp42_ctrl_cmd`를 하나만 발행하며(`erp42_controller.py:16-24,37-58`), serial Node가 이 최종 명령을 구독해 40Hz timer에서 ERP42 packet 송수신과 status 발행을 담당하도록 분리했다(`erp42_serial.py:24-27,37-46`).
- **§0-1 기여경계**: `rclpy`가 `Node`·pub/sub·timer·`spin()` 실행을 제공하고(`erp42_controller.py:8-24,60-65`; `erp42_serial.py:11-27,77-82`), 채널별 Node 분리, topic wiring, 20Hz selector와 40Hz serial I/O 주기 설정은 직접 구현했다(`erp42_controller.py:16-27`; `erp42_serial.py:24-27`).
- **한계(정직하게)**: pathtracking 제어는 별도 timer가 아니라 odometry callback 안에서 실행되고(`erp42_pathtracking.py:59-65,138-190`), 세 Node 모두 `rclpy.spin(node)`만 사용한다(`erp42_controller.py:60-65`; `erp42_pathtracking.py:204-209`; `erp42_serial.py:77-82`) — Executor·CallbackGroup을 명시한 동시성 설계는 이 경로에서 확인되지 않는다. 또한 `src/erp_driver/CMakeLists.txt:14-20`은 `erp42_serial.py` 계열만 설치 대상에 포함하고 Controller/pathtracking은 포함하지 않는다 — "노드 분리 architecture를 소스로 설계했다"와 "launch 가능한 통합 시스템을 구성했다"는 구분한다(2026-08-29 독립 codex 재검증).

> **검증 메모(2026-08-26, 독립 codex exec 재검증 — 반영 완료)**: [모션플래닝]·[안전][상태관리] 두 항목에 5-waypoint heading/cost gate·goal index 비감소 제약, `lane brake==3`/`path brake==2` sentinel 결합 디테일을 위 §4-1·§4-2 본문에 반영했다(2026-08-29).
>
> **검증 메모(2026-08-29, 독립 codex exec 2차 재검증 — 반영 완료)**: §4 전체와 §5 STAR 3건을 대상으로 file:line 재검증 + 서사 평가를 받았다(`docs/plans/2026-08-29-star-review-codex.md`). 반영한 정정: (1) LiDAR 직접제어 variant 개수 "최소 5개"→**4개**로 정정, (2) `src/`의 Controller/pathtracking selector를 "실제 배선"→**CMake/launch 미설치 상태의 소스 레벨 contract**로 하향, (3) `pcl_clustering_py`의 runtime error(빈 문자열 초기화 후 append) 및 `0804`/`0822` variant의 topic 오타·TypeError 명시, (4) STAR 1~3의 "Controller/EKF 피드백 경로 우회"(EKF도 같은 `/erp42_ctrl_cmd`를 구독하므로 오류) 및 "인지 실패에도 시스템이 무너지지 않은 구조"의 graceful-degradation 과장 표현을 아래 §5에서 재작성.

## 5. 문제 해결 사례 (STAR)

> 아래 3건은 2026-08-29 독립 codex 재검증(`docs/plans/2026-08-29-star-review-codex.md`)에서 지적된 사실 오류·과장 표현을 반영해 재작성했다. 원 버전은 "미통합·탈락 확인"을 성과처럼 서술해 채용담당자에게 변명으로 읽힐 위험이 있었고, 일부는 코드와 반대되는 서술(EKF 우회 등)을 담고 있었다.

### STAR 1 — LiDAR 회피 variant의 topic/brake 계약 불일치를 소스 레벨로 정리한 사례
- **Situation**: LiDAR 회피 코드가 채널 방식(`/erp42_ctrl_cmd/lidar` 경유)과 직접제어 방식(`/erp42_ctrl_cmd` 직접 발행)으로 나뉘어 최소 5차례(`0724`→`0822` 날짜 prefix) 재작성된 채 남아 있었다.
- **Task**: 포트폴리오에 반영하기 전, 각 variant의 topic·brake sentinel·command ownership을 소스 기준으로 재검증해 실제로 통합된 경로가 무엇인지 밝힌다.
- **Action**: `/erp42_ctrl_cmd/lidar` publisher 3개(`0724_erp42_3DObstacle_Lam.py`, `0804_LiDAR_Avoidance.py`, `0822_Obstacle_3d.py`)와 pathtracking의 `brake==2` 구독 조건(`erp42_pathtracking.py:138-146`)을 대조하고, 직접 발행 variant 4개(`0724_erp42_ObstacleAvoidance.py`, `0822_Lam_ObtAvo.py`, `0822_Ob_bangbang3d.py`, `0822_ObstackeaAv_2d.py`)가 Controller는 우회하되 EKF(`erp42_imu-gps-wheel-ekf_globalposition.py:67`)와 같은 `/erp42_ctrl_cmd` 구독자에는 그대로 전달됨을 확인했다.
- **Result**: 채널 방식 3개 어디도 `brake==2`를 발행하지 않아 pathtracking의 LiDAR 분기는 **정적 코드로 트리거 불가능**함을 확인했다(✅ 검증됨). 직접 발행 variant는 Controller를 우회하지만 EKF 구독 경로까지 우회하는 것은 아니며, 실제 위험은 "**다른 최종 command publisher(pathtracking 등)와 ownership/arbitration 없이 경쟁할 수 있다**"는 점이다. LiDAR 회피는 "최종 통합 기능"이 아니라 "복수 prototype + 미해결 integration debt"로 분류하며, 실차 회피 성능은 로그가 없어 주장하지 않는다.

### STAR 2 — 다중 command 충돌을 줄이기 위한 Controller Node arbitration prototype
- **Situation**: lane 인식 노드와 pathtracking 노드가 각자 `/erp42_ctrl_cmd` 계열 command를 만들면서, stale command나 동시 발행을 통제할 지점이 없었다(대회 보고서는 신호/표지판·장애물회피·경로추종 3단계 우선순위 Controller를 주장).
- **Task**: 최소한 lane/path 2채널이라도 하나의 지점에서 중재하고, 유효한 입력이 없을 때 안전하게 정지시키는 prototype을 만든다.
- **Action**: `erp42_controller.py:16-58`에 lane 우선 `if/elif` + 채널별 0.2초 freshness timeout + `brake==3`(lane)/`brake==2`(path) validity sentinel + 20Hz arbitration + 입력 부재 시 `brake=155` full-brake fallback을 구현하고, `src/erp42_pathtracking.py:53-57,179-190`가 `/erp42_ctrl_cmd/path`·`brake=2`를 발행해 이 Controller의 입력 계약과 맞물리도록 만들었다.
- **Result**: `src/`에서는 pathtracking output과 Controller input의 **소스 레벨 topic·값 계약**이 일치함을 확인했다(✅ 검증됨). 다만 두 파일 모두 `CMakeLists.txt`의 설치 대상도 launch 파일의 실행 대상도 아니어서 install/launch 배선과 실차 실행 근거는 없다 — "설계·구현했다"와 "대회 run에 실제로 통합됐다"를 구분한다. 팀 정리본(`erp42_main/`)은 pathtracking이 `/erp42_ctrl_cmd`에 직접 발행하고 Controller 파일 자체가 없어, 이 prototype이 최종 대회 run에 쓰였는지는 불확실하다(🔵 추론).

### STAR 3 — perception 이벤트와 무관하게 동작하는 baseline pathtracking
- **Situation**: 신호등 인식 가중치 유실, LiDAR 회피 트리거 미작동 등 인지 모듈 일부가 실전에서 빠진 상태였다(사용자 확인, 2026-08-20). 대회 주행에서 perception 기능의 보존·동작 여부가 불확실해도 기본 주행 경로는 유지돼야 했다.
- **Task**: localization·waypoint가 들어오는 동안 perception 이벤트 유무와 무관하게 기본 pathtracking command를 계속 생성하는 baseline을 본인 담당 구간(pathtracking)에서 유지한다.
- **Action**(§2 기준 본인 담당: pathtracking): `/odom_ekf`와 waypoint path를 입력으로 nearest/lookahead target을 선택하고(`erp42_pathtracking.py:84-120,128-136`), heading error 비례 steering과 `alpha=0.65` blending으로 `/erp42_ctrl_cmd`를 발행하도록 구현했다(`:175-187`). 이 경로는 perception 이벤트를 필수 입력으로 요구하지 않는다.
- **Result**: 소스상 pathtracking은 perception event 없이도 독립적으로 동작할 수 있는 구조임을 확인했다(✅ 검증됨, 구조 자체는 정적 확인). 팀 회고로는 실제 주행에서 정상 플로우가 끊기지 않고 완주까지 이어졌다고 확인되나(🔵 추론 — 실측 로그가 아닌 사용자 기억 근거), 저장소에 대회 run의 rosbag·로그가 없어 이를 "graceful degradation"이나 "인지 실패에도 시스템이 무너지지 않았다"는 정량적 robustness 성과로는 주장하지 않는다. 정확한 서술은 **"perception과 독립된 baseline pathtracking을 구현했고, 팀 회고 기준으로는 완주까지 이어졌다"**이다.

### STAR 4 — LiDAR 클러스터링 파이프라인의 런타임 결함·미연결을 발견해 통합 주장 범위를 좁힌 사례 (2026-09-16 추가)

- **Situation**: `pcl_clustering_py`/`cluster_bev`가 `/clustered_points`, `/clusters_bev`를 발행하는 LiDAR 클러스터링(본인 주 담당)을 포트폴리오에 "완성된 인지 파이프라인"으로 쓰기 전에, 실제로 정상 동작하고 판단 로직까지 연결되는지 재검증이 필요했다.
- **Task**: 코드 존재 여부가 아니라 (1) 클러스터링 코드 자체가 실제로 실행 가능한지, (2) 그 출력을 구독하는 의사결정 노드가 저장소에 있는지를 소스 기준으로 확인해야 했다.
- **Action**: `euclidean_cluster_node.py:95-101,108-118`을 직접 읽어, 리스트 변수를 빈 문자열(`''`)로 초기화한 뒤 `.append()`를 호출하는 코드가 있어 정상 입력에서도 런타임 에러가 나는 상태임을 확인했다 — `python3 -m py_compile` 문법 검증은 이 종류의 오류를 잡지 못한다는 것도 함께 확인했다. 이어서 `rg`로 `/clustered_points`, `/clusters_bev`를 구독하는 노드를 두 워크스페이스 전체에서 찾았으나 0건이었다(`architecture.md` "그래프 밖으로 분리한 topic" 표).
- **Result**: LiDAR 클러스터링을 "인지→판단까지 통합된 기능"이 아니라 "런타임 결함이 있는 실험 코드 + 다운스트림 연결 근거 없음"으로 재분류했다(✅ 검증됨). 이 검증이 없었다면 포트폴리오에 실제로는 돌지 않는 코드를 "구현 완료"로 잘못 쓸 뻔했다.
- **Competency**: 코드 존재를 성과와 동일시하지 않고 실행 가능성까지 의심하는 태도, `py_compile` 같은 얕은 검증의 한계를 알고 실제 데이터 흐름(런타임 타입, 다운스트림 구독자)까지 추적하는 검증 습관.

## 6. 정량 성과 & 한계

**대회 순위·통과 미션 수는 사용자 결정에 따라 이 문서에 기재하지 않는다.**

코드로 확인 가능한 정적 사실(전부 ✅ 검증됨 — 이번 세션에서 직접 grep/read로 재현):
- `src/` Python 116개 파일, 10,304 LOC / `erp42_main/src/` 59개 파일, 6,377 LOC (`PORTFOLIO_MAP.md` §2, cross-review로 합계 오류 정정 완료).
- LiDAR 회피 실험 variant: `/erp42_ctrl_cmd`에 직접 발행하는 것만 4개(`0724`~`0822` prefix, 2026-08-29 재검증으로 개수 정정), 최종 채택본 없음.
- Controller Node selector는 `src/`에서만 배선 확인, `erp42_main/`에서는 우회됨.

**알려진 한계(정직하게)**:
- LiDAR clustering(`pcl_clustering_py`, `cluster_bev`) 결과를 구독하는 의사결정 노드가 저장소에 없음 — 인지→판단 연결 미확인.
- `erp42_main/erp42_lanedetect_yolo.py`는 `Detection2DArray` 미정의 이름 참조로 `NameError` — 팀 최종본의 YOLO 차선 경로는 기동 자체가 실패하는 상태.
- `ntrip_client_launch.py`(erp42_main)에 RTK 인증정보가 평문으로 존재 — 포트폴리오 공개 시 마스킹 필요(`PORTFOLIO_MAP.md` §6-D).
- 실차 성능 지표(정확도%, latency, 성공률): rosbag/로그 탐색 완료(2026-08-20) — 대회 run 기록은 두 ws 어디에도 없음(팀원 PC 경로만 스크립트에 남아있음). 정량 수치 없음으로 확정.
- 다만 `rosbag_convert_clean_py`(NTRIP GPS→waypoint 생성 파이프라인, PORTFOLIO_MAP.md §6-D 근거)로 실제 HILS/실차 테스트가 있었다는 정황은 확인됨 — UKF(`robot_localization`)를 실험했다가 최종 통합본은 커스텀 선형 EKF로 대체된 것으로 추정(mission-cards.md 재확인).
- **실 주행 rosbag EKF replay 시도(2026-08-21, 부분 완료 — mission-cards.md 원본 요약, EKF 기술 상세는 §3-2)**: 사용자가 제공한 실 주행 rosbag(`rosbag2_2025_10_25-16_34_43`, 용량 제약으로 일부 센서만 녹화)의 토픽을 python sqlite3로 직접 열어 확인함(이 환경엔 `ros2 bag`/`rosbag2_py` 재생 도구 자체가 미설치라 CLI info도, 실제 replay도 불가). `/vectornav/imu`·`/vectornav/gnss`·`/ublox_gps_node/fix`·`/ebimu_data`는 있으나 **`/erp42_status`(휠 인코더)·`/erp42_ctrl_cmd`·`/scan`·`/waypoints_path`·`/odometry/filtered/global`은 전혀 없음**. **실제 replay 실행 자체는 재생 도구 미설치로 미시도** — ⚠️ 미검증. "실제환경 rosbag을 확보해 코드 레벨 한계까지 짚었으나 도구 제약으로 replay 실행은 못 했다"는 현재 상태로 남긴다(사용자가 `ros-humble-ros2bag`/`ros-humble-rosbag2-py`/`ros-humble-rosbag2-storage-default-plugins` 설치 후 재시도 가능).

## 7. 역량 태그 요약

**본인 기여로 주장 가능한 축** (§2 담당 범위 기준):
[통신] [노드설계] [모션플래닝] [상태관리] [안전] [데이터]

**같은 코드베이스에 존재하지만 본인 주 담당 아님**(§2 표 기준, 기술 이해는 §3-2):
[좌표계](EKF의 ENU 변환) [비전](차선/YOLO) [인프라](센서 드라이버 통합)

[시뮬]은 접점 낮음(Docker/ABI bridging은 vendor 패키지 수준), [HRI]는 해당없음(`Pick_Tray_SA.py`는 ERP42 프로젝트와 무관한 별개 파일로 확인됨 — `PORTFOLIO_MAP.md` §6-D).

## 8. 기반 기술 요소 (본인 담당 구간 중심)

| 요소 | 접점 | 근거 | 검증 |
|---|---|---|---|
| 센서 전력·대역폭 | Velodyne VLP16, 600rpm, 고정 IP `192.168.1.201`(`VLP16-velodyne_driver_node-params.yaml:3,10-12`) | LiDAR 담당 영역 | 🔵 |
| 레이턴시 최적화 | `erp42_serial.py` 40Hz timer(`:27`) — pathtracking/controller 담당 영역의 제어 주기 | ✅ (코드 확인) | 실측 latency는 ⚠️ 미검증 |
| CPU 최적화 | `cluster_bev_node.cpp`의 voxel leaf size 파라미터(`:28,69`)로 point cloud 다운샘플링 | LiDAR 담당 영역 | 🔵 |
| 하드웨어 매뉴얼 기반 환경구성 | ERP42 시리얼 프로토콜(`/dev/ttyUSB0`, 115200 baud), Velodyne UDP/IP 설정 | 직접 구성 | ✅ (코드 확인) |
| 언어·런타임 | LiDAR 계열은 C++(`cluster_bev`)/Python(`pcl_clustering_py`) 혼재, pathtracking/controller는 전부 Python | 직접 확인 | ✅ (코드 확인) |

## 9. Future Work / 심화 학습 계획

- **LiDAR 클러스터링→의사결정 연결**: 지금은 클러스터링 결과를 구독하는 판단 노드가 없다. 클러스터 중심점을 장애물 목록으로 변환해 pathtracking에 실제로 연결하는 게 다음 단계.
- **brake sentinel 값 규약 문서화·통일**: 여러 LiDAR variant가 서로 다른 정지값(155/0/200)을 써서 어느 것도 pathtracking의 `brake==2` 조건과 안 맞는다 — 하나의 규약으로 정리.
- **Controller Node 3-tier 완성**: `camera` 채널(신호/표지판)이 설계만 있고 구현이 전혀 없다 — YOLO 신호등/표지판 인식과 연결하면 보고서가 주장한 설계를 실제로 완성할 수 있다.
- **표지판 인식·배달·주차 미션**(mission-cards.md Future Work 재사용): 대회에서 미시도, 기존 YOLO 파이프라인·Controller Node 설계 재사용 여지 있음.
- **erp42_lanedetect_yolo.py NameError 수정**: `Detection2DArray`→`DetectionArray`로 바로잡으면 팀 최종본의 YOLO 차선 경로가 기동 가능해짐.

## 10. 전공 기반 매핑

| 전공 | 코드에 실제로 닿은 과목 | 이 프로젝트의 실제 지점 (근거) |
|---|---|---|
| **제어공학** | 비례(P) 제어, 저역통과 필터링, bicycle 운동학 모델 | waypoint pathtracking 본선 경로는 heading error P steering + `alpha=0.65` blending 직접구현(`erp42_pathtracking.py:175-190`, **내 몫**), 적분/미분 항은 미사용. 별도로 LiDAR cone-following variant(`0822_Lam_ObtAvo.py:130-135`)에서는 `wheelbase` 기반 bicycle 운동학 모델로 Pure Pursuit 조향각을 직접 도출함(**내 몫**, 2026-08-29 추가) — PID 이론은 여전히 **심화 학습 여지** |
| **CS/컴퓨터공학** | 상태머신, ROS2 통신 설계 | Controller Node의 lane/path selector + freshness timeout + fallback(`erp42_controller.py:37-58`, **내 몫**). Executor/CallbackGroup/Lock 등 동시성 설계는 이 담당 구간에서 미확인 — **심화 학습 여지** |
| **기계공학/로보틱스** | Point cloud 기하(Euclidean clustering, RANSAC 평면제거) | `pcl_clustering_py`/`cluster_bev`가 PCL 알고리즘 호출(**경계**) / ROI·파라미터 튜닝·ROS 노드 구성은 **내 몫**. Voxel downsampling·시각화도 직접 구성 |
| **전자공학** | 센서·통신 인터페이스 | ERP42 시리얼 프로토콜(40Hz packet, `erp42_serial.py`), Velodyne UDP/IP 구성 — **구성 수준, 회로 설계 아님** |

> 커버리지 요약: 제어공학·CS는 pathtracking/Controller Node 담당 구간에서 직접 구현 비중이 높다(내가 짠 코드). 기계공학은 **PCL 라이브러리가 핵심 알고리즘을 대신**하고 내 몫은 통합·튜닝이다(경계 표시). 좌표계(선형대수/ENU 변환)는 §2 표 기준 EKF 담당 구간이라 본인 전공 매핑에는 포함하지 않았다 — 팀 동료 구현에 대한 기술적 이해는 §3-2 참고.

---

## 확신도
확신도: 검증됨(코드 grep/read + cross-review 재검증, `PORTFOLIO_MAP.md`/`mission-cards.md`/`tech-stack.md`/`architecture.md` 근거 + 2026-08-29 독립 codex 2차 재검증 `docs/plans/2026-08-29-star-review-codex.md` 반영) + 사용자 확인(팀 규모, 담당 범위, 대회결과 비공개 결정, 인지모듈 실패에도 완주한 사실).
내가 채워넣은 가정: STAR 3(baseline pathtracking 사례)의 Action을 본인 담당(pathtracking) 중심으로 재서술하고 EKF는 팀 담당으로 한정했다. "완주"는 사용자 기억 근거(🔵 추론)이며 rosbag/로그로 뒷받침되지 않는다는 점을 Result에 명시했다.
2026-08-29 추가 반영: (1) `0822_Lam_ObtAvo.py`의 `wheelbase` 기반 bicycle 운동학 모델·Pure Pursuit를 §4·§10에 신설 — 이에 따라 §10 제어공학 행의 기존 "pure-pursuit 곡률제어는 미사용" 서술을 정정. (2) `mission-cards.md`의 2026-08-21 "실 주행 rosbag EKF replay 시도(부분완료)" 항목을 §6에 요약 반영 — EKF는 팀 담당이라 본인 성과로 주장하지 않음.
2026-09-17 추가 반영(사용자 요청): §2를 산문에서 담당 범위 표로 정리하고, 문서 곳곳에 흩어져 있던 "EKF/비전은 팀 담당이라 본인 성과 아님" 반복 문구를 §2 표 참조로 통합했다. 그 자리에 있던 실질 정보(코드 근거)는 삭제하지 않고 새 §3-2 "팀 프로젝트 기술 이해"로 옮겨, EKF·비전 소스에 대한 코드 읽기 수준 기술 설명을 추출했다(성과로는 주장하지 않음 — 사용자가 명시적으로 이 구분을 원함). `CLAUDE.md`(project)의 "팀 동료 담당 영역" 정의는 원본 그대로 두고 수정하지 않았다.
확인 요청: 없음.
