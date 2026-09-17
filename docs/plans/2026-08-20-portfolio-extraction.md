# 포트폴리오 요소 추출 계획

> 목적: ERP42(Wego Robotics 4륜 전기차) 기반 자율주행 대회(실도로 미션: 차선/표지판/장애물/신호/GPS 재밍 터널) 프로젝트의 소스코드에서 포트폴리오에 쓸 기술 요소를 추출한다.
> 대상: `src/`(개인 작업 ws, LiDAR·localization 포함) + `erp42_main/`(팀 정리본, 제어·비전·GPS 중심). 두 ws는 상호 보완적이라 **미션 5개를 커버하려면 둘 다 참조해야 함** ([상위 비교 README](../../README.md) 참고).

## 0. 원칙 (전역 CLAUDE.md §6-1 적용)
- 코드에 있는 것만 "구현함"으로 쓴다. **실행/로그로 확인 못 한 성능 수치(정확도%, 처리속도 fps, 성공률)는 포트폴리오에 넣지 않거나 "추정/미검증"으로 명시**한다 — 지어내지 않는다.
- 대회 결과(순위, 통과 미션 수)는 소스코드로 알 수 없다. 이건 사용자에게 별도로 물어봐야 하는 항목이다(1번 확인 요청).
- 원본 GitHub repo가 private이므로, 포트폴리오에 코드 링크/스니펫을 실을 때 공개 가능 범위(전체 repo 공개? 스니펫만 발췌?)를 사용자에게 확인한다.

## 1. 미션 → 소스 매핑 (사전 조사 결과)

| 미션 | 대응 패키지/스크립트 | 확인 상태 |
|---|---|---|
| 차선 유지 | `erp_driver/scripts/erp42_lanedetect.py`, `erp42_lanedetect_yolo.py`, `lanedetect.py`, `claude_lanedetect.py` (HSV 색공간 기반), `best_lane_yolov11.pt` (YOLOv11 커스텀 가중치) | 확인됨 — 2가지 접근(고전 HSV vs YOLO) 병행 확인, 둘의 관계(대체/보조)는 코드 내부 확인 필요 |
| 경로 추종/제어 | `erp_driver/scripts/erp42_pathtracking.py`, `erp42_pubwaypointscnuservice_pymap3d.py` | 확인됨 — pymap3d 좌표변환 + waypoint 기반 pathtracking |
| GPS/IMU/휠 오도메트리 센서퓨전 | `erp42_imu-gps-wheel-ekf_globalposition.py`(vectornav 기반, 초기형) → **`1024_EBIMU_EKF.py`(EBIMU 기반, 개인 ws에만 존재, erp42_main엔 없음)로 대체**: vectornav의 drift·센서퓨전 난이도 문제로 EBIMU(`ebimu_pkg`가 시리얼 CSV를 `/ebimu_data`로 퍼블리시)로 전환. `cb_ebimu`에서 quaternion 필드 파싱→yaw 추출, `cb_status`(휠 엔코더)로 predict, `cb_gps`에서 GPS `position_covariance` 기반 칼만 게인으로 보정 | 확인됨 — 코드 구조까지 확인. `ebimu_pkg` 자체는 시리얼 passthrough일 뿐 퓨전 로직 없음, 퓨전은 EKF 노드에 있음 |
| GPS 재밍 터널 구간 | 위 `1024_EBIMU_EKF.py`의 `cb_gps`가 GPS `position_covariance`를 그대로 칼만 필터 `R`(측정 노이즈)에 반영 → GPS 신뢰도가 낮을수록(공분산↑) 게인 `K`가 자동으로 작아져 IMU+휠 예측(dead-reckoning) 비중이 커지는 **표준 EKF의 공분산 가중 보정** 메커니즘 | 확인됨 — 별도의 "터널 모드 전환" 로직은 없고, EKF 자체의 수학적 성질이 대응 메커니즘임을 코드로 확인 |
| RTK 보정(정밀 GPS) | `ntrip_client`, `ublox_gps` | 확인됨 |
| 장애물 회피 | `src/erp_driver/scripts/0724_ObstacleAvoidance.py`, `0730_LiDAR_ObjectDetect.py`, `0804_LiDAR_Avoidance.py`, `0822_*` 계열, `pcl_clustering_py`, `cluster_bev`, `velodyne` | 확인됨(개인 ws에만 존재) — 대부분 erp42_main엔 미채택 상태로 남아 실제 "최종 채택 알고리즘"이 무엇인지는 추가 확인 필요 |
| 정밀 localization (터널 등 GPS 취약 구간 보조) | `hdl_localization`, `hdl_global_localization`, `ndt_omp`, `fast_gicp` (개인 ws) | 확인됨(개인 ws에만 존재) — erp42_main 미포함이라 실제 대회에 쓰였는지 불확실 |
| 표지판 인식 | (미발견) `yolo_ros`는 범용 YOLO wrapper — 표지판 전용 가중치/노드가 코드베이스에서 식별 안 됨 | ❌ 미확인 — grep으로 sign/traffic 키워드 매칭 없음 |
| 신호(신호등) 인식 | (미발견) 위와 동일 | ❌ 미확인 |
| 차선 인식(YOLO 모델) | `best_lane_yolov11.pt` — 이 파일을 **로드하는 코드가 ws 어디에도 없음**(grep 결과 참조는 이 계획 문서 상위 `README.md`뿐). `yolo_bringup` launch의 `model` 파라미터 기본값도 `yolo11m.pt`(공식 pretrained)라 이 커스텀 가중치와 무관. | ⚠️ CLI 배선으로 판단 — `ros2 launch yolo_bringup yolov11.launch.py model:=.../best_lane_yolov11.pt input_image_topic:=/camera{N}/image_raw` 형태로 실행 시점에 수동 지정했을 가능성이 높음. 커밋된 소스만으로는 재구성 불가. |
| 다중 카메라 → YOLO 배선 | `usb_cam`은 `camera1`(`/dev/video2`)~`camera4`(`/dev/video8`) 4대로 분리 설정됨(카메라별 용도 분리 흔적 확인됨). 그러나 `yolo_bringup`을 이 카메라들과 연결하는 launch/스크립트가 ws에 없음(`input_image_topic`을 참조하는 파일이 `yolo_ros` 패키지 자체 외엔 0건). | ⚠️ CLI 배선으로 판단 — 위와 동일한 사유로 어느 카메라가 표지판/신호/차선 중 무엇에 연결됐는지 소스만으로 복원 불가. 부가로 `erp42_lanedetect.py`(HSV 방식)는 `/usb_cam_0/image_raw`를 구독하는데 이는 현재 launch의 리매핑 결과(`camera1~4`)와 안 맞는 레거시 참조. `params_3.yaml`/`params_4.yaml`은 설정만 있고 이를 띄우는 launch 파일 자체가 없어 배선 미완성 상태로 남음. |
| IMU/INS | `vectornav`(초기), `ebimu_pkg`(후기) | 확인됨 — 이중화가 아니라 **세대교체**: vectornav의 drift·센서퓨전 난이도 문제로 EBIMU로 대체(사용자 확인). `vectornav`는 개인 ws에만 남아있고 erp42_main엔 EBIMU 관련 코드가 아예 없어, 이 교체는 팀 repo(2025-08-08 마지막 커밋) 이후 개인 ws에서 진행된 것으로 보임(1024_* 날짜 prefix와 시점상 부합) |

## 2. 실행 순서

1. **미확인 항목부터 코드 레벨로 좁힌다** (표지판/신호/GPS재밍 3건):
   - ~~`yolo_ros/yolo_bringup` 설정에서 로드하는 가중치 파일 확인~~ → 완료: `best_lane_yolov11.pt`(차선용)를 로드하는 코드가 없음, 카메라-YOLO 연결도 소스에 없음 — 둘 다 **실행 시점 CLI 인자 배선**으로 판단됨(위 표 참고). 표지판/신호 전용 모델·클래스는 여전히 미발견.
   - `erp42_imu-gps-wheel-ekf_globalposition.py` 전체를 읽고 GPS fix status(`NavSatFix.status`) 참조 여부, dead-reckoning 전환 조건 확인.
   - `0724_*`~`0822_*` 장애물회피 스크립트 중 최신/완성도 높은 것을 식별(파일 크기·구현 완성도로 1차 스크리닝 후 코드 리뷰).
2. **미션별 "문제-해결" 카드 작성** (표 1행 = 카드 1장): 문제 상황 → 사용 센서/알고리즘 → 핵심 코드(파일:라인) → 결과(검증된 것만, 없으면 "구현 완료·실차 검증 여부 미기재").
3. **아키텍처 다이어그램 초안**: 센서(카메라4·라이다·GPS·RTK·IMU·휠엔코더) → 인식/localization → 판단(경로계획/장애물회피) → 제어(erp42_serial) 흐름을 노드/토픽 그래프로 정리. `rqt_graph` 캡처가 있으면 활용, 없으면 코드 기반으로 재구성(토픽명 기준).
4. **기술 스택 표** 작성: ROS2, YOLOv8/v11, EKF, NDT/GICP(fast_gicp, ndt_omp), pymap3d, RTK-NTRIP 등 — 실제 코드에서 import/의존성으로 확인된 것만.
5. ✅ **정량 지표 수집 시도 — 완료(2026-08-20)**: `find`로 두 ws 전체를 탐색한 결과 실제 대회/HILS rosbag 파일은 없음(`robot_localization`의 vendor 테스트용 `test1~3.bag` 3개만 존재, 대회 기록 아님) — 사용자 본인 탐색도 실패, 수치 없음으로 확정. 다만 `rosbag_convert_clean_py`의 세 스크립트(`gps_convert.py`, `odom_convert`, `plot_waypoint_csv`)에 하드코딩된 팀원 PC 경로(`/home/mrlam/colcon_ws/bagfiles_ros2/...`)가 워크플로우를 그대로 증언함: NTRIP/RTK 보정된 `/ublox_gps_node/fix`를 rosbag로 기록(`hils/June_07/e4tow2/` — waypoint 파일명 `e4tow2`와 정확히 일치) → `rosbag2csv.py`로 CSV 변환 → `gps_convert.py`로 정제 → waypoint `.xls` 생성, 이라는 **waypoint 생성 파이프라인이었음을 확인**(사용자 기억과 일치, 2026-08-20). 추가로 `odom_ukf_ctrl11.csv`/`UKF_Pathtrack` 디렉토리명이 UKF 실험 흔적으로 확인돼 mission-cards.md의 UKF/EKF 판정에 반영함.
6. **사용자 확인이 필요한 비-코드 정보 취합** (대회 결과, 팀 내 본인 담당 범위, 실차 영상/사진 유무)해서 포트폴리오 초안에 반영.
7. ⚠️ **실 주행 rosbag 확보 및 EKF replay 시도 — 부분 완료(2026-08-21)**: 사용자가 실 주행 rosbag(`rosbag2_2025_10_25-16_34_43`, 용량 제약으로 일부 센서만 녹화)을 제공. 토픽 확인은 완료(`/vectornav/imu`,`/ublox_gps_node/fix`,`/ebimu_data` 有, `/erp42_status`,`/erp42_ctrl_cmd`,`/scan`,`/waypoints_path` 無 — 상세는 `mission-cards.md` "실 주행 rosbag EKF replay 검증" 행 참고). **실제 replay는 이 환경에 `ros2 bag`/`rosbag2_py` 재생 도구가 없어 미실행** — `sudo apt install ros-humble-ros2bag` 설치 여부를 사용자에게 확인 후 재시도 필요. 같은 날 Google Drive에서 발견한 `trafficlight_stop_ros2.py`(실제 내용: LiDAR 회피+pathtracking 통합 노드, 신호등 로직 없음)를 `src/erp_driver/scripts/erp42_pathtracking_lidar_integrated.py`로 정직한 이름으로 배치함 — 다만 launch 배선 근거 없어 최종 채택 여부는 미확인.

## 3. 산출물 (다음 단계에서 생성 예정, 이번엔 계획만)
- `docs/portfolio/mission-cards.md` — 미션별 문제-해결 카드
- `docs/portfolio/architecture.md` (+ 다이어그램) — 센서~제어 파이프라인
- `docs/portfolio/tech-stack.md` — 검증된 기술 스택 표
- 필요 시 Artifact로 시각화(다이어그램/카드형 요약) — 이 경우 private repo 내용 노출 범위를 먼저 확인

## 확신도
확신도: 검증됨 (grep/파일구조로 확인) — 표지판·신호·GPS재밍 대응 로직의 실재 여부는 미확인(2단계 조사 대상).
내가 채워넣은 가정:
1. 두 ws(src+erp42_main)를 합쳐야 5개 미션을 다 커버한다고 전제함
2. `process_noise_xy_min/max` 적응형 EKF를 "GPS 재밍 대응"으로 추정함 (코드 내부 로직 미확인)
확인 요청: 표지판/신호 인식 코드가 이 두 ws에 없다면 (a) 다른 저장소에 있는지, (b) 실제로 해당 미션을 다른 방식(예: 규칙 기반 정지선 인식)으로 처리했는지 알려주실 수 있나요? 이게 확인돼야 2단계 조사 범위를 정할 수 있습니다.
