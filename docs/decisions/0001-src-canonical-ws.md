# ADR 0001: `src/`를 단일 캐노니컬 워크스페이스로 확정

- 날짜: 2026-09-18
- 상태: 확정
- 목적: 실기 재배포가 아니라 **포트폴리오 공개·문서화용 단일 ws 상태**를 만들기 위함(사용자 결정)

## 배경 — 왜 두 ws가 있었는가

- **`src/`(개인 작업 ws)**: ERP42 드라이버 스택 + LiDAR/localization 실험 패키지(hdl_localization, ndt_omp, fast_gicp, velodyne, pcl_clustering_py 등)와 날짜별 실험 스크립트(`0610_...`~`0822_...`)가 정리 안 된 채 쌓여있던 개인 작업 이력.
- **`erp42_main/`**: 팀원(`Mr-HuynhLam`)이 실차 튜닝값을 반영해 정리한 뒤 GitHub private repo `Mr-HuynhLam/erp42_chungnam`(팀 작업 관리용 백업, 실기 전체 ws 스냅샷 아님)에 올린 걸 로컬로 가져온 사본. LiDAR/localization 스택은 없고 대신 `waypoint`, `yolo_ros`, `rosbag_convert_clean_py` 같은 도구가 추가돼 있었음.
- 병렬 관리는 의도적 구조가 아니라 "개인 실험 이력 vs 팀이 합의해서 공유한 부분집합"이라는 협업 과정에서 생긴 격차였음.

## 결정 사항

1. **아키텍처**: Controller 중재 노드(`erp42_controller.py`, lane 우선 → path → timeout 시 강제정지) 포함 구조를 채택. main의 "pathtracking이 serial에 직접발행" 구조는 채택하지 않음.
2. **EKF**: `erp42_ebimu_ekf_globalposition.py`(구 `1024_EBIMU_EKF.py`) 하나로 통일. vectornav 기반 EKF는 IMU drift 문제로 이미 폐기된 이전 세대로 간주해 archive 처리.
3. **main 전용 패키지**(`waypoint`, `yolo_ros`, `rosbag_convert_clean_py`, `rosbag2csv.py`, lane detect 노드 `erp42_lanedetect.py`/`erp42_lanedetect_yolo.py`)를 `src/`로 편입.
4. **실차 튜닝값**은 main 기준으로 `src/`에 반영: `ntrip_client` mountpoint, `ublox_gps` config/launch, `vectornav` port, `usb_cam` params_1~4.yaml.
5. 날짜 prefix 실험 스크립트(총 15개)는 삭제하지 않고 `erp_driver/scripts/archive/`로 이동 — 시행착오 이력 보존.
6. **`erp42_main/`은 삭제하지 않고 협업 이력 참고 자료로 격하** — 포트폴리오의 단일 기준은 `src/`.

## 검증 (Codex 교차검증, 2026-09-18)

- 전체 파일 비교 386개 중 384개 동일, 나머지 2개(vectornav EKF, pathtracking 브레이크값)는 위 결정에 따른 의도적 차이 — **실질적 콘텐츠 손실 없음**.
- `src/ublox/ublox_gps` ↔ `erp42_main/src/ublox_gps`, `src/vectornav/vectornav` ↔ `erp42_main/src/vectornav` 등 배치만 다른 패키지는 내용 동일 확인(`diff -rq` 결과 없음).
- lane 관련 파일 3종(`lanedetect.py`, `claude_lanedetect.py`, `erp42_lanedetect.py`)은 진짜 중복이 아니라 서로 다른 실험 variant — `erp42_lanedetect.py`(ROS 노드)가 canonical, 나머지 둘은 로컬 mp4 실험용 독립 스크립트.

## 병합 후 새로 드러난 이슈

- ✅ **수정 완료 (2026-09-18)**: `erp42_lanedetect.py`가 lane 발행 시 `brake`를 설정하지 않아 Controller의 `brake==3` 채택 조건을 못 만족하던 문제(main엔 Controller가 없어 드러난 적 없던 문제) — `publish_steering()`에 `e_stop=False`/`gear=0`/`speed=15`/`brake=3` 추가. `colcon build`(erp_driver)·`ast.parse` 통과, ROS 런타임/실차 미검증.
- ✅ **수정 완료 (2026-09-18)**: `erp42_lanedetect_yolo.py`의 `Detection2DArray` 미정의로 `NameError` 발생(main 시절부터 있던 버그) — import된 `DetectionArray`로 교체, `brake=1`도 Controller 조건에 맞게 `brake=3`으로 함께 수정. `colcon build`·`ast.parse` 통과, ROS 런타임/실차 미검증.
- pathtracking 속도값: main은 `self.max_linear_speed`로 파라미터화(값 50)했지만 이 ws는 하드코딩 `15` 유지 — 토픽/brake는 Controller 연동 때문에 유지해야 하지만 속도 파라미터화는 포팅 가치 있는 개선점, 미반영.

## 빌드 검증 (`colcon build --base-paths src`, 2026-09-18)

전체 27개 패키지 기준: **15 PASS / 7 FAIL / 5 미처리(FAIL 의존성 연쇄)**.

FAIL 7개 중 6개는 시스템에 `ros-humble-ament-lint-auto`, `ros-humble-ament-cmake-cppcheck` apt 패키지가 없어서(병합과 무관한 환경 문제), 1개(`ndt_omp`)는 `CMakeLists.txt:23` 자체의 문법 오류(병합 이전부터 있던 코드 버그, 미수정). 상세는 각 패키지 `log/latest_build/<pkg>/stderr.log` 참고.

## 공통 패키지별 상세 diff (참고용, 병합 전 스냅샷 기준)

<details>
<summary>erp_driver / erp_interfaces / ntrip_client / usb_cam / vectornav 세부 차이</summary>

### erp_driver
- **scripts 정리**: `src/`에는 날짜 prefix 붙은 반복 실험본 17개가 있었음. `erp42_main`은 이 중 최종 채택분만 날짜 prefix를 뗀 채 유지, 나머지는 삭제돼 있었음.
- `erp42_pathtracking.py` 튜닝값: publish 토픽(`/erp42_ctrl_cmd/path` → `/erp42_ctrl_cmd`), 속도(하드코딩 `15` → `self.max_linear_speed` 파라미터화), `brake`(`2` → `1`).
- `waypoints_real_*.xls`는 erp42_main에서 별도 `waypoint/` 패키지로 분리돼 있었음.

### erp_interfaces
완전 동일(diff 없음).

### ntrip_client
`mountpoint` 기본값만 변경: `RTK-RTCM32` → `RTK-RTCM31`.

### usb_cam
카메라 device/해상도/포맷 설정이 실차 카메라에 맞게 변경, main에 `params_3/4.yaml` 추가(카메라 2대→4대), launch 파일명 `2camera.launch.py`→`camera2.launch.py`.

### vectornav
패키지 구조 자체가 다름 — `src/`는 upstream `dawonn/vectornav` repo 그대로(서브패키지 2개), `erp42_main`은 서브패키지를 최상위로 끌어올린 배치. 내용은 `port` 값(`/dev/ttyUSB2`→`/dev/ttyUSB0`) 1줄 외 동일.
- ✅ 조치 완료(2026-08-20): `vectornav_msgs` 누락, `src/` 래퍼 부재 문제 모두 해소됨.

</details>

## GitHub 원본 repo 검증 이력 (완료, clone 삭제됨)

`erp42_main/`의 출처 확인을 위해 원본 repo(`https://github.com/Mr-HuynhLam/erp42_chungnam`, private)를 임시로 로컬 clone해서 대조한 뒤 삭제함(다른 사람 소유 private repo, 확인 목적으로만 사용, 외부 공유 없음).

- 커밋 15개, 전부 GitHub 웹 업로드(`Add files via upload`). 기간: 2025-06-02 ~ 2025-08-08.
- pristine clone 구조가 수정 전 `erp42_main/`과 100% 동일(패키지가 루트에 바로 있고 `src/` 래퍼 없음, `vectornav_msgs` 없음) — `vectornav_msgs` 누락과 `src/` 래퍼 부재는 로컬 추출 실수가 아니라 원본 repo 자체의 상태였음이 커밋 이력으로 확인됨.
- **팀 작업 관리용 repo**(사용자 확인): 실기 배포 스냅샷이 아니라 파일 백업/공유 목적 업로드. `erp42_main`을 실기의 authoritative 상태로 취급하지 말 것.
