# PORTFOLIO_SOURCE.md §4·§5 독립 검토

- 검토일: 2026-08-29
- 대상: `docs/portfolio/PORTFOLIO_SOURCE.md`의 `## 4. 핵심 기술 의사결정`, `## 5. 문제 해결 사례 (STAR)`
- 검토 범위: `src/`, `erp42_main/src/`의 실제 소스와 package/launch wiring
- 판정 기준
  - **근거 탄탄**: 해당 문장을 직접 지지하는 코드와 정적 wiring이 확인됨
  - **근거 약함**: 소스 artifact는 있으나 실행·통합·대회 적용까지 입증하지 못함
  - **인용 오류**: 인용된 코드의 실제 의미가 문장과 다르거나 반대 근거가 있음

## 총평

**현재 상태 그대로 채용용 STAR로 사용하는 것은 수정 권고**다. 코드에 남은 사실과 최종 통합 여부를 구분하려는 태도는 좋지만, STAR 1·2는 대회 당시의 문제 해결보다 2026년 포트폴리오 검증 과정의 “정직성 확인”을 성과처럼 서술한다. 채용담당자에게는 기술적 성찰보다 “미통합·탈락을 길게 해명하는 문장”으로 먼저 읽힐 가능성이 높다.

또한 다음 세 가지는 반드시 정정해야 한다.

1. `src/`의 Controller/pathtracking은 topic 이름이 맞물리지만, 두 스크립트는 `src/erp_driver/CMakeLists.txt:14-20`의 설치 대상도 `src/erp_driver/launch/erp42_base.launch.py:5-15`의 launch 대상도 아니다. 따라서 “실제로 살아있는 경로”가 아니라 **source-level wiring이 일치하는 prototype**으로 표현해야 한다.
2. 직접제어형 LiDAR 노드는 `/erp42_ctrl_cmd`를 발행하므로 Controller는 우회하지만, EKF도 같은 `/erp42_ctrl_cmd`를 구독한다(`erp42_main/src/erp_driver/scripts/erp42_imu-gps-wheel-ekf_globalposition.py:67`). “Controller/EKF 피드백 경로를 우회”한다는 문장은 코드와 맞지 않는다.
3. “인지 실패에도 시스템이 무너지지 않은 구조”를 `graceful degradation`으로 부르려면 fault detection, 전환 조건, degraded state가 있어야 한다. 현재 코드에서 확인되는 것은 perception event와 독립적으로 waypoint pathtracking이 계속 발행되는 **baseline 경로의 독립성**이지, 인지 실패를 감지해 안전하게 모드를 전환하는 recovery mechanism은 아니다.

## §4. 핵심 기술 의사결정 근거 점검

### 4-1. Waypoint pathtracking

- **판정: 근거 탄탄**
- nearest 후보와 5-waypoint window, heading gate, 거리·heading cost는 `src/erp_driver/scripts/erp42_pathtracking.py:84-120`에 있다.
- lookahead waypoint 선택은 `:128-136`, goal index 비감소 제약은 `:118-120,161-164`에 있다.
- heading error의 비례 mapping과 `alpha=0.65` blending은 `:175-187`에 있다. 두 pathtracking 파일에서 `Kp`, `Ki`, `Kd`, `PID` token은 검색되지 않았다.
- 다만 “pure-pursuit이나 PID 대신 채택”은 코드가 보여 주는 **구현 형태**이지 당시 대안을 비교해 결정했다는 기록은 아니다. 면접 문서에서는 “채택”보다 “구현”이 안전하다.
- `erp42_pathtracking.py:84,128,161`처럼 파일명이 한 번만 나온 축약 인용보다 전체 상대경로와 범위를 쓰는 편이 재검증하기 쉽다.

### 4-2. Controller Node 우선순위 selector

- **판정: source logic은 근거 탄탄, 실제 통합은 근거 약함**
- lane 우선 `if/elif`, 0.2초 freshness, `brake==3/2` sentinel, full-brake fallback은 `src/erp_driver/scripts/0702_erp42_controller.py:16-58`에 정확히 존재한다.
- `src/` pathtracking의 `/erp42_ctrl_cmd/path` 및 `brake=2`는 `src/erp_driver/scripts/erp42_pathtracking.py:53-57,179-190`에 있어 Controller의 `path_valid` 조건과 source-level contract가 맞는다.
- 그러나 Controller와 pathtracking은 `src/erp_driver/CMakeLists.txt:14-20`에 설치되지 않고, 유일한 launch 파일에도 포함되지 않는다. 실행 로그도 이 검토 범위에 없다. 따라서 §4의 “실제로 pathtracking과 배선돼 있다”는 **정적 topic contract가 연결돼 있다**로 낮춰야 한다.
- `erp42_main/src/erp_driver/scripts/erp42_pathtracking.py:53-56,182-190`은 최종 topic에 직접 발행하고 `brake=1`을 쓰며, `erp42_main/src/`에는 Controller 파일·class·node name이 없다. 팀 정리본에서 Controller를 우회한다는 판정은 탄탄하다. 단, workspace 지침상 `erp42_main/`은 “팀 정리본”이지 실기 배포 snapshot이 아니므로 대회 run 미채택은 계속 미확인으로 남겨야 한다.

### 4-3. LiDAR clustering 및 날짜별 회피 variant

- **판정: 인용 오류 포함, 구현 성숙도 근거 약함**
- `cluster_bev`는 voxel downsampling, RANSAC ground/wall removal, Euclidean clustering, BEV publish를 실제로 구현한다(`src/cluster_bev/src/cluster_bev_node.cpp:25-52,68-157,160-199`). 이 부분은 탄탄하다.
- `pcl_clustering_py`에도 voxel/RANSAC/Euclidean clustering source는 있다(`src/pcl_clustering_py/pcl_clustering_py/euclidean_cluster_node.py:15-89`). 그러나 `points_list = ''` 뒤 `append()`를 호출하고(`:95-101`), `colored_points = ''` 뒤에도 `append()`를 호출한다(`:108-118`). callback이 정상 입력을 처리하면 runtime error가 나는 상태이므로 “병행 구현”은 **실험 source를 작성함** 정도가 정확하다. `py_compile` 통과는 이 runtime type error를 검증하지 않는다.
- 날짜 prefix LiDAR 관련 파일은 7개지만 `/erp42_ctrl_cmd`에 직접 발행하는 것은 확인된 범위에서 4개다: `0724_erp42_ObstacleAvoidance.py:11`, `0822_Lam_ObtAvo.py:87-89`, `0822_Ob_bangbang3d.py:14`, `0822_ObstackeaAv_2d.py:13`. 따라서 §4의 “최소 5개의 직접제어형”은 숫자 인용 오류다.
- channel 방식도 완성된 단일 경로가 아니다. `0804_LiDAR_Avoidance.py:12`는 입력 topic이 `'/velo dyne_points'`로 잘못 쓰였고, `0822_Obstacle_3d.py:93`은 list에 정수를 더해 runtime `TypeError`가 난다. 여러 파일의 존재는 반복 실험의 근거이지 작동 검증의 근거가 아니다.
- `/clustered_points`, `/clusters_bev`의 repository 내 참조는 두 publisher 정의뿐이므로 판단 노드와 연결되지 않았다는 한계 표시는 정확하다.

### 4-4. ERP42 command 다중 채널

- **판정: 근거 약함**
- `/path`, `/lane`, `/lidar`는 실제 source에 있으나 `/camera`는 `src/`, `erp42_main/src/`에서 검색 0건이다.
- 네 채널을 하나의 구현에서 선언하거나 중재하는 코드는 없고, Controller prototype도 lane/path 두 채널만 처리한다(`src/erp_driver/scripts/0702_erp42_controller.py:16-21`). 따라서 “채널을 4개로 분리 설계”는 코드 구현 사실이 아니라 보고서/의도 수준으로 분리해야 한다.
- `erp42_main/`에서 lane publisher는 남아 있지만 subscriber가 없고, pathtracking은 최종 `/erp42_ctrl_cmd`로 직접 발행한다. dead topic이라는 한계 표시는 정확하다.

### 4-5. Node 역할 분리와 timer 기반 I/O

- **판정: source architecture는 근거 탄탄, deployable integration은 근거 약함**
- pathtracking publisher, Controller의 20 Hz selector, serial의 40 Hz I/O라는 역할 분리는 인용 범위와 일치한다(`src/erp_driver/scripts/erp42_pathtracking.py:53-65`; `0702_erp42_controller.py:16-58`; `erp42_serial.py:24-46`).
- pathtracking 제어가 별도 timer가 아니라 odometry callback에서 실행되고 세 파일이 기본 `rclpy.spin()`을 쓴다는 한계도 정확하다.
- 다만 CMake는 `erp42_serial.py`만 설치하고 Controller/pathtracking은 설치하지 않는다. “노드 분리 architecture를 source로 설계했다”와 “launch 가능한 통합 시스템을 구성했다”를 구분해야 한다.

## STAR 1 — LiDAR 회피 다중 접근

### (a) 근거 검증 결과

**종합 판정: 인용 오류. 단, `brake==2` dead trigger 발견 자체는 근거 탄탄.**

- pathtracking은 `/erp42_ctrl_cmd/lidar`를 구독하고 `brake==2`일 때 waypoint index를 갱신한다(`src/erp_driver/scripts/erp42_pathtracking.py:47-51,138-146`; `erp42_main/.../erp42_pathtracking.py`도 동일).
- 해당 channel publisher 세 개의 brake 값은 155/0(`0724_erp42_3DObstacle_Lam.py:30-41`), 항상 0(`0804_LiDAR_Avoidance.py:57-61`), 200/0(`0822_Obstacle_3d.py:64-70,113-124`)이다. `/lidar` channel에서 `brake=2`를 발행하는 source는 없다. 따라서 이 분기가 현재 저장소 조합으로 트리거되지 않는다는 결론은 탄탄하다.
- 직접제어형 variant가 `/erp42_ctrl_cmd`를 발행한다는 사실도 확인된다. 하지만 직접제어 variant는 최소 5개가 아니라 4개이며, 일부는 brake를 명시하지 않거나 runtime error가 있다.
- “Controller/EKF 피드백 경로를 우회”는 인용 오류다. 직접 명령은 Controller는 우회하지만 EKF의 `/erp42_ctrl_cmd` subscriber에는 전달된다. 더 정확한 위험은 **pathtracking 등 다른 최종 command publisher와 ownership/arbitration 없이 경쟁할 수 있다**는 점이다.
- source와 topic 계약만 확인됐으며 실제 실행, rosbag, 차량 반응은 확인되지 않았다. “각각 다른 brake 값으로 실험”의 실차 실험 여부는 코드만으로 입증할 수 없다.

### (b) 서사 평가

**어색함.** 제목의 “팀 통합에는 못 미친 것을 정직하게 남긴 사례”와 Result의 “이번 cross-review로 확인됨”, “정직하게 남긴다”는 채용 성과가 아니라 포트폴리오 감사 과정이다. Task도 “최종 채택하거나 연동시켜야 함”인데 Result는 그것을 달성하지 못했다. STAR의 Result가 문제 해결이 아니라 미완료 확인으로 끝나 변명처럼 읽힐 수 있다.

특히 “⚠️ 미검증이었던 것을 검증됨으로 승격”은 내부 문서 관리 용어라 외부 독자에게 의미가 없다. 기술적으로는 trigger mismatch, command ownership 부재, 여러 prototype의 runtime 결함을 발견한 회고 사례로 분리하는 편이 자연스럽다.

### (c) 개선 제안

대회 문제 해결 STAR로 유지하지 말고 **“통합 회고 및 interface audit”**로 이동하는 것을 권한다. STAR 형식을 꼭 유지한다면 이번 검증 행위를 명확히 별도 시점의 audit으로 써야 한다.

> **Situation**: LiDAR 회피 code가 `/erp42_ctrl_cmd/lidar` channel 방식과 최종 command 직접 발행 방식으로 나뉘어 남아 있어, 실제 integration boundary를 구분하기 어려웠다.  
> **Task**: portfolio에 반영하기 전에 각 variant의 topic, brake sentinel, command ownership을 source 기준으로 재검증했다.  
> **Action**: `/lidar` publisher 세 개와 pathtracking subscriber를 대조해 `brake==2` contract 불일치를 찾고, 직접 발행 variant가 Controller를 우회해 다른 command publisher와 경쟁할 수 있음을 분리해 기록했다.  
> **Result**: LiDAR 회피를 “최종 통합 기능”이 아니라 “복수 prototype과 미해결 integration debt”로 하향 분류했다. 실차 회피 성능은 로그가 없어 주장하지 않는다.

이 재작성은 정직하지만, 채용용 대표 STAR로는 약하다. 실제 대회 당시의 tuning 전후 수치, 재현 log, 채택 결과가 없다면 보조 회고로 두는 편이 낫다.

## STAR 2 — Controller Node prototype

### (a) 근거 검증 결과

**종합 판정: 근거 약함. selector 구현과 source-level wiring은 탄탄하지만 실제 실행·통합 근거가 없다.**

- 20 Hz timer, lane/path 우선순위, 0.2초 freshness, brake sentinel, full-brake fallback은 코드에 정확히 존재한다(`src/erp_driver/scripts/0702_erp42_controller.py:16-58`).
- `src/erp_driver/scripts/erp42_pathtracking.py:53-57,179-190`은 `/path`와 `brake=2`를 발행해 Controller의 path input contract와 맞는다.
- 그러나 두 파일은 CMake install/launch에 포함되지 않는다. 따라서 “실제로 살아있는 경로”보다 “정적으로 interface가 맞는 prototype”이 정확하다.
- `erp42_main` pathtracking이 `/erp42_ctrl_cmd`로 직접 발행하고(`erp42_main/src/erp_driver/scripts/erp42_pathtracking.py:53-56`), Controller가 `erp42_main/src/`에 없다는 비교는 정확하다. “최종 대회 run 사용 여부 불확실”이라는 제한도 적절하다.

### (b) 서사 평가

**어색함.** “팀 최종본에서 탈락한 것을 확인한 사례”라는 제목은 지원자의 설계 판단보다 실패 판정에 초점을 둔다. Situation이 “보고서는 주장한다”로 시작해 팀 문서의 오류를 반박하는 인상을 주고, Result도 “불확실하다”로 끝나므로 채용담당자가 묻는 “그래서 지원자가 어떤 문제를 어떻게 개선했는가”에 답하지 못한다.

다만 Action의 20 Hz arbitration, freshness, fail-safe brake는 기술 면접에서 설명할 만한 구체성이 있다. 대표 STAR가 아니라 **prototype design decision**으로 제시하면 설득력이 높아진다.

### (c) 개선 제안

> **Situation**: lane과 pathtracking node가 각자 command를 만들면서 stale command나 동시 발행을 한 곳에서 통제할 필요가 있었다.  
> **Task**: 두 input을 하나의 `/erp42_ctrl_cmd`로 중재하고, 유효한 input이 없을 때 정지 command를 내는 prototype을 설계했다.  
> **Action**: lane 우선 `if/elif`, channel별 0.2초 freshness, `brake==3/2` validity sentinel, 20 Hz arbitration, input 부재 시 `brake=155` fallback을 구현했다.  
> **Result**: `src/`에서는 pathtracking output과 Controller input의 source-level contract를 맞췄다. 다만 install/launch wiring과 실차 실행 증거는 남아 있지 않고, 팀 정리본은 pathtracking direct publish 구조를 사용하므로 대회 채택 성과로 주장하지 않는다.

제목도 “다중 command 충돌을 줄이기 위한 Controller prototype”처럼 기술 문제를 앞에 두는 것이 자연스럽다.

## STAR 3 — perception module 부재와 baseline pathtracking

### (a) 근거 검증 결과

**종합 판정: 근거 약함. source-level 독립성은 확인되지만 완주·고장대응 성과는 사용자 기억뿐이다.**

- 팀 정리본 pathtracking은 `/odom_ekf`와 waypoint path가 있으면 odometry callback마다 navigate를 실행하고 최종 `/erp42_ctrl_cmd`를 직접 발행한다(`erp42_main/src/erp_driver/scripts/erp42_pathtracking.py:34-65,138-190`). perception event가 없어도 이 baseline 경로가 실행될 수 있다는 구조는 확인된다.
- EKF가 `/odom_ekf`를 발행하는 source도 있다(`erp42_main/src/erp_driver/scripts/erp42_imu-gps-wheel-ekf_globalposition.py:67-74,190-230`). 다만 이 정적 연결만으로 localization 안정성이나 실제 완주를 입증할 수는 없다.
- 신호등 weight 유실과 실제 완주는 `docs/portfolio/mission-cards.md:60-67`에 “사용자 확인”으로만 기록돼 있다. 이 검토에서 rosbag, 대회 log, 결과표를 확인하지 못했으므로 코드 근거로 승격할 수 없다.
- 인지 failure를 감지하거나 mode를 전환하는 logic은 확인되지 않았다. 그러므로 “인지 실패에도 시스템이 무너지지 않은 구조”나 “graceful degradation”은 과장 소지가 있다. 확인된 사실은 **pathtracking baseline이 perception command channel과 느슨하게 결합돼 있었다**는 것이다.

### (b) 서사 평가

**세 항목 중 가장 자연스럽지만, 현재 제목과 Result는 과장될 수 있다.** “시스템이 무너지지 않았다”, “최소 기반이 튼튼해”는 완주 log와 failure injection 없이 robustness를 입증한 표현처럼 들린다. “미완성/미작동 모듈이 있어도”를 강조하면 지원자의 성과보다 팀의 결손을 변호하는 느낌도 강하다.

또한 Action은 “구조를 유지”했다고만 되어 있어 지원자가 무엇을 설계·구현·판단했는지가 흐리다. 본인 담당인 pathtracking의 구체적 기여(nearest/lookahead, heading P mapping, blending)를 넣고, EKF는 팀 담당임을 한 문장으로 제한하는 편이 낫다.

### (c) 개선 제안

> **Situation**: 대회 주행에서 일부 perception 기능의 보존·동작 여부가 불확실해도 기본 주행 경로는 독립적으로 유지할 필요가 있었다.  
> **Task**: localization과 waypoint가 들어오는 동안 perception event와 무관하게 기본 pathtracking command를 계속 생성하는 baseline을 유지했다.  
> **Action**: `/odom_ekf`와 waypoint path를 입력으로 nearest/lookahead target을 선택하고, heading error 비례 steering과 `alpha=0.65` blending으로 `/erp42_ctrl_cmd`를 발행했다. EKF는 팀원이 담당했다.  
> **Result**: source상 pathtracking은 perception event를 필수 입력으로 요구하지 않는다. 팀 회고로는 실제 run 완주가 확인됐지만, 현재 저장소에 주행 log가 없어 perception failure에 대한 robustness나 graceful degradation 성능으로는 주장하지 않는다.

가능하면 Result에 공식 완주 기록, rosbag 구간, 주행 영상 timecode 중 하나를 연결해야 대표 STAR로 설득력이 생긴다. 그런 근거가 없다면 “완주”는 **팀 회고 기준**으로 한정하고, 핵심 성과는 baseline decoupling과 pathtracking 구현으로 잡는 것이 안전하다.

## 최종 한줄 판정

- **STAR 1: 인용 오류** — dead trigger 발견은 정확하지만 직접제어 variant 수와 EKF 우회 설명이 틀리고, 대표 STAR보다 사후 integration audit에 가깝다.
- **STAR 2: 근거 약함** — selector와 topic contract는 명확하지만 install/launch/runtime 근거가 없어 “실제 배선”이나 대회 채택 성과로 읽히면 과장이다.
- **STAR 3: 근거 약함** — perception과 독립된 baseline pathtracking은 확인되지만 완주와 robustness는 사용자 기억뿐이며 `graceful degradation` 표현은 과하다.

## 검증 수준

- Source/read/grep: 수행
- Python syntax (`py_compile`): 관련 Python 파일 수행, PASS. 단 runtime type/topic 오류는 별도이며 PASS에 포함되지 않음
- Build: 미수행(코드 변경이 아닌 문서 검토)
- Runtime/ROS graph: 미수행
- Simulation: 미수행
- Hardware/대회 run: 미수행
