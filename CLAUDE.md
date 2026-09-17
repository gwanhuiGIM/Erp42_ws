# ERP42 자율주행 (2025 대학생 창작자동차 경진대회, 팀 MTP·충남대)

- **한 줄 목적**: ERP42(Wego Robotics 4륜 전기차) 기반으로 예선(Track Drive+Highway, GPS 음영구간 통과)·본선(배달, 교차로, 정적장애물, 주차) 미션을 단일 자율주행 run으로 통과하는 무인 모빌리티.
- **담당 역할 / 팀 규모**: 2~3인(팀 MTP). 본인 주 담당 — LiDAR 장애물인식/클러스터링(`pcl_clustering_py`, `cluster_bev`, `erp_driver/scripts/0724_*`~`0822_*`), Pathtracking/제어/Controller Node(`erp42_pathtracking.py`, `0702_erp42_controller.py`, `erp42_serial.py`). EKF 센서퓨전과 비전(차선/YOLO)은 팀 동료 담당 영역.
- **스택(빠짐없이 넓게)**: ROS2 Humble, Python(rclpy) + C++(rclcpp), NumPy(custom 선형 EKF, sigma-point 없음 — UKF 아님), pymap3d(geodetic→ENU), OpenCV/cv_bridge(HSV lane), Ultralytics/PyTorch(erp42_main만, YOLO), Python-PCL/PCL(clustering), pyserial(ERP42 40Hz packet, EBIMU), u-blox+NTRIP(RTK), VectorNav/EBIMU(IMU 세대교체), Velodyne VLP16(src만), pandas(waypoint Excel).
- **워크스페이스 구조**: `src/`(개인 작업본, LiDAR/localization 실험 포함) + `erp42_main/`(팀 정리본, GitHub private `Mr-HuynhLam/erp42_chungnam` 관리용 백업 — 실기 배포 스냅샷 아님). 둘 다 git repo 아님. 비교는 `README.md` 참고.
- **지시적 파일**:
  - `docs/portfolio/mission-cards.md` — 미션공지 vs 기술보고서 vs 실제 코드 대조(코드에 없는 걸 있다고 쓰지 않기 위한 단일 출처)
  - `docs/portfolio/PORTFOLIO_MAP.md` — 소스 전수 스캔(LOC, 11축 매트릭스, 파일:라인 앵커)
  - `docs/portfolio/PORTFOLIO_SOURCE.md` — 포트폴리오 원천 문서(STAR, 기술의사결정, 전공매핑)
  - `docs/portfolio/tech-stack.md`, `docs/portfolio/architecture.md` — 기술스택/토픽그래프 코드 근거
  - `docs/plans/2026-08-20-portfolio-extraction.md` — 포트폴리오 추출 계획
- **이 ws에서 강조할 만한 기술 축**: [통신] [노드설계] [모션플래닝] [상태관리] [안전] [데이터] (본인 담당 구간). 팀 코드베이스 전체로는 [좌표계] [비전] [인프라]도 존재하나 본인 주 담당 아님.
- **정직성 원칙(이 ws 한정)**: 보고서(기술보고서 PDF)와 실제 코드가 다르면 **코드가 이긴다** — 특히 YOLOv8l(보고서)↔YOLOv11(코드), UKF(보고서)↔선형 EKF(코드), LiDAR `brake==2` 조건은 발행자가 없는 dead branch. 포트폴리오 작성 시 `mission-cards.md`의 범례(✅/⚠️/❌/🗑️/📄)를 그대로 따를 것.
