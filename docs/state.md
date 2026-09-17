---
updated: 2026-08-29
---

# colcon_ws 현재 상태
> 세션 시작 시 이 파일부터 읽는다. 로그처럼 쌓지 않고 현재 상태로 덮어쓴다 (전역 CLAUDE.md §5).

## 진행 중
포트폴리오 추출 작업(`docs/plans/2026-08-20-portfolio-extraction.md`) — 산출물 8종(`docs/portfolio/*`) 작성 완료, 최종 STAR/전공매핑 문서는 `PORTFOLIO_SOURCE.md`.

- 2026-08-29: `PORTFOLIO_SOURCE.md` §4·§5를 codex exec 독립 재검증(`docs/plans/2026-08-29-star-review-codex.md`) 결과로 수정 완료 — STAR 3건 재작성(EKF 우회 서술 오류, LiDAR variant 개수 오류, graceful-degradation 과장 표현 정정), `0822_Lam_ObtAvo.py`의 bicycle 운동학 모델/Pure Pursuit 신규 반영, `mission-cards.md`의 실 주행 rosbag EKF replay 시도 이력 §6에 반영.
- 참고: 같은 날 codex에게 blind 독립 추출계획도 작성시켜 비교함(`docs/plans/2026-08-29-portfolio-extraction-codex.md`) — 큰 틀은 수렴, 검증 단계 세분화·LiDAR cone-following variant 재조명이 차이점.
- 남은 것: 실 주행 rosbag replay 자체는 `ros-humble-ros2bag` 등 재생 도구 미설치로 여전히 미실행(사용자가 `sudo apt install` 필요). 사용자 확인 대기 항목은 `PORTFOLIO_SOURCE.md` 안 각 섹션의 "확인 요청"/"사용자 확인 필요" 참고.
