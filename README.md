# LeKiwi 약통 픽업 데모

공학경진대회 제출용 데모 — 음성 명령으로 LeKiwi가 책상 위 약통을 찾아 집는 파이프라인.

에이보(별도 저장소 `robot_ws`)와 SSH로 연동된다. 설계 문서:
`docs/superpowers/specs/2026-08-12-lekiwi-pill-pickup-design.md`

## 자율주행 (이동 부분)

집기 전후의 **이동** — 지도 만들기(SLAM), 위치추정(AMCL), 목표까지 자율주행(Nav2),
왕복 미션 — 은 [`nav/`](nav/README.md) 에 있다. 집기와는 ROS2 토픽 두 개로만
붙는다: 도착하면 `/abo/pick_request` 를 발행하고, `/abo/pick_done` 을 받아야
복귀를 시작한다.
