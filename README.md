# LeKiwi 약통 픽업 데모

공학경진대회 제출용 데모 — 음성 명령으로 LeKiwi가 책상 위 약통을 찾아 집는 파이프라인.

에이보(별도 저장소 `robot_ws`)와 SSH로 연동된다. 설계 문서:
`docs/superpowers/specs/2026-08-12-lekiwi-pill-pickup-design.md`

## 자율주행 (이동 부분)

집기 전후의 **이동** — 지도 만들기(SLAM), 위치추정(AMCL), 목표까지 자율주행(Nav2),
왕복 미션 — 은 [`nav/`](nav/README.md) 에 있다. 집기와는 ROS2 토픽 두 개로만
붙는다: 도착하면 `/abo/pick_request` 를 발행하고, `/abo/pick_done` 을 받아야
복귀를 시작한다.

## 외부 저장소에서 가져온 코드

색상 지정 픽업에 쓰는 다른 저장소 코드를 한곳에 모았다.

- [`yolo_and_pick/`](yolo_and_pick/README.md) — `roboseasy/lekiwi` 의 YOLO 검출 + 집기 CLI
  (`feature/target-color-pickup`, 커밋 `f69c4da`). `--target_color`/`--result_file` 옵션 추가본.
- [`abo/`](abo/README.md) — `roboseasy-members/A-Bo_project` 의 발화 → `pick_<color>` 연동 파일 사본
  (PR #30, 커밋 `4bb2b84`).
