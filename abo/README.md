# 에이보(A-Bo_project) 연동 코드 사본

색상 지정 약통 픽업에서 **수정했거나 그대로 사용하는** 에이보 쪽 파일만 모아둔 사본이다.
실제 실행은 에이보 로봇의 `A-Bo_project` 워크스페이스에서 한다 — 여기 파일만으로는
빌드/실행되지 않는다(`dialogue_node.py` 는 같은 패키지의 다른 모듈에 의존).

- 출처: `roboseasy-members/A-Bo_project`, 브랜치 `feature/pickup-medicine-color`,
  커밋 `4bb2b84` (PR #30)
- 원본 경로: `src/ros_dialogue/` 아래 같은 상대 경로

| 파일 | 구분 | 내용 |
|---|---|---|
| `ros_dialogue/pickup_medicine_tool.py` | 신규 | LLM 도구, `color: Literal['red','blue','green']` |
| `ros_dialogue/dialogue_node.py` | 수정 | 도구 호출을 가로채 `pickup/medicine/<color>` 에 `data=<color>` 발행 |

흐름: 발화 → `dialogue_node` → `pickup/medicine/red` (도메인 77, 노트북) →
`nav/mission/medicine_relay.py` → `/abo/command "fetch center color:red"` (도메인 42) →
`nav/mission/abo_nav_bridge.py`. 시험 방법은 `docs/nav/abo-color-topic-test.md`.

원본을 고치면 이 사본도 같이 갱신할 것.
