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
| `ros_dialogue/dialogue_node.py` | 수정 | 도구 호출을 가로채 `/lekiwi_command` 에 `pick_<color>` 발행 |
| `ros_dialogue/lekiwi_control.py` | 수정 | `SKILL_MAP` 에 `pick_red/blue/green` (SSH 로 `~/pick_trigger.sh <color>`) |
| `ros_dialogue/lekiwi_command_node.py` | 사용(변경 없음) | `/lekiwi_command` 구독 → SSH 실행 |
| `test/test_lekiwi_control.py` | 수정 | 새 스킬 테스트 |

흐름: 발화 → `dialogue_node` → `/lekiwi_command "pick_red"` (도메인 77) →
`lekiwi_command_node` SSH → lekiwi01 `~/pick_trigger.sh red` (`nav/shell/pick_trigger.sh`)
→ `/abo/command "fetch center color:red"` (도메인 42) → `nav/mission/abo_nav_bridge.py`.

원본을 고치면 이 사본도 같이 갱신할 것.
