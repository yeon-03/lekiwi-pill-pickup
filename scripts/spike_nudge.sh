#!/usr/bin/env bash
# ============================================================================
#  THROWAWAY 실험 하네스 (2026-08-27) — 데이터 모으면 삭제. 이 위에 짓지 말 것.
#  spike_red_nudge.sh의 색깔 일반화 버전.
#
#  목적: "가운데 병에서 먼 병(빨강/파랑)의 하강 실패 = 충돌(측면 언더슈트)"
#        가설을 확인하고, 필요한 오픈루프 넛지량 Δ의 반복성/좌우대칭성을 잰다.
#
#  흐름:  home → align <color> → [가드] → base_nudge ×N → grasp <color> → 요약
#
#  가드:  로그에 "[마커이동]"이 있으면 = 초록 정렬 + 팔 들기까지 성공 = 팔이
#         하강 직전 자세로 올라가 있음. 없으면 앞 단계 실패 → grasp 안 하고 중단.
#
#  넛지 방향: 로봇이 판을 마주보므로 "로봇 오른쪽 = 판 왼쪽"이다.
#            red  (판 왼쪽)  → 로봇 오른쪽 → y 음수 (기본 -0.03)
#            blue (판 오른쪽) → 로봇 왼쪽  → y 양수 (기본 +0.03)
#
#  사용:  ./scripts/spike_nudge.sh red         # 7펄스, y=-0.03
#         ./scripts/spike_nudge.sh blue        # 7펄스, y=+0.03
#         ./scripts/spike_nudge.sh blue 5      # 펄스 수만 변경
#         ./scripts/spike_nudge.sh blue 7 0.03 0.3   # 색 / 펄스 / y / 지속시간
# ============================================================================
set -u
cd "$(dirname "$0")/.."

COLOR="${1:-}"
case "$COLOR" in
    red)  DEF_Y="-0.03" ;;
    blue) DEF_Y="0.03" ;;
    *) echo "사용법: $0 {red|blue} [펄스수] [y속도] [지속시간]"; exit 1 ;;
esac
NUDGES="${2:-7}"
NY="${3:-$DEF_Y}"
NDUR="${4:-0.3}"
LOG="/tmp/spike_${COLOR}_$(date +%H%M%S).log"
PYENV=(env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH)

say() { printf '\n\033[1m== %s ==\033[0m\n' "$1" | tee -a "$LOG"; }
panic_stop() {
    printf '\n\033[31m!! 중단 — 베이스 정지\033[0m\n'
    ./scripts/lekiwi.sh stop >/dev/null 2>&1 || true
    exit 130
}
trap panic_stop INT TERM

echo "로그: $LOG"
echo "설정: ${COLOR} / 넛지 ${NUDGES}회 × (y=${NY}, ${NDUR}s)" | tee -a "$LOG"

say "home"
./scripts/lekiwi.sh home 2>&1 | tee -a "$LOG"

say "${COLOR} 병 확인"
echo "  ${COLOR} 병이 걸이에 제대로 있는지 지금 보세요. 아니면 Ctrl-C." | tee -a "$LOG"
echo "  5초 후 진행..." | tee -a "$LOG"
sleep 5

say "align ${COLOR}"
./scripts/lekiwi.sh align "$COLOR" 2>&1 | tee -a "$LOG"

if ! grep -q '\[마커이동\]' "$LOG"; then
    say "가드: align/초록정렬 단계에서 실패 — 팔 자세 불명"
    echo "  → grasp 안 함. 로그($LOG) 확인 후 수동 대응." | tee -a "$LOG"
    exit 1
fi

BAILX="$(grep -oE '마커x=[0-9]+' "$LOG" | tail -1 | grep -oE '[0-9]+')"
say "옆걸음 서보 포기/도달 시점 마커x = ${BAILX:-?}"

say "오픈루프 넛지 ×${NUDGES} (${COLOR} 쪽, y=${NY})"
for i in $(seq 1 "$NUDGES"); do
    printf '  [%d/%d] ' "$i" "$NUDGES" | tee -a "$LOG"
    "${PYENV[@]}" ./venv/bin/python scripts/base_nudge.py \
        --y "$NY" --duration "$NDUR" 2>&1 | tee -a "$LOG"
    sleep 0.3
done

say "grasp ${COLOR}"
./scripts/lekiwi.sh grasp "$COLOR" 2>&1 | tee -a "$LOG"

say "요약"
{
    echo "  색 / 넛지          : ${COLOR} / ${NUDGES} × (y=${NY}, ${NDUR}s)"
    echo "  서보 포기 마커x     : ${BAILX:-?}"
    grep -oE 'shoulder_(pan|lift)=[0-9.-]+\(목표[0-9.-]+[^)]*\)' "$LOG" | tail -2 | sed 's/^/  하강 도달          : /'
    grep -E '그리퍼 도달값|판정:' "$LOG" | sed 's/^ */  /'
    echo "  (자동판정 '실패'여도 걸이에서 안 빠진 것뿐일 수 있음 — 눈으로 확인)"
    echo "  로그               : $LOG"
} | tee -a "$LOG"
