#!/usr/bin/env bash
# ============================================================================
#  THROWAWAY 실험 하네스 (2026-08-27) — 이 위에 뭘 짓지 말 것. 데이터 모으면 삭제.
#
#  목적: "빨강 67도 벽 = 충돌(베이스 우측 언더슈트)" 가설을 반복 확인하고,
#        오픈루프 넛지량 Δ의 반복성을 재는 것.
#
#  흐름:  home → align red → [가드] → base_nudge ×N (빨강 쪽) → grasp red
#
#  가드:  로그에 "[마커이동]"이 있으면 = 초록 정렬 + 팔 들기까지 성공 = 팔이
#         하강 직전 자세로 올라가 있음 (옆걸음 단계는 베이스만 움직이므로
#         어떻게 끝나든 팔은 그대로). 없으면 앞 단계에서 실패한 것이므로
#         팔 위치 불명 → grasp 안 하고 즉시 중단(블라인드 하강 방지).
#
#  사용:  ./scripts/spike_red_nudge.sh            # 기본 7펄스
#         ./scripts/spike_red_nudge.sh 6          # 펄스 수만 바꿔서
#         ./scripts/spike_red_nudge.sh 7 -0.03 0.3   # 펄스수 / y속도 / 지속시간
# ============================================================================
set -u
cd "$(dirname "$0")/.."

NUDGES="${1:-7}"
NY="${2:--0.03}"
NDUR="${3:-0.3}"
LOG="/tmp/spike_red_$(date +%H%M%S).log"
PYENV=(env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH)

say() { printf '\n\033[1m== %s ==\033[0m\n' "$1" | tee -a "$LOG"; }
panic_stop() {
    printf '\n\033[31m!! 중단 — 베이스 정지\033[0m\n'
    ./scripts/lekiwi.sh stop >/dev/null 2>&1 || true
    exit 130
}
trap panic_stop INT TERM

echo "로그: $LOG"
echo "설정: 넛지 ${NUDGES}회 × (y=${NY}, ${NDUR}s)" | tee -a "$LOG"

say "home"
./scripts/lekiwi.sh home 2>&1 | tee -a "$LOG"

say "빨강 병 확인"
echo "  빨강 병이 걸이에 제대로 있는지 지금 보세요. 아니면 Ctrl-C." | tee -a "$LOG"
echo "  5초 후 진행..." | tee -a "$LOG"
sleep 5

say "align red"
./scripts/lekiwi.sh align red 2>&1 | tee -a "$LOG"

if ! grep -q '\[마커이동\]' "$LOG"; then
    say "가드: align/초록정렬 단계에서 실패 — 팔 자세 불명"
    echo "  → grasp 안 함. 로그($LOG) 확인 후 수동 대응하세요." | tee -a "$LOG"
    exit 1
fi

BAILX="$(grep -oE '마커x=[0-9]+' "$LOG" | tail -1 | grep -oE '[0-9]+')"
say "옆걸음 서보 포기 시점 마커x = ${BAILX:-?}"

say "오픈루프 넛지 ×${NUDGES} (빨강 쪽, y=${NY})"
for i in $(seq 1 "$NUDGES"); do
    printf '  [%d/%d] ' "$i" "$NUDGES" | tee -a "$LOG"
    "${PYENV[@]}" ./venv/bin/python scripts/base_nudge.py \
        --y "$NY" --duration "$NDUR" 2>&1 | tee -a "$LOG"
    sleep 0.3
done

say "grasp red"
./scripts/lekiwi.sh grasp red 2>&1 | tee -a "$LOG"

say "요약"
{
    echo "  넛지                : ${NUDGES} × (y=${NY}, ${NDUR}s)"
    echo "  서보 포기 마커x     : ${BAILX:-?}"
    grep -oE 'shoulder_lift=[0-9.]+\(목표[0-9.]+[^)]*\)' "$LOG" | tail -1 | sed 's/^/  하강 도달          : /'
    grep -E '그리퍼 도달값|판정:' "$LOG" | sed 's/^ */  /'
    echo "  (자동판정 '실패'여도 걸이에서 안 빠진 것뿐일 수 있음 — 눈으로 확인)"
    echo "  로그               : $LOG"
} | tee -a "$LOG"
