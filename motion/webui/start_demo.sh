#!/usr/bin/env bash
# 시연 시작 — 로봇 호스트(lekiwi_host)가 안 켜져 있으면 SSH로 켜고,
# 이어서 이 PC에서 pick&place 웹 서버를 띄운다.
#
# 정지/비상정지는 웹 화면 버튼으로 하고, 이 스크립트가 켠 lekiwi_host는
# 그대로 켜둔 채로 둔다 (다음 시연 때 다시 SSH로 켤 필요 없게).
#
# 사용: ./start_demo.sh [-- <run_pickplace_ui.py 에 넘길 추가 인자...>]
# 예:   ./start_demo.sh -- --dry_run=true
set -euo pipefail

SSH_TARGET="${LEKIWI_SSH_TARGET:-roboseasy@223.194.139.15}"
ROBOT_ID="${LEKIWI_ROBOT_ID:-lekiwi01}"
ZMQ_CMD_PORT="${LEKIWI_ZMQ_CMD_PORT:-5555}"
ZMQ_OBS_PORT="${LEKIWI_ZMQ_OBS_PORT:-5556}"
REMOTE_IP="${SSH_TARGET#*@}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[start_demo] 로봇 호스트 상태 확인 중... ($SSH_TARGET)"
if ssh -o ConnectTimeout=8 -o BatchMode=yes "$SSH_TARGET" "pgrep -f lekiwi_host" >/dev/null 2>&1; then
  echo "[start_demo] lekiwi_host 이미 켜져 있음 — 그대로 사용"
else
  echo "[start_demo] lekiwi_host 꺼져 있음 — SSH로 켜는 중..."
  ssh -o ConnectTimeout=8 -o BatchMode=yes "$SSH_TARGET" \
    "nohup ~/start_lekiwi_host.sh $ROBOT_ID 14400 > ~/lekiwi_host.log 2>&1 & disown"

  echo -n "[start_demo] ZMQ 포트(${ZMQ_CMD_PORT}/${ZMQ_OBS_PORT})가 열리길 기다리는 중"
  ok=false
  for _ in $(seq 1 20); do
    if (exec 3<>"/dev/tcp/${REMOTE_IP}/${ZMQ_CMD_PORT}") 2>/dev/null; then
      exec 3<&- 3>&- 2>/dev/null || true
      ok=true
      break
    fi
    echo -n "."
    sleep 1
  done
  if [ "$ok" = true ]; then
    echo " OK"
  else
    echo " 실패"
    echo "[start_demo] error: ${ZMQ_CMD_PORT}번 포트가 20초 안에 열리지 않았습니다."
    echo "  로봇 쪽 로그 확인: ssh $SSH_TARGET 'tail -30 ~/lekiwi_host.log'"
    exit 1
  fi
fi

echo "[start_demo] 웹 서버 실행 중... (http://localhost:8000)"
cd "$SCRIPT_DIR/.."
if [ "${1:-}" = "--" ]; then
  shift  # 사용법 안내의 `-- <추가 인자>` 구분자는 파이썬 쪽엔 안 넘긴다
fi
exec env PYTHONNOUSERSITE=1 python webui/run_pickplace_ui.py \
  --robot.remote_ip="$REMOTE_IP" --robot.id="$ROBOT_ID" \
  --robot.port_zmq_cmd="$ZMQ_CMD_PORT" --robot.port_zmq_observations="$ZMQ_OBS_PORT" \
  "$@"
