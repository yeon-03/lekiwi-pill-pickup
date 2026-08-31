#!/usr/bin/env bash
# 르키위만 단독으로 띄우고/끄고/집게 하는 원클릭 스크립트 (에이보와 무관).
#
# 사용법:
#   ./scripts/lekiwi.sh up            전부 기동 (SSH로 파이 호스트까지)
#   ./scripts/lekiwi.sh status        지금 상태 점검
#   ./scripts/lekiwi.sh view                   카메라 창 (창 모서리를 끌어 크기 조절 가능)
#   ./scripts/lekiwi.sh view --fullscreen      전체화면으로
#   ./scripts/lekiwi.sh view --panel-height 600  더 크게
#   ./scripts/lekiwi.sh pick green              약통 집기 (든 채로 끝냄)
#   ./scripts/lekiwi.sh align green             하강 직전까지만 (시차보정용)
#   ./scripts/lekiwi.sh grasp green              align 으로 멈춘 자세에서 하강만 이어서
#   ./scripts/lekiwi.sh pick green --grasp-lift 62   더 깊이 내려가게
#   ./scripts/lekiwi.sh home          팔을 홈 자세로
#   ./scripts/lekiwi.sh stop          베이스 즉시 정지
#   ./scripts/lekiwi.sh down          전부 종료
#
# 2026-08-26에 실제로 겪은 함정들을 전부 반영해둠 — 아래 주석 참고.
set -u

cd "$(dirname "$0")/.."
REPO="$PWD"
PY="$REPO/venv/bin/python"
# ⚠️ 파이는 DHCP라 IP가 바뀐다(2026-08-27에 .201 -> 응답없음, 노트북도 .115 -> .108).
#    그래서 고정값을 믿지 않고, 안 되면 서브넷을 훑어서 찾는다.
#    한 번 찾으면 아래 파일에 적어두고 다음부터 먼저 그걸 시도한다.
PI_CACHE="$HOME/.lekiwi_pi"
PI_USER="${LEKIWI_USER:-roboseasy}"
PI_DEFAULT="${LEKIWI_PI:-}"
[ -z "$PI_DEFAULT" ] && [ -r "$PI_CACHE" ] && PI_DEFAULT="$(cat "$PI_CACHE")"
[ -z "$PI_DEFAULT" ] && PI_DEFAULT="$PI_USER@192.168.0.201"
PI="$PI_DEFAULT"
HOST_IP="${PI#*@}"
SHM=/dev/shm/lekiwi_cam

# ⚠️ conda/ROS 환경이 섞여 들어오면 venv의 numpy/cv2와 ABI가 안 맞아 깨진다.
# 이 저장소의 모든 파이썬 실행은 반드시 이 래퍼를 거칠 것.
run() { env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH "$PY" "$@"; }
ssh_pi() { ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "$PI" "$@" 2>/dev/null; }

is_the_pi() {   # is_the_pi <ip> — 우리 키로 붙고 lerobot_venv 가 있는가
    [ -n "$(timeout 6 ssh -o BatchMode=yes -o ConnectTimeout=4 \
              -o StrictHostKeyChecking=accept-new "$PI_USER@$1" \
              'ls -d ~/lerobot_venv 2>/dev/null' 2>/dev/null)" ]
}

find_pi() {     # 서브넷에서 르키위 파이를 찾아 $PI/$HOST_IP 를 갱신한다
    if ping -c1 -W1 "$HOST_IP" >/dev/null 2>&1 && is_the_pi "$HOST_IP"; then
        return 0
    fi
    printf '  기존 주소(%s) 응답 없음 — 같은 네트워크에서 찾는 중...\n' "$HOST_IP"
    local base me alive=()
    me=$(ip -4 -o addr show scope global | awk '{print $4}' | head -1)
    base=$(echo "${me%/*}" | cut -d. -f1-3)
    local i
    for i in $(seq 2 254); do ping -c1 -W1 "$base.$i" >/dev/null 2>&1 && echo "$base.$i" & done \
        | sort -u > /tmp/lekiwi_scan.$$ ; wait 2>/dev/null
    while read -r ip; do [ -n "$ip" ] && alive+=("$ip"); done < /tmp/lekiwi_scan.$$
    rm -f /tmp/lekiwi_scan.$$
    for ip in "${alive[@]}"; do
        timeout 2 bash -c "echo > /dev/tcp/$ip/22" 2>/dev/null || continue
        if is_the_pi "$ip"; then
            PI="$PI_USER@$ip"; HOST_IP="$ip"
            echo "$PI" > "$PI_CACHE"
            printf '  \033[32m✔\033[0m 르키위 파이 발견: %s (다음부터 여기부터 시도)\n' "$ip"
            return 0
        fi
    done
    return 1
}

ok()   { printf '  \033[32m✔\033[0m %s\n' "$1"; }
bad()  { printf '  \033[31m✘\033[0m %s\n' "$1"; }
step() { printf '\n\033[1m%s\033[0m\n' "$1"; }

host_alive()   { [ -n "$(ssh_pi 'fuser /dev/ttyACM0 2>/dev/null')" ]; }
fixed_alive()  { run -c "import sys;sys.path.insert(0,'scripts');from fixed_cam_server import fixed_frame_available;sys.exit(0 if fixed_frame_available() else 1)" 2>/dev/null; }
bridge_alive() { run -c "import sys;sys.path.insert(0,'scripts');from robot_link import bridge_available;sys.exit(0 if bridge_available() else 1)" 2>/dev/null; }

wait_for() {   # wait_for <함수> <초> <설명>
    local fn=$1 limit=$2 desc=$3 i=0
    while [ $i -lt "$limit" ]; do
        if $fn; then ok "$desc"; return 0; fi
        sleep 1; i=$((i+1))
    done
    bad "$desc — ${limit}초 안에 준비 안 됨"
    return 1
}

start_host() {
    # ⚠️ 함정 1: lekiwi_host의 connection_time_s 기본값이 30초다. 그냥 띄우면
    #    30초 뒤 조용히 죽고, 그 뒤로는 팔이 "이동" 로그만 찍고 실제로 안 움직인다
    #    (2026-08-26에 이것 때문에 하루를 날림).
    # ⚠️ 함정 2: 호스트가 2개 겹치면 시리얼 포트를 서로 뺏어 "Port is in use!"로
    #    둘 다 죽는다. 반드시 먼저 정리하고 하나만 띄운다.
    ssh_pi 'pkill -f "lekiwi.lekiwi_host"; sleep 2; pkill -9 -f "lekiwi.lekiwi_host"; sleep 1' >/dev/null
    ssh_pi "setsid nohup ~/lerobot_venv/bin/python -m lerobot.robots.lekiwi.lekiwi_host \
              --robot.id=lekiwi01 --host.connection_time_s=36000 \
              >/tmp/lekiwi_host.log 2>&1 </dev/null & echo started" >/dev/null
}

spawn() {      # spawn <로그파일> <스크립트> [인자...]
    local log=$1; shift
    setsid nohup env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
        "$PY" "$@" >"$log" 2>&1 </dev/null &
    disown
}

cmd_up() {
    step "1/4  르키위 파이 호스트 ($HOST_IP)"
    if ! find_pi; then
        bad "네트워크에서 르키위 파이를 못 찾음"
        echo "     확인할 것: 르키위 전원이 켜져 있는지 / 같은 와이파이인지"
        echo "     주소를 아신다면: LEKIWI_PI=$PI_USER@<IP> $0 up"
        exit 1
    fi
    if host_alive; then ok "이미 실행 중"
    else
        start_host
        wait_for host_alive 25 "호스트 기동" || {
            echo "  --- 파이 로그 ---"; ssh_pi 'tail -12 /tmp/lekiwi_host.log'; exit 1; }
    fi

    step "2/4  고정 USB 카메라 서버 (노트북)"
    if fixed_alive; then ok "이미 실행 중"
    else
        # 카메라 인덱스는 재연결 때마다 밀리므로 자동탐색(--usb-index 기본 -1)
        spawn /tmp/fixed_cam.log scripts/fixed_cam_server.py
        wait_for fixed_alive 20 "고정캠 기동" || { echo "  --- 로그 ---"; tail -5 /tmp/fixed_cam.log; exit 1; }
    fi

    step "3/4  로봇 브릿지 (노트북 — 르키위 연결을 혼자 붙잡고 카메라/명령 중계)"
    if bridge_alive; then ok "이미 실행 중"
    else
        spawn /tmp/robot_bridge.log scripts/robot_bridge.py --lekiwi-host "$HOST_IP"
        wait_for bridge_alive 30 "브릿지 연결" || { echo "  --- 로그 ---"; tail -6 /tmp/robot_bridge.log; exit 1; }
    fi

    step "4/4  상태"
    cmd_status
    cat <<EOF

다음:
  ./scripts/lekiwi.sh view          카메라 창 (왼쪽 고정캠 / 오른쪽 베이스캠)
  ./scripts/lekiwi.sh pick green    집기 한 사이클

⚠️ 집기 전에 확인할 것: 고정캠에 손목의 **흰 동그란 스티커**가 보여야 합니다.
   안 보이면 로봇을 판 앞으로 옮기세요(status가 알려줍니다).
EOF
}

cmd_status() {
    host_alive   && ok "파이 호스트 (시리얼 점유 중)"        || bad "파이 호스트 꺼짐"
    fixed_alive  && ok "고정캠 서버"                          || bad "고정캠 서버 꺼짐"
    bridge_alive && ok "로봇 브릿지"                          || bad "로봇 브릿지 꺼짐"
    pgrep -f "scripts/live_view_all.py" >/dev/null && ok "카메라 창" || echo "  · 카메라 창 안 떠 있음 (view 로 띄우기)"
    if fixed_alive; then
        run - <<'PYEOF'
import sys; sys.path.insert(0, 'scripts')
import cv2
from color_detect import find_white_marker, find_bottles_on_board
img = cv2.imread('/dev/shm/lekiwi_cam/fixed.jpg')
m = find_white_marker(img)
found = [c for c in ('red', 'green', 'blue') if find_bottles_on_board(img, c)]
print(f'  · 보이는 약통: {", ".join(found) if found else "없음"}')
if m:
    print(f'  \033[32m✔\033[0m 손목 흰 마커 보임 ({m[0]:.0f},{m[1]:.0f}) — 집기 가능')
else:
    print('  \033[31m✘\033[0m 손목 흰 마커 안 보임 — 로봇을 판 앞으로 옮기세요')
PYEOF
    fi
    if bridge_alive; then
        run scripts/approach_board.py --report 2>/dev/null | sed 's/^/  · /'
    fi
}

cmd_view() {
    pgrep -f "scripts/live_view_all.py" >/dev/null && { ok "이미 떠 있음"; return; }
    local VIEW_ARGS=("$@")
    # ⚠️ env는 -u 옵션들이 먼저, 변수 대입은 그 뒤. 순서 바꾸면 env가 -u를 파일로 본다.
    setsid nohup env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
        DISPLAY="${DISPLAY:-:1}" "$PY" scripts/live_view_all.py "${VIEW_ARGS[@]}" \
        >/tmp/live_view.log 2>&1 </dev/null & disown
    sleep 3
    pgrep -f "scripts/live_view_all.py" >/dev/null && ok "카메라 창 띄움 (q로 종료)" \
        || { bad "실패"; tail -5 /tmp/live_view.log; }
}

cmd_pick() {
    local color=${1:-}; shift || true
    local PICK_EXTRA=("$@")
    case "$color" in red|green|blue) ;; *)
        echo "사용법: $0 pick {red|green|blue}"; exit 1;; esac
    bridge_alive || { bad "브릿지가 꺼져 있음 — 먼저 '$0 up'"; exit 1; }
    # 되돌려놓기는 기본으로 생략(사용자 요청 2026-08-26). 필요하면 pick-return.
    run scripts/pick_cycle.py --color "$color" --lekiwi-host "$HOST_IP" "${PICK_EXTRA[@]}"
}

cmd_home()  { run scripts/goto_pose.py --preset home --lekiwi-host "$HOST_IP"; }
cmd_stop()  { run scripts/base_nudge.py --lekiwi-host "$HOST_IP" \
                  --x 0 --y 0 --theta 0 --duration 0.2 >/dev/null && ok "베이스 정지"; }

cmd_down() {
    step "종료"
    cmd_stop 2>/dev/null || true
    for pat in "scripts/live_view_all.py" "scripts/robot_bridge.py" "scripts/fixed_cam_server.py"; do
        pkill -f "$pat" >/dev/null 2>&1 && ok "$(basename "$pat") 종료"
    done
    ssh_pi 'pkill -f "lekiwi.lekiwi_host"' >/dev/null && ok "파이 호스트 종료"
    true
}

case "${1:-}" in
    up)     cmd_up ;;
    status) step "상태"; cmd_status ;;
    view)   shift; cmd_view "$@" ;;
    pick)   shift; find_pi >/dev/null || true; cmd_pick "$@" ;;
    align)  shift; find_pi >/dev/null || true; cmd_pick "$@" --stop-after-align ;;
    grasp)  shift; find_pi >/dev/null || true; cmd_pick "$@" --grasp-only ;;
    home)   find_pi >/dev/null || true; cmd_home ;;
    stop)   find_pi >/dev/null || true; cmd_stop ;;
    down)   cmd_down ;;
    *)      sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
