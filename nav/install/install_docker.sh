#!/usr/bin/env bash
# 르키위(Ubuntu 24.04 noble, aarch64)에 Docker 를 설치한다.
#
# ROS 스택 자체는 네이티브로 돌아간다. Docker 는 함대에 같은 환경을 복제하기
# 위한 것이다 -- 07·08번에는 이 이미지를 내려받아 쓰면 apt 설치를 반복하지
# 않아도 된다.
#
# 진행 상황은 ~/docker_install.log 에 남는다.
set -uo pipefail
export DEBIAN_FRONTEND=noninteractive

# 비밀번호를 stdin 으로 넣는다. 그래서 **이 함수로 파이프를 받을 수 없다** --
#   curl ... | SUDO gpg --dearmor -o key.gpg
# 처럼 쓰면 curl 출력 대신 비밀번호가 gpg 로 들어가 빈 파일이 만들어진다.
# (실제로 그렇게 만들어 docker.list 에 공백만 들어간 적이 있다.)
# 파일 내용을 넘겨야 할 때는 먼저 임시 파일에 쓰고 SUDO install/cp 로 옮길 것.
SUDO() { echo " " | sudo -S -p "" "$@"; }
step() { echo; echo "########## $* ##########"; date -Is; }

step "1/4 사전 패키지"
SUDO apt-get update -qq
SUDO apt-get install -y -qq ca-certificates curl gnupg

step "2/4 Docker apt 저장소"
SUDO install -m 0755 -d /etc/apt/keyrings

# 파이프를 쓰지 않는다 (위 SUDO 주석 참조). 임시 파일을 거친다.
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /tmp/docker.asc
gpg --batch --yes --dearmor -o /tmp/docker.gpg /tmp/docker.asc
SUDO install -m 0644 /tmp/docker.gpg /etc/apt/keyrings/docker.gpg
[ -s /etc/apt/keyrings/docker.gpg ] || { echo "키링이 비어 있다"; exit 1; }

CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
cat > /tmp/docker.list <<LIST
deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $CODENAME stable
LIST
SUDO install -m 0644 /tmp/docker.list /etc/apt/sources.list.d/docker.list
echo "저장소: $(cat /tmp/docker.list)"
SUDO apt-get update -qq

# 저장소가 실제로 붙었는지 확인한다. 여기서 걸러야 뒤의 install 이
# "no installation candidate" 로 줄줄이 실패하는 것을 막는다.
#
# 파이프를 쓰지 않는다. `cmd | grep -q PAT` 는 grep 이 첫 매칭에서 즉시
# 끝나면서 cmd 에 SIGPIPE 를 보내고, cmd 가 141 로 죽는다. 스크립트 상단의
# `set -o pipefail` 이 그 141 을 파이프라인 상태로 삼기 때문에 **매칭에
# 성공했는데 실패로 판정**된다. 실제로 이것 때문에 정상 저장소를 두고
# "찾지 못했다" 로 중단한 적이 있다.
POLICY=$(apt-cache policy docker-ce 2>/dev/null || true)
case "$POLICY" in
  *download.docker.com*) echo "저장소 확인됨" ;;
  *)
    echo "docker-ce 를 저장소에서 찾지 못했다. 위 저장소 줄과 apt update 출력을 확인할 것."
    printf '%s\n' "$POLICY" | head -8
    exit 1 ;;
esac

step "3/4 Docker 엔진"
SUDO apt-get install -y docker-ce docker-ce-cli containerd.io \
                        docker-buildx-plugin docker-compose-plugin

step "4/4 사용자 그룹"
# docker 그룹에 넣어야 sudo 없이 쓸 수 있다. 새로 로그인해야 반영된다.
SUDO usermod -aG docker "$USER"

step "완료"
SUDO docker --version
SUDO docker compose version 2>/dev/null | head -1
echo "주의: docker 그룹 반영은 재로그인 후. 그전에는 sudo docker 로 쓸 것."
df -h / | tail -1
