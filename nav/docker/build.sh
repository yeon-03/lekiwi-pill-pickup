#!/usr/bin/env bash
# 이미지를 굽는다.  bash build.sh
#
# Pi 4 에서 직접 구우면 오래 걸린다(apt 로 nav2/cartographer 를 받는다).
# 호스트 uid/gid 를 넣어야 마운트한 홈에 쓴 파일 소유자가 맞는다.
set -euo pipefail
cd "$(dirname "$0")"
docker build \
  --build-arg USER="$(id -un)" \
  --build-arg UID="$(id -u)" \
  --build-arg GID="$(id -g)" \
  -t "${LEKIWI_IMAGE:-lekiwi:jazzy}" .
echo
docker images | head -2
