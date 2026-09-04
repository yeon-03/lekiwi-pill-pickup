#!/usr/bin/env bash
# lekiwi06 (Ubuntu 24.04 noble, aarch64) 에 ROS2 Jazzy 를 네이티브 설치한다.
#
# 기존 파이(Debian trixie)는 ROS2 apt 저장소가 없어서 Docker 로 우회했지만,
# noble 은 Jazzy 의 공식 타깃이라 컨테이너가 필요 없다.
#
# 진행 상황은 ~/ros2_install.log 에 남는다.
set -uo pipefail
export DEBIAN_FRONTEND=noninteractive
SUDO() { echo " " | sudo -S -p "" "$@"; }

step() { echo; echo "########## $* ##########"; date -Is; }

step "1/6 기본 도구"
SUDO apt-get update -qq
SUDO apt-get install -y -qq software-properties-common curl gnupg lsb-release
SUDO add-apt-repository -y universe

step "2/6 ROS2 apt 소스 등록"
# 2024년부터 키링을 직접 넣는 대신 ros2-apt-source 패키지를 쓰는 방식으로 바뀌었다.
VER=$(curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' | awk -F'"' '{print $4}')
echo "ros-apt-source 버전: $VER"
CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
curl -sSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${CODENAME}_all.deb"
SUDO apt-get install -y -qq /tmp/ros2-apt-source.deb
SUDO apt-get update -qq

step "3/6 ROS2 Jazzy ros-base"
SUDO apt-get install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep

step "4/6 SLAM / 내비게이션"
SUDO apt-get install -y \
  ros-jazzy-cartographer-ros \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization

step "5/6 제어 / 로봇 기술 / 도구"
SUDO apt-get install -y \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-robot-state-publisher ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-tf2-tools \
  ros-jazzy-rmw-cyclonedds-cpp

step "6/6 마무리 (i2c 그룹, 환경설정)"
# BMI160 은 /dev/i2c-1 (root:i2c 660) 이라 그룹에 들어가야 읽을 수 있다.
SUDO usermod -aG i2c,dialout roboseasy
grep -q 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || {
  {
    echo ''
    echo '# ROS2 Jazzy'
    echo 'source /opt/ros/jazzy/setup.bash'
    echo 'export ROS_DOMAIN_ID=42'
  } >> ~/.bashrc
  echo ".bashrc 에 ROS2 환경 추가"
}
SUDO rosdep init 2>/dev/null || echo "rosdep 이미 초기화됨"
rosdep update 2>&1 | tail -2 || true

step "완료"
source /opt/ros/jazzy/setup.bash
echo "ros2 위치: $(command -v ros2)"
ros2 pkg list 2>/dev/null | wc -l | xargs echo "설치된 패키지 수:"
for p in cartographer_ros nav2_bringup ros2_controllers robot_localization; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 && echo "  $p 있음" || echo "  $p 없음"
done
df -h / | tail -1
