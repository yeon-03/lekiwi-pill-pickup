# LeKiwi 주행 스택 실행 환경 (ROS 2 Jazzy, arm64 Raspberry Pi 4)
#
# 이 이미지는 '환경'만 담는다. 우리 코드와 기체 고유 설정(lekiwi_profile.sh),
# 지도는 호스트 홈을 마운트해서 쓴다. 코드를 고칠 때마다 이미지를 다시 굽지
# 않아도 되고, 기체마다 다른 값이 이미지에 섞여 들어가지도 않는다.
#
#   빌드:  docker build -t lekiwi:jazzy .
#   실행:  bash run.sh              (device/network 인자가 많아 스크립트로 감쌌다)
#
# 패키지 목록은 install_ros2_jazzy.sh 와 같다. 그 스크립트로 네이티브 설치한
# lekiwi06 에서 전체 스택(SLAM + Nav2 + abo 왕복)이 돌아간 것을 확인했다.

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# --- ROS 패키지 -------------------------------------------------------------
# cartographer  : SLAM (4층)
# navigation2   : AMCL + 경로계획 + MPPI (5~6층)
# robot-state-publisher / xacro : URDF -> 정적 TF (1층)
# teleop / tf2-tools : 수동 주행과 진단
# rmw-cyclonedds : 기본 RMW 가 말썽일 때의 대안
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-cartographer-ros \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-tf2-tools \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# --- 파이썬 의존성 ----------------------------------------------------------
# 우리 노드가 실제로 쓰는 것만. apt 로 넣는 이유는 Ubuntu 24.04 가 PEP668 로
# 시스템 파이썬에 pip 설치를 막기 때문이다(--break-system-packages 를 쓰면
# apt 패키지와 충돌할 수 있다).
#   pyserial : ydlidar_node.py
#   numpy    : ydlidar/base/lidar_test/refine_pose
#   pillow   : refine_pose.py (지도 pgm 읽기)
#   pyyaml   : waypoints / 지도 yaml / abo 브리지
#   i2c-tools: IMU 진단 (bmi160 은 fcntl 로 직접 읽는다)
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-serial python3-numpy python3-pil python3-yaml \
      i2c-tools usbutils \
    && rm -rf /var/lib/apt/lists/*

# scservo_sdk 는 apt 에 없다. 버전을 고정한다 -- lekiwi01/06 에서 쓰던 것과 같다.
RUN pip3 install --no-cache-dir --break-system-packages feetech-servo-sdk==1.0.0

# --- 사용자 ----------------------------------------------------------------
# 호스트와 같은 uid/gid 로 만들어야 마운트한 홈에 쓴 파일의 소유자가 맞는다.
# dialout 은 /dev/ttyUSB0·ttyACM0, i2c 는 /dev/i2c-1 접근에 필요하다.
ARG USER=roboseasy
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USER} 2>/dev/null || true \
 && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USER} 2>/dev/null || true \
 && usermod -aG dialout,video,plugdev ${USER} \
 && (getent group i2c || groupadd i2c) >/dev/null \
 && usermod -aG i2c ${USER}

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_DOMAIN_ID=42
USER ${USER}
WORKDIR /home/${USER}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
