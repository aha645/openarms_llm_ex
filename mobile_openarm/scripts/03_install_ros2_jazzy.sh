#!/usr/bin/env bash
# ROS 2 Jazzy (Ubuntu 24.04 Noble) + Nav2 + SLAM Toolbox + MoveIt 2 + 도구
#   주의: 안에서 'sudo apt upgrade -y' 를 실행한다. DGX Spark 는 커널·드라이버 짝이 깨질 수 있어 실행을 막는다.
set -e
if [ -f /etc/dgx-release ]; then
  echo "DGX Spark 입니다. 이 스크립트는 apt upgrade 를 포함하므로 실행하지 않습니다."
  echo "doc/setup/dgx-spark.md 3절대로 필요한 패키지만 설치하세요."; exit 1
fi
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
# ros2-apt-source 패키지: ROS 저장소 키/주소를 자동 관리 (공식 설치 문서 방식)
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}' || true)
if [ -z "$ROS_APT_SOURCE_VERSION" ]; then
  echo "ros-apt-source 버전을 GitHub API 에서 받지 못했습니다 (네트워크 또는 요청 제한). 잠시 후 다시 실행하세요."; exit 1
fi
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-moveit ros-jazzy-moveit-configs-utils \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-compressed-image-transport \
  ros-jazzy-tf-transformations ros-jazzy-xacro liburdfdom-tools \
  python3-opencv python3-flask python3-pip python3-venv
sudo rosdep init 2>/dev/null || true
rosdep update
grep -q "/opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "완료: 새 터미널에서 'ros2 doctor' 로 확인 (All checks passed)"
