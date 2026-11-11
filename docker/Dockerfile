# 주행형 OpenArm MuJoCo + ROS 2 Jazzy 실습용 이미지 (Intel Mac / x86_64 Linux / Windows WSL2 공용)
#   - GPU 가속 없음: macOS 의 Docker 컨테이너는 Mac GPU 에 접근할 수 없으므로 Mesa llvmpipe(CPU) 로 OpenGL 4.5 렌더
#   - 화면: 컨테이너 안 가상 데스크톱(Xvfb + fluxbox) → x11vnc(5901) → noVNC(6080, 브라우저)
FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul LANG=C.UTF-8 LC_ALL=C.UTF-8

# ROS 패키지(Nav2·SLAM·MoveIt) + 가상 데스크톱/VNC + 소프트웨어 OpenGL + 한글 글꼴
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
      ros-jazzy-moveit ros-jazzy-moveit-configs-utils \
      ros-jazzy-teleop-twist-keyboard ros-jazzy-compressed-image-transport \
      python3-pip python3-venv python3-opencv python3-flask python3-yaml git wget curl unzip nano \
      xvfb x11vnc novnc websockify fluxbox xterm dbus-x11 \
      mesa-utils libgl1-mesa-dri libglx-mesa0 libegl1 libegl-mesa0 libosmesa6 libglfw3 \
      fonts-noto-cjk fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo/OpenAI 를 pip 로 설치 (이미지 기본값).
#  - 워크스페이스 준비(mobile_openarm/scripts/04_setup_workspace.sh)는 이 위에 .venv-mobile
#    (--system-site-packages)을 만들므로, 여기서 설치한 패키지는 venv 안에서 그대로 재사용된다.
#  - numpy 는 ROS apt 버전과 '정확히 같은' 버전으로 고정 (numpy 2.x 가 들어오면 apt 의 cv2·cv_bridge 가 깨짐)
#  - --ignore-installed: apt 가 설치한 typing_extensions·idna 등을 pip 가 지우려다
#    "Cannot uninstall ... installed by debian" 로 실패하는 문제 회피 (새 버전은 /usr/local 에 따로 설치됨)
RUN NPV=$(python3 -c "import numpy; print(numpy.__version__)") && \
    echo "numpy==${NPV}" > /etc/pip-constraints.txt && \
    pip3 install --no-cache-dir --break-system-packages --ignore-installed -c /etc/pip-constraints.txt \
      "mujoco==3.5.0" "openai>=1.60" "pydantic>=2.5" && \
    python3 -c "import numpy, cv2, mujoco; print('numpy', numpy.__version__, 'cv2', cv2.__version__, 'mujoco', mujoco.__version__)"

# 소프트웨어 렌더링 / 가상 디스플레이 기본값
ENV DISPLAY=:1 \
    RESOLUTION=1600x900x24 \
    VNC_PASSWORD=openarm \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe \
    LP_NUM_THREADS=4 \
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    ROS_DOMAIN_ID=0 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

COPY start-desktop.sh /usr/local/bin/start-desktop.sh
RUN chmod +x /usr/local/bin/start-desktop.sh && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /root/openarm_mobile_ws/tron1_openarm/install/setup.bash ]; then source /root/openarm_mobile_ws/tron1_openarm/install/setup.bash; elif [ -f /root/openarm_mobile_ws/mobile_openarm/install/setup.bash ]; then source /root/openarm_mobile_ws/mobile_openarm/install/setup.bash; fi' >> /root/.bashrc

WORKDIR /root/openarm_mobile_ws
EXPOSE 5901 6080 8080
ENTRYPOINT ["/usr/local/bin/start-desktop.sh"]
CMD ["sleep", "infinity"]
