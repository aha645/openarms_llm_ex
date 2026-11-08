# 환경 설정 B — Intel iMac / Docker의 Ubuntu 24.04

[README](../../README.md) · 다음: [휠형 1일차](../course/wheel-day1.md)

## 범위와 원리

대상은 기존 문서의 **Intel iMac 2019, RAM 32GB**입니다. Apple Silicon은 이 문서의 검증 대상이 아닙니다. macOS에 Ubuntu를 이중 부팅하거나 컨테이너 안에서 OS 설치 프로그램을 돌리지 않습니다. `osrf/ros:jazzy-desktop` 이미지가 **Ubuntu 24.04 + ROS 2 Jazzy**를 제공하고 Dockerfile이 Nav2·MoveIt·가상 데스크톱 등을 추가합니다.

```text
macOS Docker Desktop의 Linux VM
  → Ubuntu 24.04 / ROS 2 Jazzy 컨테이너
  → Mesa CPU 렌더링 → Xvfb → VNC / noVNC
```

Mac GPU 가속·CUDA·NVIDIA 드라이버·XQuartz는 기본 경로에서 사용하지 않습니다. Dockerfile/Compose를 소스 기준으로 검토한 절차이며, 이번 문서 작업에서 실제 iMac Docker 빌드·실행은 하지 않았습니다.

## 1. Mac 준비

1. 해당 macOS를 지원하는 **Docker Desktop for Mac (Intel)**을 설치하고 실행합니다. 지원 OS 범위는 [공식 설치 문서](https://docs.docker.com/desktop/setup/install/mac-install/)에서 확인하세요.
2. Docker 리소스 권장 시작값: CPU 6개, RAM 16GB, 디스크 64GB 이상. 사용 가능한 자원에 맞게 조절합니다.
3. Git이 없으면 macOS 명령줄 도구를 설치합니다(`xcode-select --install`).

Mac 터미널:

```bash
docker version
docker compose version
git --version
mkdir -p ~/Documents
cd ~/Documents
git clone https://github.com/aha645/openarms_llm_ex.git
cd openarms_llm_ex/docker
```

이미 받은 저장소가 있으면 clone을 생략하고 그 폴더의 `docker/`로 이동합니다.

## 2. Ubuntu + ROS 이미지 빌드·시작

Mac 터미널 (`docker/`):

```bash
docker compose build
docker compose up -d
docker compose ps
docker compose exec openarm bash
```

이제부터는 **컨테이너 안**입니다.

```bash
cd /root/openarm_mobile_ws
source /opt/ros/jazzy/setup.bash
grep PRETTY_NAME /etc/os-release
ros2 --help
command -v colcon
command -v rosdep
```

OS는 Ubuntu 24.04 계열이어야 합니다. `colcon` 또는 `rosdep`이 없다면 컨테이너에서 설치합니다. 베이스 이미지 태그는 바뀔 수 있어 명시적으로 확인합니다.

```bash
apt-get update
apt-get install -y ros-dev-tools
```

**컨테이너에서는 `scripts/03_install_ros2_jazzy.sh`를 실행하지 않습니다.** ROS는 이미지에 이미 있고, 현재 컨테이너는 root라 sudo를 전제하는 호스트용 설치 스크립트와도 다릅니다.

## 3. 에셋 준비·colcon 빌드

컨테이너 안:

```bash
cd /root/openarm_mobile_ws
# rosdep이 아직 초기화되지 않은 이미지에만 필요 (이미 초기화됐다는 메시지는 무시 가능)
rosdep init
rosdep update
bash scripts/04_setup_workspace.sh
source install/setup.bash
ros2 pkg prefix mobile_openarm_apps
```

`04`가 OpenArm 에셋·모델을 만들고 `colcon build --symlink-install --base-paths src`를 수행합니다. `rosdep init`이 이미 초기화되었다고 실패하면 다음 명령으로 진행하세요. `04` 내부 자체 점검·rosdep 실패가 무시될 수 있으므로 로그와 빌드 요약을 별도로 확인합니다.

저장소는 Mac 폴더와 공유되지만 컨테이너 내부 경로는 **`/root/openarm_mobile_ws`**로 고정됩니다. `build/`, `install/`, `log/`는 Compose named volume입니다. **Mac 호스트에서 colcon을 실행하거나 호스트의 install 경로를 source하지 마세요.**

## 4. 화면 및 통합 실행

Mac 브라우저:

```text
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
```

기본 VNC 비밀번호는 `openarm`입니다. 필요하면 `docker/docker-compose.yml`에서 변경 후 컨테이너를 재생성합니다. 포트는 localhost에만 바인딩되어 있습니다.

컨테이너 안:

```bash
source /opt/ros/jazzy/setup.bash
source /root/openarm_mobile_ws/install/setup.bash
ros2 launch mobile_openarm_apps full_system.launch.py \
  planner:=offline world:=world_light.xml viewer:=false rviz:=false
```

Mac 브라우저: **http://localhost:8080**

`viewer:=false`는 MuJoCo 별도 창만 끄며 대시보드 카메라는 유지됩니다. 시뮬/주행 → 5초 뒤 MoveIt → 12초 뒤 Task Manager·웹이 시작되며 실제 초기화는 더 걸릴 수 있습니다.

새 Mac 터미널에서 ROS 터미널을 추가하려면:

```bash
cd ~/Documents/openarms_llm_ex/docker
docker compose exec openarm bash
```

그 컨테이너 터미널에서:

```bash
source /opt/ros/jazzy/setup.bash
source /root/openarm_mobile_ws/install/setup.bash
ros2 topic echo /task/status --field data
```

`idle`을 확인하고 다른 컨테이너 터미널에서 다음을 보냅니다.

```bash
ros2 topic pub --once /task/command std_msgs/msg/String \
  "{data: '순찰하고 복귀해'}"
```

이 단계에서는 웹 접속·상태 수신·명령에 대한 상태 전환까지 확인합니다. 실제 운반 검증은 2일차에서 진행합니다.

## 5. 종료·재개와 문제 해결

launch 터미널에서 `Ctrl+C` 후 Mac의 `docker/`에서 실행합니다.

```bash
docker compose stop         # 일시 정지
docker compose start        # 재개
# 컨테이너 삭제 (소스와 named volume은 유지)
docker compose down
```

`docker compose down -v`는 빌드·설치 볼륨을 삭제하므로 평소에는 사용하지 않습니다. 재개 후 컨테이너 터미널에 들어가 환경을 source하고 launch를 다시 실행합니다.

| 증상 | 확인·조치 |
|---|---|
| Docker daemon 연결 실패 | Docker Desktop 실행 여부 |
| noVNC 접속 실패 | Mac에서 `docker compose logs openarm`, 6080 포트 충돌 |
| 렌더링 문제 | 컨테이너에서 `glxinfo -B`, llvmpipe 렌더러 확인 |
| 너무 느림 | light 월드, viewer/RViz 끄기, Docker CPU/RAM 확인 |
| ROS 패키지 없음 | 컨테이너에서 빌드 성공 여부와 `/root/openarm_mobile_ws/install/setup.bash` 확인 |

**옵션 주의:** `camera_hz`·`rate_factor`·`lidar_rays`는 현재 `sim.launch.py` 인자입니다. `nav.launch.py`와 `full_system.launch.py`가 모두 전달하는 것은 아니므로 모든 launch에 무작정 붙이지 않습니다. 먼저 `ros2 launch <패키지> <파일> --show-args`와 launch 소스를 확인하세요.

환경 설정 완료 후 **[휠형 1일차](../course/wheel-day1.md)**로 이동합니다. 교육의 T1/T2/T3는 각각 별도 **컨테이너 터미널**입니다.
