# 환경 설정 A — 현재 노트북 / Ubuntu 24.04 네이티브

[README](../../README.md) · 다음: [휠형 1일차](../course/wheel-day1.md)

## 범위와 대상

Ubuntu 24.04 설치 완료 상태에서 **ROS 2 Jazzy → 소스 다운로드 → 에셋·모델 준비 → colcon build → 오프라인 통합 실행**까지만 진행합니다. Ubuntu 자체 설치·파티션 변경은 다루지 않습니다.

문서 재구성 시 현재 노트북에서 확인한 정보: Ubuntu 24.04.5 LTS, Python 3.12.3, RAM 약 19 GiB, Intel HD Graphics 630 + GTX 1050 Mobile. 당시 `ros2`, `/opt/ros`, `nvidia-smi`, 루트 `install/`은 확인되지 않았습니다. 이 정보는 설치 후 달라집니다. 이번 문서 작업에서 실제 설치·빌드는 수행하지 않았습니다.

## 1. 소스 받기

인터넷과 sudo 권한이 필요합니다. **이미 이 저장소를 받았다면 다시 clone하지 말고 기존 폴더를 사용**합니다.

```bash
sudo apt update
sudo apt install -y git
mkdir -p ~/Documents
cd ~/Documents
git clone https://github.com/aha645/openarms_llm_ex.git
cd openarms_llm_ex
export OPENARM_WS="$PWD"
bash scripts/00_check_system.sh
```

현재 노트북의 기존 폴더는 다음입니다.

```bash
cd /home/liky/Documents/openarms_llm_ex
export OPENARM_WS="$PWD"
```

이후 명령은 이 루트에서 실행합니다. `~/openarm_mobile_ws`로 이름을 바꿀 필요가 없습니다.

## 2. 그래픽 준비

기본 실습은 **CUDA·PyTorch 없이** 가능합니다. 먼저 Ubuntu의 그래픽 환경으로 저사양 월드를 실행하고, 필요할 때 드라이버를 조정합니다.

```bash
sudo apt install -y mesa-utils libgl1-mesa-dri libglx-mesa0 libegl1 libegl-mesa0 libosmesa6 libglfw3
glxinfo -B
```

- GPU 사용 여부와 OpenGL 렌더러를 확인합니다. 그래픽 세션 안의 터미널에서 실행하세요.
- GTX 1050은 Pascal 세대입니다. 기존 `scripts/01_install_nvidia_driver.sh`는 **580 설치·패키지 hold·PRIME 전환을 수행**하므로 필수 단계로 자동 실행하지 않습니다. Ubuntu의 ‘추가 드라이버’와 Secure Boot 상태, apt에서 제공되는 드라이버 지원을 먼저 확인하세요.
- 드라이버를 바꾸면 재부팅 후 확인합니다. NVIDIA를 설치하지 않았거나 활성화하지 않았다면 `nvidia-smi`가 없다고 해서 전체 실습이 불가능한 것은 아닙니다.
- 소프트웨어 렌더링이 필요하면 실행 터미널에서 `export LIBGL_ALWAYS_SOFTWARE=1`을 시도할 수 있습니다. CPU 부하는 커집니다.

## 3. ROS 2 Jazzy 설치

```bash
bash scripts/03_install_ros2_jazzy.sh
source /opt/ros/jazzy/setup.bash
ros2 --help
command -v colcon
```

스크립트는 ROS apt 저장소 등록, Desktop/RViz, Nav2, SLAM Toolbox, MoveIt, teleop, Python 도구 설치와 rosdep 초기화를 수행합니다. **`sudo apt upgrade -y`도 실행하므로 사전에 내용을 확인하고 진행**하세요. 네트워크·sudo가 필요합니다.

## 4. 에셋·모델 생성 및 빌드

```bash
cd "$OPENARM_WS"
bash scripts/04_setup_workspace.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix mobile_openarm_sim
ros2 pkg prefix mobile_openarm_apps
```

`04_setup_workspace.sh`가 하는 일:

1. `third_party/openarm_mujoco`에 OpenArm 에셋 clone 및 메시 복사
2. apt NumPy 버전을 유지하며 MuJoCo 3.5.0 / OpenAI / Pydantic 2 설치
3. 휠+OpenArm MJCF, 월드, URDF, SRDF, 지도 생성
4. EGL/OSMesa로 자체 점검 시도
5. rosdep 후 `colcon build --symlink-install --base-paths src`

**주의:** 스크립트는 자체 점검과 rosdep 실패를 무시할 수 있습니다(`|| true`). 마지막에 빌드 완료라고 나와도 센서 렌더링까지 검증됐다는 의미는 아닙니다. `Failed` 패키지가 없는지 빌드 요약을 확인하고, 필요하면 직접 점검하세요.

```bash
python3 src/mobile_openarm_sim/tools/selftest.py world_light.xml
```

실패하면 로그를 확인하고 수정한 뒤 재실행합니다. 그래픽 백엔드는 장치·세션에 따라 다르므로 EGL 실패만으로 CPU/화면 실행까지 실패한다고 단정하지 않습니다.

## 5. 첫 통합 실행 (API 키 불필요)

```bash
ros2 launch mobile_openarm_apps full_system.launch.py \
  planner:=offline world:=world_light.xml rviz:=false
```

브라우저: **http://localhost:8080**

시뮬+Nav2 → 약 5초 뒤 MoveIt → 약 12초 뒤 Task Manager·대시보드가 시작됩니다. 이는 고정 지연이지 준비 완료 판정은 아닙니다. 노드 초기화 로그를 기다리세요. 뷰어가 느리면 종료 후 `viewer:=false`를 추가합니다. 대시보드의 카메라는 계속 사용합니다.

새 터미널에서 확인:

```bash
cd /home/liky/Documents/openarms_llm_ex   # 다른 위치에 받았다면 해당 경로
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /task/status --field data
```

`state: idle`을 확인합니다. 시험 명령은 다른 터미널에서 보냅니다.

```bash
ros2 topic pub --once /task/command std_msgs/msg/String \
  "{data: '순찰하고 복귀해'}"
```

이 단계의 완료 기준은 패키지 탐색 성공, 통합 launch가 유지됨, 대시보드 접속, Task Manager `idle` 및 명령에 대한 상태 전환입니다. 운반까지의 상세 검증은 2일차에서 합니다. 종료는 launch 터미널의 `Ctrl+C`입니다.

## 자주 막히는 부분

| 증상 | 확인·조치 |
|---|---|
| 패키지 없음 | 현재 루트의 `install/setup.bash`를 source했는지, colcon 빌드 실패 여부 |
| 메시 파일 없음 | 단순 colcon만 하지 말고 `04_setup_workspace.sh`의 에셋 준비 완료 여부 |
| NumPy/cv2 import 오류 | ROS 환경에 NumPy 2.x를 임의 설치하지 말 것. 설치 스크립트의 제약 버전 확인 |
| 화면/카메라 오류 | `glxinfo -B`, 그래픽 세션, MuJoCo 백엔드 로그 확인. `viewer:=false`는 카메라 렌더까지 끄지 않음 |
| 8080 접속 실패 | dashboard 시작 로그와 포트 중복 확인. 12초 후에도 초기화가 더 걸릴 수 있음 |

설치가 끝났다면 **[휠형 1일차](../course/wheel-day1.md)**로 이동합니다. TRON1은 [별도 에셋·빌드 준비](../course/tron1-day1.md)가 추가로 필요합니다.
