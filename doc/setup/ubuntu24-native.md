# 환경 설정 — Ubuntu 24.04 노트북 (네이티브)

[README](../../README.md) · 다음: [휠형 교육](../wheel/README.md) · DGX Spark 는 [별도 문서](dgx-spark.md)

## 범위와 대상

Ubuntu 24.04 설치 완료 상태에서 **ROS 2 Jazzy → 소스 다운로드 → 에셋·모델 준비 → colcon build → 오프라인 통합 실행**까지만 진행합니다. Ubuntu 자체 설치·파티션 변경은 다루지 않습니다.

대상 예: Ubuntu 24.04.5 LTS, Python 3.12, RAM 16 GB 이상, Intel 내장 GPU + GTX 1050 급. **CUDA 툴킷·PyTorch 는 필요 없습니다** (물리 시뮬·Nav2·MoveIt·OpenAI API 모두 CPU/OpenGL 로 동작). RAM 8 GB 이면 시뮬+Nav2+MoveIt+RViz 동시 실행 시 6~8 GB 를 써서 빠듯하므로 `world:=world_light.xml`, `viewer:=false`, RViz 하나만 쓰기를 권합니다.

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
bash mobile_openarm/scripts/00_check_system.sh
```

`00_check_system.sh` 는 OS·CPU·RAM·디스크·GPU·드라이버·세션(Xorg/Wayland)·Secure Boot·OpenGL 렌더러를 출력만 합니다(설치하지 않음). 이미 받은 폴더라면 그 폴더로 `cd` 한 뒤 `export OPENARM_WS="$PWD"` 만 실행합니다.

이후 명령은 이 저장소 루트에서 실행합니다. `~/openarm_mobile_ws`로 이름을 바꿀 필요가 없습니다.

저장소 루트 아래 두 폴더가 각각 colcon 워크스페이스입니다.

```text
openarms_llm_ex/
  mobile_openarm/   # 휠형: scripts/, src/, .venv-mobile/, build/ install/ log/
  tron1_openarm/    # TRON1 이족: mobile_openarm 위에 겹쳐(overlay) 빌드, PPO 학습용 .venv-biped/
```

## 2. 그래픽 준비

기본 실습은 **CUDA·PyTorch 없이** 가능합니다. 먼저 Ubuntu의 그래픽 환경으로 저사양 월드를 실행하고, 필요할 때 드라이버를 조정합니다.

```bash
sudo apt install -y mesa-utils libgl1-mesa-dri libglx-mesa0 libegl1 libegl-mesa0 libosmesa6 libglfw3
glxinfo -B
```

- GPU 사용 여부와 OpenGL 렌더러를 확인합니다. 그래픽 세션 안의 터미널에서 실행하세요. renderer 가 `llvmpipe` 나 `Mesa Intel` 이면 NVIDIA dGPU 가 쓰이지 않는 것입니다.
- **로그인 세션:** 로그인 화면 오른쪽 아래 톱니바퀴에서 **Ubuntu on Xorg** 를 선택하세요. Wayland 에서는 RViz2·MuJoCo 뷰어가 불안정하거나 검은 화면이 됩니다.
- **NVIDIA 드라이버 (GTX 1050 = Pascal):** 580 브랜치가 Pascal 을 지원하는 마지막 드라이버입니다(590 이상 미지원, `-open` 변형은 Turing 이상 전용). `mobile_openarm/scripts/01_install_nvidia_driver.sh` 는 **580 설치·`apt-mark hold`·`prime-select nvidia` 를 수행**하므로 필수 단계로 자동 실행하지 않습니다. Ubuntu 의 '추가 드라이버'에서 제공되는 버전을 먼저 확인하고, 필요할 때만 실행하세요. `cuda-drivers` 패키지는 설치하지 마세요(드라이버를 590+ 로 올림).
- **하이브리드 그래픽(Optimus):** 내장 GPU 로 렌더링되면 뷰어가 끊깁니다. `sudo prime-select nvidia` 후 재부팅하거나, 실행 시 `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` 를 앞에 붙입니다.
- **Secure Boot:** 켜져 있으면 드라이버 설치 중 MOK 비밀번호를 정하고, 재부팅 시 파란 화면에서 'Enroll MOK' → 비밀번호를 입력해야 드라이버 모듈이 로드됩니다(아니면 `nvidia-smi` 오류).
- 드라이버를 바꾸면 재부팅 후 `nvidia-smi` 와 `glxinfo -B | grep renderer` 로 확인합니다. NVIDIA 를 쓰지 않아도 전체 실습은 가능합니다. 소프트웨어 렌더링이 필요하면 실행 터미널에서 `export LIBGL_ALWAYS_SOFTWARE=1` 을 시도할 수 있습니다(CPU 부하 증가).

## 3. ROS 2 Jazzy 설치

```bash
bash mobile_openarm/scripts/03_install_ros2_jazzy.sh
source /opt/ros/jazzy/setup.bash
ros2 --help
command -v colcon
```

스크립트는 ROS apt 저장소 등록, Desktop/RViz, Nav2, SLAM Toolbox, MoveIt, teleop, Python 도구 설치와 rosdep 초기화를 수행합니다. **`sudo apt upgrade -y`도 실행하므로 사전에 내용을 확인하고 진행**하세요. 네트워크·sudo가 필요합니다.

## 4. 에셋·모델 생성 및 빌드

```bash
cd "$OPENARM_WS"
bash mobile_openarm/scripts/04_setup_workspace.sh
source /opt/ros/jazzy/setup.bash
source mobile_openarm/.venv-mobile/bin/activate
source mobile_openarm/install/setup.bash
ros2 pkg prefix mobile_openarm_sim
ros2 pkg prefix mobile_openarm_apps
```

`04_setup_workspace.sh`가 하는 일 (경로는 `mobile_openarm/` 기준):

1. `scripts/setup_venv_mobile.sh`로 Python 가상환경 `.venv-mobile` 생성(`--system-site-packages`) 후 MuJoCo 3.5.0 / OpenAI / Pydantic 2 / Flask 설치. NumPy는 ROS(apt) 버전으로 고정
2. `third_party/openarm_mujoco`에 OpenArm 에셋 clone 및 메시 복사
3. venv의 Python으로 휠+OpenArm MJCF, 월드, URDF, SRDF, 지도 생성
4. EGL/OSMesa로 자체 점검 시도
5. rosdep 후 venv의 Python으로 `python -m colcon build --symlink-install --base-paths src` → `ros2 run` 실행 파일이 `.venv-mobile`의 Python으로 실행됨
6. `~/.bashrc`에 `source <저장소>/mobile_openarm/install/setup.bash` 추가

`.venv-mobile`은 SLAM·Nav2·MoveIt 연동 노드, LLM 계획기, Flask 대시보드가 함께 쓰는 ROS 실행용 환경입니다. 자세한 구성은 [Python 가상환경](python-venv.md)을 참고하세요. `ros2 launch`/`ros2 run`은 활성화 없이도 이 venv를 쓰지만, `python3`로 도구를 직접 실행할 때는 위처럼 `activate`가 필요합니다.

**주의:** 스크립트는 자체 점검과 rosdep 실패를 무시할 수 있습니다(`|| true`). 마지막에 빌드 완료라고 나와도 센서 렌더링까지 검증됐다는 의미는 아닙니다. `Failed` 패키지가 없는지 빌드 요약을 확인하고, 필요하면 직접 점검하세요.

```bash
python3 mobile_openarm/src/mobile_openarm_sim/tools/selftest.py world_light.xml
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
cd "$OPENARM_WS"                           # 새 터미널이면: cd ~/Documents/openarms_llm_ex
source /opt/ros/jazzy/setup.bash
source mobile_openarm/install/setup.bash
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
| 패키지 없음 | `mobile_openarm/install/setup.bash`를 source했는지, colcon 빌드 실패 여부. 폴더 재구성 전의 `~/.bashrc` 줄(`source <저장소>/install/setup.bash`)이 남아 있으면 삭제 |
| 메시 파일 없음 | 단순 colcon만 하지 말고 `04_setup_workspace.sh`의 에셋 준비 완료 여부 |
| `No module named 'mujoco'` | `python3`로 직접 실행했다면 `source mobile_openarm/.venv-mobile/bin/activate`. `ros2 run`에서 나면 `head -n 1 mobile_openarm/install/mobile_openarm_apps/lib/mobile_openarm_apps/task_manager`가 venv Python인지 확인 후 `04`로 다시 빌드 |
| NumPy/cv2 import 오류 | `.venv-mobile`에 NumPy 2.x를 임의 설치하지 말 것. `mobile_openarm/.venv-mobile/ros-constraints.txt`의 고정 버전 확인 |
| 화면/카메라 오류 | `glxinfo -B`, 그래픽 세션, MuJoCo 백엔드 로그 확인. `viewer:=false`는 카메라 렌더까지 끄지 않음 |
| 8080 접속 실패 | dashboard 시작 로그와 포트 중복 확인. 12초 후에도 초기화가 더 걸릴 수 있음 |
| MuJoCo 뷰어가 안 열리거나 검은 화면 | Wayland 세션 또는 내장 GPU → Ubuntu on Xorg, `prime-select nvidia`, 안 되면 `viewer:=false` |
| 모델 로드에 RAM 1.9 GB | 시각 메시 47개(약 160만 면) → `world:=world_light.xml` (0.2 GB) |
| `colcon build --symlink-install` 에서 "option --editable not recognized" | setuptools 80 이상(conda 등) → conda 를 끄고 apt setuptools 68 을 쓰는 `.venv-mobile` 로 빌드. 안 되면 `--symlink-install` 없이 빌드 |
| Nav2 가 매초 "Detected jump back in time" | (고친 문제) 시뮬 시간 변환 반올림 — `bridge_node.py` `stamp()` 가 전체 ns 로 나눔. 같은 증상이면 최신 코드인지 확인 |
| controller_server "Controller period more then model dt" | MPPI 는 controller_frequency ≥ 1/model_dt(20 Hz) 필요. 20 Hz 유지, 대신 batch·time_steps 축소(적용됨) |
| "Timed out while waiting for action server ... compute_path_to_pose" | CPU 가 바쁨 → `default_server_timeout: 500`, planner 5 Hz (적용됨) |
| MoveIt "Start state out of bounds" (error -26) | 위치제어 오버슈트로 관절 한계 0.001 rad 초과 → 브리지가 `/joint_states` 를 한계 안으로 clamp (적용됨) |

### 실시간 배속 확인

```bash
ros2 launch mobile_openarm_sim sim.launch.py rviz:=false viewer:=false &
ros2 topic hz /clock     # 100Hz 가까우면 실시간. 크게 낮으면 lidar_rays:=180, camera_hz 낮추기, rate_factor:=0.5
nvidia-smi dmon -s u     # 뷰어·카메라가 GPU 를 쓰는지 확인
```

모든 노드가 시뮬 시간을 쓰므로 실시간보다 느려도 동작 결과는 같습니다. 전원 연결·성능 모드에서 실행하세요.

설치가 끝났다면 **[휠형 교육](../wheel/README.md)** 으로 이동합니다. TRON1 은 [별도 모델 생성·빌드](../tron1/day1.md)가 추가로 필요합니다.
