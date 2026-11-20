# 환경 설정 — NVIDIA DGX Spark (GB10, ARM64)

[README](../../README.md) · 다음: [휠형 교육](../wheel/README.md) · [Python 가상환경](python-venv.md)

## 범위

| 항목 | 이 장비 (확인한 값) |
|---|---|
| OS | DGX OS 7 (Ubuntu 24.04.5 LTS 기반), `cat /etc/dgx-release` |
| CPU / 메모리 | ARM64(aarch64) 20코어, 128 GB **통합 메모리** (CPU·GPU 공유) |
| GPU | NVIDIA GB10 (Blackwell, compute capability 12.1) |
| 커널 / 드라이버 | `linux-nvidia-hwe-24.04` (예: 7.0.0-1025-nvidia) / `nvidia-driver-580-open` |
| CUDA | CUDA 13.0 툴킷이 미리 설치됨 (`/usr/local/cuda-13.0`). **cuDNN·NCCL 은 없음** |

이 문서는 **ROS 2 Jazzy → 가상환경 → colcon 빌드**까지만 다룹니다. 휠형은 4·5절, TRON1 은 6·7절이며 쓰려는 쪽만 설치해도 됩니다. 실행은 교육 문서([휠형](../wheel/README.md) · [TRON1](../tron1/day1.md))에서 시작합니다.

**설치 스크립트를 한 번에 돌리지 않고, 명령을 한 줄씩 복사해 붙여 넣으며** 진행합니다. 각 명령의 결과를 확인한 뒤 다음으로 넘어가세요. Ubuntu PC 용 `doc/setup/install_nvidia_driver.sh`·`install_ros2_jazzy.sh` 는 **DGX Spark 에서 실행하지 않습니다.**

## ⚠ 먼저 읽기: `sudo apt upgrade` 를 함부로 실행하지 마세요

DGX Spark 의 GPU 는 **NVIDIA 커널(`linux-nvidia-hwe`)과 그 커널에 맞춘 드라이버(`nvidia-driver-580-open`)가 함께 맞아야** 동작합니다. `sudo apt upgrade`, `sudo apt full-upgrade`, `sudo apt dist-upgrade` 는 커널·드라이버·CUDA 관련 패키지까지 한꺼번에 올릴 수 있고, 이 짝이 어긋나면 **재부팅 후 `nvidia-smi` 가 실패하거나 GPU 를 인식하지 못합니다.**

- Ubuntu PC 용 `doc/setup/install_ros2_jazzy.sh` 는 안에서 `sudo apt upgrade -y` 를 실행하므로 **DGX Spark 에서는 쓰지 마세요.** 아래 3절처럼 필요한 패키지만 설치합니다.
- `sudo apt update` (목록 새로 받기)는 안전합니다. 패키지를 바꾸지 않습니다.
- `sudo apt install <패키지>` 도 의존성 때문에 다른 패키지를 올릴 수 있으므로, **설치 전에 `--simulate` 로 `linux-*`, `nvidia-*`, `cuda-*` 가 바뀌는지 확인**합니다(3-4).
- 시스템 업데이트는 NVIDIA 가 안내하는 DGX OS 업데이트 절차(DGX Dashboard 의 업데이트 기능 등)로 하세요.

현재 상태를 먼저 기록해 두면 문제가 생겼을 때 비교할 수 있습니다.

```bash
nvidia-smi
```

```bash
uname -r
```

```bash
cat /etc/dgx-release
```

```bash
dpkg -l | grep -E "^ii +(nvidia-driver|linux-image-nvidia|cuda-toolkit-13)" | awk '{print $2, $3}'
```

**(선택) 커널·드라이버 고정.** 실수로 올라가지 않게 묶어 둘 수 있습니다. 대신 이 패키지의 보안 업데이트도 막히므로, NVIDIA 안내에 따라 업데이트할 때는 먼저 풀어야 합니다.

```bash
sudo apt-mark hold nvidia-driver-580-open linux-nvidia-hwe-24.04 linux-image-nvidia-hwe-24.04
```

```bash
apt-mark showhold
```

풀 때: `sudo apt-mark unhold nvidia-driver-580-open linux-nvidia-hwe-24.04 linux-image-nvidia-hwe-24.04`

## 1. 기본 도구 확인

```bash
sudo apt update
```

```bash
sudo apt install --simulate git curl python3-venv mesa-utils | grep -E "^Inst (linux|nvidia|libnvidia|cuda)" || echo "커널·드라이버 변경 없음"
```

`커널·드라이버 변경 없음` 이 나오면 설치합니다.

```bash
sudo apt install -y git curl python3-venv mesa-utils
```

## 2. 저장소 받기

```bash
mkdir -p ~/Documents && cd ~/Documents
```

```bash
git clone https://github.com/aha645/openarms_llm_ex.git
```

```bash
cd ~/Documents/openarms_llm_ex
```

이후 명령은 이 **저장소 루트**에서 실행합니다(다른 위치에 받았다면 그 경로로).

## 3. ROS 2 Jazzy 설치 (apt upgrade 없이)

ROS 2 Jazzy 는 Ubuntu 24.04 ARM64 를 공식 지원합니다. 이미 설치되어 있으면(`ls /opt/ros/jazzy`) 3-4 의 확인만 하고 넘어가도 됩니다.

### 3-1. 로케일·universe 저장소

```bash
locale
```

`UTF-8` 이 보이면 다음으로 넘어갑니다. 아니면:

```bash
sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

```bash
sudo add-apt-repository -y universe
```

### 3-2. ROS apt 저장소 등록 (`ros2-apt-source`)

이미 등록되어 있는지 확인합니다. `ii ros2-apt-source` 가 나오면 3-3 으로 넘어갑니다.

```bash
dpkg -l ros2-apt-source | grep ^ii
```

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && echo $ROS_APT_SOURCE_VERSION
```

```bash
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
```

```bash
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### 3-3. 목록 갱신

```bash
sudo apt update
```

### 3-4. 설치할 패키지가 커널·드라이버를 건드리는지 먼저 확인

```bash
export ROS_PKGS="ros-jazzy-desktop ros-dev-tools ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-moveit ros-jazzy-moveit-configs-utils ros-jazzy-teleop-twist-keyboard ros-jazzy-compressed-image-transport ros-jazzy-tf-transformations ros-jazzy-xacro liburdfdom-tools python3-opencv python3-flask python3-pip"
```

```bash
sudo apt install --simulate $ROS_PKGS | grep -E "^Inst (linux|nvidia|libnvidia|cuda)" || echo "커널·드라이버 변경 없음"
```

**`커널·드라이버 변경 없음` 일 때만** 다음을 실행합니다. 무엇인가 출력되면 멈추고 그 패키지가 왜 바뀌는지 확인하세요.

### 3-5. 설치

```bash
sudo apt install -y $ROS_PKGS
```

### 3-6. rosdep 초기화

```bash
sudo rosdep init
```

`already exists` 메시지는 이미 초기화된 것이므로 무시합니다.

```bash
rosdep update
```

### 3-7. 확인

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
ros2 doctor
```

`All <N> checks passed` 가 나오면 정상입니다. 전체 보고서는 `ros2 doctor --report > ~/ros2_doctor_report.txt` 로 파일에 저장해서 봅니다(`| head` 로 자르면 출력이 끊기며 `BrokenPipeError` 가 나지만 무시해도 되는 현상입니다).

```bash
grep -qF "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

마지막 줄은 새 터미널마다 ROS 를 자동으로 불러오게 `~/.bashrc` 에 한 줄을 추가합니다(이미 있으면 추가하지 않음).

## 4. 휠형 워크스페이스 — 가상환경 `.venv-mobile`

`mobile_openarm/scripts/setup_workspace.sh` 가 하는 일을 한 줄씩 나눠 실행합니다. 가상환경을 쓰는 이유는 [Python 가상환경](python-venv.md)에 있습니다. 모든 명령은 저장소 루트에서 시작합니다.

```bash
cd ~/Documents/openarms_llm_ex && export PYTHONNOUSERSITE=1
```

`PYTHONNOUSERSITE=1` 은 `~/.local` 에 남은 예전 pip 설치가 섞이지 않게 합니다(이 터미널에만 적용).

```bash
/usr/bin/python3 -m venv --system-site-packages mobile_openarm/.venv-mobile
```

```bash
touch mobile_openarm/.venv-mobile/COLCON_IGNORE
```

ROS(apt) 의 NumPy 버전을 제약 파일에 적습니다. pip 가 NumPy 2.x 를 들여오면 cv_bridge 등이 깨지기 때문입니다.

```bash
env -u PYTHONPATH /usr/bin/python3 -c 'import numpy; print("numpy==" + numpy.__version__)' > mobile_openarm/.venv-mobile/ros-constraints.txt && cat mobile_openarm/.venv-mobile/ros-constraints.txt
```

```bash
env -u PYTHONPATH mobile_openarm/.venv-mobile/bin/python -m pip install -c mobile_openarm/.venv-mobile/ros-constraints.txt 'mujoco==3.5.0' 'openai>=1.60' 'pydantic>=2.5' flask
```

```bash
mobile_openarm/.venv-mobile/bin/python -c 'import numpy, cv2, mujoco, openai, pydantic; print("numpy", numpy.__version__, "mujoco", mujoco.__version__, "pydantic", pydantic.VERSION)'
```

이 장비에서 확인한 결과: `numpy 1.26.4 mujoco 3.5.0 pydantic 2.14.3`. NumPy 가 1.26.x 가 아니면 멈추고 제약 파일을 확인하세요. (같은 일을 하는 스크립트: `bash mobile_openarm/scripts/setup_venv_mobile.sh`)

## 5. 휠형 워크스페이스 — 자체 점검·빌드

> **터미널:** 앞 절과 **같은 터미널**에서 이어서 진행합니다. 새 터미널이라면 먼저 `cd ~/Documents/openarms_llm_ex && export PYTHONNOUSERSITE=1` 을 입력하세요.

> **모델은 저장소에 들어 있습니다.** MJCF·URDF·SRDF·지도·OpenArm 메시가 git 에 포함되어 있어 따로 받거나 만들지 않습니다.
> OpenArm 원본에서 다시 만들 때만 `bash mobile_openarm/scripts/regen_models.sh` 를 씁니다.

### 5-1. 시뮬레이션 단독 점검 (ROS 불필요)

패키지 폴더로 이동하고 가상환경의 Python 을 변수로 둡니다(`PY` 는 이 터미널에서만 유효).

```bash
cd ~/Documents/openarms_llm_ex/mobile_openarm/src/mobile_openarm_sim && PY=../../.venv-mobile/bin/python
```

```bash
MUJOCO_GL=egl $PY tools/selftest.py world_light.xml
```

`총 14개 중 실패 0개` 가 나오면 정상입니다(이 장비에서 확인).

### 5-2. colcon 빌드

```bash
cd ~/Documents/openarms_llm_ex/mobile_openarm && source /opt/ros/jazzy/setup.bash
```

rosdep 이 무엇을 설치하려는지 먼저 봅니다.

```bash
rosdep install --from-paths src --ignore-src -r -y --simulate
```

목록에 문제가 없으면(대부분 이미 설치되어 비어 있음) 실행합니다.

```bash
rosdep install --from-paths src --ignore-src -r -y
```

**`colcon build` 가 아니라 가상환경의 Python 으로** colcon 을 실행합니다. 그래야 `ros2 run` 실행 파일 첫 줄이 가상환경 Python 이 됩니다.

```bash
.venv-mobile/bin/python -m colcon build --symlink-install --base-paths src
```

```bash
head -n 1 install/mobile_openarm_apps/lib/mobile_openarm_apps/task_manager
```

`#!/.../mobile_openarm/.venv-mobile/bin/python` 이 나와야 합니다.

```bash
grep -qF "source $PWD/install/setup.bash" ~/.bashrc || echo "source $PWD/install/setup.bash" >> ~/.bashrc
```

설치가 끝나면 터미널마다 저장소 루트에서 다음 한 줄로 ROS 2, 가상환경 `.venv-mobile`, 휠형 패키지를 한 번에 불러옵니다(`bash` 로 실행하면 지금 터미널에 남지 않으므로 반드시 `source`).

```bash
source mobile_openarm/scripts/env.sh
```

휠형 설치는 여기까지입니다. 실행은 **[휠형 교육](../wheel/README.md)** 에서 시작합니다.

## 6. (TRON1) 워크스페이스 — 가상환경 `.venv-biped`·빌드

TRON1 은 휠형과 코드·패키지·가상환경을 나눠 쓰지 않는 **독립** 시스템입니다. 4·5절(휠형)을 건너뛰고 3절(ROS) 다음에 바로 해도 됩니다. 한 번에 하는 스크립트는 `bash tron1_openarm/scripts/setup_workspace.sh` 이고,
아래는 그것을 한 줄씩 나눈 것입니다. 모델·메시·보행 정책은 저장소에 들어 있어 따로 만들지 않습니다
(원본 LimX TRON1·OpenArm 에서 다시 만들 때만 `bash tron1_openarm/scripts/regen_models.sh`).

**터미널:** 🆕 새 터미널 하나에서 6절 끝까지 진행합니다.

```bash
cd ~/Documents/openarms_llm_ex/tron1_openarm && export PYTHONNOUSERSITE=1
```

① 시뮬용 가상환경 — ROS 의 rclpy·OpenCV·NumPy 를 같이 쓰도록 `--system-site-packages`

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-biped && touch .venv-biped/COLCON_IGNORE
```

② NumPy 를 ROS(apt) 버전으로 고정 (pip 가 2.x 를 깔면 cv_bridge 가 깨짐)

```bash
env -u PYTHONPATH /usr/bin/python3 -c 'import numpy; print("numpy==" + numpy.__version__)' > .venv-biped/ros-constraints.txt && cat .venv-biped/ros-constraints.txt
```

③ 패키지 설치 — mujoco(물리), openai·pydantic(LLM 계획기), flask(대시보드)

```bash
env -u PYTHONPATH .venv-biped/bin/python -m pip install -c .venv-biped/ros-constraints.txt 'mujoco==3.5.0' 'openai>=1.60' 'pydantic>=2.5' flask
```

④ ROS 를 깨끗하게 불러오기 (다른 워크스페이스가 섞이지 않게)

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH && source /opt/ros/jazzy/setup.bash
```

⑤ 시뮬 자체 점검 (ROS 없이: 서기·걷기·회전·팔 동작·LiDAR·카메라) — `총 10개 중 실패 0개`

```bash
cd src/tron1_openarm_sim && MUJOCO_GL=egl ../../.venv-biped/bin/python tools/selftest_tron1.py sf/world_light.xml deployed_policies/tron1sf_openarm_policy.npz; cd ../..
```

⑥ 빌드 — `src/` 의 TRON1 패키지 3 개(`tron1_openarm_sim`·`tron1_openarm_moveit_config`·`tron1_openarm_apps`, 휠형 코드는 쓰지 않음). **가상환경의 python 으로 colcon 을 실행**해야 실행 파일이 이 가상환경으로 돕니다

```bash
rosdep install --from-paths src --ignore-src -r -y; .venv-biped/bin/python -m colcon build --symlink-install --base-paths src
```

⑦ 확인

```bash
head -n 1 install/tron1_openarm_apps/lib/tron1_openarm_apps/task_manager
```

`#!/.../tron1_openarm/.venv-biped/bin/python` 이 나와야 합니다. 이후 TRON1 터미널은 저장소 루트에서 `source tron1_openarm/scripts/env.sh` 한 줄이면 ROS 2·`.venv-biped`·TRON1 패키지를 한 번에 불러옵니다(휠형 경로는 지움). 교육은 [TRON1 1일차](../tron1/day1.md)부터입니다.

## 7. (TRON1) 보행 PPO 학습용 가상환경 `.venv-biped-train` — JAX GPU

보행 정책을 직접 학습할 때만 필요합니다.

ROS 쪽 NumPy(1.26)와 JAX 가 요구하는 NumPy(2.x)가 충돌하므로 **시스템 패키지를 보지 않는 독립 가상환경**을 따로 만듭니다.
한 번에 하는 스크립트: `bash tron1_openarm/scripts/setup_venv_biped_train_ppo.sh` (GPU·드라이버를 보고 JAX 종류를 고름).

**터미널:** 🆕 새 터미널 하나에서 7절 끝까지 진행합니다. `~/.bashrc` 가 ROS 를 자동으로 불러오는 터미널이어도 되도록 모든 명령을 `env -u PYTHONPATH` 로 **ROS 의 `PYTHONPATH` 를 지우고** 실행합니다. 이 접두어를 빼지 마세요.

```bash
cd ~/Documents/openarms_llm_ex/tron1_openarm && export PYTHONNOUSERSITE=1
```

```bash
nvidia-smi --query-gpu=name,compute_cap,driver_version --format=csv
```

GB10 은 compute capability `12.1`, 드라이버 580 이상이므로 CUDA 13 용 JAX 를 씁니다.

```bash
env -u PYTHONPATH /usr/bin/python3 -m venv .venv-biped-train && touch .venv-biped-train/COLCON_IGNORE
```

```bash
env -u PYTHONPATH .venv-biped-train/bin/python -m pip install --upgrade pip
```

```bash
env -u PYTHONPATH .venv-biped-train/bin/python -m pip install 'jax[cuda13]' optax 'mujoco==3.5.0' 'mujoco-mjx==3.5.0' 'mujoco-warp==3.5.0.2' 'warp-lang==1.12.1' onnx onnxruntime pyyaml scipy
```

`warp-lang` 은 1.12.1 로 고정합니다. 최신판(1.19)은 MuJoCo MJX 3.5.0 과 맞지 않아 GPU 물리를 올리지 못합니다(`put_model: 'int' has no attribute 'WARP'`). 1.12.1 은 GB10(CUDA 13 드라이버)에서도 동작합니다.

JAX 는 소스 빌드하지 않고 공식 wheel 을 씁니다. `jax[cuda13]` 은 cuDNN·cuBLAS·NCCL 등 CUDA 런타임 wheel 을 venv 안에 함께 받습니다(약 3.5 GB). DGX OS 에는 CUDA 13.0 툴킷은 있지만 **cuDNN·NCCL 이 없어서** 필요합니다.

```bash
env -u PYTHONPATH XLA_PYTHON_CLIENT_PREALLOCATE=false .venv-biped-train/bin/python -c 'import jax, numpy; print("jax", jax.__version__, "numpy", numpy.__version__, jax.devices())'
```

`CudaDevice` 가 나와야 합니다. `CpuDevice` 만 나오면 GPU 를 못 쓰는 것이므로 `nvidia-smi` 와 설치한 JAX 변형을 확인하세요.

학습 실행(`bash scripts/train_ppo_gpu.sh 30 sf`)과 판정 기준은 [보행 학습 14-11절](../tron1/locomotion-rl.md)에 있습니다. DGX Spark 는 CPU 와 GPU 가 128 GB 를 함께 쓰므로 스크립트가 `XLA_PYTHON_CLIENT_PREALLOCATE=false` 로 JAX 의 GPU 메모리 선점을 끕니다.

## 자주 막히는 부분

| 증상 | 확인·조치 |
|---|---|
| 재부팅 후 `nvidia-smi` 실패 | 커널·드라이버 짝이 어긋남. `uname -r` 과 `dpkg -l \| grep nvidia-driver` 를 처음 기록과 비교. 직전에 `apt upgrade` 를 했다면 NVIDIA DGX OS 복구·업데이트 안내를 따름 |
| `apt install --simulate` 에 `linux-*`/`nvidia-*` 가 나옴 | 설치를 멈추고 어떤 패키지가 끌어오는지 `apt-cache depends` 로 확인. 필요하면 해당 패키지를 빼고 설치 |
| `No module named 'mujoco'` | `python3` 로 직접 실행했다면 `source mobile_openarm/scripts/env.sh` (TRON1 은 `source tron1_openarm/scripts/env.sh`). `ros2 run` 에서 나면 5-2(TRON1 은 6절 ⑦)의 shebang 확인 후 venv Python 으로 다시 빌드 |
| 그냥 `colcon build` 를 했음 | `/usr/bin/colcon` 은 시스템 Python 이라 실행 파일이 `/usr/bin/python3` 로 기록됨 → `.venv-mobile/bin/python -m colcon build ...` 로 다시 빌드 |
| JAX 가 `CpuDevice` 만 봄 | `pip show jax-cuda13-plugin` 확인, 드라이버 580 이상인지 확인 |
