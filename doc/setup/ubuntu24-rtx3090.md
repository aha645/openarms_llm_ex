# 환경 설정 — Ubuntu 24.04 + NVIDIA RTX 3090 (네이티브)

[README](../../README.md) · 다음: [휠형 교육](../wheel/README.md) · [TRON1 교육](../tron1/day1.md) · DGX Spark 는 [별도 문서](dgx-spark.md)

## 범위와 대상

Ubuntu 24.04 가 설치된 NVIDIA RTX 3090 데스크톱에서 **소스 다운로드 → NVIDIA 드라이버·그래픽 확인 → ROS 2 Jazzy → 가상환경 → colcon 빌드**까지만 다룹니다.
휠형은 4절, TRON1 은 5절이며 쓰려는 쪽만 설치해도 됩니다. TRON1 보행 정책을 GPU 로 학습할 때만 6절의 학습용 가상환경을 더 만듭니다.
실행은 교육 문서([휠형](../wheel/README.md) · [TRON1](../tron1/day1.md))에서 시작합니다. Ubuntu 자체 설치·파티션 변경은 다루지 않습니다.

| 항목 | 이 문서의 기준 |
|---|---|
| OS | Ubuntu 24.04 LTS (x86_64), Python 3.12 |
| GPU | NVIDIA GeForce RTX 3090 (Ampere, compute capability 8.6, 24 GB) |
| NVIDIA 드라이버 | 525 이상 (이 저장소의 보행 학습은 드라이버 550 에서 검증, 580 도 동작) |
| RAM | 32 GB 이상 권장 (시뮬 + Nav2 + MoveIt + RViz 동시 실행에 6~8 GB, GPU 학습은 별도) |
| CUDA 툴킷·cuDNN | **설치하지 않습니다.** 시뮬·ROS 는 CPU/OpenGL 로 돌고, GPU 학습용 JAX wheel 이 CUDA 런타임을 가상환경 안에 함께 받습니다 |

GPU 가 여러 대인 PC(예: GTX 1080 Ti + RTX 3090)도 됩니다. 학습 스크립트가 compute capability 가 가장 높은 GPU 하나만 쓰도록 고정합니다(6절).

## 1. 소스 받기

인터넷과 sudo 권한이 필요합니다. **이미 이 저장소를 받았다면 다시 clone하지 말고 기존 폴더를 사용**합니다.

```bash
sudo apt update
sudo apt install -y git
mkdir -p ~/Documents
cd ~/Documents
git clone https://github.com/aha645/openarms_llm_ex.git
cd openarms_llm_ex
bash doc/setup/check_system.sh
```

`check_system.sh` 는 OS·CPU·RAM·디스크·GPU·드라이버·세션(Xorg/Wayland)·Secure Boot·OpenGL 렌더러를 출력만 합니다(설치하지 않음).

이후 명령은 이 저장소 루트에서 실행합니다. 교육 문서의 `~/openarm_mobile_ws` 도 이 폴더를 뜻하므로 이름을 바꿀 필요가 없습니다. 1~4절은 한 터미널에서 순서대로 진행하고, 터미널을 새로 열었다면 `cd ~/Documents/openarms_llm_ex`(다른 위치에 받았다면 그 경로)부터 입력하세요.

저장소 루트 아래 두 폴더가 각각 **서로 독립된** colcon 워크스페이스입니다.

```text
openarms_llm_ex/
  mobile_openarm/   # 휠형: scripts/, src/, .venv-mobile/, build/ install/ log/
  tron1_openarm/    # TRON1 이족: scripts/, src/, .venv-biped/(시뮬·미션), .venv-biped-train/(GPU 학습), build/ install/ log/
```

## 2. NVIDIA 드라이버·그래픽 확인

먼저 지금 드라이버가 동작하는지 봅니다.

```bash
nvidia-smi --query-gpu=name,driver_version,compute_cap,memory.total --format=csv
sudo apt install -y mesa-utils libgl1-mesa-dri libglx-mesa0 libegl1 libegl-mesa0 libosmesa6 libglfw3
glxinfo -B | grep -E "OpenGL renderer|OpenGL version"
```

- `NVIDIA GeForce RTX 3090, 550.xx, 8.6, 24576 MiB` 처럼 나오고 드라이버가 **525 이상**이면 드라이버는 그대로 둡니다. GPU 학습용 JAX 는 드라이버에 맞는 것을 자동으로 고릅니다(6절).
- `glxinfo` 의 renderer 가 `NVIDIA GeForce RTX 3090` 이어야 시뮬 뷰어·RViz 가 GPU 로 그립니다. `llvmpipe` 면 NVIDIA 드라이버가 화면에 쓰이지 않는 것입니다.
- **드라이버가 없거나 525 미만일 때만** Ubuntu 의 '추가 드라이버'에서 `nvidia-driver-580`(proprietary 또는 open)을 고르거나 아래 스크립트를 씁니다. 스크립트는 580 설치 + `apt-mark hold`(자동으로 590+ 로 올라가지 않게)를 하며, 끝나면 재부팅합니다.

  ```bash
  bash doc/setup/install_nvidia_driver.sh
  sudo reboot
  ```

- `cuda-drivers`·`cuda-toolkit` 패키지는 설치하지 마세요. 드라이버를 다른 버전으로 올려 학습용 JAX 선택이 바뀝니다.
- **Secure Boot:** 켜져 있으면 드라이버 설치 중 MOK 비밀번호를 정하고, 재부팅 시 파란 화면에서 'Enroll MOK' → 비밀번호를 입력해야 드라이버 모듈이 로드됩니다(아니면 `nvidia-smi` 오류).
- **로그인 세션:** 로그인 화면 오른쪽 아래 톱니바퀴에서 **Ubuntu on Xorg** 를 고르세요. Wayland 에서는 RViz2·MuJoCo 뷰어가 불안정하거나 검은 화면이 됩니다.

## 3. ROS 2 Jazzy 설치

같은 터미널에서 이어서 진행합니다(새 터미널이면 먼저 `cd ~/Documents/openarms_llm_ex`).

```bash
bash doc/setup/install_ros2_jazzy.sh
source /opt/ros/jazzy/setup.bash
ros2 --help
command -v colcon
```

스크립트는 ROS apt 저장소 등록, Desktop/RViz, Nav2, SLAM Toolbox, MoveIt, teleop, Python 도구 설치와 rosdep 초기화를 수행합니다. **`sudo apt upgrade -y`도 실행하므로 사전에 내용을 확인하고 진행**하세요(드라이버는 2절의 `apt-mark hold` 로 고정되어 함께 올라가지 않습니다. DGX Spark 에서는 스크립트가 스스로 중단합니다). 네트워크·sudo가 필요합니다. xacro·`check_urdf`(liburdfdom-tools)도 함께 설치합니다.

## 4. 휠형 워크스페이스 설치·빌드

같은 터미널에서 이어서 진행합니다. `install_ros2_jazzy.sh` 가 `~/.bashrc` 에 ROS 줄을 넣었으므로 새 터미널이어도 됩니다.

```bash
cd ~/Documents/openarms_llm_ex          # 다른 위치에 받았다면 그 경로
bash mobile_openarm/scripts/setup_workspace.sh
source mobile_openarm/scripts/env.sh       # ROS 2 + .venv-mobile + 휠형 패키지 (터미널마다)
ros2 pkg prefix mobile_openarm_sim
ros2 pkg prefix mobile_openarm_apps
```

`mobile_openarm/scripts/setup_workspace.sh`가 하는 일 (경로는 `mobile_openarm/` 기준):

1. `scripts/setup_venv_mobile.sh`로 Python 가상환경 `.venv-mobile` 생성(`--system-site-packages`) 후 MuJoCo 3.5.0 / OpenAI / Pydantic 2 / Flask 설치. NumPy는 ROS(apt) 버전으로 고정
2. 저장소에 들어 있는 모델(휠+OpenArm MJCF, 월드, URDF, SRDF, 지도, OpenArm 메시)이 있는지 확인 — 인터넷으로 원본을 받거나 다시 만들지 않음
   (OpenArm 원본에서 다시 만들 때만 `bash mobile_openarm/scripts/regen_models.sh`)
3. EGL/OSMesa로 자체 점검 시도
4. rosdep 후 venv의 Python으로 `python -m colcon build --symlink-install --base-paths src` → `ros2 run` 실행 파일이 `.venv-mobile`의 Python으로 실행됨
5. `~/.bashrc`에 `source <저장소>/mobile_openarm/install/setup.bash` 추가

설치가 끝나면 터미널마다 `source mobile_openarm/scripts/env.sh` 한 줄로 ROS 2, 가상환경 `.venv-mobile`, 휠형 패키지를 한 번에 불러옵니다(`bash` 로 실행하면 지금 터미널에 남지 않으므로 반드시 `source`). 가상환경 구성은 [Python 가상환경](python-venv.md)에 있습니다.

**주의:** 스크립트는 자체 점검과 rosdep 실패를 무시할 수 있습니다(`|| true`). 마지막에 빌드 완료라고 나와도 센서 렌더링까지 검증됐다는 의미는 아닙니다. `Failed` 패키지가 없는지 빌드 요약을 확인하고, 필요하면 직접 점검하세요.

```bash
source mobile_openarm/scripts/env.sh       # 위 블록과 같은 터미널이면 이미 불러옴
python3 mobile_openarm/src/mobile_openarm_sim/tools/selftest.py world_light.xml
```

`총 14개 중 실패 0개` 면 정상입니다.

## 5. TRON1 설치 (휠형과 독립)

ROS 2 Jazzy 설치(3절) 뒤 휠형 설치 없이 바로 할 수 있습니다. TRON1 은 휠형과 코드·패키지·가상환경을 나눠 쓰지 않는 독립 시스템입니다.

```bash
cd ~/Documents/openarms_llm_ex
bash tron1_openarm/scripts/setup_workspace.sh
source tron1_openarm/scripts/env.sh        # ROS 2 + .venv-biped + TRON1 패키지 (터미널마다)
ros2 pkg prefix tron1_openarm_apps
```

`setup_workspace.sh` 가 하는 일 (경로는 `tron1_openarm/` 기준):

1. `scripts/setup_venv_biped.sh` 로 가상환경 `.venv-biped` 생성(`--system-site-packages`, 휠형과 같은 구성, NumPy 는 ROS 버전 고정)
2. 시뮬 자체 점검 (ROS 없이 서기·걷기·회전·LiDAR·카메라·팔 동작, `총 10개 중 실패 0개` 면 정상)
3. 다른 워크스페이스 경로를 지운 깨끗한 ROS 환경에서, venv 의 Python 으로 `src/` 의 TRON1 패키지 3 개(`tron1_openarm_sim`·`tron1_openarm_moveit_config`·`tron1_openarm_apps`)를 colcon 빌드

모델·메시·보행 정책은 저장소에 들어 있습니다(원본에서 다시 만들 때만 `bash tron1_openarm/scripts/regen_models.sh`). `env.sh` 는 휠형을 불러온 터미널이어도 휠형 경로와 가상환경을 지우고 TRON1 것만 남깁니다.

## 6. (선택) TRON1 보행 학습용 가상환경 — JAX GPU

보행 정책을 직접 학습할 때만 만듭니다. 시뮬·미션(`.venv-biped`)과 따로 두는 이유는 ROS 의 NumPy(1.26)와 JAX 가 요구하는 NumPy(2.x)가 충돌하기 때문입니다.

```bash
cd ~/Documents/openarms_llm_ex
bash tron1_openarm/scripts/setup_venv_biped_train_ppo.sh
```

스크립트가 하는 일:

1. GPU 와 드라이버를 보고 JAX 종류를 고릅니다 (`tron1_openarm/scripts/_gpu_select.sh`).

   | 드라이버 | 설치 | RTX 3090 에서 |
   |---|---|---|
   | 525 ~ 579 (예: 550) | `jax[cuda12]` + `mujoco-warp 3.5.0.2` + `warp-lang 1.12.1` | 이 저장소의 보행 학습(v3~v10)을 한 조합 |
   | 580 이상 | `jax[cuda13]` + `mujoco-warp 3.5.0.2` + `warp-lang 1.12.1` | 동작 (DGX Spark 와 같은 조합) |

   `warp-lang` 은 두 경우 모두 1.12.1 로 고정합니다. 최신판(1.19)은 MuJoCo MJX 3.5.0 과 맞지 않아 GPU 물리를 올리지 못합니다. 자동 선택을 바꾸려면 `JAX_SPEC='jax[cuda12]' bash ...` 처럼 지정합니다.
2. GPU 가 여러 대면 compute capability 가 가장 높은 GPU 하나만 보이게 `CUDA_VISIBLE_DEVICES` 를 고정합니다 (JAX 는 PCI 순서상 첫 GPU 에 계산을 올리므로 구형 GPU 가 먼저 잡히는 것을 막음).
3. `tron1_openarm/.venv-biped-train` 을 시스템 패키지가 안 보이는 독립 가상환경으로 만들고 JAX·optax·`mujoco==3.5.0`·`mujoco-mjx`·MuJoCo Warp·onnx 를 설치합니다.
4. 마지막에 `장치: [CudaDevice(id=0)] → gpu` 처럼 JAX 가 GPU 를 잡았는지 출력합니다.

설치를 따로 확인할 때 (`env -u PYTHONPATH` 는 `~/.bashrc` 가 불러온 ROS 의 Python 경로를 지움):

```bash
cd ~/Documents/openarms_llm_ex/tron1_openarm
env -u PYTHONPATH .venv-biped-train/bin/python -c "import jax, warp; print(jax.devices(), 'warp', warp.__version__)"
```

`CudaDevice` 와 `warp 1.12.1` 이 나오면 됩니다. `CpuDevice` 만 나오면 드라이버(2절)와 설치된 JAX 종류(`pip show jax-cuda12-plugin` 또는 `jax-cuda13-plugin`)를 확인하세요.
학습 실행(`bash tron1_openarm/scripts/train_ppo_gpu.sh 30 sf`)과 판정은 [보행 학습](../tron1/locomotion-rl.md)에 있습니다. RTX 3090 에서 환경 4096 개로 초당 약 8만 정책 스텝이 나왔습니다.

## 자주 막히는 부분

| 증상 | 확인·조치 |
|---|---|
| `ros2 pkg prefix` 가 패키지를 못 찾음 | `source <로봇 폴더>/scripts/env.sh` 를 했는지, 빌드 요약에 `Failed` 패키지가 없는지 |
| 휠형 `setup_workspace.sh` 가 "모델 파일이 없습니다" 로 멈춤 | 모델·메시는 저장소에 들어 있음 → `git status` 로 지워진 파일을 확인해 되살리거나 `bash mobile_openarm/scripts/regen_models.sh` |
| `No module named 'mujoco'` | `python3` 로 직접 실행했다면 `source <로봇 폴더>/scripts/env.sh` 로 가상환경을 불러왔는지. `ros2 run` 에서 나면 아래처럼 실행 파일 첫 줄이 venv Python 인지 확인 후 설치 스크립트로 다시 빌드 |
| 실행 파일 첫 줄이 `/usr/bin/python3` | 그냥 `colcon build` 를 했음 → `.venv-mobile/bin/python -m colcon build --symlink-install --base-paths src` (TRON1 은 `.venv-biped`) |
| NumPy/cv2 import 오류 | venv 에 NumPy 2.x 를 임의로 설치하지 말 것. `<venv>/ros-constraints.txt` 의 고정 버전 확인 |
| `colcon build --symlink-install` 에서 "option --editable not recognized" | setuptools 80 이상(conda 등) → conda 를 끄고 apt setuptools 68 을 쓰는 venv 로 빌드. 안 되면 `--symlink-install` 없이 빌드 |
| 자체 점검이 렌더링에서 실패 | `glxinfo -B` 로 2절 확인. 점검 실패는 빌드를 멈추지 않으므로 빌드 성공과 따로 봄 |
| GPU 학습이 `put_model: 'int' has no attribute 'WARP'` 로 멈춤 | `warp-lang` 이 1.12.1 이 아님 → `env -u PYTHONPATH tron1_openarm/.venv-biped-train/bin/python -m pip install 'warp-lang==1.12.1'` |
| JAX 가 `CpuDevice` 만 봄 | 드라이버 525 미만이거나, 드라이버를 바꾼 뒤 가상환경을 다시 만들지 않음 → `rm -rf tron1_openarm/.venv-biped-train` 후 6절 다시 |

실행 파일 첫 줄 확인:

```bash
head -n 1 mobile_openarm/install/mobile_openarm_apps/lib/mobile_openarm_apps/task_manager   # 휠형
head -n 1 tron1_openarm/install/tron1_openarm_apps/lib/tron1_openarm_apps/task_manager       # TRON1
```
