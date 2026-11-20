# 프로젝트 Python 가상환경 — `.venv-mobile` / `.venv-biped` / `.venv-biped-train`

[README](../../README.md) · [Ubuntu 24.04 + RTX 3090 환경 설정](ubuntu24-rtx3090.md)

## 1. 왜 가상환경이 세 개인가

휠형(`mobile_openarm/`)과 TRON1(`tron1_openarm/`)은 **서로 독립된 워크스페이스**입니다. 어느 한쪽만 설치해도 되고, 각자 자기 가상환경을 씁니다.
TRON1 은 여기에 GPU 학습용 가상환경이 하나 더 있습니다.

```text
openarms_llm_ex/
  mobile_openarm/                              # 휠형
    scripts/setup_workspace.sh                 # .venv-mobile → 자체 점검 → colcon build
    scripts/setup_venv_mobile.sh               # .venv-mobile 생성 (setup_workspace.sh 가 자동 호출)
    scripts/env.sh                             # 터미널마다 source: ROS 2 + .venv-mobile + 휠형 install
    .venv-mobile/                              # 휠형 ROS 실행용 (CPU)
  tron1_openarm/                               # TRON1 (휠형 설치 없이 이 폴더만으로)
    scripts/setup_venv_biped.sh                # .venv-biped 생성 (setup_workspace.sh 가 자동 호출)
    scripts/setup_workspace.sh                 # .venv-biped → 자체 점검 → colcon build
    scripts/env.sh                             # 터미널마다 source: ROS 2 + .venv-biped + TRON1 install
    scripts/setup_venv_biped_train_ppo.sh      # .venv-biped-train 생성 (train_ppo_gpu.sh 가 없으면 자동 호출)
    .venv-biped/                               # TRON1 ROS 실행·미션·정책 평가용 (CPU)
    .venv-biped-train/                         # TRON1 보행 PPO 학습용 (GPU)
```

| | `.venv-mobile` (휠형) | `.venv-biped` (TRON1 시뮬) | `.venv-biped-train` (TRON1 학습) |
|---|---|---|---|
| 하는 일 | 휠형 시뮬·ROS 브리지, SLAM·Nav2·MoveIt, LLM 계획기, 대시보드 | TRON1 시뮬·ROS 브리지·카메라, 집기 앱·작업 관리자·LLM·대시보드, 보행 정책 평가(`eval_policy.py`)·학습 보기(`watch_training.py`) | 보행 정책 PPO 학습(`train_ppo_gpu.py`), ONNX 내보내기 |
| 장치 | CPU | CPU | **GPU** |
| 설치 패키지 | mujoco 3.5.0, openai, pydantic v2, flask | (같음) | JAX(GPU)·optax·mujoco-mjx·mujoco-warp·onnx |
| 시스템 패키지 | 보임 (`--system-site-packages`) | 보임 | 안 보임 (독립) |
| NumPy | ROS(apt) 버전 **고정** (1.26) | ROS(apt) 버전 **고정** | pip 가 JAX 에 맞는 2.x |
| ROS 와 함께 | 함께 씀 | 함께 씀 | **ROS 와 분리** |

ROS 2 Jazzy 의 `cv_bridge` 등은 apt 의 NumPy 1.26 에 맞춰 빌드되어 있어, ROS 실행 환경에 NumPy 2.x 가 들어오면 import 오류가 납니다. 반대로 최신 JAX·optax 는 NumPy 2.x 를 요구합니다.
그래서 TRON1 은 ROS 로 도는 시뮬 환경(`.venv-biped`)과 GPU 학습 환경(`.venv-biped-train`)을 나눕니다. 학습 결과(`best.npz`)는 NumPy 배열 파일이라 시뮬 쪽에서 그대로 읽습니다.

venv 폴더는 Git 에서 제외되고(`.gitignore`), colcon 이 안의 패키지를 찾지 않도록 `COLCON_IGNORE` 가 생깁니다. 모든 스크립트가 실행 위치와 무관하게 자기 워크스페이스 아래에 만들며, 여러 번 실행해도 기존 환경을 재사용합니다. **다른 컴퓨터의 venv 를 복사하지 말고** 장치마다 새로 만드세요.

준비:

```bash
sudo apt install -y python3-venv
```

## 2. `.venv-mobile` — 휠형 ROS 실행용

보통은 따로 실행할 필요가 없습니다. `bash mobile_openarm/scripts/setup_workspace.sh` 가 먼저 이 환경을 만들고, 자체 점검과 colcon 빌드까지 이 환경의 Python 으로 실행합니다. 환경만 다시 만들거나 의존성만 다시 맞출 때:

```bash
bash mobile_openarm/scripts/setup_venv_mobile.sh
```

하는 일:

1. `mobile_openarm/.venv-mobile` 을 `--system-site-packages` 로 생성 → apt 로 설치한 ROS Python 패키지(rclpy, cv_bridge, OpenCV, NumPy, colcon)를 그대로 사용
2. apt NumPy 버전을 `ros-constraints.txt` 에 기록하고 그 버전으로 고정한 채 `mujoco==3.5.0`, `openai>=1.60`, `pydantic>=2.5`, `flask` 설치
3. NumPy 가 apt 버전 그대로인지, Pydantic 이 v2 인지 확인

`~/.local` 이나 시스템 Python 은 건드리지 않습니다. Ubuntu 24.04 apt 의 Pydantic v1 은 그대로 두고 venv 안의 v2 가 먼저 읽힙니다.

### 빌드와 실행에서 venv 가 쓰이는 방식

`mobile_openarm/scripts/setup_workspace.sh` 는 venv 의 Python 으로 colcon 을 실행합니다.

```bash
cd mobile_openarm
.venv-mobile/bin/python -m colcon build --symlink-install --base-paths src
```

colcon 은 빌드한 Python 경로를 `ros2 run` 실행 파일 첫 줄(shebang)에 기록합니다. 그래서 **`ros2 launch`/`ros2 run` 은 venv 를 활성화하지 않아도 `.venv-mobile` 의 Python 으로 실행**됩니다. 확인:

```bash
head -n 1 mobile_openarm/install/mobile_openarm_apps/lib/mobile_openarm_apps/task_manager
```

`#!/.../mobile_openarm/.venv-mobile/bin/python` 이 나와야 합니다. `/usr/bin/python3` 가 나오면 venv 밖에서 빌드된 것이므로 위 명령으로 다시 빌드하세요. 직접 `colcon build` 할 때도 같은 방식(`.venv-mobile/bin/python -m colcon build ...` 또는 venv 활성화 후 `python -m colcon build ...`)을 씁니다.

반면 `python3` 로 도구를 직접 실행할 때(`selftest.py`, 모델 생성 도구)는 venv 를 켜야 합니다. 새 터미널마다 저장소 루트에서 `env.sh` 를 `source` 하면 ROS 2(`/opt/ros/jazzy/setup.bash`), venv(`activate`), 휠형 패키지(`install/setup.bash`)를 한 번에 불러옵니다. 앞서 불러온 다른 워크스페이스·venv 경로는 지웁니다.

```bash
source mobile_openarm/scripts/env.sh
python -c 'import sys, mujoco; print(sys.executable, mujoco.__version__)'
```

출력 경로가 `mobile_openarm/.venv-mobile/bin/python` 이어야 합니다. (TRON1 은 이 venv 를 쓰지 않습니다 — 3절)

`~/.local` 에 예전 pip 설치(mujoco·openai·pydantic)가 남아 있으면 venv 보다 먼저 읽혀 섞일 수 있습니다. `setup_venv_mobile.sh` 가 발견하면 경고하고 지우는 명령을 알려 줍니다.

## 3. `.venv-biped` — TRON1 시뮬·미션용 (CPU)

보통은 따로 실행할 필요가 없습니다. ROS 2 Jazzy 설치 뒤 다음 한 줄이 이 환경을 만들고, 자체 점검과 colcon 빌드까지 이 환경의 Python 으로 실행합니다.

```bash
bash tron1_openarm/scripts/setup_workspace.sh
```

환경만 다시 만들 때: `bash tron1_openarm/scripts/setup_venv_biped.sh`

하는 일 (`setup_venv_biped.sh`):

1. `tron1_openarm/.venv-biped` 를 `--system-site-packages` 로 생성 → apt 의 ROS Python 패키지(rclpy, cv_bridge, OpenCV, NumPy, colcon) 그대로 사용
2. apt NumPy 버전을 `ros-constraints.txt` 에 기록하고 고정한 채 `mujoco==3.5.0`, `openai>=1.60`, `pydantic>=2.5`, `flask` 설치
3. NumPy 가 apt 버전 그대로인지 확인

`setup_workspace.sh` 는 이어서 `src/` 의 TRON1 패키지 3 개를 이 venv 의 Python 으로 빌드합니다.
`tron1_openarm_sim`(시뮬·브리지·카메라 서버·launch), `tron1_openarm_moveit_config`(MoveIt), `tron1_openarm_apps`(집기 스킬·작업 관리자·LLM·대시보드)이며, 휠형 코드는 쓰지 않습니다.
그래서 **TRON1 의 모든 `ros2 run`/`ros2 launch` 실행 파일이 `.venv-biped` 로 돕니다**. 확인:

```bash
head -n 1 tron1_openarm/install/tron1_openarm_apps/lib/tron1_openarm_apps/task_manager
# #!/.../tron1_openarm/.venv-biped/bin/python
```

`python3` 로 도구를 직접 실행할 때(`eval_policy.py`, `watch_training.py`, 모델 생성 도구)도 터미널마다 `env.sh` 한 줄이면 됩니다 (ROS 2 + `.venv-biped` + TRON1 패키지, 휠형 경로는 지움):

```bash
source tron1_openarm/scripts/env.sh
```

> 휠형(`mobile_openarm_*`)과 TRON1(`tron1_openarm_*`)은 서로 독립된 시스템이라 코드를 나눠 쓰지 않습니다. 두 시스템은 같은 구조에서 출발했으므로
> 한쪽에서 고친 버그가 다른 쪽에도 필요할 수 있습니다. 그때는 두 곳을 각각 고칩니다 (예: 집기 흐름은 `robot_skills.py`).

## 4. `.venv-biped-train` — TRON1 보행 PPO 학습용 (GPU)

```bash
bash tron1_openarm/scripts/setup_venv_biped_train_ppo.sh
```

`train_ppo_gpu.sh`(`train_ppo_cpu.sh`)가 환경이 없으면 자동으로 호출하므로 바로 학습을 시작해도 됩니다.

하는 일:

1. GPU 확인 후 JAX 변형 결정 (`JAX_SPEC` 으로 직접 지정 가능). GPU 가 여러 대면 compute capability 가 가장 높은 GPU 기준 (`scripts/_gpu_select.sh`)

   | 조건 | 설치 |
   |---|---|
   | 드라이버 ≥ 580 이고 compute capability ≥ 7.5 (DGX Spark GB10 등) | `jax[cuda13]` + `mujoco-warp` + `warp-lang==1.12.1` |
   | 드라이버 ≥ 525 (예: 드라이버 550 + RTX 3090) | `jax[cuda12]` + `mujoco-warp` + `warp-lang==1.12.1` |
   | NVIDIA GPU 없음 / 드라이버 < 525 | `jax[cpu]` |

   GPU 가 여러 대(예: GTX 1080 Ti + RTX 3090)면 학습 스크립트가 `CUDA_VISIBLE_DEVICES` 를 가장 좋은 GPU 로 고정합니다 (JAX 는 PCI 순서상 첫 GPU 에 계산을 올리기 때문).

2. `tron1_openarm/.venv-biped-train` 을 시스템 패키지가 **안 보이는** 독립 venv 로 생성 (ROS 의 `PYTHONPATH` 도 지우고 실행)
3. JAX, optax, `mujoco==3.5.0`(시뮬 쪽과 같은 버전 → 같은 모델 파일), `mujoco-mjx==3.5.0`, onnx, onnxruntime, pyyaml, scipy 설치 (+ GPU 면 MuJoCo Warp)
4. NumPy 버전, `jax.devices()` 를 출력해 GPU 사용 여부 확인

JAX 는 소스에서 빌드하지 않습니다. 공식 wheel 에 CUDA 런타임이 들어 있어 **NVIDIA 드라이버만 있으면** 됩니다(CUDA Toolkit 불필요). 자동 선택이 맞지 않으면:

```bash
rm -rf tron1_openarm/.venv-biped-train
JAX_SPEC='jax[cuda12]' bash tron1_openarm/scripts/setup_venv_biped_train_ppo.sh
```

학습 실행(`bash tron1_openarm/scripts/train_ppo_gpu.sh 30 sf`)과 학습 흐름은 [보행 학습 14-11절](../tron1/locomotion-rl.md)에 있습니다. 학습 스크립트를 직접 실행할 때는 `python` 앞에 `env -u PYTHONPATH` 를 붙입니다. venv 는 환경 변수까지 격리하지 않아서, ROS 가 설정한 `PYTHONPATH`(NumPy 1.26 용 패키지 경로)가 남아 있으면 JAX 쪽 NumPy 2.x 와 섞이기 때문입니다.

## 5. 사용 종료·재생성

```bash
deactivate                                   # 현재 셸의 venv 해제
rm -rf mobile_openarm/.venv-mobile           # 다시 만들기: bash mobile_openarm/scripts/setup_workspace.sh
rm -rf tron1_openarm/.venv-biped             # 다시 만들기: bash tron1_openarm/scripts/setup_workspace.sh
rm -rf tron1_openarm/.venv-biped-train       # 다시 만들기: bash tron1_openarm/scripts/setup_venv_biped_train_ppo.sh
```

venv 를 같은 경로에 다시 만들면 기존 빌드의 shebang 은 그대로 유효합니다. 저장소 폴더를 옮겼다면 venv 와 `build/ install/` 을 모두 지우고 설치 스크립트를 다시 실행하세요.
