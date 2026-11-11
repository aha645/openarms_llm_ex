# 프로젝트 Python 가상환경 — `.venv-mobile` / `.venv-biped`

[README](../../README.md) · [노트북 환경 설정](ubuntu24-native.md)

## 1. 왜 가상환경이 두 개인가

```text
openarms_llm_ex/
  mobile_openarm/
    scripts/setup_venv_mobile.sh   # .venv-mobile 생성 (04_setup_workspace.sh 가 자동 호출)
    .venv-mobile/                  # ROS 실행용: apt 의 rclpy·cv_bridge·NumPy 1.26 + mujoco·openai·pydantic v2·flask
  tron1_openarm/
    scripts/setup_venv_biped.sh    # .venv-biped 생성 (11_train_policy.sh 가 없으면 자동 호출)
    .venv-biped/                   # PPO 학습용: ROS 와 완전 분리, JAX(GPU)·optax·mujoco + NumPy 2.x
```

| | `.venv-mobile` | `.venv-biped` |
|---|---|---|
| 하는 일 | MuJoCo 시뮬·ROS 브리지, SLAM·Nav2·MoveIt 연동, LLM 계획기, Flask 대시보드 (휠형·TRON1 ROS 실행 공용) | TRON1+OpenArm 보행 정책 PPO 학습(팔 자세가 계속 바뀌는 상태), 평가, ONNX 내보내기 |
| 시스템 패키지 | 보임 (`--system-site-packages`) | 안 보임 (독립) |
| NumPy | ROS(apt) 버전 **고정** (Ubuntu 24.04 = 1.26) | pip 가 JAX 에 맞는 버전 설치 (2.x) |
| ROS 와 함께 | 함께 씀 | **ROS 를 source 하지 않은 터미널** 권장 |

ROS 2 Jazzy 의 `cv_bridge` 등은 apt 의 NumPy 1.26 에 맞춰 빌드되어 있어, ROS 실행 환경에 NumPy 2.x 가 들어오면 import 오류가 납니다. 반대로 최신 JAX·optax 는 NumPy 2.x 를 요구합니다. 한 환경에 둘 수 없으므로 학습 환경을 따로 둡니다.

venv 폴더는 Git 에서 제외되고(`.gitignore`), colcon 이 안의 패키지를 찾지 않도록 `COLCON_IGNORE` 가 생깁니다. 두 스크립트 모두 실행 위치와 무관하게 자기 워크스페이스 아래에 만들며, 여러 번 실행해도 기존 환경을 재사용합니다. x86/ARM64 모두 `/usr/bin/python3` 를 쓰므로 **다른 컴퓨터의 venv 를 복사하지 말고** 장치마다 새로 만드세요.

준비:

```bash
sudo apt install -y python3-venv
```

## 2. `.venv-mobile` — ROS 실행용

보통은 따로 실행할 필요가 없습니다. `bash mobile_openarm/scripts/04_setup_workspace.sh` 가 먼저 이 환경을 만들고, 모델 생성과 colcon 빌드까지 이 환경의 Python 으로 실행합니다. 환경만 다시 만들거나 의존성만 다시 맞출 때:

```bash
bash mobile_openarm/scripts/setup_venv_mobile.sh
```

하는 일:

1. `mobile_openarm/.venv-mobile` 을 `--system-site-packages` 로 생성 → apt 로 설치한 ROS Python 패키지(rclpy, cv_bridge, OpenCV, NumPy, colcon)를 그대로 사용
2. apt NumPy 버전을 `ros-constraints.txt` 에 기록하고 그 버전으로 고정한 채 `mujoco==3.5.0`, `openai>=1.60`, `pydantic>=2.5`, `flask` 설치
3. NumPy 가 apt 버전 그대로인지, Pydantic 이 v2 인지 확인

예전 방식(`pip install --user --break-system-packages --ignore-installed`)과 달리 `~/.local` 이나 시스템 Python 을 건드리지 않습니다. Ubuntu 24.04 apt 의 Pydantic v1 은 그대로 두고 venv 안의 v2 가 먼저 읽힙니다.

### 빌드와 실행에서 venv 가 쓰이는 방식

`04_setup_workspace.sh` 는 venv 의 Python 으로 colcon 을 실행합니다.

```bash
cd mobile_openarm
.venv-mobile/bin/python -m colcon build --symlink-install --base-paths src
```

colcon 은 빌드한 Python 경로를 `ros2 run` 실행 파일 첫 줄(shebang)에 기록합니다. 그래서 **`ros2 launch`/`ros2 run` 은 venv 를 활성화하지 않아도 `.venv-mobile` 의 Python 으로 실행**됩니다. 확인:

```bash
head -n 1 mobile_openarm/install/mobile_openarm_apps/lib/mobile_openarm_apps/task_manager
```

`#!/.../mobile_openarm/.venv-mobile/bin/python` 이 나와야 합니다. `/usr/bin/python3` 가 나오면 venv 밖에서 빌드된 것이므로 위 명령으로 다시 빌드하세요. 직접 `colcon build` 할 때도 같은 방식(`.venv-mobile/bin/python -m colcon build ...` 또는 venv 활성화 후 `python -m colcon build ...`)을 씁니다.

반면 `python3` 로 도구를 직접 실행할 때(`selftest.py`, `python3 -m mujoco.viewer`, 모델 생성 도구)는 활성화가 필요합니다. 저장소 루트에서:

```bash
source /opt/ros/jazzy/setup.bash
source mobile_openarm/.venv-mobile/bin/activate
source mobile_openarm/install/setup.bash        # TRON1 은 tron1_openarm/install/setup.bash
python -c 'import sys, mujoco; print(sys.executable, mujoco.__version__)'
```

출력 경로가 `mobile_openarm/.venv-mobile/bin/python` 이어야 합니다. TRON1 ROS 노드도 이 venv 로 빌드·실행합니다(`10_setup_tron1.sh`).

### 예전 `pip --user` 설치가 남아 있는 경우

폴더 재구성 전의 `04_setup_workspace.sh` 를 실행한 컴퓨터는 `~/.local` 에 mujoco·openai·pydantic 이 남아 있을 수 있습니다. `--system-site-packages` venv 는 `~/.local` 을 venv 보다 먼저 읽으므로 섞일 수 있습니다. `setup_venv_mobile.sh` 가 발견하면 경고하며, 다음 명령으로 정리합니다.

```bash
/usr/bin/python3 -m pip uninstall --user --break-system-packages mujoco openai pydantic pydantic-core
```

`python3` 를 직접 실행하는 터미널에서는 `export PYTHONNOUSERSITE=1` 로 `~/.local` 을 끌 수도 있습니다. 스크립트들은 내부에서 이 값을 설정합니다.

## 3. `.venv-biped` — TRON1 보행 PPO 학습용

```bash
bash tron1_openarm/scripts/setup_venv_biped.sh
```

`11_train_policy.sh` 가 환경이 없으면 자동으로 호출하므로 바로 학습을 시작해도 됩니다.

하는 일:

1. GPU 확인 후 JAX 변형 결정 (`JAX_SPEC` 으로 직접 지정 가능)

   | 조건 | 설치 |
   |---|---|
   | compute capability ≥ 7.5 (RTX 20xx 이후, DGX Spark GB10 등) | `jax[cuda13]` |
   | 7.5 미만 (GTX 1050 = 6.1 등, CUDA 13 미지원) | `jax[cuda12]` |
   | NVIDIA GPU 없음 | `jax[cpu]` |

2. `tron1_openarm/.venv-biped` 를 시스템 패키지가 **안 보이는** 독립 venv 로 생성 (ROS 의 `PYTHONPATH` 도 지우고 실행)
3. JAX, optax, `mujoco==3.5.0`(ROS 쪽과 같은 버전 → 같은 모델 파일 사용), onnx, onnxruntime, pyyaml, scipy 설치
4. NumPy 버전, `jax.devices()`, GPU 행렬곱을 출력해 GPU 사용 여부 확인

JAX 는 소스에서 빌드하지 않습니다. 공식 wheel 에 CUDA 런타임이 들어 있어 **NVIDIA 드라이버만 있으면** 됩니다(CUDA Toolkit 불필요). 자동 선택이 맞지 않으면:

```bash
rm -rf tron1_openarm/.venv-biped
JAX_SPEC='jax[cuda12]' bash tron1_openarm/scripts/setup_venv_biped.sh
```

### 학습 실행

```bash
bash tron1_openarm/scripts/11_train_policy.sh 90     # 90분 학습 → 평가
```

`11_train_policy.sh` 는 `.venv-biped` 의 Python 을 `PYTHONPATH` 없이 실행합니다. 학습 환경(`tron1_openarm/rl/tron1_env.py`)은 에피소드 도중에도 팔 목표 자세를 이름 자세·무작위 자세로 1.5~3초에 걸쳐 계속 바꾸면서 보행을 학습합니다. 신경망(PPO 업데이트)은 JAX GPU, MuJoCo 물리 rollout 은 CPU 스레드에서 돕니다. JAX 가 GPU 메모리를 미리 75% 잡지 않도록 `XLA_PYTHON_CLIENT_PREALLOCATE=false` 를 기본으로 둡니다(CPU·GPU 메모리를 공유하는 DGX Spark 에서 중요).

직접 실행할 때는 ROS 를 source 하지 않은 새 터미널에서:

```bash
export PYTHONNOUSERSITE=1
source tron1_openarm/.venv-biped/bin/activate
cd tron1_openarm/rl
python train_ppo.py --minutes 90 --init-std 0.08 --lr 2e-5 --out ../runs/my_run
```

`~/.bashrc` 가 ROS 를 자동 source 한다면 `env -u PYTHONPATH python ...` 처럼 `PYTHONPATH` 를 지우고 실행하세요. venv 는 환경 변수까지 격리하지 않습니다. 학습 결과(`best.npz`)는 NumPy 배열 파일이라 `.venv-mobile` 쪽 ROS 브리지에서 그대로 읽습니다.

## 4. 사용 종료·재생성

```bash
deactivate                                   # 현재 셸의 venv 해제
rm -rf mobile_openarm/.venv-mobile           # 다시 만들기: bash mobile_openarm/scripts/04_setup_workspace.sh
rm -rf tron1_openarm/.venv-biped             # 다시 만들기: bash tron1_openarm/scripts/setup_venv_biped.sh
```

`.venv-mobile` 을 지우고 다시 만들면 venv 경로가 같으므로 기존 빌드의 shebang 은 그대로 유효합니다. 저장소 폴더를 옮겼다면 venv 와 `build/ install/` 을 모두 지우고 `04`(TRON1 은 `10`도)를 다시 실행하세요. 환경 간 전환에는 새 터미널을 쓰는 것이 명확합니다.
