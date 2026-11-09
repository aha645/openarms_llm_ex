# 프로젝트 폴더 아래 Python 가상환경 만들기

[README](../../README.md)

## 목적과 범위

`scripts/05_setup_venv.sh`는 실행 위치와 무관하게 저장소 루트에 환경을 생성합니다. x86/ARM64 모두 `/usr/bin/python3`를 사용하며, 장치별 바이너리는 해당 장치에서 새로 설치합니다. 다른 컴퓨터의 venv를 복사하지 마세요.

```text
openarms_llm_ex/
  .venv-ros/     # ROS 실행용: apt의 Python 패키지 접근 허용
  .venv-train/   # 학습용: 시스템 site-packages와 분리
```

이 스크립트는 **가상환경 생성과 선택적인 ROS 앱 의존성 설치만** 수행합니다. 모델 생성·colcon 빌드·GPU 학습 환경 구성까지 대체하지 않습니다. venv는 Git에서 제외하며 colcon 탐색 방지용 `COLCON_IGNORE`도 생성합니다. 기존 환경은 재사용하되 격리 설정이 다르면 덮어쓰지 않고 중단합니다.

## 1. 준비

```bash
sudo apt install -y python3-venv
```

가상환경을 만드는 기능을 설치합니다. venv 안에서는 자체 pip를 사용하므로 기본 생성에 시스템 `python3-pip`는 필수가 아닙니다.

저장소 루트에서 다음 명령을 실행합니다.

## 2. ROS용 환경

```bash
bash scripts/05_setup_venv.sh ros
```

`.venv-ros`를 `--system-site-packages` 방식으로 만듭니다. apt로 설치한 OpenCV·NumPy 등을 재사용할 수 있지만 완전 격리는 아닙니다. 이 명령만으로 라이브러리를 추가 설치하지 않습니다.

프로젝트 Python 의존성까지 설치하려면:

```bash
sudo apt install -y python3-numpy python3-opencv
```

```bash
bash scripts/05_setup_venv.sh ros --install
```

MuJoCo 3.5.0, OpenAI, Pydantic 2, Flask를 venv에 준비합니다. apt NumPy 버전을 제약 파일에 고정해 pip 의존성 처리 중 변경되지 않게 합니다. ARM64에서 wheel이나 그래픽 의존성 오류가 나오면 해당 로그를 확인해야 하며 Spark 검증 완료를 의미하지 않습니다.

새 터미널마다 저장소 루트에서:

```bash
export PYTHONNOUSERSITE=1
```

`~/.local`의 과거 사용자 패키지가 환경에 섞이지 않게 합니다.

```bash
source .venv-ros/bin/activate
```

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
python -c 'import sys; print(sys.executable)'
```

출력이 저장소의 `.venv-ros/bin/python`인지 확인합니다. 이미 빌드했다면 ROS 환경 뒤에 `source install/setup.bash`도 필요합니다.

### 기존 설치·빌드와의 관계 — 반드시 읽기

현재 `04_setup_workspace.sh`는 `pip --user --break-system-packages` 방식입니다. **이 venv 안에서 실행하지 마세요.** 이번 변경은 기존 스크립트를 자동 전환하지 않습니다. 처음 모델을 생성하는 절차까지 venv로 바꾸려면 기존 스크립트의 Python 설치·생성·빌드 단계를 함께 이식해야 합니다.

또한 venv 활성화만으로 기존 `ros2 run` 실행 파일의 Python이 바뀌지는 않습니다. 기존 빌드에 기록된 shebang을 확인하고 venv Python으로 다시 빌드해야 합니다. colcon이 이 Python에서 import되는지 먼저 확인합니다.

```bash
python -c 'import colcon_core'
```

에셋·모델 준비가 이미 끝난 작업 공간이라면, 기존 빌드와 섞지 않도록 별도 출력 경로를 사용해 빌드할 수 있습니다.

```bash
python -m colcon build --symlink-install --base-paths src --build-base .venv-ros/build --install-base .venv-ros/install
```

```bash
source .venv-ros/install/setup.bash
```

```bash
head -n 1 .venv-ros/install/mobile_openarm_apps/lib/mobile_openarm_apps/task_manager
```

첫 줄이 의도한 venv Python을 가리키는지 확인하세요. TRON1은 모델 준비 후 `--base-paths src tron1_openarm/src`로 두 트리를 포함해야 합니다. 과거 `install/setup.bash`가 자동 source되는 터미널보다 새 터미널에서 의도한 overlay만 불러오는 것이 좋습니다.

## 3. 학습용 환경

```bash
bash scripts/05_setup_venv.sh train
```

`.venv-train`을 독립 환경으로 만듭니다. JAX·CUDA·MJX를 자동 설치하지 않습니다. Spark의 ARM64·Blackwell·드라이버에 맞는 지원 조합을 별도로 선택해야 하기 때문입니다.

ROS 환경을 불러오지 않은 새 터미널에서:

```bash
export PYTHONNOUSERSITE=1
```

```bash
source .venv-train/bin/activate
```

```bash
python -c 'import sys; print(sys.executable)'
```

현재 터미널의 `PYTHONPATH`나 자동 source된 ROS overlay가 있다면 그것도 별도로 정리해야 합니다. venv는 환경 변수까지 완전히 격리하지 않습니다.

기존 `11_train_policy.sh`는 자체 환경을 만드는 별도 절차이므로 이 `.venv-train`을 자동 사용한다고 가정하지 마세요. 학습 패키지 설치 후 학습 Python을 직접 지정하거나 해당 스크립트도 별도로 대응해야 합니다. 현재 학습 환경의 MuJoCo 물리는 CPU rollout이며, GPU JAX 설치만으로 MJX 환경으로 바뀌지는 않습니다.

## 4. 환경 사용 종료

```bash
deactivate
```

현재 셸의 venv 선택을 해제합니다. 이미 불러온 ROS 환경이나 직접 설정한 `PYTHONNOUSERSITE`까지 되돌리지는 않습니다. 환경 간 전환에는 새 터미널을 사용하는 것이 명확합니다.
