#!/usr/bin/env bash
# ============================================================================
# [학습용] 보행 정책 PPO 학습 가상환경 만들기 (GPU)  →  tron1_openarm/.venv-biped-train
#
#   시뮬용(.venv-biped)과 왜 따로인가:
#     학습에 쓰는 JAX(GPU)는 numpy 2.x 가 필요한데, ROS 는 numpy 1.26 이어야 한다 → 한 환경에 둘 수 없다.
#     그래서 ROS 와 완전히 분리된 가상환경을 만든다 (ROS 를 몰라도 됨).
#
#   필요: NVIDIA GPU + 드라이버 (CUDA 는 따로 설치 안 해도 됨 — JAX 가 같이 받아 온다)
#   사용법:  bash tron1_openarm/scripts/setup_venv_biped_train_ppo.sh
#            (12_train_mjx.sh 가 가상환경이 없으면 자동으로 불러 줌)
#   JAX 종류를 직접 고를 때:  JAX_SPEC='jax[cuda12]' bash ...
# ============================================================================
set -e

T1=$(cd "$(dirname "$0")/.." && pwd)   # tron1_openarm 폴더
VENV=$T1/.venv-biped-train             # 만들 가상환경 위치

# 1) GPU 확인 → 설치할 JAX 종류 결정 (_gpu_select.sh)
#      드라이버 ≥ 580 → jax[cuda13],  ≥ 525 → jax[cuda12] (예: RTX 3090 + 드라이버 550),  GPU 없음 → jax[cpu]
source "$T1/scripts/_gpu_select.sh"
JAX_SPEC=${JAX_SPEC:-$AUTO_JAX_SPEC}
echo "== GPU: ${GPU_NAME:-없음} (드라이버 $DRIVER_MAJOR) → $JAX_SPEC"
pin_gpu                                # GPU 가 여러 대면 가장 좋은 것 하나만 쓰게

# 2) 가상환경 만들기 — 시스템(ROS) 패키지를 보지 않는 독립 환경
if [ ! -f "$VENV/pyvenv.cfg" ]; then
  echo "== 가상환경 만들기: $VENV"
  env -u PYTHONPATH /usr/bin/python3 -m venv "$VENV"
fi
touch "$VENV/COLCON_IGNORE"
PIP="env -u PYTHONPATH PYTHONNOUSERSITE=1 $VENV/bin/python -m pip"

# 3) 패키지 설치
#    JAX + optax : 신경망·PPO  /  mujoco-mjx, mujoco-warp : GPU 물리  /  onnx : 실로봇 배포 형식으로 내보내기
echo "== 패키지 설치"
$PIP install -q --upgrade pip
$PIP install -q "$JAX_SPEC" optax 'mujoco==3.5.0' 'mujoco-mjx==3.5.0' onnx onnxruntime pyyaml scipy
case "$JAX_SPEC" in
  *cuda13*) $PIP install -q 'mujoco-warp==3.5.0.2' ;;
  *cuda12*) $PIP install -q 'mujoco-warp==3.5.0.2' 'warp-lang==1.12.1' ;;   # 드라이버 525~579 용 warp
  *)        echo "[알림] GPU 가 없어 GPU 물리(mujoco-warp)는 설치하지 않습니다" ;;
esac

# 4) 확인: JAX 가 GPU 를 보는지
env -u PYTHONPATH PYTHONNOUSERSITE=1 XLA_PYTHON_CLIENT_PREALLOCATE=false "$VENV/bin/python" -c "
import numpy, jax, mujoco
print('numpy', numpy.__version__, '| jax', jax.__version__, '| mujoco', mujoco.__version__)
print('장치:', jax.devices(), '→', jax.default_backend())
"
echo "== 완료: $VENV"
echo "   학습:  bash tron1_openarm/scripts/12_train_mjx.sh 30 sf      (30분, GPU 물리)"
