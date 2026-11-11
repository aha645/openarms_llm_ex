#!/usr/bin/env bash
# TRON1 + OpenArm 보행 정책 PPO 학습용 Python 가상환경(.venv-biped) — ROS 와 완전히 분리
#   왜 따로 만드나:
#     ROS 2 Jazzy(apt)의 cv_bridge 등은 numpy 1.26 에 맞춰 빌드되어 있고, .venv-mobile 은 그 버전에 고정된다.
#     최신 JAX(GPU)·optax 는 numpy 2.x 를 요구하므로 같은 환경에 두면 한쪽이 깨진다.
#     → 시스템 site-packages 를 보지 않는 독립 venv 에 JAX·MuJoCo 를 설치한다 (numpy 는 pip 가 맞는 버전으로).
#   JAX 는 소스 빌드가 아니라 공식 wheel(CUDA 런타임 포함)을 설치한다. NVIDIA 드라이버만 있으면 된다.
#   사용: bash tron1_openarm/scripts/setup_venv_biped.sh           (11_train_policy.sh 가 없으면 자동 호출)
#         JAX_SPEC='jax[cuda12]' bash tron1_openarm/scripts/setup_venv_biped.sh   # JAX 변형 직접 지정
#   JAX_SPEC 를 안 주면 GPU 를 보고 고른다:
#     compute capability ≥ 7.5 (RTX 20xx 이후, DGX Spark GB10 등) → jax[cuda13]
#     그보다 낮음 (GTX 1050 = 6.1 등, CUDA 13 미지원)            → jax[cuda12]
#     NVIDIA GPU 없음                                             → jax[cpu]
set -euo pipefail
T1=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)     # tron1_openarm
VENV=$T1/.venv-biped
PYTHON=/usr/bin/python3

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 0
fi
(( $# == 0 )) || { echo "인자가 필요 없습니다. --help 참고" >&2; exit 2; }
[[ -x "$PYTHON" ]] || { echo "$PYTHON 가 없습니다" >&2; exit 1; }

# 1) JAX 변형 결정 (예전 11_train_policy.sh 의 RL_JAX 도 받아 줌)
JAX_SPEC=${JAX_SPEC:-${RL_JAX:-}}
if [[ -z "$JAX_SPEC" ]]; then
  CC=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -n 1 | tr -d ' ' || true)
  if [[ -z "$CC" ]]; then
    JAX_SPEC='jax[cpu]'
    echo "[알림] NVIDIA GPU(nvidia-smi)를 찾지 못해 CPU 용 JAX 를 설치합니다."
  elif awk -v c="$CC" 'BEGIN { exit !(c >= 7.5) }'; then
    JAX_SPEC='jax[cuda13]'
  else
    JAX_SPEC='jax[cuda12]'
  fi
  echo "GPU compute capability: ${CC:-없음} → $JAX_SPEC"
fi

# 2) 독립 가상환경 생성 (시스템 site-packages 사용 안 함)
if [[ -e "$VENV" && ! -f "$VENV/pyvenv.cfg" ]]; then
  echo "가상환경이 아닌 파일/폴더가 있어 덮어쓰지 않습니다: $VENV" >&2; exit 1
fi
if [[ -f "$VENV/pyvenv.cfg" ]]; then
  if ! grep -qi '^include-system-site-packages = false$' "$VENV/pyvenv.cfg"; then
    echo "기존 $VENV 가 시스템 패키지를 보도록 만들어져 있습니다. 지우고 다시 실행하세요: rm -rf $VENV" >&2
    exit 1
  fi
else
  # ROS 를 source 한 터미널이어도 ROS 의 PYTHONPATH 가 섞이지 않게
  if ! env -u PYTHONPATH PYTHONNOUSERSITE=1 "$PYTHON" -m venv "$VENV"; then
    echo "venv 생성 실패. 설치 후 다시 실행: sudo apt install python3-venv" >&2; exit 1
  fi
fi
touch "$VENV/COLCON_IGNORE"

# 3) 설치: JAX(GPU) + 학습·평가·ONNX 내보내기 의존성. mujoco 는 ROS 쪽과 같은 3.5.0 (모델 호환)
PIP=(env -u PYTHONPATH PYTHONNOUSERSITE=1 "$VENV/bin/python" -m pip)
"${PIP[@]}" install --upgrade pip
"${PIP[@]}" install "$JAX_SPEC" optax 'mujoco==3.5.0' onnx onnxruntime pyyaml scipy

# 4) 확인: numpy 2.x 로 분리됐는지, JAX 가 GPU 를 보는지
env -u PYTHONPATH PYTHONNOUSERSITE=1 XLA_PYTHON_CLIENT_PREALLOCATE=false JAX_SPEC="$JAX_SPEC" "$VENV/bin/python" - <<'PY'
import os, sys, numpy, jax, jax.numpy as jnp, optax, mujoco
print("Python :", sys.executable)
print("NumPy  :", numpy.__version__, "(ROS 쪽 numpy 와 별개)")
print("JAX    :", jax.__version__, "| optax:", optax.__version__, "| MuJoCo:", mujoco.__version__)
print("Devices:", jax.devices())
x = jnp.ones((1024, 1024)); print("matmul ok:", float((x @ x).sum()), "on", jax.default_backend())
if "cuda" in os.environ["JAX_SPEC"] and jax.default_backend() != "gpu":
    print("[경고] GPU 용 JAX 를 설치했지만 GPU 를 쓰지 못합니다. nvidia-smi / 드라이버 버전과 JAX_SPEC 을 확인하세요.")
PY

printf '\n준비 완료: %s\n' "$VENV"
echo "ROS 를 source 하지 않은 새 터미널에서 쓰는 것을 권장합니다:"
echo "  export PYTHONNOUSERSITE=1 && source $VENV/bin/activate"
echo "학습 실행: bash tron1_openarm/scripts/11_train_policy.sh [분]"
