#!/usr/bin/env bash
# TRON1 + OpenArm 보행 정책 PPO 학습용 Python 가상환경(.venv-biped) — ROS 와 완전히 분리
#   왜 따로 만드나:
#     ROS 2 Jazzy(apt)의 cv_bridge 등은 numpy 1.26 에 맞춰 빌드되어 있고, .venv-mobile 은 그 버전에 고정된다.
#     최신 JAX(GPU)·optax 는 numpy 2.x 를 요구하므로 같은 환경에 두면 한쪽이 깨진다.
#     → 시스템 site-packages 를 보지 않는 독립 venv 에 JAX·MuJoCo 를 설치한다 (numpy 는 pip 가 맞는 버전으로).
#   JAX 는 소스 빌드가 아니라 공식 wheel(CUDA 런타임 포함)을 설치한다. NVIDIA 드라이버만 있으면 된다.
#   사용: bash tron1_openarm/scripts/setup_venv_biped.sh           (11_train_policy.sh 가 없으면 자동 호출)
#         JAX_SPEC='jax[cuda12]' bash tron1_openarm/scripts/setup_venv_biped.sh   # JAX 변형 직접 지정
#   JAX_SPEC 를 안 주면 _gpu_select.sh 가 고른다 (GPU 가 여러 개면 compute capability 가 가장 높은 것 기준):
#     드라이버 ≥ 580 + compute capability ≥ 7.5 → jax[cuda13]
#     드라이버 ≥ 525                            → jax[cuda12]   (예: 드라이버 550 + RTX 3090)
#     NVIDIA GPU 없음 / 드라이버가 너무 오래됨     → jax[cpu]
#   MJX(GPU 물리, rl/train_mjx.py)용 mujoco-mjx 도 함께 설치한다.
set -euo pipefail
T1=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)     # tron1_openarm
VENV=$T1/.venv-biped
PYTHON=/usr/bin/python3

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 0
fi
(( $# == 0 )) || { echo "인자가 필요 없습니다. --help 참고" >&2; exit 2; }
[[ -x "$PYTHON" ]] || { echo "$PYTHON 가 없습니다" >&2; exit 1; }

# 1) JAX 변형 결정 (예전 11_train_policy.sh 의 RL_JAX 도 받아 줌)
JAX_SPEC=${JAX_SPEC:-${RL_JAX:-}}
# shellcheck source=_gpu_select.sh
source "$T1/scripts/_gpu_select.sh"
if [[ -z "$JAX_SPEC" ]]; then
  JAX_SPEC=$AUTO_JAX_SPEC
  echo "GPU: ${GPU_NAME:-없음} (compute capability ${GPU_CC:-–}, 드라이버 ${DRIVER_MAJOR}) → $JAX_SPEC"
fi
pin_gpu

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
"${PIP[@]}" install "$JAX_SPEC" optax 'mujoco==3.5.0' 'mujoco-mjx==3.5.0' onnx onnxruntime pyyaml scipy
# MuJoCo Warp (MJX 의 GPU 물리 커널, 12_train_mjx.sh 기본 백엔드). warp-lang 최신판은 CUDA 13 드라이버(≥ 580)가 필요하다.
# 드라이버 525~579 는 CUDA 12.9 로 빌드된 warp-lang 1.12.1 을 쓴다 (드라이버 550 + RTX 3090 에서 확인).
case "$JAX_SPEC" in
  *cuda13*) "${PIP[@]}" install 'mujoco-warp==3.5.0.2' ;;
  *cuda12*) "${PIP[@]}" install 'mujoco-warp==3.5.0.2' 'warp-lang==1.12.1' ;;
  *) echo "[알림] GPU 가 없어 MuJoCo Warp 는 설치하지 않습니다 (12_train_mjx.sh 는 GPU 필요)" ;;
esac

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
if "cuda" in os.environ["JAX_SPEC"]:
    try:
        import warp as wp
        wp.config.quiet = True
        wp.init()
        n = len(wp.get_cuda_devices())
        print("Warp   :", wp.config.version, "| CUDA 장치", n, "" if n else "← 0 이면 드라이버가 이 warp-lang 빌드보다 오래됨")
    except Exception as e:  # noqa: BLE001
        print("[경고] MuJoCo Warp 를 불러오지 못했습니다:", e)
PY

printf '\n준비 완료: %s\n' "$VENV"
echo "직접 쓸 때 (~/.bashrc 가 ROS 를 자동으로 불러오는 터미널이어도 됨):"
echo "  export PYTHONNOUSERSITE=1 && source $VENV/bin/activate"
echo "  env -u PYTHONPATH python ...     # ROS 의 PYTHONPATH(NumPy 1.26 경로)를 지우고 실행"
echo "학습 실행: bash tron1_openarm/scripts/11_train_policy.sh [분]   (CPU 물리)"
echo "          bash tron1_openarm/scripts/12_train_mjx.sh [분]      (GPU 물리, MJX)"
[[ -n "$GPU_UUID" ]] && echo "GPU 가 여러 대면 위 스크립트들이 CUDA_VISIBLE_DEVICES=$GPU_UUID ($GPU_NAME) 로 고정합니다"
