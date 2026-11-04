#!/usr/bin/env bash
# 보행 정책 PPO 미세조정 (선택) — ROS 와 분리된 가상환경에서 실행 (JAX 는 numpy 2.x 가 필요해 ROS 의 numpy 1.26 과 충돌)
#   사용: bash tron1_openarm/scripts/11_train_policy.sh [분]      기본 90분
#   GPU(NVIDIA) 가 있으면: RL_JAX="jax[cuda12]" bash ... (신경망만 GPU, MuJoCo 물리는 CPU)
set -e
T1=$(cd "$(dirname "$0")/.." && pwd)
MIN=${1:-90}
VENV=$HOME/tron1_rl_venv
if [ ! -x "$VENV/bin/python" ]; then
  python3 -m venv "$VENV"
  env -u PYTHONPATH "$VENV/bin/pip" install -q "${RL_JAX:-jax[cpu]}" optax "mujoco==3.5.0" onnx onnxruntime pyyaml scipy
fi
PY="env -u PYTHONPATH $VENV/bin/python"
cd "$T1/rl"
RUN=$T1/runs/$(date +%Y%m%d_%H%M%S)
$PY train_ppo.py --minutes "$MIN" --init-std 0.08 --lr 2e-5 --critic-warmup 20 --out "$RUN" | tee "$RUN.log"
$PY eval_policy.py "$T1/policies/limx_pf_tron1a_pretrained.npz" "$RUN/best.npz"
echo "결과가 좋으면: cp $RUN/best.npz $T1/src/tron1_openarm_sim/policies/tron1_openarm_policy.npz"
echo "ONNX(LimX 배포 형식): $PY export_onnx.py $RUN/best.npz $T1/policies/onnx"
