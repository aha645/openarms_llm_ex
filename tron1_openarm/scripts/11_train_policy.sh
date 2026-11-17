#!/usr/bin/env bash
# 보행 정책 PPO 미세조정 (선택) — 팔 자세가 계속 바뀌는 상태에서 TRON1+OpenArm 보행을 학습
#   ROS 와 분리된 가상환경 tron1_openarm/.venv-biped 에서 실행
#   (JAX 는 numpy 2.x 가 필요해 ROS 의 numpy 1.26 과 충돌 → setup_venv_biped.sh 참고)
#   사용: bash tron1_openarm/scripts/11_train_policy.sh [분]      기본 90분
#   신경망(PPO 업데이트)은 JAX GPU, MuJoCo 물리 rollout 은 CPU 스레드에서 돈다.
#   GPU 종류 지정: JAX_SPEC='jax[cuda12]' bash ...   (venv 를 처음 만들 때만 적용)
set -eo pipefail
T1=$(cd "$(dirname "$0")/.." && pwd)
MIN=${1:-90}
VENV=$T1/.venv-biped
[ -x "$VENV/bin/python" ] || bash "$T1/scripts/setup_venv_biped.sh"
# GPU 가 여러 대면 가장 좋은 GPU 하나만 보이게 (JAX 가 PCI 순서상 첫 GPU 를 쓰지 않도록)
source "$T1/scripts/_gpu_select.sh"
pin_gpu
# ROS 환경(PYTHONPATH)·~/.local 이 섞이지 않게 실행.
# XLA_PYTHON_CLIENT_PREALLOCATE=false: JAX 가 GPU 메모리 75% 를 미리 잡지 않게
#   (DGX Spark 처럼 CPU·GPU 가 메모리를 공유하는 장비에서 MuJoCo rollout 메모리가 부족해지는 것 방지)
PY=(env -u PYTHONPATH PYTHONNOUSERSITE=1 XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}" "$VENV/bin/python")
cd "$T1/rl"
BACKEND=$("${PY[@]}" -c 'import jax; print(jax.default_backend())')
"${PY[@]}" -c 'import jax; print("JAX backend:", jax.default_backend(), jax.devices())'
if [ "$BACKEND" != "gpu" ]; then
  echo "[경고] JAX 가 GPU 를 쓰지 못합니다(backend=$BACKEND). CPU 학습은 매우 느려 ${MIN}분으로는 의미 있는 결과가 나오지 않습니다."
  echo "       GPU PC(DGX Spark 등)에서 실행하는 것을 권장합니다. 계속하려면 기다리고, 그만두려면 5초 안에 Ctrl+C."
  sleep 5
fi
RUN=$T1/runs/$(date +%Y%m%d_%H%M%S)
"${PY[@]}" train_ppo.py --minutes "$MIN" --init-std 0.08 --lr 2e-5 --critic-warmup 20 --out "$RUN" | tee "$RUN.log"
# 주의: eval_policy.py 는 저장소에 커밋된 기록 policies/eval_results.json 을 덮어쓴다
"${PY[@]}" eval_policy.py "$T1/policies/limx_pf_tron1a_pretrained.npz" "$RUN/best.npz"
echo "평가 결과를 $T1/policies/eval_results.json 에 덮어썼습니다 (git diff 로 확인, 되돌리려면 git checkout -- 그 파일)."
echo "결과가 좋으면 (PF 정책 교체 — SF 교육의 tron1sf_openarm_policy.npz 에는 영향 없음, 먼저 원본 백업):"
echo "  cp $RUN/best.npz $T1/src/tron1_openarm_sim/policies/tron1_openarm_policy.npz"
echo "ONNX(LimX 배포 형식): cd $T1/rl && env -u PYTHONPATH $VENV/bin/python export_onnx.py $RUN/best.npz $T1/policies/onnx"
