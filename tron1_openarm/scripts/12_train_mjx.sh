#!/usr/bin/env bash
# 보행 정책 PPO — GPU 물리(MJX + MuJoCo Warp) 판. 11_train_policy.sh(CPU 물리)와 같은 알고리즘·정책 형식(npz)
#   사용: bash tron1_openarm/scripts/12_train_mjx.sh [분] [발 sf|pf] [추가 인자...]      기본 30분, sf
#         예) bash tron1_openarm/scripts/12_train_mjx.sh 60 sf --envs 8192
#   RTX 3090 (드라이버 550) 측정: 환경 4096 개 약 80,000 정책 스텝/s (CPU 32코어 rollout 의 약 4배)
#   학습용 발은 '발바닥 상자'(mjx_model.py). 학습 뒤 배포 조건(CPU, 메시 발, 2 ms)으로 자동 평가한다.
set -eo pipefail
T1=$(cd "$(dirname "$0")/.." && pwd)
MIN=${1:-30}
FOOT=${2:-sf}
shift 2 2>/dev/null || shift $#
VENV=$T1/.venv-biped
[ -x "$VENV/bin/python" ] || bash "$T1/scripts/setup_venv_biped.sh"
source "$T1/scripts/_gpu_select.sh"
pin_gpu
PY=(env -u PYTHONPATH PYTHONNOUSERSITE=1 XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}" "$VENV/bin/python")
cd "$T1/rl"
if [ "$("${PY[@]}" -c 'import jax; print(jax.default_backend())' 2>/dev/null | tail -n 1)" != "gpu" ]; then
  echo "[오류] JAX 가 GPU 를 쓰지 못합니다. GPU 가 없으면 11_train_policy.sh (CPU 물리)를 쓰세요." >&2
  exit 1
fi
RUN=$T1/runs/mjx_${FOOT}_$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN"
"${PY[@]}" train_mjx.py --foot "$FOOT" --minutes "$MIN" --out "$RUN" "$@" 2>&1 | grep -v "^Module .* load on device" | tee "$RUN.log"
# 배포 조건(실습 월드·메시 발·물리 2 ms)에서 시작 정책과 비교. 결과는 run 폴더에만 저장 (커밋된 eval_results.json 은 그대로)
BASE=$T1/src/tron1_openarm_sim/policies/$([ "$FOOT" = sf ] && echo tron1sf_openarm_policy.npz || echo tron1_openarm_policy.npz)
CUDA_VISIBLE_DEVICES= "${PY[@]}" eval_policy.py "$BASE" "$RUN/best.npz" "$RUN/final.npz" --out="$RUN/eval.json"
echo "평가 결과: $RUN/eval.json  (위 표: 첫 블록 = 지금 배포 정책, 다음 = 이번 학습)"
echo "좋으면 배포 정책 교체 (먼저 원본 백업): cp $RUN/best.npz $BASE"
