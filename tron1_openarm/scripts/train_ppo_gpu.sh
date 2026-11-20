#!/usr/bin/env bash
# 보행 정책 PPO — GPU 판 (물리: MJX + MuJoCo Warp, 신경망: JAX GPU). train_ppo_cpu.sh 와 같은 알고리즘·보상·사용법
#   화면이 있으면 학습 그래프·동작 창을 같이 띄운다 (VIEW=0 으로 끔). 중간 정책 저장 간격 CK_MIN 분 (기본 2)
#   사용: bash tron1_openarm/scripts/train_ppo_gpu.sh [분] [발 sf|pf] [추가 인자...]      기본 30분, sf
#         예) bash tron1_openarm/scripts/train_ppo_gpu.sh 60 sf --envs 8192
#   RTX 3090 (드라이버 550) 측정: 환경 4096 개 약 80,000 정책 스텝/s (CPU 32코어 rollout 의 약 4배)
#   보상·교란은 CPU 판과 같은 train_config.py (SF 기본값 = 배포 중인 SF v3 정책을 만든 설정, PF 에는 적용 안 함).
#   학습 모델은 발만 바닥과 닿는 단순화 모델(mjx_model.py). 학습 뒤 배포 조건(CPU, 메시 발, 2 ms)으로 자동 평가한다.
set -eo pipefail
T1=$(cd "$(dirname "$0")/.." && pwd)
MIN=${1:-30}
FOOT=${2:-sf}
shift 2 2>/dev/null || shift $#
VENV=$T1/.venv-biped-train          # GPU 학습용 (시뮬·미션용 .venv-biped 와 별개)
[ -x "$VENV/bin/python" ] || bash "$T1/scripts/setup_venv_biped_train_ppo.sh"
source "$T1/scripts/_gpu_select.sh"
pin_gpu
PY=(env -u PYTHONPATH PYTHONNOUSERSITE=1 XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}" "$VENV/bin/python")
cd "$T1/rl"
if [ "$("${PY[@]}" -c 'import jax; print(jax.default_backend())' 2>/dev/null | tail -n 1)" != "gpu" ]; then
  echo "[오류] JAX 가 GPU 를 쓰지 못합니다. GPU 가 없으면 train_ppo_cpu.sh (CPU 판)를 쓰세요." >&2
  exit 1
fi
RUN=$T1/runs/gpu_${FOOT}_$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN"
# 실제 적용한 환경 변수를 보존 (이전 실험의 export 가 남아 있는지도 확인 가능).
env | LC_ALL=C sort | grep '^TRON1_' > "$RUN/training_env.txt" || true
# 학습 보기 (화면이 있으면 자동, VIEW=0 으로 끔): 그래프 창 + 중간 정책 동작 창 (rl/watch_training.py, 시뮬용 .venv-biped 의 Python)
# 그래프는 log.json(25 반복마다), 동작 창은 중간 정책(ckNNNmin.npz, CK_MIN 분마다)이 새로 저장될 때마다 바뀐다. 학습이 끝나면 함께 닫는다
VIEWERS=()
if [ -n "${DISPLAY:-}" ] && [ "${VIEW:-1}" != 0 ] && [ -x "$T1/.venv-biped/bin/python" ]; then
  VPY=(env -u PYTHONPATH PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= MUJOCO_GL=glfw "$T1/.venv-biped/bin/python" "$T1/rl/watch_training.py")
  "${VPY[@]}" plot "$RUN" > "$RUN/watch_plot.log" 2>&1 & VIEWERS+=($!)
  "${VPY[@]}" play "$RUN" > "$RUN/watch_play.log" 2>&1 & VIEWERS+=($!)
  trap 'kill "${VIEWERS[@]}" 2>/dev/null || true' EXIT
  echo "학습 보기: 그래프·동작 창 (DISPLAY=$DISPLAY, 끄려면 VIEW=0)"
fi
"${PY[@]}" train_ppo_gpu.py --foot "$FOOT" --minutes "$MIN" --out "$RUN" --ck-min "${CK_MIN:-2}" "$@" 2>&1 | grep -v "^Module .* load on device" | tee "$RUN.log"
# 배포 조건(실습 월드·메시 발·물리 2 ms)에서 시작 정책과 비교. 결과는 run 폴더에만 저장 (커밋된 eval_results.json 은 그대로)
BASE=$T1/src/tron1_openarm_sim/deployed_policies/$([ "$FOOT" = sf ] && echo tron1sf_openarm_policy.npz || echo tron1_openarm_policy.npz)
CUDA_VISIBLE_DEVICES= "${PY[@]}" eval_policy.py "$BASE" "$RUN/best.npz" "$RUN/final.npz" --out="$RUN/eval.json"
echo "평가 결과: $RUN/eval.json  (위 표: 첫 블록 = 지금 배포 정책, 다음 = 이번 학습)"
echo "좋으면 배포 정책 교체 (먼저 원본 백업): cp $RUN/best.npz $BASE"
