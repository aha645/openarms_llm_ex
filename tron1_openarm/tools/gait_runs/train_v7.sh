#!/bin/bash
# 보행 재학습 v7: 옆걸음·대각선 명령 따로, 발 들기 목표 속도별  (doc/tron1/locomotion-rl.md 14절)
# 시작 정책 runs/gait_v6/seed1/final.npz, 시드 1·2 를 차례로 60 분씩, 5 분마다 중간 정책(ckNNNmin.npz). 끝나면 배포 정책·이전 후보와 함께 평가.
#   bash tron1_openarm/tools/gait_runs/train_v7.sh           (GPU PC, setup_venv_biped_train_ppo.sh 의 .venv-biped-train·setup_workspace.sh 의 .venv-biped 필요)
# 학습 보기: python3 tron1_openarm/rl/watch_training.py plot|play tron1_openarm/runs/gait_v7
set -o pipefail
T1=$(cd "$(dirname "$0")/../.." && pwd)
cd $T1 && source scripts/_gpu_select.sh && pin_gpu
cd $T1/rl
export TRON1_HAND_F=25 TRON1_REACH_P=0.35 TRON1_STAND_FEET=5 TRON1_STAND_LOAD=3 TRON1_STAND_TWIST=5 \
       TRON1_STAND_TILT=4 TRON1_EP_S=60 TRON1_STAND_LONG_MAX_S=50 \
       TRON1_VY=0.35 TRON1_VY_P=0.6 TRON1_CLEAR=100 TRON1_CLEAR_ADAPT=1 TRON1_TRACK_VY=3 TRON1_WALK_HEADING=2 TRON1_STAND_LIFT=2 TRON1_STAND_PLANTED=1 TRON1_FD_SIDE=0.3 TRON1_CMD_MIX=v7
OUT=$T1/runs/gait_v7
mkdir -p $OUT
for s in 1 2; do
  echo "================ 시드 $s 학습 시작 $(date +%H:%M) ================"
  env -u PYTHONPATH PYTHONNOUSERSITE=1 XLA_PYTHON_CLIENT_PREALLOCATE=false ../.venv-biped-train/bin/python train_ppo_gpu.py \
    --foot sf --init $T1/runs/gait_v6/seed1/final.npz --init-std 0.15 --impl warp --envs 4096 --minutes 60 --seed $s --ck-min 5 --out $OUT/seed$s 2>&1 \
    | grep --line-buffered -v "Module .* load on device" | tee $OUT/seed$s.log
done
echo "================ 평가 $(date +%H:%M) ================"
CUDA_VISIBLE_DEVICES= $T1/.venv-biped/bin/python eval_policy.py \
  ../src/tron1_openarm_sim/deployed_policies/tron1sf_openarm_policy.npz $T1/runs/gait_v5/seed1/final.npz $T1/runs/gait_v6/seed1/final.npz $OUT/seed1/best.npz $OUT/seed1/final.npz \
  $OUT/seed2/best.npz $OUT/seed2/final.npz --out=$OUT/eval.json | tee $OUT/eval.log
echo "================ 학습 끝 $(date +%H:%M) ================" | tee $OUT/DONE
