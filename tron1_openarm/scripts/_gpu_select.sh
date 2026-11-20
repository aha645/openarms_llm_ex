# shellcheck shell=bash
# GPU 고르기 (setup_venv_biped_train_ppo.sh · train_ppo_cpu.sh · train_ppo_gpu.sh 가 source 해서 씀)
#
# 정하는 값
#   GPU_UUID / GPU_NAME / GPU_CC  : compute capability 가 가장 높은 NVIDIA GPU (같으면 메모리 큰 쪽)
#   DRIVER_MAJOR                  : NVIDIA 드라이버 주 버전 (예: 550)
#   AUTO_JAX_SPEC                 : 이 PC 에 맞는 JAX 변형
#       CUDA 13 wheel 은 드라이버 ≥ 580 + compute capability ≥ 7.5 가 필요하다
#       CUDA 12 wheel 은 드라이버 ≥ 525
#       (예전에는 첫 번째 GPU 의 compute capability 만 봐서, 드라이버 550 + RTX 3090 PC 에 cuda13 을 깔아 실패할 수 있었다)
# 함수
#   pin_gpu : CUDA_VISIBLE_DEVICES 가 비어 있으면 위 GPU 하나만 보이게 고정
#             (GPU 가 여러 개면 JAX 는 모두 잡고 0 번(=PCI 순서상 첫 GPU, 예: 구형 GTX)에 계산을 올린다)

GPU_UUID="" GPU_NAME="" GPU_CC="" DRIVER_MAJOR=0 AUTO_JAX_SPEC='jax[cpu]'
if command -v nvidia-smi >/dev/null 2>&1; then
  _best=$(nvidia-smi --query-gpu=compute_cap,memory.total,uuid,name --format=csv,noheader,nounits 2>/dev/null \
          | sort -t, -k1,1gr -k2,2nr | head -n 1 || true)
  if [[ -n "$_best" ]]; then
    GPU_CC=$(echo "$_best" | cut -d, -f1 | tr -d ' ')
    GPU_UUID=$(echo "$_best" | cut -d, -f3 | tr -d ' ')
    GPU_NAME=$(echo "$_best" | cut -d, -f4 | sed 's/^ *//')
    DRIVER_MAJOR=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -n 1 | cut -d. -f1 | tr -d ' ')
    if (( DRIVER_MAJOR >= 580 )) && awk -v c="$GPU_CC" 'BEGIN { exit !(c >= 7.5) }'; then
      AUTO_JAX_SPEC='jax[cuda13]'
    elif (( DRIVER_MAJOR >= 525 )); then
      AUTO_JAX_SPEC='jax[cuda12]'
    else
      echo "[알림] NVIDIA 드라이버 $DRIVER_MAJOR 는 JAX CUDA wheel 에 너무 오래되었습니다 (≥ 525 필요) → CPU 용 JAX" >&2
    fi
  fi
fi

pin_gpu() {
  if [[ -n "$GPU_UUID" && -z "${CUDA_VISIBLE_DEVICES:-}" ]]; then
    export CUDA_VISIBLE_DEVICES="$GPU_UUID"
    echo "GPU 고정: $GPU_NAME (compute capability $GPU_CC, 드라이버 $DRIVER_MAJOR) — CUDA_VISIBLE_DEVICES=$GPU_UUID"
  fi
}
