#!/usr/bin/env bash
# [선택] CUDA / PyTorch — 본 교육과정(MuJoCo·Nav2·MoveIt·OpenAI API)은 CUDA 가 필요 없다.
# 로컬 학습/추론 실습을 따로 할 때만 설치.  GTX 1050 = compute capability 6.1 (Pascal)
#   - CUDA 13.x : Pascal 미지원 → 설치 금지
#   - CUDA 12.6 : 마지막으로 무리 없이 쓰는 선택 (드라이버 580 과 호환)
#   - PyTorch   : cu126 휠만 sm_61 커널 포함, 그것도 2.14 가 마지막 (2.15 부터 Pascal 휠 없음)
set -e
VENV=${VENV:-$HOME/venvs/torch}
python3 -m venv "$VENV"
source "$VENV/bin/activate"
pip install --upgrade pip
pip install torch==2.14.0 torchvision==0.29.0 --index-url https://download.pytorch.org/whl/cu126
python - <<'PY'
import torch
print("torch", torch.__version__, "cuda", torch.version.cuda, "available", torch.cuda.is_available())
if torch.cuda.is_available():
    print(torch.cuda.get_device_name(0), torch.cuda.get_device_capability(0), torch.cuda.get_arch_list())
    x = torch.randn(1024, 1024, device="cuda"); print("matmul ok", (x @ x).sum().item())
PY
# PyTorch 휠에는 CUDA 런타임이 들어 있어 별도 CUDA Toolkit 이 필요 없다.
# nvcc(커스텀 CUDA 확장 빌드)가 꼭 필요할 때만 CUDA Toolkit 12.6 을 설치:
#   wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
#   sudo dpkg -i cuda-keyring_1.1-1_all.deb && sudo apt update
#   sudo apt install -y cuda-toolkit-12-6        # (cuda-drivers 패키지는 설치하지 말 것: 드라이버를 590+ 로 올림)
