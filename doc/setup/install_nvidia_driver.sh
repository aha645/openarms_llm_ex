#!/usr/bin/env bash
# NVIDIA 드라이버 580 설치·고정 (Ubuntu 24.04 PC, 선택 단계 — 드라이버가 없거나 525 미만일 때만). Pascal(GTX 10xx)부터 Ampere(RTX 3090)까지 지원한다.
# 580 브랜치가 Pascal 을 지원하는 마지막 브랜치이며, 590 이상(open 커널 모듈 포함)은 GTX 10xx 를 지원하지 않으므로 580 으로 고정한다.
# DGX Spark 에서는 실행하지 않는다 (전용 커널·드라이버가 이미 맞춰져 있음).
#   사용: bash doc/setup/install_nvidia_driver.sh   (선택, 휠형·TRON1 공통. 끝나면 재부팅)
set -e
if [ -f /etc/dgx-release ]; then
  echo "DGX Spark 입니다. 이 스크립트를 실행하지 마세요 (doc/setup/dgx-spark.md 참고)."; exit 1
fi
if ! lspci | grep -iE "vga|3d" | grep -qi nvidia; then
  echo "NVIDIA GPU 가 보이지 않습니다. 이 PC 에는 NVIDIA 드라이버가 필요 없습니다 (Intel/AMD 그래픽으로 실습 가능)."; exit 0
fi
sudo apt update
sudo apt install -y build-essential dkms linux-headers-$(uname -r) mesa-utils
ubuntu-drivers devices || true
# '-open' 변형은 Turing(GTX 16xx) 이상 전용 → Pascal 은 proprietary(nvidia-driver-580) 사용
sudo apt install -y nvidia-driver-580
# 커널/드라이버 자동 업그레이드로 590+ 가 들어오지 않게 고정
sudo apt-mark hold nvidia-driver-580 || true
# 하이브리드(Optimus) 노트북: 시뮬레이션은 항상 dGPU 로 (배터리는 더 씀)
sudo prime-select nvidia || true
echo
echo "Secure Boot 가 켜져 있으면 설치 중 MOK 비밀번호를 정하고, 재부팅 시 파란 화면에서 'Enroll MOK' → 비밀번호 입력."
echo "지금 재부팅하세요(sudo reboot). 재부팅 후: nvidia-smi 와 'glxinfo -B | grep renderer' 가 내 NVIDIA GPU 이름을 보여야 합니다."
