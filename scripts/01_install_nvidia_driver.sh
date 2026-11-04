#!/usr/bin/env bash
# GTX 1050 (Pascal) 용 NVIDIA 드라이버 — 580 브랜치가 Pascal 을 지원하는 마지막 브랜치.
# 590 이상(open 커널 모듈 포함)은 GTX 10xx 를 지원하지 않으므로 반드시 580 으로 고정한다.
set -e
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
echo "재부팅 후: nvidia-smi 와 'glxinfo -B | grep renderer' 가 GeForce GTX 1050 을 보여야 합니다."
