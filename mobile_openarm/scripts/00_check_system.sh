#!/usr/bin/env bash
# 노트북 사양/상태 점검 (설치 전에 실행)
echo "== OS ==";      lsb_release -ds 2>/dev/null || cat /etc/os-release | head -2
echo "== 커널 ==";    uname -r
echo "== CPU ==";     lscpu | grep -E "Model name|^CPU\(s\)"
echo "== RAM ==";     free -h | head -2
echo "== 디스크 ==";  df -h / | tail -1; lsblk -d -o NAME,SIZE,ROTA,MODEL | sed 's/ROTA/HDD(1)/'
echo "== GPU ==";     lspci | grep -iE "vga|3d"
echo "== NVIDIA 드라이버 =="; (nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv 2>/dev/null) || echo "nvidia-smi 없음 (드라이버 미설치)"
echo "== 세션 ==";    echo "XDG_SESSION_TYPE=$XDG_SESSION_TYPE  (wayland 이면 RViz/MuJoCo 뷰어가 불안정 → 로그인 화면에서 'Ubuntu on Xorg' 권장)"
echo "== Secure Boot =="; (mokutil --sb-state 2>/dev/null) || echo "mokutil 없음"
echo "== PRIME ==";   (prime-select query 2>/dev/null) || echo "prime-select 없음 (드라이버 설치 후 생김)"
echo "== OpenGL ==";  (glxinfo -B 2>/dev/null | grep -E "OpenGL renderer|OpenGL version") || echo "sudo apt install mesa-utils 후 다시 확인"
