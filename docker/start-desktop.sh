#!/usr/bin/env bash
# 컨테이너 시작 시: 가상 디스플레이 :1 → 창 관리자 → VNC(5901) → noVNC 웹(6080) → CMD 실행
set -e
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1           # 컨테이너 재시작 시 남은 잠금 파일 정리

Xvfb :1 -screen 0 "${RESOLUTION:-1600x900x24}" -nolisten tcp +extension GLX &
for i in $(seq 1 50); do [ -e /tmp/.X11-unix/X1 ] && break; sleep 0.1; done

fluxbox >/tmp/fluxbox.log 2>&1 &
(sleep 1; xterm -geometry 120x35+20+20 -fa 'DejaVu Sans Mono' -fs 11 -e bash -l) &

mkdir -p /root/.vnc
x11vnc -storepasswd "${VNC_PASSWORD:-openarm}" /root/.vnc/passwd >/dev/null 2>&1
x11vnc -display :1 -rfbauth /root/.vnc/passwd -rfbport 5901 -forever -shared -noxdamage -quiet \
       >/tmp/x11vnc.log 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5901 >/tmp/novnc.log 2>&1 &

echo "[desktop] 브라우저: http://localhost:6080/vnc.html  (비밀번호: \$VNC_PASSWORD)"
echo "[desktop] macOS 화면 공유: vnc://localhost:5901"
exec "$@"
