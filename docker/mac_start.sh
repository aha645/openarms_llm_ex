#!/usr/bin/env bash
# Mac 에서 실행: 이미지 빌드 → 컨테이너 시작 → 워크스페이스 준비 → 브라우저로 화면 열기
set -e
cd "$(dirname "$0")"
docker info >/dev/null 2>&1 || { echo "Docker Desktop 이 실행 중이 아닙니다. 응용 프로그램에서 Docker 를 먼저 켜세요."; exit 1; }
docker compose build
docker compose up -d
if ! docker compose exec -T openarm test -f /root/openarm_mobile_ws/mobile_openarm/install/setup.bash; then
  echo "== 처음 실행: 가상환경(.venv-mobile)·OpenArm 에셋·모델 생성·colcon build (10~20분) =="
  docker compose exec -T openarm bash -lc "bash mobile_openarm/scripts/04_setup_workspace.sh"
fi
open "http://localhost:6080/vnc.html?autoconnect=1&resize=scale"
echo "브라우저 비밀번호: openarm (docker-compose.yml 의 VNC_PASSWORD)"
