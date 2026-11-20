#!/usr/bin/env bash
# Mac 터미널에서 실행 (컨테이너 안이 아님): 이미지 빌드 → 컨테이너 시작 → 워크스페이스 준비 → 브라우저로 화면 열기
set -e
cd "$(dirname "$0")"
docker info >/dev/null 2>&1 || { echo "Docker Desktop 이 실행 중이 아닙니다. 응용 프로그램에서 Docker 를 먼저 켜세요."; exit 1; }
echo "== 이미지 빌드 (처음 1회 20~40분, 이후에는 캐시로 금방 끝남) =="
docker compose build
docker compose up -d
if ! docker compose exec -T openarm test -f /root/openarm_mobile_ws/mobile_openarm/install/setup.bash; then
  echo "== 처음 실행: 휠형 가상환경(.venv-mobile)·자체 점검·colcon build (10~20분) =="
  docker compose exec -T openarm bash -lc "apt-get update -qq && bash mobile_openarm/scripts/setup_workspace.sh"
fi
open "http://localhost:6080/vnc.html?autoconnect=1&resize=scale"
echo "VNC 비밀번호: 기본값 openarm (바꿨다면 docker-compose.yml 의 VNC_PASSWORD 값)"
