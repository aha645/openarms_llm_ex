# shellcheck shell=bash
# 휠형 터미널 환경 한 번에 불러오기: ROS 2 Jazzy + 가상환경(.venv-mobile) + 휠형 워크스페이스(install/)
#   사용 (새 터미널마다, 저장소 루트에서):  source mobile_openarm/scripts/env.sh
#   bash 로 실행하면 별도 셸에서 끝나 지금 터미널에는 적용되지 않으므로 반드시 source 로 부른다.
#   다른 워크스페이스·가상환경을 불러온 터미널이어도 그 경로를 지우고 휠형 것만 남긴다.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "이 파일은 source 로 불러야 지금 터미널에 적용됩니다:  source $0"
  exit 1
fi
_ENV_WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "[오류] ROS 2 Jazzy 가 없습니다. doc/setup/ 의 설치 문서를 먼저 따라 하세요."
  unset _ENV_WS; return 1
fi
# 앞서 불러온 가상환경·워크스페이스 경로 지우기
if [ -n "${VIRTUAL_ENV:-}" ] && type deactivate >/dev/null 2>&1; then deactivate; fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1          # ~/.local 에 pip 로 깔린 것이 섞이지 않게
if [ -f "$_ENV_WS/.venv-mobile/bin/activate" ]; then
  source "$_ENV_WS/.venv-mobile/bin/activate"
else
  echo "[알림] 가상환경 .venv-mobile 가 없습니다 → bash mobile_openarm/scripts/setup_workspace.sh"
fi
if [ -f "$_ENV_WS/install/setup.bash" ]; then
  source "$_ENV_WS/install/setup.bash"
  echo "휠형 환경: ROS 2 $ROS_DISTRO · python $(command -v python3) · $_ENV_WS/install"
else
  echo "[알림] 휠형 워크스페이스가 아직 빌드되지 않았습니다 → bash mobile_openarm/scripts/setup_workspace.sh"
fi
unset _ENV_WS
