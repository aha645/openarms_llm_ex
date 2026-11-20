#!/usr/bin/env bash
# ============================================================================
# [2단계] TRON1 워크스페이스 설치 (가상환경 → 점검 → 빌드)
#
#   전제: ROS 2 Jazzy 설치 완료 (doc/setup/ — Mac Docker·우분투 노트북·DGX Spark·우분투+GPU 공통)
#   tron1_openarm 폴더 하나로 끝난다 (휠형 mobile_openarm 과 코드를 나눠 쓰지 않는다). src/ 에 패키지 3개가 들어 있다:
#     tron1_openarm_sim            : MuJoCo 시뮬레이션(보행 정책)·ROS 2 브리지·카메라 서버·launch·모델
#     tron1_openarm_moveit_config  : MoveIt 2 설정(SRDF·컨트롤러)·SRDF 생성 도구
#     tron1_openarm_apps           : 로봇 스킬(집기·놓기·옆걸음)·작업 관리자·LLM 계획기·웹 대시보드
#
#   사용법:  bash tron1_openarm/scripts/setup_workspace.sh
#   끝나면 새 터미널마다:  source tron1_openarm/scripts/env.sh
# ============================================================================
set -e

T1=$(cd "$(dirname "$0")/.." && pwd)   # tron1_openarm 폴더
PY=$T1/.venv-biped/bin/python          # 1단계에서 만드는 가상환경의 python

# 0) ROS 가 설치돼 있는지
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "[오류] ROS 2 Jazzy 가 없습니다. 노트북은 bash doc/setup/install_ros2_jazzy.sh, 그 밖의 환경은 doc/setup/ 의 설치 문서를 따라 하세요."
  exit 1
fi

# 1) 가상환경 (.venv-biped)
bash "$T1/scripts/setup_venv_biped.sh"

# 2) ROS 환경 불러오기
#    다른 워크스페이스(install/)를 불러온 터미널이어도 섞이지 않게 ROS 기본 경로만 남긴 뒤 불러온다
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1              # ~/.local 에 pip 로 깔린 것이 섞이지 않게

# 3) 시뮬레이션 자체 점검 (ROS 없이): 서기·걷기·회전·팔 동작·LiDAR·카메라
#    보행 정책은 저장소에 들어 있다: src/tron1_openarm_sim/deployed_policies/tron1sf_openarm_policy.npz (Sole-foot, 기본)
echo "== 자체 점검 (Sole-foot)"
cd "$T1/src/tron1_openarm_sim"
MUJOCO_GL=egl "$PY" tools/selftest_tron1.py sf/world_light.xml deployed_policies/tron1sf_openarm_policy.npz || true

# 4) 의존 ROS 패키지 설치 + 빌드
#    가상환경의 python 으로 colcon 을 실행해야 ros2 run 실행 파일이 이 가상환경으로 돈다
echo "== 빌드"
cd "$T1"
rosdep install --from-paths src --ignore-src -r -y || true
"$PY" -m colcon build --symlink-install --base-paths src

echo
echo "== 완료. 터미널마다 (ROS 2 + .venv-biped + TRON1 install 을 한 번에):"
echo "   source $T1/scripts/env.sh"
echo "   ros2 launch tron1_openarm_sim sim.launch.py              # 시뮬만"
echo "   bash tron1_openarm/tools/validation/mission_test.sh sf _m1   # 전체 미션 시험"
