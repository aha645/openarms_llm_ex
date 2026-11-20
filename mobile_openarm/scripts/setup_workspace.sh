#!/usr/bin/env bash
# mobile_openarm 워크스페이스: .venv-mobile → 자체 점검 → colcon build
#   사용: bash mobile_openarm/scripts/setup_workspace.sh
#   워크스페이스 루트 = 이 스크립트의 상위 폴더(mobile_openarm/). 실행 위치와 무관.
#   모델(MJCF·URDF·SRDF·지도·OpenArm 메시)은 저장소에 들어 있는 것을 그대로 쓴다 — 인터넷으로 원본을 받지 않고, 생성 파일을 덮어쓰지 않는다.
#   OpenArm 원본에서 모델을 다시 만들 때만: bash mobile_openarm/scripts/regen_models.sh
set -e
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy 가 없습니다. 먼저 설치하세요: 노트북은 doc/setup/install_ros2_jazzy.sh, DGX Spark 는 doc/setup/dgx-spark.md 3절"; exit 1
fi
WS=$(cd "$(dirname "$0")/.." && pwd)
SIM=$WS/src/mobile_openarm_sim
VENV=$WS/.venv-mobile
PY=$VENV/bin/python
source /opt/ros/jazzy/setup.bash

# 1) Python 가상환경(.venv-mobile): mujoco·openai·pydantic v2·flask
#    numpy 는 ROS apt 버전 그대로 고정해야 cv_bridge 등이 안 깨짐 (setup_venv_mobile.sh 참고)
bash "$WS/scripts/setup_venv_mobile.sh"
export PYTHONNOUSERSITE=1      # ~/.local 의 예전 pip --user 설치가 섞이지 않게

# 2) 저장소의 모델·메시 확인 (OpenArm 원본 enactic/openarm_mujoco, Apache-2.0 — 출처: src/mobile_openarm_sim/mjcf/MESHES_NOTICE.md)
for f in "$SIM/mjcf/mobile_openarm.xml" "$SIM/mjcf/world.xml" "$SIM/mjcf/meshes/visual" "$SIM/urdf/mobile_openarm.urdf" \
         "$WS/src/mobile_openarm_moveit_config/config/mobile_openarm.srdf"; do
  [ -e "$f" ] || { echo "모델 파일이 없습니다: $f — 저장소를 다시 받거나 bash mobile_openarm/scripts/regen_models.sh"; exit 1; }
done

# 3) 시뮬레이션 단독 점검 (ROS 불필요)
cd "$SIM"
MUJOCO_GL=egl "$PY" tools/selftest.py || MUJOCO_GL=osmesa "$PY" tools/selftest.py || true

# 4) 빌드 — venv 의 Python 으로 colcon 을 실행해야 ros2 run 실행 파일 첫 줄(shebang)이 venv Python 이 된다
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y || true
"$PY" -m colcon build --symlink-install --base-paths src
echo "실행 파일 Python: $(head -n 1 "$WS/install/mobile_openarm_apps/lib/mobile_openarm_apps/task_manager")"
grep -qF "source $WS/install/setup.bash" ~/.bashrc || echo "source $WS/install/setup.bash" >> ~/.bashrc
# 폴더 재구성 전(저장소 루트 = 워크스페이스) 경로가 ~/.bashrc 에 남아 있으면 알림
OLD=$(dirname "$WS")/install/setup.bash
grep -qF "source $OLD" ~/.bashrc && echo "[알림] ~/.bashrc 의 예전 줄 'source $OLD' 는 더 이상 없는 경로입니다. 지워 주세요."
echo "빌드 완료. 이 스크립트는 별도 셸에서 돌았으므로 지금 터미널에는 아직 적용되지 않았습니다."
echo "터미널마다 (ROS 2 + .venv-mobile + 휠형 install 을 한 번에):"
echo "  source $WS/scripts/env.sh"
