#!/usr/bin/env bash
# mobile_openarm 워크스페이스: .venv-mobile → OpenArm 에셋 받기 → MJCF/URDF/SRDF 생성 → colcon build
#   사용: bash mobile_openarm/scripts/04_setup_workspace.sh
#   워크스페이스 루트 = 이 스크립트의 상위 폴더(mobile_openarm/). 실행 위치와 무관.
set -e
WS=$(cd "$(dirname "$0")/.." && pwd)
SIM=$WS/src/mobile_openarm_sim
VENV=$WS/.venv-mobile
PY=$VENV/bin/python
source /opt/ros/jazzy/setup.bash

# 1) Python 가상환경(.venv-mobile): mujoco·openai·pydantic v2·flask
#    numpy 는 ROS apt 버전 그대로 고정해야 cv_bridge 등이 안 깨짐 (setup_venv_mobile.sh 참고)
bash "$WS/scripts/setup_venv_mobile.sh"
export PYTHONNOUSERSITE=1      # ~/.local 의 예전 pip --user 설치가 섞이지 않게

# 2) OpenArm MuJoCo 에셋 (enactic/openarm_mujoco, Apache-2.0) — v1 메시 48MB
mkdir -p "$WS/third_party"
if [ ! -d "$WS/third_party/openarm_mujoco" ]; then
  git clone --depth 1 https://github.com/enactic/openarm_mujoco "$WS/third_party/openarm_mujoco"
fi
# third_party 는 ROS 패키지가 아님 → colcon 이 openarm_mujoco/setup.py 를 패키지로 읽지 않도록 제외
touch "$WS/third_party/COLCON_IGNORE"
rm -rf "$SIM/mjcf/meshes"
cp -r "$WS/third_party/openarm_mujoco/v1/meshes" "$SIM/mjcf/meshes"

# 3) 모델 생성 (MJCF → URDF → SRDF, 지도, Nav2 파라미터) — venv 의 Python(mujoco 포함)으로
cd "$SIM"
"$PY" tools/build_mjcf.py --src "$WS/third_party/openarm_mujoco/v1/openarm_bimanual.xml" --out mjcf/mobile_openarm.xml
"$PY" tools/build_mjcf.py --src "$WS/third_party/openarm_mujoco/v1/openarm_bimanual.xml" --out mjcf/mobile_openarm_light.xml --light
"$PY" tools/build_world.py --out mjcf --config config
"$PY" tools/mjcf_to_urdf.py --mjcf mjcf/mobile_openarm.xml --out urdf/mobile_openarm.urdf
"$PY" tools/make_map.py --world mjcf/world.xml --out maps/world_map
"$PY" "$WS/src/mobile_openarm_moveit_config/tools/make_srdf.py" --mjcf mjcf/mobile_openarm.xml \
        --out "$WS/src/mobile_openarm_moveit_config/config/mobile_openarm.srdf"

# 4) 시뮬레이션 단독 점검 (ROS 불필요)
MUJOCO_GL=egl "$PY" tools/selftest.py || MUJOCO_GL=osmesa "$PY" tools/selftest.py || true

# 5) 빌드 — venv 의 Python 으로 colcon 을 실행해야 ros2 run 실행 파일 첫 줄(shebang)이 venv Python 이 된다
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y || true
"$PY" -m colcon build --symlink-install --base-paths src
echo "실행 파일 Python: $(head -n 1 "$WS/install/mobile_openarm_apps/lib/mobile_openarm_apps/task_manager")"
grep -qF "source $WS/install/setup.bash" ~/.bashrc || echo "source $WS/install/setup.bash" >> ~/.bashrc
# 폴더 재구성 전(저장소 루트 = 워크스페이스) 경로가 ~/.bashrc 에 남아 있으면 알림
OLD=$(dirname "$WS")/install/setup.bash
grep -qF "source $OLD" ~/.bashrc && echo "[알림] ~/.bashrc 의 예전 줄 'source $OLD' 는 더 이상 없는 경로입니다. 지워 주세요."
echo "빌드 완료: source $WS/install/setup.bash"
echo "python3 를 직접 쓰는 명령(selftest·mujoco.viewer 등) 전에는: source $VENV/bin/activate"
