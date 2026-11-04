#!/usr/bin/env bash
# 워크스페이스: OpenArm 에셋 받기 → MJCF/URDF/SRDF 생성 → Python 의존성 → colcon build
#   사용: bash 04_setup_workspace.sh   (워크스페이스 루트 = 이 스크립트의 상위 폴더)
set -e
WS=$(cd "$(dirname "$0")/.." && pwd)
SIM=$WS/src/mobile_openarm_sim
source /opt/ros/jazzy/setup.bash

# 1) OpenArm MuJoCo 에셋 (enactic/openarm_mujoco, Apache-2.0) — v1 메시 48MB
mkdir -p "$WS/third_party"
if [ ! -d "$WS/third_party/openarm_mujoco" ]; then
  git clone --depth 1 https://github.com/enactic/openarm_mujoco "$WS/third_party/openarm_mujoco"
fi
# third_party 는 ROS 패키지가 아님 → colcon 이 openarm_mujoco/setup.py 를 패키지로 읽지 않도록 제외
touch "$WS/third_party/COLCON_IGNORE"
rm -rf "$SIM/mjcf/meshes"
cp -r "$WS/third_party/openarm_mujoco/v1/meshes" "$SIM/mjcf/meshes"

# 2) Python 의존성: mujoco 는 pip, numpy 는 ROS apt 버전(1.26) 그대로 유지해야 cv_bridge 등이 안 깨짐
#    (--break-system-packages 대신 --user + 제약파일로 numpy 업그레이드를 막는다)
# numpy 는 apt 버전과 같은 버전으로 고정. --ignore-installed: apt 패키지(typing_extensions 등) 제거 시도로 실패하는 것 방지
echo "numpy==$(python3 -c 'import numpy; print(numpy.__version__)')" > /tmp/ros-constraints.txt
if ! python3 -c "import mujoco, openai, pydantic; assert int(pydantic.VERSION[0]) >= 2" 2>/dev/null; then
  # pydantic: Ubuntu 24.04 apt 는 v1(1.10) → OpenAI Structured Outputs 용 v2 를 사용자 영역에 설치
  python3 -m pip install --user --break-system-packages --ignore-installed -c /tmp/ros-constraints.txt \
      "mujoco==3.5.0" "openai>=1.60" "pydantic>=2.5"
fi
python3 -c "import numpy, mujoco, cv2; print('numpy', numpy.__version__, 'mujoco', mujoco.__version__, 'cv2', cv2.__version__)"

# 3) 모델 생성 (MJCF → URDF → SRDF, 지도, Nav2 파라미터)
cd "$SIM"
python3 tools/build_mjcf.py --src "$WS/third_party/openarm_mujoco/v1/openarm_bimanual.xml" --out mjcf/mobile_openarm.xml
python3 tools/build_mjcf.py --src "$WS/third_party/openarm_mujoco/v1/openarm_bimanual.xml" --out mjcf/mobile_openarm_light.xml --light
python3 tools/build_world.py --out mjcf --config config
python3 tools/mjcf_to_urdf.py --mjcf mjcf/mobile_openarm.xml --out urdf/mobile_openarm.urdf
python3 tools/make_map.py --world mjcf/world.xml --out maps/world_map
python3 "$WS/src/mobile_openarm_moveit_config/tools/make_srdf.py" --mjcf mjcf/mobile_openarm.xml \
        --out "$WS/src/mobile_openarm_moveit_config/config/mobile_openarm.srdf"

# 4) 시뮬레이션 단독 점검 (ROS 불필요)
MUJOCO_GL=egl python3 tools/selftest.py || MUJOCO_GL=osmesa python3 tools/selftest.py || true

# 5) 빌드
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --symlink-install --base-paths src
grep -q "$WS/install/setup.bash" ~/.bashrc || echo "source $WS/install/setup.bash" >> ~/.bashrc
echo "빌드 완료: source $WS/install/setup.bash"
