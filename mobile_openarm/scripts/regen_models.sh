#!/usr/bin/env bash
# (선택) 바퀴형 모델 다시 만들기: OpenArm 원본(enactic/openarm_mujoco, Apache-2.0)을 third_party 에 받아
#        메시 복사 → MJCF → 월드·locations.yaml → URDF → 지도 → SRDF 를 새로 생성한다.
#   사용: bash mobile_openarm/scripts/regen_models.sh
#   언제: OpenArm 원본을 새 버전으로 바꾸거나, 생성 도구(src/mobile_openarm_sim/tools/*.py)를 고쳤을 때만.
#         평소 설치(mobile_openarm/scripts/setup_workspace.sh)는 저장소에 들어 있는 모델·메시를 그대로 쓰므로 이 스크립트가 필요 없다.
#   주의: 생성 파일(mjcf/, urdf/, config/locations.yaml, maps/, SRDF)을 덮어쓴다. 직접 고친 내용은 생성 도구 쪽에 반영할 것.
#         끝나면 git diff 로 바뀐 내용을 확인하고, 검증 후 커밋한다.
set -e
WS=$(cd "$(dirname "$0")/.." && pwd)
SIM=$WS/src/mobile_openarm_sim
PY=$WS/.venv-mobile/bin/python
[ -x "$PY" ] || { echo "먼저 bash mobile_openarm/scripts/setup_workspace.sh 로 .venv-mobile 을 만드세요"; exit 1; }
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
OA_REF=${OPENARM_MUJOCO_REF:-}   # 특정 커밋/태그로 받으려면 OPENARM_MUJOCO_REF=<커밋>
mkdir -p "$WS/third_party"
touch "$WS/third_party/COLCON_IGNORE"   # third_party 는 ROS 패키지가 아님
if [ ! -d "$WS/third_party/openarm_mujoco" ]; then
  git clone https://github.com/enactic/openarm_mujoco "$WS/third_party/openarm_mujoco"
fi
[ -n "$OA_REF" ] && git -C "$WS/third_party/openarm_mujoco" checkout -q "$OA_REF"
echo "openarm_mujoco: $(git -C "$WS/third_party/openarm_mujoco" log -1 --format='%H %cs')"
OA=$WS/third_party/openarm_mujoco/v1/openarm_bimanual.xml
rm -rf "$SIM/mjcf/meshes"
cp -r "$WS/third_party/openarm_mujoco/v1/meshes" "$SIM/mjcf/meshes"
cd "$SIM"
"$PY" tools/build_mjcf.py --src "$OA" --out mjcf/mobile_openarm.xml
"$PY" tools/build_mjcf.py --src "$OA" --out mjcf/mobile_openarm_light.xml --light
"$PY" tools/wrist_cameras.py mjcf/mobile_openarm.xml mjcf/mobile_openarm_light.xml   # 양손 손목 카메라
"$PY" tools/build_world.py --out mjcf --config config
"$PY" tools/mjcf_to_urdf.py --mjcf mjcf/mobile_openarm.xml --out urdf/mobile_openarm.urdf
"$PY" tools/make_map.py --world mjcf/world.xml --out maps/world_map
"$PY" "$WS/src/mobile_openarm_moveit_config/tools/make_srdf.py" --mjcf mjcf/mobile_openarm.xml \
        --out "$WS/src/mobile_openarm_moveit_config/config/mobile_openarm.srdf"
echo "모델 재생성 완료. git diff 로 바뀐 내용을 확인한 뒤 mobile_openarm/scripts/setup_workspace.sh 의 빌드(또는 colcon build)를 다시 하세요."
echo "메시 출처를 바꿨다면 src/mobile_openarm_sim/mjcf/MESHES_NOTICE.md 의 커밋도 고치세요."
