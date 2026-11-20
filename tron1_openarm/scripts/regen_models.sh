#!/usr/bin/env bash
# (선택) TRON1 + OpenArm 모델 다시 만들기: LimX TRON1 로봇 설명(limxdynamics/tron1-robot-description, Apache-2.0)과
#        OpenArm 원본(enactic/openarm_mujoco, Apache-2.0)을 third_party 에 받아 MJCF·URDF·SRDF·월드·Nav2 파라미터를 새로 생성한다.
#   사용: bash tron1_openarm/scripts/regen_models.sh
#   언제: LimX·OpenArm 원본을 새 버전으로 바꾸거나, 생성 도구(src/tron1_openarm_sim/tools/*.py)를 고쳤을 때만.
#         평소 설치(setup_workspace.sh)는 저장소에 들어 있는 모델·메시를 그대로 쓰므로 이 스크립트가 필요 없다.
#   주의: 생성 파일(mjcf/, urdf/, SRDF 2개, config/nav2_params_tron1.yaml)을 덮어쓴다. 끝나면 git diff 로 확인하고,
#         집기 시험(tools/validation/pick_test.sh)·전체 미션으로 검증한 뒤 커밋한다.
set -e
T1=$(cd "$(dirname "$0")/.." && pwd)
SIM=$T1/src/tron1_openarm_sim
MV=$T1/src/tron1_openarm_moveit_config/config
PY=$T1/.venv-biped/bin/python
[ -x "$PY" ] || bash "$T1/scripts/setup_venv_biped.sh"
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
# xacro (LimX 설명을 URDF 로 펼침)
if ! ros2 pkg prefix xacro >/dev/null 2>&1; then
  if [ "$(id -u)" -eq 0 ]; then APT=(apt-get); else APT=(sudo apt-get); fi
  "${APT[@]}" install -y ros-jazzy-xacro liburdfdom-tools
fi
# 원본 받기 (특정 커밋: TRON1_DESC_REF=<커밋>, OPENARM_MUJOCO_REF=<커밋>)
mkdir -p "$T1/third_party"
touch "$T1/third_party/COLCON_IGNORE"
[ -d "$T1/third_party/tron1-robot-description" ] || git clone https://github.com/limxdynamics/tron1-robot-description.git "$T1/third_party/tron1-robot-description"
[ -d "$T1/third_party/openarm_mujoco" ] || git clone https://github.com/enactic/openarm_mujoco "$T1/third_party/openarm_mujoco"
[ -n "${TRON1_DESC_REF:-}" ] && git -C "$T1/third_party/tron1-robot-description" checkout -q "$TRON1_DESC_REF"
[ -n "${OPENARM_MUJOCO_REF:-}" ] && git -C "$T1/third_party/openarm_mujoco" checkout -q "$OPENARM_MUJOCO_REF"
echo "tron1-robot-description: $(git -C "$T1/third_party/tron1-robot-description" log -1 --format='%H %cs')"
echo "openarm_mujoco:          $(git -C "$T1/third_party/openarm_mujoco" log -1 --format='%H %cs')"
D=$T1/third_party/tron1-robot-description/pointfoot
OA=$T1/third_party/openarm_mujoco/v1/openarm_bimanual.xml
SRDF_TOOL=$T1/src/tron1_openarm_moveit_config/tools/make_srdf.py
cd "$SIM"
# Point-foot
"$PY" tools/build_tron1_openarm.py --tron1 "$D/PF_TRON1A" --openarm "$OA" --out mjcf
"$PY" tools/build_world_tron1.py --out mjcf --config config
"$PY" tools/make_urdf_tron1.py --mjcf mjcf/tron1_openarm.xml --out urdf/tron1_openarm.urdf
"$PY" "$SRDF_TOOL" --mjcf mjcf/tron1_openarm.xml --out "$MV/tron1_openarm.srdf" --samples 2000
"$PY" tools/patch_srdf_home.py "$MV/tron1_openarm.srdf"
# Sole-foot (교육 기준). SF 월드는 PF 와 같은 config 를 쓰므로 여기서 생기는 config 는 버린다
"$PY" tools/build_tron1_openarm.py --foot sf --base-height 0.73 --tron1 "$D/SF_TRON1A" --openarm "$OA" --out mjcf/sf
TMPCFG=$(mktemp -d)
"$PY" tools/build_world_tron1.py --out mjcf/sf --config "$TMPCFG"
rm -rf "$TMPCFG"
"$PY" tools/make_urdf_tron1.py --mjcf mjcf/sf/tron1_openarm.xml --out urdf/tron1_openarm_sf.urdf \
        --base-height 0.73 --mesh-prefix package://tron1_openarm_sim/mjcf/sf/
"$PY" "$SRDF_TOOL" --mjcf mjcf/sf/tron1_openarm.xml --out "$MV/tron1_openarm_sf.srdf" --samples 2000
"$PY" tools/patch_srdf_home.py "$MV/tron1_openarm_sf.srdf"
"$PY" tools/make_nav2_params_tron1.py --src tools/nav2_params_base.yaml --out config/nav2_params_tron1.yaml
echo "모델 재생성 완료. git diff 로 확인한 뒤 setup_workspace.sh(자체 점검·빌드)를 다시 실행하세요."
echo "원본 버전을 바꿨다면 src/tron1_openarm_sim/mjcf/MESHES_NOTICE.md 의 커밋도 고치세요."
