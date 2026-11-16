#!/usr/bin/env bash
# TRON1(Point-foot/Sole-foot) + OpenArm 실습 환경 준비
#   전제: 바퀴형 워크스페이스 설치(mobile_openarm/scripts/04_setup_workspace.sh)를 먼저 끝냈을 것
#         — OpenArm 에셋·.venv-mobile(mujoco 등)·바퀴형 ROS 패키지를 재사용
#   사용: bash tron1_openarm/scripts/10_setup_tron1.sh
#   빌드: tron1_openarm/ 가 별도 colcon 워크스페이스이며, mobile_openarm/install 위에 겹쳐(overlay) 빌드한다
#   다시 실행하면 2)의 생성 파일(mjcf/, urdf/, SRDF 2개, config/nav2_params_tron1.yaml)을 원래대로 덮어쓴다.
#   직접 고친 생성 파일은 미리 백업하거나 생성 도구(tools/) 쪽을 고칠 것.
set -e
T1=$(cd "$(dirname "$0")/.." && pwd)            # tron1_openarm 워크스페이스
MW=$(dirname "$T1")/mobile_openarm              # 바퀴형 워크스페이스 (underlay)
SIM=$T1/src/tron1_openarm_sim
PY=$MW/.venv-mobile/bin/python                  # ROS 실행용 venv (TRON1 ROS 노드도 같이 씀)
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy 가 없습니다. 먼저 설치하세요: 노트북은 mobile_openarm/scripts/03_install_ros2_jazzy.sh, DGX Spark 는 doc/setup/dgx-spark.md 3절"; exit 1
fi
source /opt/ros/jazzy/setup.bash
if [ ! -x "$PY" ] || [ ! -f "$MW/install/setup.bash" ] || [ ! -f "$MW/third_party/openarm_mujoco/v1/openarm_bimanual.xml" ]; then
  echo "먼저 bash mobile_openarm/scripts/04_setup_workspace.sh 를 실행하세요"; exit 1
fi
# xacro·check_urdf: 03 스크립트·DGX 3절·Docker 이미지가 이미 설치한다. 없을 때만 설치하고, 실패하면 멈춘다
if ! ros2 pkg prefix xacro >/dev/null 2>&1 || ! command -v check_urdf >/dev/null 2>&1; then
  echo "== ros-jazzy-xacro / liburdfdom-tools 설치 =="
  if [ "$(id -u)" -eq 0 ]; then APT=(apt-get); else APT=(sudo apt-get); fi
  "${APT[@]}" install -y ros-jazzy-xacro liburdfdom-tools || {
    echo "설치 실패. 직접 설치한 뒤 다시 실행하세요: ${APT[*]} install -y ros-jazzy-xacro liburdfdom-tools"; exit 1; }
fi
source "$MW/install/setup.bash"                 # 바퀴형 패키지(mobile_openarm_sim·apps·moveit_config)를 underlay 로
export PYTHONNOUSERSITE=1

# 1) LimX TRON1 로봇 설명 (xacro/URDF/MJCF/메시, Apache-2.0)
mkdir -p "$T1/third_party"
touch "$T1/third_party/COLCON_IGNORE"
if [ ! -d "$T1/third_party/tron1-robot-description" ]; then
  git clone --depth 1 https://github.com/limxdynamics/tron1-robot-description.git "$T1/third_party/tron1-robot-description"
fi
# (선택) LimX 시뮬레이터·배포 코드 — 비교·참고용
[ -d "$T1/third_party/tron1-mujoco-sim" ] || git clone --depth 1 https://github.com/limxdynamics/tron1-mujoco-sim.git "$T1/third_party/tron1-mujoco-sim" || true

# 2) 모델 생성: xacro → URDF → MJCF + OpenArm 접목 → 월드 → ROS 용 URDF → SRDF → Nav2 파라미터
cd "$SIM"
"$PY" tools/build_tron1_openarm.py --tron1 "$T1/third_party/tron1-robot-description/pointfoot/PF_TRON1A" \
        --openarm "$MW/third_party/openarm_mujoco/v1/openarm_bimanual.xml" --out mjcf
"$PY" tools/build_world_tron1.py --wheeled-tools "$MW/src/mobile_openarm_sim/tools" --out mjcf --config config
"$PY" tools/make_urdf_tron1.py --wheeled-tools "$MW/src/mobile_openarm_sim/tools" --mjcf mjcf/tron1_openarm.xml --out urdf/tron1_openarm.urdf
"$PY" "$MW/src/mobile_openarm_moveit_config/tools/make_srdf.py" --mjcf mjcf/tron1_openarm.xml \
        --out "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm.srdf" --samples 2000
sed -i 's/<robot name="mobile_openarm">/<robot name="tron1_openarm">/' "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm.srdf"
"$PY" tools/patch_srdf_home.py "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm.srdf"   # 이족용 home (팔 벌림)
# Sole-foot(발목 있음) 변형: 같은 순서로 mjcf/sf, urdf/tron1_openarm_sf.urdf, tron1_openarm_sf.srdf
"$PY" tools/build_tron1_openarm.py --foot sf --base-height 0.73 --tron1 "$T1/third_party/tron1-robot-description/pointfoot/SF_TRON1A" \
        --openarm "$MW/third_party/openarm_mujoco/v1/openarm_bimanual.xml" --out mjcf/sf
# SF 월드는 PF 와 같은 config(config/locations.yaml 등)를 쓰므로, 여기서 생성되는 config 는 /tmp 에 버린다
"$PY" tools/build_world_tron1.py --wheeled-tools "$MW/src/mobile_openarm_sim/tools" --out mjcf/sf --config /tmp/tron1_sf_cfg
"$PY" tools/make_urdf_tron1.py --wheeled-tools "$MW/src/mobile_openarm_sim/tools" --mjcf mjcf/sf/tron1_openarm.xml --out urdf/tron1_openarm_sf.urdf \
        --base-height 0.73 --mesh-prefix package://tron1_openarm_sim/mjcf/sf/
"$PY" "$MW/src/mobile_openarm_moveit_config/tools/make_srdf.py" --mjcf mjcf/sf/tron1_openarm.xml \
        --out "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm_sf.srdf" --samples 2000
sed -i 's/<robot name="mobile_openarm">/<robot name="tron1_openarm">/' "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm_sf.srdf"
"$PY" tools/patch_srdf_home.py "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm_sf.srdf"
"$PY" tools/make_nav2_params_tron1.py --src "$MW/src/mobile_openarm_sim/config/nav2_params.yaml" --out config/nav2_params_tron1.yaml
cp -n "$MW/src/mobile_openarm_sim/maps/"world_map.* maps/ 2>/dev/null || true

# 3) 보행 정책: 저장소에 학습 결과가 들어 있음 — policies/tron1_openarm_policy.npz (PF, launch 기본값),
#    policies/tron1sf_openarm_policy.npz (SF, 교육 기준)
#    직접 학습하려면 11_train_policy.sh (별도 가상환경 .venv-biped 에서 JAX GPU 로 학습)
ls -la "$SIM/policies/"

# 4) 시뮬레이션 단독 점검 (ROS 불필요): 서기·걷기·회전·팔 동작·LiDAR·카메라
echo "== 자체 점검: PF =="
MUJOCO_GL=egl "$PY" tools/selftest_tron1.py || MUJOCO_GL=osmesa "$PY" tools/selftest_tron1.py || true
echo "== 자체 점검: SF (교육 기준) =="
SFT=(tools/selftest_tron1.py sf/world_light.xml policies/tron1sf_openarm_policy.npz)
MUJOCO_GL=egl "$PY" "${SFT[@]}" || MUJOCO_GL=osmesa "$PY" "${SFT[@]}" || true

# 5) 빌드: tron1_openarm 워크스페이스만 빌드 (바퀴형 패키지는 위에서 source 한 underlay 를 사용 — DAY2 앱도 그대로)
cd "$T1"
"$PY" -m colcon build --symlink-install --base-paths src
echo "빌드 완료. 이 스크립트는 별도 셸에서 돌았으므로 지금 터미널에는 아직 적용되지 않았습니다."
echo "새 터미널마다(TRON1 은 ~/.bashrc 에 자동 추가하지 않음, Docker 는 이미지가 자동으로 함):"
echo "  source $T1/install/setup.bash   (바퀴형 underlay 도 함께 불러옴)"
echo "실행 예 (교육 기준 Sole-foot): ros2 launch tron1_openarm_sim sim.launch.py foot:=sf world:=world_light.xml"
echo "DAY2 앱을 ros2 run 으로 따로 실행하는 터미널 (SF 기준):"
echo "  export OPENARM_BIPED=1 OPENARM_SRDF=\$(ros2 pkg prefix --share tron1_openarm_moveit_config)/config/tron1_openarm_sf.srdf"
echo "  (PF 로 비교할 때만 tron1_openarm.srdf)"
