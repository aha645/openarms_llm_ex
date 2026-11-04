#!/usr/bin/env bash
# TRON1(Point-foot) + OpenArm 실습 환경 준비
#   전제: 바퀴형 워크스페이스 설치(scripts/04_setup_workspace.sh)를 먼저 끝냈을 것 — OpenArm 에셋·mujoco·ROS 패키지를 재사용
#   사용: bash tron1_openarm/scripts/10_setup_tron1.sh
set -e
WS=$(cd "$(dirname "$0")/../.." && pwd)          # 저장소(워크스페이스) 루트
T1=$WS/tron1_openarm
SIM=$T1/src/tron1_openarm_sim
source /opt/ros/jazzy/setup.bash
sudo apt-get install -y ros-jazzy-xacro liburdfdom-tools >/dev/null || true

# 1) LimX TRON1 로봇 설명 (xacro/URDF/MJCF/메시, Apache-2.0)
mkdir -p "$T1/third_party"
touch "$T1/third_party/COLCON_IGNORE"
if [ ! -d "$T1/third_party/tron1-robot-description" ]; then
  git clone --depth 1 https://github.com/limxdynamics/tron1-robot-description.git "$T1/third_party/tron1-robot-description"
fi
# (선택) LimX 시뮬레이터·배포 코드 — 비교·참고용
[ -d "$T1/third_party/tron1-mujoco-sim" ] || git clone --depth 1 https://github.com/limxdynamics/tron1-mujoco-sim.git "$T1/third_party/tron1-mujoco-sim" || true
test -f "$WS/third_party/openarm_mujoco/v1/openarm_bimanual.xml" || { echo "먼저 scripts/04_setup_workspace.sh 를 실행하세요"; exit 1; }

# 2) 모델 생성: xacro → URDF → MJCF + OpenArm 접목 → 월드 → ROS 용 URDF → SRDF → Nav2 파라미터
cd "$SIM"
python3 tools/build_tron1_openarm.py --tron1 "$T1/third_party/tron1-robot-description/pointfoot/PF_TRON1A" \
        --openarm "$WS/third_party/openarm_mujoco/v1/openarm_bimanual.xml" --out mjcf
python3 tools/build_world_tron1.py --wheeled-tools "$WS/src/mobile_openarm_sim/tools" --out mjcf --config config
python3 tools/make_urdf_tron1.py --wheeled-tools "$WS/src/mobile_openarm_sim/tools" --mjcf mjcf/tron1_openarm.xml --out urdf/tron1_openarm.urdf
python3 "$WS/src/mobile_openarm_moveit_config/tools/make_srdf.py" --mjcf mjcf/tron1_openarm.xml \
        --out "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm.srdf" --samples 2000
sed -i 's/<robot name="mobile_openarm">/<robot name="tron1_openarm">/' "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm.srdf"
python3 tools/patch_srdf_home.py "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm.srdf"   # 이족용 home (팔 벌림)
# Sole-foot(발목 있음) 변형: 같은 순서로 mjcf/sf, urdf/tron1_openarm_sf.urdf, tron1_openarm_sf.srdf
python3 tools/build_tron1_openarm.py --foot sf --base-height 0.73 --tron1 "$T1/third_party/tron1-robot-description/pointfoot/SF_TRON1A" \
        --openarm "$WS/third_party/openarm_mujoco/v1/openarm_bimanual.xml" --out mjcf/sf
python3 tools/build_world_tron1.py --wheeled-tools "$WS/src/mobile_openarm_sim/tools" --out mjcf/sf --config /tmp/tron1_sf_cfg
python3 tools/make_urdf_tron1.py --wheeled-tools "$WS/src/mobile_openarm_sim/tools" --mjcf mjcf/sf/tron1_openarm.xml --out urdf/tron1_openarm_sf.urdf \
        --base-height 0.73 --mesh-prefix package://tron1_openarm_sim/mjcf/sf/
python3 "$WS/src/mobile_openarm_moveit_config/tools/make_srdf.py" --mjcf mjcf/sf/tron1_openarm.xml \
        --out "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm_sf.srdf" --samples 2000
sed -i 's/<robot name="mobile_openarm">/<robot name="tron1_openarm">/' "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm_sf.srdf"
python3 tools/patch_srdf_home.py "$T1/src/tron1_openarm_moveit_config/config/tron1_openarm_sf.srdf"
python3 tools/make_nav2_params_tron1.py --src "$WS/src/mobile_openarm_sim/config/nav2_params.yaml" --out config/nav2_params_tron1.yaml
cp -n "$WS/src/mobile_openarm_sim/maps/"world_map.* maps/ 2>/dev/null || true

# 3) 보행 정책: 저장소에 학습 결과(policies/tron1_openarm_policy.npz)가 들어 있음. 직접 학습하려면 11_train_policy.sh
ls -la "$SIM/policies/"

# 4) 시뮬레이션 단독 점검 (ROS 불필요): 서기·걷기·회전·팔 동작·LiDAR·카메라
MUJOCO_GL=egl python3 tools/selftest_tron1.py || MUJOCO_GL=osmesa python3 tools/selftest_tron1.py || true

# 5) 빌드 (바퀴형 패키지 + TRON1 패키지 함께 — DAY2 앱은 바퀴형 것을 그대로 씀)
cd "$WS"
colcon build --symlink-install --base-paths src tron1_openarm/src
echo "빌드 완료: source $WS/install/setup.bash"
echo "실행 예: ros2 launch tron1_openarm_sim sim.launch.py"
echo "DAY2 앱을 ros2 run 으로 따로 실행할 때: export OPENARM_BIPED=1 OPENARM_SRDF=\$(ros2 pkg prefix --share tron1_openarm_moveit_config)/config/tron1_openarm.srdf"
