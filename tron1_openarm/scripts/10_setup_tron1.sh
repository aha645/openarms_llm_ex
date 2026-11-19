#!/usr/bin/env bash
# TRON1(Point-foot/Sole-foot) + OpenArm 실습 환경 준비: 모델·정책 확인 → 자체 점검 → colcon build
#   전제: 바퀴형 워크스페이스 설치(mobile_openarm/scripts/04_setup_workspace.sh)를 먼저 끝냈을 것
#         — .venv-mobile(mujoco 등)·바퀴형 ROS 패키지를 재사용
#   사용: bash tron1_openarm/scripts/10_setup_tron1.sh
#   빌드: tron1_openarm/ 가 별도 colcon 워크스페이스이며, mobile_openarm/install 위에 겹쳐(overlay) 빌드한다
#   모델(MJCF·URDF·SRDF·메시)과 보행 정책은 저장소에 들어 있는 것을 그대로 쓴다 — 인터넷으로 원본을 받지 않고, 생성 파일을 덮어쓰지 않는다.
#   LimX TRON1·OpenArm 원본에서 모델을 다시 만들 때만: bash tron1_openarm/scripts/13_regen_tron1_models.sh
set -e
T1=$(cd "$(dirname "$0")/.." && pwd)            # tron1_openarm 워크스페이스
MW=$(dirname "$T1")/mobile_openarm              # 바퀴형 워크스페이스 (underlay)
SIM=$T1/src/tron1_openarm_sim
PY=$MW/.venv-mobile/bin/python                  # ROS 실행용 venv (TRON1 ROS 노드도 같이 씀)
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy 가 없습니다. 먼저 설치하세요: 노트북은 mobile_openarm/scripts/03_install_ros2_jazzy.sh, DGX Spark 는 doc/setup/dgx-spark.md 3절"; exit 1
fi
source /opt/ros/jazzy/setup.bash
if [ ! -x "$PY" ] || [ ! -f "$MW/install/setup.bash" ]; then
  echo "먼저 bash mobile_openarm/scripts/04_setup_workspace.sh 를 실행하세요"; exit 1
fi
source "$MW/install/setup.bash"                 # 바퀴형 패키지(mobile_openarm_sim·apps·moveit_config)를 underlay 로
export PYTHONNOUSERSITE=1
# 1)~2) 저장소의 모델·메시 확인 (출처·라이선스: src/tron1_openarm_sim/mjcf/MESHES_NOTICE.md)
for f in mjcf/sf/tron1_openarm.xml mjcf/sf/world.xml mjcf/sf/meshes/tron1 mjcf/sf/meshes/visual mjcf/tron1_openarm.xml mjcf/meshes/tron1 \
         urdf/tron1_openarm_sf.urdf urdf/tron1_openarm.urdf config/nav2_params_tron1.yaml; do
  [ -e "$SIM/$f" ] || { echo "모델 파일이 없습니다: $SIM/$f — 저장소를 다시 받거나 bash tron1_openarm/scripts/13_regen_tron1_models.sh"; exit 1; }
done
# 3) 보행 정책: 저장소에 학습 결과가 들어 있음 — policies/tron1sf_openarm_policy.npz (SF, launch 기본값·교육 기준),
#    policies/tron1_openarm_policy.npz (PF, foot:=pf)
#    직접 학습하려면 11_train_policy.sh (별도 가상환경 .venv-biped 에서 JAX GPU 로 학습)
ls -la "$SIM/policies/"

# 4) 시뮬레이션 단독 점검 (ROS 불필요): 서기·걷기·회전·팔 동작·LiDAR·카메라
cd "$SIM"
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
