#!/usr/bin/env bash
# ============================================================================
# [1단계] TRON1 시뮬레이션용 파이썬 가상환경 만들기  →  tron1_openarm/.venv-biped
#
#   이 가상환경으로 도는 것: MuJoCo 시뮬, ROS 노드(브리지·카메라·집기 앱·작업 관리자·
#   LLM 계획기·대시보드), 보행 정책 평가(eval_policy.py)  — 모두 CPU
#   (GPU 로 PPO 학습할 때는 따로: setup_venv_biped_train_ppo.sh → .venv-biped-train)
#
#   사용법:  bash tron1_openarm/scripts/setup_venv_biped.sh
#            (setup_workspace.sh 가 자동으로 불러 주므로 보통은 직접 안 해도 됨)
#   여러 번 실행해도 괜찮다 (있으면 그대로 쓰고 빠진 것만 설치).
# ============================================================================
set -e

T1=$(cd "$(dirname "$0")/.." && pwd)   # tron1_openarm 폴더
VENV=$T1/.venv-biped                   # 만들 가상환경 위치

# 1) 가상환경 만들기
#    --system-site-packages : ROS 가 apt 로 설치한 rclpy·OpenCV·numpy 를 그대로 같이 쓴다
if [ ! -f "$VENV/pyvenv.cfg" ]; then
  echo "== 가상환경 만들기: $VENV"
  /usr/bin/python3 -m venv --system-site-packages "$VENV"
elif ! grep -q "include-system-site-packages = true" "$VENV/pyvenv.cfg"; then
  echo "[오류] $VENV 가 예전 학습용 가상환경입니다 (학습용은 이제 .venv-biped-train)."
  echo "       지우고 다시 실행하세요:  rm -rf $VENV"
  exit 1
fi
touch "$VENV/COLCON_IGNORE"            # colcon 빌드가 가상환경 폴더를 건드리지 않게

# 2) numpy 버전 고정
#    ROS 의 cv_bridge 등은 apt 의 numpy(1.26) 로 만들어져 있어서, pip 가 numpy 2.x 를 깔면 깨진다
NUMPY_VER=$(env -u PYTHONPATH /usr/bin/python3 -c "import numpy; print(numpy.__version__)")
echo "numpy==$NUMPY_VER" > "$VENV/ros-constraints.txt"

# 3) 필요한 패키지 설치
#    mujoco : 물리 시뮬레이터  /  openai, pydantic : LLM 계획기  /  flask : 웹 대시보드
echo "== 패키지 설치 (numpy $NUMPY_VER 고정)"
env -u PYTHONPATH PYTHONNOUSERSITE=1 "$VENV/bin/python" -m pip install -q \
    -c "$VENV/ros-constraints.txt"  'mujoco==3.5.0' 'openai>=1.60' 'pydantic>=2.5' flask

# 4) 확인
env -u PYTHONPATH PYTHONNOUSERSITE=1 "$VENV/bin/python" -c "
import numpy, cv2, mujoco, flask, openai, pydantic
print('numpy', numpy.__version__, '| opencv', cv2.__version__, '| mujoco', mujoco.__version__, '| pydantic', pydantic.VERSION)
assert numpy.__version__ == '$NUMPY_VER', 'numpy 가 ROS 버전과 다릅니다'
"
echo "== 완료: $VENV"
echo "   python 을 직접 쓸 때(eval_policy.py 등):  source $VENV/bin/activate"
