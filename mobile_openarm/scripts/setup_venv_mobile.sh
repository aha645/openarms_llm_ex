#!/usr/bin/env bash
# mobile_openarm 용 Python 가상환경(.venv-mobile) 생성 + 의존성 설치
#   용도: MuJoCo 시뮬·ROS 브리지, SLAM/Nav2/MoveIt 연동 노드, LLM 계획기(OpenAI), Flask 웹 대시보드
#         (휠형과 TRON1 의 ROS 실행이 함께 쓴다. TRON1 PPO 학습은 별도 .venv-biped)
#   사용: bash mobile_openarm/scripts/setup_venv_mobile.sh      (04_setup_workspace.sh 가 자동 호출)
#   - --system-site-packages: apt 로 설치된 rclpy·cv_bridge·OpenCV·NumPy·colcon 을 그대로 쓴다
#   - numpy 는 apt(ROS) 버전으로 고정: pip 가 numpy 2.x 를 들이면 numpy 1.x 로 빌드된 cv_bridge 등이 깨진다
#   - 여러 번 실행해도 안전: 이미 있으면 재사용하고, 빠진 패키지만 설치한다
set -euo pipefail
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)     # mobile_openarm 워크스페이스
VENV=$WS/.venv-mobile
PYTHON=/usr/bin/python3

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: bash mobile_openarm/scripts/setup_venv_mobile.sh"
  echo "  $VENV 생성(--system-site-packages) → mujoco==3.5.0, openai, pydantic v2, flask 설치"
  exit 0
fi
(( $# == 0 )) || { echo "인자가 필요 없습니다. --help 참고" >&2; exit 2; }
[[ -x "$PYTHON" ]] || { echo "$PYTHON 가 없습니다" >&2; exit 1; }

# 1) 가상환경 생성 (빈 폴더는 허용: Docker named volume 마운트 지점)
if [[ -d "$VENV" && ! -f "$VENV/pyvenv.cfg" && -n "$(ls -A "$VENV")" ]] || [[ -e "$VENV" && ! -d "$VENV" ]]; then
  echo "가상환경이 아닌 파일/폴더가 있어 덮어쓰지 않습니다: $VENV" >&2; exit 1
fi
if [[ -f "$VENV/pyvenv.cfg" ]]; then
  if ! grep -qi '^include-system-site-packages = true$' "$VENV/pyvenv.cfg"; then
    echo "기존 $VENV 가 --system-site-packages 없이 만들어졌습니다. 지우고 다시 실행하세요: rm -rf $VENV" >&2
    exit 1
  fi
else
  if ! PYTHONNOUSERSITE=1 "$PYTHON" -m venv --system-site-packages "$VENV"; then
    echo "venv 생성 실패. 설치 후 다시 실행: sudo apt install python3-venv" >&2; exit 1
  fi
fi
touch "$VENV/COLCON_IGNORE"      # colcon 이 venv 안의 setup.py 를 패키지로 찾지 않도록

# 2) ROS(apt) 쪽 numpy 버전 읽기 — ~/.local 이나 PYTHONPATH 에 가려지지 않은 apt 버전
if ! NPV=$(env -u PYTHONPATH PYTHONNOUSERSITE=1 "$PYTHON" -c 'import numpy, cv2; print(numpy.__version__)'); then
  echo "ROS 쪽 Python 패키지가 없습니다: sudo apt install python3-numpy python3-opencv" >&2; exit 1
fi
printf 'numpy==%s\n' "$NPV" > "$VENV/ros-constraints.txt"

# 3) 의존성 설치 (venv 안이므로 --user / --break-system-packages 불필요)
env -u PYTHONPATH PYTHONNOUSERSITE=1 "$VENV/bin/python" -m pip install \
  -c "$VENV/ros-constraints.txt" 'mujoco==3.5.0' 'openai>=1.60' 'pydantic>=2.5' flask

# 4) 확인: numpy 가 apt 버전 그대로인지, pydantic 이 v2 인지
env -u PYTHONPATH PYTHONNOUSERSITE=1 NPV="$NPV" "$VENV/bin/python" - <<'PY'
import os, sys, numpy, cv2, mujoco, flask, openai, pydantic
print("Python  :", sys.executable)
print("NumPy   :", numpy.__version__, "| OpenCV:", cv2.__version__, "| MuJoCo:", mujoco.__version__)
print("OpenAI  :", openai.__version__, "| Pydantic:", pydantic.VERSION)
assert numpy.__version__ == os.environ["NPV"], "numpy 가 ROS(apt) 버전과 다릅니다"
assert int(pydantic.VERSION.split(".")[0]) >= 2, "pydantic v2 가 아닙니다"
PY

# 5) 예전 04_setup_workspace.sh(pip --user)가 ~/.local 에 남긴 패키지 경고
#    --system-site-packages venv 는 ~/.local 을 venv 보다 먼저 읽으므로 섞일 수 있다
USER_SITE=$("$PYTHON" -c 'import site; print(site.getusersitepackages())')
LEFT=$(ls -d "$USER_SITE"/{mujoco,openai,pydantic,pydantic_core} 2>/dev/null || true)
if [[ -n "$LEFT" ]]; then
  echo
  echo "[경고] ~/.local 에 예전 설치가 남아 있어 venv 패키지보다 먼저 읽힐 수 있습니다:"
  echo "$LEFT" | sed 's/^/  /'
  echo "  정리: $PYTHON -m pip uninstall --user --break-system-packages mujoco openai pydantic pydantic-core"
fi

printf '\n준비 완료: %s\n' "$VENV"
echo "현재 터미널에서 python3 를 직접 쓸 때(모델 생성·selftest·mujoco.viewer):"
echo "  source $VENV/bin/activate"
echo "ros2 run/launch 실행 파일은 이 venv 의 Python 으로 빌드되면(04_setup_workspace.sh) 활성화 없이도 이 venv 를 씁니다."
