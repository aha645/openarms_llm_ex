#!/usr/bin/env bash
# Create project-local environments; does not run the legacy workspace installer.
set -euo pipefail
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MODE=${1:-ros}
INSTALL=${2:-}
if [[ "$MODE" == "--help" || "$MODE" == "-h" ]]; then
  echo "Usage: bash scripts/05_setup_venv.sh [ros|train] [--install]"
  echo "  ros:   .venv-ros, uses system packages; --install installs ROS app dependencies"
  echo "  train: .venv-train, isolated; GPU dependencies must be selected separately"
  exit 0
fi
if [[ "$MODE" != ros && "$MODE" != train ]] || [[ -n "$INSTALL" && "$INSTALL" != --install ]] || (( $# > 2 )); then
  echo "Invalid arguments; use --help" >&2; exit 2
fi
if [[ "$MODE" == train && -n "$INSTALL" ]]; then
  echo "Training dependency installation is intentionally not automated (ARM64/CUDA compatibility)." >&2
  exit 2
fi
PYTHON=/usr/bin/python3
VENV="$WS/.venv-$MODE"
if [[ ! -x "$PYTHON" ]]; then
  echo "Missing $PYTHON" >&2; exit 1
fi
if [[ -e "$VENV" && ! -f "$VENV/pyvenv.cfg" ]]; then
  echo "Refusing to overwrite non-venv directory: $VENV" >&2; exit 1
fi
if [[ -f "$VENV/pyvenv.cfg" ]]; then
  EXPECTED=false; [[ "$MODE" == ros ]] && EXPECTED=true
  if ! grep -qi "^include-system-site-packages = $EXPECTED$" "$VENV/pyvenv.cfg"; then
    echo "Existing environment has incompatible system-site-packages setting: $VENV" >&2
    exit 1
  fi
else
  ARGS=()
  [[ "$MODE" == ros ]] && ARGS+=(--system-site-packages)
  if ! PYTHONNOUSERSITE=1 "$PYTHON" -m venv "${ARGS[@]}" "$VENV"; then
    echo "venv creation failed. Install: sudo apt install python3-venv" >&2
    exit 1
  fi
fi
# Prevent colcon from discovering packages inside the virtual environment.
touch "$VENV/COLCON_IGNORE"
if [[ "$INSTALL" == --install ]]; then
  # Read apt's NumPy, not a version shadowed by ~/.local or the virtualenv.
  if ! NPV=$(env -u PYTHONPATH PYTHONNOUSERSITE=1 "$PYTHON" -c 'import numpy, cv2; print(numpy.__version__)'); then
    echo "Install ROS-side dependencies first: sudo apt install python3-numpy python3-opencv" >&2
    exit 1
  fi
  printf 'numpy==%s\n' "$NPV" > "$VENV/ros-constraints.txt"
  env -u PYTHONPATH PYTHONNOUSERSITE=1 "$VENV/bin/python" -m pip install \
    -c "$VENV/ros-constraints.txt" 'mujoco==3.5.0' 'openai>=1.60' 'pydantic>=2.5' flask
  env -u PYTHONPATH PYTHONNOUSERSITE=1 "$VENV/bin/python" -c \
    'import sys, numpy, cv2, mujoco, flask, openai, pydantic; print("Python:", sys.executable); print("NumPy:", numpy.__version__, "OpenCV:", cv2.__version__, "MuJoCo:", mujoco.__version__)'
fi
printf '\nEnvironment ready: %s\n' "$VENV"
printf 'Activate in the current terminal:\n  export PYTHONNOUSERSITE=1\n  source "%s/bin/activate"\n' "$VENV"
if [[ "$MODE" == ros ]]; then
  echo '  source /opt/ros/jazzy/setup.bash'
  echo 'Do not run legacy 04_setup_workspace.sh inside this venv (it uses pip --user).'
  echo 'See doc/setup/project-venv.md for build and runtime constraints.'
else
  echo 'Use a fresh terminal without ROS/PYTHONPATH overlays for training.'
  echo 'No JAX/CUDA/MJX dependencies have been installed.'
fi
