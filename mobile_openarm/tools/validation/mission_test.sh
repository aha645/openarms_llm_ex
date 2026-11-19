#!/bin/bash
# 바퀴형 회귀 시험: 사용  [VIEW=1] [MAX_STEPS=20] bash mission_test.sh <기록 이름> ["자연어 명령"]   (04_setup_workspace.sh 로 빌드 후. 로그: $LOG_DIR, 기본 /tmp/wheel_logs)
# 전체 시스템(시뮬 + Nav2 + MoveIt + 작업 관리자 + 오프라인 플래너)을 띄우고 자연어 명령 하나를 끝까지 실행한 뒤,
# /sim/objects 정답 위치로 상자가 놓기 작업대 위에 있는지, 팔이 어디에 닿았는지(contact_recorder.py) 확인한다. 명령 전달·기록은 TRON1 의 mission_cli.py 를 같이 쓴다.
# MAX_STEPS: 계획 최대 단계(바퀴형 기본 12 — "상자 모두 옮기고 돌아와" 13 단계는 20 으로 올려야 함)
tag=$1; cmd=${2:-"빨간 상자를 왼쪽 작업대로 옮기고 돌아와"}
S=${LOG_DIR:-/tmp/wheel_logs}; mkdir -p $S; D=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$D/../../.." && pwd)
cd "$D/../.." && source /opt/ros/jazzy/setup.bash && source .venv-mobile/bin/activate && source install/setup.bash
[ -n "${MAX_STEPS:-}" ] && export OPENARM_MAX_STEPS=$MAX_STEPS
V=$([ -n "${VIEW:-}" ] && echo true || echo false)
L=$S/full_$tag.log
setsid ros2 launch mobile_openarm_apps full_system.launch.py planner:=offline viewer:=$V rviz:=$V > $L 2>&1 &
PG=$!
for i in $(seq 1 360); do   # 작업 관리자 준비 + Nav2 활성 + MoveIt 준비, 그 뒤 5 s (최대 180 s)
  grep -q "task_manager 준비 완료" $L 2>/dev/null && grep -q "Managed nodes are active" $L 2>/dev/null \
    && grep -q "You can start planning now" $L 2>/dev/null && break
  sleep 0.5
done
sleep 5
echo "준비까지 $((i / 2 + 5)) s"
if [ -n "${VIEW:-}" ]; then   # 손목 카메라 영상 창 (camera_process:=true 기본일 때)
  for side in left right; do (ros2 run rqt_image_view rqt_image_view /wrist_${side}_camera/image_raw/compressed > $S/view_${side}_$tag.log 2>&1 &); done
fi
python3 $D/contact_recorder.py $S/contacts_$tag.log > /dev/null 2>&1 &
CR=$!
T0=$(date +%s)
timeout 6000 python3 $ROOT/tron1_openarm/tools/validation/mission_cli.py "$cmd" 5800 > $S/mission_$tag.log 2>&1
echo "=== 미션 ($(( $(date +%s) - T0 )) s): $cmd"; grep -E "^LOG|state=|FINAL" $S/mission_$tag.log | tail -40
python3 - <<'PY'
import json, rclpy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("wheel_check"); got = {}
n.create_subscription(String, "/sim/objects", lambda m: got.setdefault("o", json.loads(m.data)["objects"]), 10)
for _ in range(100):
    if "o" in got: break
    rclpy.spin_once(n, timeout_sec=0.1)
for k, b in sorted(got.get("o", {}).items()):
    on = -2.95 < b[0] < -2.05 and 2.85 < b[1] < 3.35 and abs(b[2] - 0.725) < 0.03
    print(f"{k}: ({b[0]:.3f}, {b[1]:.3f}, {b[2]:.3f})  놓기 작업대 위: {on}")
PY
kill $CR 2>/dev/null
echo "=== 팔 접촉 (작업대·상자·벽, 쥔 상자 제외)"; grep -v "(없음)" $S/contacts_$tag.log | tail -30
pkill -f "rqt_image_view /wrist_" 2>/dev/null
kill -- -$PG 2>/dev/null; sleep 3; kill -9 -- -$PG 2>/dev/null
