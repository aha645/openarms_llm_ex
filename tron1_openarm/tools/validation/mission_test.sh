#!/bin/bash
# 사용: [VIEW=1] [BOX_SOURCE=aruco(기본 color: 마커 없이 색)] [START=x,y,yaw(rad)] bash mission_test.sh <pf|sf> <기록 이름> ["자연어 명령"]   (10_setup_tron1.sh 로 tron1_openarm 빌드 후. 로그: $LOG_DIR, 기본 /tmp/tron1_logs)
# 전체 시스템(시뮬 + Nav2 + MoveIt + 작업 관리자 + 오프라인 LLM 플래너)을 띄우고, 정답 기록기를 켠 채
# 자연어 명령 하나를 끝까지 실행한 뒤 상자가 어디에 놓였는지 정답으로 확인한다.
foot=$1; tag=$2; cmd=${3:-"빨간 상자를 왼쪽 작업대로 옮기고 돌아와"}
S=${LOG_DIR:-/tmp/tron1_logs}; mkdir -p $S; D=$(cd "$(dirname "$0")" && pwd)
cd "$D/../.." && source install/setup.bash   # tron1_openarm 워크스페이스 (mobile_openarm underlay 포함)
SA=(); if [ -n "${START:-}" ]; then IFS=, read -r sx sy syaw <<< "$START"; SA=(start_x:=$sx start_y:=$sy start_yaw:=$syaw); fi
(ros2 launch tron1_openarm_sim full_system.launch.py "${SA[@]}" foot:=$foot planner:=offline box_source:=${BOX_SOURCE:-color} viewer:=$([ -n "$VIEW" ] && echo true || echo false) rviz:=$([ -n "$VIEW" ] && echo true || echo false) > $S/${foot}_full${tag}.log 2>&1 &)
# 준비될 때까지만 (예전 고정 90 s: 실제로는 작업 관리자 준비 약 16 s). 작업 관리자 준비 + Nav2 활성 + MoveIt 준비, 그 뒤 5 s. 최대 180 s
L=$S/${foot}_full${tag}.log
for i in $(seq 1 360); do
  grep -q "task_manager 준비 완료" $L 2>/dev/null && grep -q "Managed nodes are active" $L 2>/dev/null \
    && grep -q "You can start planning now" $L 2>/dev/null && break
  sleep 0.5
done
sleep 5
echo "준비까지 $((i / 2 + 5)) s"
(python3 $D/grasp_recorder.py $S/mrec_${foot}${tag}.jsonl > /dev/null 2>&1 &)
timeout 6000 python3 $D/mission_cli.py "$cmd" 5800 > $S/${foot}_mission${tag}.log 2>&1
echo "=== $foot mission"; grep -E "^LOG|status|완료|실패" $S/${foot}_mission${tag}.log | tail -40
ps -eo pid,args | grep -E "tron1_bridge|tron1_cameras|rviz2|move_group|robot_state_publisher|ros2 launch|grasp_recorder|nav2|amcl|controller_server|planner_server|bt_navigator|task_manager|llm_planner|dashboard|map_server|lifecycle|behavior_server|smoother|velocity_smoother|waypoint" | grep -v grep | awk '{print $1}' | xargs -r kill -9; sleep 3
python3 - <<PY
import json, numpy as np
L=[json.loads(l) for l in open('$S/mrec_${foot}${tag}.jsonl')]
b=np.array(L[-1]['box0'])
print('$foot final box0', np.round(b,3), '| on place table (x -2.95..-2.05, y 2.85..3.35, z~0.725):',
      bool(-2.95 < b[0] < -2.05 and 2.85 < b[1] < 3.35 and abs(b[2]-0.725) < 0.03),
      '| max tilt', max(r['tilt'] for r in L), '| final gt', L[-1].get('gt'))
PY
