#!/bin/bash
# 사용: bash mission_test.sh <pf|sf> <기록 이름> ["자연어 명령"]   (저장소 루트에서 colcon build 후. 로그: $LOG_DIR, 기본 /tmp/tron1_logs)
# 전체 시스템(시뮬 + Nav2 + MoveIt + 작업 관리자 + 오프라인 LLM 플래너)을 띄우고, 정답 기록기를 켠 채
# 자연어 명령 하나를 끝까지 실행한 뒤 상자가 어디에 놓였는지 정답으로 확인한다.
foot=$1; tag=$2; cmd=${3:-"빨간 상자를 왼쪽 작업대로 옮기고 돌아와"}
S=${LOG_DIR:-/tmp/tron1_logs}; mkdir -p $S; D=$(cd "$(dirname "$0")" && pwd)
cd "$D/../../.." && source install/setup.bash
(ros2 launch tron1_openarm_sim full_system.launch.py foot:=$foot planner:=offline viewer:=false rviz:=false > $S/${foot}_full${tag}.log 2>&1 &)
sleep 90
(python3 $D/grasp_recorder.py $S/mrec_${foot}${tag}.jsonl > /dev/null 2>&1 &)
timeout 6000 python3 $D/mission_cli.py "$cmd" 5800 > $S/${foot}_mission${tag}.log 2>&1
echo "=== $foot mission"; grep -E "^LOG|status|완료|실패" $S/${foot}_mission${tag}.log | tail -40
ps -eo pid,args | grep -E "tron1_bridge|move_group|robot_state_publisher|ros2 launch|grasp_recorder|nav2|amcl|controller_server|planner_server|bt_navigator|task_manager|llm_planner|dashboard|map_server|lifecycle|behavior_server|smoother|velocity_smoother|waypoint" | grep -v grep | awk '{print $1}' | xargs -r kill -9; sleep 3
python3 - <<PY
import json, numpy as np
L=[json.loads(l) for l in open('$S/mrec_${foot}${tag}.jsonl')]
b=np.array(L[-1]['box0'])
print('$foot final box0', np.round(b,3), '| on place table (x -2.95..-2.05, y 2.85..3.35, z~0.725):',
      bool(-2.95 < b[0] < -2.05 and 2.85 < b[1] < 3.35 and abs(b[2]-0.725) < 0.03),
      '| max tilt', max(r['tilt'] for r in L), '| final gt', L[-1].get('gt'))
PY
