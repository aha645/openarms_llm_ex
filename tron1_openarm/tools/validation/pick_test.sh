#!/bin/bash
# 사용: bash pick_test.sh <pf|sf> <기록 이름>   (저장소 루트에서 colcon build 후. 로그: $LOG_DIR, 기본 /tmp/tron1_logs)
# 시뮬 + MoveIt 을 띄우고, 정답 기록기(grasp_recorder.py)를 켠 채 pick_place 를 한 번 실행한 뒤 결과를 요약한다
foot=$1; tag=$2; S=${LOG_DIR:-/tmp/tron1_logs}; mkdir -p $S; D=$(cd "$(dirname "$0")" && pwd)
cd "$D/../../.." && source install/setup.bash
(ros2 launch tron1_openarm_sim sim.launch.py foot:=$foot start_x:=2.5 start_y:=2.45 start_yaw:=1.5708 viewer:=false rviz:=false > $S/${foot}_sim.log 2>&1 &)
(ros2 launch tron1_openarm_moveit_config move_group.launch.py foot:=$foot rviz:=false > $S/${foot}_mg.log 2>&1 &)
sleep 45
(python3 $D/grasp_recorder.py $S/rec_${foot}${tag}.jsonl > /dev/null 2>&1 &)
SR=tron1_openarm.srdf; [ $foot = sf ] && SR=tron1_openarm_sf.srdf
export OPENARM_BIPED=1 OPENARM_SRDF=$(ros2 pkg prefix tron1_openarm_moveit_config)/share/tron1_openarm_moveit_config/config/$SR
timeout 1500 ros2 run mobile_openarm_apps pick_place --ros-args -p object:=box_0 > $S/${foot}pk${tag}.log 2>&1
echo "=== $foot"; grep -vE "RTPS|^\s*$" $S/${foot}pk${tag}.log | grep -E "\] \[|ERROR|완료" | sed 's/.*\]: //'
ps -eo pid,args | grep -E "tron1_bridge|move_group|robot_state_publisher|ros2 launch|rec2.py" | grep -v grep | awk '{print $1}' | xargs -r kill -9; sleep 3
python3 - <<PY
import json, numpy as np
L=[json.loads(l) for l in open('$S/rec_${foot}${tag}.jsonl')]
b0=np.array(L[0]['box0'])
cont=sorted({c for r in L for c in (r.get('ac') or [])})
mn=min(L,key=lambda r:np.linalg.norm(np.array(r['tcp'])-np.array(r['box0'])))
print('$foot contacts', cont, '| closest', np.round(np.array(mn['tcp'])-np.array(mn['box0']),3), '| max box rise %.3f'%(max(r['box0'][2] for r in L)-b0[2]), '| max tilt', max(r['tilt'] for r in L), 'final box', np.round(L[-1]['box0'],3))
PY
