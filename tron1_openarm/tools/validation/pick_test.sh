#!/bin/bash
# 사용: bash pick_test.sh <pf|sf> <기록 이름>   (10_setup_tron1.sh 로 tron1_openarm 빌드 후. 로그: $LOG_DIR, 기본 /tmp/tron1_logs)
#       VIEW=1 bash pick_test.sh sf _r1          → MuJoCo 뷰어(로봇 움직임) + RViz(MoveIt 계획 장면) + 양 손목 카메라 영상 창을 띄워 눈으로 확인
#       OBJ=box_1 BOXES="box_1=2.43,3.00,-15" bash pick_test.sh sf _l1   → 집을 상자(box_1 = 로봇 왼쪽 → 왼팔)와 상자 위치(월드 x,y,yaw도)
# 시뮬 + MoveIt 을 띄우고, 정답 기록기(grasp_recorder.py)를 켠 채 pick_place 를 한 번 실행한 뒤 결과를 요약한다
foot=$1; tag=$2; S=${LOG_DIR:-/tmp/tron1_logs}; mkdir -p $S; D=$(cd "$(dirname "$0")" && pwd)
cd "$D/../.." && source install/setup.bash   # tron1_openarm 워크스페이스 (mobile_openarm underlay 포함)
V=false; [ "${VIEW:-0}" = 1 ] && V=true
BA=(); [ -n "${BOXES:-}" ] && BA=("box_poses:=$BOXES")   # 빈 값(box_poses:=)은 ros2 launch 가 거부
echo "로그: $S/${foot}_sim.log (시뮬·브리지), $S/${foot}_mg.log (MoveIt), $S/${foot}pk${tag}.log (집기 앱)"
(ros2 launch tron1_openarm_sim sim.launch.py foot:=$foot start_x:=2.5 start_y:=2.45 start_yaw:=1.5708 viewer:=$V rviz:=false "${BA[@]}" > $S/${foot}_sim.log 2>&1 &)
(ros2 launch tron1_openarm_moveit_config move_group.launch.py foot:=$foot rviz:=$V > $S/${foot}_mg.log 2>&1 &)
sleep 45
if [ "$V" = true ]; then   # 손목 카메라 영상 (카메라 서버 tron1_cameras 가 내는 JPEG 압축 영상)
  for s in left right; do (ros2 run rqt_image_view rqt_image_view /wrist_${s}_camera/image_raw/compressed > $S/${foot}_view_$s.log 2>&1 &); done
fi
(python3 $D/grasp_recorder.py $S/rec_${foot}${tag}.jsonl ${OBJ:-box_0} > /dev/null 2>&1 &)
SR=tron1_openarm.srdf; [ $foot = sf ] && SR=tron1_openarm_sf.srdf
export OPENARM_BIPED=1 OPENARM_SRDF=$(ros2 pkg prefix tron1_openarm_moveit_config)/share/tron1_openarm_moveit_config/config/$SR
timeout 1500 ros2 run mobile_openarm_apps pick_place --ros-args -p object:=${OBJ:-box_0} > $S/${foot}pk${tag}.log 2>&1
echo "=== $foot"; grep -vE "RTPS|^\s*$" $S/${foot}pk${tag}.log | grep -E "\] \[|ERROR|완료" | sed 's/.*\]: //'
# RViz 는 먼저 정상 종료(SIGINT) — 다른 노드를 kill -9 로 먼저 지우면 남은 rviz2 가 종료하며 SIGSEGV 로 죽어 크래시 창이 뜬다
pkill -INT -x rviz2 2>/dev/null; pkill -INT -f "rqt_image_view /wrist_" 2>/dev/null; sleep 2
ps -eo pid,args | grep -E "tron1_bridge|tron1_cameras|move_group|robot_state_publisher|ros2 launch|grasp_recorder|rviz2|rqt_image_view /wrist_" | grep -v grep | awk '{print $1}' | xargs -r kill -9; sleep 3
python3 - <<PY
import json, numpy as np
L=[json.loads(l) for l in open('$S/rec_${foot}${tag}.jsonl')]
b0=np.array(L[0]['box0'])
cont=sorted({c for r in L for c in (r.get('ac') or [])})
k=min(('tcp','tcp_l'),key=lambda k:min(np.linalg.norm(np.array(r[k])-np.array(r['box0'])) for r in L if r.get(k)))   # 집은 손
mn=min((r for r in L if r.get(k)),key=lambda r:np.linalg.norm(np.array(r[k])-np.array(r['box0'])))
print('$foot ${OBJ:-box_0}', 'left' if k=='tcp_l' else 'right', 'contacts', cont, '| closest', np.round(np.array(mn[k])-np.array(mn['box0']),3), '| max box rise %.3f'%(max(r['box0'][2] for r in L)-b0[2]), '| max tilt', max(r['tilt'] for r in L), 'final box', np.round(L[-1]['box0'],3))
PY
