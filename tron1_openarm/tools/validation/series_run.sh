#!/bin/bash
# 무작위 배치 연속 집기 시험 (화면에 보이게, VIEW=1). 사용: [BOX_SOURCE=color] bash series_run.sh <기록 접두어> r1 r2 ... (배치: series_layouts.txt)
#   결과: $LOG_DIR/series_<접두어>.txt (RESULT 이름 놓기완료수 최종상자위치), 각 시험 로그 $LOG_DIR/series_<접두어><이름>.log
D=$(cd "$(dirname "$0")" && pwd); S=${LOG_DIR:-/tmp/tron1_logs}; mkdir -p $S
pre=$1; shift
cd "$D/../../.."
for k in "$@"; do
  read -r name obj boxes < <(grep "^$k " $D/series_layouts.txt)
  echo "===== $name $obj $boxes"
  OBJ=$obj BOXES="$boxes" VIEW=1 BOX_SOURCE=${BOX_SOURCE:-aruco} bash tron1_openarm/tools/validation/pick_test.sh sf _$pre$name 2>&1 | tee $S/series_$pre$name.log \
    | grep -E "\[pick\] (완료|box)|\[place\] 완료|실패|오류|contacts|pick_place"
  echo "RESULT $name $(grep -c 'place\] 완료' $S/series_$pre$name.log) $(grep -o 'final box.*' $S/series_$pre$name.log)" | tee -a $S/series_$pre.txt
done
echo SERIES_DONE | tee -a $S/series_$pre.txt
