#!/usr/bin/env python3
"""집기·미션 시험 기록(grasp_recorder.py 의 rec_*.jsonl, 0.2 s 마다 한 줄)에서 서 있어야 할 때 얼마나 걸었는지 센다.
    python3 step_time.py /tmp/tron1_logs/rec_sf_v9bA_r*.jsonl
  걸음 모드: "st"(정책 서기 모드) 가 false 인 시간 — 이동 명령으로 걷는 시간도 포함
  방향 바로잡기: 걸음 모드이면서 명령이 회전만(|vx| < 0.02, |wz| > 0.005) — 브리지 서기 방향 유지(STAND_YAW_TOL)가 건 걸음
  최대 기울기: "tilt" 최댓값 [°]
보행 정책 후보를 집기 시험으로 비교할 때 사용 (doc/tron1/locomotion-rl.md 14-7).
"""
import json
import sys

DT = 0.2
if len(sys.argv) < 2:
    sys.exit(__doc__)
for f in sys.argv[1:]:
    L = []
    for line in open(f):
        try:
            L.append(json.loads(line))
        except ValueError:
            pass
    if not L:
        print(f"{f}: 기록 없음")
        continue
    walk = sum(1 for r in L if r.get("st") is False)
    turn = sum(1 for r in L if r.get("st") is False and r.get("cmd") and abs(r["cmd"][0]) < 0.02 and abs(r["cmd"][1]) > 0.005)
    tilt = max((r.get("tilt") or 0.0) for r in L)
    print(f"{f}: 전체 {len(L) * DT:6.1f} s, 걸음 모드 {walk * DT:5.1f} s, 방향 바로잡기 {turn * DT:5.1f} s, 최대 기울기 {tilt:.1f}°")
