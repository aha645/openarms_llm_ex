#!/usr/bin/env python3
"""바퀴형 nav2_params.yaml → TRON1(이족) 용으로 바꿔 저장.

    python3 make_nav2_params_tron1.py --src <mobile_openarm_sim>/config/nav2_params.yaml --out ../config/nav2_params_tron1.yaml

바퀴형과 달라지는 것 (이유)
  * 속도 한계  vx 0.40→0.35, 후진 -0.25→-0.20, wz 1.2→0.8 : 보행 정책이 안정적으로 따라가는 범위 (rl/eval_policy.py 결과)
  * 가속 한계  ax 2.0→0.8, az 2.5→1.5 : 보행은 발을 디딜 때만 힘을 낼 수 있어 급가속을 못 따라감
  * 도착 허용  xy 0.12→0.15 m, yaw 0.15→0.20 rad : 제자리 걸음 중에도 몸통이 좌우로 ±2~3 cm 흔들림
  * footprint 0.55x0.52→0.46x0.44 m : TRON1 몸통·다리 + 내린 팔(좌우 ±0.2 m)
  * AMCL 오도메트리 잡음 alpha 0.2→0.4 : 다리 오도메트리는 바퀴보다 미끄러짐·출렁임이 큼
  * 진행 확인  0.3 m / 20 s → 0.25 m / 30 s : 느린 보행 + 회전에 시간이 더 걸림
"""
import argparse

import yaml

FOOTPRINT = "[[0.24, 0.22], [0.24, -0.22], [-0.22, -0.22], [-0.22, 0.22]]"

ap = argparse.ArgumentParser()
ap.add_argument("--src", required=True)
ap.add_argument("--out", required=True)
a = ap.parse_args()
p = yaml.safe_load(open(a.src))

amcl = p["amcl"]["ros__parameters"]
for k in ("alpha1", "alpha2", "alpha3", "alpha4"):
    amcl[k] = 0.4
c = p["controller_server"]["ros__parameters"]
c["progress_checker"]["required_movement_radius"] = 0.25
c["progress_checker"]["movement_time_allowance"] = 30.0
c["general_goal_checker"]["xy_goal_tolerance"] = 0.15
c["general_goal_checker"]["yaw_goal_tolerance"] = 0.20
f = c["FollowPath"]
f.update(vx_max=0.35, vx_min=-0.20, wz_max=0.8, ax_max=0.8, ax_min=-1.0, az_max=1.5, vx_std=0.15, wz_std=0.3)
for cm in ("local_costmap", "global_costmap"):
    cp = p[cm][cm]["ros__parameters"]
    cp["footprint"] = FOOTPRINT
    cp["inflation_layer"]["inflation_radius"] = 0.45
vs = p["velocity_smoother"]["ros__parameters"]
vs["max_velocity"] = [0.35, 0.0, 0.8]
vs["min_velocity"] = [-0.20, 0.0, -0.8]
vs["max_accel"] = [0.8, 0.0, 1.5]
vs["max_decel"] = [-1.0, 0.0, -1.5]
bs = p["behavior_server"]["ros__parameters"]
bs["max_rotational_vel"] = 0.8
with open(a.out, "w") as fp:
    fp.write("# 자동 생성: tools/make_nav2_params_tron1.py (원본: mobile_openarm_sim/config/nav2_params.yaml)\n")
    yaml.safe_dump(p, fp, sort_keys=False)
print("wrote", a.out)
