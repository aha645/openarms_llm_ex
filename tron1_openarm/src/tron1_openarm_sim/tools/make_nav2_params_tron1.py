#!/usr/bin/env python3
"""바퀴형 nav2_params.yaml → TRON1(이족) 용으로 바꿔 저장.

    python3 make_nav2_params_tron1.py --src <mobile_openarm_sim>/config/nav2_params.yaml --out ../config/nav2_params_tron1.yaml

바퀴형과 달라지는 것 (이유)
  * 속도 한계  vx 0.40→0.35, 후진 -0.25→-0.20, wz 1.2→0.8 : 보행 정책이 안정적으로 따라가는 범위 (rl/eval_policy.py 결과)
  * 가속 한계  ax 2.0→0.8, az 2.5→1.5 : 보행은 발을 디딜 때만 힘을 낼 수 있어 급가속을 못 따라감
  * 도착 허용  xy 0.12→0.15 m, yaw 0.15→0.20 rad : 제자리 걸음 중에도 몸통이 좌우로 ±2~3 cm 흔들림
  * footprint 0.55x0.52→0.46x0.64 m : TRON1 몸통·다리 + 바깥으로 벌려 내린 팔(좌우 ±0.31 m)
  * AMCL 오도메트리 잡음 alpha 0.2→0.4 : 다리 오도메트리는 바퀴보다 미끄러짐·출렁임이 큼
  * 진행 확인  0.3 m / 20 s → 0.25 m / 30 s : 느린 보행 + 회전에 시간이 더 걸림
"""
import argparse

import yaml

FOOTPRINT = "[[0.24, 0.32], [0.24, -0.32], [-0.22, -0.32], [-0.22, 0.32]]"   # 팔을 벌린 home 자세(손 y ±0.31 m) 포함

ap = argparse.ArgumentParser()
ap.add_argument("--src", required=True)
ap.add_argument("--out", required=True)
a = ap.parse_args()
p = yaml.safe_load(open(a.src))

amcl = p["amcl"]["ros__parameters"]
# 이족 오도메트리: 방향(yaw)은 IMU 라 정확하지만 이동 거리는 바퀴보다 부정확하다 (SF 곡선 주행에서 20 % 넘게 많이 읽음).
# 넷 다 0.8 로 올렸더니(미션 시험 6·7) 회전 잡음까지 커져 라이다 매칭이 방향을 11~40° 잘못 바꾸다 위치를 잃었다 →
# 회전→회전(alpha1)은 믿고, 이동→이동(alpha3)만 덜 믿는다. 라이다로 더 자주 고친다 (0.1 m / 0.1 rad 마다)
amcl["alpha1"] = 0.1      # 회전이 회전에 주는 잡음
amcl["alpha2"] = 0.2      # 이동이 회전에 주는 잡음
amcl["alpha3"] = 0.8      # 이동이 이동에 주는 잡음
amcl["alpha4"] = 0.3      # 회전이 이동에 주는 잡음
amcl["update_min_d"] = 0.1
amcl["update_min_a"] = 0.1
c = p["controller_server"]["ros__parameters"]
c["progress_checker"]["required_movement_radius"] = 0.25
c["progress_checker"]["movement_time_allowance"] = 60.0   # 이족은 회전·출발이 느려 30 s 안에 0.25 m 를 못 가는 경우가 있었다
# 충돌 감시기(collision_monitor)의 '접근 감속'(1.2 s 앞 충돌 예측) 끄기: 반응이 느린 이족은 작업대 근처에서 계속 감속·정지되어
# '진행 못 함' 5번 뒤 25 분 동안 명령 0 으로 멈췄다 (SF 미션 시험 13). 장애물 회피는 비용 지도 + MPPI 가 계속 맡는다
cm = p["collision_monitor"]["ros__parameters"]
cm["FootprintApproach"]["enabled"] = False
bsp = p["behavior_server"]["ros__parameters"]
if "spin" in bsp and isinstance(bsp["spin"], dict):
    bsp["spin"]["time_allowance"] = 30.0
# 목표 근처의 아주 작은 명령에서 이족은 방향이 ±25° 출렁여 0.15 m / 0.2 rad 안에 못 들어갔다 (SF 미션 시험 2, 600 s 시간 초과).
# 도착 뒤에는 도킹(yaw 보정 + 카메라로 상자 거리 보정)이 위치를 다시 맞추므로 목표 허용 범위를 넓힌다
c["general_goal_checker"]["xy_goal_tolerance"] = 0.25
c["general_goal_checker"]["yaw_goal_tolerance"] = 0.35
f = c["FollowPath"]
# wz_max 0.8 → 0.5: SF 는 회전 응답이 늦어 큰 회전 명령에서 방향이 ±60° 출렁이며 경로를 크게 돌아갔다 (SF 미션 시험 10)
f.update(vx_max=0.35, vx_min=-0.20, wz_max=0.5, ax_max=0.8, ax_min=-1.0, az_max=1.0, vx_std=0.15, wz_std=0.2)
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
