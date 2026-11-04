#!/usr/bin/env python3
"""TRON1 + OpenArm 시뮬레이션 자체 점검 (ROS 불필요).

    python3 selftest_tron1.py [world.xml|world_light.xml] [정책.npz]

바퀴형 tools/selftest.py 와 같은 생각: ROS 를 켜기 전에 물리·센서·보행이 맞는지 먼저 확인한다.
"""
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(os.path.dirname(PKG)))
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(REPO, "src", "mobile_openarm_sim"))

import numpy as np  # noqa: E402

from tron1_openarm_sim.biped_core import BipedOpenArmSim  # noqa: E402

WORLD = os.path.join(PKG, "mjcf", sys.argv[1] if len(sys.argv) > 1 else "world_light.xml")
POLICY = sys.argv[2] if len(sys.argv) > 2 else os.path.join(PKG, "policies", "tron1_openarm_policy.npz")
results = []


def check(name, ok, info=""):
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {info}", flush=True)


def run(sim, sec, v=0.0, w=0.0):
    n = int(sec / 0.01)
    for _ in range(n):
        sim.set_cmd_vel(v, w)
        sim.step(5)
        if sim.fallen:
            return False
    return True


t0 = time.time()
sim = BipedOpenArmSim(WORLD, enable_camera=True, move_person=False, policy_path=POLICY, camera_width=960, camera_height=720)
m = sim.m
check("모델 로드", True, f"질량 {m.body_subtreemass[sim.base_id]:.1f} kg, 물리 dt {m.opt.timestep * 1000:.0f} ms, 정책 {os.path.basename(POLICY)}")
sim.set_base_pose(0.0, -0.6, 0.0)
ok = run(sim, 8.0)
h, tilt = sim.base_height_and_tilt()
check("제자리 서기 8 s (명령 0, 제자리 걸음)", ok and 0.62 < h < 0.74 and tilt < 8, f"몸통 높이 {h:.3f} m, 기울기 {tilt:.1f}°")
gt0 = sim.ground_truth_pose()
od0 = sim.odom.copy()
ok = run(sim, 10.0, 0.3, 0.0)
gt1 = sim.ground_truth_pose()
dist = float(np.hypot(*(gt1[:2] - gt0[:2])))
dod = float(np.hypot(*(sim.odom[:2] - od0[:2])))
check("직진 0.3 m/s × 10 s", ok and dist > 2.0, f"실제 이동 {dist:.2f} m, 방향 변화 {math.degrees(gt1[2] - gt0[2]):+.1f}°")
check("다리 오도메트리 거리 오차 < 10%", ok and abs(dod - dist) / max(dist, 1e-3) < 0.10, f"odom {dod:.2f} m vs 정답 {dist:.2f} m")
y0 = gt1[2]
ok = run(sim, 8.0, 0.0, 0.5)
gt2 = sim.ground_truth_pose()
dy = math.atan2(math.sin(gt2[2] - y0), math.cos(gt2[2] - y0))
turned = (gt2[2] - y0)
check("제자리 회전 0.5 rad/s × 8 s", ok, f"회전량 {math.degrees(dy):+.0f}° (목표 +229° → 2π 넘으면 감아서 표시)")
ok = run(sim, 3.0)
check("회전 후 정지", ok)
scan = None
for _ in range(30):
    sim.step(5)
    scan = sim.maybe_scan() or scan
r = scan[2]
check("LiDAR 360 빔", np.isfinite(r).sum() > 300, f"유효 {np.isfinite(r).sum()} / {len(r)}, 높이 {sim.d.site_xpos[sim.lidar_site][2]:.2f} m")
# 작업대 A 앞에서 카메라·ArUco
sim.reset(2.5, 2.45, math.pi / 2)
run(sim, 2.0)
dets = []
for _ in range(60):
    sim.step(5)
    c = sim.maybe_camera()
    if c is not None and len(c[1]) >= 3:
        dets = c[1]
        break
ids = sorted(int(d["id"]) for d in dets)
check("작업대 A 앞에서 ArUco 3개 검출", ids == [0, 1, 2], f"ids {ids}")
if dets:
    # 정답: 상자 윗면 마커 중심(세계) → base_link 좌표.  카메라 렌더·ArUco·깊이 보정·좌표 변환을 한 번에 검증
    Rb = sim.d.xmat[sim.base_id].reshape(3, 3)
    pb = sim.d.xpos[sim.base_id]
    errs = []
    for det in dets:
        bw = sim.d.xpos[sim.m.body(f"box_{det['id']}").id] + np.array([0, 0, 0.025])
        errs.append(float(np.linalg.norm(Rb.T @ (bw - pb) - det["position_base"])))
    check("ArUco 위치 오차 < 2 cm (정답 대비)", max(errs) < 0.02, f"최대 {max(errs) * 100:.1f} cm")
# 팔 동작 중 균형
ok = True
from tron1_openarm_sim.arm_poses import POSES  # noqa: E402  SRDF 와 같은 이족용 이름 자세
for name in ("ready", "carry", "hands_up", "home"):
    q, qr = POSES[name]
    sim.start_arm_trajectory("left", [f"openarm_left_joint{i}" for i in range(1, 8)], [(2.0, q, None)])
    sim.start_arm_trajectory("right", [f"openarm_right_joint{i}" for i in range(1, 8)], [(2.0, qr, None)])
    ok &= run(sim, 2.5)
gtp = sim.ground_truth_pose()
check("팔 자세 바꾸는 동안 서 있기 (ready→carry→hands_up→home)", ok, f"그동안 몸통 이동 {np.hypot(gtp[0] - 2.5, gtp[1] - 2.45):.3f} m")
print(f"총 {len(results)}개 중 실패 {results.count(False)}개 ({time.time() - t0:.0f} s)")
sys.exit(0 if all(results) else 1)
