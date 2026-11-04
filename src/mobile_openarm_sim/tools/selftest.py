#!/usr/bin/env python3
"""ROS 없이 시뮬레이션 핵심 기능을 검증한다 (설치 직후 1차 점검용).

    cd ~/openarm_mobile_ws/src/mobile_openarm_sim
    MUJOCO_GL=egl python3 tools/selftest.py        # GPU/EGL
    MUJOCO_GL=osmesa python3 tools/selftest.py     # GPU 없이 소프트웨어 렌더
    python3 tools/selftest.py world_light.xml       # 저사양 모드 월드

검사 항목: 모델 로드 / 실시간 배속 / 직진·회전 오도메트리 / LiDAR / ArUco 포즈 오차 / IK + 집기
"""
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from mobile_openarm_sim.sim_core import MobileOpenArmSim  # noqa: E402
from mobile_openarm_sim.kinematics import ArmIK, approach_rotation  # noqa: E402

XML = os.path.join(HERE, "..", "mjcf", sys.argv[1] if len(sys.argv) > 1 else "world.xml")
results = []


def check(name, ok, info=""):
    results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {info}")


def run(sim, sec):
    sim.step(int(sec / sim.m.opt.timestep))


def move_arm(sim, side, q, dur=2.0):
    names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    sim.start_arm_trajectory(side, names, [(dur, list(q), None)])
    run(sim, dur + 0.5)


def grip(sim, side, val, dur=1.0):
    sim.start_gripper_trajectory(side, [(dur, [val], None)])
    run(sim, dur + 0.5)


def main():
    sim = MobileOpenArmSim(XML, move_person=False, lidar_noise=0.0)
    check("모델 로드", True, f"nq={sim.m.nq} nu={sim.m.nu}")

    t = time.time()
    run(sim, 2.0)
    rtf = 2.0 / (time.time() - t)
    check("실시간 배속(정지 상태)", rtf > 1.0, f"RTF={rtf:.2f}x")

    # 직진 1.0 m
    gt0 = sim.ground_truth_pose()
    for _ in range(int(2.5 / 0.01)):
        sim.set_cmd_vel(0.4, 0.0)
        sim.step(5)
    sim.set_cmd_vel(0, 0)
    run(sim, 1.0)
    gt1 = sim.ground_truth_pose()
    dist = np.linalg.norm(gt1[:2] - gt0[:2])
    check("직진 주행", 0.8 < dist < 1.2, f"이동 {dist:.3f} m, odom x={sim.odom[0]:.3f}")
    check("휠 오도메트리 오차", abs(sim.odom[0] - dist) < 0.05, f"|odom-gt|={abs(sim.odom[0]-dist):.3f} m")

    # 제자리 회전 90도
    for _ in range(int(math.pi / 2 / 0.6 / 0.01)):
        sim.set_cmd_vel(0.0, 0.6)
        sim.step(5)
    sim.set_cmd_vel(0, 0)
    run(sim, 1.0)
    dyaw = math.degrees(sim.ground_truth_pose()[2] - gt1[2])
    check("제자리 회전", 60 < dyaw < 120, f"{dyaw:.1f} deg, odom yaw={math.degrees(sim.odom[2]):.1f}")

    # LiDAR: 원점 부근에서 남쪽 벽까지 거리
    sim.set_base_pose(0.0, 0.0, 0.0)
    run(sim, 0.2)
    sim._last_lidar_t = -1e9
    a0, inc, r = sim.maybe_scan()
    finite = np.isfinite(r).sum()
    i_s = int(round((-math.pi / 2 - a0) / inc))   # 오른쪽(-y) 방향 = 남쪽 벽
    check("LiDAR", finite > 300, f"유효 {finite}/{len(r)}  남쪽 거리 {r[i_s]:.2f} m (기대≈3.10: 남쪽 crate 앞면)")

    # ArUco: 작업대 앞
    sim.set_base_pose(2.5, 2.45, math.pi / 2)
    run(sim, 0.5)
    if sim.enable_camera:
        sim._last_cam_t = -1e9
        rgb, dets = sim.maybe_camera()
        ids = sorted(d["id"] for d in dets)
        check("ArUco 검출", ids == [0, 1, 2], f"ids={ids}")
        Rb = sim.d.xmat[sim.base_id].reshape(3, 3)
        pb = sim.d.xpos[sim.base_id]
        for det in dets:
            gt = sim.d.body(f"box_{det['id']}").xpos + np.array([0, 0, 0.025])
            est = Rb @ det["position_base"] + pb
            err = np.linalg.norm(est - gt)
            check(f"ArUco {det['id']} 위치오차", err < 0.02, f"{err*1000:.1f} mm")
        try:
            import cv2
            from mobile_openarm_sim.aruco_utils import ArucoDetector
            cv2.imwrite(os.path.join(HERE, "selftest_camera.png"), ArucoDetector.draw(rgb, dets))
        except Exception:
            pass

    # IK + 집기 (오른팔, 45도 숙인 접근: TCP z축 = 접근 방향, z축 회전 자유)
    ik = ArmIK(sim.m, "right")
    box = sim.d.body("box_0").xpos.copy()
    fwd = sim.d.xmat[sim.base_id].reshape(3, 3)[:, 0]
    R_grasp = approach_rotation(fwd, 45)
    appr = R_grasp[:, 2]
    pre = box - appr * 0.10
    q_pre, ok1 = ik.solve_multi(sim.d, pre, R_grasp, q0=[0.3, 0, 0, 1.5, 0, 0, 0], axis_only=True)
    q_grasp, ok2 = ik.solve(sim.d, box, R_grasp, q0=q_pre, axis_only=True)
    check("IK (접근/파지 자세)", ok1 and ok2, f"q_pre={np.round(q_pre, 2)}")
    grip(sim, "right", 0.044)
    move_arm(sim, "right", q_pre, 3.0)
    move_arm(sim, "right", q_grasp, 2.0)
    grip(sim, "right", 0.0, 1.0)
    lift = box + np.array([0, 0, 0.12]) - appr * 0.03
    q_lift, _ = ik.solve(sim.d, lift, R_grasp, q0=q_grasp, axis_only=True)
    move_arm(sim, "right", q_lift, 2.0)
    z = sim.d.body("box_0").xpos[2]
    check("집어 들기", z > box[2] + 0.06, f"상자 높이 {box[2]:.3f} → {z:.3f}, attached={sim.attached_object('right')}")

    # 들고 이동 후 내려놓기
    for _ in range(int(1.5 / 0.01)):
        sim.set_cmd_vel(-0.3, 0.0)
        sim.step(5)
    sim.set_cmd_vel(0, 0)
    run(sim, 1.0)
    check("파지 유지(후진 중)", sim.attached_object("right") == "box_0")
    grip(sim, "right", 0.044)
    run(sim, 1.0)
    check("놓기", sim.attached_object("right") is None, f"상자 z={sim.d.body('box_0').xpos[2]:.3f}")

    n_fail = sum(1 for _, ok in results if not ok)
    print(f"\n총 {len(results)}개 중 실패 {n_fail}개")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
