#!/usr/bin/env python3
"""학습 과정 보기 — 학습(train_mjx.py)은 GPU 에서 화면 없이 수천 대를 동시에 돌리므로, 두 창으로 따로 본다.

    python3 watch_training.py plot <학습 폴더>    # 학습 지표 그래프 (10 s 마다 log.json 을 다시 그림)
    python3 watch_training.py play <학습 폴더>    # MuJoCo 뷰어: 가장 최근 중간 정책으로 앞·뒤·옆 걷기를 되풀이
    (학습 폴더 = train_mjx.py --out. 중간 정책은 --ck-min 분마다 ckNNNmin.npz 로 저장됨)

play: 전진 0.6 → 후진 0.4 → 왼쪽 0.3 → 오른쪽 0.3 m/s (각 6 s, 사이 2 s 서기)를 반복하고, 한 바퀴가 끝날 때 새 중간 정책이
있으면 바꿔 끼운다. 화면 왼쪽 위: 정책 파일, 명령, 실제 속도(몸통 기준, 1 s 평균), 발 들기(두 발 높이 차, 관절 각도 정기구학
+ IMU 자세 — 실로봇에서도 같은 방법). 배포와 같은 조건(SF 월드, 물리 2 ms, 상반신 충돌 켜짐)이라 학습 화면이 아니라
'지금 정책을 꺼내 쓰면 이렇게 걷는다' 를 보여 준다.
"""
import glob
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "tron1_openarm_sim"))

# MuJoCo 뷰어 글자는 내장 영문 글꼴이라 한글이 깨진다 → 화면 글자는 영문
SCHEDULE = [("forward 0.6 m/s", (0.6, 0.0, 0.0), 6.0), ("stand", (0, 0, 0), 2.0),
            ("backward 0.4 m/s", (-0.4, 0.0, 0.0), 6.0), ("stand", (0, 0, 0), 2.0),
            ("left 0.3 m/s", (0.0, 0.3, 0.0), 6.0), ("stand", (0, 0, 0), 2.0),
            ("right 0.3 m/s", (0.0, -0.3, 0.0), 6.0), ("stand", (0, 0, 0), 2.0)]


def current_run(path):
    """학습 폴더, 또는 여러 시드 폴더를 담은 상위 폴더 → 지금 학습 중인(가장 최근에 log.json 이 바뀐) 폴더."""
    logs = glob.glob(os.path.join(path, "log.json")) or glob.glob(os.path.join(path, "*", "log.json"))
    return os.path.dirname(max(logs, key=os.path.getmtime)) if logs else path


def latest_policy(run):
    run = current_run(run)
    cks = sorted(glob.glob(os.path.join(run, "ck*min.npz")), key=os.path.getmtime)
    if cks:
        return cks[-1]
    for n in ("best.npz", "final.npz"):
        if os.path.exists(os.path.join(run, n)):
            return os.path.join(run, n)
    return None


def plot(run):
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    keys = [("lin_track", "속도 추종 (1 = 명령대로)"), ("vy_track", "옆 방향 추종 (치우침·옆걸음)"), ("clear", "발 들기 부족 [cm²] (작을수록 높이 듦)"),
            ("fall_rate", "넘어짐 비율"), ("heading", "직진 방향 오차 (3° = 1, 작을수록 일자)"), ("liftoff", "서 있을 때 발 뗀 비율")]
    for f in ("NanumGothic", "Noto Sans CJK KR", "Noto Sans CJK JP", "UnDotum"):
        try:
            matplotlib.font_manager.findfont(f, fallback_to_default=False)
            plt.rcParams["font.family"] = f
            break
        except ValueError:
            continue
    fig, axs = plt.subplots(2, 3, figsize=(13, 7))
    fig.canvas.manager.set_window_title(f"학습 지표 — {run}")
    while plt.fignum_exists(fig.number):
        cur = current_run(run)
        try:
            log = json.load(open(os.path.join(cur, "log.json")))
        except (OSError, ValueError):
            log = []
        if log:
            t = np.array([r["t"] for r in log]) / 60.0
            for ax, (k, title) in zip(axs.ravel(), keys):
                ax.clear()
                y = np.array([r.get(k, np.nan) for r in log], float)
                ax.plot(t, y, lw=1)
                ax.set_title(title, fontsize=10)
                ax.set_xlabel("학습 시간 [분]", fontsize=8)
                ax.grid(alpha=0.3)
            fig.suptitle(f"{os.path.relpath(cur, os.path.dirname(run))}: 반복 {log[-1]['it']}, {log[-1]['t'] / 60:.1f} 분", fontsize=11)
            fig.tight_layout()
        plt.pause(10.0)


def play(run):
    import mujoco
    import mujoco.viewer
    from tron1_openarm_sim.locomotion_policy import LocomotionPolicy
    m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, "src/tron1_openarm_sim/mjcf/sf/world.xml"))
    d = mujoco.MjData(m)
    b = m.body("base_link").id
    feet = [m.body(n).id for n in ("ankle_L_Link", "ankle_R_Link")]
    dec = int(round(0.02 / m.opt.timestep))
    for i in range(m.nu):
        if "finger" in m.actuator(i).name:
            d.ctrl[i] = 0.044
    d.mocap_pos[m.body_mocapid[m.body("person").id]] = [-4.5, -3.5, 0]
    state = {"path": None, "pol": None}

    def load():
        p = latest_policy(run)
        while p is None:
            print(f"{run} 에 아직 중간 정책이 없음 — 기다리는 중", flush=True)
            time.sleep(10)
            p = latest_policy(run)
        if p != state["path"]:
            state["path"], state["pol"] = p, LocomotionPolicy(p)
            print(f"정책: {p}", flush=True)
        pol = state["pol"]
        jq = [m.jnt_qposadr[m.joint(j).id] for j in pol.joints]
        jv = [m.jnt_dofadr[m.joint(j).id] for j in pol.joints]
        act = [m.actuator(j + "_ctrl").id for j in pol.joints]
        mujoco.mj_resetData(m, d)
        d.qpos[0:3] = [0.0, -0.6, 0.76]
        d.qpos[jq] = pol.spec.default_q
        d.ctrl[act] = pol.spec.default_q
        for i in range(m.nu):
            if "finger" in m.actuator(i).name:
                d.ctrl[i] = 0.044
        d.mocap_pos[m.body_mocapid[m.body("person").id]] = [-4.5, -3.5, 0]
        mujoco.mj_forward(m, d)
        return pol, jq, jv, act

    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 3.0, -15, 135
        while v.is_running():
            pol, jq, jv, act = load()
            vel = []
            for name, cmd, sec in SCHEDULE:
                pol.stand_still = all(c == 0 for c in cmd)
                lift = []
                for k in range(int(sec / 0.02)):
                    t0 = time.time()
                    tgt = pol.act(d.sensor("imu_gyro").data, d.sensor("imu_quat").data, d.qpos[jq], d.qvel[jv], cmd)
                    d.ctrl[act] = tgt
                    for _ in range(dec):
                        mujoco.mj_step(m, d)
                    R = d.xmat[b].reshape(3, 3)
                    vel = (vel + [R.T @ d.qvel[0:3]])[-50:]
                    Ri = np.zeros(9)
                    mujoco.mju_quat2Mat(Ri, d.sensor("imu_quat").data)
                    fz = (((d.xpos[feet] - d.xpos[b]) @ R) @ Ri.reshape(3, 3).T)[:, 2]
                    lift = (lift + [abs(fz[0] - fz[1])])[-100:]
                    vm = np.mean(vel, 0)
                    fell = d.xpos[b][2] < 0.45
                    with v.lock():
                        v.cam.lookat[:] = d.xpos[b]
                        v.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                     "policy\ncommand\nvelocity\nfoot lift",
                                     f"{os.path.relpath(state['path'], os.path.dirname(run))}\n{name}\n"
                                     f"fwd {vm[0]:+.2f}  side {vm[1]:+.2f} m/s\n{100 * max(lift):.1f} cm"
                                     + ("\nFELL - restart" if fell else "")))
                    v.sync()
                    if not v.is_running() or fell:
                        break
                    time.sleep(max(0.0, 0.02 - (time.time() - t0)))
                if not v.is_running() or d.xpos[b][2] < 0.45:
                    break


if __name__ == "__main__":
    if len(sys.argv) < 3 or sys.argv[1] not in ("plot", "play"):
        sys.exit(__doc__)
    (plot if sys.argv[1] == "plot" else play)(os.path.abspath(sys.argv[2]))
