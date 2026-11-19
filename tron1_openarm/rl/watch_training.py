#!/usr/bin/env python3
"""학습 과정 보기 — 학습(train_mjx.py)은 GPU 에서 화면 없이 수천 대를 동시에 돌리므로, 두 창으로 따로 본다.

    python3 watch_training.py plot <학습 폴더>    # 학습 지표 그래프 (10 s 마다 log.json 을 다시 그림)
    python3 watch_training.py play <학습 폴더>    # MuJoCo 뷰어: 가장 최근 중간 정책으로 앞·뒤·옆 걷기를 되풀이 (가벼운 모델)
    python3 watch_training.py play <학습 폴더> --full   # 전체 메시 모델 (학습이 GPU 를 다 쓰는 동안에는 느림)
    (학습 폴더 = train_mjx.py --out. 중간 정책은 --ck-min 분마다 ckNNNmin.npz 로 저장됨)

play: (OpenCV 창, q 로 끝내기) 전진 0.6 → 후진 0.4 → 왼쪽 0.3 → 오른쪽 0.3 m/s (각 6 s, 사이 2 s 서기)를 반복하고, 한 바퀴가 끝날 때 새 중간 정책이
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
    keys = [("lin_track", "속도 추종 (1 = 명령대로)"), ("side_vy", "옆걸음 명령일 때 옆 추종"), ("sway", "서 있을 때 몸통 밀림 (3 cm = 1)"),
            ("fall_rate", "넘어짐 비율"), ("head_straight", "직진할 때 방향 오차 (3° = 1)"), ("slow_track", "느린 미세 이동 추종 (1 = 명령대로)")]
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


VIEW_HZ = 10.0     # 뷰어 갱신 [Hz]. 매 정책 스텝(50 Hz)마다 2560x1440 전체 메시를 그렸더니, 학습이 같은 GPU 를 98 % 쓰는 동안
                   # 화면 그리기가 밀려 시뮬이 거의 멈추고 창이 '응답 없음' 이 되었다 (물리만은 실시간의 4.6 배)


def play(run, full=False):
    """화면 밖 렌더러(mujoco.Renderer)로 그려 OpenCV 창에 띄운다. MuJoCo 뷰어(launch_passive)는 그리기 스레드와 잠금을 따로 써서,
    학습이 GPU 를 다 쓰는 동안 그리기 스레드가 잠금을 기다리며 멈추고 시뮬도 같이 멈췄다 (두 번 모두 '왼쪽 옆걸음' 중, 시뮬 시간 고정)."""
    import cv2
    import mujoco
    from tron1_openarm_sim.locomotion_policy import LocomotionPolicy
    m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, "src/tron1_openarm_sim/mjcf/sf", "world.xml" if full else "world_light.xml"))
    d = mujoco.MjData(m)
    b = m.body("base_link").id
    feet = [m.body(n).id for n in ("ankle_L_Link", "ankle_R_Link")]
    dec = int(round(0.02 / m.opt.timestep))
    W, H = min(640, m.vis.global_.offwidth), min(480, m.vis.global_.offheight)
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 3.0, -15, 135
    win = "TRON1 학습 중 정책 보행 (q: 끝내기)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 960, 720)
    state = {"path": None, "pol": None}

    def load():
        p = latest_policy(run)
        while p is None:
            print(f"{run} 에 아직 중간 정책이 없음 — 기다리는 중", flush=True)
            if cv2.waitKey(10000) & 0xFF == ord("q"):
                sys.exit(0)
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

    last_view, rt0, st0 = 0.0, time.time(), 0.0
    while True:
        pol, jq, jv, act = load()
        vel = []
        for name, cmd, sec in SCHEDULE:
            pol.stand_still = all(c == 0 for c in cmd)
            lift = []
            for k in range(int(sec / 0.02)):
                t0 = time.time()
                d.ctrl[act] = pol.act(d.sensor("imu_gyro").data, d.sensor("imu_quat").data, d.qpos[jq], d.qvel[jv], cmd)
                for _ in range(dec):
                    mujoco.mj_step(m, d)
                R = d.xmat[b].reshape(3, 3)
                vel = (vel + [R.T @ d.qvel[0:3]])[-50:]
                Ri = np.zeros(9)
                mujoco.mju_quat2Mat(Ri, d.sensor("imu_quat").data)
                fz = (((d.xpos[feet] - d.xpos[b]) @ R) @ Ri.reshape(3, 3).T)[:, 2]
                lift = (lift + [abs(fz[0] - fz[1])])[-100:]
                fell = d.xpos[b][2] < 0.45
                if time.time() - last_view >= 1.0 / VIEW_HZ or fell:
                    vm = np.mean(vel, 0)
                    rtf = (d.time - st0) / max(time.time() - rt0, 1e-3)
                    last_view, rt0, st0 = time.time(), time.time(), d.time
                    cam.lookat[:] = d.xpos[b]
                    r.update_scene(d, camera=cam)
                    img = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
                    img = cv2.resize(img, (960, 720))
                    lines = [f"policy  {os.path.relpath(state['path'], os.path.dirname(run))}", f"command {name}",
                             f"velocity fwd {vm[0]:+.2f}  side {vm[1]:+.2f} m/s", f"foot lift {100 * max(lift):.1f} cm",
                             f"sim time {d.time:.1f} s  (x{rtf:.2f} real time)"] + (["FELL - restart"] if fell else [])
                    for i, t in enumerate(lines):
                        cv2.putText(img, t, (12, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
                        cv2.putText(img, t, (12, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.imshow(win, img)
                    if (cv2.waitKey(1) & 0xFF) == ord("q") or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                        r.close()
                        cv2.destroyAllWindows()
                        return
                    if fell:
                        break
                time.sleep(max(0.0, 0.02 - (time.time() - t0)))
            if d.xpos[b][2] < 0.45:
                break


if __name__ == "__main__":
    if len(sys.argv) < 3 or sys.argv[1] not in ("plot", "play"):
        sys.exit(__doc__)
    if sys.argv[1] == "plot":
        plot(os.path.abspath(sys.argv[2]))
    else:
        play(os.path.abspath(sys.argv[2]), full="--full" in sys.argv)
