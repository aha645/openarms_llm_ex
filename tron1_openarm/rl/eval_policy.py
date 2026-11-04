#!/usr/bin/env python3
"""보행 정책 평가 — 학습에 쓰지 않은 '배포 조건'(실습 월드, 물리 2 ms, 상반신 충돌 켜짐)에서 시험.

    python3 eval_policy.py ../policies/limx_pf_tron1a_pretrained.npz ../policies/tron1_openarm_policy.npz

시험 항목 (각 정책, 시드 3개)
  1. 제자리 30 s (명령 0)           : 넘어짐, 흐름(drift) 속도 [m/s], 회전 흐름 [rad/s]
  2. 직진 0.3 m/s 20 s              : 실제 전진 속도, 옆 흐름, 진행 방향 오차
  3. 제자리 회전 0.6 rad/s 15 s      : 실제 회전 속도
  4. 팔 동작 중 제자리 (home→ready→carry→hands_up→home, 각 2 s) : 넘어짐, 흐름
  5. 밀기: 제자리에서 2 s 마다 0.3 m/s 옆으로 밀기 10 회 : 넘어짐
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "tron1_openarm_sim"))
from tron1_openarm_sim.locomotion_policy import JOINTS, LocomotionPolicy  # noqa: E402

XML = os.path.join(ROOT, "src/tron1_openarm_sim/mjcf/world_light.xml")
POSES = {"home": [0] * 7, "ready": [0, 0, 0, 1.57, 0, 0, 0], "carry": [-0.3, 0, 1.2, 2.0, 0, 0, 0],
         "hands_up": [0, 0, 0, 2.0, 0, 0, 0]}


class Runner:
    def __init__(self, npz, seed):
        self.m = mujoco.MjModel.from_xml_path(XML)
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.rng = np.random.default_rng(seed)
        self.jq = [m.jnt_qposadr[m.joint(j).id] for j in JOINTS]
        self.jv = [m.jnt_dofadr[m.joint(j).id] for j in JOINTS]
        self.act = [m.actuator(j + "_ctrl").id for j in JOINTS]
        self.arm = {s: [m.actuator(f"{s}_joint{i}_ctrl").id for i in range(1, 8)] for s in ("left", "right")}
        self.b = m.body("base_link").id
        self.dec = int(round(0.02 / m.opt.timestep))
        for i in range(m.nu):
            if "finger" in m.actuator(i).name:
                self.d.ctrl[i] = 0.044
        pm = m.body("person").id
        self.person = m.body_mocapid[pm]
        self.d.mocap_pos[self.person] = [-4.5, -3.5, 0]     # 사람은 구석으로 치움
        # 넓은 남쪽 구역(0, -1.0)에서 시작 (기둥·벽과 떨어진 곳)
        self.d.qpos[0:3] = [0.0, -0.6, 0.71]
        mujoco.mj_forward(m, self.d)
        self.pol = LocomotionPolicy(npz)
        self.fell = False

    def arm_to(self, name_l, name_r=None):
        L = POSES[name_l]
        R = list(POSES[name_r or name_l])
        if (name_r or name_l) == "carry":
            R = [0.3, 0, -1.2, 2.0, 0, 0, 0]
        self.d.ctrl[self.arm["left"]] = L
        self.d.ctrl[self.arm["right"]] = R

    def run(self, sec, cmd, push=None):
        d = self.d
        log = []
        for k in range(int(sec / 0.02)):
            t = self.pol.act(d.sensor("imu_gyro").data, d.sensor("imu_quat").data, d.qpos[self.jq], d.qvel[self.jv], cmd)
            d.ctrl[self.act] = t
            if push and k % push[0] == push[0] - 1:
                R = d.xmat[self.b].reshape(3, 3)
                d.qvel[0:3] += R @ np.array([0, push[1] * self.rng.choice([-1, 1]), 0])
            for _ in range(self.dec):
                mujoco.mj_step(self.m, d)
            R = d.xmat[self.b].reshape(3, 3)
            v = R.T @ d.qvel[0:3]
            log.append([d.time, d.qpos[0], d.qpos[1], np.arctan2(R[1, 0], R[0, 0]), v[0], v[1], d.qvel[5], d.xpos[self.b][2]])
            if d.xpos[self.b][2] < 0.45:
                self.fell = True
                break
        return np.array(log)


def stats(log, skip=1.0):
    L = log[log[:, 0] > log[0, 0] + skip]
    if len(L) < 5:
        return dict(n=0)
    T = L[-1, 0] - L[0, 0]
    dx, dy = L[-1, 1] - L[0, 1], L[-1, 2] - L[0, 2]
    yaw0 = L[0, 3]
    fwd = dx * np.cos(yaw0) + dy * np.sin(yaw0)
    side = -dx * np.sin(yaw0) + dy * np.cos(yaw0)
    dyaw = np.unwrap(L[:, 3])
    return dict(v_fwd=fwd / T, v_side=side / T, w=(dyaw[-1] - dyaw[0]) / T, h=float(L[:, 7].mean()),
                h_std=float(L[:, 7].std()))


def evaluate(npz, seeds=(0, 1, 2)):
    res = {k: [] for k in ("stand", "walk", "turn", "arms", "push")}
    falls = {k: 0 for k in res}
    for s in seeds:
        tests = [("stand", lambda r: r.run(30, (0, 0, 0))),
                 ("walk", lambda r: (r.run(2, (0, 0, 0)), r.run(20, (0.3, 0, 0)))[1]),
                 ("turn", lambda r: (r.run(2, (0, 0, 0)), r.run(15, (0, 0, 0.6)))[1]),
                 ("arms", None), ("push", lambda r: (r.run(2, (0, 0, 0)), r.run(20, (0, 0, 0), push=(100, 0.3)))[1])]
        for name, fn in tests:
            r = Runner(npz, s)
            if name == "arms":
                logs = [r.run(2, (0, 0, 0))]
                for p in ("ready", "carry", "hands_up", "home", "carry", "home"):
                    if r.fell:
                        break
                    r.arm_to(p)
                    logs.append(r.run(2.5, (0, 0, 0)))
                log = np.concatenate([x for x in logs if len(x)])
            else:
                log = fn(r)
            falls[name] += int(r.fell)
            if not r.fell:
                res[name].append(stats(log))
    out = {}
    for k, v in res.items():
        v = [x for x in v if x.get("n", 1)]
        agg = {kk: round(float(np.mean([x[kk] for x in v])), 3) for kk in (v[0].keys() if v else [])}
        agg["falls"] = f"{falls[k]}/{len(seeds)}"
        out[k] = agg
    return out


if __name__ == "__main__":
    allres = {}
    for p in sys.argv[1:]:
        r = evaluate(p)
        allres[os.path.basename(p)] = r
        print(os.path.basename(p))
        for k, v in r.items():
            print(f"  {k:6s} {v}")
    with open(os.path.join(ROOT, "policies", "eval_results.json"), "w") as fp:
        json.dump(allres, fp, indent=1, ensure_ascii=False)
