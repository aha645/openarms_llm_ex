#!/usr/bin/env python3
"""보행 정책 평가 — 학습에 쓰지 않은 '배포 조건'(실습 월드, 물리 2 ms, 상반신 충돌 켜짐)에서 시험.

    python3 eval_policy.py ../src/tron1_openarm_sim/deployed_policies/tron1sf_openarm_policy.npz ../src/tron1_openarm_sim/deployed_policies/tron1_openarm_policy.npz
      (배포 정책 SF·PF 는 src/tron1_openarm_sim/deployed_policies/, LimX 사전학습 정책은 rl/pretrained_limx/, 평가 기록은 rl/eval_records/)

시험 항목 (각 정책, 시드 3개) — 바닥만 남기고 벽·기둥·작업대 충돌은 끔 (긴 걷기 시험이 벽에 막히지 않게)
  1. 제자리 30 s (명령 0)           : 넘어짐, 흐름(drift) 속도 [m/s], 회전 흐름 [rad/s]
  2. 직진 0.3 m/s 20 s              : 실제 전진 속도, 옆 흐름, 진행 방향 오차
  3. 제자리 회전 0.6 rad/s 15 s      : 실제 회전 속도
  4. 팔 동작 중 제자리 (home→ready→carry→hands_up→home, 각 2 s) : 넘어짐, 흐름
  5. 밀기: 제자리에서 2 s 마다 0.3 m/s 옆으로 밀기 10 회 : 넘어짐
  6. 한 팔만 carry (오른팔 carry, 왼팔 home) 제자리 20 s : 흐름 — DAY2 앱은 한 팔씩 움직인다
  7. 빠른 걸음 (v4 학습 범위 확인, 각 15 s): fast 전진 0.6 / back 후진 −0.4 / side_l 왼쪽 0.3 / side_r 오른쪽 −0.3 m/s
  8. walk_arms: 팔을 2.5 s 마다 다른 자세로 바꾸며 직진 0.3 m/s 15 s — 팔을 움직여도 일자로 걷는가 (yaw_deg: 방향 변화 [도])
  9. 느린 미세 이동 (각 6 s): slow_f 전진 0.05 / slow_b 후진 0.05 / slow_s 옆 0.04 m/s — 작업대 앞 위치 맞추기·제자리 복귀에 쓰는 속도
  (서 있는 항목: liftoff = 한 발이라도 접촉력 5 N 아래인 시간 비율 — 발바닥을 붙이고 서 있는가)
  (걷는 항목: lift = 두 발 높이 차이의 95 백분위 [m] — 발을 얼마나 드는가. 관절 각도 정기구학 + IMU 자세로 구함,
   실로봇에서도 같은 방법. track = 명령 방향 실제 속도 / 명령 속도, drift = 명령에 수직인 흐름 / 명령 속도)
  (모든 항목: 몸통 평균 높이 h, 기울기 tilt[도] — 기울면 머리 카메라·팔 좌표가 틀어진다)
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "tron1_openarm_sim"))
from tron1_openarm_sim.locomotion_policy import LocomotionPolicy  # noqa: E402

XML = {"pf": os.path.join(ROOT, "src/tron1_openarm_sim/mjcf/world_light.xml"),
       "sf": os.path.join(ROOT, "src/tron1_openarm_sim/mjcf/sf/world_light.xml")}
SPAWN = {"pf": 0.71, "sf": 0.76}
from tron1_openarm_sim.arm_poses import POSES as _P  # noqa: E402
POSES = {k: v[0] for k, v in _P.items()}            # 왼팔 기준 (arm_to 가 오른팔은 거울상으로)


class Runner:
    def __init__(self, npz, seed):
        self.pol = LocomotionPolicy(npz)
        foot = self.pol.spec.name
        JOINTS = self.pol.joints
        self.m = mujoco.MjModel.from_xml_path(XML[foot])
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.rng = np.random.default_rng(seed)
        self.jq = [m.jnt_qposadr[m.joint(j).id] for j in JOINTS]
        self.jv = [m.jnt_dofadr[m.joint(j).id] for j in JOINTS]
        self.act = [m.actuator(j + "_ctrl").id for j in JOINTS]
        self.arm = {s: [m.actuator(f"{s}_joint{i}_ctrl").id for i in range(1, 8)] for s in ("left", "right")}
        self.b = m.body("base_link").id
        self.feet = [m.body(n).id for n in (("ankle_L_Link", "ankle_R_Link") if foot == "sf" else ("foot_L_Link", "foot_R_Link"))]
        self.dec = int(round(0.02 / m.opt.timestep))
        for i in range(m.nu):
            if "finger" in m.actuator(i).name:
                self.d.ctrl[i] = 0.044
        # 벽·기둥·작업대(세계에 고정된 물체) 충돌을 끈다 — 바닥만 남김. 빠른 걸음(0.6 m/s × 15 s = 9 m)이 실습 월드 벽(x 4.8 m)에
        # 부딪혀 멈춘 시간까지 평균에 들어가 속도가 0.31 m/s 로 낮게 나왔다 (실제로는 0.6 m/s 로 걸음). 물리 조건(2 ms, 상반신 충돌)은 그대로
        for g in range(m.ngeom):
            if m.body_weldid[m.geom_bodyid[g]] == 0 and m.geom(g).name != "floor":
                m.geom_contype[g] = m.geom_conaffinity[g] = 0
        pm = m.body("person").id
        self.person = m.body_mocapid[pm]
        self.d.mocap_pos[self.person] = [-4.5, -3.5, 0]     # 사람은 구석으로 치움
        # 넓은 남쪽 구역(0, -1.0)에서 시작 (기둥·벽과 떨어진 곳)
        self.d.qpos[0:3] = [0.0, -0.6, SPAWN[foot]]
        self.d.qpos[self.jq] = self.pol.spec.default_q
        self.d.ctrl[self.act] = self.pol.spec.default_q
        mujoco.mj_forward(m, self.d)
        self.fell = False

    def arm_to(self, name_l, name_r=None):
        L = _P[name_l][0]
        R = _P[name_r or name_l][1]
        self.d.ctrl[self.arm["left"]] = L
        self.d.ctrl[self.arm["right"]] = R

    def run(self, sec, cmd, push=None):
        d = self.d
        log = []
        # SF: 명령 0 이면 '제자리 서기' (시뮬 브리지와 같은 규칙). PF 는 해당 없음
        self.pol.stand_still = all(c == 0 for c in cmd)
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
            # 발 높이 차이: 몸통 기준 발 위치(관절 각도 정기구학)를 IMU 자세로 돌린 높이의 차이
            qi = d.sensor("imu_quat").data
            Ri = np.zeros(9)
            mujoco.mju_quat2Mat(Ri, qi)
            pb = (d.xpos[self.feet] - d.xpos[self.b]) @ R
            fz = (pb @ Ri.reshape(3, 3).T)[:, 2]
            log.append([d.time, d.qpos[0], d.qpos[1], np.arctan2(R[1, 0], R[0, 0]), v[0], v[1], d.qvel[5], d.xpos[self.b][2],
                        np.degrees(np.arccos(min(1.0, R[2, 2]))), abs(fz[0] - fz[1]),
                        min(d.sensor("foot_L_touch").data[0], d.sensor("foot_R_touch").data[0])])
            if d.xpos[self.b][2] < 0.45:
                self.fell = True
                break
        return np.array(log)


def stats(log, skip=1.0, cmd=None):
    L = log[log[:, 0] > log[0, 0] + skip]
    if len(L) < 5:
        return dict(n=0)
    T = L[-1, 0] - L[0, 0]
    dx, dy = L[-1, 1] - L[0, 1], L[-1, 2] - L[0, 2]
    yaw0 = L[0, 3]
    fwd = dx * np.cos(yaw0) + dy * np.sin(yaw0)
    side = -dx * np.sin(yaw0) + dy * np.cos(yaw0)
    dyaw = np.unwrap(L[:, 3])
    # sway: 그 구간 동안 몸통 위치가 움직인 범위(앞뒤·좌우 중 큰 쪽) [m] — 조작 중 '가만히 서 있는가' 의 지표
    out = dict(v_fwd=fwd / T, v_side=side / T, w=(dyaw[-1] - dyaw[0]) / T, h=float(L[:, 7].mean()),
               h_std=float(L[:, 7].std()), tilt=float(L[:, 8].mean()),
               sway=float(max(np.ptp(L[:, 1]), np.ptp(L[:, 2]))), lift=float(np.percentile(L[:, 9], 95)),
               liftoff=float(np.mean(L[:, 10] < 5.0)), yaw_deg=float(np.degrees(dyaw[-1] - dyaw[0])))
    if cmd is not None and np.hypot(cmd[0], cmd[1]) > 0:
        u = np.array(cmd[:2]) / np.hypot(cmd[0], cmd[1])
        v = np.array([out["v_fwd"], out["v_side"]])
        out["track"] = float(v @ u / np.hypot(cmd[0], cmd[1]))
        out["drift"] = float((u[0] * v[1] - u[1] * v[0]) / np.hypot(cmd[0], cmd[1]))
    return out


def evaluate(npz, seeds=(0, 1, 2)):
    res = {k: [] for k in ("stand", "walk", "turn", "arms", "push", "one_arm", "fast", "back", "side_l", "side_r", "walk_arms",
                           "slow_f", "slow_b", "slow_s")}
    CMDS = {"walk": (0.3, 0, 0), "fast": (0.6, 0, 0), "back": (-0.4, 0, 0), "side_l": (0, 0.3, 0), "side_r": (0, -0.3, 0),
            "walk_arms": (0.3, 0, 0), "slow_f": (0.05, 0, 0), "slow_b": (-0.05, 0, 0), "slow_s": (0, 0.04, 0)}
    falls = {k: 0 for k in res}
    for s in seeds:
        tests = [("stand", lambda r: r.run(30, (0, 0, 0))),
                 ("walk", lambda r: (r.run(2, (0, 0, 0)), r.run(20, (0.3, 0, 0)))[1]),
                 ("turn", lambda r: (r.run(2, (0, 0, 0)), r.run(15, (0, 0, 0.6)))[1]),
                 ("arms", None), ("push", lambda r: (r.run(2, (0, 0, 0)), r.run(20, (0, 0, 0), push=(100, 0.3)))[1]),
                 ("one_arm", lambda r: (r.arm_to("home", "carry"), r.run(3, (0, 0, 0)), r.run(20, (0, 0, 0)))[2])]
        for nm in ("fast", "back", "side_l", "side_r"):
            tests.append((nm, lambda r, c=CMDS[nm]: (r.run(2, (0, 0, 0)), r.run(15, c))[1]))
        tests.append(("walk_arms", None))
        for nm in ("slow_f", "slow_b", "slow_s"):
            tests.append((nm, lambda r, c=CMDS[nm]: (r.run(2, (0, 0, 0)), r.run(6, c))[1]))
        for name, fn in tests:
            r = Runner(npz, s)
            if name == "walk_arms":
                r.run(2, (0, 0, 0))
                logs = []
                for pl, pr in (("ready", "home"), ("carry", "carry"), ("home", "ready"), ("hands_up", "hands_up"), ("carry", "home"), ("home", "home")):
                    if r.fell:
                        break
                    r.arm_to(pl, pr)
                    logs.append(r.run(2.5, (0.3, 0, 0)))
                log = np.concatenate([x for x in logs if len(x)])
            elif name == "arms":
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
                res[name].append(stats(log, cmd=CMDS.get(name)))
    out = {}
    for k, v in res.items():
        v = [x for x in v if x.get("n", 1)]
        agg = {kk: round(float(np.mean([x[kk] for x in v])), 3) for kk in (v[0].keys() if v else [])}
        agg["falls"] = f"{falls[k]}/{len(seeds)}"
        out[k] = agg
    return out


if __name__ == "__main__":
    allres = {}
    # --out=<경로>: 결과 저장 위치 (기본: 저장소에 커밋된 rl/eval_records/eval_results.json 을 덮어씀)
    out_path = os.path.join(ROOT, "rl", "eval_records", "eval_results.json")
    args = []
    for x in sys.argv[1:]:
        if x.startswith("--out="):
            out_path = x.split("=", 1)[1]
        else:
            args.append(x)
    for p in args:
        r = evaluate(p)
        key = os.path.basename(p) if os.path.basename(p) not in ("best.npz", "final.npz") else \
            os.path.basename(os.path.dirname(p)) + "/" + os.path.basename(p)
        allres[key] = r
        print(key)
        for k, v in r.items():
            print(f"  {k:6s} {v}")
    with open(out_path, "w") as fp:
        json.dump(allres, fp, indent=1, ensure_ascii=False)
