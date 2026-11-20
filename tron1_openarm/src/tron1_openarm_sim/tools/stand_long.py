"""SF 오래 서 있기 시험 — 상자를 쥔 팔(carry, 상자 무게)로 몇 분 서 있을 때 중심이 시간에 따라 무너지는지 잰다.
    python3 tools/stand_long.py <world_light.xml> <정책.npz> [초, 기본 180] [raw]

집기 한 번에 작업대 앞에서 3~5 분 서 있는데, 학습 에피소드는 20 s (서기 구간 10~20 s) 였다.
무작위 배치 r6: 상자를 든 채 서 있는 동안 기울기 4 → 9.6°, 몸통 방향이 천천히 돌다가 걸음이 시작돼 작업대에서 0.5 m 벗어났다.
raw: 서기 중 방향 바로잡기 걸음(브리지)을 끄고 정책만 본다.

출력: 10 s 마다 (시간, 서기 여부, 몸통 방향 변화°, 기울기°, 몸통 이동 cm, 발 이동 cm), 끝에 요약 한 줄(JSON)
"""
import json
import math
import sys

import numpy as np

sys.path.insert(0, '.')
from tron1_openarm_sim.biped_core import BipedOpenArmSim  # noqa: E402
from tron1_openarm_sim.arm_poses import POSES  # noqa: E402
from tron1_openarm_sim.sim_core import yaw_of  # noqa: E402


def tilt_deg(q):
    w, x, y, z = q
    upz = 1 - 2 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, upz))))


def run(world, policy, T, side, mass, raw):
    sim = BipedOpenArmSim(world, enable_camera=False, move_person=False, policy_path=policy)
    sim.stand_heading_hold = not raw
    m, d = sim.m, sim.d
    sim.set_base_pose(0, -0.6, 0)
    names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    hand = m.body(f"openarm_{side}_hand").id
    feet = [m.body(n).id for n in ("ankle_L_Link", "ankle_R_Link")]
    w_box = np.array([0, 0, -9.81 * mass])

    def go(sec, force=None):
        for _ in range(int(sec / 0.01)):
            sim.set_cmd_vel(0, 0)
            if force is not None:
                d.xfrc_applied[hand, :3] = force
            sim.step(5)
            if force is not None:
                d.xfrc_applied[hand, :3] = 0

    go(6.0)
    sim.start_arm_trajectory(side, names, [(3.0, list(POSES["carry"][0 if side == "left" else 1]), None)])
    go(5.0, w_box)
    y0, b0 = yaw_of(d.xquat[sim.base_id]), d.xpos[sim.base_id][:2].copy()
    f0 = np.array([d.xpos[f][:2] for f in feet])
    rows, steps, prev = [], 0, bool(sim.policy.standing)
    for k in range(int(T / 0.01)):
        go(0.01, w_box)
        s = bool(sim.policy.standing)
        if prev and not s:
            steps += 1
        prev = s
        if (k + 1) % 1000 == 0:
            yaw = math.degrees(yaw_of(d.xquat[sim.base_id]) - y0)
            fp = np.array([d.xpos[f][:2] for f in feet])
            rows.append(dict(t=round((k + 1) * 0.01), st=s, yaw=round((yaw + 180) % 360 - 180, 1),
                             tilt=round(tilt_deg(d.xquat[sim.base_id]), 1),
                             base_cm=round(100 * float(np.linalg.norm(d.xpos[sim.base_id][:2] - b0)), 1),
                             foot_cm=round(100 * float(np.max(np.linalg.norm(fp - f0, axis=1))), 1), steps=steps))
            print(json.dumps(rows[-1]), flush=True)
            if sim.fallen:
                break
    return dict(summary=True, side=side, mass=mass, raw=raw, T=T, steps=steps, fallen=bool(sim.fallen),
                yaw_end=rows[-1]["yaw"], tilt_first=rows[0]["tilt"], tilt_max=max(r["tilt"] for r in rows),
                base_cm_end=rows[-1]["base_cm"], foot_cm_end=rows[-1]["foot_cm"])


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    args = [a for a in sys.argv[3:] if a != "raw"]
    T = float(args[0]) if args else 180.0
    raw = "raw" in sys.argv[3:]
    for side, mass in [(s, 0.4) for s in __import__("os").environ.get("SIDES", "left,right").split(",")]:
        print(json.dumps(run(sys.argv[1], sys.argv[2], T, side, mass, raw), ensure_ascii=False), flush=True)
