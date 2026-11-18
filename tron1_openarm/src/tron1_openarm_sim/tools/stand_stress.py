"""SF 서기 버티기 시험 — 집기 동작을 서 있는 채로 재현해 '발을 디딘 채 버티는지' 잰다 (오프라인 MuJoCo, 배포 제어기 그대로).
    python3 tools/stand_stress.py <world_light.xml> <정책.npz> [seed 개수, 기본 3] [raw]

시나리오 (양팔 × 상자 옆 거리 2 가지 = 4 개, 각 seed 마다 손 반력 크기를 조금씩 바꿈):
  명령 0 으로 서기 → carry → 접근점(상자 10 cm 앞) → 파지 위치 → 손 반력(상자를 밀고 쥘 때, 0.4 s)
  → 상자 무게 → 들어 올림 → 느리게 carry (집기 앱 HOLD_VEL) → 팔 내림
  팔 관절각은 집기 앱과 같은 접근 자세(pitch 20°, roll −90°)를 MuJoCo 정기구학으로 풀어 만든다.
raw: 서기 중 방향 바로잡기 걸음(STAND_YAW_TOL)을 끄고 정책만 본다.

지표
  steps   : 서기 모드가 풀린 횟수 (= 디딤 걸음을 시작한 횟수). 이유별: yaw(방향 바로잡기), unstable(한 발 하중·흔들림)
  foot_cm : 두 발이 처음 디딘 자리에서 움직인 최대 거리
  yaw_deg : 몸통 방향 변화, base_cm: 몸통 위치 변화, fallen
"""
import json
import math
import sys

import mujoco
import numpy as np

sys.path.insert(0, '.'); sys.path.insert(0, '../../../mobile_openarm/src/mobile_openarm_sim')
sys.path.insert(0, '../../../mobile_openarm/src/mobile_openarm_apps')
from tron1_openarm_sim.biped_core import BipedOpenArmSim  # noqa: E402
from tron1_openarm_sim.arm_poses import POSES  # noqa: E402
from mobile_openarm_apps.geometry import approach_rotation  # noqa: E402
from mobile_openarm_sim.sim_core import yaw_of  # noqa: E402

FWD = 0.53                  # 상자 중심 (base_footprint 앞 거리, 집기 앱 IDEAL_REACH 근처)
BOX_Z = 0.725
PRESS_N = float(__import__("os").environ.get("PRESS_N", 30.0))   # 손 누름 힘 최대 [N]


# 오른팔 접근·파지 자세 예 (stand_drift.py) — 왼팔은 거울상 (arm_poses.mirror)
PRE_R = [0.25, 0.35, -0.5, 1.45, 0.44, -0.41, -0.73]
GRASP_R = [0.44, 0.2, -0.48, 0.96, 0.45, -0.26, -0.5]


def solve_ik(sim, side, pos_b, R_b, q0, iters=300, restarts=40):
    """여러 시작점(q0 근처 무작위, 무작위)에서 푼 해 중 q0 에 가장 가까운 것.
    MoveIt 은 지금 자세(carry)에서 시작해 풀므로 그 근처 팔 모양이 실제 집기 자세다. 첫 해를 쓰면 관절 2 가 한계(2.9 rad)
    근처인 다른 가지가 나왔다."""
    names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    lo, hi = sim.m.jnt_range[[sim.m.joint(n).id for n in names]].T
    rng = np.random.default_rng(0)
    q0 = np.asarray(q0, float)
    starts = [q0] + [np.clip(q0 + rng.normal(0, 0.5, 7), lo, hi) for _ in range(restarts)] + \
             [rng.uniform(lo, hi) for _ in range(restarts // 2)]
    sols = [r for r in (_solve_ik(sim, side, pos_b, R_b, q, iters) for q in starts) if r is not None]
    return min(sols, key=lambda q: float(np.linalg.norm(q - q0))) if sols else None


def _solve_ik(sim, side, pos_b, R_b, q0, iters=300):
    """손끝(hand_tcp)을 base 기준 pos_b·R_b 로 — 감쇠 최소제곱 (MuJoCo 자코비안). 실패하면 None."""
    m = sim.m
    d = mujoco.MjData(m)
    d.qpos[:] = sim.d.qpos
    names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    qa = [m.jnt_qposadr[m.joint(n).id] for n in names]
    va = [m.jnt_dofadr[m.joint(n).id] for n in names]
    lo, hi = m.jnt_range[[m.joint(n).id for n in names]].T
    d.qpos[qa] = q0
    tcp = m.body(f"openarm_{side}_hand_tcp").id
    base = m.body("base_link").id
    for _ in range(iters):
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        Rb = d.xmat[base].reshape(3, 3)
        pb = d.xpos[base] * np.array([1, 1, 0])                         # base_footprint ≈ 몸통 아래 바닥 (기울기 무시)
        p_w = pb + Rb @ pos_b
        R_w = Rb @ R_b
        e_p = p_w - d.xpos[tcp]
        Rc = d.xmat[tcp].reshape(3, 3)
        e_r = 0.5 * sum(np.cross(Rc[:, k], R_w[:, k]) for k in range(3))
        if np.linalg.norm(e_p) < 1e-3 and np.linalg.norm(e_r) < 1e-2:
            return d.qpos[qa].copy()
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jacBody(m, d, jp, jr, tcp)
        J = np.vstack([jp[:, va], jr[:, va]])
        e = np.concatenate([e_p, 0.5 * e_r])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), e)
        d.qpos[qa] = np.clip(d.qpos[qa] + dq, lo, hi)
    return None


def run(world, policy, side, lat, seed, raw=False):
    rng = np.random.default_rng(seed)
    sim = BipedOpenArmSim(world, enable_camera=False, move_person=False, policy_path=policy)
    sim.stand_heading_hold = not raw
    m, d = sim.m, sim.d
    sim.set_base_pose(0, -0.6, 0)
    names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    sgn = -1.0 if side == "right" else 1.0
    carry = POSES["carry"][0 if side == "left" else 1]
    R = approach_rotation(20.0, -90.0)
    box = np.array([FWD, sgn * lat, BOX_Z])
    q_pre = solve_ik(sim, side, box - R[:, 2] * 0.10 + [0, 0, 0.07], R, carry)
    q_grasp = solve_ik(sim, side, box + R[:, 2] * 0.01, R, q_pre if q_pre is not None else carry)
    if q_pre is None or q_grasp is None:
        return dict(side=side, lat=lat, seed=seed, error="IK 실패")
    q_lift = solve_ik(sim, side, box + R[:, 2] * 0.01 + [0, 0, 0.10], R, q_grasp)
    hand = m.body(f"openarm_{side}_hand").id
    feet = [m.body(n).id for n in ("ankle_L_Link", "ankle_R_Link")]
    st = dict(steps=0, yaw=0, unstable=0)
    rec = dict(foot0=None, foot=0.0, yaw0=None, base0=None)
    prev = [False]
    phase = ["settle"]
    events = []

    def go(T, force=None):
        for _ in range(int(T / 0.01)):
            sim.set_cmd_vel(0, 0)
            if force is not None:
                d.xfrc_applied[hand, :3] = force
            sim.step(5)
            if force is not None:
                d.xfrc_applied[hand, :3] = 0
            s = bool(sim.policy.standing)
            if phase[0] == "settle":
                prev[0] = s
                continue
            if prev[0] and not s:
                events.append(f"{phase[0]}:{'yaw' if sim._stand_fix is not None else 'unstable'}")
                st["steps"] += 1
                if sim._stand_fix is not None:
                    st["yaw"] += 1
                else:
                    st["unstable"] += 1
            prev[0] = s
            fp = np.array([d.xpos[f][:2] for f in feet])
            if rec["foot0"] is None:
                rec["foot0"], rec["yaw0"], rec["base0"] = fp.copy(), yaw_of(d.xquat[sim.base_id]), d.xpos[sim.base_id][:2].copy()
            if rec["foot0"] is not None:
                rec["foot"] = max(rec["foot"], float(np.max(np.linalg.norm(fp - rec["foot0"], axis=1))))

    def arm(q, T):
        sim.start_arm_trajectory(side, names, [(T, list(q), None)])

    go(6.0)                                             # 서기 모드 진입·안정 (측정 안 함)
    if not sim.policy.standing:
        return dict(side=side, lat=lat, seed=seed, error="서기 모드에 안 들어감")
    phase[0] = "carry"; arm(carry, 3.0); go(6.0)
    phase[0] = "pre"; arm(q_pre, 3.0); go(7.0)
    phase[0] = "grasp"; arm(q_grasp, 2.0); go(4.0)
    # 손 반력: 손가락이 상자 옆면을 밀고(옆) 손이 상자를 앞으로 미는 힘 (몸통 기준 → 세계)
    Rb = d.xmat[sim.base_id].reshape(3, 3)
    f_push = Rb @ np.array([rng.uniform(5, 12), sgn * rng.uniform(-10, 10), 0.0])
    phase[0] = "push"; go(0.4, force=f_push)
    # 손이 작업대·상자에 막혀 팔 제어기가 계속 미는 경우 (실제 집기 s1c: 손가락-작업대 접촉 뒤 몸통 6.9° 기울며 디딤 시작).
    # 아래(작업대), 앞(상자), 옆 중 하나로 PRESS_N 크기, 2 s
    dirs = [np.array([0, 0, -1.0]), Rb @ np.array([1.0, 0, 0]), Rb @ np.array([0, sgn, 0])]
    phase[0] = "press"; go(2.0, force=dirs[seed % 3] * PRESS_N * rng.uniform(0.7, 1.0))
    go(2.0)
    w_box = np.array([0, 0, -9.81 * rng.uniform(0.08, 0.5)])   # 상자 무게 (80 g ~ 0.5 kg)
    phase[0] = "lift"; arm(q_lift, 2.0); go(3.0, force=w_box)
    phase[0] = "fold"; arm(carry, 10.0); go(14.0, force=w_box)             # HOLD_VEL 0.1: 집기 앱이 쥔 팔을 접는 속도
    phase[0] = "home"; arm(POSES["home"][0 if side == "left" else 1], 3.0); go(5.0)
    yaw = math.degrees(yaw_of(d.xquat[sim.base_id]) - rec["yaw0"])
    return dict(side=side, lat=lat, seed=seed, steps=st["steps"], steps_yaw=st["yaw"], steps_unstable=st["unstable"],
                foot_cm=round(100 * rec["foot"], 1), yaw_deg=round((yaw + 180) % 360 - 180, 1),
                base_cm=round(100 * float(np.linalg.norm(d.xpos[sim.base_id][:2] - rec["base0"])), 1),
                fallen=bool(sim.fallen), events=events)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    args = [a for a in sys.argv[3:] if a != "raw"]
    nseed = int(args[0]) if args else 3
    raw = "raw" in sys.argv[3:]
    rows = []
    for side in ("right", "left"):
        for lat in (0.08, 0.15):
            for seed in range(nseed):
                r = run(sys.argv[1], sys.argv[2], side, lat, seed, raw)
                rows.append(r)
                print(json.dumps(r, ensure_ascii=False), flush=True)
    ok = [r for r in rows if "error" not in r]
    print(json.dumps(dict(summary=True, runs=len(ok), steps_total=sum(r["steps"] for r in ok),
                          no_step_runs=sum(r["steps"] == 0 for r in ok),
                          foot_cm_mean=round(float(np.mean([r["foot_cm"] for r in ok])), 1),
                          foot_cm_max=max(r["foot_cm"] for r in ok),
                          yaw_abs_mean=round(float(np.mean([abs(r["yaw_deg"]) for r in ok])), 1),
                          fallen=sum(r["fallen"] for r in ok)), ensure_ascii=False), flush=True)
