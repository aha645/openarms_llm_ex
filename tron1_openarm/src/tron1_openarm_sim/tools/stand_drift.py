"""SF 서기 모드에서 몸통이 얼마나 흐르는지 (오프라인 MuJoCo, 실제 배포 제어기 BipedOpenArmSim 사용).
    python3 tools/stand_drift.py <world_light.xml> <정책.npz> [home|arm ...] [raw]

명령 0 으로 4 s 서서 서기 모드에 들어간 뒤 45 s 동안:
  home : 팔을 내린 채 가만히
  arm  : 오른팔 carry(arm_poses.py) → pre → grasp (sway_analysis.py 와 같은 자세), 각 15 s
  raw  : 서기 중 방향 바로잡기(biped_core STAND_YAW_TOL)를 끄고 정책만의 흐름을 잰다
지표: 몸통 회전 [°/s], 위치 흐름 [cm/s], 두 발의 회전 [°], 서기 모드 비율.
SF 서기 모드에서는 보행 시계가 멈춰 발을 떼지 않고 정책이 wz·v 명령에 반응하지 않으므로, 흐른 방향은
서기 모드를 잠깐 풀어 걸어서 되돌린다 (검증 기록 10절, 보행 학습 11절).
"""
import json
import math
import sys

import numpy as np

sys.path.insert(0, '.')
from tron1_openarm_sim.biped_core import BipedOpenArmSim  # noqa: E402
from tron1_openarm_sim.arm_poses import RIGHT  # noqa: E402
from tron1_openarm_sim.sim_core import yaw_of  # noqa: E402

PRE = [0.25, 0.35, -0.5, 1.45, 0.44, -0.41, -0.73]
GRASP = [0.44, 0.2, -0.48, 0.96, 0.45, -0.26, -0.5]


def run(world, policy, mode, raw=False):
    sim = BipedOpenArmSim(world, enable_camera=False, move_person=False, policy_path=policy)
    sim.stand_heading_hold = not raw
    m, d = sim.m, sim.d
    sim.set_base_pose(0, -0.6, 0)
    names = [f"openarm_right_joint{i}" for i in range(1, 8)]
    feet = [m.body(n).id for n in ("ankle_L_Link", "ankle_R_Link")]
    rec = []

    def go(T, log=True):
        for _ in range(int(T / 0.01)):
            sim.set_cmd_vel(0, 0)
            sim.step(5)
            if log:
                rec.append([yaw_of(d.xquat[sim.base_id]), *d.qpos[sim.base_qadr:sim.base_qadr + 2],
                            *(yaw_of(d.xquat[f]) for f in feet), float(sim.policy.standing)])
    go(4.0, log=False)
    if mode == "arm":
        for q in (RIGHT["carry"], PRE, GRASP):
            sim.start_arm_trajectory("right", names, [(2.0, q, None)])
            go(15.0)
    else:
        go(45.0)
    R = np.array(rec)
    T = len(R) * 0.01
    yaw = np.unwrap(R[:, 0])
    return dict(mode=mode + (" raw" if raw else ""), yaw_deg_s=round(math.degrees(yaw[-1] - yaw[0]) / T, 3),
                drift_cm_s=round(100 * float(np.linalg.norm(R[-1, 1:3] - R[0, 1:3])) / T, 3),
                foot_yaw_deg=[round(math.degrees(np.unwrap(R[:, k])[-1] - R[0, k]), 1) for k in (3, 4)],
                standing=round(float(R[:, 5].mean()), 2), fallen=bool(sim.fallen))


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    raw = "raw" in sys.argv[3:]
    for mode in [a for a in sys.argv[3:] if a != "raw"] or ["home", "arm"]:
        print(json.dumps(run(sys.argv[1], sys.argv[2], mode, raw)), flush=True)
