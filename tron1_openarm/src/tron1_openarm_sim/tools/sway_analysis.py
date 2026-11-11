"""몸통 흔들림 원인 분해 실험 (오프라인 MuJoCo, 실제 배포 제어기 BipedOpenArmSim 사용).
    python3 tools/sway_analysis.py <world_light.xml> <정책.npz> <normal|massless_arms|slow|fixed_base|short_column>

조작 순서: 서기 4 s → 오른팔 carry → pre(접근점) → grasp(상자 앞), 각 이동 T_move 초 + 유지 3 s
지표(각 단계): 몸통 위치 출렁임 범위, 기울기 범위, 손끝 오차(실제 − 이상) 와 그 분해:
  몸통 몫 = (실제 몸통 자세 + 명령 관절각)으로 계산한 손끝 − 이상(처음 몸통 자세 + 명령 관절각)
  팔 몫   = 실제 손끝 − (실제 몸통 자세 + 명령 관절각) = 관절 추종 오차(중력 처짐)
"""
import sys, json, math, numpy as np, mujoco
sys.path.insert(0, '.'); sys.path.insert(0, '../../../mobile_openarm/src/mobile_openarm_sim')
from tron1_openarm_sim.biped_core import BipedOpenArmSim
world, policy, variant = sys.argv[1], sys.argv[2], sys.argv[3]
T_move = 6.0 if variant == "slow" else 2.0
sim = BipedOpenArmSim(world, enable_camera=False, move_person=False, policy_path=policy)
m, d = sim.m, sim.d
if variant == "massless_arms":
    for i in range(m.nbody):
        if m.body(i).name.startswith(("openarm_left", "openarm_right")):
            m.body_mass[i] *= 0.01; m.body_inertia[i] *= 0.01
sim.set_base_pose(0, -0.6, 0)
names = [f"openarm_right_joint{i}" for i in range(1, 8)]
qadr = [m.jnt_qposadr[m.joint(n).id] for n in names]
hand = m.body("openarm_right_hand").id
def tcp(dd):
    return dd.xpos[hand] + dd.xmat[hand].reshape(3, 3) @ [0, 0, 0.09]
fixed = variant == "fixed_base"
def step(T):
    rec = []
    for k in range(int(T / 0.01)):
        sim.set_cmd_vel(0, 0)
        if fixed:
            a = sim.base_qadr
            d.qpos[a:a+7] = base0; d.qvel[sim.m.jnt_dofadr[m.joint("base_free").id]:][:6] = 0
        sim.step(5)
        if k % 5 == 0:
            R = d.xmat[sim.base_id].reshape(3, 3)
            rec.append((d.xpos[sim.base_id].copy(), math.degrees(math.acos(min(1, R[2, 2])))))
    return rec
base0 = None
step(4.0)
a = sim.base_qadr; base0 = d.qpos[a:a+7].copy()
d_ideal = mujoco.MjData(m)
out = {}
for tag, q in (("carry", [0.3, 0, -1.2, 2.0, 0, 0, 0]), ("pre", [0.25, 0.35, -0.5, 1.45, 0.44, -0.41, -0.73]),
               ("grasp", [0.44, 0.2, -0.48, 0.96, 0.45, -0.26, -0.5])):
    sim.start_arm_trajectory("right", names, [(T_move, q, None)])
    rec = step(T_move + 3.0)
    P = np.array([r[0] for r in rec]); tilt = np.array([r[1] for r in rec])
    # 이상 손끝: 처음 몸통 자세 + 명령 관절각
    d_ideal.qpos[:] = d.qpos; d_ideal.qpos[a:a+7] = base0; d_ideal.qpos[qadr] = q; mujoco.mj_kinematics(m, d_ideal); ideal = tcp(d_ideal).copy()
    d_ideal.qpos[:] = d.qpos; d_ideal.qpos[qadr] = q; mujoco.mj_kinematics(m, d_ideal); cmd_actual_base = tcp(d_ideal).copy()
    actual = tcp(d).copy()
    out[tag] = dict(sway_cm=round(100 * float(max(np.ptp(P[:, 0]), np.ptp(P[:, 1]))), 1),
                    tilt_range=[round(float(tilt.min()), 1), round(float(tilt.max()), 1)],
                    hand_err_cm=round(100 * float(np.linalg.norm(actual - ideal)), 1),
                    from_body_cm=round(100 * float(np.linalg.norm(cmd_actual_base - ideal)), 1),
                    from_arm_sag_cm=round(100 * float(np.linalg.norm(actual - cmd_actual_base)), 1),
                    fallen=bool(sim.fallen))
print(json.dumps({"variant": variant, "foot": sim.foot, **out}, ensure_ascii=False))
