"""파지 보조 시험 — 쥐고·들고·놓는 동안 몸통이 밀리는지 (하반신 서기 정책 + 파지 보조만 따로 떼어 봄).
    [OFF=0,0.01,0] python3 tools/grasp_assist_test.py new
OFF: 손끝(TCP) 기준 상자 위치 비킴 [m] (TCP y = 손가락 벌림 방향). 실제 집기에서는 손 위치 오차 0.5~1 cm 로 손가락 하나가 먼저 닿는다.
로봇을 빈 곳에 세우고 오른팔 ready → 상자를 손가락 사이에(손가락이 닿을 때까지 붙잡고, 그 뒤 상판처럼 높이만 받침) →
쥐기 → carry 로 들기 → 놓기. 단계마다 몸통 이동 [cm], 서기 풀림 수, 최대 기울기, 손가락-상자 힘 [N], 상자-손끝 거리 [cm].
결과 (validation-log 16절): 예전 '위치 덮어쓰기' 부착은 OFF 1 cm 에서 몸통 98 cm 밀려 넘어짐, weld 는 0.1 cm.
mode: new = 시뮬의 파지 보조(sim_core._update_attached), weld = 이 스크립트가 직접 weld 를 켬 (비교용)"""
import sys, os, math, json
import numpy as np, mujoco
SIM = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, SIM)
from tron1_openarm_sim.biped_core import BipedOpenArmSim
from tron1_openarm_sim.arm_poses import POSES
mode = sys.argv[1]
src = SIM + '/mjcf/sf/world.xml'
xml = open(src).read()
if mode == 'weld':
    xml = xml.replace('</worldbody>', '</worldbody>\n  <equality><weld name="grasp_r_box_0" body1="openarm_right_hand_tcp" body2="box_0" active="false" solref="0.01 1"/></equality>', 1)
tmp = SIM + '/mjcf/sf/_grasp_exp.xml'
open(tmp, 'w').write(xml)
try:
    sim = BipedOpenArmSim(tmp, enable_camera=False, move_person=False, policy_path=SIM + '/deployed_policies/tron1sf_openarm_policy.npz')
finally:
    os.remove(tmp)
m, d = sim.m, sim.d
sim.grasp_assist = (mode in ('teleport', 'new'))
sim.set_base_pose(0, -0.6, 0)
box = m.body('box_0').id; jadr = m.jnt_qposadr[m.body_jntadr[box]]; dadr = m.jnt_dofadr[m.body_jntadr[box]]
tcp = sim.tcp['right']
fingers = {m.body('openarm_right_left_finger').id, m.body('openarm_right_right_finger').id}
eq = m.equality('grasp_r_box_0').id if mode == 'weld' else None
names = [f"openarm_right_joint{i}" for i in range(1, 8)]
state = dict(pin=False, welded=False)
log = []
def pin_box():
    p = d.xpos[tcp] + d.xmat[tcp].reshape(3, 3) @ np.array([float(v) for v in os.environ.get("OFF", "0,0,0").split(",")])
    d.qpos[jadr:jadr+3] = p; d.qpos[jadr+3:jadr+7] = d.xquat[tcp]; d.qvel[dadr:dadr+6] = 0
def finger_force():
    f = 0.0; n = 0; c6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]; b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if (b1 == box and b2 in fingers) or (b2 == box and b1 in fingers):
            mujoco.mj_contactForce(m, d, i, c6); f += abs(c6[0]); n += 1
    return f, n
def go(sec, phase):
    for _ in range(int(sec / 0.01)):
        sim.set_cmd_vel(0, 0)
        for _ in range(5):
            if state['pin']:
                pin_box()
            elif state.get('support') and sim.attached['right'] is None and not state['welded']:
                # 상판 흉내: 높이·자세만 받쳐 주고 수평으로는 밀릴 수 있게
                d.qpos[jadr + 2] = state['support'][0]; d.qpos[jadr+3:jadr+7] = state['support'][1]; d.qvel[dadr+2:dadr+6] = 0
            sim.step(1)
            if mode == 'weld' and not state['welded'] and d.ctrl[sim.grip_act['right']] < 0.03:
                f, n = finger_force()
                if n >= 2 or (state['pin'] and n >= 1 and d.qpos[sim.grip_qadr['right']] < 0.03):
                    # weld: 지금 상대 자세를 고정 (relpos/relquat: body1 기준 body2)
                    R1 = d.xmat[tcp].reshape(3, 3)
                    m.eq_data[eq, 3:6] = R1.T @ (d.xpos[box] - d.xpos[tcp])
                    q = np.zeros(4); qi = np.zeros(4); mujoco.mju_negQuat(qi, d.xquat[tcp]); mujoco.mju_mulQuat(q, qi, d.xquat[box])
                    m.eq_data[eq, 6:10] = q; m.eq_data[eq, 0:3] = 0
                    d.eq_active[eq] = 1; state['welded'] = True; state['pin'] = False
            if mode in ('teleport', 'new') and state['pin'] and finger_force()[1] >= 1:
                state['pin'] = False        # 손가락이 닿으면 놓아 줌 (작업대 위 상자처럼 밀릴 수 있게)
                state['support'] = (float(d.qpos[jadr + 2]), d.qpos[jadr+3:jadr+7].copy())
        f, n = finger_force()
        log.append(dict(phase=phase, xy=d.xpos[sim.base_id][:2].copy(), st=bool(sim.policy.standing),
                        ff=f, tilt=math.degrees(math.acos(min(1, d.xmat[sim.base_id][8]))),
                        boxrel=float(np.linalg.norm(d.xpos[box] - d.xpos[tcp]))))
go(5, 'settle')
sim.start_arm_trajectory('right', names, [(3.0, list(POSES['ready'][1]), None)])
go(5, 'reach')
sim.start_gripper_trajectory('right', [(0.5, [0.044], None)])
go(1, 'open')
state['pin'] = True
go(1, 'pinned')
sim.start_gripper_trajectory('right', [(1.0, [0.0], None)])
go(3, 'close')
state['support'] = None
sim.start_arm_trajectory('right', names, [(6.0, list(POSES['carry'][1]), None)])
go(8, 'lift')
go(5, 'hold')
sim.start_gripper_trajectory('right', [(1.0, [0.044], None)])
if mode == 'weld':
    pass
for _ in range(1):
    pass
def release_watch(sec):
    for _ in range(int(sec / 0.01)):
        if mode == 'weld' and state['welded'] and d.ctrl[sim.grip_act['right']] > 0.03:
            d.eq_active[eq] = 0; state['welded'] = False
        go(0.01, 'release')
release_watch(4)
out = {}
for ph in ['close', 'lift', 'hold', 'release']:
    L = [r for r in log if r['phase'] == ph]
    i0 = log.index(L[0]); x0 = log[i0 - 1]['xy']
    steps = sum(1 for a, b in zip([log[i0-1]] + L, L) if a['st'] and not b['st'])
    out[ph] = dict(base_cm=round(100 * max(np.linalg.norm(r['xy'] - x0) for r in L), 2), stand_release=steps,
                   tilt_max=round(max(r['tilt'] for r in L), 1), finger_force_max=round(max(r['ff'] for r in L), 1),
                   finger_force_mean=round(float(np.mean([r['ff'] for r in L])), 1),
                   box_tcp_cm=round(100 * float(np.mean([r['boxrel'] for r in L])), 2))
print(mode, json.dumps(out, ensure_ascii=False))
print('fallen', sim.fallen)
