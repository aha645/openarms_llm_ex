"""MuJoCo 자코비안 기반 감쇠최소자승(DLS) IK — MoveIt 없이 IK 원리를 이해하기 위한 교육용 모듈.

ROS 의존성이 없어서 단독 테스트(tools/selftest.py)에도 사용한다.
"""
import mujoco
import numpy as np


def quat_to_mat(q):
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, np.asarray(q, dtype=float))
    return m.reshape(3, 3)


def mat_to_quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(9))
    return q


def orientation_error(R_cur, R_des):
    """회전 오차 벡터 (world 기준, 축-각)."""
    R_err = R_des @ R_cur.T
    q = mat_to_quat(R_err)
    if q[0] < 0:
        q = -q
    return 2.0 * q[1:]


class ArmIK:
    def __init__(self, model, side, tip_body=None):
        self.m = model
        self.side = side
        self.joint_names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
        self.jids = [model.joint(n).id for n in self.joint_names]
        self.qadr = np.array([model.jnt_qposadr[j] for j in self.jids])
        self.dadr = np.array([model.jnt_dofadr[j] for j in self.jids])
        self.lo = model.jnt_range[self.jids, 0]
        self.hi = model.jnt_range[self.jids, 1]
        self.tip = model.body(tip_body or f"openarm_{side}_hand_tcp").id

    def solve(self, data_in, pos, R=None, q0=None, iters=300, tol=2e-3, damping=0.05, rot_weight=0.3,
              axis_only=False):
        """world 좌표 목표 pos(3), R(3x3) → 관절각(7). 실패 시 (q, False).

        axis_only=True: R 의 z축(접근 방향)만 맞추고 그 축 회전은 자유 (MoveIt 의 z축 tolerance=π 와 동일)."""
        d = mujoco.MjData(self.m)
        d.qpos[:] = data_in.qpos
        d.mocap_pos[:] = data_in.mocap_pos
        d.mocap_quat[:] = data_in.mocap_quat
        if q0 is not None:
            d.qpos[self.qadr] = q0
        jacp = np.zeros((3, self.m.nv))
        jacr = np.zeros((3, self.m.nv))
        ok = False
        for _ in range(iters):
            mujoco.mj_kinematics(self.m, d)
            mujoco.mj_comPos(self.m, d)
            cur_p = d.xpos[self.tip]
            cur_R = d.xmat[self.tip].reshape(3, 3)
            e_p = pos - cur_p
            err = [e_p]
            mujoco.mj_jacBody(self.m, d, jacp, jacr, self.tip)
            J = [jacp[:, self.dadr]]
            if R is not None:
                if axis_only:
                    e_r = np.cross(cur_R[:, 2], R[:, 2])
                else:
                    e_r = orientation_error(cur_R, R)
                err.append(rot_weight * e_r)
                J.append(rot_weight * jacr[:, self.dadr])
            e = np.concatenate(err)
            J = np.vstack(J)
            if np.linalg.norm(e_p) < tol and (R is None or np.linalg.norm(err[1]) / rot_weight < 0.03):
                ok = True
                break
            dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(J.shape[0]), e)
            q = d.qpos[self.qadr] + np.clip(dq, -0.2, 0.2)
            d.qpos[self.qadr] = np.clip(q, self.lo, self.hi)
        return d.qpos[self.qadr].copy(), ok

    def solve_multi(self, data_in, pos, R=None, q0=None, restarts=30, seed=0, **kw):
        """여러 초기값으로 재시도 (KDL 의 random restart 와 같은 아이디어)."""
        rng = np.random.default_rng(seed)
        seeds = ([q0] if q0 is not None else []) + [rng.uniform(self.lo, self.hi) for _ in range(restarts)]
        for s in seeds:
            q, ok = self.solve(data_in, pos, R, q0=s, **kw)
            if ok:
                return q, True
        return q, False


def approach_rotation(forward, pitch_deg):
    """로봇 전방 벡터 기준, 아래로 pitch_deg 숙인 접근방향을 z축으로 하는 회전행렬 (x축 수평)."""
    a = np.radians(pitch_deg)
    f = np.asarray(forward, float)
    f = f - np.array([0, 0, f[2]])
    f /= np.linalg.norm(f)
    z = f * np.cos(a) + np.array([0, 0, -1.0]) * np.sin(a)
    x = np.cross([0, 0, 1.0], z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(f, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])
