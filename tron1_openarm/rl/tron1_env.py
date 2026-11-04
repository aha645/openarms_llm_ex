"""TRON1 + OpenArm 보행 강화학습 환경 (MuJoCo, CPU 병렬).

LimX tron1-rl-isaacgym 의 pointfoot_flat 과 같은 관측·행동·보상 규약을 MuJoCo 로 옮겼다.
달라진 점: 상반신(OpenArm 양팔 14.4 kg)이 얹혀 있고, 학습 중 팔 자세가 계속 바뀐다.

* 병렬화: mujoco.rollout.Rollout (C 스레드) 로 N 개 환경을 한 번에 진행
* 정책 주기 50 Hz, 학습 물리 dt 4 ms (5 스텝). 배포(시뮬레이터)는 2 ms 로 다시 검증한다.
* 도메인 랜덤화: 몸통 질량(-0.5~+2 kg)·무게중심(±2 cm)·바닥 마찰(0.4~1.2)·PD 게인(±15%)·
  행동 지연(0~12 ms)·밀기(push)·관측 잡음·팔 자세(이름 자세 + 무작위)
"""
import math

import mujoco
import numpy as np
from mujoco import rollout

LEG = ["abad_L_joint", "hip_L_joint", "knee_L_joint", "abad_R_joint", "hip_R_joint", "knee_R_joint"]
KP, KD, TAU_MAX, ACTION_SCALE = 42.0, 3.5, 80.0, 0.25
POLICY_DT = 0.02
GAIT = np.array([2.0, 0.5, 0.5, 0.1], np.float32)
CMD_SCALE = np.array([2.0, 2.0, 0.25], np.float32)
OBS_DIM, HIST = 30, 10
BASE_HEIGHT_TARGET = 0.68

ARM_POSES = {   # SRDF(mobile_openarm_moveit_config) 의 이름 자세 — DAY2 앱이 실제로 쓰는 자세들
    "home": ([0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]),
    "ready": ([0, 0, 0, 1.57, 0, 0, 0], [0, 0, 0, 1.57, 0, 0, 0]),
    "carry": ([-0.3, 0, 1.2, 2.0, 0, 0, 0], [0.3, 0, -1.2, 2.0, 0, 0, 0]),
    "hands_up": ([0, 0, 0, 2.0, 0, 0, 0], [0, 0, 0, 2.0, 0, 0, 0]),
}
REWARD_SCALES = dict(
    keep_balance=1.0, tracking_lin_vel=3.0, tracking_ang_vel=1.5, base_height=-20.0, lin_vel_z=-0.5,
    ang_vel_xy=-0.05, torques=-8e-5, action_rate=-0.01, action_smooth=-0.01, orientation=-10.0,
    dof_pos_limits=-2.0, feet_distance=-100.0, contact_force=-2.0, contact_vel=-2.0, termination=-20.0)


def ncdf(x, s=0.05):
    return 0.5 * (1.0 + np.vectorize(math.erf)(x / (s * math.sqrt(2))))


def desired_contacts(gait_index):
    """LimX base_task._step_contact_targets 와 같은 식. 반환 (N,2) 왼발·오른발 접지 기대값 0~1."""
    off, dur = GAIT[1], GAIT[2]
    fi = np.stack([gait_index, gait_index + off + 1.0], 1) % 1.0
    st = fi < dur
    fi = np.where(st, fi * (0.5 / dur), 0.5 + (fi - dur) * (0.5 / (1 - dur)))
    return ncdf(fi) * (1 - ncdf(fi - 0.5)) + ncdf(fi - 1) * (1 - ncdf(fi - 1.5))


def quat_rot_inv(q, v):
    """q (N,4) wxyz, v (N,3) → R(q)^T v."""
    w, x, y, z = q[:, 0:1], q[:, 1:2], q[:, 2:3], q[:, 3:4]
    qv = np.concatenate([x, y, z], 1)
    a = v * (2.0 * w ** 2 - 1.0)
    b = np.cross(qv, v) * w * 2.0
    c = qv * np.sum(qv * v, 1, keepdims=True) * 2.0
    return a - b + c


class Tron1OpenArmEnv:
    def __init__(self, xml, num_envs=128, num_models=8, seed=0, nthread=2, randomize=True,
                 episode_s=20.0, cmd_ranges=None, push=True):
        self.rng = np.random.default_rng(seed)
        base = mujoco.MjModel.from_xml_path(xml)
        self.dt = base.opt.timestep
        self.nsub = int(round(POLICY_DT / self.dt))
        self.N = num_envs
        self.randomize = randomize
        self.push = push
        self.ep_len = int(episode_s / POLICY_DT)
        self.cmd_ranges = cmd_ranges or dict(vx=(-0.35, 0.6), vy=(-0.2, 0.2), wz=(-0.9, 0.9))
        # --- 인덱스
        self.qadr = np.array([base.jnt_qposadr[base.joint(j).id] for j in LEG])
        self.vadr = np.array([base.jnt_dofadr[base.joint(j).id] for j in LEG])
        self.leg_act = np.array([base.actuator(j + "_ctrl").id for j in LEG])
        self.arm_act = {s: np.array([base.actuator(f"{s}_joint{i}_ctrl").id for i in range(1, 8)])
                        for s in ("left", "right")}
        self.arm_qadr = {s: np.array([base.jnt_qposadr[base.joint(f"openarm_{s}_joint{i}").id] for i in range(1, 8)])
                         for s in ("left", "right")}
        self.finger_act = [base.actuator(f"{s}_finger1_ctrl").id for s in ("left", "right")]
        self.jrange = base.jnt_range[[base.joint(j).id for j in LEG]].copy()
        self.arm_range = {s: base.actuator_ctrlrange[self.arm_act[s]].copy() for s in self.arm_act}

        def sadr(n):
            i = base.sensor(n).id
            return slice(base.sensor_adr[i], base.sensor_adr[i] + base.sensor_dim[i])
        self.s = {n: sadr(n) for n in ["imu_gyro", "imu_quat", "base_linvel", "foot_L_touch", "foot_R_touch",
                                        "foot_L_pos", "foot_R_pos", "foot_L_vel", "foot_R_vel"]}
        self.s_tau = [sadr(j + "_tau") for j in LEG]
        # --- 모델 변형 (도메인 랜덤화)
        self.models = []
        bid = base.body("base_link").id
        for k in range(num_models):
            m = mujoco.MjModel.from_xml_path(xml)
            if randomize and k > 0:
                m.body_mass[bid] += self.rng.uniform(-0.5, 2.0)
                m.body_ipos[bid] += self.rng.uniform([-0.02, -0.015, -0.02], [0.02, 0.015, 0.02])
                fr = self.rng.uniform(0.4, 1.2)
                m.geom_friction[:, 0] = np.where(m.geom_contype > 0, fr, m.geom_friction[:, 0])
                g = self.rng.uniform(0.85, 1.15)
                for a in self.leg_act:
                    m.actuator_gainprm[a, 0] = KP * g
                    m.actuator_biasprm[a, 1] = -KP * g
                    m.actuator_biasprm[a, 2] = -KD * self.rng.uniform(0.85, 1.15)
            self.models.append(m)
        self.model_of_env = np.zeros(self.N, np.int64)      # set_difficulty() 가 변형 모델을 섞음
        self.model_list = [self.models[0]] * self.N
        self.datas = [mujoco.MjData(base) for _ in range(nthread)]
        self.roll = rollout.Rollout(nthread=nthread)
        self.spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
        self.ns = mujoco.mj_stateSize(base, self.spec)
        d = mujoco.MjData(base)
        mujoco.mj_forward(base, d)
        self.init_state = np.zeros(self.ns)
        mujoco.mj_getState(base, d, self.init_state, self.spec)
        self.q0 = 1                                   # state 안에서 qpos 시작 (0 = time)
        self.v0 = 1 + base.nq
        self.nu = base.nu
        self.nsens = base.nsensordata
        # --- 버퍼
        N = self.N
        self.state = np.tile(self.init_state, (N, 1))
        self.ctrl = np.zeros((N, self.nu))
        self.ctrl[:, self.finger_act] = 0.044
        self.sens = np.zeros((N, self.nsens))
        self.last_action = np.zeros((N, 6), np.float32)
        self.last_action2 = np.zeros((N, 6), np.float32)
        self.last_target = np.zeros((N, 6))
        self.gait_index = np.zeros(N)
        self.cmd = np.zeros((N, 3), np.float32)
        self.hist = np.zeros((N, HIST * OBS_DIM), np.float32)
        self.t = np.zeros(N, np.int64)
        self.delay = np.zeros(N, np.int64)
        self.arm_from = np.zeros((N, 14))
        self.arm_to = np.zeros((N, 14))
        self.arm_t0 = np.zeros(N)
        self.arm_T = np.ones(N)
        self.ep_ret = np.zeros(N)
        self.difficulty = 0.0       # 0 → 1: 잡음·지연·밀기·팔 동작 강도 (train_ppo.py 가 낙상률을 보며 올림)
        self.stats = {}
        self.reset(np.arange(N))

    def set_difficulty(self, d):
        self.difficulty = float(np.clip(d, 0, 1))
        n_rand = int(round(self.N * self.difficulty))
        self.model_of_env = np.where(np.arange(self.N) < n_rand, 1 + np.arange(self.N) % (len(self.models) - 1), 0)
        self.model_list = [self.models[i] for i in self.model_of_env]

    # ------------------------------------------------------------------ helpers
    def _sample_arm(self, n):
        out = np.zeros((n, 14))
        names = list(ARM_POSES)
        for i in range(n):
            r = self.rng.random()
            arm_level = max(self.difficulty, 0.6)   # 팔 자세는 조작(DAY2)에 꼭 필요 → 난이도와 별개로 최소 60%
            if r > arm_level:
                q = np.zeros(14)
            elif r < 0.7 * arm_level:
                L, R = ARM_POSES[names[self.rng.integers(len(names))]]
                q = np.array(L + R, float)
            else:   # 작업 중 자세 흉내: 이름 자세 두 개 사이 + 무작위
                a, b = (ARM_POSES[names[j]] for j in self.rng.integers(len(names), size=2))
                lam = self.rng.random()
                q = lam * np.array(a[0] + a[1]) + (1 - lam) * np.array(b[0] + b[1])
                q += self.rng.normal(0, 0.25, 14)
            lo = np.concatenate([self.arm_range["left"][:, 0], self.arm_range["right"][:, 0]])
            hi = np.concatenate([self.arm_range["left"][:, 1], self.arm_range["right"][:, 1]])
            out[i] = np.clip(q, lo, hi)
        return out

    def _resample_cmd(self, idx):
        n = len(idx)
        r = self.cmd_ranges
        c = np.stack([self.rng.uniform(*r["vx"], n), self.rng.uniform(*r["vy"], n) * (self.rng.random(n) < 0.3),
                      self.rng.uniform(*r["wz"], n)], 1)
        z = self.rng.random(n)
        c[z < 0.25] = 0.0                                   # 25%: 제자리(명령 0) — Nav2 정지·도킹 대기
        c[(z >= 0.25) & (z < 0.45), 1:] = 0.0               # 20%: 직진만 (가장 흔한 주행)
        c[(z >= 0.45) & (z < 0.6), 0:2] = 0.0               # 15%: 제자리 회전 (도킹 yaw 보정)
        c[np.abs(c) < 0.05] = 0.0
        self.cmd[idx] = c

    def reset(self, idx):
        if len(idx) == 0:
            return
        st = np.tile(self.init_state, (len(idx), 1))
        arm = self._sample_arm(len(idx))
        for k, i in enumerate(idx):
            m = self.models[self.model_of_env[i]]
            q = st[k, self.q0:self.q0 + m.nq]
            yaw = self.rng.uniform(-math.pi, math.pi)
            q[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
            q[self.qadr] += self.rng.uniform(-0.05, 0.05, 6)
            q[self.arm_qadr["left"]] = arm[k, :7]
            q[self.arm_qadr["right"]] = arm[k, 7:]
            st[k, self.q0:self.q0 + m.nq] = q
        self.state[idx] = st
        self.ctrl[idx][:, :] = 0
        self.ctrl[np.ix_(idx, self.arm_act["left"])] = arm[:, :7]
        self.ctrl[np.ix_(idx, self.arm_act["right"])] = arm[:, 7:]
        self.ctrl[np.ix_(idx, self.finger_act)] = 0.044
        self.arm_from[idx] = arm
        self.arm_to[idx] = arm
        self.arm_t0[idx] = 0
        self.arm_T[idx] = 1
        self.last_action[idx] = 0
        self.last_action2[idx] = 0
        self.last_target[idx] = 0
        self.gait_index[idx] = 0
        self.t[idx] = 0
        self.delay[idx] = self.rng.integers(0, 1 + int(round(3 * self.difficulty)), len(idx)) if self.randomize else 0
        self.ep_ret[idx] = 0
        self.hist[idx] = 0
        self._resample_cmd(idx)
        self.sens[idx] = 0
        self._hist_reset = np.zeros(self.N, bool)
        self._hist_reset[idx] = True

    # ------------------------------------------------------------------ observation
    def _obs(self, noise=True):
        S = self.sens
        q = self.state[:, self.q0:][:, self.qadr]
        dq = self.state[:, self.v0:][:, self.vadr]
        quat = S[:, self.s["imu_quat"]]
        quat = np.where(np.linalg.norm(quat, axis=1, keepdims=True) < 0.5,
                        self.state[:, self.q0 + 3:self.q0 + 7], quat)
        gyro = S[:, self.s["imu_gyro"]]
        grav = quat_rot_inv(quat, np.tile([0.0, 0.0, -1.0], (self.N, 1)))
        clock = np.stack([np.sin(2 * np.pi * self.gait_index), np.cos(2 * np.pi * self.gait_index)], 1)
        obs = np.concatenate([gyro * 0.25, grav, q, dq * 0.05, self.last_action, clock,
                              np.tile(GAIT, (self.N, 1))], 1).astype(np.float32)
        if noise and self.randomize:
            sc = np.concatenate([np.full(3, 0.2 * 0.25), np.full(3, 0.05), np.full(6, 0.01), np.full(6, 1.5 * 0.05),
                                 np.zeros(12)]).astype(np.float32)
            obs += self.rng.uniform(-1, 1, obs.shape).astype(np.float32) * sc * self.difficulty
        r = self._hist_reset
        self.hist[r] = np.tile(obs[r], HIST)
        self.hist[~r] = np.concatenate([self.hist[~r, OBS_DIM:], obs[~r]], 1)
        self._hist_reset[:] = False
        return obs

    def observe(self):
        """정책 입력 재료: (obs 30, 관측 기록 300, 명령 3, 특권정보=실제 선속도×2 3)."""
        obs = self._obs()
        lin = self.sens[:, self.s["base_linvel"]] * 2.0
        return obs, self.hist.copy(), (self.cmd * CMD_SCALE).astype(np.float32), lin.astype(np.float32)

    # ------------------------------------------------------------------ step
    def step(self, action):
        N = self.N
        a = np.clip(action, -100, 100).astype(np.float32)
        q = self.state[:, self.q0:][:, self.qadr]
        dq = self.state[:, self.v0:][:, self.vadr]
        lo = (q + (KD * dq - TAU_MAX) / KP) / ACTION_SCALE
        hi = (q + (KD * dq + TAU_MAX) / KP) / ACTION_SCALE
        a = np.clip(a, lo, hi).astype(np.float32)
        target = a * ACTION_SCALE
        # 팔: 3~6초마다 새 자세로 1.5~3초에 걸쳐 이동 (MoveIt 궤적 흉내)
        tsec = self.t * POLICY_DT
        new = ((tsec - self.arm_t0) > self.arm_T) & (self.rng.random(N) < POLICY_DT / 3.0 * max(self.difficulty, 0.6))
        idx = np.nonzero(new)[0]
        if len(idx):
            cur = self._arm_now(tsec)
            self.arm_from[idx] = cur[idx]
            self.arm_to[idx] = self._sample_arm(len(idx))
            self.arm_t0[idx] = tsec[idx]
            self.arm_T[idx] = self.rng.uniform(1.5, 3.0, len(idx))
        arm = self._arm_now(tsec)
        ctrl = np.repeat(self.ctrl[:, None, :], self.nsub, 1)
        ctrl[:, :, self.arm_act["left"]] = arm[:, None, :7]
        ctrl[:, :, self.arm_act["right"]] = arm[:, None, 7:]
        ctrl[:, :, self.leg_act] = target[:, None, :]
        for k in range(1, 4):     # 행동 지연: 처음 k 스텝은 이전 목표 유지
            sel = self.delay >= k
            ctrl[np.ix_(sel, [k - 1], self.leg_act)] = self.last_target[sel][:, None, :]
        # 밀기: 평균 7초마다 몸통 속도 ±0.5 m/s
        if self.push and self.randomize:
            pz = self.rng.random(N) < POLICY_DT / 7.0
            if pz.any():
                self.state[pz, self.v0:self.v0 + 2] += self.rng.uniform(-0.5, 0.5, (pz.sum(), 2)) * self.difficulty
        st, se = self.roll.rollout(self.model_list, self.datas, self.state, ctrl, nstep=self.nsub)
        self.state = st[:, -1, :].copy()
        self.sens = se[:, -1, :].copy()
        self.ctrl[:, self.leg_act] = target
        self.last_target = target
        self.gait_index = (self.gait_index + POLICY_DT * GAIT[0]) % 1.0
        self.t += 1

        rew, terms = self._reward(a)
        self.last_action2 = self.last_action
        self.last_action = a
        h = self.state[:, self.q0 + 2]
        quat = self.state[:, self.q0 + 3:self.q0 + 7]
        upz = 1 - 2 * (quat[:, 1] ** 2 + quat[:, 2] ** 2)
        fell = (h < 0.45) | (upz < 0.7) | ~np.isfinite(self.state).all(1)
        timeout = self.t >= self.ep_len
        rew = rew + fell * REWARD_SCALES["termination"]
        self.ep_ret += rew
        done = fell | timeout
        # 명령 재설정 (5초마다)
        rs = (self.t % 250 == 0) & ~done
        self._resample_cmd(np.nonzero(rs)[0])
        info = {"fell": fell, "timeout": timeout, "terms": terms, "ep_ret": self.ep_ret.copy(), "ep_t": self.t.copy()}
        self.reset(np.nonzero(done)[0])
        return rew.astype(np.float32), done, info

    def _arm_now(self, tsec):
        s = np.clip((tsec - self.arm_t0) / self.arm_T, 0, 1)
        s = s * s * (3 - 2 * s)
        return self.arm_from + (self.arm_to - self.arm_from) * s[:, None]

    def _reward(self, a):
        S = self.sens
        quat = self.state[:, self.q0 + 3:self.q0 + 7]
        lin = S[:, self.s["base_linvel"]]
        gyro = S[:, self.s["imu_gyro"]]
        grav = quat_rot_inv(quat, np.tile([0.0, 0.0, -1.0], (self.N, 1)))
        h = self.state[:, self.q0 + 2]
        q = self.state[:, self.q0:][:, self.qadr]
        tau = np.stack([S[:, s][:, 0] for s in self.s_tau], 1)
        F = np.stack([S[:, self.s["foot_L_touch"]][:, 0], S[:, self.s["foot_R_touch"]][:, 0]], 1)
        fv = np.stack([np.linalg.norm(S[:, self.s["foot_L_vel"]], axis=1),
                       np.linalg.norm(S[:, self.s["foot_R_vel"]], axis=1)], 1)
        fp = S[:, self.s["foot_L_pos"]][:, :2] - S[:, self.s["foot_R_pos"]][:, :2]
        des = desired_contacts(self.gait_index)
        lo, hi = self.jrange[:, 0] * 0.95, self.jrange[:, 1] * 0.95
        t = dict(
            keep_balance=np.ones(self.N),
            tracking_lin_vel=np.exp(-np.sum((self.cmd[:, :2] - lin[:, :2]) ** 2, 1) / 0.05),   # LimX 0.2 → 0.05: 옆 흐름·속도 부족을 더 강하게
            tracking_ang_vel=np.exp(-(self.cmd[:, 2] - gyro[:, 2]) ** 2 / 0.1),
            base_height=(h - BASE_HEIGHT_TARGET) ** 2,
            lin_vel_z=lin[:, 2] ** 2,
            ang_vel_xy=np.sum(gyro[:, :2] ** 2, 1),
            torques=np.sum(tau ** 2, 1),
            action_rate=np.sum((a - self.last_action) ** 2, 1),
            action_smooth=np.sum((a - 2 * self.last_action + self.last_action2) ** 2, 1),
            orientation=np.sum(grav[:, :2] ** 2, 1),
            dof_pos_limits=np.sum(np.clip(lo - q, 0, None) + np.clip(q - hi, 0, None), 1),
            feet_distance=np.clip(0.115 - np.linalg.norm(fp, axis=1), 0, 1),
            contact_force=np.mean((1 - des) * (1 - np.exp(-F ** 2 / 25.0)), 1),
            contact_vel=np.mean(des * (1 - np.exp(-fv ** 2 / 0.25)), 1),
        )
        r = sum(REWARD_SCALES[k] * v for k, v in t.items()) * POLICY_DT
        return r, t
