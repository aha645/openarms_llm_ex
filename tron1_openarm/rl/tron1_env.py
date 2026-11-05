"""TRON1 + OpenArm 보행 강화학습 환경 (MuJoCo, CPU 병렬).

LimX tron1-rl-isaacgym 의 pointfoot_flat 과 같은 관측·행동·보상 규약을 MuJoCo 로 옮겼다.
달라진 점: 상반신(OpenArm 양팔 14.4 kg)이 얹혀 있고, 학습 중 팔 자세가 계속 바뀐다.

* 병렬화: mujoco.rollout.Rollout (C 스레드) 로 N 개 환경을 한 번에 진행
* 정책 주기 50 Hz, 학습 물리 dt 4 ms (5 스텝). 배포(시뮬레이터)는 2 ms 로 다시 검증한다.
* 도메인 랜덤화: 몸통 질량(-0.5~+2 kg)·무게중심(±2 cm)·바닥 마찰(0.4~1.2)·PD 게인(±15%)·
  행동 지연(0~12 ms)·밀기(push)·관측 잡음·팔 자세(이름 자세 + 무작위)
"""
import math

import os
import sys

import mujoco
import numpy as np
from mujoco import rollout

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "tron1_openarm_sim"))
from tron1_openarm_sim.locomotion_policy import PF, SF, SF_BASE_HEIGHT_CMD  # noqa: E402  관측·행동 규약을 실행기와 공유

ACTION_SCALE = 0.25
POLICY_DT = 0.02
CMD_SCALE = np.array([2.0, 2.0, 0.25], np.float32)
BASE_HEIGHT_TARGET = {"pf": 0.68, "sf": SF_BASE_HEIGHT_CMD}

from tron1_openarm_sim.arm_poses import POSES as ARM_POSES  # noqa: E402  SRDF 이름 자세와 같은 값 (어깨를 낮춘 설계용)
REWARD_SCALES = dict(
    keep_balance=1.0, tracking_lin_vel=3.0, tracking_ang_vel=1.5, base_height=-20.0, lin_vel_z=-0.5,
    ang_vel_xy=-0.05, torques=-8e-5, action_rate=-0.01, action_smooth=-0.01, orientation=-60.0,
    dof_pos_limits=-2.0, feet_distance=-100.0, contact_force=-2.0, contact_vel=-2.0, termination=-20.0)


def ncdf(x, s=0.05):
    return 0.5 * (1.0 + np.vectorize(math.erf)(x / (s * math.sqrt(2))))


def desired_contacts(gait_index, gait):
    """LimX base_task._step_contact_targets 와 같은 식. 반환 (N,2) 왼발·오른발 접지 기대값 0~1."""
    off, dur = gait[1], gait[2]
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
                 episode_s=20.0, cmd_ranges=None, push=True, foot="pf"):
        self.rng = np.random.default_rng(seed)
        self.foot = foot
        self.sp = PF if foot == "pf" else SF
        LEG = self.sp.joints
        self.nj = len(LEG)
        self.n_cmd = self.sp.n_cmd
        self.obs_dim, self.HIST = self.sp.obs_dim, self.sp.hist
        self.KP, self.KD, self.TAU = self.sp.kp.astype(np.float64), self.sp.kd.astype(np.float64), self.sp.tau.astype(np.float64)
        self.q_def = self.sp.default_q.astype(np.float64)
        self.GAIT = self.sp.gait
        self.rs = dict(REWARD_SCALES)
        self.ang_sigma = 0.1
        if foot == "sf":
            # SF 1차 미세조정은 회전 추종이 약했다(0.6 rad/s 명령에 0.08) → 회전 추종 보상을 2배
            self.rs["tracking_ang_vel"] = 3.0
            # SF 3차(sf_v3): 시뮬 집기 시험에서 SF 는 ① 느린 명령(0.04~0.08 m/s)을 거의 무시(2~3 s 에 1 cm)하고
            # ② 0.25 m/s 직진 6 s 동안 13~17° 돌아갔다(제자리 유지 루프가 회전을 명령해도 못 따라감).
            # → 회전 추종 보상을 더 키우고 허용 폭을 좁혀(0.1 → 0.05) 작은 회전 오차도 벌점이 되게 한다
            self.rs["tracking_ang_vel"] = 5.0
            self.ang_sigma = 0.05
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
                for i, a in enumerate(self.leg_act):
                    m.actuator_gainprm[a, 0] = self.KP[i] * g
                    m.actuator_biasprm[a, 1] = -self.KP[i] * g
                    m.actuator_biasprm[a, 2] = -self.KD[i] * self.rng.uniform(0.85, 1.15)
            self.models.append(m)
        # 상반신(OpenArm) 질량 배율 — 무게 커리큘럼용. SF 사전학습 정책은 14.5 kg 상반신을 얹으면 1초 만에 넘어져
        # (상반신 질량을 0 으로 하면 잘 선다) PPO 가 배울 거리가 없다 → 가벼운 상반신에서 시작해 1.0 까지 올린다
        self.payload_bodies = [i for i in range(base.nbody) if base.body(i).name.startswith("openarm_")]
        self._pl_base = [(m.body_mass[self.payload_bodies].copy(), m.body_inertia[self.payload_bodies].copy()) for m in self.models]
        self.payload = 1.0
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
        nj = self.nj
        self.last_action = np.zeros((N, nj), np.float32)
        self.last_action2 = np.zeros((N, nj), np.float32)
        self.last_target = np.tile(self.q_def, (N, 1))
        self.gait_index = np.zeros(N)
        self.cmd = np.zeros((N, 3), np.float32)
        self.want_still = np.zeros(N, bool)     # SF: 제자리 서기 명령 (명령 0 일 때 대부분)
        self.standing = np.zeros(N, bool)       # SF: 보행 시계를 멈추고 서 있는 중
        self.hist = np.zeros((N, self.HIST * self.obs_dim), np.float32)
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

    def set_payload(self, s):
        self.payload = float(np.clip(s, 0.0, 1.0))
        for m, (bm, bi) in zip(self.models, self._pl_base):
            m.body_mass[self.payload_bodies] = bm * self.payload
            m.body_inertia[self.payload_bodies] = bi * self.payload + 1e-6 * (1 - self.payload)

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
                # 왼팔·오른팔 자세를 따로 고른다 (절반). DAY2 앱은 한 팔씩 움직인다(예: 오른팔 carry + 왼팔 home).
                # 양팔이 항상 같은 자세로만 학습한 v7 은 이런 비대칭 자세에서 옆으로 0.05 m/s 흘러가 버렸다.
                iL, iR = self.rng.integers(len(names), size=2)
                if self.rng.random() < 0.5:
                    iR = iL
                q = np.array(ARM_POSES[names[iL]][0] + ARM_POSES[names[iR]][1], float)
            else:   # 작업 중 자세 흉내: 이름 자세 두 개 사이 + 무작위 (팔마다 따로)
                q = np.zeros(14)
                for s in (0, 1):
                    a, b = (ARM_POSES[names[j]][s] for j in self.rng.integers(len(names), size=2))
                    lam = self.rng.random()
                    q[7 * s:7 * s + 7] = lam * np.array(a) + (1 - lam) * np.array(b)
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
        if self.foot == "sf":
            # SF: 15% 는 느린 미세 이동 (도킹·옆걸음·작은 회전, |vx| ≤ 0.12, |vy| ≤ 0.08, |wz| ≤ 0.3).
            # 예전에는 |명령| < 0.05 를 0 으로 바꿔 느린 명령을 학습한 적이 없었다 → 0.04~0.08 m/s 에서 거의 안 움직임
            slow = (z >= 0.6) & (z < 0.75)
            k = int(slow.sum())
            if k:
                c[slow] = np.stack([self.rng.uniform(-0.12, 0.12, k), self.rng.uniform(-0.08, 0.08, k) * (self.rng.random(k) < 0.5),
                                    self.rng.uniform(-0.3, 0.3, k) * (self.rng.random(k) < 0.5)], 1)
            c[np.abs(c) < 0.015] = 0.0
        else:
            c[np.abs(c) < 0.05] = 0.0
        self.cmd[idx] = c
        if self.foot == "sf":
            # SF: 명령 0 인 경우의 80% 는 '제자리 서기'(발 안 떼기) — 조작할 때 쓰는 모드. 나머지는 제자리 걸음
            zero = np.all(c == 0, 1)
            self.want_still[idx] = zero & (self.rng.random(n) < 0.8)
            self.standing[idx] = False

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
            q[self.qadr] = self.q_def + self.rng.uniform(-0.05, 0.05, self.nj)
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
        self.last_target[idx] = self.q_def
        self.ctrl[np.ix_(idx, self.leg_act)] = self.q_def
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
        nj = self.nj
        obs = np.concatenate([gyro * 0.25, grav, q - self.q_def, dq * 0.05, self.last_action, clock,
                              np.tile(self.GAIT, (self.N, 1))], 1).astype(np.float32)
        if noise and self.randomize:
            sc = np.concatenate([np.full(3, 0.2 * 0.25), np.full(3, 0.05), np.full(nj, 0.01), np.full(nj, 1.5 * 0.05),
                                 np.zeros(nj + 6)]).astype(np.float32)
            obs += self.rng.uniform(-1, 1, obs.shape).astype(np.float32) * sc * self.difficulty
        r = self._hist_reset
        self.hist[r] = np.tile(obs[r], self.HIST)
        self.hist[~r] = np.concatenate([self.hist[~r, self.obs_dim:], obs[~r]], 1)
        self._hist_reset[:] = False
        return obs

    def observe(self):
        """정책 입력 재료: (obs 30, 관측 기록 300, 명령 3, 특권정보=실제 선속도×2 3)."""
        obs = self._obs()
        lin = self.sens[:, self.s["base_linvel"]] * 2.0
        return obs, self.hist.copy(), self.command_vector(), lin.astype(np.float32)

    def command_vector(self):
        c = (self.cmd * CMD_SCALE).astype(np.float32)
        if self.foot == "pf":
            return c
        still = self.standing.astype(np.float32)
        c = c * (1 - still)[:, None]
        return np.concatenate([c, np.full((self.N, 1), SF_BASE_HEIGHT_CMD, np.float32), still[:, None]], 1)

    def _advance_clock(self):
        """SF 제자리 서기: 두 발 다 디딘 위상(0 또는 0.5 근처)에 오면 시계를 멈춘다 (LimX 와 같은 규칙)."""
        if self.foot == "sf":
            g = self.gait_index
            win = self.want_still & ~self.standing & ((g < 0.05) | (g >= 0.95) | ((g >= 0.5) & (g < 0.55)))
            self.gait_index[win] = np.where((g[win] >= 0.5) & (g[win] < 0.55), 0.5, 0.0)
            self.standing |= win
            self.standing &= self.want_still
            mov = ~self.standing
            self.gait_index[mov] = (self.gait_index[mov] + POLICY_DT * self.GAIT[0]) % 1.0
        else:
            self.gait_index = (self.gait_index + POLICY_DT * self.GAIT[0]) % 1.0

    # ------------------------------------------------------------------ step
    def step(self, action):
        N = self.N
        a = np.clip(action, -100, 100).astype(np.float32)
        q = self.state[:, self.q0:][:, self.qadr]
        dq = self.state[:, self.v0:][:, self.vadr]
        qd = q - self.q_def
        lo = (qd + (self.KD * dq - self.TAU) / self.KP) / ACTION_SCALE
        hi = (qd + (self.KD * dq + self.TAU) / self.KP) / ACTION_SCALE
        a = np.clip(a, lo, hi).astype(np.float32)
        target = a * ACTION_SCALE + self.q_def
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
        self._advance_clock()
        self.t += 1

        rew, terms = self._reward(a)
        self.last_action2 = self.last_action
        self.last_action = a
        h = self.state[:, self.q0 + 2]
        quat = self.state[:, self.q0 + 3:self.q0 + 7]
        upz = 1 - 2 * (quat[:, 1] ** 2 + quat[:, 2] ** 2)
        fell = (h < 0.45) | (upz < 0.7) | ~np.isfinite(self.state).all(1)
        timeout = self.t >= self.ep_len
        rew = rew + fell * self.rs["termination"]
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
        des = desired_contacts(self.gait_index, self.GAIT)
        if self.foot == "sf":
            des[self.standing] = 1.0       # 서 있을 때는 두 발 모두 디딤 (발 속도 0 이 보상)
        lo, hi = self.jrange[:, 0] * 0.95, self.jrange[:, 1] * 0.95
        t = dict(
            keep_balance=np.ones(self.N),
            tracking_lin_vel=np.exp(-np.sum((self.cmd[:, :2] - lin[:, :2]) ** 2, 1) / 0.05),   # LimX 0.2 → 0.05: 옆 흐름·속도 부족을 더 강하게
            tracking_ang_vel=np.exp(-(self.cmd[:, 2] - gyro[:, 2]) ** 2 / self.ang_sigma),
            base_height=(h - BASE_HEIGHT_TARGET[self.foot]) ** 2,
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
        r = sum(self.rs[k] * v for k, v in t.items()) * POLICY_DT
        return r, t
