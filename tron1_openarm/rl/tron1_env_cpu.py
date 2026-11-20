"""TRON1 + OpenArm 보행 강화학습 환경 — CPU 판 (MuJoCo, mujoco.rollout C 스레드 병렬).
LimX tron1-rl-isaacgym 의 pointfoot_flat 과 같은 관측·행동·보상 규약을 MuJoCo 로 옮겼다.
달라진 점: 상반신(OpenArm 양팔 14.4 kg)이 얹혀 있고, 학습 중 팔 자세가 계속 바뀐다.

보상·명령·팔 자세·손 외력·에피소드 설정은 train_config.py 를 GPU 판(tron1_env_gpu.py)과 함께 쓴다.
기본값은 배포 중인 SF v3 정책을 만든 설정이다. 두 판은 물리를 어디서 돌리는지만 다르다:
  * 이 판: 메시·기본 접촉 솔버, 변형 모델 7개를 돌려 쓰는 도메인 랜덤화, 환경 수 백 개 (CPU 코어 수만큼 스레드)
  * GPU 판: 발만 바닥과 닿는 단순화 모델(mjx_model.py)·솔버 반복 16, 환경마다 따로 뽑는 랜덤화, 환경 수 천 개
* 정책 주기 50 Hz, 학습 물리 dt 4 ms (5 스텝). 배포(시뮬레이터)는 2 ms 로 다시 검증한다.
* 도메인 랜덤화: 몸통 질량(-0.5~+2 kg)·무게중심(±2 cm)·바닥 마찰(0.4~1.2)·PD 게인(±15%)·
  행동 지연(0~12 ms)·밀기(push)·관측 잡음·팔 자세(이름 자세 + 무작위 + 집기 자세)·서 있을 때 손 외력
"""
import math
import os
import sys

import mujoco
import numpy as np
from mujoco import rollout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mjx_model import build_cpu_model  # noqa: E402
from train_config import *  # noqa: E402,F401,F403  보상·명령·팔 자세·손 외력 설정 (GPU 학습과 공유)
from train_config import ARM_POSES, FEET_BODIES, PF, SF, SF_BASE_HEIGHT_CMD, episode_s, reach_p, reward_scales  # noqa: E402
from tron1_openarm_sim.arm_poses import mirror  # noqa: E402


def ncdf(x, s=0.05):
    return 0.5 * (1.0 + np.vectorize(math.erf)(x / (s * math.sqrt(2))))


def desired_contacts(gait_index, gait):
    """LimX base_task._step_contact_targets 와 같은 식. 반환 (N,2) 왼발·오른발 접지 기대값 0~1."""
    off, dur = gait[1], gait[2]
    fi = np.stack([gait_index, gait_index + off + 1.0], 1) % 1.0
    st = fi < dur
    fi = np.where(st, fi * (0.5 / dur), 0.5 + (fi - dur) * (0.5 / (1 - dur)))
    return ncdf(fi) * (1 - ncdf(fi - 0.5)) + ncdf(fi - 1) * (1 - ncdf(fi - 1.5))


def yaw_of_wxyz(q):
    return np.arctan2(2 * (q[..., 0] * q[..., 3] + q[..., 1] * q[..., 2]), 1 - 2 * (q[..., 2] ** 2 + q[..., 3] ** 2))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def quat_rot_inv(q, v):
    """q (N,4) wxyz, v (N,3) → R(q)^T v."""
    w, x, y, z = q[:, 0:1], q[:, 1:2], q[:, 2:3], q[:, 3:4]
    qv = np.concatenate([x, y, z], 1)
    a = v * (2.0 * w ** 2 - 1.0)
    b = np.cross(qv, v) * w * 2.0
    c = qv * np.sum(qv * v, 1, keepdims=True) * 2.0
    return a - b + c


def quat_to_mat(q):
    """q (N,4) wxyz → (N,3,3)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], 1),
                     np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], 1),
                     np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], 1)], 1)


class Tron1EnvCpu:
    def __init__(self, xml, num_envs=128, num_models=8, seed=0, nthread=2, randomize=True,
                 ep_s=None, cmd_ranges=None, push=True, foot="sf", load_model=None):
        # load_model: (xml, 발 몸체) → MjModel. 기본은 MJCF 그대로 + 발 센서 (mjx_model.build_cpu_model).
        #             MJX 와 같은 단순화 모델로 비교할 때 mjx_parity.py 가 build_mjx_model(feet=...) 을 넘긴다
        feet = FEET_BODIES[foot]
        load = (lambda x: load_model(x, feet)) if load_model else (lambda x: build_cpu_model(x, feet))
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
        self.rs, self.ang_sigma = reward_scales(foot)      # train_config.py (GPU 학습과 같은 값)
        base = load(xml)
        self.dt = base.opt.timestep
        self.nsub = int(round(POLICY_DT / self.dt))
        self.N = num_envs
        self.randomize = randomize
        self.push = push
        self.ep_len = int((episode_s(foot) if ep_s is None else ep_s) / POLICY_DT)   # 기본: SF 60 s / PF 20 s (train_config)
        self.reach_p = reach_p(foot)
        self.cmd_ranges = cmd_ranges or CMD_RANGES
        # --- 인덱스
        self.qadr = np.array([base.jnt_qposadr[base.joint(j).id] for j in LEG])
        self.vadr = np.array([base.jnt_dofadr[base.joint(j).id] for j in LEG])
        self.leg_act = np.array([base.actuator(j + "_ctrl").id for j in LEG])
        self.arm_act = {s: np.array([base.actuator(f"{s}_joint{i}_ctrl").id for i in range(1, 8)])
                        for s in ("left", "right")}
        self.arm_qadr = {s: np.array([base.jnt_qposadr[base.joint(f"openarm_{s}_joint{i}").id] for i in range(1, 8)])
                         for s in ("left", "right")}
        self.finger_act = [base.actuator(f"{s}_finger1_ctrl").id for s in ("left", "right")]
        self.hand_bid = [base.body(f"openarm_{s}_hand").id for s in ("left", "right")]
        self.jrange = base.jnt_range[[base.joint(j).id for j in LEG]].copy()
        self.arm_range = {s: base.actuator_ctrlrange[self.arm_act[s]].copy() for s in self.arm_act}
        self.arm_lo = np.concatenate([self.arm_range["left"][:, 0], self.arm_range["right"][:, 0]])
        self.arm_hi = np.concatenate([self.arm_range["left"][:, 1], self.arm_range["right"][:, 1]])
        # 집기·들기 자세 (train_config.REACH_R, 오른팔 기준 → 왼팔은 거울상), 다른 팔은 home/carry
        self.reach_r = np.array(REACH_R, float)
        self.reach_l = np.array([mirror(q) for q in REACH_R], float)
        self.rest_r = np.array([ARM_POSES[n][1] for n in ("home", "carry")], float)
        self.rest_l = np.array([ARM_POSES[n][0] for n in ("home", "carry")], float)

        def sadr(n):
            i = base.sensor(n).id
            return slice(base.sensor_adr[i], base.sensor_adr[i] + base.sensor_dim[i])
        self.s = {n: sadr(n) for n in ["imu_gyro", "imu_quat", "base_linvel", "foot_L_touch", "foot_R_touch",
                                        "foot_L_pos", "foot_R_pos", "foot_L_vel", "foot_R_vel",
                                        "train_foot_L_xpos", "train_foot_R_xpos", "train_foot_L_angvel", "train_foot_R_angvel"]}
        self.s_tau = [sadr(j + "_tau") for j in LEG]
        # --- 모델 변형 (도메인 랜덤화)
        self.models = []
        bid = base.body("base_link").id
        for k in range(num_models):
            m = load(xml)
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
        # 매 물리 스텝 넣는 값: 구동기 명령 + 몸체 외력(손 외력용 xfrc_applied)
        self.ctrl_spec = int(mujoco.mjtState.mjSTATE_CTRL) | int(mujoco.mjtState.mjSTATE_XFRC_APPLIED)
        self.ns = mujoco.mj_stateSize(base, self.spec)
        d = mujoco.MjData(base)
        mujoco.mj_forward(base, d)
        self.init_state = np.zeros(self.ns)
        mujoco.mj_getState(base, d, self.init_state, self.spec)
        self.q0 = 1                                   # state 안에서 qpos 시작 (0 = time)
        self.v0 = 1 + base.nq
        self.nu = base.nu
        self.nbody = base.nbody
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
        self.stand_t = np.zeros(N)              # SF: 서기 시작 뒤 경과 시간 [s]
        self.yaw0 = np.zeros(N)                 # SF: 서기 방향 기준 (서기 시작 STAND_YAW_SETTLE 뒤의 방향)
        self.walk_yaw = np.zeros(N)             # SF: 직진 방향 기준 (walk_heading)
        self.stand_xy = np.zeros((N, 2))        # SF: 서기 위치 기준 (stand_sway)
        self.v_avg = np.zeros((N, 2))           # 몸통 수평 속도 0.5 s 평균 (tracking_slow)
        self.hand_f = np.zeros((N, 2, 3))       # 손에 거는 힘 (왼손, 오른손) × xyz [N] (세계 기준)
        self.f_until = np.zeros(N, np.int64)    # 지금 힘(또는 쉬는 구간)이 끝나는 스텝
        self.cmd_until = np.full(N, 250, np.int64)   # 다음 명령 재설정 스텝
        self.hist = np.zeros((N, self.HIST * self.obs_dim), np.float32)
        self.t = np.zeros(N, np.int64)
        self.delay = np.zeros(N, np.int64)
        self.arm_from = np.zeros((N, 14))
        self.arm_to = np.zeros((N, 14))
        self.arm_t0 = np.zeros(N)
        self.arm_T = np.ones(N)
        self.ep_ret = np.zeros(N)
        self.difficulty = 0.0       # 0 → 1: 잡음·지연·밀기·팔 동작 강도 (train_ppo_cpu.py 가 낙상률을 보며 올림)
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
            if self.reach_p > 0 and self.rng.random() < self.reach_p:
                # 한 팔은 집기 자세(+작은 잡음), 다른 팔은 home/carry — 실제 집기처럼 (GPU 판과 같은 규칙)
                ir = self.rng.integers(len(self.reach_r))
                io = self.rng.integers(2)
                nz = 0.05 * self.rng.normal(size=7)
                if self.rng.random() < 0.5:
                    q = np.concatenate([self.rest_l[io], self.reach_r[ir] + nz])
                else:
                    q = np.concatenate([self.reach_l[ir] + nz, self.rest_r[io]])
            out[i] = np.clip(q, self.arm_lo, self.arm_hi)
        return out

    def _sample_cmd(self):
        """명령 하나 → (cmd[3], want_still, dur). GPU 판 _sample_cmd 와 같은 분포."""
        rng, r = self.rng, self.cmd_ranges

        def gen():
            return np.array([rng.uniform(*r["vx"]), rng.uniform(*r["vy"]) * (rng.random() < VY_P), rng.uniform(*r["wz"])])
        c = gen()
        z = rng.random()
        if z < 0.25:
            c = np.zeros(3)                                 # 25%: 제자리(명령 0) — Nav2 정지·도킹 대기
        elif z < 0.45:
            c = c * [1.0, 0.0, 0.0]                         # 20%: 직진만 (가장 흔한 주행)
        elif z < 0.6:
            c = c * [0.0, 0.0, 1.0]                         # 15%: 제자리 회전 (도킹 yaw 보정)
        if self.foot != "sf":
            c[np.abs(c) < 0.05] = 0.0
            return c, False, 250
        turn = np.array([0.0, 0.0, rng.uniform(0.3, 0.9) * (-1.0 if rng.random() < 0.5 else 1.0)])
        # 느린 미세 이동 (도킹·옆걸음·작은 회전, |vx| ≤ 0.12, |vy| ≤ 0.08, |wz| ≤ 0.3)
        slow = np.array([rng.uniform(-0.12, 0.12), rng.uniform(-0.08, 0.08) * (rng.random() < 0.5),
                         rng.uniform(-0.3, 0.3) * (rng.random() < 0.5)])
        if 0.75 <= z < 0.9:
            c = turn                                        # SF: 제자리 회전 15% 더 (합 30%)
        elif 0.6 <= z < 0.75:
            c = slow                                        # SF: 15% 느린 미세 이동
        if CMD_MIX in ("v7", "v9", "v10"):
            g = gen()
            side = np.array([0.0, rng.uniform(0.1, r["vy"][1]) * (-1.0 if rng.random() < 0.5 else 1.0), 0.0])
            diag = np.array([rng.uniform(*r["vx"]), rng.uniform(*r["vy"]), 0.0])
            edges = {"v7": (0.20, 0.35, 0.50, 0.60, 0.70, 0.85), "v9": (0.30, 0.42, 0.57, 0.67, 0.75, 0.87),
                     "v10": (0.25, 0.35, 0.48, 0.57, 0.72, 0.85)}[CMD_MIX]
            choices = [np.zeros(3), g * [1.0, 0.0, 0.0], side, diag, slow, turn]
            c = next((ch for e, ch in zip(edges, choices) if z < e), g)
        c = np.where(np.abs(c) < 0.015, 0.0, c)
        # SF: 명령 0 인 경우의 80% 는 '제자리 서기'(발 안 떼기) — 조작할 때 쓰는 모드. 나머지는 제자리 걸음
        want_still = bool(np.all(c == 0) and rng.random() < 0.8)
        long = want_still and rng.random() < STAND_LONG_P
        dur = int(rng.integers(STAND_LONG_RANGE[0], STAND_LONG_RANGE[1] + 1)) if long else 250
        return c, want_still, dur

    def _resample_cmd(self, idx):
        for i in idx:
            c, still, dur = self._sample_cmd()
            self.cmd[i] = c
            self.want_still[i] = still
            self.standing[i] = False
            self.stand_t[i] = 0.0
            self.cmd_until[i] = self.t[i] + dur

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
            self.walk_yaw[i] = yaw
            self.stand_xy[i] = q[0:2]
        self.state[idx] = st
        self.ctrl[idx] = 0
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
        self.v_avg[idx] = 0
        self.hand_f[idx] = 0
        self.f_until[idx] = 0
        self.yaw0[idx] = 0
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
        """정책 입력 재료: (obs, 관측 기록, 명령, 특권정보=실제 선속도×2 3)."""
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
        """SF 제자리 서기: 두 발 다 디딘 위상(0 또는 0.5 근처)에 오면 시계를 멈춘다 (LimX 와 같은 규칙).
        서기·직진 기준 방향과 서기 기준 위치는 STAND_YAW_TAU 로 천천히 따라간다 (GPU 판과 같음)."""
        if self.foot != "sf":
            self.gait_index = (self.gait_index + POLICY_DT * self.GAIT[0]) % 1.0
            return
        g = self.gait_index
        win = self.want_still & ~self.standing & ((g < 0.05) | (g >= 0.95) | ((g >= 0.5) & (g < 0.55)))
        g = np.where(win, np.where((g >= 0.5) & (g < 0.55), 0.5, 0.0), g)
        self.standing = (self.standing | win) & self.want_still
        self.gait_index = np.where(self.standing, g, (g + POLICY_DT * self.GAIT[0]) % 1.0)
        self.stand_t = np.where(self.standing, self.stand_t + POLICY_DT, 0.0)
        yaw = yaw_of_wxyz(self.state[:, self.q0 + 3:self.q0 + 7])
        follow = POLICY_DT / STAND_YAW_TAU if STAND_YAW_TAU > 0 else 0.0
        self.yaw0 = np.where(self.stand_t < STAND_YAW_SETTLE, yaw, self.yaw0 + follow * wrap(yaw - self.yaw0))
        straight = ~self.standing & (np.abs(self.cmd[:, 2]) < 0.05)
        self.walk_yaw = np.where(straight, self.walk_yaw + follow * wrap(yaw - self.walk_yaw), yaw)
        xy = self.state[:, self.q0:self.q0 + 2]
        settle = (self.stand_t < STAND_YAW_SETTLE)[:, None]
        self.stand_xy = np.where(settle, xy, self.stand_xy + follow * (xy - self.stand_xy))
        self.v_avg = self.v_avg + (POLICY_DT / SLOW_TAU) * (self.sens[:, self.s["base_linvel"]][:, :2] - self.v_avg)

    def _hand_force(self):
        """손 외력: 힘을 거는 구간(0.3~3 s)과 쉬는 구간(0.5~3 s)을 번갈아. 크기 0~HAND_F_MAX × 난이도(0.3~1),
        방향은 몸통 기준 앞쪽·아래쪽이 더 많게 (상자를 밀고, 작업대를 누르는 경우). GPU 판 _hand_force 와 같은 규칙."""
        rng = self.rng
        due = np.nonzero(self.t >= self.f_until)[0]
        level = float(np.clip(2.0 * self.difficulty, 0.3, 1.0))
        yaw = yaw_of_wxyz(self.state[:, self.q0 + 3:self.q0 + 7])
        for i in due:
            was_on = bool(np.any(self.hand_f[i] != 0))
            if was_on:
                self.hand_f[i] = 0.0
                dur = rng.uniform(0.5, 3.0)
            else:
                mag = rng.uniform(0.0, HAND_F_MAX) * level
                v = np.array([rng.uniform(-0.5, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 0.3)])
                v /= max(np.linalg.norm(v), 1e-3)
                c, sn = math.cos(yaw[i]), math.sin(yaw[i])
                f = np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1], v[2]]) * mag
                which = rng.integers(5)                     # 0,1: 왼손 / 2,3: 오른손 / 4: 두 손 (각 절반)
                self.hand_f[i, 0] = f if which <= 1 else (0.5 * f if which == 4 else 0.0)
                self.hand_f[i, 1] = f if which in (2, 3) else (0.5 * f if which == 4 else 0.0)
                dur = rng.uniform(0.3, 3.0)
            self.f_until[i] = self.t[i] + int(dur / POLICY_DT)

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
        nc = self.nu + 6 * self.nbody
        ctrl = np.zeros((N, self.nsub, nc))
        ctrl[:, :, :self.nu] = self.ctrl[:, None, :]
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
        # 손 외력: 서 있고 서기 0.5 s 뒤부터 (SF)
        if HAND_F_MAX > 0 and self.foot == "sf":
            self._hand_force()
            on = (self.standing & (self.stand_t > 0.5)).astype(float)
            for h, b in enumerate(self.hand_bid):
                c0 = self.nu + 6 * b
                ctrl[:, :, c0:c0 + 3] = (self.hand_f[:, h] * on[:, None])[:, None, :]
        st, se = self.roll.rollout(self.model_list, self.datas, self.state, ctrl, control_spec=self.ctrl_spec,
                                   nstep=self.nsub)
        self.state = st[:, -1, :].copy()
        self.sens = se[:, -1, :].copy()
        self.ctrl[:, self.leg_act] = target
        self.ctrl[:, self.arm_act["left"]] = arm[:, :7]
        self.ctrl[:, self.arm_act["right"]] = arm[:, 7:]
        self.last_target = target
        self._advance_clock()
        self.t += 1

        bad = ~(np.isfinite(self.state).all(1) & np.isfinite(self.sens).all(1))
        rew, terms = self._reward(a)
        rew = np.where(bad, 0.0, rew)
        terms = {k: np.where(bad, 0.0, v) for k, v in terms.items()}
        self.last_action2 = self.last_action
        self.last_action = a
        h = self.state[:, self.q0 + 2]
        quat = self.state[:, self.q0 + 3:self.q0 + 7]
        upz = 1 - 2 * (quat[:, 1] ** 2 + quat[:, 2] ** 2)
        fell = (h < 0.45) | (upz < 0.7) | bad
        timeout = self.t >= self.ep_len
        rew = rew + fell * self.rs["termination"]
        self.ep_ret += rew
        done = fell | timeout
        # 명령 재설정 (5초마다)
        rs = (self.t >= self.cmd_until) & ~done
        self._resample_cmd(np.nonzero(rs)[0])
        info = {"fell": fell, "bad": bad, "timeout": timeout, "terms": terms, "ep_ret": self.ep_ret.copy(), "ep_t": self.t.copy()}
        self.reset(np.nonzero(done)[0])
        return rew.astype(np.float32), done, info

    def _arm_now(self, tsec):
        s = np.clip((tsec - self.arm_t0) / self.arm_T, 0, 1)
        s = s * s * (3 - 2 * s)
        return self.arm_from + (self.arm_to - self.arm_from) * s[:, None]

    def _reward(self, a):
        """보상 항 — GPU 판 Tron1EnvGpu._reward 와 같은 식·같은 계수 (train_config.reward_scales)."""
        S = self.sens
        quat = self.state[:, self.q0 + 3:self.q0 + 7]
        lin = S[:, self.s["base_linvel"]]
        gyro = S[:, self.s["imu_gyro"]]
        grav = quat_rot_inv(quat, np.tile([0.0, 0.0, -1.0], (self.N, 1)))
        h = self.state[:, self.q0 + 2]
        q = self.state[:, self.q0:][:, self.qadr]
        tau = np.stack([S[:, s][:, 0] for s in self.s_tau], 1)
        F = np.stack([S[:, self.s["foot_L_touch"]][:, 0], S[:, self.s["foot_R_touch"]][:, 0]], 1)
        fvel = np.stack([S[:, self.s["foot_L_vel"]], S[:, self.s["foot_R_vel"]]], 1)           # (N, 2, 3)
        fv = np.linalg.norm(fvel, axis=2)
        fvxy = np.linalg.norm(fvel[:, :, :2], axis=2)
        fp = S[:, self.s["foot_L_pos"]][:, :2] - S[:, self.s["foot_R_pos"]][:, :2]
        # 발 높이: 관절 각도 정기구학(몸통 기준 발 위치)을 IMU 자세로 돌려 '세계 위쪽' 높이로 (GPU 판과 같은 계산)
        base = self.state[:, self.q0:self.q0 + 3]
        Rb = quat_to_mat(quat)
        fx = np.stack([S[:, self.s["train_foot_L_xpos"]], S[:, self.s["train_foot_R_xpos"]]], 1)  # (N, 2, 3)
        p_b = np.einsum("nfi,nij->nfj", fx - base[:, None, :], Rb)                               # 몸통 기준
        Ri = quat_to_mat(S[:, self.s["imu_quat"]])
        fz = np.einsum("nfj,nkj->nfk", p_b, Ri)[:, :, 2]
        twist = np.stack([S[:, self.s["train_foot_L_angvel"]][:, 2], S[:, self.s["train_foot_R_angvel"]][:, 2]], 1)
        vcmd = np.linalg.norm(self.cmd[:, :2], axis=1)
        clear_h = np.clip(0.06 + 0.07 * vcmd, 0.06, self.GAIT[3])[:, None] if CLEAR_ADAPT else self.GAIT[3]
        settled = (self.standing & (self.stand_t >= STAND_YAW_SETTLE)).astype(np.float64)
        des = desired_contacts(self.gait_index, self.GAIT)
        if self.foot == "sf":
            des[self.standing] = 1.0       # 서 있을 때는 두 발 모두 디딤 (발 속도 0 이 보상)
        lo, hi = self.jrange[:, 0] * 0.95, self.jrange[:, 1] * 0.95
        yaw = yaw_of_wxyz(quat)
        walking_straight = (~self.standing & (np.abs(self.cmd[:, 2]) < 0.05)).astype(np.float64)
        slow_cmd = (~self.standing & (vcmd > 0.005) & (vcmd < 0.15)).astype(np.float64)
        Fmin = F.min(1)
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
            feet_distance=np.clip(0.115 - np.linalg.norm(fp, axis=1), 0, 1) * np.where(np.abs(self.cmd[:, 1]) > 0.05, FD_SIDE_SCALE, 1.0),
            contact_force=np.mean((1 - des) * (1 - np.exp(-F ** 2 / 25.0)), 1),
            contact_vel=np.mean(des * (1 - np.exp(-fv ** 2 / 0.25)), 1),
            stand_drift=self.standing * (1 - np.exp(-(gyro[:, 2] / STAND_DRIFT_SIGMA[0]) ** 2
                                                    - np.sum(lin[:, :2] ** 2, 1) / STAND_DRIFT_SIGMA[1] ** 2)),
            stand_yaw=(self.standing & (self.stand_t >= STAND_YAW_SETTLE))
            * np.minimum(np.abs(wrap(yaw - self.yaw0)) / STAND_YAW_REF, 2.0),
            stand_feet=settled * np.mean(1 - np.exp(-(fvxy / STAND_FEET_SIGMA) ** 2), 1),
            stand_twist=settled * np.mean(1 - np.exp(-(twist / STAND_TWIST_SIGMA) ** 2), 1),
            stand_load=settled * np.clip((STAND_LOAD_MIN - Fmin / np.maximum(F.sum(1), 1.0)) / STAND_LOAD_MIN, 0.0, 1.0),
            stand_tilt=settled * np.clip((np.linalg.norm(grav[:, :2], axis=1) - STAND_TILT_FREE) / STAND_TILT_REF, 0.0, 2.0),
            feet_clearance=np.sum((1 - des) * np.clip(clear_h - (fz - fz.min(1, keepdims=True)), 0.0, None) ** 2, 1),
            tracking_vy=np.exp(-(self.cmd[:, 1] - lin[:, 1]) ** 2 / TRACK_VY_SIGMA),
            walk_heading=walking_straight * np.minimum(np.abs(wrap(yaw - self.walk_yaw)) / WALK_HEADING_REF, WALK_HEADING_CAP),
            walk_yawrate=walking_straight * np.minimum((gyro[:, 2] / 0.05) ** 2, 4.0),
            stand_liftoff=settled * (Fmin < STAND_LIFT_F),
            stand_sway=settled * np.minimum(np.linalg.norm(self.state[:, self.q0:self.q0 + 2] - self.stand_xy, axis=1) / STAND_SWAY_REF, 2.0),
            tracking_slow=slow_cmd * np.exp(-(np.linalg.norm(self.cmd[:, :2] - self.v_avg, axis=1)
                                              / np.maximum(vcmd, 0.03)) ** 2 / 0.25),
            stand_planted=settled * (Fmin >= STAND_LIFT_F) * np.mean(np.exp(-(fvxy / STAND_FEET_SIGMA) ** 2), 1),
        )
        r = sum(self.rs[k] * v for k, v in t.items()) * POLICY_DT
        return r, t
