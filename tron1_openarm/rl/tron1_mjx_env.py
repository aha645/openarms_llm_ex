"""TRON1 + OpenArm 보행 강화학습 환경 — MJX(GPU) 판.

tron1_env.py(CPU, mujoco.rollout)와 같은 관측·행동·보상·명령·팔 동작·도메인 랜덤화 규약을 순수 JAX 함수로 옮겼다.
환경 수천 개를 jax.vmap 으로 한꺼번에 GPU 에서 진행하고, 수집(rollout)까지 jit 안에서 끝나 CPU↔GPU 복사가 없다.

CPU 판과 다른 점 (mjx_model.py)
  * SF 발 충돌: 메시 → 평평한 발바닥 상자, 바닥과는 발만 닿음
  * 접촉 솔버 반복 상한 16 / 선 탐색 16 (CPU 기본 100 / 50). MJX 속도의 대부분을 좌우한다.
    4 / 8 이면 CPU 에서도 이 모델이 발산(NaN)했고, 8 은 3~8 % 낙상, 16 부터 기본값과 같은 결과 (mjx_parity.py)
  * 도메인 랜덤화: CPU 판은 변형 모델 7개를 돌려 썼지만, 여기서는 환경마다 따로 뽑는다 (질량·무게중심·마찰·PD 게인)
    난이도(difficulty) 비율만큼의 환경에만 적용하는 규칙은 같다.
"""
import math
import os
import sys
from typing import NamedTuple

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
from mujoco import mjx

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "tron1_openarm_sim"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mjx_model import build_mjx_model  # noqa: E402
from tron1_env import REWARD_SCALES, STAND_DRIFT_SCALE, STAND_DRIFT_SIGMA  # noqa: E402  보상 계수는 CPU 판과 공유
from tron1_openarm_sim.arm_poses import POSES as ARM_POSES  # noqa: E402
from tron1_openarm_sim.locomotion_policy import PF, SF, SF_BASE_HEIGHT_CMD  # noqa: E402

ACTION_SCALE = 0.25
POLICY_DT = 0.02
CMD_SCALE = jnp.array([2.0, 2.0, 0.25], jnp.float32)
BASE_HEIGHT_TARGET = {"pf": 0.68, "sf": SF_BASE_HEIGHT_CMD}
CMD_RANGES = dict(vx=(-0.35, 0.6), vy=(-0.2, 0.2), wz=(-0.9, 0.9))


class EnvState(NamedTuple):
    d: mjx.Data
    last_action: jax.Array
    last_action2: jax.Array
    last_target: jax.Array
    gait_index: jax.Array
    cmd: jax.Array
    want_still: jax.Array
    standing: jax.Array
    hist: jax.Array
    obs: jax.Array           # 마지막 관측 (잡음 포함, hist 의 마지막 항목과 같음)
    t: jax.Array
    delay: jax.Array
    arm_from: jax.Array
    arm_to: jax.Array
    arm_t0: jax.Array
    arm_T: jax.Array
    ep_ret: jax.Array
    key: jax.Array


class RandParams(NamedTuple):
    """환경마다 한 번 뽑아 두는 모델 변형 (난이도에 따라 켜고 끔)."""
    dmass: jax.Array       # 몸통 질량 변화 [kg]
    dcom: jax.Array        # 몸통 무게중심 이동 (3) [m]
    friction: jax.Array    # 접촉 마찰 계수
    kp_scale: jax.Array    # 다리 PD 게인 배율
    kd_scale: jax.Array


class Tron1MjxEnv:
    def __init__(self, xml, foot="sf", iterations=16, ls_iterations=16, cone=None, episode_s=20.0, randomize=True, push=True,
                 impl="jax", num_envs=4096, contacts_per_env=16, njmax=128):
        """impl: "jax" (MJX 순수 JAX) 또는 "warp" (MuJoCo Warp 커널 — 같은 결과, RTX 3090 에서 수 배 빠름).
        warp 는 접촉 버퍼를 모든 환경이 함께 쓰므로 환경 수(num_envs)·환경당 접촉 상한을 미리 정한다.
        njmax 는 환경당 제약(접촉·관절 한계·등식) 행 수 상한 (서 있을 때 약 22, 팔이 관절 한계에 닿으면 더 늘어난다).
        64 로 두었을 때 4096 환경 학습이 도중에 NaN 으로 망가졌다 → 128."""
        self.foot = foot
        sp = PF if foot == "pf" else SF
        self.sp = sp
        self.nj, self.n_cmd, self.obs_dim, self.HIST = len(sp.joints), sp.n_cmd, sp.obs_dim, sp.hist
        self.KP, self.KD, self.TAU = (jnp.asarray(v, jnp.float32) for v in (sp.kp, sp.kd, sp.tau))
        self.q_def = jnp.asarray(sp.default_q, jnp.float32)
        self.GAIT = jnp.asarray(sp.gait, jnp.float32)
        self.rs = dict(REWARD_SCALES)
        self.ang_sigma = 0.1
        if foot == "sf":                       # CPU 판 SF 4차 설정과 같음
            self.rs["tracking_ang_vel"] = 6.0
            self.ang_sigma = 0.25
            self.rs["stand_drift"] = -STAND_DRIFT_SCALE
        self.randomize, self.push = randomize, push
        m, self.model_notes = build_mjx_model(xml, iterations=iterations, ls_iterations=ls_iterations, cone=cone)
        self.m = m
        self.impl = impl
        self.mx = mjx.put_model(m, impl=impl)
        self.dt = m.opt.timestep
        self.nsub = int(round(POLICY_DT / self.dt))
        self.ep_len = int(episode_s / POLICY_DT)
        J = sp.joints
        self.qadr = jnp.array([m.jnt_qposadr[m.joint(j).id] for j in J])
        self.vadr = jnp.array([m.jnt_dofadr[m.joint(j).id] for j in J])
        self.leg_act = jnp.array([m.actuator(j + "_ctrl").id for j in J])
        arm_act = [m.actuator(f"{s}_joint{i}_ctrl").id for s in ("left", "right") for i in range(1, 8)]
        self.arm_act = jnp.array(arm_act)
        self.arm_qadr = jnp.array([m.jnt_qposadr[m.joint(f"openarm_{s}_joint{i}").id]
                                   for s in ("left", "right") for i in range(1, 8)])
        self.finger_act = jnp.array([m.actuator(f"{s}_finger1_ctrl").id for s in ("left", "right")])
        self.arm_lo = jnp.asarray(m.actuator_ctrlrange[arm_act, 0], jnp.float32)
        self.arm_hi = jnp.asarray(m.actuator_ctrlrange[arm_act, 1], jnp.float32)
        jr = m.jnt_range[[m.joint(j).id for j in J]]
        self.jlo, self.jhi = jnp.asarray(jr[:, 0] * 0.95), jnp.asarray(jr[:, 1] * 0.95)

        def sadr(n):
            i = m.sensor(n).id
            return int(m.sensor_adr[i]), int(m.sensor_dim[i])
        self.S = {n: sadr(n) for n in ["imu_gyro", "imu_quat", "base_linvel", "foot_L_touch", "foot_R_touch",
                                        "foot_L_pos", "foot_R_pos", "foot_L_vel", "foot_R_vel"]}
        self.S_tau = jnp.array([sadr(j + "_tau")[0] for j in J])
        # 이름 자세 표 (자세 수, 14)
        names = list(ARM_POSES)
        self.poses = jnp.array([ARM_POSES[n][0] + ARM_POSES[n][1] for n in names], jnp.float32)
        # 랜덤화 대상 인덱스
        self.base_bid = m.body("base_link").id
        self.contact_geoms = jnp.array([g for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]])
        self.payload_bodies = jnp.array([i for i in range(m.nbody) if m.body(i).name.startswith("openarm_")])
        # 기준 상태
        if impl == "warp":
            self.d0 = mjx.make_data(m, impl="warp", naconmax=num_envs * contacts_per_env, njmax=njmax)
        else:
            d0 = mujoco.MjData(m)
            mujoco.mj_forward(m, d0)
            self.d0 = mjx.put_data(m, d0)
        self.qpos0 = jnp.asarray(m.qpos0, jnp.float32)

    # ------------------------------------------------------------------ 모델 (난이도·상반신 질량 반영, 환경별)
    def sample_rand(self, key, n):
        k = jax.random.split(key, 5)
        return RandParams(
            dmass=jax.random.uniform(k[0], (n,), minval=-0.5, maxval=2.0),
            dcom=jax.random.uniform(k[1], (n, 3), minval=jnp.array([-0.02, -0.015, -0.02]), maxval=jnp.array([0.02, 0.015, 0.02])),
            friction=jax.random.uniform(k[2], (n,), minval=0.4, maxval=1.2),
            kp_scale=jax.random.uniform(k[3], (n,), minval=0.85, maxval=1.15),
            kd_scale=jax.random.uniform(k[4], (n,), minval=0.85, maxval=1.15))

    def batched_model(self, rp: RandParams, difficulty, payload):
        """환경별 모델 필드와 vmap in_axes. 앞쪽 round(N·difficulty) 개 환경만 랜덤화 (CPU 판 규칙)."""
        mx = self.mx
        n = rp.dmass.shape[0]
        on = (jnp.arange(n) < jnp.round(n * difficulty)).astype(jnp.float32) * float(self.randomize)
        bm = jnp.tile(mx.body_mass, (n, 1))
        bm = bm.at[:, self.base_bid].add(on * rp.dmass)
        bm = bm.at[:, self.payload_bodies].multiply(payload)
        bi = jnp.tile(mx.body_inertia, (n, 1, 1))
        bi = bi.at[:, self.payload_bodies].set(mx.body_inertia[self.payload_bodies] * payload + 1e-6 * (1 - payload))
        bp = jnp.tile(mx.body_ipos, (n, 1, 1)).at[:, self.base_bid].add(on[:, None] * rp.dcom)
        fr0 = mx.geom_friction[:, 0]
        fr = jnp.tile(mx.geom_friction, (n, 1, 1))
        fr = fr.at[:, self.contact_geoms, 0].set(
            fr0[self.contact_geoms][None, :] * (1 - on[:, None]) + (on * rp.friction)[:, None])
        g = 1 + on * (rp.kp_scale - 1)
        kd = 1 + on * (rp.kd_scale - 1)
        gp = jnp.tile(mx.actuator_gainprm, (n, 1, 1)).at[:, self.leg_act, 0].set(self.KP[None, :] * g[:, None])
        bpm = jnp.tile(mx.actuator_biasprm, (n, 1, 1))
        bpm = bpm.at[:, self.leg_act, 1].set(-self.KP[None, :] * g[:, None])
        bpm = bpm.at[:, self.leg_act, 2].set(-self.KD[None, :] * kd[:, None])
        fields = dict(body_mass=bm, body_inertia=bi, body_ipos=bp, geom_friction=fr, actuator_gainprm=gp, actuator_biasprm=bpm)
        mb = mx.tree_replace(fields)
        axes = jax.tree_util.tree_map(lambda _: None, mx).tree_replace({k: 0 for k in fields})
        return mb, axes

    # ------------------------------------------------------------------ 보조
    def _sens(self, d, name):
        a, n = self.S[name]
        return d.sensordata[a:a + n]

    def _sample_arm(self, key, difficulty):
        k = jax.random.split(key, 8)
        P = self.poses.shape[0]
        arm_level = jnp.maximum(difficulty, 0.6)
        r = jax.random.uniform(k[0])
        iL, iR = jax.random.randint(k[1], (2,), 0, P)
        iR = jnp.where(jax.random.uniform(k[2]) < 0.5, iL, iR)
        named = jnp.concatenate([self.poses[iL, :7], self.poses[iR, 7:]])
        a, b = jax.random.randint(k[3], (2, 2), 0, P)
        lam = jax.random.uniform(k[4], (2,))
        mix = jnp.concatenate([lam[0] * self.poses[a[0], :7] + (1 - lam[0]) * self.poses[b[0], :7],
                               lam[1] * self.poses[a[1], 7:] + (1 - lam[1]) * self.poses[b[1], 7:]])
        mix = mix + 0.25 * jax.random.normal(k[5], (14,))
        q = jnp.where(r > arm_level, jnp.zeros(14), jnp.where(r < 0.7 * arm_level, named, mix))
        return jnp.clip(q, self.arm_lo, self.arm_hi)

    def _sample_cmd(self, key):
        k = jax.random.split(key, 14)
        r = CMD_RANGES
        c = jnp.array([jax.random.uniform(k[0], minval=r["vx"][0], maxval=r["vx"][1]),
                       jax.random.uniform(k[1], minval=r["vy"][0], maxval=r["vy"][1]) * (jax.random.uniform(k[2]) < 0.3),
                       jax.random.uniform(k[3], minval=r["wz"][0], maxval=r["wz"][1])])
        z = jax.random.uniform(k[4])
        c = jnp.where(z < 0.25, 0.0, c)
        c = jnp.where((z >= 0.25) & (z < 0.45), c * jnp.array([1.0, 0.0, 0.0]), c)
        c = jnp.where((z >= 0.45) & (z < 0.6), c * jnp.array([0.0, 0.0, 1.0]), c)
        if self.foot == "sf":
            turn = jnp.array([0.0, 0.0, jax.random.uniform(k[5], minval=0.3, maxval=0.9) * jnp.where(jax.random.uniform(k[6]) < 0.5, -1.0, 1.0)])
            c = jnp.where((z >= 0.75) & (z < 0.9), turn, c)
            slow = jnp.array([jax.random.uniform(k[7], minval=-0.12, maxval=0.12),
                              jax.random.uniform(k[8], minval=-0.08, maxval=0.08) * (jax.random.uniform(k[9]) < 0.5),
                              jax.random.uniform(k[10], minval=-0.3, maxval=0.3) * (jax.random.uniform(k[11]) < 0.5)])
            c = jnp.where((z >= 0.6) & (z < 0.75), slow, c)
            c = jnp.where(jnp.abs(c) < 0.015, 0.0, c)
            want_still = jnp.all(c == 0) & (jax.random.uniform(k[12]) < 0.8)
        else:
            c = jnp.where(jnp.abs(c) < 0.05, 0.0, c)
            want_still = jnp.array(False)
        return c.astype(jnp.float32), want_still

    def command_vector(self, s):
        c = s.cmd * CMD_SCALE
        if self.foot == "pf":
            return c
        still = s.standing.astype(jnp.float32)
        return jnp.concatenate([c * (1 - still), jnp.array([SF_BASE_HEIGHT_CMD], jnp.float32), still[None]])

    def _obs_raw(self, s):
        d = s.d
        q = d.qpos[self.qadr]
        dq = d.qvel[self.vadr]
        quat = self._sens(d, "imu_quat")
        quat = jnp.where(jnp.linalg.norm(quat) < 0.5, d.qpos[3:7], quat)
        gyro = self._sens(d, "imu_gyro")
        grav = quat_rot_inv(quat, jnp.array([0.0, 0.0, -1.0]))
        clock = jnp.array([jnp.sin(2 * jnp.pi * s.gait_index), jnp.cos(2 * jnp.pi * s.gait_index)])
        return jnp.concatenate([gyro * 0.25, grav, q - self.q_def, dq * 0.05, s.last_action, clock, self.GAIT]).astype(jnp.float32)

    def _noise(self, key, obs, difficulty):
        nj = self.nj
        sc = jnp.concatenate([jnp.full(3, 0.2 * 0.25), jnp.full(3, 0.05), jnp.full(nj, 0.01), jnp.full(nj, 1.5 * 0.05),
                              jnp.zeros(nj + 6)])
        return obs + jax.random.uniform(key, obs.shape, minval=-1, maxval=1) * sc * difficulty * float(self.randomize)

    # ------------------------------------------------------------------ reset / observe
    def reset(self, key, difficulty):
        k = jax.random.split(key, 6)
        arm = self._sample_arm(k[0], difficulty)
        yaw = jax.random.uniform(k[1], minval=-math.pi, maxval=math.pi)
        q = self.qpos0.at[3:7].set(jnp.array([jnp.cos(yaw / 2), 0, 0, jnp.sin(yaw / 2)]))
        q = q.at[self.qadr].set(self.q_def + jax.random.uniform(k[2], (self.nj,), minval=-0.05, maxval=0.05))
        q = q.at[self.arm_qadr].set(arm)
        ctrl = jnp.zeros_like(self.d0.ctrl).at[self.arm_act].set(arm).at[self.finger_act].set(0.044).at[self.leg_act].set(self.q_def)
        d = self.d0.replace(qpos=q, qvel=jnp.zeros_like(self.d0.qvel), ctrl=ctrl, time=jnp.zeros_like(self.d0.time),
                            sensordata=jnp.zeros_like(self.d0.sensordata))
        cmd, still = self._sample_cmd(k[3])
        delay = jax.random.randint(k[4], (), 0, 1 + jnp.round(3 * difficulty).astype(jnp.int32)) * int(self.randomize)
        z = jnp.zeros(self.nj, jnp.float32)
        s = EnvState(d=d, last_action=z, last_action2=z, last_target=self.q_def, gait_index=jnp.array(0.0),
                     cmd=cmd, want_still=still, standing=jnp.array(False), hist=jnp.zeros(self.HIST * self.obs_dim, jnp.float32),
                     obs=jnp.zeros(self.obs_dim, jnp.float32),
                     t=jnp.array(0, jnp.int32), delay=delay, arm_from=arm, arm_to=arm, arm_t0=jnp.array(0.0),
                     arm_T=jnp.array(1.0), ep_ret=jnp.array(0.0), key=k[5])
        obs = self._obs_raw(s)
        return s._replace(obs=obs, hist=jnp.tile(obs, self.HIST))

    def observe(self, s):
        """(obs, 관측 기록, 명령, 특권정보=실제 선속도×2). CPU 판 observe() 와 같은 의미.
        관측(잡음 포함)과 기록은 step() 이 갱신해 둔 것을 읽는다."""
        lin = jnp.nan_to_num(self._sens(s.d, "base_linvel") * 2.0)
        return s.obs, s.hist, self.command_vector(s), lin.astype(jnp.float32)

    # ------------------------------------------------------------------ step
    def step(self, mx, s: EnvState, action, difficulty):
        key, k_arm, k_new, k_push, k_pv, k_cmd, k_reset, k_noise = jax.random.split(s.key, 8)
        d = s.d
        a = jnp.clip(action, -100, 100).astype(jnp.float32)
        q = d.qpos[self.qadr]
        dq = d.qvel[self.vadr]
        qd = q - self.q_def
        lo = (qd + (self.KD * dq - self.TAU) / self.KP) / ACTION_SCALE
        hi = (qd + (self.KD * dq + self.TAU) / self.KP) / ACTION_SCALE
        a = jnp.clip(a, lo, hi)
        target = a * ACTION_SCALE + self.q_def
        # 팔: 3~6초마다 새 자세로 1.5~3초에 걸쳐 이동
        tsec = s.t * POLICY_DT
        new = ((tsec - s.arm_t0) > s.arm_T) & (jax.random.uniform(k_new) < POLICY_DT / 3.0 * jnp.maximum(difficulty, 0.6))
        cur = self._arm_now(s, tsec)
        k_a1, k_a2 = jax.random.split(k_arm)
        arm_from = jnp.where(new, cur, s.arm_from)
        arm_to = jnp.where(new, self._sample_arm(k_a1, difficulty), s.arm_to)
        arm_t0 = jnp.where(new, tsec, s.arm_t0)
        arm_T = jnp.where(new, jax.random.uniform(k_a2, minval=1.5, maxval=3.0), s.arm_T)
        s = s._replace(arm_from=arm_from, arm_to=arm_to, arm_t0=arm_t0, arm_T=arm_T)
        arm = self._arm_now(s, tsec)
        ctrl = d.ctrl.at[self.arm_act].set(arm)
        # 밀기: 평균 7초마다 몸통 속도 ±0.5 m/s × 난이도
        if self.push and self.randomize:
            pz = jax.random.uniform(k_push) < POLICY_DT / 7.0
            dv = jax.random.uniform(k_pv, (2,), minval=-0.5, maxval=0.5) * difficulty * pz
            d = d.replace(qvel=d.qvel.at[0:2].add(dv))

        def sub(d, i):      # 행동 지연: 처음 delay 스텝은 이전 목표
            leg = jnp.where(i < s.delay, s.last_target, target)
            d = d.replace(ctrl=ctrl.at[self.leg_act].set(leg))
            return mjx.step(mx, d), None
        d, _ = jax.lax.scan(sub, d, jnp.arange(self.nsub))
        s = s._replace(d=d, last_target=target)
        s = self._advance_clock(s)
        s = s._replace(t=s.t + 1)
        # 물리가 발산한 환경(NaN·무한대)은 넘어짐으로 처리하고 이번 스텝 값을 0 으로 막는다.
        # 하나라도 NaN 이 섞이면 보상 → 이점(advantage) 정규화 → 기울기로 번져 정책 전체가 NaN 이 된다
        bad = ~(jnp.all(jnp.isfinite(d.qpos)) & jnp.all(jnp.isfinite(d.qvel)) & jnp.all(jnp.isfinite(d.sensordata)))
        rew, terms = self._reward(s, a)
        rew = jnp.where(bad, 0.0, rew)
        terms = {k: jnp.where(bad, 0.0, v) for k, v in terms.items()}
        s = s._replace(last_action2=s.last_action, last_action=a)
        h = d.qpos[2]
        quat = d.qpos[3:7]
        upz = 1 - 2 * (quat[1] ** 2 + quat[2] ** 2)
        fell = (h < 0.45) | (upz < 0.7) | bad
        timeout = s.t >= self.ep_len
        rew = rew + fell * self.rs["termination"]
        ep_ret = s.ep_ret + rew
        done = fell | timeout
        info = {"fell": fell, "bad": bad, "timeout": timeout, "terms": terms, "ep_ret": ep_ret, "ep_t": s.t}
        # 명령 재설정 (5초마다)
        rs = (s.t % 250 == 0) & ~done
        c, still = self._sample_cmd(k_cmd)
        s = s._replace(cmd=jnp.where(rs, c, s.cmd), want_still=jnp.where(rs, still, s.want_still),
                       standing=jnp.where(rs, False, s.standing), ep_ret=ep_ret, key=key)
        # 관측 기록 갱신 (CPU 판은 observe() 에서 하던 일)
        obs = jnp.nan_to_num(self._noise(k_noise, self._obs_raw(s), difficulty), nan=0.0, posinf=0.0, neginf=0.0)
        s = s._replace(obs=obs, hist=jnp.concatenate([s.hist[self.obs_dim:], obs]))
        # 끝난 환경은 새로 시작
        r = self.reset(k_reset, difficulty)
        s = self._select(done, r, s)
        return s, rew.astype(jnp.float32), done, info

    # 리셋 때 바꾸는 물리 상태. 나머지(접촉·제약·중간 계산 값)는 다음 mjx.step 이 처음부터 다시 계산한다.
    # Data 전체를 where 로 섞으면 환경마다 수십 KB 를 복사하고, warp 의 공용 접촉 버퍼와도 맞지 않는다
    RESET_FIELDS = ("time", "qpos", "qvel", "act", "ctrl", "qacc_warmstart", "qfrc_applied", "xfrc_applied", "sensordata")

    def _select(self, done, r, s):
        d = s.d.replace(**{f: jnp.where(done, getattr(r.d, f), getattr(s.d, f)) for f in self.RESET_FIELDS})
        rest = jax.tree_util.tree_map(lambda x, y: jnp.where(done, x, y), r._replace(d=None), s._replace(d=None))
        return rest._replace(d=d)

    def _arm_now(self, s, tsec):
        u = jnp.clip((tsec - s.arm_t0) / s.arm_T, 0, 1)
        u = u * u * (3 - 2 * u)
        return s.arm_from + (s.arm_to - s.arm_from) * u

    def _advance_clock(self, s):
        g = s.gait_index
        if self.foot == "sf":
            win = s.want_still & ~s.standing & ((g < 0.05) | (g >= 0.95) | ((g >= 0.5) & (g < 0.55)))
            g = jnp.where(win, jnp.where((g >= 0.5) & (g < 0.55), 0.5, 0.0), g)
            standing = (s.standing | win) & s.want_still
            g = jnp.where(standing, g, (g + POLICY_DT * self.GAIT[0]) % 1.0)
            return s._replace(gait_index=g, standing=standing)
        return s._replace(gait_index=(g + POLICY_DT * self.GAIT[0]) % 1.0)

    def _reward(self, s, a):
        d = s.d
        quat = d.qpos[3:7]
        lin = self._sens(d, "base_linvel")
        gyro = self._sens(d, "imu_gyro")
        grav = quat_rot_inv(quat, jnp.array([0.0, 0.0, -1.0]))
        h = d.qpos[2]
        q = d.qpos[self.qadr]
        tau = d.sensordata[self.S_tau]
        F = jnp.array([self._sens(d, "foot_L_touch")[0], self._sens(d, "foot_R_touch")[0]])
        fv = jnp.array([jnp.linalg.norm(self._sens(d, "foot_L_vel")), jnp.linalg.norm(self._sens(d, "foot_R_vel"))])
        fp = self._sens(d, "foot_L_pos")[:2] - self._sens(d, "foot_R_pos")[:2]
        des = desired_contacts(s.gait_index, self.GAIT)
        if self.foot == "sf":
            des = jnp.where(s.standing, 1.0, des)
        t = dict(
            keep_balance=jnp.array(1.0),
            tracking_lin_vel=jnp.exp(-jnp.sum((s.cmd[:2] - lin[:2]) ** 2) / 0.05),
            tracking_ang_vel=jnp.exp(-(s.cmd[2] - gyro[2]) ** 2 / self.ang_sigma),
            base_height=(h - BASE_HEIGHT_TARGET[self.foot]) ** 2,
            lin_vel_z=lin[2] ** 2,
            ang_vel_xy=jnp.sum(gyro[:2] ** 2),
            torques=jnp.sum(tau ** 2),
            action_rate=jnp.sum((a - s.last_action) ** 2),
            action_smooth=jnp.sum((a - 2 * s.last_action + s.last_action2) ** 2),
            orientation=jnp.sum(grav[:2] ** 2),
            dof_pos_limits=jnp.sum(jnp.clip(self.jlo - q, 0, None) + jnp.clip(q - self.jhi, 0, None)),
            feet_distance=jnp.clip(0.115 - jnp.linalg.norm(fp), 0, 1),
            contact_force=jnp.mean((1 - des) * (1 - jnp.exp(-F ** 2 / 25.0))),
            contact_vel=jnp.mean(des * (1 - jnp.exp(-fv ** 2 / 0.25))),
            stand_drift=s.standing * (1 - jnp.exp(-(gyro[2] / STAND_DRIFT_SIGMA[0]) ** 2
                                                  - jnp.sum(lin[:2] ** 2) / STAND_DRIFT_SIGMA[1] ** 2)),
        )
        r = sum(self.rs[k] * v for k, v in t.items()) * POLICY_DT
        return r, t


def quat_rot_inv(q, v):
    """q (wxyz) → R(q)^T v."""
    w, qv = q[0], q[1:]
    return v * (2.0 * w ** 2 - 1.0) - jnp.cross(qv, v) * w * 2.0 + qv * jnp.dot(qv, v) * 2.0


def _ncdf(x, s=0.05):
    return 0.5 * (1.0 + jax.scipy.special.erf(x / (s * math.sqrt(2))))


def desired_contacts(gait_index, gait):
    """LimX base_task._step_contact_targets 와 같은 식 (왼발, 오른발 접지 기대값)."""
    off, dur = gait[1], gait[2]
    fi = jnp.array([gait_index, gait_index + off + 1.0]) % 1.0
    st = fi < dur
    fi = jnp.where(st, fi * (0.5 / dur), 0.5 + (fi - dur) * (0.5 / (1 - dur)))
    return _ncdf(fi) * (1 - _ncdf(fi - 0.5)) + _ncdf(fi - 1) * (1 - _ncdf(fi - 1.5))
