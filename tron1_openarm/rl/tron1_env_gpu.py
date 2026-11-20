"""TRON1 + OpenArm 보행 강화학습 환경 — GPU 판 (MJX / MuJoCo Warp).

tron1_env_cpu.py(CPU, mujoco.rollout)와 같은 관측·행동·보상·명령·팔 동작·손 외력·도메인 랜덤화 규약을 순수 JAX 함수로 옮겼다.
보상·명령·교란 설정은 train_config.py 를 CPU 판과 함께 쓴다.
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
from train_config import *  # noqa: E402,F401,F403  보상·명령·팔 자세·손 외력 설정 (CPU 학습과 공유)
from train_config import ARM_POSES, FEET_BODIES, PF, SF, SF_BASE_HEIGHT_CMD, episode_s, reach_p, reward_scales  # noqa: E402

CMD_SCALE = jnp.array([2.0, 2.0, 0.25], jnp.float32)


class EnvState(NamedTuple):
    d: mjx.Data
    last_action: jax.Array
    last_action2: jax.Array
    last_target: jax.Array
    gait_index: jax.Array
    cmd: jax.Array
    want_still: jax.Array
    standing: jax.Array
    stand_t: jax.Array       # SF: 서기 시작 뒤 경과 시간 [s]
    yaw0: jax.Array          # SF: 서기 방향 기준
    walk_yaw: jax.Array      # SF: 직진 방향 기준 (v6 walk_heading)
    stand_xy: jax.Array      # SF: 서기 위치 기준 (v9 stand_sway)
    v_avg: jax.Array         # 몸통 수평 속도 0.5 s 평균 (v10 느린 명령 추종)
    cmd_until: jax.Array     # 다음 명령 재설정 스텝
    hist: jax.Array
    obs: jax.Array           # 마지막 관측 (잡음 포함, hist 의 마지막 항목과 같음)
    t: jax.Array
    delay: jax.Array
    arm_from: jax.Array
    arm_to: jax.Array
    arm_t0: jax.Array
    arm_T: jax.Array
    ep_ret: jax.Array
    hand_f: jax.Array        # 손에 거는 힘 (왼손, 오른손) × xyz [N] (세계 기준)
    f_until: jax.Array       # 지금 힘(또는 쉬는 구간)이 끝나는 스텝
    key: jax.Array


class RandParams(NamedTuple):
    """환경마다 한 번 뽑아 두는 모델 변형 (난이도에 따라 켜고 끔)."""
    dmass: jax.Array       # 몸통 질량 변화 [kg]
    dcom: jax.Array        # 몸통 무게중심 이동 (3) [m]
    friction: jax.Array    # 접촉 마찰 계수
    kp_scale: jax.Array    # 다리 PD 게인 배율
    kd_scale: jax.Array


class Tron1EnvGpu:
    def __init__(self, xml, foot="sf", iterations=16, ls_iterations=16, cone=None, ep_s=None, randomize=True, push=True,
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
        self.rs, self.ang_sigma = reward_scales(foot)      # train_config.py (CPU 학습과 같은 값)
        self.randomize, self.push = randomize, push
        m, self.model_notes = build_mjx_model(xml, iterations=iterations, ls_iterations=ls_iterations, cone=cone)
        self.m = m
        self.impl = impl
        self.mx = mjx.put_model(m, impl=impl)
        self.dt = m.opt.timestep
        self.nsub = int(round(POLICY_DT / self.dt))
        self.ep_len = int((episode_s(foot) if ep_s is None else ep_s) / POLICY_DT)   # 기본: SF 60 s / PF 20 s (train_config)
        self.reach_p = reach_p(foot)
        J = sp.joints
        self.qadr = jnp.array([m.jnt_qposadr[m.joint(j).id] for j in J])
        self.vadr = jnp.array([m.jnt_dofadr[m.joint(j).id] for j in J])
        self.leg_act = jnp.array([m.actuator(j + "_ctrl").id for j in J])
        arm_act = [m.actuator(f"{s}_joint{i}_ctrl").id for s in ("left", "right") for i in range(1, 8)]
        self.arm_act = jnp.array(arm_act)
        self.foot_bid = jnp.array([m.body(n).id for n in FEET_BODIES[foot]])
        self.hand_bid = jnp.array([m.body(f"openarm_{s}_hand").id for s in ("left", "right")])
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
        from tron1_openarm_sim.arm_poses import mirror
        self.reach_r = jnp.array(REACH_R, jnp.float32)
        self.reach_l = jnp.array([mirror(q) for q in REACH_R], jnp.float32)
        self.rest_r = jnp.array([ARM_POSES[n][1] for n in ("home", "carry")], jnp.float32)
        self.rest_l = jnp.array([ARM_POSES[n][0] for n in ("home", "carry")], jnp.float32)
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
        if self.reach_p > 0:
            # 한 팔은 집기 자세(+작은 잡음), 다른 팔은 home/carry — 실제 집기처럼
            k2 = jax.random.split(k[6], 4)
            ir = jax.random.randint(k2[0], (), 0, self.reach_r.shape[0])
            io = jax.random.randint(k2[1], (), 0, 2)
            right = jax.random.uniform(k2[2]) < 0.5
            nz = 0.05 * jax.random.normal(k2[3], (7,))
            reach = jnp.where(right, jnp.concatenate([self.rest_l[io], self.reach_r[ir] + nz]),
                              jnp.concatenate([self.reach_l[ir] + nz, self.rest_r[io]]))
            q = jnp.where(jax.random.uniform(k[7]) < self.reach_p, reach, q)
        return jnp.clip(q, self.arm_lo, self.arm_hi)

    def _sample_cmd(self, key):
        k = jax.random.split(key, 20)
        r = CMD_RANGES
        c = jnp.array([jax.random.uniform(k[0], minval=r["vx"][0], maxval=r["vx"][1]),
                       jax.random.uniform(k[1], minval=r["vy"][0], maxval=r["vy"][1]) * (jax.random.uniform(k[2]) < VY_P),
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
            if CMD_MIX in ("v7", "v9", "v10"):
                gen = jnp.array([jax.random.uniform(k[0], minval=r["vx"][0], maxval=r["vx"][1]),
                                 jax.random.uniform(k[1], minval=r["vy"][0], maxval=r["vy"][1]) * (jax.random.uniform(k[2]) < VY_P),
                                 jax.random.uniform(k[3], minval=r["wz"][0], maxval=r["wz"][1])])
                vy_max = r["vy"][1]
                side = jnp.array([0.0, jax.random.uniform(k[14], minval=0.1, maxval=vy_max)
                                  * jnp.where(jax.random.uniform(k[15]) < 0.5, -1.0, 1.0), 0.0])
                diag = jnp.array([jax.random.uniform(k[16], minval=r["vx"][0], maxval=r["vx"][1]),
                                  jax.random.uniform(k[17], minval=r["vy"][0], maxval=r["vy"][1]), 0.0])
                c = jnp.select([z < 0.20, z < 0.35, z < 0.50, z < 0.60, z < 0.70, z < 0.85],
                               [jnp.zeros(3), gen * jnp.array([1.0, 0.0, 0.0]), side, diag, slow, turn], gen)
            if CMD_MIX == "v10":  # 느린 미세 이동 15 % 로 되돌림 (v9 8 %), 서기 25 %
                c = jnp.select([z < 0.25, z < 0.35, z < 0.48, z < 0.57, z < 0.72, z < 0.85],
                               [jnp.zeros(3), gen * jnp.array([1.0, 0.0, 0.0]), side, diag, slow, turn], gen)
            if CMD_MIX == "v9":   # 서기 비율을 다시 30 % 로 (서서 작업하는 안정성), 옆걸음·대각선은 v7 그대로
                c = jnp.select([z < 0.30, z < 0.42, z < 0.57, z < 0.67, z < 0.75, z < 0.87],
                               [jnp.zeros(3), gen * jnp.array([1.0, 0.0, 0.0]), side, diag, slow, turn], gen)
            c = jnp.where(jnp.abs(c) < 0.015, 0.0, c)
            want_still = jnp.all(c == 0) & (jax.random.uniform(k[12]) < 0.8)
            k13a, k13b = jax.random.split(k[13])
            long = want_still & (jax.random.uniform(k13a) < STAND_LONG_P)
            dur = jnp.where(long, jax.random.randint(k13b, (), STAND_LONG_RANGE[0], STAND_LONG_RANGE[1] + 1), 250)
        else:
            c = jnp.where(jnp.abs(c) < 0.05, 0.0, c)
            want_still = jnp.array(False)
            dur = jnp.array(250)
        return c.astype(jnp.float32), want_still, dur.astype(jnp.int32)

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
        cmd, still, dur = self._sample_cmd(k[3])
        delay = jax.random.randint(k[4], (), 0, 1 + jnp.round(3 * difficulty).astype(jnp.int32)) * int(self.randomize)
        z = jnp.zeros(self.nj, jnp.float32)
        s = EnvState(d=d, last_action=z, last_action2=z, last_target=self.q_def, gait_index=jnp.array(0.0),
                     cmd=cmd, want_still=still, standing=jnp.array(False), stand_t=jnp.array(0.0), yaw0=jnp.array(0.0),
                     walk_yaw=_yaw(q[3:7]), stand_xy=q[0:2], v_avg=jnp.zeros(2, jnp.float32),
                     cmd_until=dur, hist=jnp.zeros(self.HIST * self.obs_dim, jnp.float32),
                     obs=jnp.zeros(self.obs_dim, jnp.float32),
                     t=jnp.array(0, jnp.int32), delay=delay, arm_from=arm, arm_to=arm, arm_t0=jnp.array(0.0),
                     arm_T=jnp.array(1.0), ep_ret=jnp.array(0.0), hand_f=jnp.zeros((2, 3), jnp.float32),
                     f_until=jnp.array(0, jnp.int32), key=k[5])
        obs = self._obs_raw(s)
        return s._replace(obs=obs, hist=jnp.tile(obs, self.HIST))

    def observe(self, s):
        """(obs, 관측 기록, 명령, 특권정보=실제 선속도×2). CPU 판 observe() 와 같은 의미.
        관측(잡음 포함)과 기록은 step() 이 갱신해 둔 것을 읽는다."""
        lin = jnp.nan_to_num(self._sens(s.d, "base_linvel") * 2.0)
        return s.obs, s.hist, self.command_vector(s), lin.astype(jnp.float32)

    # ------------------------------------------------------------------ step
    def step(self, mx, s: EnvState, action, difficulty):
        key, k_arm, k_new, k_push, k_pv, k_cmd, k_reset, k_noise, k_f = jax.random.split(s.key, 9)
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

        xfrc = d.xfrc_applied
        if HAND_F_MAX > 0 and self.foot == "sf":
            s = self._hand_force(s, k_f, difficulty)
            on = (s.standing & (s.stand_t > 0.5)).astype(jnp.float32)
            xfrc = jnp.zeros_like(d.xfrc_applied).at[self.hand_bid, :3].set(s.hand_f * on)

        def sub(d, i):      # 행동 지연: 처음 delay 스텝은 이전 목표
            leg = jnp.where(i < s.delay, s.last_target, target)
            d = d.replace(ctrl=ctrl.at[self.leg_act].set(leg), xfrc_applied=xfrc)
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
        rs = (s.t >= s.cmd_until) & ~done
        c, still, dur = self._sample_cmd(k_cmd)
        s = s._replace(cmd=jnp.where(rs, c, s.cmd), want_still=jnp.where(rs, still, s.want_still),
                       standing=jnp.where(rs, False, s.standing), stand_t=jnp.where(rs, 0.0, s.stand_t),
                       cmd_until=jnp.where(rs, s.t + dur, s.cmd_until), ep_ret=ep_ret, key=key)
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

    def _hand_force(self, s, key, difficulty):
        """손 외력: 힘을 거는 구간(0.3~3 s)과 쉬는 구간(0.5~3 s)을 번갈아. 크기 0~HAND_F_MAX × 난이도(0.3~1),
        방향은 몸통 기준 앞쪽·아래쪽이 더 많게 (상자를 밀고, 작업대를 누르는 경우)."""
        k = jax.random.split(key, 6)
        due = s.t >= s.f_until
        was_on = jnp.any(s.hand_f != 0)
        level = jnp.clip(2.0 * difficulty, 0.3, 1.0)
        mag = jax.random.uniform(k[0], minval=0.0, maxval=HAND_F_MAX) * level
        v = jnp.array([jax.random.uniform(k[1], minval=-0.5, maxval=1.0), jax.random.uniform(k[2], minval=-1.0, maxval=1.0),
                       jax.random.uniform(k[3], minval=-1.0, maxval=0.3)])
        v = v / jnp.maximum(jnp.linalg.norm(v), 1e-3)
        yaw = _yaw(s.d.qpos[3:7])
        c, sn = jnp.cos(yaw), jnp.sin(yaw)
        f = jnp.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1], v[2]]) * mag
        which = jax.random.randint(k[4], (), 0, 5)                   # 0,1: 왼손 / 2,3: 오른손 / 4: 두 손 (각 절반)
        fl = jnp.where(which <= 1, f, jnp.where(which == 4, 0.5 * f, 0.0))
        fr = jnp.where((which == 2) | (which == 3), f, jnp.where(which == 4, 0.5 * f, 0.0))
        new_f = jnp.stack([fl, fr])
        dur = jnp.where(was_on, jax.random.uniform(k[5], minval=0.5, maxval=3.0),
                        jax.random.uniform(k[5], minval=0.3, maxval=3.0))
        hand_f = jnp.where(due, jnp.where(was_on, jnp.zeros_like(new_f), new_f), s.hand_f)
        f_until = jnp.where(due, s.t + (dur / POLICY_DT).astype(jnp.int32), s.f_until)
        return s._replace(hand_f=hand_f.astype(jnp.float32), f_until=f_until)

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
            stand_t = jnp.where(standing, s.stand_t + POLICY_DT, 0.0)
            yaw = _yaw(s.d.qpos[3:7])
            follow = POLICY_DT / STAND_YAW_TAU if STAND_YAW_TAU > 0 else 0.0
            yaw0 = jnp.where(stand_t < STAND_YAW_SETTLE, yaw, s.yaw0 + follow * _wrap(yaw - s.yaw0))
            straight = ~standing & (jnp.abs(s.cmd[2]) < 0.05)
            walk_yaw = jnp.where(straight, s.walk_yaw + follow * _wrap(yaw - s.walk_yaw), yaw)
            xy = s.d.qpos[0:2]
            stand_xy = jnp.where(stand_t < STAND_YAW_SETTLE, xy, s.stand_xy + follow * (xy - s.stand_xy))
            v_avg = s.v_avg + (POLICY_DT / SLOW_TAU) * (self._sens(s.d, "base_linvel")[:2] - s.v_avg)
            return s._replace(gait_index=g, standing=standing, stand_t=stand_t, yaw0=yaw0, walk_yaw=walk_yaw, stand_xy=stand_xy,
                              v_avg=v_avg)
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
        # 발 높이: 실제 로봇에는 발 위치 센서가 없다 → 관절 각도의 정기구학(몸통 기준 발 위치, 몸통의 실제 위치·자세와 무관)을
        # IMU 자세로 돌려 '세계 위쪽' 높이로. 두 발의 차이만 쓰므로 발 원점이 발바닥보다 조금 위인 것은 상쇄된다
        Rb = d.xmat[self.base_bid].reshape(3, 3)
        p_b = (d.xpos[self.foot_bid] - d.xpos[self.base_bid]) @ Rb          # (2, 3) 몸통 기준 = 정기구학
        fz = (p_b @ quat_to_mat(self._sens(d, "imu_quat")).T)[:, 2]
        clear_h = (jnp.clip(0.06 + 0.07 * jnp.linalg.norm(s.cmd[:2]), 0.06, self.GAIT[3]) if CLEAR_ADAPT else self.GAIT[3])
        fvxy = jnp.array([jnp.linalg.norm(self._sens(d, "foot_L_vel")[:2]), jnp.linalg.norm(self._sens(d, "foot_R_vel")[:2])])
        settled = (s.standing & (s.stand_t >= STAND_YAW_SETTLE)).astype(jnp.float32)
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
            feet_distance=jnp.clip(0.115 - jnp.linalg.norm(fp), 0, 1) * jnp.where(jnp.abs(s.cmd[1]) > 0.05, FD_SIDE_SCALE, 1.0),
            contact_force=jnp.mean((1 - des) * (1 - jnp.exp(-F ** 2 / 25.0))),
            contact_vel=jnp.mean(des * (1 - jnp.exp(-fv ** 2 / 0.25))),
            stand_drift=s.standing * (1 - jnp.exp(-(gyro[2] / STAND_DRIFT_SIGMA[0]) ** 2
                                                  - jnp.sum(lin[:2] ** 2) / STAND_DRIFT_SIGMA[1] ** 2)),
            stand_yaw=(s.standing & (s.stand_t >= STAND_YAW_SETTLE))
            * jnp.minimum(jnp.abs(_wrap(_yaw(quat) - s.yaw0)) / STAND_YAW_REF, 2.0),
            stand_feet=settled * jnp.mean(1 - jnp.exp(-(fvxy / STAND_FEET_SIGMA) ** 2)),
            stand_twist=settled * jnp.mean(1 - jnp.exp(-(d.cvel[self.foot_bid, 2] / STAND_TWIST_SIGMA) ** 2)),
            stand_load=settled * jnp.clip((STAND_LOAD_MIN - jnp.min(F) / jnp.maximum(jnp.sum(F), 1.0)) / STAND_LOAD_MIN, 0.0, 1.0),
            stand_tilt=settled * jnp.clip((jnp.linalg.norm(grav[:2]) - STAND_TILT_FREE) / STAND_TILT_REF, 0.0, 2.0),
            feet_clearance=jnp.sum((1 - des) * jnp.clip(clear_h - (fz - jnp.min(fz)), 0.0, None) ** 2),
            tracking_vy=jnp.exp(-(s.cmd[1] - lin[1]) ** 2 / TRACK_VY_SIGMA),
            walk_heading=(~s.standing & (jnp.abs(s.cmd[2]) < 0.05)).astype(jnp.float32)
            * jnp.minimum(jnp.abs(_wrap(_yaw(quat) - s.walk_yaw)) / WALK_HEADING_REF, WALK_HEADING_CAP),
            walk_yawrate=(~s.standing & (jnp.abs(s.cmd[2]) < 0.05)).astype(jnp.float32)
            * jnp.minimum((gyro[2] / 0.05) ** 2, 4.0),
            stand_liftoff=settled * (jnp.min(F) < STAND_LIFT_F).astype(jnp.float32),
            stand_sway=settled * jnp.minimum(jnp.linalg.norm(d.qpos[0:2] - s.stand_xy) / STAND_SWAY_REF, 2.0),
            tracking_slow=((~s.standing) & (jnp.linalg.norm(s.cmd[:2]) > 0.005) & (jnp.linalg.norm(s.cmd[:2]) < 0.15)).astype(jnp.float32)
            * jnp.exp(-(jnp.linalg.norm(s.cmd[:2] - s.v_avg) / jnp.maximum(jnp.linalg.norm(s.cmd[:2]), 0.03)) ** 2 / 0.25),
            stand_planted=settled * (jnp.min(F) >= STAND_LIFT_F).astype(jnp.float32)
            * jnp.mean(jnp.exp(-(fvxy / STAND_FEET_SIGMA) ** 2)),
        )
        r = sum(self.rs[k] * v for k, v in t.items()) * POLICY_DT
        return r, t


def _yaw(q):
    return jnp.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))


def _wrap(a):
    return (a + jnp.pi) % (2 * jnp.pi) - jnp.pi


def quat_to_mat(q):
    """q (wxyz) → 3×3 회전 행렬."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    return jnp.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


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
