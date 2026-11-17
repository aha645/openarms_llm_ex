#!/usr/bin/env python3
"""CPU 학습 환경(tron1_env, 메시 발·기본 솔버) ↔ MJX 학습 환경(tron1_mjx_env) 물리 비교.

같은 정책(평균 행동, 탐색 잡음 없음)을 같은 명령으로 두 환경에서 돌려 결과가 비슷한지 본다.
랜덤화·밀기는 끄고, 환경 N 개 평균. 팔 자세는 두 환경 모두 같은 규칙으로 무작위.
MJX 에서 학습한 정책이 CPU(배포 쪽 물리)에서도 같게 움직이려면 이 표가 비슷해야 한다.

    python3 mjx_parity.py --foot sf --policy ../src/tron1_openarm_sim/policies/tron1sf_openarm_policy.npz
"""
import argparse
import json
import os
import sys
import time

import jax
import jax.numpy as jnp
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import policy_jax as P  # noqa: E402
from tron1_env import Tron1OpenArmEnv  # noqa: E402
from tron1_mjx_env import Tron1MjxEnv  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCEN = {   # 이름: (vx, vy, wz, 제자리 서기)
    "stand": (0.0, 0.0, 0.0, True),
    "walk_0.3": (0.3, 0.0, 0.0, False),
    "turn_0.6": (0.0, 0.0, 0.6, False),
    "strafe_0.15": (0.0, 0.15, 0.0, False),
    "slow_0.08": (0.08, 0.0, 0.0, False),
}
STEPS = 240            # 4.8 s (5 s 마다 명령이 바뀌기 전)
SKIP = 50              # 처음 1 s 는 가속 구간이라 평균에서 뺌


def summarize(vx, vy, wz, h, tilt, fell):
    ok = ~fell.any(0)
    sl = slice(SKIP, None)
    return dict(fall=f"{int((~ok).sum())}/{len(ok)}",
                vx=round(float(vx[sl][:, ok].mean()), 3), vy=round(float(vy[sl][:, ok].mean()), 3),
                wz=round(float(wz[sl][:, ok].mean()), 3), h=round(float(h[sl][:, ok].mean()), 3),
                tilt_deg=round(float(np.degrees(tilt[sl][:, ok].mean())), 2))


def run_cpu(a, params, c):
    load = None
    if a.cpu_sole == "box":       # MJX 와 같은 발바닥 상자 + CPU 기본 솔버 → MJX 수치 차이만 따로 본다
        from mjx_model import build_mjx_model
        load = lambda x: build_mjx_model(x)[0]  # noqa: E731
    env = Tron1OpenArmEnv(a.xml, num_envs=a.n, seed=0, nthread=os.cpu_count(), randomize=False, push=False, foot=a.foot,
                          load_model=load)
    env.cmd[:] = c[:3]
    env.want_still[:] = c[3]
    env.standing[:] = False
    pol = jax.jit(P.actor_mean)
    rec = {k: [] for k in ("vx", "vy", "wz", "h", "tilt", "fell")}
    for _ in range(STEPS):
        obs, hist, cmd, lin = env.observe()
        act = np.asarray(pol(params, obs, hist, cmd))
        _, done, info = env.step(act)
        rec["fell"].append(info["fell"])
        S = env.sens
        v = S[:, env.s["base_linvel"]]
        rec["vx"].append(v[:, 0]); rec["vy"].append(v[:, 1]); rec["wz"].append(S[:, env.s["imu_gyro"]][:, 2])
        rec["h"].append(env.state[:, env.q0 + 2])
        q = env.state[:, env.q0 + 3:env.q0 + 7]
        rec["tilt"].append(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
        env.cmd[:] = c[:3]                      # 넘어져 다시 시작한 환경도 같은 명령
        env.want_still[:] = c[3]
    return summarize(*(np.array(rec[k]) for k in ("vx", "vy", "wz", "h", "tilt", "fell")))


def run_mjx(a, params, c, cone):
    env = Tron1MjxEnv(a.xml, foot=a.foot, iterations=a.iterations, ls_iterations=a.ls_iterations, cone=cone,
                      randomize=False, push=False, impl=a.impl, num_envs=a.n)
    rp = env.sample_rand(jax.random.PRNGKey(0), a.n)
    mb, axes = env.batched_model(rp, 0.0, 1.0)
    st = jax.vmap(env.reset, in_axes=(0, None))(jax.random.split(jax.random.PRNGKey(1), a.n), 0.0)
    st = st._replace(cmd=jnp.tile(jnp.asarray(c[:3], jnp.float32), (a.n, 1)), want_still=jnp.full(a.n, bool(c[3])))
    vstep = jax.vmap(env.step, in_axes=(axes, 0, 0, None))

    @jax.jit
    def roll(st):
        def body(st, _):
            obs, hist, cmd, lin = jax.vmap(env.observe)(st)
            act = P.actor_mean(params, obs, hist, cmd)
            st, _, done, info = vstep(mb, st, act, 0.0)
            st = st._replace(cmd=jnp.tile(jnp.asarray(c[:3], jnp.float32), (a.n, 1)), want_still=jnp.full(a.n, bool(c[3])))
            d = st.d
            S = d.sensordata
            lv = S[:, env.S["base_linvel"][0]:env.S["base_linvel"][0] + 3]
            gz = S[:, env.S["imu_gyro"][0] + 2]
            q = d.qpos[:, 3:7]
            tilt = jnp.arccos(jnp.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1))
            return st, (lv[:, 0], lv[:, 1], gz, d.qpos[:, 2], tilt, info["fell"])
        return jax.lax.scan(body, st, None, length=STEPS)[1]
    out = [np.asarray(x) for x in roll(st)]
    return summarize(*out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--foot", default="sf")
    ap.add_argument("--xml", default="")
    ap.add_argument("--policy", default=os.path.join(ROOT, "src/tron1_openarm_sim/policies/tron1sf_openarm_policy.npz"))
    ap.add_argument("--n", type=int, default=256)
    ap.add_argument("--impl", choices=["jax", "warp"], default="jax", help="MJX 백엔드")
    ap.add_argument("--iterations", type=int, default=16)
    ap.add_argument("--ls-iterations", type=int, default=16)
    ap.add_argument("--only", default="", help="cpu,mjx_elliptic,mjx_pyramidal 중 쉼표로")
    ap.add_argument("--cpu-sole", choices=["mesh", "box"], default="box",
                    help="CPU 쪽 발 충돌: box = MJX 와 같은 상자 (기본, MJX 수치 비교), mesh = 원래 학습·배포 모델")
    a = ap.parse_args()
    if not a.xml:
        a.xml = os.path.join(ROOT, "src/tron1_openarm_sim/mjcf", "sf" if a.foot == "sf" else "", "tron1_openarm_train.xml")
    npz = np.load(a.policy)
    params = {"encoder": P.from_npz(npz, "encoder"), "policy": P.from_npz(npz, "policy")}
    which = a.only.split(",") if a.only else ["cpu", "mjx_elliptic", "mjx_pyramidal"]
    res = {}
    for name, c in SCEN.items():
        for w in which:
            t = time.time()
            r = run_cpu(a, params, c) if w == "cpu" else run_mjx(a, params, c, 1 if w == "mjx_elliptic" else 0)
            res.setdefault(name, {})[w] = r
            print(f"{name:12s} {w:14s} {json.dumps(r)}  ({time.time() - t:.0f} s)", flush=True)
    print(json.dumps(res, ensure_ascii=False))


if __name__ == "__main__":
    main()
