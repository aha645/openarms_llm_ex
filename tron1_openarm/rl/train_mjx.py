#!/usr/bin/env python3
"""TRON1 + OpenArm 보행 정책 PPO — MJX(GPU 물리) 판.

train_ppo.py(CPU 물리)와 알고리즘·신경망·가중치 형식(npz)·커리큘럼이 같다. 다른 점은
  * 환경: tron1_mjx_env.Tron1MjxEnv (발바닥 상자, 솔버 반복 16) — 수천 개를 GPU 에서 한꺼번에
  * 수집(rollout)·GAE·PPO 갱신이 모두 jax.jit 안에서 돈다 (반복당 CPU↔GPU 복사는 기록용 숫자뿐)

    python3 train_mjx.py --foot sf --init ../src/tron1_openarm_sim/policies/tron1sf_openarm_policy.npz --minutes 30
    python3 train_mjx.py --foot sf --init scratch --envs 4096 --minutes 240     # 처음부터 (관측 규약을 바꿀 때)

결과: <out>/best.npz · final.npz (locomotion_policy.py 가 그대로 읽음), last.pkl (이어 학습), log.json
"""
import argparse
import json
import os
import pickle
import sys
import time
from functools import partial

import jax
import jax.numpy as jnp
import numpy as np
import optax

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import policy_jax as P  # noqa: E402
from tron1_mjx_env import Tron1MjxEnv  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def parse():
    ap = argparse.ArgumentParser()
    ap.add_argument("--foot", choices=["pf", "sf"], default="sf")
    ap.add_argument("--xml", default="", help="기본: 발 형태별 *_train.xml")
    ap.add_argument("--init", default="", help="npz 경로 또는 scratch. 기본: 발 형태별 배포 정책")
    ap.add_argument("--out", default=os.path.join(ROOT, "runs", "mjx_" + time.strftime("%Y%m%d_%H%M%S")))
    ap.add_argument("--envs", type=int, default=4096)
    ap.add_argument("--steps", type=int, default=24, help="반복당 환경별 스텝 수")
    ap.add_argument("--minutes", type=float, default=30)
    ap.add_argument("--max-iters", type=int, default=100000)
    ap.add_argument("--lr", type=float, default=2e-5)
    ap.add_argument("--init-std", type=float, default=0.1)
    ap.add_argument("--critic-warmup", type=int, default=20)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--max-difficulty", type=float, default=0.5)
    ap.add_argument("--payload-start", type=float, default=1.0)
    ap.add_argument("--train-encoder", type=int, default=1)
    ap.add_argument("--resume", default="")
    ap.add_argument("--iterations", type=int, default=16, help="접촉 솔버 반복 상한 (mjx_parity.py 로 확인한 값)")
    ap.add_argument("--ls-iterations", type=int, default=16)
    ap.add_argument("--impl", choices=["warp", "jax"], default="warp",
                    help="MJX 백엔드. warp = MuJoCo Warp 커널 (RTX 3090 에서 jax 대비 수 배), jax = 순수 JAX")
    a = ap.parse_args()
    sub = "sf" if a.foot == "sf" else ""
    if not a.xml:
        a.xml = os.path.join(ROOT, "src/tron1_openarm_sim/mjcf", sub, "tron1_openarm_train.xml")
    if not a.init:
        a.init = os.path.join(ROOT, "src/tron1_openarm_sim/policies",
                              "tron1sf_openarm_policy.npz" if a.foot == "sf" else "tron1_openarm_policy.npz")
    return a


def log_prob(mean, log_std, a):
    return jnp.sum(-0.5 * ((a - mean) / jnp.exp(log_std)) ** 2 - log_std - 0.5 * jnp.log(2 * jnp.pi), -1)


def critic_input(obs, lin, cmd, est):
    return jnp.concatenate([obs, lin, cmd, est], -1)


def main():
    a = parse()
    os.makedirs(a.out, exist_ok=True)
    print("JAX:", jax.default_backend(), jax.devices(), flush=True)
    env = Tron1MjxEnv(a.xml, foot=a.foot, iterations=a.iterations, ls_iterations=a.ls_iterations, impl=a.impl, num_envs=a.envs)
    print("MJX 모델:", "; ".join(env.model_notes), flush=True)
    N, T = a.envs, a.steps
    nj, enc_in, pol_in = env.nj, env.HIST * env.obs_dim, 3 + env.obs_dim + env.n_cmd
    key = jax.random.PRNGKey(a.seed)

    if a.resume:
        with open(a.resume, "rb") as fp:
            ck = pickle.load(fp)
        params, critic, log_std = ck["params"], ck["critic"], jnp.asarray(ck["log_std"])
        difficulty, payload = ck.get("difficulty", 0.0), ck.get("payload", 1.0)
    else:
        if a.init == "scratch":
            key, k1, k2 = jax.random.split(key, 3)
            params = {"encoder": P.mlp_init(k1, [enc_in, 256, 128, 3]), "policy": P.mlp_init(k2, [pol_in, 512, 256, 128, nj], 0.01)}
            log_std = jnp.full(nj, np.log(1.0), jnp.float32)
        else:
            npz = np.load(a.init)
            params = {"encoder": P.from_npz(npz, "encoder"), "policy": P.from_npz(npz, "policy")}
            log_std = jnp.full(nj, np.log(a.init_std), jnp.float32)
        key, k3 = jax.random.split(key)
        critic = P.mlp_init(k3, [env.obs_dim + 3 + env.n_cmd + 3, 512, 256, 128, 1], 1.0)
        difficulty, payload = 0.0, a.payload_start

    actor_tx = optax.chain(optax.clip_by_global_norm(1.0), optax.inject_hyperparams(optax.adam)(learning_rate=a.lr))
    critic_tx = optax.chain(optax.clip_by_global_norm(1.0), optax.adam(1e-3))
    enc_tx = optax.adam(1e-4)
    actor_state = actor_tx.init({"policy": params["policy"], "log_std": log_std})
    critic_state = critic_tx.init(critic)
    enc_state = enc_tx.init(params["encoder"])

    key, kr, ks = jax.random.split(key, 3)
    rp = env.sample_rand(kr, N)
    _, axes = env.batched_model(rp, 0.0, 1.0)            # vmap 축 구조 (값과 무관)
    vstep = jax.vmap(env.step, in_axes=(axes, 0, 0, None))
    vobs = jax.vmap(env.observe)
    states = jax.jit(jax.vmap(env.reset, in_axes=(0, None)))(jax.random.split(ks, N), difficulty)
    gamma, lam, clip = 0.99, 0.95, 0.2

    def value(critic, params, obs, hist, cmd, lin):
        est = P.mlp_apply(params["encoder"], hist)
        return P.mlp_apply(critic, critic_input(obs, lin, cmd, est))[..., 0]

    @jax.jit
    def rollout(params, log_std, critic, states, key, difficulty, payload):
        mb, _ = env.batched_model(rp, difficulty, payload)

        def body(carry, _):
            st, key = carry
            key, k = jax.random.split(key)
            obs, hist, cmd, lin = vobs(st)
            mean = P.actor_mean(params, obs, hist, cmd)
            act = mean + jnp.exp(log_std) * jax.random.normal(k, mean.shape)
            lp = log_prob(mean, log_std, act)
            v = value(critic, params, obs, hist, cmd, lin)
            st, rew, done, info = vstep(mb, st, act, difficulty)
            out = dict(bad=info["bad"], obs=obs, hist=hist, cmd=cmd, lin=lin, act=act, lp=lp, rew=rew, done=done.astype(jnp.float32),
                       val=v, tout=info["timeout"].astype(jnp.float32), fell=info["fell"],
                       ep_ret=jnp.where(done, info["ep_ret"], 0.0), ep_t=jnp.where(done, info["ep_t"], 0),
                       lin_track=info["terms"]["tracking_lin_vel"], ang_track=info["terms"]["tracking_ang_vel"],
                       stand_drift=info["terms"]["stand_drift"], standing=st.standing.astype(jnp.float32))
            return (st, key), out
        (states, key), tr = jax.lax.scan(body, (states, key), None, length=T)
        obs, hist, cmd, lin = vobs(states)
        last_v = value(critic, params, obs, hist, cmd, lin)
        # GAE (시간 초과는 실패가 아니므로 가치로 부트스트랩)
        rew = tr["rew"] + gamma * tr["val"] * tr["tout"]

        def gae(g, x):
            r, v, nv, dn = x
            nonterm = 1.0 - dn
            g = r + gamma * nv * nonterm - v + gamma * lam * nonterm * g
            return g, g
        next_v = jnp.concatenate([tr["val"][1:], last_v[None]], 0)
        _, adv = jax.lax.scan(gae, jnp.zeros(N), (rew, tr["val"], next_v, tr["done"]), reverse=True)
        tr["adv"], tr["ret"] = adv, adv + tr["val"]
        stats = dict(bad=tr["bad"].sum(), falls=tr["fell"].sum(), n_done=(tr["done"] > 0).sum(), ret_sum=tr["ep_ret"].sum(),
                     len_sum=tr["ep_t"].sum(), lin_track=tr["lin_track"].mean(), ang_track=tr["ang_track"].mean(),
                     stand_drift=tr["stand_drift"].sum() / jnp.maximum(tr["standing"].sum(), 1.0), standing=tr["standing"].mean())
        return states, tr, stats, key

    @partial(jax.jit, static_argnums=(7,))
    def update(params, log_std, critic, actor_state, critic_state, enc_state, tr, train_enc, lr, actor_on, key):
        M = N * T
        flat = {k: tr[k].reshape(M, *tr[k].shape[2:]) for k in ("obs", "hist", "cmd", "lin", "act", "lp", "adv", "ret", "val")}
        flat["adv"] = (flat["adv"] - flat["adv"].mean()) / (flat["adv"].std() + 1e-8)
        nb = 4
        actor_state[1].hyperparams["learning_rate"] = lr

        def mb_step(carry, idx):
            ap, critic, actor_state, critic_state, stop = carry
            b = {k: v[idx] for k, v in flat.items()}

            def actor_loss(ap):
                mean = P.actor_mean({"encoder": params["encoder"], "policy": ap["policy"]}, b["obs"], b["hist"], b["cmd"])
                lp = log_prob(mean, ap["log_std"], b["act"])
                ratio = jnp.exp(lp - b["lp"])
                s1, s2 = ratio * b["adv"], jnp.clip(ratio, 1 - clip, 1 + clip) * b["adv"]
                ent = jnp.sum(ap["log_std"] + 0.5 * jnp.log(2 * jnp.pi * jnp.e))
                return -jnp.mean(jnp.minimum(s1, s2)) - 0.0005 * ent, jnp.mean(b["lp"] - lp)

            def critic_loss(cp):
                est = jax.lax.stop_gradient(P.mlp_apply(params["encoder"], b["hist"]))
                v = P.mlp_apply(cp, critic_input(b["obs"], b["lin"], b["cmd"], est))[..., 0]
                vc = b["val"] + jnp.clip(v - b["val"], -clip, clip)
                return jnp.mean(jnp.maximum((v - b["ret"]) ** 2, (vc - b["ret"]) ** 2))
            (al, kl), ag = jax.value_and_grad(actor_loss, has_aux=True)(ap)
            on = actor_on * (1.0 - stop)
            ag = jax.tree_util.tree_map(lambda g: g * on, ag)
            upd, actor_state = actor_tx.update(ag, actor_state, ap)
            ap = optax.apply_updates(ap, upd)
            cl, cg = jax.value_and_grad(critic_loss)(critic)
            cupd, critic_state = critic_tx.update(cg, critic_state, critic)
            critic = optax.apply_updates(critic, cupd)
            # KL 조기 종료: 사전학습 정책을 한 번에 크게 바꾸지 않도록 (이후 미니배치는 크리틱만)
            stop = jnp.maximum(stop, actor_on * (kl > 0.02))
            return (ap, critic, actor_state, critic_state, stop), (kl, al, cl)

        def epoch(carry, k):
            idx = jax.random.permutation(k, M).reshape(nb, M // nb)
            return jax.lax.scan(mb_step, carry, idx)
        key, *ks = jax.random.split(key, 6)
        ap = {"policy": params["policy"], "log_std": log_std}
        (ap, critic, actor_state, critic_state, _), (kls, als, cls) = jax.lax.scan(
            epoch, (ap, critic, actor_state, critic_state, 0.0), jnp.stack(ks))
        params = {"encoder": params["encoder"], "policy": ap["policy"]}
        el = jnp.array(0.0)
        if train_enc:        # PPO 가 끝난 뒤 따로: 인코더가 실제 선속도(×2)를 맞히도록 (LimX 방식)
            def enc_step(carry, idx):
                enc, st = carry
                loss, g = jax.value_and_grad(lambda e: jnp.mean((P.mlp_apply(e, flat["hist"][idx]) - flat["lin"][idx]) ** 2))(enc)
                u, st = enc_tx.update(g, st, enc)
                return (optax.apply_updates(enc, u), st), loss
            key, k1, k2 = jax.random.split(key, 3)
            idx = jnp.concatenate([jax.random.permutation(k, M).reshape(nb, M // nb) for k in (k1, k2)])
            (enc, enc_state), els = jax.lax.scan(enc_step, (params["encoder"], enc_state), idx)
            params = {"encoder": enc, "policy": params["policy"]}
            el = els.mean()
        return params, ap["log_std"], critic, actor_state, critic_state, enc_state, kls.mean(), cls.mean(), el

    t_start = time.time()
    log, fall_hist, rets, lens = [], [], [], []
    best, last_pl, lr = -1e9, 0, a.lr
    it = 0
    while it < a.max_iters and (time.time() - t_start) < a.minutes * 60:
        it += 1
        t0 = time.time()
        states, tr, st, key = rollout(params, log_std, critic, states, key, difficulty, payload)
        actor_on = 0.0 if (it <= a.critic_warmup and not a.resume) else 1.0
        key, ku = jax.random.split(key)
        (params, log_std, critic, actor_state, critic_state, enc_state, kl, vl, el) = update(
            params, log_std, critic, actor_state, critic_state, enc_state, tr,
            bool(a.train_encoder and actor_on), lr, actor_on, ku)
        st = jax.device_get(st)
        kl = float(kl)
        dt_it = time.time() - t0
        if actor_on:
            lr = lr / 1.5 if kl > 0.03 else (lr * 1.5 if kl < 0.008 else lr)
            lr = float(np.clip(lr, 3e-6, 3e-4))
        fall_hist.append(float(st["falls"]) / (N * T))
        fr = float(np.mean(fall_hist[-10:]))
        if st["n_done"] > 0:
            rets.append(float(st["ret_sum"]) / float(st["n_done"]))
            lens.append(float(st["len_sum"]) / float(st["n_done"]))
        # 커리큘럼 (train_ppo.py 와 같음): 상반신 무게 → 난이도
        if payload < 1.0:
            if actor_on and it > a.critic_warmup + 10 and fr < 0.002 and it - last_pl > 10:
                payload, last_pl = min(1.0, payload + 0.05), it
        elif actor_on and it > a.critic_warmup + 10 and fr < 0.0003 and difficulty < a.max_difficulty:
            difficulty = min(a.max_difficulty, difficulty + 0.02)
        elif fr > 0.0015 and difficulty > 0:
            difficulty = max(0.0, difficulty - 0.02)
        rec = dict(it=it, t=round(time.time() - t_start, 1), steps=it * N * T, sps=round(N * T / dt_it),
                   fall_rate=round(fr, 5), difficulty=round(difficulty, 3), payload=round(payload, 2),
                   ep_ret=round(float(np.mean(rets[-20:])), 3) if rets else 0.0,
                   ep_len=round(float(np.mean(lens[-20:])), 1) if lens else 0.0, kl=round(kl, 5), lr=lr,
                   std=[round(float(x), 3) for x in np.exp(np.asarray(log_std))], value_loss=round(float(vl), 4),
                   enc_loss=round(float(el), 4), lin_track=round(float(st["lin_track"]), 3),
                   ang_track=round(float(st["ang_track"]), 3), diverged=int(st["bad"]),
                   stand_drift=round(float(st["stand_drift"]), 4), standing=round(float(st["standing"]), 3))
        log.append(rec)
        if it % 5 == 0 or it == 1:
            print(json.dumps(rec, ensure_ascii=False), flush=True)
        score = rec["lin_track"] + 0.5 * rec["ang_track"] - 50 * fr + 0.5 * difficulty - 10 * (1 - payload)
        if it % 25 == 0:
            with open(os.path.join(a.out, "last.pkl"), "wb") as fp:
                pickle.dump({"params": jax.device_get(params), "critic": jax.device_get(critic), "log_std": jax.device_get(log_std),
                             "difficulty": difficulty, "payload": payload, "it": it}, fp)
            with open(os.path.join(a.out, "log.json"), "w") as fp:
                json.dump(log, fp)
        if actor_on and it > a.critic_warmup + 20 and score > best:
            best = score
            np.savez(os.path.join(a.out, "best.npz"), **P.to_npz_dict(params["encoder"], "encoder"),
                     **P.to_npz_dict(params["policy"], "policy"))
    np.savez(os.path.join(a.out, "final.npz"), **P.to_npz_dict(params["encoder"], "encoder"),
             **P.to_npz_dict(params["policy"], "policy"))
    with open(os.path.join(a.out, "log.json"), "w") as fp:
        json.dump(log, fp)
    with open(os.path.join(a.out, "last.pkl"), "wb") as fp:
        pickle.dump({"params": jax.device_get(params), "critic": jax.device_get(critic), "log_std": jax.device_get(log_std),
                     "difficulty": difficulty, "payload": payload, "it": it}, fp)
    print("done", it, "iters", a.out)


if __name__ == "__main__":
    main()
