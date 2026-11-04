#!/usr/bin/env python3
"""TRON1 + OpenArm 보행 정책 PPO 미세조정 (JAX, CPU/GPU 모두 동작).

    python3 train_ppo.py --minutes 90                 # 이 저장소 기본값 (2코어 CPU 에서 약 1천만 스텝)
    python3 train_ppo.py --envs 512 --minutes 240      # 코어가 많은 PC
    python3 train_ppo.py --init scratch                # 사전학습 없이 처음부터 (수억 스텝 필요 — GPU 권장)

시작점: LimX 가 Isaac Gym 으로 학습해 공개한 PF_TRON1A 정책(policies/limx_pf_tron1a_pretrained.npz).
그대로는 OpenArm 14.4 kg 를 얹으면 옆으로 미끄러지듯 흐르고(약 0.1 m/s), 잡음·팔 동작이 있으면 넘어진다.
→ 같은 네트워크 구조·관측 규약을 유지한 채 PPO 로 미세조정한다 (전이학습).

알고리즘 (LimX rsl_rl 변형과 같은 구성)
  * 액터  : 인코더(관측 기록 300 → 추정 선속도 3, 출력 detach) + 정책 MLP(36 → 512-256-128 → 6)
  * 크리틱: 새로 만듦 (관측 30 + 실제 선속도 3 + 명령 3 + 추정속도 3 → 512-256-128 → 1)
  * 인코더: 실제 선속도를 맞히도록 별도 지도학습 (MSE)
  * 처음 --critic-warmup 번은 크리틱만 학습 (사전학습 액터를 망가뜨리지 않도록)
  * 난이도 커리큘럼: 낙상률이 낮으면 잡음·지연·밀기·팔 동작·모델 랜덤화를 조금씩 올림
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
from tron1_env import Tron1OpenArmEnv  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def parse():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default=os.path.join(ROOT, "src/tron1_openarm_sim/mjcf/tron1_openarm_train.xml"))
    ap.add_argument("--init", default=os.path.join(ROOT, "policies/limx_pf_tron1a_pretrained.npz"))
    ap.add_argument("--out", default=os.path.join(ROOT, "runs", time.strftime("%Y%m%d_%H%M%S")))
    ap.add_argument("--envs", type=int, default=128)
    ap.add_argument("--steps", type=int, default=24, help="반복당 환경별 스텝 수")
    ap.add_argument("--minutes", type=float, default=90)
    ap.add_argument("--max-iters", type=int, default=100000)
    ap.add_argument("--lr", type=float, default=2e-4)
    ap.add_argument("--init-std", type=float, default=0.15)
    ap.add_argument("--critic-warmup", type=int, default=30)
    ap.add_argument("--threads", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--resume", default="")
    ap.add_argument("--train-encoder", type=int, default=1,
                    help="1: PPO 뒤에 인코더를 실제 선속도로 지도학습 (LimX 방식). 0: 사전학습 인코더 고정")
    return ap.parse_args()


def critic_input(obs, lin, cmd, est):
    return jnp.concatenate([obs, lin, cmd, est], -1)


def log_prob(mean, log_std, a):
    return jnp.sum(-0.5 * ((a - mean) / jnp.exp(log_std)) ** 2 - log_std - 0.5 * jnp.log(2 * jnp.pi), -1)


def main():
    a = parse()
    os.makedirs(a.out, exist_ok=True)
    key = jax.random.PRNGKey(a.seed)
    env = Tron1OpenArmEnv(a.xml, num_envs=a.envs, seed=a.seed, nthread=a.threads)

    if a.resume:
        with open(a.resume, "rb") as fp:
            ck = pickle.load(fp)
        params, critic, log_std, diff0 = ck["params"], ck["critic"], ck["log_std"], ck.get("difficulty", 0.0)
    else:
        if a.init == "scratch":
            key, k1, k2 = jax.random.split(key, 3)
            params = {"encoder": P.mlp_init(k1, [300, 256, 128, 3]), "policy": P.mlp_init(k2, [36, 512, 256, 128, 6], 0.01)}
            log_std = jnp.full(6, np.log(1.0), jnp.float32)
        else:
            npz = np.load(a.init)
            params = {"encoder": P.from_npz(npz, "encoder"), "policy": P.from_npz(npz, "policy")}
            log_std = jnp.full(6, np.log(a.init_std), jnp.float32)
        key, k3 = jax.random.split(key)
        critic = P.mlp_init(k3, [39, 512, 256, 128, 1], 1.0)
        diff0 = 0.0
    env.set_difficulty(diff0)

    actor_tx = optax.chain(optax.clip_by_global_norm(1.0), optax.inject_hyperparams(optax.adam)(learning_rate=a.lr))
    critic_tx = optax.chain(optax.clip_by_global_norm(1.0), optax.adam(1e-3))
    enc_tx = optax.adam(1e-4)
    actor_state = actor_tx.init({"policy": params["policy"], "log_std": log_std})
    critic_state = critic_tx.init(critic)
    enc_state = enc_tx.init(params["encoder"])
    lr = a.lr

    @jax.jit
    def policy_step(params, log_std, obs, hist, cmd, key):
        mean = P.actor_mean(params, obs, hist, cmd)
        act = mean + jnp.exp(log_std) * jax.random.normal(key, mean.shape)
        return act, log_prob(mean, log_std, act), mean

    @jax.jit
    def values(critic, params, obs, hist, cmd, lin):
        est = P.mlp_apply(params["encoder"], hist)
        return P.mlp_apply(critic, critic_input(obs, lin, cmd, est))[..., 0]

    @partial(jax.jit, static_argnums=(10,))
    def update(params, log_std, critic, actor_state, critic_state, enc_state, batch, lr_now, clip, actor_on, _unused):
        obs, hist, cmd, lin, act, old_lp, adv, ret, old_v = batch

        def actor_loss(ap):
            mean = P.actor_mean({"encoder": params["encoder"], "policy": ap["policy"]}, obs, hist, cmd)
            lp = log_prob(mean, ap["log_std"], act)
            ratio = jnp.exp(lp - old_lp)
            s1 = ratio * adv
            s2 = jnp.clip(ratio, 1 - clip, 1 + clip) * adv
            ent = jnp.sum(ap["log_std"] + 0.5 * jnp.log(2 * jnp.pi * jnp.e))
            kl = jnp.mean(old_lp - lp)
            return -jnp.mean(jnp.minimum(s1, s2)) - 0.0005 * ent, kl

        def critic_loss(cp):
            est = jax.lax.stop_gradient(P.mlp_apply(params["encoder"], hist))
            v = P.mlp_apply(cp, critic_input(obs, lin, cmd, est))[..., 0]
            vc = old_v + jnp.clip(v - old_v, -clip, clip)
            return jnp.mean(jnp.maximum((v - ret) ** 2, (vc - ret) ** 2))

        def enc_loss(ep):
            return jnp.mean((P.mlp_apply(ep, hist) - lin) ** 2)

        ap = {"policy": params["policy"], "log_std": log_std}
        (al, kl), ag = jax.value_and_grad(actor_loss, has_aux=True)(ap)
        ag = jax.tree_util.tree_map(lambda g: g * actor_on, ag)
        actor_state[1].hyperparams["learning_rate"] = lr_now
        upd, actor_state = actor_tx.update(ag, actor_state, ap)
        ap = optax.apply_updates(ap, upd)
        cl, cg = jax.value_and_grad(critic_loss)(critic)
        cupd, critic_state = critic_tx.update(cg, critic_state, critic)
        critic = optax.apply_updates(critic, cupd)
        new_params = {"encoder": params["encoder"], "policy": ap["policy"]}
        return new_params, ap["log_std"], critic, actor_state, critic_state, enc_state, al, cl, 0.0, kl

    @jax.jit
    def update_encoder(enc, enc_state, hist, lin):
        """PPO 가 끝난 뒤 따로: 인코더가 실제 선속도(×2)를 맞히도록 (LimX ppo.py 의 extra_loss 와 같음)."""
        def enc_loss(ep):
            return jnp.mean((P.mlp_apply(ep, hist) - lin) ** 2)
        el, eg = jax.value_and_grad(enc_loss)(enc)
        eupd, enc_state = enc_tx.update(eg, enc_state, enc)
        return optax.apply_updates(enc, eupd), enc_state, el

    N, T = a.envs, a.steps
    gamma, lam = 0.99, 0.95
    t_start = time.time()
    log = []
    best = -1e9
    ep_returns, ep_lens, fall_hist = [], [], []
    obs, hist, cmd, lin = env.observe()
    it = 0
    while it < a.max_iters and (time.time() - t_start) < a.minutes * 60:
        it += 1
        buf = {k: [] for k in ("obs", "hist", "cmd", "lin", "act", "lp", "rew", "done", "val", "tout")}
        falls = 0
        terms_acc = {}
        for _ in range(T):
            key, k = jax.random.split(key)
            act, lp, _ = policy_step(params, log_std, obs, hist, cmd, k)
            v = values(critic, params, obs, hist, cmd, lin)
            act_np = np.asarray(act)
            rew, done, info = env.step(act_np)
            for nm, x in (("obs", obs), ("hist", hist), ("cmd", cmd), ("lin", lin), ("act", act_np),
                          ("lp", np.asarray(lp)), ("rew", rew), ("done", done.astype(np.float32)),
                          ("val", np.asarray(v)), ("tout", info["timeout"].astype(np.float32))):
                buf[nm].append(x)
            falls += int(info["fell"].sum())
            for kk, vv in info["terms"].items():
                terms_acc[kk] = terms_acc.get(kk, 0.0) + float(np.mean(vv)) / T
            fin = np.nonzero(done)[0]
            ep_returns += list(info["ep_ret"][fin])
            ep_lens += list(info["ep_t"][fin])
            obs, hist, cmd, lin = env.observe()
        last_v = np.asarray(values(critic, params, obs, hist, cmd, lin))
        B = {k: np.stack(v) for k, v in buf.items()}
        # 시간 초과는 실패가 아니므로 가치로 부트스트랩
        B["rew"] = B["rew"] + gamma * B["val"] * B["tout"]
        adv = np.zeros((T, N), np.float32)
        g = np.zeros(N, np.float32)
        for t in reversed(range(T)):
            nv = last_v if t == T - 1 else B["val"][t + 1]
            nonterm = 1.0 - B["done"][t]
            delta = B["rew"][t] + gamma * nv * nonterm - B["val"][t]
            g = delta + gamma * lam * nonterm * g
            adv[t] = g
        ret = adv + B["val"]
        flat = lambda x: x.reshape(T * N, *x.shape[2:])  # noqa: E731
        data = [flat(B["obs"]), flat(B["hist"]), flat(B["cmd"]), flat(B["lin"]), flat(B["act"]), flat(B["lp"]),
                flat(adv), flat(ret), flat(B["val"])]
        data[6] = (data[6] - data[6].mean()) / (data[6].std() + 1e-8)
        actor_on = 0.0 if (it <= a.critic_warmup and not a.resume) else 1.0
        kls, als, cls, els = [], [], [], []
        nb = 4
        stop = False
        for _ep in range(5):
            perm = np.random.permutation(T * N)
            for mb in np.array_split(perm, nb):
                batch = tuple(jnp.asarray(x[mb]) for x in data)
                (params, log_std, critic, actor_state, critic_state, enc_state,
                 al, cl, el, kl) = update(params, log_std, critic, actor_state, critic_state, enc_state, batch,
                                          lr, 0.2, (0.0 if stop else actor_on), 0)
                kls.append(float(kl))
                als.append(float(al))
                cls.append(float(cl))
                # KL 조기 종료: 사전학습 정책을 한 번에 크게 바꾸지 않도록 (이후 미니배치는 크리틱만 학습)
                if actor_on and float(kl) > 0.02:
                    stop = True
        for _ep in range(2 if (a.train_encoder and actor_on) else 0):
            for mb in np.array_split(np.random.permutation(T * N), nb):
                enc, enc_state, el = update_encoder(params["encoder"], enc_state, jnp.asarray(data[1][mb]),
                                                    jnp.asarray(data[3][mb]))
                params = {"encoder": enc, "policy": params["policy"]}
                els.append(float(el))
        kl = float(np.mean(kls))
        if actor_on:
            lr = lr / 1.5 if kl > 0.03 else (lr * 1.5 if kl < 0.008 else lr)
            lr = float(np.clip(lr, 3e-6, 3e-4))
        fall_rate = falls / (N * T)          # 스텝당 낙상 비율
        fall_hist.append(fall_rate)
        fr = float(np.mean(fall_hist[-10:]))
        # 낙상률(스텝당) 0.0003 = 환경 하나가 평균 67초에 한 번 넘어지는 정도
        if actor_on and it > a.critic_warmup + 10 and fr < 0.0003 and env.difficulty < 1.0:
            env.set_difficulty(env.difficulty + 0.02)
        elif fr > 0.0015 and env.difficulty > 0:
            env.set_difficulty(env.difficulty - 0.02)
        er = float(np.mean(ep_returns[-200:])) if ep_returns else 0.0
        el_ = float(np.mean(ep_lens[-200:])) if ep_lens else 0.0
        rec = dict(it=it, t=round(time.time() - t_start, 1), steps=it * N * T, fall_rate=round(fr, 5),
                   difficulty=round(env.difficulty, 3), ep_ret=round(er, 3), ep_len=round(el_, 1), kl=round(kl, 5),
                   lr=lr, std=[round(float(x), 3) for x in np.exp(np.asarray(log_std))], value_loss=round(float(np.mean(cls)), 4),
                   enc_loss=round(float(np.mean(els)) if els else 0.0, 4),
                   lin_track=round(terms_acc["tracking_lin_vel"], 3), ang_track=round(terms_acc["tracking_ang_vel"], 3))
        log.append(rec)
        if it % 5 == 0 or it == 1:
            print(json.dumps(rec, ensure_ascii=False), flush=True)
        score = rec["lin_track"] + 0.5 * rec["ang_track"] - 50 * fr + 0.5 * env.difficulty
        save = {"params": jax.device_get(params), "critic": jax.device_get(critic),
                "log_std": jax.device_get(log_std), "difficulty": env.difficulty, "it": it}
        if it % 25 == 0:
            with open(os.path.join(a.out, "last.pkl"), "wb") as fp:
                pickle.dump(save, fp)
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
        pickle.dump({"params": jax.device_get(params), "critic": jax.device_get(critic),
                     "log_std": jax.device_get(log_std), "difficulty": env.difficulty, "it": it}, fp)
    print("done", it, "iters", a.out)


if __name__ == "__main__":
    main()
