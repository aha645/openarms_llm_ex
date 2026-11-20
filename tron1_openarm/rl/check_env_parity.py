#!/usr/bin/env python3
"""CPU 학습 환경(tron1_env_cpu)과 GPU 학습 환경(tron1_env_gpu)의 보상·설정이 같은지 확인한다 (보상 항목을 고친 뒤 실행).

    cd tron1_openarm/rl
    env -u PYTHONPATH PYTHONNOUSERSITE=1 ../.venv-biped-train/bin/python check_env_parity.py          # SF·PF
확인하는 것: 보상 계수(train_config.reward_scales)·회전 추종 폭·에피소드 길이·집기 자세 확률·관측 크기가 같고,
두 환경이 몇 스텝 돌며 내는 보상 항목 이름이 같은지. 물리 결과 비교는 mjx_parity.py.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
import jax  # noqa: E402
import jax.numpy as jnp  # noqa: E402
from tron1_env_cpu import Tron1EnvCpu  # noqa: E402
from tron1_env_gpu import Tron1EnvGpu  # noqa: E402

N = 8
for foot in ("sf", "pf"):
    xml = os.path.join(ROOT, "src/tron1_openarm_sim/mjcf", "sf" if foot == "sf" else "", "tron1_openarm_train.xml")
    c = Tron1EnvCpu(xml, num_envs=N, nthread=4, foot=foot)
    g = Tron1EnvGpu(xml, foot=foot, impl="jax", num_envs=N)
    diff = set(c.rs.items()) ^ set(g.rs.items())
    assert not diff, f"{foot} 보상 계수가 다름: {diff}"
    for k in ("ang_sigma", "ep_len", "reach_p", "nj", "obs_dim", "n_cmd"):
        assert getattr(c, k) == getattr(g, k), f"{foot} {k}: CPU {getattr(c, k)} / GPU {getattr(g, k)}"
    c.set_difficulty(0.5)
    for _ in range(20):
        _, _, ci = c.step(np.zeros((N, c.nj), np.float32))
    rp = g.sample_rand(jax.random.PRNGKey(0), N)
    mb, axes = g.batched_model(rp, 0.5, 1.0)
    st = jax.vmap(g.reset, in_axes=(0, None))(jax.random.split(jax.random.PRNGKey(1), N), 0.5)
    _, _, _, gi = jax.jit(jax.vmap(g.step, in_axes=(axes, 0, 0, None)))(mb, st, jnp.zeros((N, g.nj)), 0.5)
    assert set(ci["terms"]) == set(gi["terms"]), f"{foot} 보상 항목 이름이 다름: {set(ci['terms']) ^ set(gi['terms'])}"
    on = sorted(k for k, v in c.rs.items() if v)
    print(f"{foot}: 같음 — 보상 항목 {len(c.rs)}개 (켜진 것 {len(on)}: {', '.join(on)}), 에피소드 {c.ep_len * 0.02:.0f} s, 집기 자세 {c.reach_p}")
print("OK: CPU·GPU 학습 환경의 보상·설정이 같습니다")
