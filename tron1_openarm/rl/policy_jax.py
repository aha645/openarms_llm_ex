"""JAX 로 쓴 액터(인코더+정책)·크리틱 MLP. 가중치 형식은 locomotion_policy.py(npz) 와 같다."""
import jax
import jax.numpy as jnp
import numpy as np


def mlp_init(key, sizes, scale_last=1.0):
    ps = []
    for i, (a, b) in enumerate(zip(sizes[:-1], sizes[1:])):
        key, k = jax.random.split(key)
        w = jax.random.normal(k, (a, b)) * np.sqrt(2.0 / a)
        if i == len(sizes) - 2:
            w = w * scale_last
        ps.append((w.astype(jnp.float32), jnp.zeros(b, jnp.float32)))
    return ps


def mlp_apply(ps, x):
    for i, (w, b) in enumerate(ps):
        x = x @ w + b
        if i < len(ps) - 1:
            x = jax.nn.elu(x)
    return x


def from_npz(npz, prefix):
    ps, i = [], 0
    while f"{prefix}_W{i}" in npz:
        ps.append((jnp.asarray(npz[f"{prefix}_W{i}"]), jnp.asarray(npz[f"{prefix}_b{i}"])))
        i += 1
    return ps


def to_npz_dict(ps, prefix):
    out = {}
    for i, (w, b) in enumerate(ps):
        out[f"{prefix}_W{i}"] = np.asarray(w, np.float32)
        out[f"{prefix}_b{i}"] = np.asarray(b, np.float32)
    return out


def actor_mean(params, obs, hist, cmd):
    est = jax.lax.stop_gradient(mlp_apply(params["encoder"], hist))   # LimX: output_detach=True
    return mlp_apply(params["policy"], jnp.concatenate([est, obs, cmd], -1))
