#!/usr/bin/env python3
"""LimX 공개 정책(ONNX: policy.onnx + encoder.onnx) → 이 저장소의 numpy 정책 형식(npz).

    python3 onnx_to_npz.py .../model/SF_TRON1A/policy/isaacgym ../policies/limx_sf_tron1a_pretrained.npz

LimX ONNX 는 MatMul → Add → Elu 가 반복되는 단순 MLP 이다. 가중치를 순서대로 꺼내
encoder_W0, encoder_b0, … policy_W3, policy_b3 로 저장한다 (locomotion_policy.mlp 가 읽는 이름).
"""
import os
import sys

import numpy as np
import onnx
from onnx import numpy_helper


def layers(path):
    m = onnx.load(path)
    init = {t.name: numpy_helper.to_array(t) for t in m.graph.initializer}
    Ws, bs = [], []
    for n in m.graph.node:
        if n.op_type == "MatMul":
            Ws.append(init[n.input[1]])
        elif n.op_type == "Add":
            bs.append(init[n.input[1]] if n.input[1] in init else init[n.input[0]])
    assert len(Ws) == len(bs), path
    return Ws, bs


def main(src_dir, out):
    d = {}
    for prefix, fn in (("encoder", "encoder.onnx"), ("policy", "policy.onnx")):
        Ws, bs = layers(os.path.join(src_dir, fn))
        for i, (W, b) in enumerate(zip(Ws, bs)):
            d[f"{prefix}_W{i}"] = W.astype(np.float32)
            d[f"{prefix}_b{i}"] = b.astype(np.float32)
        print(prefix, [W.shape for W in Ws])
    np.savez(out, **d)
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
