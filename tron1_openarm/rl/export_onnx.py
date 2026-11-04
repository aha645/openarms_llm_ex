#!/usr/bin/env python3
"""학습한 정책(npz) → LimX 배포 형식 ONNX (encoder.onnx, policy.onnx).

    python3 export_onnx.py ../policies/tron1_openarm_policy.npz ../policies/onnx

입출력 이름·크기가 LimX tron1-rl-deploy-python 과 같다 (입력 'mlp_input': 인코더 300 / 정책 36).
→ 실로봇에서 LimX 배포 코드(PointfootController.py)의 model 폴더에 넣어 시험할 수 있는 형태.
   (단, 실로봇에 OpenArm 을 얹었을 때의 안전 검증은 별도로 반드시 해야 한다 — doc/tron1/04-real-robot.md)
"""
import os
import sys

import numpy as np
import onnx
import onnxruntime as ort
from onnx import TensorProto, helper, numpy_helper


def build(npz, prefix, path):
    Ws, bs = [], []
    i = 0
    while f"{prefix}_W{i}" in npz:
        Ws.append(npz[f"{prefix}_W{i}"])
        bs.append(npz[f"{prefix}_b{i}"])
        i += 1
    nodes, inits = [], []
    x = "mlp_input"
    for i, (W, b) in enumerate(zip(Ws, bs)):
        inits += [numpy_helper.from_array(W.astype(np.float32), f"W{i}"),
                  numpy_helper.from_array(b.astype(np.float32), f"b{i}")]
        nodes.append(helper.make_node("MatMul", [x, f"W{i}"], [f"mm{i}"]))
        out = f"lin{i}" if i < len(Ws) - 1 else "output"
        nodes.append(helper.make_node("Add", [f"mm{i}", f"b{i}"], [out]))
        if i < len(Ws) - 1:
            nodes.append(helper.make_node("Elu", [out], [f"h{i}"], alpha=1.0))
            x = f"h{i}"
    g = helper.make_graph(nodes, prefix,
                          [helper.make_tensor_value_info("mlp_input", TensorProto.FLOAT, [Ws[0].shape[0]])],
                          [helper.make_tensor_value_info("output", TensorProto.FLOAT, [Ws[-1].shape[1]])], inits)
    model = helper.make_model(g, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    onnx.save(model, path)
    # 검증
    xin = np.random.randn(Ws[0].shape[0]).astype(np.float32)
    h = xin
    for i in range(len(Ws)):
        h = h @ Ws[i] + bs[i]
        if i < len(Ws) - 1:
            h = np.where(h > 0, h, np.expm1(np.minimum(h, 0)))
    y = ort.InferenceSession(path).run(None, {"mlp_input": xin})[0]
    print(f"{path}: numpy 와 최대 차이 {np.abs(y - h).max():.2e}")


if __name__ == "__main__":
    npz = np.load(sys.argv[1])
    out = sys.argv[2]
    os.makedirs(out, exist_ok=True)
    build(npz, "encoder", os.path.join(out, "encoder.onnx"))
    build(npz, "policy", os.path.join(out, "policy.onnx"))
