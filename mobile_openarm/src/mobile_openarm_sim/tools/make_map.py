#!/usr/bin/env python3
"""MuJoCo 월드에서 '정답 지도'(LiDAR 높이 단면)를 만든다 → maps/world_map.{pgm,yaml}

SLAM(Day1-04)을 건너뛰고 Nav2(Day1-05)부터 실습하거나, SLAM 결과와 비교할 때 사용.
    python3 make_map.py --world ../mjcf/world.xml --out ../maps/world_map
"""
import argparse
import os

import mujoco
import numpy as np

SKIP_PREFIX = ("box_", "person", "floor")


def main(world, out, res=0.05, z=0.33):
    m = mujoco.MjModel.from_xml_path(world)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    robot_root = m.body("base_link").id
    xmin, xmax, ymin, ymax = -5.5, 5.5, -4.5, 4.5
    W = int(round((xmax - xmin) / res))
    H = int(round((ymax - ymin) / res))
    xs = xmin + (np.arange(W) + 0.5) * res
    ys = ymin + (np.arange(H) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    P = np.stack([X.ravel(), Y.ravel(), np.full(X.size, z)], 1)
    occ = np.zeros(X.size, bool)
    for g in range(m.ngeom):
        name = m.geom(g).name
        b = m.geom_bodyid[g]
        # 로봇 몸체 제외
        root = b
        while m.body_parentid[root] != 0:
            root = m.body_parentid[root]
        if root == robot_root or name.startswith(SKIP_PREFIX) or m.geom_group[g] > 1:
            continue
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX:
            continue
        R = d.geom_xmat[g].reshape(3, 3)
        local = (P - d.geom_xpos[g]) @ R
        if name.endswith("_top"):
            # 작업대 상판: 라이다엔 다리만 보이지만, 로봇이 밑으로 파고들지 않도록 지도에선 통째로 막음
            local[:, 2] = 0.0
        inside = np.all(np.abs(local) <= m.geom_size[g] + 1e-9, axis=1)
        occ |= inside
    img = np.where(occ.reshape(H, W), 0, 254).astype(np.uint8)[::-1]   # PGM 은 위가 +y
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out + ".pgm", "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode())
        f.write(img.tobytes())
    with open(out + ".yaml", "w") as f:
        f.write(f"image: {os.path.basename(out)}.pgm\nmode: trinary\nresolution: {res}\n"
                f"origin: [{xmin}, {ymin}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f"wrote {out}.pgm ({W}x{H}), occupied cells={occ.sum()}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    main(a.world, a.out)
