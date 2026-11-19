"""작업대 앞 모서리 인식 시험 (table_edge.py): 작업대 앞 여러 자리·방향에 서서 머리 카메라 깊이로 잰 모서리 거리·방향을 정답과 비교.
    python3 tools/table_edge_test.py <world.xml> <정책.npz>
자리: 모서리까지 0.35/0.55/0.75 m × 방향 0/±10°/+15° × 옆 0/±0.2 m 일부, 팔 home 과 carry(시야 안의 팔 — self-filter 확인).
"""
import json
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "../../../../mobile_openarm/src/mobile_openarm_sim"))
from tron1_openarm_sim import table_edge  # noqa: E402
from tron1_openarm_sim.arm_poses import POSES  # noqa: E402
from tron1_openarm_sim.biped_core import BipedOpenArmSim  # noqa: E402
from tron1_openarm_sim.camera_server import OPT  # noqa: E402

W, H = 960, 720
EDGE_Y, TABLE_X = 2.85, 2.5          # pick_table 앞 모서리 (월드 y), 중심 x


def run_case(world, policy, dist, yaw_off_deg, lat, arms):
    sim = BipedOpenArmSim(world, enable_camera=False, move_person=False, policy_path=policy)
    yaw = math.pi / 2 + math.radians(yaw_off_deg)
    # 로봇이 바라보는 방향으로 모서리까지 dist 가 되도록 (y=0 시선)
    bx, by = TABLE_X - lat, EDGE_Y - dist * math.sin(yaw)
    bx -= dist * math.cos(yaw)
    sim.set_base_pose(bx, by, yaw)
    for _ in range(300):
        sim.set_cmd_vel(0, 0)
        sim.step(5)
    if arms == "carry":
        for s, k in (("left", 0), ("right", 1)):
            sim.start_arm_trajectory(s, [f"openarm_{s}_joint{i}" for i in range(1, 8)], [(2.0, list(POSES["carry"][k]), None)])
    for _ in range(500):
        sim.set_cmd_vel(0, 0)
        sim.step(5)
    m, d = sim.m, sim.d
    cid = m.camera("head_camera").id
    fy = (H / 2) / math.tan(math.radians(m.cam_fovy[cid]) / 2)
    K = np.array([[fy, 0, W / 2], [0, fy, H / 2], [0, 0, 1.0]])
    r = mujoco.Renderer(m, H, W)
    r.enable_depth_rendering()
    r.update_scene(d, camera=cid)
    depth = r.render().copy()
    r.disable_depth_rendering()
    r.enable_segmentation_rendering()
    r.update_scene(d, camera=cid)
    seg = r.render().copy()
    r.disable_segmentation_rendering()
    r.close()
    Rw, pw = d.cam_xmat[cid].reshape(3, 3) @ OPT, d.cam_xpos[cid].copy()
    b = sim.base_id
    p0 = d.xpos[b].copy()
    yb = math.atan2(d.xmat[b].reshape(3, 3)[1, 0], d.xmat[b].reshape(3, 3)[0, 0])
    Rz = np.array([[math.cos(yb), -math.sin(yb), 0], [math.sin(yb), math.cos(yb), 0], [0, 0, 1]])
    to_base = lambda P: (P - np.array([p0[0], p0[1], 0.0])) @ Rz  # noqa: E731  (정답 자세로 — 시험은 인식만 본다)
    mask = table_edge.robot_mask_from_seg(seg, table_edge.robot_geoms(m))
    res = table_edge.detect(depth, K, Rw, pw, to_base, mask)
    # 정답 (실제 서 있는 자세 기준)
    true_dist = (EDGE_Y - p0[1]) / math.sin(yb)
    true_yaw = math.pi / 2 - yb
    row = {"dist": dist, "yaw_off": yaw_off_deg, "lat": lat, "arms": arms}
    if res is None:
        row["result"] = "없음"
    else:
        occ = np.array(res["occ"]) if res["occ"] else np.zeros((0, 2))
        boxes = [to_base(d.xpos[m.body(f"box_{i}").id][None])[0][:2] for i in range(3)]
        hit = sum(1 for bp in boxes if len(occ) and np.min(np.linalg.norm(occ - bp, axis=1)) < 0.03)
        false = int(sum(1 for c in occ if min(np.linalg.norm(c - bp) for bp in boxes) > 0.05))
        row.update(occ_boxes=hit, occ_false=false)
        row.update(err_dist_cm=round(100 * (res["dist"] - true_dist), 2),
                   err_yaw_deg=round(math.degrees(res["yaw"] - true_yaw), 2),
                   height=res["height"], n_edge=res["n_edge"], rms_cm=res["rms_cm"])
    return row


def main():
    world, policy = sys.argv[1], sys.argv[2]
    cases = [(0.35, 0, 0, "home"), (0.55, 0, 0, "home"), (0.75, 0, 0, "home"),
             (0.55, 10, 0, "home"), (0.55, -10, 0, "home"), (0.75, 15, 0, "home"),
             (0.45, 0, 0.2, "home"), (0.45, 0, -0.2, "home"), (0.65, -10, 0.15, "home"),
             (0.35, 0, 0, "carry"), (0.55, 10, 0, "carry"), (0.45, -10, -0.15, "carry")]
    rows = []
    for c in cases:
        rows.append(run_case(world, policy, *c))
        print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
    ok = [r for r in rows if "err_dist_cm" in r]
    ed = np.abs([r["err_dist_cm"] for r in ok]) if ok else np.zeros(1)
    ey = np.abs([r["err_yaw_deg"] for r in ok]) if ok else np.zeros(1)
    print(json.dumps({"summary": True, "n": len(rows), "found": len(ok), "dist_cm_mean": round(float(ed.mean()), 2),
                      "dist_cm_max": round(float(ed.max()), 2), "yaw_deg_mean": round(float(ey.mean()), 2),
                      "yaw_deg_max": round(float(ey.max()), 2)}, ensure_ascii=False))


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    main()
