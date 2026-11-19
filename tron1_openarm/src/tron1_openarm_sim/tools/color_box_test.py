"""상자 인식 비교 — ArUco 마커 vs 색(color_boxes): 머리 카메라 한 장에서 잰 상자 윗면 중심·방향을 정답과 비교.
    python3 tools/color_box_test.py <world.xml> <정책.npz> [배치 파일, 기본 ../../tools/validation/series_layouts.txt]

집기 시험과 같은 자리(작업대 앞 2.5, 2.45, 90°)에 서서 6 s 뒤 머리 카메라(960x720)를 렌더링한다.
배치마다: 마커/색 각각 상자별 위치 오차 [cm]·방향 오차 [°]. 끝에 요약(JSON).
카메라 서버(camera_server.py)와 같은 처리: 마커는 깊이 보정 + 수평면 방향, 색은 color_boxes.detect.
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
from mobile_openarm_sim.aruco_utils import ArucoDetector  # noqa: E402
from mobile_openarm_sim import color_boxes  # noqa: E402
from tron1_openarm_sim.biped_core import BipedOpenArmSim  # noqa: E402
from tron1_openarm_sim.bridge_biped import parse_box_poses  # noqa: E402
from mobile_openarm_sim.camera_server import OPT, depth_correct, flat_rotation  # noqa: E402

COLOR_IDS = {"red": 0, "green": 1, "blue": 2}
W, H = 960, 720


def yaw90(a):
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


def box_truth(m, d, name):
    b = m.body(name).id
    R = d.xmat[b].reshape(3, 3)
    top = d.xpos[b] + R @ np.array([0, 0, 0.025])
    return top, math.atan2(R[1, 0], R[0, 0])


def measure(sim, r, which=("aruco", "color")):
    m, d = sim.m, sim.d
    cid = m.camera("head_camera").id
    fy = (H / 2) / math.tan(math.radians(m.cam_fovy[cid]) / 2)
    K = np.array([[fy, 0, W / 2], [0, fy, H / 2], [0, 0, 1.0]])
    r.update_scene(d, camera=cid)
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
    rgb = r.render().copy()
    r.enable_depth_rendering()
    r.update_scene(d, camera=cid)
    depth = r.render().copy()
    r.disable_depth_rendering()
    Rw, pw = d.cam_xmat[cid].reshape(3, 3) @ OPT, d.cam_xpos[cid].copy()
    out = {"aruco": {}, "color": {}}
    dets = ArucoDetector("DICT_4X4_50", 0.04, K).detect(rgb) if "aruco" in which else []
    depth_correct(depth, K, dets)
    for det in dets:
        pm = Rw @ det["position"] + pw
        Rm = flat_rotation(K, det, Rw, pw, pm)
        out["aruco"][int(det["id"])] = (pm, math.atan2(Rm[1, 0], Rm[0, 0]))
    for c in (color_boxes.detect(rgb, depth, K, Rw, pw, COLOR_IDS) if "color" in which else []):
        out["color"][c["id"]] = (c["p_w"], c["yaw_w"])
    return out, rgb


def main():
    world, policy = sys.argv[1], sys.argv[2]
    lay = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, "../../../tools/validation/series_layouts.txt")
    layouts = [ln.split() for ln in open(lay, encoding="utf-8") if ln.strip() and not ln.startswith("#")]
    layouts = [("default", None, "")] + [(a[0], a[1], a[2]) for a in layouts]
    errs = {"aruco": [], "color": []}
    miss = {"aruco": 0, "color": 0}
    for name, _obj, boxes in layouts:
        sim = BipedOpenArmSim(world, enable_camera=False, move_person=False, policy_path=policy)
        sim.set_base_pose(2.5, 2.45, math.pi / 2)
        for b, (x, y, yaw) in parse_box_poses(boxes).items():
            sim.set_box_pose(b, x, y, math.radians(yaw))
        for _ in range(600):
            sim.set_cmd_vel(0, 0)
            sim.step(5)
        r = mujoco.Renderer(sim.m, H, W)
        res, rgb = measure(sim, r, ("aruco",))
        color_boxes.hide_box_markers(sim.m)                     # 같은 장면에서 마커만 지우고 색으로
        res["color"] = measure(sim, r, ("color",))[0]["color"]
        r.close()
        row = {"layout": name}
        for mid in (0, 1, 2):
            top, yaw = box_truth(sim.m, sim.d, f"box_{mid}")
            for k in ("aruco", "color"):
                if mid in res[k]:
                    p, y = res[k][mid]
                    e = (round(100 * float(np.linalg.norm(p - top)), 2), round(math.degrees(abs(yaw90(y - yaw))), 1))
                    errs[k].append(e)
                    row[f"{k}{mid}"] = e
                else:
                    miss[k] += 1
                    row[f"{k}{mid}"] = "없음"
        print(json.dumps(row, ensure_ascii=False), flush=True)
        if os.environ.get("SAVE"):
            import cv2
            img = color_boxes.draw(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [])
            cv2.imwrite(os.path.join(os.environ["SAVE"], f"head_{name}.png"), img)
    summ = {"summary": True}
    for k in errs:
        a = np.array(errs[k]) if errs[k] else np.zeros((1, 2))
        summ[k] = {"n": len(errs[k]), "miss": miss[k], "pos_cm_mean": round(float(a[:, 0].mean()), 2),
                   "pos_cm_max": round(float(a[:, 0].max()), 2), "yaw_deg_mean": round(float(a[:, 1].mean()), 1),
                   "yaw_deg_max": round(float(a[:, 1].max()), 1)}
    print(json.dumps(summ, ensure_ascii=False))


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    main()
