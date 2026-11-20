"""작업대 앞 모서리 찾기 (마커 없이): 머리 카메라 깊이 영상 → 상판 높이·앞 모서리까지 거리·방향.

작업대 마커는 다가가야 읽을 수 있어 '어느 작업대인지' 를 정하는 데는 맞지 않는다 (사용자 지적) — 어느 작업대인지는
지도 좌표(locations.yaml, 출발 지점 기준 왼쪽/오른쪽)로 정하고, 도착한 뒤 작업대에 대한 정밀 정렬만 이것으로 한다.
  1) 깊이 → base_footprint 3D 점 (영상을 STRIDE 화소 간격으로). 로봇 자신(팔·손·상자 든 손)은 분할 영상으로 뺀다
     (실로봇의 self-filter 와 같음)
  2) 높이 0.55~0.95 m 점의 높이 분포에서 가장 큰 봉우리 = 상판 높이 (상자 윗면은 5 cm 위라 따로 떨어짐)
  3) 상판 점을 옆(y) 2 cm 칸으로 나눠 칸마다 가장 가까운(x 최소) 점 = 앞 모서리 후보 → 직선 x = a + b·y 를 RANSAC 으로
     (모든 두 점 쌍 중 1 cm 안에 드는 칸이 가장 많은 직선 → 그 칸들로 최소제곱). 화면 양옆에서는 '가장 가까운 점' 이
     모서리가 아니라 시야 경계(비스듬한 선)라서, 단순 최소제곱은 방향이 7~8° 틀렸다 (오프라인 시험).
     거리 = a (y=0 에서), 정면으로 마주 보려면 돌 각도 = −atan(b)
  4) 점유 칸: 모서리 안쪽(작업대 위)에서 상판보다 1~30 cm 높은 점 → 2 cm 칸 (상자·모르는 물건 모두. 로봇 자신은 1) 에서 뺐다)
     → 놓을 빈자리 찾기 (robot_skills._free_place). 손에 든 상자는 로봇 몸이 아니라 여기 들어오므로 쓰는 쪽에서 손끝 근처를 뺀다
"""
import math

import numpy as np

STRIDE = 4
Z_RANGE = (0.55, 0.95)
X_RANGE = (0.15, 1.6)
Y_MAX = 0.7
BIN_Z = 0.005
TOP_TOL = 0.008
BIN_Y = 0.02
MIN_TOP = 300
MIN_BINS = 8
INLIER = 0.01
OCC_Z = (0.010, 0.30)    # 점유로 볼 상판 위 높이 [m]
OCC_DEPTH = 0.60         # 모서리에서 안쪽으로 볼 깊이 [m]
OCC_CELL = 0.02


def detect(depth, K, Rw, pw, to_base, robot_mask=None):
    """depth: 카메라 z 깊이 [m]. Rw, pw: 카메라 광학 → 세계. to_base(P_w Nx3) → base_footprint Nx3.
    robot_mask: 로봇 자신이 찍힌 화소 True (같은 크기). 결과 dict 또는 None."""
    H, W = depth.shape
    vs, us = np.mgrid[STRIDE // 2:H:STRIDE, STRIDE // 2:W:STRIDE]
    vs, us = vs.ravel(), us.ravel()
    z = depth[vs, us]
    ok = (z > 0.05) & (z < 3.0)
    if robot_mask is not None:
        ok &= ~robot_mask[vs, us]
    us, vs, z = us[ok], vs[ok], z[ok]
    if len(z) < MIN_TOP:
        return None
    Kinv = np.linalg.inv(K)
    pc = (Kinv @ np.vstack([us, vs, np.ones_like(us)])) * z
    P = to_base((Rw @ pc).T + pw)
    P_all = P
    sel = (P[:, 2] > Z_RANGE[0]) & (P[:, 2] < Z_RANGE[1]) & (P[:, 0] > X_RANGE[0]) & (P[:, 0] < X_RANGE[1]) \
        & (np.abs(P[:, 1]) < Y_MAX)
    P = P[sel]
    if len(P) < MIN_TOP:
        return None
    hist, edges = np.histogram(P[:, 2], bins=np.arange(Z_RANGE[0], Z_RANGE[1] + BIN_Z, BIN_Z))
    k = int(np.argmax(hist))
    if hist[k] < MIN_TOP // 3:
        return None
    zc = 0.5 * (edges[k] + edges[k + 1])
    top = P[np.abs(P[:, 2] - zc) < TOP_TOL]
    if len(top) < MIN_TOP:
        return None
    h = float(np.median(top[:, 2]))
    yb = np.floor(top[:, 1] / BIN_Y).astype(int)
    pts = []
    for b in np.unique(yb):
        m = yb == b
        if m.sum() >= 3:
            i = np.argmin(top[m, 0])
            pts.append(top[m][i, :2])
    if len(pts) < MIN_BINS:
        return None
    pts = np.array(pts)
    best = None
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            dy = pts[j, 1] - pts[i, 1]
            if abs(dy) < 0.06:
                continue
            b = (pts[j, 0] - pts[i, 0]) / dy
            if abs(b) > 1.0:                                  # 45° 넘게 돌아선 모서리는 보지 않음
                continue
            a = pts[i, 0] - b * pts[i, 1]
            inl = np.abs(pts[:, 0] - (a + b * pts[:, 1])) < INLIER
            if best is None or inl.sum() > best.sum():
                best = inl
    if best is None or best.sum() < MIN_BINS or np.ptp(pts[best, 1]) < 0.15:
        return None
    keep = best
    A = np.vstack([np.ones(keep.sum()), pts[keep, 1]]).T
    a, b = np.linalg.lstsq(A, pts[keep, 0], rcond=None)[0]
    res = np.abs(pts[keep, 0] - (a + b * pts[keep, 1]))
    xe = a + b * P_all[:, 1]                                   # 각 점의 옆 위치에서 모서리 x
    above = (P_all[:, 2] > h + OCC_Z[0]) & (P_all[:, 2] < h + OCC_Z[1]) & (P_all[:, 0] > xe + 0.005) \
        & (P_all[:, 0] < xe + OCC_DEPTH) & (np.abs(P_all[:, 1]) < Y_MAX)
    occ = []
    if above.any():
        cells, cnt = np.unique(np.floor(P_all[above, :2] / OCC_CELL).astype(int), axis=0, return_counts=True)
        occ = [[round((cx + 0.5) * OCC_CELL, 3), round((cy + 0.5) * OCC_CELL, 3)] for (cx, cy), n in zip(cells, cnt) if n >= 2]
    return {"dist": round(float(a), 4), "yaw": round(-math.atan(b), 4), "height": round(h, 4),
            "n_top": int(len(top)), "n_edge": int(keep.sum()), "rms_cm": round(100 * float(np.sqrt((res ** 2).mean())), 2),
            "y_span": [round(float(pts[keep, 1].min()), 3), round(float(pts[keep, 1].max()), 3)],
            "occ": occ, "occ_cell": OCC_CELL}


def robot_geoms(m, base_body="base_link"):
    """로봇 자신의 geom 표 (분할 영상에서 빼기용): base_link 와 같은 뿌리 몸체에 달린 geom."""
    root = m.body_rootid[m.body(base_body).id]
    return np.array([m.body_rootid[m.geom_bodyid[g]] == root for g in range(m.ngeom)], bool)


def robot_mask_from_seg(seg, is_robot):
    """mujoco.Renderer 분할 영상 (H, W, 2: [id, 종류]) → 로봇 화소 True."""
    import mujoco
    ids, typ = seg[..., 0], seg[..., 1]
    m = (typ == int(mujoco.mjtObj.mjOBJ_GEOM)) & (ids >= 0)
    out = np.zeros(ids.shape, bool)
    out[m] = is_robot[ids[m]]
    return out
