"""색으로 상자 찾기 (마커 없이): RGB-D 한 장에서 빨강·초록·파랑 상자의 윗면 중심과 방향을 구한다.

ArUco 마커는 원래 작업대(위치) 를 찾으려던 것이고, 상자는 색만으로 구분하면 된다 (사용자 제안).
  1) HSV 로 색마다 영역을 나눈다 (작업대 나무색 h≈10, 사람 주황 h≈16 은 빨강 범위 밖)
  2) 영역의 화소를 깊이 영상으로 3D(세계)로 되돌린다
  3) 가장 높은 점에서 1 cm 안의 점 = 윗면 → 수평면 위 최소 넓이 사각형(cv2.minAreaRect)
     → 중심 = 윗면 중심 (마커 중심과 같은 점), 방향 = 사각형 변 방향 (90° 주기, ±45°)
  4) 사각형 변이 상자 크기(5 cm)에서 많이 벗어나면(팔·손가락에 가림, 두 상자 붙음) 버린다
결과는 마커 검출과 같은 형식으로 카메라 서버가 발행 → 집기 앱은 바꾸지 않는다.
"""
import math

import cv2
import numpy as np

# OpenCV HSV (h 0~180, s·v 0~255). 시뮬 상자 rgba: 빨강 (0.85,0.2,0.2) h 0, 초록 (0.2,0.7,0.3) h 64, 파랑 (0.2,0.4,0.9) h 109
HSV_RANGES = {
    "red": [((0, 140, 60), (6, 255, 255)), ((174, 140, 60), (180, 255, 255))],
    "green": [((50, 120, 50), (80, 255, 255))],
    "blue": [((100, 140, 60), (125, 255, 255))],
}
TOP_BAND = 0.010        # 윗면으로 볼 높이 폭 [m]
SIZE_TOL = 0.015        # 윗면 사각형 변 허용 오차 [m]
MIN_TOP_PX = 30         # 윗면 화소가 이보다 적으면 버림
KERNEL = np.ones((3, 3), np.uint8)


def detect(rgb, depth, K, Rw, pw, color_ids, box_size=0.05, z_range=(0.3, 1.5)):
    """rgb·depth(카메라 z 깊이 [m]) → [{id, color, p_w(윗면 중심, 세계), yaw_w, size, n_top, corners(영상 4점)}].
    Rw, pw: 카메라 광학 좌표(x 오른쪽, y 아래, z 앞) → 세계. color_ids: {"red": 0, ...}."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    Kinv = np.linalg.inv(K)
    out = []
    for color, mid in color_ids.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in HSV_RANGES.get(color, []):
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
        n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
        if n <= 1:
            continue
        best = None
        for k in np.argsort(-stats[1:, cv2.CC_STAT_AREA]) + 1:      # 큰 영역부터 — 상자 크기에 맞는 첫 영역
            if stats[k, cv2.CC_STAT_AREA] < MIN_TOP_PX:
                break
            vs, us = np.nonzero(lab == k)
            z = depth[vs, us]
            ok = (z > 0.03) & (z < 5.0)
            if ok.sum() < MIN_TOP_PX:
                continue
            us, vs, z = us[ok], vs[ok], z[ok]
            pc = (Kinv @ np.vstack([us, vs, np.ones_like(us)])) * z          # 카메라 광학 좌표 3×N
            P = (Rw @ pc).T + pw
            if not (z_range[0] < np.percentile(P[:, 2], 50) < z_range[1]):
                continue
            ztop = np.percentile(P[:, 2], 97)
            top = P[:, 2] > ztop - TOP_BAND
            if top.sum() < MIN_TOP_PX:
                continue
            xy = P[top, :2].astype(np.float32)
            (cx, cy), (w, h), ang = cv2.minAreaRect(xy * 1000.0)          # mm 단위로 (작은 값의 수치 문제 피함)
            w, h = w / 1000.0, h / 1000.0
            if abs(w - box_size) > SIZE_TOL or abs(h - box_size) > SIZE_TOL:
                continue
            yaw = math.radians(ang)
            yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4          # 90° 주기 → ±45°
            ztop_mean = float(np.median(P[top, 2]))
            pts_w = cv2.boxPoints(((cx, cy), (w * 1000, h * 1000), ang)) / 1000.0
            corners = []
            for x, y in pts_w:                                                 # 영상에 그릴 윗면 네 모서리
                q = Rw.T @ (np.array([x, y, ztop_mean]) - pw)
                if q[2] > 1e-3:
                    corners.append((K @ (q / q[2]))[:2])
            best = {"id": mid, "color": color, "p_w": np.array([cx / 1000.0, cy / 1000.0, ztop_mean]),
                    "yaw_w": yaw, "size": (round(w, 4), round(h, 4)), "n_top": int(top.sum()),
                    "corners": np.array(corners, np.float32) if len(corners) == 4 else None}
            break
        if best is not None:
            out.append(best)
    return out


def hide_box_markers(m):
    """상자 윗면 마커 판을 상자 색으로 칠한다 → 마커 없는 색 상자 (모델을 그 자리에서 바꿈). 바꾼 개수를 돌려준다."""
    n = 0
    for i in range(m.nbody):
        name = m.body(i).name
        if not name.startswith("box_"):
            continue
        try:
            g, b = m.geom(f"{name}_marker").id, m.geom(f"{name}_geom").id
        except KeyError:
            continue
        m.geom_matid[g] = -1
        m.geom_rgba[g] = m.geom_rgba[b]
        n += 1
    return n


def draw(img, dets):
    for d in dets:
        if d.get("corners") is None:
            continue
        c = d["corners"].astype(np.int32)
        cv2.polylines(img, [c], True, (255, 255, 0), 2)
        cv2.putText(img, f"{d['color']} {d['id']}", tuple(int(v) for v in c[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    return img
