#!/usr/bin/env python3
"""실습용 MuJoCo 월드 생성기: 방 2개 + 복도문 + 작업대 2개 + ArUco 상자 3개 + 움직이는 사람.

    python3 build_world.py --out ../mjcf

생성물: mjcf/world.xml, mjcf/textures/aruco_*.png, config/locations.yaml
"""
import argparse
import os

import cv2
import numpy as np
import yaml

ARUCO_DICT = "DICT_4X4_50"
MARKER_SIZE = 0.04       # m (검은 테두리 포함 마커 한 변)
BOX_SIZE = 0.05          # m (정육면체 한 변) — OpenArm v1 그리퍼 최대 벌림 약 88mm
TABLE_H = 0.70

# 작업대 (x, y, 반폭x, 반깊이y)
PICK_TABLE = (2.5, 3.1, 0.45, 0.25)
PLACE_TABLE = (-2.5, 3.1, 0.45, 0.25)
BOXES = [  # (marker_id, x, y, rgba)
    (0, 2.62, 2.97, "0.85 0.2 0.2 1"),
    (1, 2.42, 2.99, "0.2 0.7 0.3 1"),
    (2, 2.80, 3.05, "0.2 0.4 0.9 1"),
]

LOCATIONS = {
    # 이름: [x, y, yaw(deg)]  — map 좌표 (SLAM 시작 위치 = 월드 원점 기준)
    "home": [0.0, 0.0, 0.0],
    # 작업대 앞 0.70m 지점: Nav2 가 팽창(inflation) 영역 밖에서 여유 있게 도착하고,
    # 이후 '도킹'(yaw 보정 + 직진 미세이동)으로 0.40m 까지 접근
    "pick_station": [2.5, 2.15, 90.0],
    "place_station": [-2.5, 2.15, 90.0],
    "door": [0.0, 1.5, 90.0],
    "patrol_1": [3.5, -2.8, 0.0],
    "patrol_2": [3.5, 0.6, 90.0],
    "patrol_3": [-3.5, 0.6, 180.0],
    "patrol_4": [-3.5, -2.8, -90.0],
}


def marker_png(dict_name, mid, px=200, border_px=20):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    if hasattr(cv2.aruco, "generateImageMarker"):        # OpenCV >= 4.7
        img = cv2.aruco.generateImageMarker(d, mid, px)
    else:                                               # OpenCV 4.6 (Ubuntu 24.04 apt)
        img = cv2.aruco.drawMarker(d, mid, px)
    img = cv2.copyMakeBorder(img, border_px, border_px, border_px, border_px,
                             cv2.BORDER_CONSTANT, value=255)
    return img


def wall(name, x, y, sx, sy, h=1.0):
    return (f'    <geom name="{name}" type="box" pos="{x} {y} {h/2}" size="{sx} {sy} {h/2}" '
            f'material="wall" group="0"/>\n')


def table(name, x, y, hx, hy):
    s = f'    <body name="{name}" pos="{x} {y} 0">\n'
    s += (f'      <geom name="{name}_top" type="box" pos="0 0 {TABLE_H-0.02}" size="{hx} {hy} 0.02" '
          f'material="wood" group="0"/>\n')
    for i, (lx, ly) in enumerate([(hx-0.04, hy-0.04), (-hx+0.04, hy-0.04), (hx-0.04, -hy+0.04), (-hx+0.04, -hy+0.04)]):
        s += (f'      <geom name="{name}_leg{i}" type="box" pos="{lx} {ly} {(TABLE_H-0.04)/2}" '
              f'size="0.025 0.025 {(TABLE_H-0.04)/2}" material="metal" group="0"/>\n')
    s += '    </body>\n'
    return s


def build(out_dir, cfg_dir):
    tex_dir = os.path.join(out_dir, "textures")
    os.makedirs(tex_dir, exist_ok=True)
    for mid, *_ in BOXES:
        # 테두리(quiet zone) 포함 이미지를 plate 전체에 입힘 → 실제 검은 마커 크기 = MARKER_SIZE
        img = marker_png(ARUCO_DICT, mid)
        cv2.imwrite(os.path.join(tex_dir, f"aruco_{mid}.png"), img)
    plate_half = MARKER_SIZE / 2 * (240 / 200)   # 흰 테두리(quiet zone)까지 포함한 plate 반폭 = 24mm

    x = []
    x.append('<mujoco model="openarm_mobile_world">\n')
    x.append('  <include file="mobile_openarm.xml"/>\n')
    x.append('  <statistic center="0 0 0.8" extent="4"/>\n')
    x.append('  <visual>\n    <headlight diffuse="0.5 0.5 0.5" ambient="0.35 0.35 0.35" specular="0 0 0"/>\n'
             '    <global azimuth="135" elevation="-25" offwidth="1280" offheight="720"/>\n'
             '    <quality shadowsize="2048"/>\n  </visual>\n')
    x.append('  <asset>\n')
    x.append('    <texture type="skybox" builtin="gradient" rgb1="0.6 0.7 0.8" rgb2="0.2 0.2 0.25" width="256" height="1536"/>\n')
    x.append('    <texture type="2d" name="floor_tex" builtin="checker" mark="edge" rgb1="0.75 0.75 0.72" '
             'rgb2="0.68 0.68 0.65" markrgb="0.5 0.5 0.5" width="300" height="300"/>\n')
    x.append('    <material name="floor" texture="floor_tex" texuniform="true" texrepeat="8 8" reflectance="0.05"/>\n')
    x.append('    <material name="wall" rgba="0.92 0.9 0.85 1"/>\n')
    x.append('    <material name="wood" rgba="0.6 0.45 0.3 1"/>\n')
    x.append('    <material name="metal" rgba="0.5 0.5 0.55 1"/>\n')
    x.append('    <material name="person" rgba="0.95 0.6 0.2 1"/>\n')
    for mid, *_ in BOXES:
        x.append(f'    <texture type="2d" name="aruco_{mid}_tex" file="textures/aruco_{mid}.png"/>\n')
        x.append(f'    <material name="aruco_{mid}" texture="aruco_{mid}_tex" rgba="1 1 1 1" specular="0" shininess="0"/>\n')
    x.append('  </asset>\n')

    x.append('  <worldbody>\n')
    x.append('    <light pos="0 0 4" dir="0 0 -1" directional="true" diffuse="0.5 0.5 0.5" castshadow="false"/>\n')
    x.append('    <geom name="floor" type="plane" size="0 0 0.05" material="floor" group="0"/>\n')
    # 외벽 10m x 8m
    x.append(wall("wall_n", 0, 4.05, 5.1, 0.05))
    x.append(wall("wall_s", 0, -4.05, 5.1, 0.05))
    x.append(wall("wall_e", 5.05, 0, 0.05, 4.1))
    x.append(wall("wall_w", -5.05, 0, 0.05, 4.1))
    # 북쪽 작업실 / 남쪽 순찰구역 칸막이 (가운데 2m 문)
    x.append(wall("part_w", -3.0, 1.5, 2.0, 0.05))
    x.append(wall("part_e", 3.0, 1.5, 2.0, 0.05))
    # 남쪽 구역 기둥/선반 (SLAM 특징점 + 회피 대상)
    x.append(wall("pillar_1", 1.8, -1.6, 0.2, 0.2))
    x.append(wall("pillar_2", -1.8, -1.6, 0.2, 0.2))
    x.append(wall("shelf_1", 4.6, -1.0, 0.35, 0.6, h=1.2))
    x.append(wall("shelf_2", -4.6, -1.0, 0.35, 0.6, h=1.2))
    x.append(wall("crate", 0.0, -3.4, 0.4, 0.3, h=0.6))
    # 작업대
    x.append(table("pick_table", *PICK_TABLE))
    x.append(table("place_table", *PLACE_TABLE))
    # ArUco 상자
    for mid, bx, by, rgba in BOXES:
        h = BOX_SIZE / 2
        x.append(f'    <body name="box_{mid}" pos="{bx} {by} {TABLE_H + h + 0.001}">\n')
        x.append('      <freejoint/>\n')
        x.append(f'      <geom name="box_{mid}_geom" type="box" size="{h} {h} {h}" rgba="{rgba}" mass="0.08" '
                 'friction="1.5 0.02 0.002" condim="4" group="1" solref="0.005 1"/>\n')
        x.append(f'      <geom name="box_{mid}_marker" type="box" pos="0 0 {h + 0.0004}" '
                 f'size="{plate_half:.4f} {plate_half:.4f} 0.0004" material="aruco_{mid}" '
                 'contype="0" conaffinity="0" group="1" mass="0"/>\n')
        x.append('    </body>\n')
    # 움직이는 사람 (mocap: bridge 노드가 왕복 이동시킴)
    x.append('    <body name="person" mocap="true" pos="-3.0 -1.0 0">\n')
    x.append('      <geom name="person_body" type="capsule" fromto="0 0 0.25 0 0 1.45" size="0.22" material="person" group="0"/>\n')
    x.append('      <geom name="person_head" type="sphere" pos="0 0 1.62" size="0.12" material="person" group="0"/>\n')
    x.append('    </body>\n')
    # 관찰용 카메라
    x.append('    <camera name="overview" pos="0 -7.5 7" xyaxes="1 0 0 0 0.68 0.73" fovy="55"/>\n')
    x.append('  </worldbody>\n')
    x.append('</mujoco>\n')
    with open(os.path.join(out_dir, "world.xml"), "w", encoding="utf-8") as fp:
        fp.write("".join(x))
    # 저사양 모드 월드 (build_mjcf.py --light 로 만든 mobile_openarm_light.xml 사용)
    with open(os.path.join(out_dir, "world_light.xml"), "w", encoding="utf-8") as fp:
        fp.write("".join(x).replace('file="mobile_openarm.xml"', 'file="mobile_openarm_light.xml"'))

    os.makedirs(cfg_dir, exist_ok=True)
    with open(os.path.join(cfg_dir, "locations.yaml"), "w", encoding="utf-8") as fp:
        yaml.safe_dump({
            "aruco": {"dictionary": ARUCO_DICT, "marker_size": MARKER_SIZE, "box_size": BOX_SIZE,
                      "table_height": TABLE_H},
            "objects": {f"box_{mid}": {"marker_id": mid, "color": c}
                        for (mid, *_), c in zip(BOXES, ["red", "green", "blue"])},
            "locations": LOCATIONS,
            "tables": {n: {"center": [t[0], t[1]], "size": [2 * t[2], 2 * t[3]], "height": TABLE_H}
                       for n, t in (("pick_table", PICK_TABLE), ("place_table", PLACE_TABLE))},
            "station_table": {"pick_station": "pick_table", "place_station": "place_table"},
            "patrol_route": ["patrol_1", "patrol_2", "patrol_3", "patrol_4"],
        }, fp, allow_unicode=True, sort_keys=False)
    print("world written to", out_dir)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="mjcf directory")
    ap.add_argument("--config", required=True, help="config directory")
    a = ap.parse_args()
    build(a.out, a.config)
