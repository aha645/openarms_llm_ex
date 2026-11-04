#!/usr/bin/env python3
"""실습 월드(방·작업대·ArUco 상자·사람)는 바퀴형과 똑같이 쓰고, 로봇만 TRON1+OpenArm 으로 바꾼다.

    python3 build_world_tron1.py --wheeled-tools <mobile_openarm_sim>/tools --out ../mjcf --config ../config

mobile_openarm_sim/tools/build_world.py 를 그대로 불러 월드를 만든 뒤 include 대상만 바꾼다.
→ 지도(world_map), 위치(locations.yaml), 작업대·상자 좌표가 바퀴형과 1:1 로 같다.
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--wheeled-tools", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--config", required=True)
a = ap.parse_args()
sys.path.insert(0, os.path.abspath(a.wheeled_tools))
import build_world  # noqa: E402

build_world.build(a.out, a.config)
for name, robot in (("world.xml", "tron1_openarm.xml"), ("world_light.xml", "tron1_openarm_light.xml")):
    p = os.path.join(a.out, name)
    s = open(p, encoding="utf-8").read()
    s = s.replace('file="mobile_openarm.xml"', f'file="{robot}"').replace('file="mobile_openarm_light.xml"', f'file="{robot}"')
    s = s.replace('model="openarm_mobile_world"', 'model="tron1_openarm_world"')
    open(p, "w", encoding="utf-8").write(s)
    print("world →", p, "(robot:", robot + ")")
