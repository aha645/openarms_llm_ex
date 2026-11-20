#!/usr/bin/env python3
"""실습 월드(방·작업대·ArUco 상자·사람)를 만들고 로봇 include 를 TRON1+OpenArm 으로 둔다.

    python3 build_world_tron1.py --out ../mjcf --config ../config

tools/build_world.py 로 월드를 만든다.
→ 지도(world_map), 위치(locations.yaml), 작업대·상자 좌표는 build_world.py 가 정한다.
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--config", required=True)
a = ap.parse_args()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_world  # noqa: E402

build_world.build(a.out, a.config)      # world.xml → tron1_openarm.xml, world_light.xml → tron1_openarm_light.xml
for name in ("world.xml", "world_light.xml"):
    print("world →", os.path.join(a.out, name))
