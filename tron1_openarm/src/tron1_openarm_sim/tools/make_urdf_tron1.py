#!/usr/bin/env python3
"""tron1_openarm.xml(MJCF) → urdf/tron1_openarm.urdf (ROS 용: robot_state_publisher·RViz·MoveIt).

바퀴형의 tools/mjcf_to_urdf.py 를 그대로 재사용하고 두 값만 바꾼다.
  * 메시 경로 package://tron1_openarm_sim/mjcf/
  * base_footprint → base_link 높이: 바퀴 반지름 0.08 → 서 있는 높이 0.68 (NOMINAL_BASE_HEIGHT)
왜 xacro 결과(URDF)를 바로 쓰지 않나: ROS 쪽 URDF 에는 OpenArm 팔·그리퍼·카메라·LiDAR 프레임까지 들어가야 하고,
시뮬레이터(MJCF)와 기구학이 1:1 로 같아야 하므로, 최종 MJCF 를 정본으로 두고 다시 뽑는다 (바퀴형과 같은 원칙).

    python3 make_urdf_tron1.py --wheeled-tools <mobile_openarm_sim>/tools --mjcf ../mjcf/tron1_openarm.xml --out ../urdf/tron1_openarm.urdf
"""
import argparse
import os
import re
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--wheeled-tools", required=True)
ap.add_argument("--mjcf", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--base-height", type=float, default=0.68, help="pf 0.68, sf 0.73")
ap.add_argument("--mesh-prefix", default="package://tron1_openarm_sim/mjcf/", help="sf: package://tron1_openarm_sim/mjcf/sf/")
a = ap.parse_args()
sys.path.insert(0, os.path.abspath(a.wheeled_tools))
import mjcf_to_urdf as M  # noqa: E402

M.PKG = a.mesh_prefix
M.WHEEL_RADIUS = a.base_height
M.convert(a.mjcf, a.out)
s = open(a.out, encoding="utf-8").read()
s = s.replace('<robot name="mobile_openarm">', '<robot name="tron1_openarm">')
s = s.replace("자동 생성: tools/mjcf_to_urdf.py (원본 MJCF: mjcf/mobile_openarm.xml)",
              "자동 생성: tron1_openarm_sim/tools/make_urdf_tron1.py (원본 MJCF: mjcf/tron1_openarm.xml, 하반신 xacro: LimX TRON1)")
open(a.out, "w", encoding="utf-8").write(s)
n_link = len(re.findall("<link ", s))
n_joint = len(re.findall('<joint name="[^"]+" type="(revolute|continuous|prismatic)"', s))
print(f"links {n_link}, 움직이는 관절 {n_joint}")
