#!/usr/bin/env python3
"""TRON1 용 SRDF 의 'home' 이름 자세를 바꾼다: 팔을 바깥으로 0.2 rad(11.5°) 벌린 자세. (+ 바퀴 passive 관절 제거)

    python3 patch_srdf_home.py ../../tron1_openarm_moveit_config/config/tron1_openarm.srdf

이유: OpenArm 의 home(모든 관절 0, 팔을 곧게 내림)에서는 손·아래팔이 TRON1 몸통 상자(폭 0.27 m) 옆면에
3.6 cm 파고든다. 어깨 roll(joint2)을 바깥쪽으로 0.2 rad 돌리면 4.5 cm 떨어진다 (02-model.md 4-5).
"""
import sys
import xml.etree.ElementTree as ET

HOME_J2 = 0.20
path = sys.argv[1]
tree = ET.parse(path)
for gs in tree.getroot().findall("group_state"):
    if gs.get("name") != "home":
        continue
    for j in gs.findall("joint"):
        if j.get("name") == "openarm_left_joint2":
            j.set("value", str(-HOME_J2))
        elif j.get("name") == "openarm_right_joint2":
            j.set("value", str(HOME_J2))
# 바퀴형 SRDF 생성기에서 넘어온 바퀴 passive 관절 제거 (TRON1 URDF 에 없는 관절 → move_group 경고)
root = tree.getroot()
for pj in list(root.findall("passive_joint")):
    if "wheel" in pj.get("name", ""):
        root.remove(pj)
ET.indent(tree, "  ")
tree.write(path, encoding="utf-8", xml_declaration=True)
print("home: openarm_left_joint2 = -%.2f, openarm_right_joint2 = +%.2f  →" % (HOME_J2, HOME_J2), path)
