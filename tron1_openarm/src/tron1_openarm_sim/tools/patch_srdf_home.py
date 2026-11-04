#!/usr/bin/env python3
"""TRON1 용 SRDF 의 이름 자세(home·ready·carry·hands_up)를 이족용 값(tron1_openarm_sim/arm_poses.py)으로 바꾼다.
(+ 바퀴형 SRDF 생성기에서 넘어온 바퀴 passive 관절 제거)

    python3 patch_srdf_home.py ../../tron1_openarm_moveit_config/config/tron1_openarm.srdf

이유
  * home: OpenArm 의 home(모든 관절 0, 팔을 곧게 내림)에서는 손·아래팔이 TRON1 몸통 상자 옆면에 파고든다
    → 어깨 roll(joint2)을 바깥쪽으로 0.2 rad 벌림 (02-model.md 4-5)
  * ready·carry·hands_up: 어깨를 몸통 윗면 바로 위로 낮춘 설계에서는 바퀴형 자세가 몸통 상자를 관통한다
    → 위팔을 앞으로 든 자세 (06-sway-analysis.md 6절)
"""
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from tron1_openarm_sim.arm_poses import POSES  # noqa: E402

path = sys.argv[1]
tree = ET.parse(path)
root = tree.getroot()
for gs in root.findall("group_state"):
    name, group = gs.get("name"), gs.get("group", "")
    if name not in POSES:
        continue
    side = "left" if "left" in group else "right" if "right" in group else None
    vals = {}
    if side:
        vals = {f"openarm_{side}_joint{i + 1}": v for i, v in enumerate(POSES[name][0 if side == "left" else 1])}
    else:   # 양팔 그룹
        for s, k in (("left", 0), ("right", 1)):
            vals.update({f"openarm_{s}_joint{i + 1}": v for i, v in enumerate(POSES[name][k])})
    for j in gs.findall("joint"):
        if j.get("name") in vals:
            j.set("value", f"{vals[j.get('name')]:g}")
for pj in list(root.findall("passive_joint")):
    if "wheel" in pj.get("name", ""):
        root.remove(pj)
ET.indent(tree, "  ")
tree.write(path, encoding="utf-8", xml_declaration=True)
print("이족용 이름 자세 적용:", ", ".join(POSES), "→", path)
