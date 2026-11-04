#!/usr/bin/env python3
"""SRDF 자동 생성 (MoveIt Setup Assistant 의 'Self-Collisions' 단계를 MuJoCo 로 대신 수행).

  * 그룹: left_arm / right_arm (체인 openarm_*_link0 → openarm_*_hand_tcp), both_arms, 그리퍼
  * 자기충돌 비활성 쌍: 인접 링크 + MJCF <exclude> + N 회 랜덤 자세에서 한 번도 접촉하지 않은 쌍
      python3 make_srdf.py --mjcf ../../mobile_openarm_sim/mjcf/mobile_openarm.xml --out ../config/mobile_openarm.srdf
"""
import argparse
import itertools
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ARM = {s: [f"openarm_{s}_joint{i}" for i in range(1, 8)] for s in ("left", "right")}


def main(mjcf, out, samples):
    m = mujoco.MjModel.from_xml_path(mjcf)
    d = mujoco.MjData(m)
    root = m.body("base_link").id
    bodies = [b for b in range(m.nbody) if b == root or m.body_rootid[b] == root]
    # 충돌 geom 이 있는 바디만
    coll_bodies = sorted({m.geom_bodyid[g] for g in range(m.ngeom)
                          if m.geom_bodyid[g] in bodies and m.geom_contype[g] != 0})
    names = {b: m.body(b).name for b in bodies}

    disabled = {}

    def dis(a, b, reason):
        k = tuple(sorted((names[a], names[b])))
        disabled.setdefault(k, reason)

    for b in bodies:
        p = m.body_parentid[b]
        if p in names:
            dis(b, p, "Adjacent")
    for i in range(m.nexclude):
        sig = m.exclude_signature[i]
        b1, b2 = sig >> 16, sig & 0xFFFF
        if b1 in names and b2 in names:
            dis(b1, b2, "Adjacent")

    # 모든 로봇 충돌 geom 끼리 접촉 판정을 강제로 켜고(부모-자식/exclude 무시) 샘플링
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
    qadr = {s: [m.jnt_qposadr[m.joint(n).id] for n in ARM[s]] for s in ARM}
    jid = {s: [m.joint(n).id for n in ARM[s]] for s in ARM}
    rng = np.random.default_rng(0)
    ever = set()
    always = None
    for k in range(samples):
        for s in ARM:
            lo, hi = m.jnt_range[jid[s], 0], m.jnt_range[jid[s], 1]
            d.qpos[qadr[s]] = rng.uniform(lo, hi) if k > 0 else 0.0
        mujoco.mj_forward(m, d)
        now = set()
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if b1 in names and b2 in names and b1 != b2:
                now.add(tuple(sorted((names[b1], names[b2]))))
        ever |= now
        always = now if always is None else (always & now)
    for a, b in itertools.combinations(coll_bodies, 2):
        k = tuple(sorted((names[a], names[b])))
        if k not in ever:
            disabled.setdefault(k, "Never")
    for k in always or ():
        disabled.setdefault(k, "Default")

    r = ET.Element("robot", name="mobile_openarm")
    for s in ("left", "right"):
        g = ET.SubElement(r, "group", name=f"{s}_arm")
        ET.SubElement(g, "chain", base_link=f"openarm_{s}_link0", tip_link=f"openarm_{s}_hand_tcp")
    g = ET.SubElement(r, "group", name="both_arms")
    ET.SubElement(g, "group", name="left_arm")
    ET.SubElement(g, "group", name="right_arm")
    for s in ("left", "right"):
        g = ET.SubElement(r, "group", name=f"{s}_gripper")
        ET.SubElement(g, "link", name=f"openarm_{s}_hand")
        ET.SubElement(g, "link", name=f"openarm_{s}_left_finger")
        ET.SubElement(g, "link", name=f"openarm_{s}_right_finger")
        ET.SubElement(g, "joint", name=f"openarm_{s}_finger_joint1")
    # 오른팔 기준 자세. 왼팔은 joint1/joint3 부호를 뒤집어 좌우 대칭 (실측: MuJoCo FK 로 충돌 없음 확인)
    poses = {"home": [0, 0, 0, 0, 0, 0, 0],
             "ready": [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0],
             "carry": [0.3, 0.0, -1.2, 2.0, 0.0, 0.0, 0.0],   # 손이 footprint 안(x≈0.26m)으로 들어오는 운반 자세
             "hands_up": [0, 0, 0, 2.0, 0, 0, 0]}

    def side_q(side, q):
        if side == "right":
            return list(q)
        return [-q[0], q[1], -q[2]] + list(q[3:])

    for s in ("left", "right"):
        for pname, q in poses.items():
            gs = ET.SubElement(r, "group_state", name=pname, group=f"{s}_arm")
            for i, v in enumerate(side_q(s, q)):
                ET.SubElement(gs, "joint", name=f"openarm_{s}_joint{i+1}", value=f"{v:.4f}")
    for pname, q in poses.items():
        if pname == "carry":       # 양팔 동시 carry 는 손끼리 충돌
            continue
        gs = ET.SubElement(r, "group_state", name=pname, group="both_arms")
        for s in ("left", "right"):
            for i, v in enumerate(side_q(s, q)):
                ET.SubElement(gs, "joint", name=f"openarm_{s}_joint{i+1}", value=f"{v:.4f}")
    for s in ("left", "right"):
        for pname, v in (("open", 0.044), ("closed", 0.0)):
            gs = ET.SubElement(r, "group_state", name=pname, group=f"{s}_gripper")
            ET.SubElement(gs, "joint", name=f"openarm_{s}_finger_joint1", value=str(v))
        ET.SubElement(r, "end_effector", name=f"{s}_hand", parent_link=f"openarm_{s}_hand_tcp",
                      group=f"{s}_gripper", parent_group=f"{s}_arm")
    for wj in ("left_wheel_joint", "right_wheel_joint"):
        ET.SubElement(r, "passive_joint", name=wj)
    for (a, b), reason in sorted(disabled.items()):
        ET.SubElement(r, "disable_collisions", link1=a, link2=b, reason=reason)
    ET.indent(r, "  ")
    with open(out, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n<!-- 자동 생성: tools/make_srdf.py -->\n')
        f.write(ET.tostring(r, encoding="unicode"))
    print(f"wrote {out}: {len(disabled)} disabled pairs (samples={samples}), ever-colliding={len(ever)}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--samples", type=int, default=3000)
    a = ap.parse_args()
    main(a.mjcf, a.out, a.samples)
