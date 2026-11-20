#!/usr/bin/env python3
"""mobile_openarm.xml(MJCF) → mobile_openarm.urdf 변환기.

시뮬레이터(MJCF)와 ROS(URDF: TF/RViz/MoveIt/Nav2)가 *같은 기구학*을 쓰도록
MJCF 를 정본(single source of truth)으로 두고 URDF 를 자동 생성한다.
(교육과정 Day1-03 "URDF/Xacro 로봇 모델을 MJCF 와 연결" 실습용)

    python3 mjcf_to_urdf.py --mjcf ../mjcf/mobile_openarm.xml --out ../urdf/mobile_openarm.urdf

* 바디 프레임/관절 축/한계/관성: MuJoCo 컴파일 결과에서 읽음
* 메시 geom 의 pos/quat/scale: XML 원문에서 읽음 (MuJoCo 는 컴파일 시 메시를 무게중심으로 옮기므로)
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

PKG = "package://mobile_openarm_sim/mjcf/"
WHEEL_RADIUS = 0.08


def quat_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def mat_to_rpy(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return quat_to_rpy(q)


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def origin(parent, xyz, rpy):
    o = ET.SubElement(parent, "origin")
    o.set("xyz", fmt(xyz))
    o.set("rpy", fmt(rpy))


def parse_floats(s, default):
    return np.array([float(x) for x in s.split()]) if s else np.array(default, float)


def xml_geoms(mjcf_path):
    """geom name → (attrs, class) 와 mesh name → (file, scale) 를 XML 에서 수집."""
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    meshes = {}
    for mesh in root.iter("mesh"):
        meshes[mesh.get("name")] = (mesh.get("file"), mesh.get("scale", "1 1 1"))
    geoms = {}
    for g in root.iter("geom"):
        if g.get("name"):
            geoms[g.get("name")] = g
    meshdir = root.find("compiler").get("meshdir", "")
    return geoms, meshes, meshdir


def geom_quat(g):
    if g.get("quat"):
        return parse_floats(g.get("quat"), [1, 0, 0, 0])
    return np.array([1.0, 0, 0, 0])


def add_geometry(elem, m, gid, xg, meshes, meshdir):
    t = m.geom_type[gid]
    geo = ET.SubElement(elem, "geometry")
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        mname = xg.get("mesh")
        f, scale = meshes[mname]
        e = ET.SubElement(geo, "mesh")
        e.set("filename", PKG + os.path.join(meshdir, f))
        e.set("scale", scale)
    elif t == mujoco.mjtGeom.mjGEOM_BOX:
        e = ET.SubElement(geo, "box")
        e.set("size", fmt(2 * m.geom_size[gid]))
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        e = ET.SubElement(geo, "cylinder")
        e.set("radius", f"{m.geom_size[gid][0]:.6g}")
        e.set("length", f"{2 * m.geom_size[gid][1]:.6g}")
    elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
        e = ET.SubElement(geo, "sphere")
        e.set("radius", f"{m.geom_size[gid][0]:.6g}")
    else:
        return False
    return True


def convert(mjcf_path, out_path):
    m = mujoco.MjModel.from_xml_path(mjcf_path)
    xgeoms, meshes, meshdir = xml_geoms(mjcf_path)
    robot = ET.Element("robot", name="mobile_openarm")

    # 재질
    for name, rgba in (("black", "0.25 0.25 0.25 1"), ("silver", "0.8 0.8 0.8 1"),
                       ("white", "0.85 0.85 0.88 1"), ("pink", "0.95 0.35 0.55 1")):
        mat = ET.SubElement(robot, "material", name=name)
        ET.SubElement(mat, "color", rgba=rgba)

    ET.SubElement(robot, "link", name="base_footprint")
    base_id = m.body("base_link").id

    def link_name(bid):
        return m.body(bid).name

    def add_link(bid):
        link = ET.SubElement(robot, "link", name=link_name(bid))
        mass = float(m.body_mass[bid])
        if mass > 1e-6:
            inert = ET.SubElement(link, "inertial")
            origin(inert, m.body_ipos[bid], quat_to_rpy(m.body_iquat[bid]))
            ET.SubElement(inert, "mass", value=f"{mass:.6g}")
            I = m.body_inertia[bid]
            ET.SubElement(inert, "inertia", ixx=f"{I[0]:.6g}", iyy=f"{I[1]:.6g}", izz=f"{I[2]:.6g}",
                          ixy="0", ixz="0", iyz="0")
        for gid in range(m.ngeom):
            if m.geom_bodyid[gid] != bid:
                continue
            gname = m.geom(gid).name
            xg = xgeoms.get(gname)
            if xg is None:
                continue
            cls = xg.get("class", "")
            is_visual = cls == "visual" or gname.endswith("_vis") or (
                cls != "collision" and m.geom_contype[gid] == 0)
            is_collision = cls == "collision" or (cls == "" and m.geom_contype[gid] != 0)
            if m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH:
                pos = parse_floats(xg.get("pos"), [0, 0, 0])
                quat = geom_quat(xg)
            else:
                pos = m.geom_pos[gid]
                quat = m.geom_quat[gid]
            targets = []
            if is_visual or (cls == "" and is_collision):   # 베이스의 기본 도형은 visual+collision
                targets.append("visual")
            if is_collision:
                targets.append("collision")
            for tag in targets:
                e = ET.SubElement(link, tag)
                origin(e, pos, quat_to_rpy(quat))
                if not add_geometry(e, m, gid, xg, meshes, meshdir):
                    link.remove(e)
                    continue
                if tag == "visual":
                    rgba = m.geom_rgba[gid]
                    matid = m.geom_matid[gid]
                    if matid >= 0:
                        rgba = m.mat_rgba[matid]
                    me = ET.SubElement(e, "material", name=f"{gname}_mat")
                    ET.SubElement(me, "color", rgba=fmt(rgba))
        return link

    def add_joint(bid):
        parent = m.body_parentid[bid]
        jn = m.body_jntnum[bid]
        if jn == 0:
            jtype, jname = "fixed", f"{link_name(parent)}_to_{link_name(bid)}"
        else:
            jid = m.body_jntadr[bid]
            assert jn == 1, link_name(bid)
            assert np.allclose(m.jnt_pos[jid], 0), "jnt_pos != 0 은 지원하지 않음"
            jname = m.joint(jid).name
            t = m.jnt_type[jid]
            if t == mujoco.mjtJoint.mjJNT_HINGE:
                jtype = "revolute" if m.jnt_limited[jid] else "continuous"
            elif t == mujoco.mjtJoint.mjJNT_SLIDE:
                jtype = "prismatic"
            else:
                raise RuntimeError(t)
        j = ET.SubElement(robot, "joint", name=jname, type=jtype)
        ET.SubElement(j, "parent", link=link_name(parent))
        ET.SubElement(j, "child", link=link_name(bid))
        origin(j, m.body_pos[bid], quat_to_rpy(m.body_quat[bid]))
        if jtype != "fixed":
            ET.SubElement(j, "axis", xyz=fmt(m.jnt_axis[jid]))
            # 힘 한계: 액추에이터 forcerange
            effort, vel = 100.0, 10.0
            for a in range(m.nu):
                if m.actuator_trnid[a][0] == jid:
                    effort = float(max(abs(m.actuator_forcerange[a]))) or effort
            if jname.endswith("_wheel_joint"):
                vel = 30.0
            if "finger" in jname:
                vel = 0.2
            if jtype in ("revolute", "prismatic"):
                lo, hi = m.jnt_range[jid]
                ET.SubElement(j, "limit", lower=f"{lo:.6g}", upper=f"{hi:.6g}", effort=f"{effort:.6g}",
                              velocity=f"{vel:.6g}")
            else:
                ET.SubElement(j, "limit", effort=f"{effort:.6g}", velocity=f"{vel:.6g}")
            if jname.endswith("finger_joint2"):
                ET.SubElement(j, "mimic", joint=jname.replace("joint2", "joint1"), multiplier="1", offset="0")

    # base_footprint → base_link (바퀴축 높이)
    j = ET.SubElement(robot, "joint", name="base_footprint_to_base_link", type="fixed")
    ET.SubElement(j, "parent", link="base_footprint")
    ET.SubElement(j, "child", link="base_link")
    origin(j, [0, 0, WHEEL_RADIUS], [0, 0, 0])
    add_link(base_id)

    # base_link 하위 트리 전체 (깊이 우선)
    def children(bid):
        return [b for b in range(m.nbody) if m.body_parentid[b] == bid and b != bid]

    stack = children(base_id)
    while stack:
        b = stack.pop(0)
        add_link(b)
        add_joint(b)
        stack = children(b) + stack

    # 센서 프레임
    def fixed_frame(name, parent, xyz, rpy):
        ET.SubElement(robot, "link", name=name)
        jj = ET.SubElement(robot, "joint", name=f"{parent}_to_{name}", type="fixed")
        ET.SubElement(jj, "parent", link=parent)
        ET.SubElement(jj, "child", link=name)
        origin(jj, xyz, rpy)

    fixed_frame("lidar_link", "base_link", m.site_pos[m.site("lidar_site").id], [0, 0, 0])
    fixed_frame("imu_link", "base_link", m.site_pos[m.site("imu_site").id], [0, 0, 0])
    cid = m.camera("head_camera").id
    Rc = np.zeros(9)
    mujoco.mju_quat2Mat(Rc, m.cam_quat[cid])
    Rc = Rc.reshape(3, 3)       # 열: 카메라 x(오른쪽), y(위), z(뒤)
    xc, yc, zc = Rc[:, 0], Rc[:, 1], Rc[:, 2]
    cam_parent = m.body(m.cam_bodyid[cid]).name
    R_link = np.column_stack([-zc, -xc, yc])       # ROS 카메라 링크: x 전방, y 좌, z 위
    fixed_frame("head_camera_link", cam_parent, m.cam_pos[cid], mat_to_rpy(R_link))
    R_opt_in_link = R_link.T @ np.column_stack([xc, -yc, -zc])   # optical: x 오른쪽, y 아래, z 전방
    fixed_frame("head_camera_optical_frame", "head_camera_link", [0, 0, 0], mat_to_rpy(R_opt_in_link))

    ET.indent(robot, "  ")
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0"?>\n<!-- 자동 생성: tools/mjcf_to_urdf.py (원본 MJCF: mjcf/mobile_openarm.xml) -->\n')
        f.write(ET.tostring(robot, encoding="unicode"))
    print("wrote", out_path)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    convert(a.mjcf, a.out)
