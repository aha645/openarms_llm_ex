#!/usr/bin/env python3
"""TRON1 Point-foot 하반신 + OpenArm v1 양팔 상반신 → MuJoCo 모델(MJCF) 생성기.

    python3 build_tron1_openarm.py \
        --tron1 ~/openarm_mobile_ws/tron1_openarm/third_party/tron1-robot-description/pointfoot/PF_TRON1A \
        --openarm ~/openarm_mobile_ws/third_party/openarm_mujoco/v1/openarm_bimanual.xml \
        --out ../mjcf

생성물 (mjcf/)
  tron1_openarm.xml        시뮬레이터용 (시각 메시 포함)
  tron1_openarm_light.xml  저사양용 (시각 메시 제거)
  tron1_openarm_train.xml  강화학습용 (상반신 충돌 끔, 바닥만 있는 장면 포함)
  tron1_openarm.urdf.generated  xacro 처리 결과 (확인용)

처리 순서 (왜 이렇게 했는지는 doc/tron1/02-model.md)
  1. xacro  : urdf/tron1_openarm.urdf.xacro → URDF.  TRON1 의 링크 길이·관절 한계·질량·관성은 LimX xacro(const.xacro)가 정본
  2. MuJoCo : URDF 를 MuJoCo 로 바로 컴파일 → mj_saveLastXML 로 MJCF 로 저장 (URDF 의 <mujoco> 확장 태그 사용)
  3. 보정   : URDF 에는 다리 충돌 형상이 구 2개뿐이라, LimX 가 MuJoCo 용으로 검증한 robot.xml 의 충돌 원통/상자를 옮겨 온다.
              관절 armature/damping 도 LimX robot.xml 값(0.01/0.01). 몸통 이름 base_Link → base_link(ROS REP-105, 기존 스택과 호환)
  4. 구동기 : 다리 6관절에 PD 위치 구동기 (kp 42, kd 3.5, 토크 ±80 Nm) — LimX 배포 코드(params.yaml)와 같은 값
  5. 상반신 : OpenArm 양팔 MJCF 의 몸통(openarm_body_link0)을 openarm_mount_link 아래에 접목.
              75 cm 기둥은 짧은 기둥으로 교체해 어깨 높이를 바퀴형 로봇과 같은 1.0 m 로 맞춘다.
  6. 센서   : IMU(TRON1 base 원점), LiDAR(몸통 앞쪽 아래, 0.57 m), 머리 카메라(바퀴형과 같은 위치)
"""
import argparse
import copy
import math
import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

# ------------------------------------------------------------------ 치수 (근거: doc/tron1/02-model.md)
NOMINAL_BASE_HEIGHT = 0.68      # 상반신을 얹고 서 있을 때 base_link 높이 [m] (LimX 학습 목표 base_height_target 0.68 과 같게, 시험 측정값 0.677)
SHOULDER_HEIGHT = 1.08          # 어깨(openarm_*_link0) 높이 [m]. 바퀴형 0.999 보다 8 cm 높게: 내린 손이 엉덩이 모터 위로 지나가도록 (02-model.md 4-1)
ARM_ORIGIN_Z = 0.697            # OpenArm: body_link0 원점 → 어깨 높이 (openarm_bimanual.xml 의 site arm_origin)
MOUNT_Z = 0.030                 # urdf/tron1_openarm.urdf.xacro 의 mount_z
MOUNT_X = 0.08                  # 상반신 앞뒤 위치 [m]. 0 이면 내린 손이 TRON1 엉덩이 모터(abad)에 닿음 → 0.08 에서 2.1 cm 틈 (doc/tron1/02-model.md 4-3)
COLUMN_TOP_ABOVE_SHOULDER = 0.065   # 원본 기둥 꼭대기(브래킷 윗면)는 어깨보다 6.5 cm 위
ORIG_COLUMN_LEN = 0.75          # 원본 OpenArm 기둥 길이 [m]
ORIG_BODY_MASS = 3.74           # 원본 openarm_body_link0 질량 [kg] (기둥 + 어깨 브래킷)
BRACKET_MASS = 0.9              # 그중 어깨 브래킷·배선 몫으로 남기는 질량 [kg] (가정)
LIDAR_POS = (0.17, 0.0, -0.13)  # base_link 기준. 상자 더미(높이 0.6 m)를 볼 수 있게 지면 0.57 m
CAMERA_POS = (0.10, 0.0, 0.86)  # openarm_body_link0 기준 (바퀴형과 동일 → 지면 기준 높이도 동일)
CAMERA_PITCH_DEG = 50.0
CAMERA_FOVY = 70.0
LEG_JOINTS = ["abad_L_joint", "hip_L_joint", "knee_L_joint", "abad_R_joint", "hip_R_joint", "knee_R_joint"]
KP, KD, TAU_MAX = 42.0, 3.5, 80.0


def f(*v):
    return " ".join(f"{x:.6g}" for x in v)


def sub(parent, tag, **attrs):
    e = ET.SubElement(parent, tag)
    for k, v in attrs.items():
        e.set(k.rstrip("_"), str(v))
    return e


# ------------------------------------------------------------------ 1. xacro → URDF
def run_xacro(xacro_file, tron1_dir, tmp, mount_x):
    """LimX xacro 는 $(find robot_description) 를 쓴다. ROS 패키지로 빌드하지 않아도 되도록
    임시 복사본에서 실제 경로로 바꾼 뒤 처리한다 (ROS 워크스페이스에 넣어 빌드했다면 그대로 써도 된다)."""
    import xacro
    desc_root = os.path.dirname(os.path.dirname(os.path.abspath(tron1_dir)))   # .../tron1-robot-description
    tdir = os.path.join(tmp, "PF")
    shutil.copytree(tron1_dir, tdir)
    for root, _, files in os.walk(os.path.join(tdir, "xacro")):
        for fn in files:
            p = os.path.join(root, fn)
            s = open(p, encoding="utf-8").read()
            s = s.replace("$(find robot_description)/pointfoot/PF_TRON1A", tdir)
            s = s.replace("$(find robot_description)", desc_root)
            open(p, "w", encoding="utf-8").write(s)
    doc = xacro.process_file(xacro_file, mappings={"tron1_dir": tdir, "mount_x": str(mount_x)})
    return doc.toprettyxml(indent="  "), tdir


# ------------------------------------------------------------------ 2. URDF → MJCF
def urdf_to_mjcf(urdf_xml, mesh_dir, tmp):
    root = ET.fromstring(urdf_xml)
    # contact_*_Link: 발끝 접촉점 표시용 링크(질량 1e-6 kg, 관성 0) → MuJoCo 가 '관성 고유값이 양수가 아님' 으로 거부.
    # 물리적 의미가 없으므로 MuJoCo 모델에서는 뺀다 (발 충돌구 foot_*_Link 가 실제 접촉을 담당).
    for e in list(root):
        if (e.tag == "link" and e.get("name", "").startswith("contact_")) or (
                e.tag == "joint" and e.find("child") is not None
                and e.find("child").get("link", "").startswith("contact_")):
            root.remove(e)
    for mesh in root.iter("mesh"):
        mesh.set("filename", os.path.basename(mesh.get("filename")))
    mj = ET.SubElement(root, "mujoco")
    sub(mj, "compiler", meshdir=mesh_dir, balanceinertia="true", discardvisual="false",
        fusestatic="false", strippath="true")
    urdf_path = os.path.join(tmp, "tron1.urdf")
    ET.ElementTree(root).write(urdf_path)
    m = mujoco.MjModel.from_xml_path(urdf_path)
    out = os.path.join(tmp, "tron1_from_urdf.xml")
    mujoco.mj_saveLastXML(out, m)
    return ET.parse(out).getroot(), m


# ------------------------------------------------------------------ 3. LimX MuJoCo 충돌 형상 이식
LIMX_TO_OURS = {"base_Link": "base_link"}


def limx_collisions(limx_xml):
    """LimX robot.xml 의 *_collision geom 을 (몸체 이름 → geom 목록) 으로."""
    r = ET.parse(limx_xml).getroot()
    out = {}
    for body in r.iter("body"):
        for g in body.findall("geom"):
            n = g.get("name", "")
            if n.endswith("_collision") and not n.startswith("foot_"):
                out.setdefault(body.get("name"), []).append(copy.deepcopy(g))
    return out


def build(tron1_dir, openarm_xml, out_dir, xacro_file, mount_x=None, shoulder=None):
    mount_x = MOUNT_X if mount_x is None else mount_x
    SH = SHOULDER_HEIGHT if shoulder is None else shoulder
    os.makedirs(out_dir, exist_ok=True)
    mesh_root = os.path.join(out_dir, "meshes")
    tron_mesh = os.path.join(mesh_root, "tron1")
    os.makedirs(tron_mesh, exist_ok=True)
    for fn in os.listdir(os.path.join(tron1_dir, "meshes")):
        shutil.copy(os.path.join(tron1_dir, "meshes", fn), tron_mesh)
    # OpenArm 메시: openarm_mujoco/v1/meshes → mjcf/meshes (visual/, collision/ …)
    oa_mesh = os.path.join(os.path.dirname(openarm_xml), "meshes")
    for name in os.listdir(oa_mesh):
        src = os.path.join(oa_mesh, name)
        dst = os.path.join(mesh_root, name)
        if os.path.isdir(src) and not os.path.exists(dst):
            shutil.copytree(src, dst)
        elif os.path.isfile(src):
            shutil.copy(src, dst)

    with tempfile.TemporaryDirectory() as tmp:
        urdf_xml, _ = run_xacro(xacro_file, tron1_dir, tmp, mount_x)
        with open(os.path.join(out_dir, "tron1_openarm.urdf.generated"), "w", encoding="utf-8") as fp:
            fp.write(urdf_xml)
        troot, tm = urdf_to_mjcf(urdf_xml, tron_mesh, tmp)
    tron_mass = float(tm.body_subtreemass[0])
    print(f"[1-2] xacro → URDF → MJCF: TRON1 질량 {tron_mass:.3f} kg, 몸체 {tm.nbody - 1}개, 관절 {tm.njnt}개")

    # ---------------- OpenArm 모델을 바탕으로 시작 (기본값·재질·메시·팔 구동기 유지)
    tree = ET.parse(openarm_xml)
    root = tree.getroot()
    root.set("model", "tron1_openarm")
    root.find("compiler").set("meshdir", "meshes")
    opt = root.find("option")
    opt.set("timestep", "0.002")             # 2 ms (정책 50 Hz = 10 스텝)
    world = root.find("worldbody")
    torso = world.find("body[@name='openarm_body_link0']")
    world.remove(torso)
    for e in list(world):
        if e.tag in ("camera", "site", "light", "geom"):
            world.remove(e)

    # TRON1 메시 asset
    asset = root.find("asset")
    for mesh in troot.find("asset").findall("mesh"):
        mesh.set("file", "tron1/" + os.path.basename(mesh.get("file")))
        asset.append(mesh)

    # TRON1 몸체 트리 (URDF 에서 온 것)
    tbase = troot.find("worldbody").find("body[@name='base_Link']")
    if tbase is None:   # URDF 루트 링크 geom 은 worldbody 에 바로 들어갈 수 있음 → 감싸서 몸체로
        raise RuntimeError("base_Link body 를 찾지 못함")
    lim = limx_collisions(os.path.join(tron1_dir, "xml", "robot.xml"))
    for body in tbase.iter("body"):
        name = body.get("name")
        # URDF 에서 온 geom: 시각 메시는 class=visual 로, 충돌 구는 발만 남김
        for g in list(body.findall("geom")):
            if g.get("type") == "mesh":
                g.set("contype", "0")
                g.set("conaffinity", "0")
                g.set("group", "2")
                g.set("rgba", "0.79 0.82 0.93 1" if "hip" not in name else "0.95 0.95 0.95 1")
                g.attrib.pop("class", None)
            else:
                if name.startswith("foot_"):
                    g.set("name", name.replace("_Link", "_collision"))
                    g.set("friction", "0.8 0.005 0.0001")
                    g.set("condim", "3")
                    g.set("group", "3")
                    g.set("rgba", "0.2 0.2 0.2 1")
                else:
                    body.remove(g)
        for g in lim.get(name, []):
            g.set("group", "3")
            g.set("contype", "1")
            g.set("conaffinity", "1")
            g.set("rgba", "0 0 1 0")
            body.insert(0, g)
        for j in body.findall("joint"):
            j.set("armature", "0.01")
            j.set("damping", "0.01")
    tbase.set("name", "base_link")
    tbase.set("pos", f(0, 0, NOMINAL_BASE_HEIGHT + 0.03))
    tbase.attrib.pop("quat", None)
    fj = ET.Element("freejoint", name="base_free")
    tbase.insert(0, fj)
    world.append(tbase)
    world.insert(0, ET.Element("light", directional="true", pos="0 0 3", dir="0 0 -1", diffuse="0.6 0.6 0.6"))

    # ---------------- 상반신 접목
    mount = next(b for b in tbase.iter("body") if b.get("name") == "openarm_mount_link")
    body_z = SH - ARM_ORIGIN_Z - (NOMINAL_BASE_HEIGHT + MOUNT_Z)   # mount 기준 body_link0 원점 높이 (음수: 몸통 안쪽)
    torso.set("pos", f(0, 0, body_z))
    # 원본 75 cm 기둥(메시 6개 + 충돌 메시) → 짧은 기둥(상자)
    for g in list(torso.findall("geom")):
        torso.remove(g)
    col_bottom = -body_z + 0.007                       # mount 판 윗면 (body_link0 기준)
    col_top = ARM_ORIGIN_Z + COLUMN_TOP_ABOVE_SHOULDER
    col_len = col_top - col_bottom
    col_mass = (ORIG_BODY_MASS - BRACKET_MASS) * col_len / ORIG_COLUMN_LEN + BRACKET_MASS
    com_z = (col_bottom + col_top) / 2 * (col_mass - BRACKET_MASS) / col_mass + (ARM_ORIGIN_Z + 0.03) * BRACKET_MASS / col_mass
    inertial = torso.find("inertial")
    inertial.set("pos", f(-0.005, 0, com_z))
    inertial.set("mass", f"{col_mass:.4f}")
    inertial.set("diaginertia", f(col_mass * col_len ** 2 / 12 + 0.01, col_mass * col_len ** 2 / 12 + 0.01, 0.006))
    sub(torso, "geom", name="openarm_column_collision", type="box", size=f(0.03, 0.03, col_len / 2),
        pos=f(0, 0, (col_bottom + col_top) / 2), group="3", rgba="0 0 0 0", contype="1", conaffinity="1")
    sub(torso, "geom", name="openarm_column_vis", type="box", size=f(0.03, 0.03, col_len / 2),
        pos=f(0, 0, (col_bottom + col_top) / 2), material="matte_black", contype="0", conaffinity="0",
        group="2", mass="0")
    sub(torso, "geom", name="openarm_shoulder_bracket_vis", type="box", size=f(0.05, 0.11, 0.035),
        pos=f(0, 0, ARM_ORIGIN_Z + 0.03), material="metal_silver", contype="0", conaffinity="0",
        group="2", mass="0")
    a = math.radians(CAMERA_PITCH_DEG)
    sub(torso, "camera", name="head_camera", pos=f(*CAMERA_POS), fovy=f(CAMERA_FOVY),
        xyaxes=f(0, -1, 0, math.sin(a), 0, math.cos(a)))
    sub(torso, "geom", name="head_camera_vis", type="box", size="0.02 0.05 0.02",
        pos=f(CAMERA_POS[0] - 0.02, 0, CAMERA_POS[2]), rgba="0.1 0.1 0.1 1",
        contype="0", conaffinity="0", group="2", mass="0")
    sub(torso, "geom", name="head_mast_vis", type="cylinder", size="0.012 0.06",
        pos=f(CAMERA_POS[0] - 0.05, 0, CAMERA_POS[2] - 0.06), rgba="0.3 0.3 0.3 1",
        contype="0", conaffinity="0", group="2", mass="0")
    mount.append(torso)

    # ---------------- 센서
    sub(tbase, "site", name="imu_site", pos="0 0 0", size="0.01", group="4")
    sub(tbase, "site", name="lidar_site", pos=f(*LIDAR_POS), size="0.02", rgba="0.1 0.1 0.1 1", group="4")
    sub(tbase, "geom", name="lidar_vis", type="cylinder", size="0.035 0.025",
        pos=f(LIDAR_POS[0], 0, LIDAR_POS[2] - 0.01), rgba="0.1 0.1 0.1 1",
        contype="0", conaffinity="0", group="2", mass="0")
    sub(tbase, "geom", name="lidar_bracket_vis", type="box", size="0.03 0.03 0.05",
        pos=f(LIDAR_POS[0] - 0.04, 0, LIDAR_POS[2] + 0.04), rgba="0.3 0.3 0.3 1",
        contype="0", conaffinity="0", group="2", mass="0")

    # 상반신 기둥·판 ↔ TRON1 몸통 은 서로 겹쳐 있으므로 충돌 제외
    contact = root.find("contact")
    if contact is None:
        contact = sub(root, "contact")
    for b1, b2 in (("base_link", "openarm_mount_link"), ("base_link", "openarm_body_link0"),
                   ("openarm_mount_link", "openarm_body_link0"), ("base_link", "openarm_left_link0"),
                   ("base_link", "openarm_right_link0")):
        sub(contact, "exclude", body1=b1, body2=b2)

    # ---------------- 다리 구동기 (PD 위치 제어, 토크 제한) + 센서
    act = root.find("actuator")
    for j in LEG_JOINTS:
        sub(act, "position", name=f"{j}_ctrl", joint=j, kp=f"{KP:g}", kv=f"{KD:g}",
            forcelimited="true", forcerange=f(-TAU_MAX, TAU_MAX), ctrllimited="false")
    sens = root.find("sensor")
    if sens is None:
        sens = sub(root, "sensor")
    for e in list(sens):
        sens.remove(e)
    sub(sens, "gyro", name="imu_gyro", site="imu_site")
    sub(sens, "accelerometer", name="imu_acc", site="imu_site")
    sub(sens, "framequat", name="imu_quat", objtype="site", objname="imu_site")
    sub(sens, "velocimeter", name="base_linvel", site="imu_site")          # 학습 보상·인코더 정답용 (몸통 좌표 선속도)
    for side in ("L", "R"):
        fb = next(b for b in tbase.iter("body") if b.get("name") == f"foot_{side}_Link")
        sub(fb, "site", name=f"foot_{side}_site", pos="0 0 0", size="0.045", group="4")
        sub(sens, "touch", name=f"foot_{side}_touch", site=f"foot_{side}_site")
        sub(sens, "framepos", name=f"foot_{side}_pos", objtype="site", objname=f"foot_{side}_site")
        sub(sens, "framelinvel", name=f"foot_{side}_vel", objtype="site", objname=f"foot_{side}_site")
    for j in LEG_JOINTS:
        sub(sens, "actuatorfrc", name=f"{j}_tau", actuator=f"{j}_ctrl")

    ET.indent(tree, "  ")
    header = ("<!-- 자동 생성 파일: tron1_openarm/src/tron1_openarm_sim/tools/build_tron1_openarm.py\n"
              "     하반신: LimX TRON1 PF_TRON1A (tron1-robot-description, Apache-2.0) xacro → URDF → MJCF\n"
              "     상반신: enactic/openarm_mujoco v1 (Apache-2.0), 기둥만 짧게 교체 -->\n")
    full = os.path.join(out_dir, "tron1_openarm.xml")
    with open(full, "w", encoding="utf-8") as fp:
        fp.write(header + ET.tostring(root, encoding="unicode"))

    # 저사양: 시각 메시 제거, 충돌 형상을 보이게
    light = copy.deepcopy(root)
    parents = {c: p for p in light.iter() for c in p}
    for g in list(light.iter("geom")):
        if g.get("class") == "visual" or (g.get("type") == "mesh" and g.get("contype") == "0"):
            parents[g].remove(g)
    used = {g.get("mesh") for g in light.iter("geom") if g.get("mesh")}
    for mesh in list(light.find("asset").findall("mesh")):
        if mesh.get("name") not in used:
            light.find("asset").remove(mesh)
    for d in light.iter("default"):
        if d.get("class") == "collision":
            d.find("geom").set("group", "2")
            d.find("geom").set("material", "matte_black")
    for g in light.iter("geom"):
        if g.get("group") == "3" and g.get("rgba") in ("0 0 1 0", "0 0 0 0"):
            g.set("group", "2")
            g.set("rgba", "0.75 0.78 0.9 1")
    with open(os.path.join(out_dir, "tron1_openarm_light.xml"), "w", encoding="utf-8") as fp:
        fp.write(header + ET.tostring(light, encoding="unicode"))

    # 학습용: 저사양 + 상반신 충돌 끔 + 바닥 (계산 가볍게; 팔 무게·관성은 그대로)
    train = copy.deepcopy(light)
    tb = train.find("worldbody").find("body[@name='base_link']")
    oa = next(b for b in tb.iter("body") if b.get("name") == "openarm_body_link0")
    for g in oa.iter("geom"):
        g.set("contype", "0")
        g.set("conaffinity", "0")
    for d in train.iter("default"):
        if d.get("class") == "collision":
            d.find("geom").set("contype", "0")
            d.find("geom").set("conaffinity", "0")
    tw = train.find("worldbody")
    sub(tw, "geom", name="floor", type="plane", size="0 0 0.05", rgba="0.6 0.6 0.6 1",
        friction="0.8 0.005 0.0001")
    train.find("option").set("timestep", "0.004")   # 학습은 4 ms (정책 50 Hz = 5 스텝) — 속도 2배, 배포는 2 ms 로 검증
    with open(os.path.join(out_dir, "tron1_openarm_train.xml"), "w", encoding="utf-8") as fp:
        fp.write(header + ET.tostring(train, encoding="unicode"))

    # ---------------- 요약
    m = mujoco.MjModel.from_xml_path(full)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    up = m.body("openarm_body_link0").id
    print(f"[5] 상반신 접목: 기둥 {col_len:.3f} m (원본 {ORIG_COLUMN_LEN} m), 기둥+브래킷 {col_mass:.2f} kg (원본 {ORIG_BODY_MASS} kg)")
    print(f"    전체 질량 {m.body_subtreemass[0]:.2f} kg = TRON1 {tron_mass - 0.6:.2f} + 어댑터 판 0.6 + 상반신 {m.body_subtreemass[up]:.2f}")
    sh = d.xpos[m.body("openarm_left_link0").id][2]
    cam = d.cam_xpos[m.camera("head_camera").id][2]
    print(f"    어깨 높이 {sh:.3f} m (목표 {SH}), 머리 카메라 {cam:.3f} m, LiDAR {d.site_xpos[m.site('lidar_site').id][2]:.3f} m "
          f"(base 높이 {d.xpos[m.body('base_link').id][2]:.3f} m 기준)")
    print("wrote", full)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--tron1", required=True, help=".../tron1-robot-description/pointfoot/PF_TRON1A")
    ap.add_argument("--openarm", required=True, help=".../openarm_mujoco/v1/openarm_bimanual.xml")
    ap.add_argument("--out", default=os.path.join(here, "..", "mjcf"))
    ap.add_argument("--xacro", default=os.path.join(here, "..", "urdf", "tron1_openarm.urdf.xacro"))
    ap.add_argument("--mount-x", type=float, default=None, help="상반신 앞뒤 위치 [m] (기본 MOUNT_X)")
    ap.add_argument("--shoulder", type=float, default=None, help="서 있을 때 어깨 높이 [m] (기본 SHOULDER_HEIGHT)")
    a = ap.parse_args()
    build(os.path.abspath(a.tron1), os.path.abspath(a.openarm), os.path.abspath(a.out), os.path.abspath(a.xacro), a.mount_x, a.shoulder)
