#!/usr/bin/env python3
"""OpenArm v1 양팔(MJCF) + 차동구동 바퀴 베이스 → mobile_openarm.xml 생성기.

원본: https://github.com/enactic/openarm_mujoco (v1/openarm_bimanual.xml, Apache-2.0)
사용법:
    python3 build_mjcf.py --src ~/openarm_mujoco/v1/openarm_bimanual.xml --out ../mjcf/mobile_openarm.xml

원본 상반신(몸통 기둥 + 7DOF 양팔 + 그리퍼)은 그대로 두고, 그 아래에
  - base_link (freejoint, 섀시 박스 25kg)
  - 좌/우 구동 바퀴 (hinge + velocity actuator)
  - 앞/뒤 캐스터 (마찰 거의 0인 구)
  - LiDAR site, IMU site, 머리 카메라
를 추가한다.  좌표계는 ROS REP-103 (x 전방, y 좌측, z 위).
"""
import argparse
import copy
import math
import xml.etree.ElementTree as ET

# ---- 베이스 치수 (URDF 생성기와 반드시 같은 값을 사용) ----
WHEEL_RADIUS = 0.08
WHEEL_SEPARATION = 0.44          # 좌우 바퀴 중심 사이 거리
WHEEL_WIDTH = 0.04
CHASSIS_SIZE = (0.50, 0.36, 0.25)  # x, y, z (전체 길이)
CHASSIS_BOTTOM = 0.05            # 지면에서 섀시 바닥까지
CASTER_RADIUS = 0.04
CASTER_X = 0.20
CASTER_LIFT = 0.004             # 캐스터를 4mm 띄워 하중이 구동바퀴에 실리도록 (미끄럼 방지)
TORSO_X = 0.12                   # 상반신을 섀시 앞쪽으로 12cm (팔 도달거리 확보)
TORSO_Z = CHASSIS_BOTTOM + CHASSIS_SIZE[2] - WHEEL_RADIUS   # base_link(바퀴축) → 몸통 원점 높이
LIDAR_POS = (0.22, 0.0, CHASSIS_BOTTOM + CHASSIS_SIZE[2] + 0.04 - WHEEL_RADIUS)
CAMERA_POS = (0.10, 0.0, 0.86)    # 몸통(openarm_body_link0) 기준
CAMERA_PITCH_DEG = 50.0           # 아래로 숙인 각도
CAMERA_FOVY = 70.0


def f(*v):
    return " ".join(f"{x:.6g}" for x in v)


def sub(parent, tag, **attrs):
    e = ET.SubElement(parent, tag)
    for k, v in attrs.items():
        e.set(k.rstrip("_"), str(v))
    return e


def make_light(root):
    """저사양 모드: 고해상도 시각 메시(OBJ 47개, 약 160만 면)를 빼고 충돌 메시(STL)를 보이게 한다.
    모델 컴파일 메모리 약 1.9GB → 수백 MB, GPU 렌더 부하도 크게 감소 (GTX 1050 2GB / RAM 8GB 대응)."""
    parents = {c: p for p in root.iter() for c in p}
    for g in list(root.iter("geom")):
        if g.get("class") == "visual":
            parents[g].remove(g)
    used = {g.get("mesh") for g in root.iter("geom") if g.get("mesh")}
    asset = root.find("asset")
    for mesh in list(asset.findall("mesh")):
        if mesh.get("name") not in used:
            asset.remove(mesh)
    for d in root.iter("default"):
        if d.get("class") == "collision":
            d.find("geom").set("group", "2")          # 렌더되는 그룹으로
            d.find("geom").set("material", "matte_black")
    for g in root.iter("geom"):
        if g.get("class") == "collision" and g.get("material") == "collision_material":
            g.attrib.pop("material")


def build(src, out, light=False):
    tree = ET.parse(src)
    root = tree.getroot()
    root.set("model", "mobile_openarm")
    if light:
        make_light(root)

    # 2ms 타임스텝: 2017년 노트북 CPU에서도 실시간에 가깝게 돌리기 위해 (원본 1ms)
    opt = root.find("option")
    opt.set("timestep", "0.002")

    world = root.find("worldbody")
    torso = world.find("body[@name='openarm_body_link0']")
    world.remove(torso)
    # 원본의 고정 카메라/사이트 제거 (월드 파일에서 새로 정의)
    for e in list(world):
        if e.tag in ("camera", "site"):
            world.remove(e)

    # ---------------- 모바일 베이스 ----------------
    base = sub(world, "body", name="base_link", pos=f(0, 0, WHEEL_RADIUS + 0.002))
    sub(base, "freejoint", name="base_free")
    cx, cy, cz = CHASSIS_SIZE
    chassis_center_z = CHASSIS_BOTTOM + cz / 2 - WHEEL_RADIUS
    sub(base, "geom", name="chassis", type="box", size=f(cx / 2, cy / 2, cz / 2),
        pos=f(0, 0, chassis_center_z), mass="25", rgba="0.85 0.85 0.88 1", group="2",
        contype="1", conaffinity="1")
    # 보기용 범퍼 (충돌 없음)
    sub(base, "geom", name="bumper_vis", type="box", size=f(cx / 2 + 0.01, cy / 2 + 0.01, 0.02),
        pos=f(0, 0, CHASSIS_BOTTOM + 0.03 - WHEEL_RADIUS), rgba="0.95 0.35 0.55 1",
        contype="0", conaffinity="0", group="2", mass="0")
    for name, sx in (("front", CASTER_X), ("rear", -CASTER_X)):
        sub(base, "geom", name=f"caster_{name}", type="sphere", size=f(CASTER_RADIUS),
            pos=f(sx, 0, CASTER_RADIUS - WHEEL_RADIUS + CASTER_LIFT), friction="0.001 0.0001 0.0001",
            condim="1", priority="2", mass="0.3", rgba="0.2 0.2 0.2 1", group="2")
    sub(base, "site", name="imu_site", pos=f(0, 0, chassis_center_z), size="0.01", group="4")
    sub(base, "site", name="lidar_site", pos=f(*LIDAR_POS), size="0.02", rgba="0.1 0.1 0.1 1", group="4")
    sub(base, "geom", name="lidar_vis", type="cylinder", size="0.035 0.025",
        pos=f(LIDAR_POS[0], 0, LIDAR_POS[2] - 0.01), rgba="0.1 0.1 0.1 1",
        contype="0", conaffinity="0", group="2", mass="0")

    for side, sy in (("left", WHEEL_SEPARATION / 2), ("right", -WHEEL_SEPARATION / 2)):
        w = sub(base, "body", name=f"{side}_wheel_link", pos=f(0, sy, 0))
        # 바퀴 축 = y축. hinge axis 0 1 0 → +회전 시 전진
        sub(w, "joint", name=f"{side}_wheel_joint", type="hinge", axis="0 1 0", damping="0.05",
            armature="0.01")
        sub(w, "geom", name=f"{side}_wheel", type="cylinder", size=f(WHEEL_RADIUS, WHEEL_WIDTH / 2),
            quat=f(math.cos(math.pi / 4), math.sin(math.pi / 4), 0, 0), mass="1.5",
            friction="1.5 0.01 0.001", rgba="0.15 0.15 0.15 1", group="2", condim="3")

    # ---------------- 상반신 (원본 OpenArm) ----------------
    torso.set("pos", f(TORSO_X, 0, TORSO_Z))
    base.append(torso)
    # 머리 카메라 (몸통 기준, 앞쪽 아래를 봄). MuJoCo 카메라는 -z 를 바라보고 +y 가 위.
    a = math.radians(CAMERA_PITCH_DEG)
    sub(torso, "camera", name="head_camera", pos=f(*CAMERA_POS), fovy=f(CAMERA_FOVY),
        xyaxes=f(0, -1, 0, math.sin(a), 0, math.cos(a)))
    sub(torso, "geom", name="head_camera_vis", type="box", size="0.02 0.05 0.02",
        pos=f(CAMERA_POS[0] - 0.02, 0, CAMERA_POS[2]), rgba="0.1 0.1 0.1 1",
        contype="0", conaffinity="0", group="2", mass="0")
    # 상반신 기둥과 머리 카메라를 잇는 보기용 막대
    sub(torso, "geom", name="head_mast_vis", type="cylinder", size="0.012 0.06",
        pos=f(CAMERA_POS[0] - 0.05, 0, CAMERA_POS[2] - 0.06), rgba="0.3 0.3 0.3 1",
        contype="0", conaffinity="0", group="2", mass="0")

    # ---------------- 액추에이터 / 센서 ----------------
    act = root.find("actuator")
    for side in ("left", "right"):
        sub(act, "velocity", name=f"{side}_wheel_vel", joint=f"{side}_wheel_joint", kv="12",
            forcelimited="true", forcerange="-25 25", ctrllimited="true", ctrlrange="-30 30")
    sens = root.find("sensor")
    if sens is None:
        sens = sub(root, "sensor")
    for e in list(sens):          # 원본의 world_site 센서 제거 (site 를 지웠으므로)
        sens.remove(e)
    sub(sens, "gyro", name="imu_gyro", site="imu_site")
    sub(sens, "accelerometer", name="imu_acc", site="imu_site")
    sub(sens, "framequat", name="imu_quat", objtype="site", objname="imu_site")

    ET.indent(tree, "  ")
    header = ("<!-- 자동 생성 파일: tools/build_mjcf.py 로 다시 만드세요.\n"
              "     상반신/팔/그리퍼 모델: enactic/openarm_mujoco v1 (Apache-2.0) -->\n")
    with open(out, "w", encoding="utf-8") as fp:
        fp.write(header)
        fp.write(ET.tostring(root, encoding="unicode"))
    print("wrote", out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--light", action="store_true", help="저사양 모드 (시각 메시 제거)")
    a = ap.parse_args()
    build(a.src, a.out, a.light)
