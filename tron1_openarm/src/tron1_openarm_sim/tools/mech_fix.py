#!/usr/bin/env python3
"""상반신 접목부·손목 카메라·TRON1 손잡이를 실제 기구처럼 고친다 (MJCF·URDF 를 그 자리에서 수정, 여러 번 돌려도 같음).

    python3 tools/mech_fix.py mjcf/sf/tron1_openarm.xml mjcf/sf/tron1_openarm_light.xml mjcf/tron1_openarm.xml \
        mjcf/tron1_openarm_light.xml urdf/tron1_openarm_sf.urdf urdf/tron1_openarm.urdf
build_tron1_openarm.py 가 모델을 새로 만들 때도 자동으로 적용한다. 학습용 모델(*_train.xml)은 건너뛴다
(상반신 충돌을 끈 모델이고, 시각 형상만 바뀌므로 학습 결과에 영향 없음).

고치는 것 (VIEW=1 집기 시험을 눈으로 보고 나온 문제)
  1) 어깨 브래킷이 어깨 모터와 겹침: 브래킷 상자 폭이 ±0.11 m 였는데 양 어깨 link0 안쪽 면은 ±0.03 m
     → 브래킷을 두 link0 사이(±0.03 m)에 넣고, 높이를 link0 과 같게(어깨 축 ±0.065 m) 해 어깨 모터가 브래킷 옆면에 붙게 한다.
  2) 상반신이 몸통 위에 떠 보임: 기둥 아래끝(mount 판 윗면, base+0.037)과 TRON1 몸통 메시 윗면(base+0.017) 사이 2 cm 가 비어 있었다
     → 어댑터 판(질량 0.6 kg 은 openarm_mount_link 에 이미 있음)을 몸통 윗면 전체에 그린다.
  3) 손목 카메라가 손 뒤 4.5 cm 에 떠 있음: 손등(손 좌표 x −0.028)과 카메라 몸체(x −0.073) 사이에 브래킷을 그린다.
  5) RViz 에 하체(몸통·다리)가 안 보임: URDF 의 TRON1 링크 visual 이 충돌 도형(상자·원기둥)이고 색이 rgba "0 0 1 0"(투명)
     이었다 → MJCF 와 같은 STL 메시(링크 원점, 같은 색)를 visual 로 넣는다 (URDF 만).
  4) 팔꿈치가 TRON1 몸통 옆 손잡이를 관통: 손잡이가 시각 메시에만 있고 충돌 상자(y ±0.13)에는 없어서
     MoveIt·물리 모두 모르고 지나갔다 (carry→home 경유 경로에서 팔꿈치·아래팔 틈 0 cm)
     → 손잡이 충돌 상자(base 기준 x −0.067~0.067, y ±0.147~0.173, z −0.12~−0.08)를 넣는다.
        MJCF 에서는 다리(hip·knee)와는 충돌 제외 (SRDF 와 같음: base_link–hip/knee "Never").
  6) 어깨 브래킷이 흰(은색) 상자로 보임 — 바퀴형은 OpenArm 원본의 검은 어깨 브래킷 메시(body_link0_5, 몸통 기준 z 0.613~0.773)
     → 같은 메시를 그대로 쓴다 (몸통 좌표계가 같아 위치 그대로: 어깨 = body_link0 기준 z 0.697). 원본 75 cm 기둥은 TRON1 에서
     짧은 기둥(상자, mount 판 위 0.634~0.762)인데 이 브래킷 안에 들어가 보이지 않는다. 저사양 모델(메시 없음)은 상자를 검게만.
     URDF 도 같은 메시 (RViz). 시각 전용(질량 0, 충돌 없음) — 충돌은 기둥 상자 그대로.
  모두 질량 0 (시각) 또는 충돌 전용이라 전체 질량·관성은 그대로.
"""
import os
import re
import sys

# base_link 기준 손잡이 (시각 메시에서 잰 값: z base−0.12~−0.08 구간 y 최대 0.172, x −0.066~0.067)
HANDLE_POS = (0.0, 0.160, -0.100)
HANDLE_HALF = (0.067, 0.013, 0.020)
# openarm_mount_link 기준 어댑터 판: TRON1 몸통 윗면(base 기준 x −0.093~0.106, y ±0.093, 높이 base+0.017)을 덮고
# 상반신 기둥(mount x ±0.03) 앞쪽까지. 윗면 = mount 판 윗면(mount +0.007)
PLATE_POS = (-0.0615, 0.0, -0.003)
PLATE_HALF = (0.1115, 0.093, 0.010)
# openarm_body_link0 기준 어깨 브래킷: 양 link0 안쪽 면(y ±0.03) 사이, 어깨 축(z 0.697) 중심
BRACKET_POS = (0.0, 0.0, 0.697)
BRACKET_HALF = (0.05, 0.03, 0.065)
BRACKET_MESH = "body_link0_5.obj"      # OpenArm 원본 어깨 브래킷 (검정) — 6)
MATTE_BLACK = "0.247059 0.247059 0.247059 1"
# 손 좌표 손목 카메라 브래킷: 손등(x −0.028) → 카메라 몸체 앞면(x −0.073), 손바닥판 두께 안(z −0.011~0.001)
WRIST_BRACKET_POS = (-0.0505, 0.0, -0.005)
WRIST_BRACKET_HALF = (0.0225, 0.015, 0.006)
WRIST_CAM_POS = (-0.085, 0.0, -0.01)
WRIST_CAM_HALF = (0.012, 0.02, 0.01)


def _f(*v):
    return " ".join(f"{x:g}" for x in v)


def _indent_after(s, pos):
    return re.search(r"\n([ \t]*)[^\n]*$", s[:pos]).group(1) + "  "


def fix_mjcf(s):
    notes = []
    # 1) 어깨 브래킷
    pat = r'(<geom name="openarm_shoulder_bracket_vis"[^>]*?)size="[^"]*"([^>]*?)pos="[^"]*"'
    new = re.sub(pat, rf'\1size="{_f(*BRACKET_HALF)}"\2pos="{_f(*BRACKET_POS)}"', s)
    if new != s:
        notes.append("어깨 브래킷 → link0 사이")
        s = new
    # 6) 어깨 브래킷 → OpenArm 원본 검은 브래킷 메시 (메시 없는 저사양 모델은 상자를 검게)
    m = re.search(r'<geom name="openarm_shoulder_bracket_vis" type="box"[^>]*/>', s)
    if m is not None:
        if f'<mesh name="{BRACKET_MESH}"' in s:
            g = (f'<geom name="openarm_shoulder_bracket_vis" type="mesh" mesh="{BRACKET_MESH}" material="matte_black" '
                 f'contype="0" conaffinity="0" group="2" mass="0" />')
            notes.append("어깨 브래킷 → 원본 검은 메시")
        else:
            g = m.group(0).replace('material="metal_silver"', 'material="matte_black"')
            notes.append("어깨 브래킷 상자 → 검정")
        if g != m.group(0):
            s = s[:m.start()] + g + s[m.end():]
        else:
            notes.pop()
    # 2) 어댑터 판
    if 'name="openarm_adapter_plate_vis"' not in s:
        m = re.search(r'<body name="openarm_mount_link"[^>]*>', s)
        ind = _indent_after(s, m.start())
        s = s[:m.end()] + (f'\n{ind}<geom name="openarm_adapter_plate_vis" type="box" size="{_f(*PLATE_HALF)}" '
                           f'pos="{_f(*PLATE_POS)}" material="metal_silver" contype="0" conaffinity="0" group="2" mass="0" />'
                           ) + s[m.end():]
        notes.append("어댑터 판")
    # 3) 손목 카메라 브래킷
    for side in ("left", "right"):
        name = f"wrist_{side}_camera_bracket_vis"
        if name in s:
            continue
        m = re.search(rf'<geom name="wrist_{side}_camera_vis"[^>]*/>', s)
        if m is None:
            continue
        ind = _indent_after(s, m.start())
        s = s[:m.end()] + (f'\n{ind}<geom name="{name}" type="box" size="{_f(*WRIST_BRACKET_HALF)}" '
                           f'pos="{_f(*WRIST_BRACKET_POS)}" rgba="0.25 0.25 0.25 1" contype="0" conaffinity="0" group="2" mass="0" />'
                           ) + s[m.end():]
        notes.append(f"{side} 손목 카메라 브래킷")
    # 4) 손잡이 충돌 상자 (+ 다리와 충돌 제외)
    if "base_handle_L_collision" not in s:
        m = re.search(r'<geom name="base_collision"[^>]*/>', s)
        g = m.group(0)
        grp = re.search(r'group="(\d)"', g).group(1)
        rgba = re.search(r'rgba="([^"]*)"', g).group(1)
        ind = _indent_after(s, m.start())
        ins = ""
        for side, sy in (("L", 1), ("R", -1)):
            p = (HANDLE_POS[0], sy * HANDLE_POS[1], HANDLE_POS[2])
            ins += (f'\n{ind}<geom name="base_handle_{side}_collision" type="box" size="{_f(*HANDLE_HALF)}" pos="{_f(*p)}" '
                    f'rgba="{rgba}" group="{grp}" contype="1" conaffinity="1" mass="0" />')
        s = s[:m.end()] + ins + s[m.end():]
        notes.append("손잡이 충돌 상자")
    if '<exclude body1="base_link" body2="hip_L_Link"' not in s and "<contact>" in s:
        ex = "".join(f'\n    <exclude body1="base_link" body2="{b}_{lr}_Link" />' for lr in ("L", "R") for b in ("hip", "knee"))
        s = s.replace("<contact>", "<contact>" + ex, 1)
        notes.append("base–다리 충돌 제외")
    return s, notes


def _urdf_box_visual(name, xyz, half, rgba):
    return (f'\n    <visual>\n      <origin xyz="{_f(*xyz)}" rpy="0 0 0" />\n      <geometry>\n        <box size="{_f(*[2 * h for h in half])}" />'
            f'\n      </geometry>\n      <material name="{name}_mat">\n        <color rgba="{rgba}" />\n      </material>\n    </visual>')


TRON1_LINKS = {"base_link": ("base_Link", "0.79 0.82 0.93 1")}
for _lr in ("L", "R"):
    for _n, _c in (("abad", "0.79 0.82 0.93 1"), ("hip", "0.95 0.95 0.95 1"), ("knee", "0.79 0.82 0.93 1"),
                   ("ankle", "0.79 0.82 0.93 1"), ("foot", "0.79 0.82 0.93 1")):
        TRON1_LINKS[f"{_n}_{_lr}_Link"] = (f"{_n}_{_lr}_Link", _c)


def fix_urdf(s, mesh_dir=None):
    """mesh_dir: TRON1 STL 메시 폴더 URI (예: package://tron1_openarm_sim/mjcf/sf/meshes/tron1/). 없으면 URDF 안의 참조로 추측."""
    notes = []
    # 5) TRON1 하체 메시 visual (RViz 표시용)
    if "tron1_mesh_vis_mat" not in s:
        # SF URDF 는 발목 메시를 이미 package://.../mjcf/sf/meshes/tron1/ 로 참조, PF 는 메시 참조가 없음 → mjcf/meshes/tron1/
        if mesh_dir is None:
            m = re.search(r'filename="(package://[^"]*/meshes/tron1/)[a-zA-Z_]+\.STL"', s)
            mesh_dir = m.group(1) if m else "package://tron1_openarm_sim/mjcf/meshes/tron1/"
        if True:
            for link, (mesh, rgba) in TRON1_LINKS.items():
                ml = re.search(rf'<link name="{link}">', s)
                if ml is None:
                    continue
                vis = (f'\n    <visual>\n      <origin xyz="0 0 0" rpy="0 0 0" />\n      <geometry>\n'
                       f'        <mesh filename="{mesh_dir}{mesh}.STL" scale="1 1 1" />\n      </geometry>\n'
                       f'      <material name="tron1_mesh_vis_mat_{link}">\n        <color rgba="{rgba}" />\n      </material>\n'
                       f'    </visual>')
                s = s[:ml.end()] + vis + s[ml.end():]
            s = s.replace('<material name="tron1_mesh_vis_mat_base_link">', '<material name="tron1_mesh_vis_mat_base_link"><!-- tron1_mesh_vis_mat -->', 1)
            notes.append("TRON1 하체 메시 visual")
    # 1) 어깨 브래킷 (openarm_shoulder_bracket_vis_mat 을 쓰는 visual)
    pat = (r'(<visual>\s*<origin xyz=")[^"]*(" rpy="[^"]*" />\s*<geometry>\s*<box size=")[^"]*'
           r'(" />\s*</geometry>\s*<material name="openarm_shoulder_bracket_vis_mat")')
    new = re.sub(pat, rf'\g<1>{_f(*BRACKET_POS)}\g<2>{_f(*[2 * h for h in BRACKET_HALF])}\g<3>', s)
    if new != s:
        notes.append("어깨 브래킷")
        s = new
    # 6) 어깨 브래킷 → OpenArm 원본 검은 브래킷 메시 (팔 메시와 같은 폴더 구조: .../meshes/visual/body/)
    pat = (r'<visual>\s*<origin xyz="[^"]*" rpy="[^"]*" />\s*<geometry>\s*<box size="[^"]*" />\s*</geometry>\s*'
           r'<material name="openarm_shoulder_bracket_vis_mat">\s*<color rgba="[^"]*" />')
    m = re.search(pat, s)
    arm = re.search(r'filename="(package://[^"]*/meshes/visual/)arm/', s)
    if m is not None and arm is not None:
        vis = (f'<visual>\n      <origin xyz="0 0 0" rpy="0 0 0" />\n      <geometry>\n'
               f'        <mesh filename="{arm.group(1)}body/{BRACKET_MESH}" scale="0.001 0.001 0.001" />\n      </geometry>\n'
               f'      <material name="openarm_shoulder_bracket_vis_mat">\n        <color rgba="{MATTE_BLACK}" />')
        s = s[:m.start()] + vis + s[m.end():]
        notes.append("어깨 브래킷 → 원본 검은 메시")
    # 2) 어댑터 판
    if "openarm_adapter_plate_vis_mat" not in s:
        m = re.search(r'<link name="openarm_mount_link">', s)
        s = s[:m.end()] + _urdf_box_visual("openarm_adapter_plate_vis", PLATE_POS, PLATE_HALF, "0.796078 0.796078 0.796078 1") + s[m.end():]
        notes.append("어댑터 판")
    # 3) 손목 카메라 몸체 + 브래킷 (URDF 에는 카메라 몸체도 없었다)
    for side in ("left", "right"):
        if f"wrist_{side}_camera_bracket_vis_mat" in s:
            continue
        m = re.search(rf'<link name="openarm_{side}_hand">', s)
        if m is None:
            continue
        s = s[:m.end()] + _urdf_box_visual(f"wrist_{side}_camera_vis", WRIST_CAM_POS, WRIST_CAM_HALF, "0.1 0.1 0.1 1") \
            + _urdf_box_visual(f"wrist_{side}_camera_bracket_vis", WRIST_BRACKET_POS, WRIST_BRACKET_HALF, "0.25 0.25 0.25 1") + s[m.end():]
        notes.append(f"{side} 손목 카메라")
    # 4) 손잡이 충돌 (MoveIt 이 팔꿈치 경로를 피하도록)
    if "base_handle_L" not in s:
        m = re.search(r'<link name="base_link">', s)
        ins = ""
        for side, sy in (("L", 1), ("R", -1)):
            p = (HANDLE_POS[0], sy * HANDLE_POS[1], HANDLE_POS[2])
            ins += (f'\n    <!-- base_handle_{side}: TRON1 몸통 옆 손잡이 (tools/mech_fix.py) -->'
                    f'\n    <collision>\n      <origin xyz="{_f(*p)}" rpy="0 0 0" />\n      <geometry>\n'
                    f'        <box size="{_f(*[2 * h for h in HANDLE_HALF])}" />\n      </geometry>\n    </collision>')
        s = s[:m.end()] + ins + s[m.end():]
        notes.append("손잡이 충돌 상자")
    return s, notes


def apply_mech_fix(path):
    if path.endswith("_train.xml"):
        return ["학습용 모델은 건너뜀"]
    with open(path, encoding="utf-8") as fp:
        s = fp.read()
    s2, notes = (fix_urdf if path.endswith(".urdf") else fix_mjcf)(s)
    if s2 != s:
        with open(path, "w", encoding="utf-8") as fp:
            fp.write(s2)
    return notes or ["이미 적용됨"]


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for p in sys.argv[1:]:
        print(f"{os.path.basename(p)}: {', '.join(apply_mech_fix(p))}")
