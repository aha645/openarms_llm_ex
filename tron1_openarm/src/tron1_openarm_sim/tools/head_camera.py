#!/usr/bin/env python3
"""머리 카메라 높이·숙임을 MJCF·URDF 에 맞춘다 (그 자리에서 수정, 여러 번 돌려도 같음).

    python3 tools/head_camera.py mjcf/sf/tron1_openarm.xml mjcf/sf/tron1_openarm_light.xml mjcf/sf/tron1_openarm_train.xml \
        mjcf/tron1_openarm.xml mjcf/tron1_openarm_light.xml mjcf/tron1_openarm_train.xml \
        urdf/tron1_openarm_sf.urdf urdf/tron1_openarm.urdf
build_tron1_openarm.py 도 같은 값(CAMERA_HEIGHT, CAMERA_PITCH_DEG)을 쓴다.

높이 1.243 → 1.07 m (마스트를 절반으로, 사용자 제안): 같은 자리에서 상자 마커 32~36 → 38~43 px (+20 %), 접근 자세에서 팔이
마커를 가리지 않음 (1.00 m 는 접근 중 팔이 옆 상자를 가림). 짧은 마스트는 몸통이 기울 때 덜 흔들리고 실로봇에서 더 단단하다.
숙임은 작업대를 보는 점(base 앞 0.44 m, 높이 0.75 m)이 예전(1.243 m, 62°)과 같도록 → 51°.
"""
import math
import re
import sys

HEIGHT = 1.07                 # 지면 높이 [m] (서 있을 때)
OLD_HEIGHT = 1.243
AIM = (0.44, 0.75)            # base 기준 시선이 지나는 점 (앞, 높이)
CAM_FWD = 0.18                # base 기준 카메라 앞 위치 (mount_x 0.08 + 0.10)
PITCH = math.degrees(math.atan2(HEIGHT - AIM[1], AIM[0] - CAM_FWD))
MAST_BOT_ABOVE_ARM = 0.065    # 어깨 브래킷 윗면 = 어깨 축 + 0.065 (build_tron1_openarm.py)


def _f(*v):
    return " ".join(f"{x:g}" for x in v)


def fix_mjcf(s):
    m = re.search(r'<camera name="head_camera" pos="([^"]*)"', s)
    if m is None:
        return s, []
    a = math.radians(PITCH)
    xy = _f(0, -1, 0, round(math.sin(a), 6), 0, round(math.cos(a), 6))
    if f'xyaxes="{xy}"' in s:                      # 숙임이 이미 새 값이면 높이도 바꾼 것
        return s, ["이미 적용됨"]
    x, y, z = map(float, m.group(1).split())
    arm_z = float(re.search(r'<body name="openarm_left_link0" pos="[^ ]+ [^ ]+ ([^"]+)"', s).group(1))
    mast_bot = arm_z + MAST_BOT_ABOVE_ARM          # 어깨 브래킷 윗면 (body_link0 기준)
    nz = round(z - (OLD_HEIGHT - HEIGHT), 4)
    s = re.sub(r'(<camera name="head_camera" pos=")[^"]*(" fovy="[^"]*" xyaxes=")[^"]*"',
               rf'\g<1>{_f(x, y, nz)}\g<2>{xy}"', s)
    s = re.sub(r'(<geom name="head_camera_vis"[^>]*?pos=")[^"]*"', rf'\g<1>{_f(x - 0.02, 0, nz)}"', s)
    ln = max(0.06, nz - mast_bot)
    s = re.sub(r'(<geom name="head_mast_vis" type="cylinder" size=")[^"]*(" pos=")[^"]*"',
               rf'\g<1>{_f(0.012, round(ln / 2, 4))}\g<2>{_f(x - 0.05, 0, round(mast_bot + ln / 2, 4))}"', s)
    return s, [f"카메라 z {z} → {nz} (지면 {HEIGHT} m), 숙임 {PITCH:.1f}°, 마스트 {ln:.3f} m"]


def fix_urdf(s):
    m = re.search(r'(<joint name="openarm_body_link0_to_head_camera_link" type="fixed">.*?<origin xyz=")([^"]*)(" rpy="0 )([^ ]+)( 0" />)', s, re.S)
    if m is None:
        return s, []
    rp = f"{math.radians(PITCH):.6f}"
    if m.group(4) == rp:
        return s, ["이미 적용됨"]
    x, y, z = map(float, m.group(2).split())
    nz = round(z - (OLD_HEIGHT - HEIGHT), 4)
    s = s[:m.start()] + m.group(1) + _f(x, y, nz) + m.group(3) + rp + m.group(5) + s[m.end():]
    notes = [f"head_camera_link z {z} → {nz}, pitch {PITCH:.1f}°"]
    # 시각 형상 (카메라 몸체·마스트) — body_link0 의 visual
    mc = re.search(r'(<origin xyz=")([^"]*)(" rpy="[^"]*" />\s*<geometry>\s*<box size="0.04 0.1 0.04" />\s*</geometry>\s*<material name="head_camera_vis_mat")', s)
    if mc:
        cx, cy, cz = map(float, mc.group(2).split())
        s = s[:mc.start()] + mc.group(1) + _f(cx, cy, round(cz - (OLD_HEIGHT - HEIGHT), 4)) + mc.group(3) + s[mc.end():]
    mm = re.search(r'(<origin xyz=")([^"]*)(" rpy="[^"]*" />\s*<geometry>\s*<cylinder radius="0.012" length=")([^"]*)(" />\s*</geometry>\s*<material name="head_mast_vis_mat")', s)
    if mm:
        mx, my, mz = map(float, mm.group(2).split())
        L = float(mm.group(4))
        bot = mz - L / 2
        nl = max(0.06, L - (OLD_HEIGHT - HEIGHT))
        s = s[:mm.start()] + mm.group(1) + _f(mx, my, round(bot + nl / 2, 4)) + mm.group(3) + f"{nl:.4f}" + mm.group(5) + s[mm.end():]
    return s, notes


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for p in sys.argv[1:]:
        s = open(p, encoding="utf-8").read()
        s2, notes = (fix_urdf if p.endswith(".urdf") else fix_mjcf)(s)
        if s2 != s:
            open(p, "w", encoding="utf-8").write(s2)
        print(f"{p}: {', '.join(notes) or '해당 없음'}")
