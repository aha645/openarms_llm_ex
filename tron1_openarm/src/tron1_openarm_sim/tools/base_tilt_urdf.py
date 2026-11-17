#!/usr/bin/env python3
"""URDF 의 고정 base_footprint → base_link 를 '가상 관절 3개'로 바꾼다 (이족 전용).

    base_footprint ─ base_height_joint (prismatic z) ─ base_pitch_joint (revolute y) ─ base_roll_joint (revolute x) ─ base_link

왜: 이족은 서 있는 높이가 ±2 cm 출렁이고, 팔을 뻗으면 몸통이 최대 7° 숙여진다. 고정 변환(높이 0.68/0.73, 기울기 0)이면
MoveIt·TF 가 이 기울기를 몰라 0.5 m 앞 손끝·상자·작업대 상판 위치가 수 cm 어긋난다.
값은 브리지(biped_core.BipedOpenArmSim.joint_state)가 IMU 자세 + 다리 기구학 높이로 /joint_states 에 함께 발행한다.
관절 순서 Ry(pitch)·Rx(roll) 은 R = Rz(yaw)·Ry(pitch)·Rx(roll) 분해와 같다 (yaw 는 odom → base_footprint 가 담당).

    python3 base_tilt_urdf.py ../urdf/tron1_openarm.urdf ../urdf/tron1_openarm_sf.urdf     # 이미 만든 URDF 를 그 자리에서 고침
make_urdf_tron1.py 가 URDF 를 새로 만들 때도 이 함수를 부른다. 이미 바뀐 파일에 다시 실행해도 그대로다.
"""
import re
import sys

JOINT_RE = re.compile(
    r'(?P<indent>[ \t]*)<joint name="base_footprint_to_base_link" type="fixed">\s*'
    r'<parent link="base_footprint"\s*/>\s*<child link="base_link"\s*/>\s*'
    r'<origin xyz="0 0 (?P<h>[-0-9.eE]+)" rpy="0 0 0"\s*/>\s*</joint>\n?')

HEIGHT_RANGE = (0.30, 1.00)      # [m] 넘어졌을 때도 TF 가 끊기지 않을 만큼 넓게
TILT_LIMIT = 1.0                  # [rad] (넘어짐 판정 45° 보다 넓게)


def add_base_tilt_joints(urdf_text):
    m = JOINT_RE.search(urdf_text)
    if m is None:
        if 'name="base_height_joint"' in urdf_text:
            return urdf_text                       # 이미 바뀐 파일
        raise ValueError("고정 관절 base_footprint_to_base_link 를 찾지 못했습니다")
        # (형식이 다르면 mjcf_to_urdf.py 출력이 바뀐 것 — 이 정규식을 맞추세요)
    i = m.group("indent")
    lim = f'<limit lower="{-TILT_LIMIT}" upper="{TILT_LIMIT}" effort="0" velocity="10" />'
    block = (
        f'{i}<!-- 이족: 서 있는 높이 {m.group("h")} m 고정 대신 실제 높이·기울기 (tools/base_tilt_urdf.py). '
        f'값은 브리지가 /joint_states 로 발행 -->\n'
        f'{i}<joint name="base_height_joint" type="prismatic">\n'
        f'{i}  <parent link="base_footprint" />\n{i}  <child link="base_height_link" />\n'
        f'{i}  <origin xyz="0 0 0" rpy="0 0 0" />\n{i}  <axis xyz="0 0 1" />\n'
        f'{i}  <limit lower="{HEIGHT_RANGE[0]}" upper="{HEIGHT_RANGE[1]}" effort="0" velocity="10" />\n'
        f'{i}</joint>\n'
        f'{i}<link name="base_height_link" />\n'
        f'{i}<joint name="base_pitch_joint" type="revolute">\n'
        f'{i}  <parent link="base_height_link" />\n{i}  <child link="base_pitch_link" />\n'
        f'{i}  <origin xyz="0 0 0" rpy="0 0 0" />\n{i}  <axis xyz="0 1 0" />\n{i}  {lim}\n'
        f'{i}</joint>\n'
        f'{i}<link name="base_pitch_link" />\n'
        f'{i}<joint name="base_roll_joint" type="revolute">\n'
        f'{i}  <parent link="base_pitch_link" />\n{i}  <child link="base_link" />\n'
        f'{i}  <origin xyz="0 0 0" rpy="0 0 0" />\n{i}  <axis xyz="1 0 0" />\n{i}  {lim}\n'
        f'{i}</joint>\n')
    return urdf_text[:m.start()] + block + urdf_text[m.end():]


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for path in sys.argv[1:]:
        with open(path, encoding="utf-8") as f:
            s = f.read()
        out = add_base_tilt_joints(s)
        if out != s:
            with open(path, "w", encoding="utf-8") as f:
                f.write(out)
        print(f"{path}: {'가상 관절 3개 추가' if out != s else '이미 적용됨'}")
