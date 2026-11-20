#!/usr/bin/env python3
"""MJCF 의 양손(openarm_*_hand)에 손목 카메라를 단다 (그 자리에서 파일 수정, 이미 있으면 건드리지 않음).
값은 mobile_openarm_sim/wrist_cameras.py. 학습용 모델(*_train.xml)에는 달지 않는다 (카메라를 안 쓰고, MJX 모델을 바꾸지 않으려고).

    python3 tools/wrist_cameras.py mjcf/mobile_openarm.xml mjcf/mobile_openarm_light.xml
mobile_openarm/scripts/regen_models.sh 가 모델을 새로 만들 때도 자동으로 적용한다.
"""
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mobile_openarm_sim import wrist_cameras as WC  # noqa: E402


def _f(*v):
    return " ".join(f"{x:g}" for x in v)


def apply_wrist_cameras(path):
    """path 의 MJCF 에 손목 카메라를 넣는다. 반환: 넣은 카메라 이름 목록 (비어 있으면 이미 있음)."""
    with open(path, encoding="utf-8") as fp:
        s = fp.read()
    added = []
    for side in WC.SIDES:
        name = WC.CAM_NAME[side]
        if f'name="{name}"' in s:
            continue
        m = re.search(rf'(<body name="{WC.HAND_BODY[side]}"[^>]*>)', s)
        if m is None:
            raise RuntimeError(f"{path}: {WC.HAND_BODY[side]} body 없음")
        indent = re.search(r"\n([ \t]*)[^\n]*$", s[:m.start()]).group(1) + "  "
        ins = (f'\n{indent}<camera name="{name}" pos="{_f(*WC.POS_IN_HAND)}" fovy="{WC.FOVY:g}" xyaxes="{_f(*WC.XYAXES)}" />'
               f'\n{indent}<geom name="{name}_vis" type="box" size="{_f(*WC.BODY_HALF)}" pos="{_f(*WC.POS_IN_HAND)}" '
               f'rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0" group="2" mass="0" />'
               f'\n{indent}  <geom name="{name}_bracket_vis" type="box" size="{_f(*WC.BRACKET_HALF)}" pos="{_f(*WC.BRACKET_POS)}" '
               f'rgba="0.25 0.25 0.25 1" contype="0" conaffinity="0" group="2" mass="0" />')
        s = s[:m.end()] + ins + s[m.end():]
        added.append(name)
    if added:
        with open(path, "w", encoding="utf-8") as fp:
            fp.write(s)
    return added


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for p in sys.argv[1:]:
        if p.endswith("_train.xml"):
            print(f"{p}: 학습용 모델은 건너뜀")
            continue
        print(f"{p}: {apply_wrist_cameras(p) or '이미 있음'}")
