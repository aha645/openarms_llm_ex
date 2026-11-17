#!/usr/bin/env python3
"""SF 모델의 발목 충돌 메시(ankle_*_collision)를 평평한 발바닥 상자로 바꾼다 (그 자리에서 파일 수정).

왜: tron1_openarm_sim/sole.py 설명 참고. 요약하면 메시 발은 MuJoCo 충돌 특성 때문에 제자리에서 비현실적으로 잘 돌고,
GPU 학습(MJX)은 상자 발을 쓰므로 배포 모델도 같은 발이어야 학습 결과가 그대로 옮겨진다.
마찰·condim·contype·group·rgba 등 다른 속성은 그대로 두고 형상만 바꾼다. 이미 바뀐 파일은 건드리지 않는다.

    python3 tools/sf_sole_box.py mjcf/sf/tron1_openarm.xml mjcf/sf/tron1_openarm_light.xml mjcf/sf/tron1_openarm_train.xml
build_tron1_openarm.py --foot sf 가 모델을 새로 만들 때도 자동으로 적용한다.
"""
import os
import re
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from tron1_openarm_sim.sole import sole_box  # noqa: E402

TAG = re.compile(r'<geom name="(ankle_[LR]_collision)"([^>]*?)/>')


def apply_sole_box(path):
    """path 의 MJCF 를 고친다. 반환: 바꾼 geom 설명 목록 (비어 있으면 이미 적용됨)."""
    with open(path, encoding="utf-8") as fp:
        s = fp.read()
    if not any('type="mesh"' in m.group(2) for m in TAG.finditer(s)):
        return []
    m = mujoco.MjModel.from_xml_path(path)
    notes = []

    def repl(mt):
        name, attrs = mt.group(1), mt.group(2)
        if 'type="mesh"' not in attrs:
            return mt.group(0)
        c, h = sole_box(m, m.geom(name).id)
        attrs = re.sub(r'\s(type|mesh|pos|quat|size)="[^"]*"', "", attrs)
        fmt = lambda v: " ".join(f"{x:.4f}" for x in v)  # noqa: E731
        notes.append(f"{name}: 상자 {np.round(2 * h, 3)} m, 중심 {np.round(c, 3)}")
        return (f'<geom name="{name}" type="box" pos="{fmt(c)}" size="{fmt(h)}"{attrs} />'
                .replace("  />", " />"))
    s = TAG.sub(repl, s)
    with open(path, "w", encoding="utf-8") as fp:
        fp.write(s)
    mujoco.MjModel.from_xml_path(path)          # 다시 읽어 확인
    return notes


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for p in sys.argv[1:]:
        n = apply_sole_box(p)
        print(f"{p}: " + ("; ".join(n) if n else "이미 적용됨"))
