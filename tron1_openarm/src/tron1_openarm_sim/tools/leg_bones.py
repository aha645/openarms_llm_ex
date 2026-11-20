#!/usr/bin/env python3
"""저사양 모델(tron1_openarm_light.xml)의 다리를 보이게 한다 (MJCF 를 그 자리에서 수정, 여러 번 돌려도 같음).

    python3 tools/leg_bones.py mjcf/tron1_openarm_light.xml mjcf/sf/tron1_openarm_light.xml
build_tron1_openarm.py 가 모델을 새로 만들 때도 자동으로 적용한다.

문제: 저사양 모델은 시각 메시를 지우고 충돌 형상만 보이게 한다. 다리의 충돌 형상은 관절마다 떨어진 짧은 원기둥(허벅지 0.15 m,
정강이 0.26 m 중 일부)뿐이라 화면에서 다리가 이어져 보이지 않고, PF 발(공)은 숨김 그룹(3)이라 아예 안 보인다.
      또 몸통 상자 아래끝(base −0.167 m)과 고관절 모터(abad, base −0.26 m) 사이가 비어 다리가 허리에 붙어 보이지 않는다.
고침: 다리 링크마다 자기 관절에서 다음 관절까지 캡슐(뼈대)을 그리고, 고관절(hip)·무릎(knee) 관절 자리에 관절 축 방향 원통을,
      발끝에 발 모양을, 몸통과 두 고관절 사이에 골반 상자를 그린다. 허벅지·정강이 충돌 원기둥은 뼈대와 어긋나 막대가 두 개로
      보이므로 화면에서만 숨긴다(그룹 3 — 충돌·물리는 그룹과 무관해 그대로).
  시각 전용 — 충돌 없음(contype·conaffinity 0), 질량 0(링크 관성은 <inertial> 그대로), 그룹 2(LiDAR 는 그룹 0·1 만 봄).
  따라서 물리·센서·학습 결과는 그대로이고 화면만 바뀐다. 학습 모델(*_train.xml)에는 넣지 않는다.
"""
import re
import sys

RGBA = "0.75 0.78 0.9 1"
# 링크 → (다음 링크, 뼈대 반지름)
CHAIN = (("abad", "hip", 0.035), ("hip", "knee", 0.045), ("knee", None, 0.024))
# 관절 원통: 링크 → (반지름, 반폭). hip·knee 관절 축은 y 방향
JOINTS = (("hip", 0.062, 0.04), ("knee", 0.042, 0.032))
# 골반: base_link 기준 (몸통 상자 아래끝 z −0.167 → 고관절 모터 중심 z −0.26, 두 고관절 y ±0.105 사이)
PELVIS = dict(pos="0.03 0 -0.21", size="0.07 0.10 0.05")
HIDE = ("hip", "knee")   # 화면에서 숨길 충돌 원기둥 ({link}_{side}_collision)


def _body_open(s, name):
    m = re.search(rf'<body name="{name}"[^>]*>', s)
    if not m:
        raise ValueError(f"body {name} 없음")
    return m


def _child_pos(s, name):
    m = _body_open(s, name)
    p = re.search(r'pos="([^"]+)"', m.group(0))
    return p.group(1) if p else "0 0 0"


def _insert(s, body, geom):
    m = _body_open(s, body)
    indent = re.match(r"\s*", s[s.rfind("\n", 0, m.start()) + 1:]).group(0) + "  "
    return s[:m.end()] + "\n" + indent + geom + s[m.end():]


def _geom(name, **a):
    attrs = " ".join(f'{k}="{v}"' for k, v in a.items())
    return (f'<geom name="{name}" {attrs} rgba="{RGBA}" group="2" contype="0" conaffinity="0" mass="0" />')


def fix_mjcf(s):
    if "_bone_vis" in s:
        return s, []
    notes = []
    s = _insert(s, "base_link", _geom("pelvis_vis", type="box", **PELVIS))
    notes.append("골반")
    for side in ("L", "R"):
        for link in HIDE:
            name = f"{link}_{side}_collision"
            s, n = re.subn(rf'(<geom name="{name}"[^>]*?)group="2"', r'\1group="3"', s)
            if n != 1:
                raise ValueError(f"{name} 를 숨기지 못함")
        sole = f"ankle_{side}_Link" if f'name="ankle_{side}_Link"' in s else None
        end = sole or f"foot_{side}_Link"
        for link, nxt, r in CHAIN:
            body = f"{link}_{side}_Link"
            child = f"{nxt}_{side}_Link" if nxt else end
            s = _insert(s, body, _geom(f"{body}_bone_vis", type="capsule", size=r,
                                       fromto=f"0 0 0 {_child_pos(s, child)}"))
        for link, r, w in JOINTS:
            body = f"{link}_{side}_Link"
            s = _insert(s, body, _geom(f"{link}_{side}_joint_vis", type="cylinder", size=r, fromto=f"0 {-w} 0 0 {w} 0"))
        if sole:   # SF: 발목 관절 → 발바닥 상자 윗면
            s = _insert(s, sole, _geom(f"{sole}_bone_vis", type="capsule", size=0.02, fromto="0 0 0 0 0 -0.045"))
        else:      # PF: 공 모양 발 (충돌 공은 숨김 그룹 3)
            s = _insert(s, end, _geom(f"{end}_bone_vis", type="sphere", size=0.032))
        notes.append(f"{side} 다리 뼈대·고관절·무릎")
    return s, notes


def apply_leg_bones(path):
    with open(path, encoding="utf-8") as fp:
        s = fp.read()
    s2, notes = fix_mjcf(s)
    if s2 != s:
        with open(path, "w", encoding="utf-8") as fp:
            fp.write(s2)
    return notes or ["이미 적용됨"]


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for p in sys.argv[1:]:
        print(p, ", ".join(apply_leg_bones(p)))
