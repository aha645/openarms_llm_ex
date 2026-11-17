"""MJX(GPU 물리) 학습용 모델 만들기 — CPU 학습 모델(*_train.xml)에서 충돌을 단순화한다.

왜 바꾸나
  * SF 발: 지금 모델 파일(mjcf/sf/*.xml)은 이미 평평한 발바닥 상자다 (tools/sf_sole_box.py, tron1_openarm_sim/sole.py).
    예전 메시 발 모델을 넣으면 같은 함수로 상자로 바꾼다 (MJX 는 메시 충돌이 매우 느리다).
  * 바닥과 닿는 것은 발만 남긴다 (다리끼리·몸통·무릎 충돌 끔). 넘어짐은 몸통 높이·기울기로 판정하므로
    학습에 필요 없고, 접촉 후보 수가 GPU 시간을 좌우한다.
  * PF 는 발이 이미 구(sphere)라 그대로 둔다.
질량·관성·관절·구동기·센서는 손대지 않는다 (같은 정책 규약).

    python3 mjx_model.py ../src/tron1_openarm_sim/mjcf/sf/tron1_openarm_train.xml     # 바꾼 내용 출력 + MJX 로드 확인
"""
import os
import sys

import mujoco
import numpy as np

# 발바닥 상자 계산은 배포 모델(tools/sf_sole_box.py)과 같은 함수를 쓴다. 지금 SF 모델 파일은 이미 상자 발이라
# 이 변환은 예전(메시 발) 모델 파일을 넣었을 때만 동작한다.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "tron1_openarm_sim"))
from tron1_openarm_sim.sole import sole_box as _sole_box  # noqa: E402


def build_mjx_model(xml, verbose=False, iterations=None, ls_iterations=None, cone=None):
    """xml(MJCF) → (MjModel, 바꾼 내용 설명 목록). 반환 모델은 mjx.put_model 로 GPU 에 올릴 수 있다."""
    m0 = mujoco.MjModel.from_xml_path(xml)
    spec = mujoco.MjSpec.from_file(xml)
    notes = []
    feet = []
    for g in range(m0.ngeom):
        if not (m0.geom_contype[g] or m0.geom_conaffinity[g]):
            continue
        name = m0.geom(g).name
        body = m0.body(m0.geom_bodyid[g]).name
        gs = spec.geom(name)
        if m0.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        if body.startswith(("ankle_", "foot_")):
            feet.append(name)
            if m0.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                c, h = _sole_box(m0, g)
                gs.type = mujoco.mjtGeom.mjGEOM_BOX
                gs.meshname = ""
                gs.pos, gs.quat, gs.size = c, [1, 0, 0, 0], h
                notes.append(f"{name}: 메시 → 발바닥 상자 {np.round(2 * h, 3)} m (중심 {np.round(c, 3)})")
            # 발은 바닥(contype 1)과만 닿게: contype 2 / conaffinity 1 → 발끼리(2&1=0) 안 닿음
            gs.contype, gs.conaffinity = 2, 1
        else:
            gs.contype, gs.conaffinity = 0, 0
            notes.append(f"{name}: 충돌 끔")
    m = spec.compile()
    # 접촉 솔버: CPU 기본(반복 100, 선 탐색 50)은 MJX 에서 매 스텝 거의 그 횟수만큼 돌아 매우 느리다
    if iterations is not None:
        m.opt.iterations = iterations
    if ls_iterations is not None:
        m.opt.ls_iterations = ls_iterations
    if cone is not None:
        m.opt.cone = cone
    # 바닥은 contype 1 / conaffinity 3 → 발(contype 2)과 닿음
    for g in range(m.ngeom):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            m.geom_contype[g], m.geom_conaffinity[g] = 1, 3
    if verbose:
        print("\n".join(notes))
        print("발:", feet)
    return m, notes


if __name__ == "__main__":
    from mujoco import mjx
    m, _ = build_mjx_model(sys.argv[1], verbose=True)
    mx = mjx.put_model(m)
    print("MJX put_model OK — nv", m.nv, "ngeom", m.ngeom)
