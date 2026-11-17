"""MJX(GPU 물리) 학습용 모델 만들기 — CPU 학습 모델(*_train.xml)에서 충돌을 단순화한다.

왜 바꾸나
  * SF 발목 충돌 형상은 꼭짓점 8천여 개 메시다. MJX 는 메시 충돌을 GPU 에서 매우 느리게 처리하므로
    발바닥을 상자(box)로 바꾼다. 상자 크기는 메시 볼록 껍질의 '바닥 쪽 1 cm' 꼭짓점이 차지하는 범위로 정한다.
  * 바닥과 닿는 것은 발만 남긴다 (다리끼리·몸통·무릎 충돌 끔). 넘어짐은 몸통 높이·기울기로 판정하므로
    학습에 필요 없고, 접촉 후보 수가 GPU 시간을 좌우한다.
  * PF 는 발이 이미 구(sphere)라 그대로 둔다.
질량·관성·관절·구동기·센서는 손대지 않는다 (같은 정책 규약).

    python3 mjx_model.py ../src/tron1_openarm_sim/mjcf/sf/tron1_openarm_train.xml     # 바꾼 내용 출력 + MJX 로드 확인
"""
import sys

import mujoco
import numpy as np

SOLE_THICK = 0.01          # 발바닥 상자 두께 [m]
SOLE_BAND = 0.002          # 메시 최저점에서 이 높이 안의 꼭짓점 = 평평한 발바닥 면
# 2 mm 로 정한 근거: CPU 시뮬(메시 충돌)로 서기·걷기·회전·옆걸음 21 s 동안 왼발 접촉점 4701 개를 모으니
# 발 좌표 x −8.6~+9.1 cm(17.7 cm), y 폭 4.8 cm 에 생겼다. 2 mm 띠 = 18.2 × 4.6 cm 로 일치.
# 1 cm 띠로 잡으면 둥근 옆면까지 들어가 폭이 7.8 cm 가 되어 실제보다 옆으로 잘 버티는 발이 된다.


def _sole_box(m, g):
    """메시 geom g 의 발바닥 상자 (geom 이 붙은 body 좌표의 중심, 반크기)."""
    mid = m.geom_dataid[g]
    a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
    v = np.array(m.mesh_vert[a:a + n], float)
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, m.geom_quat[g])
    vb = v @ R.reshape(3, 3).T + m.geom_pos[g]           # geom 좌표 → body 좌표
    zmin = vb[:, 2].min()
    sole = vb[vb[:, 2] < zmin + SOLE_BAND]
    lo, hi = sole[:, :2].min(0), sole[:, :2].max(0)
    center = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, zmin + SOLE_THICK / 2])
    half = np.array([(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, SOLE_THICK / 2])
    return center, half


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
