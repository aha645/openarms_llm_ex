"""Sole-foot(SF) 발바닥 충돌 상자 — 배포 모델(tools/sf_sole_box.py)과 GPU 학습 모델(rl/mjx_model.py)이 같이 쓴다.

LimX SF 발목 메시를 그대로 충돌 형상으로 쓰면, MuJoCo 의 메시–평면 충돌이 접촉점을 1 개만 만드는 경우가 많다
(서 있는 발 기준 1 개 222 회 / 3 개 109 회). 접촉점이 1 개면 회전 마찰이 0 이라 같은 정책이 상자 발보다
제자리에서 3 배 잘 돌았다 (0.53 vs 0.17 rad/s). 실제 고무 발바닥은 면으로 닿으므로 평평한 상자로 바꾼다.

상자 = 메시 최저점에서 SOLE_BAND 안의 꼭짓점(평평한 발바닥 면)이 차지하는 범위.
  SF_TRON1A: 0.182 × 0.046 m. CPU 시뮬에서 실제 접촉점 4701 개가 생긴 범위(17.7 × 4.8 cm)와 일치.
  1 cm 띠로 잡으면 둥근 옆면까지 들어가 폭 7.8 cm 가 되어 실제보다 옆으로 잘 버티는 발이 된다.
"""
import mujoco
import numpy as np

SOLE_BAND = 0.002          # [m] 메시 최저점에서 이 높이 안의 꼭짓점 = 평평한 발바닥 면
SOLE_THICK = 0.01          # [m] 상자 두께 (바닥면이 메시 최저점과 같은 높이)


def sole_box(m, g, band=SOLE_BAND, thick=SOLE_THICK):
    """메시 geom g 의 발바닥 상자 → (중심, 반크기). 좌표는 geom 이 붙은 body 기준, 자세는 body 축과 같음."""
    mid = m.geom_dataid[g]
    a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
    v = np.array(m.mesh_vert[a:a + n], float)
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, m.geom_quat[g])
    vb = v @ R.reshape(3, 3).T + m.geom_pos[g]           # geom 좌표 → body 좌표
    zmin = vb[:, 2].min()
    sole = vb[vb[:, 2] < zmin + band]
    lo, hi = sole[:, :2].min(0), sole[:, :2].max(0)
    center = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, zmin + thick / 2])
    half = np.array([(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, thick / 2])
    return center, half
