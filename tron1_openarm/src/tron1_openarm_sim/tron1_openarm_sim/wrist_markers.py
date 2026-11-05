"""손목(openarm_*_link7) ArUco 마커 정의 — MJCF 생성기와 집기 기능(robot_skills)이 같은 값을 쓴다.

왜 손목인가
  * 손바닥(hand) 옆면은 손가락이 덮고, 손 뒤쪽 절반은 link7 안에 들어가 있어 마커 붙일 자리가 없다.
  * link7 은 손과 고정(관절 없음)이므로 link7 위의 마커 위치를 알면 손끝(TCP)도 정확히 안다.
  * 접근 자세(roll ±70~90°)에서는 link7 의 ±x 면이, roll 0° 에서는 ±y 면이 위(카메라 쪽)를 향한다 → 네 면 모두에 붙인다.

좌표: link7 기준. TCP = link7 + (0, 0, 0.19)  (hand = link7 + 0.1001, hand_tcp = hand + 0.09)
"""
import numpy as np

MARKER_SIZE = 0.04            # 검은 부분 한 변 [m] — 상자 마커와 같은 크기 (ArucoDetector 가 한 크기만 씀)
PLATE_HALF = MARKER_SIZE * 240 / 200 / 2   # 흰 테두리 포함 판 반변 (build_world.marker_png: 200px + 테두리 20px×2)
TCP_IN_LINK7 = np.array([0.0, 0.0, 0.1901])
FACE_Z = 0.045                # 마커 중심 높이 (link7 메시 z −0.022~0.106 의 가운데쯤, 손·손가락과 안 겹침)
FACE_OFF = 0.0305             # link7 표면(±x 0.029, ±y 0.038)에서 살짝 밖

# (id, 마커 중심 [link7], 면 바깥 방향 [link7])
FACES = {
    "right": [(40, (FACE_OFF, 0, FACE_Z), (1, 0, 0)), (41, (-FACE_OFF, 0, FACE_Z), (-1, 0, 0)),
              (42, (0, 0.0395, FACE_Z), (0, 1, 0)), (43, (0, -0.0395, FACE_Z), (0, -1, 0))],
    "left": [(44, (FACE_OFF, 0, FACE_Z), (1, 0, 0)), (45, (-FACE_OFF, 0, FACE_Z), (-1, 0, 0)),
             (46, (0, 0.0395, FACE_Z), (0, 1, 0)), (47, (0, -0.0395, FACE_Z), (0, -1, 0))],
}
IDS = {s: [f[0] for f in v] for s, v in FACES.items()}
ALL_IDS = set(IDS["right"]) | set(IDS["left"])


def tcp_from_marker(marker_id, p_marker_base, R_link7_base):
    """카메라로 잰 마커 중심(base 좌표) + link7 자세(정기구학) → 손끝 위치(base 좌표)."""
    for side, faces in FACES.items():
        for mid, pos, _ in faces:
            if mid == marker_id:
                return np.asarray(p_marker_base) + np.asarray(R_link7_base) @ (TCP_IN_LINK7 - np.array(pos))
    raise KeyError(marker_id)
