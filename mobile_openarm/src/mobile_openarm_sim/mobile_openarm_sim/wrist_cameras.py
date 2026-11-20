"""손목 카메라 정의 — MJCF 후처리(tools/wrist_cameras.py), 카메라 서버(camera_server.py), 집기 기능(robot_skills)이 같은 값을 쓴다.

왜 손목 카메라인가
  머리 카메라(1.24 m)는 손이 상자에 다가가면 손·손가락이 상자 윗면 마커를 위에서 가린다 (검증 기록 11절: 실패 원인).
  손에 고정한 카메라는 다가갈수록 상자가 크게 보이고, 카메라→손끝이 강체라 팔 처짐·몸통 흔들림과 상관없이
  '손 기준 상자 위치' 를 바로 잰다.

어디에 다는가 (손 좌표계: z = 접근 방향, 손끝 TCP = (0, 0, 0.09), 손가락은 ±y 로 벌어짐)
  * 손가락 판은 x ±0.031, z 0.011~0.096 → 카메라는 손의 −x 쪽 8.5 cm (브래킷), z −0.01.
  * 집기 앱의 접근 자세(roll −70~−90°)에서 손 −x 가 위를 향한다 → 카메라가 위에서 상자 윗면(마커)을 내려다본다.
    (roll +70~+90° 이면 카메라가 아래로 가므로, 손목 카메라가 있을 때 집기 앱은 음의 roll 을 먼저 시도한다)
  * 손끝 앞 상자 윗면 중심(손 좌표 약 (−0.025, 0, 0.095))을 보도록 손 축에서 30° 기울임.
    pitch 20° 접근이면 수평 아래 약 50° 로 내려다봄 → 마커를 44° 비스듬히, 파지 위치에서 약 12 cm 거리.

좌표 관례: MuJoCo 카메라는 −z 를 보고 +y 가 영상 위. 광학 좌표(ROS optical: x 오른쪽, y 아래, z 앞) = MuJoCo 카메라 축 × diag(1, −1, −1).
"""
import numpy as np

SIDES = ("left", "right")
CAM_NAME = {s: f"wrist_{s}_camera" for s in SIDES}
HAND_BODY = {s: f"openarm_{s}_hand" for s in SIDES}
POS_IN_HAND = (-0.085, 0.0, -0.01)
# 카메라 x축 = 손 −y, 카메라 y축(영상 위) = 손 −x 쪽으로 기운 방향 → 시선(−z) = 손 축에서 +x 쪽으로 30°
XYAXES = (0.0, -1.0, 0.0, -0.868, 0.0, 0.496)
FOVY = 80.0                     # 접근점(약 22 cm)~파지 위치(약 12 cm) 모두 상자가 화면 안에 들어오도록
WIDTH, HEIGHT = 640, 480
BODY_HALF = (0.012, 0.02, 0.01)   # 시각용 카메라 몸체 (충돌·질량 없음)
# 손과 카메라 몸체를 잇는 브래킷 (시각용)
BRACKET_POS = (-0.0505, 0.0, -0.005)
BRACKET_HALF = (0.0225, 0.015, 0.006)


def intrinsics(width=WIDTH, height=HEIGHT, fovy=FOVY):
    fy = (height / 2) / np.tan(np.radians(fovy) / 2)
    return np.array([[fy, 0, width / 2], [0, fy, height / 2], [0, 0, 1.0]])
