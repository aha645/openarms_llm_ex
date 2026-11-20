"""TRON1 보행 PPO 학습 설정 — CPU 학습(tron1_env_cpu.py)과 GPU 학습(tron1_env_gpu.py)이 함께 쓴다.

보상 항·계수, 명령 분포, 팔 자세, 손 외력, 에피소드 길이를 여기 한 곳에서 정한다. 두 학습은 물리를 어디서 돌리는지만 다르고
(CPU: mujoco.rollout, GPU: MJX/MuJoCo Warp) 보상·교란은 같아야 같은 정책이 나온다.

SF 기본값 = 배포 중인 SF v3 정책(runs/upright/seed1)을 만든 설정 (보행 학습 13-5절):
  손 외력 25 N, 집기 자세 35 %, 서기 벌점(발 구름 5·하중 쏠림 3·발바닥 비틀림 5·기울기 4), 에피소드 60 s, 긴 서기 최대 50 s.
PF 에는 v3 조건을 적용하지 않는다 (손 외력·서기 벌점 없음, 집기 자세 0 %, 에피소드 20 s — 배포 PF 정책의 학습 조건).
환경 변수(TRON1_*)를 직접 주면 발 형태와 관계없이 그 값이 우선한다. v4~v10 재학습 실험 항목은 기본 꺼짐
(tools/gait_runs/train_v*.sh 가 켬).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "tron1_openarm_sim"))
from tron1_openarm_sim.arm_poses import POSES as ARM_POSES  # noqa: E402,F401  SRDF 이름 자세와 같은 값
from tron1_openarm_sim.locomotion_policy import PF, SF, SF_BASE_HEIGHT_CMD  # noqa: E402,F401  관측·행동 규약을 실행기와 공유

ACTION_SCALE = 0.25
POLICY_DT = 0.02
CMD_SCALE = np.array([2.0, 2.0, 0.25], np.float32)
BASE_HEIGHT_TARGET = {"pf": 0.68, "sf": SF_BASE_HEIGHT_CMD}

REWARD_SCALES = dict(
    keep_balance=1.0, tracking_lin_vel=3.0, tracking_ang_vel=1.5, base_height=-20.0, lin_vel_z=-0.5,
    ang_vel_xy=-0.05, torques=-8e-5, action_rate=-0.01, action_smooth=-0.01, orientation=-60.0,
    dof_pos_limits=-2.0, feet_distance=-100.0, contact_force=-2.0, contact_vel=-2.0, termination=-20.0, stand_drift=0.0, stand_yaw=0.0)
# SF 서기 모드 제자리 흐름 벌점 (SF 만 켬, 아래 Tron1OpenArmEnv·Tron1MjxEnv). 이 항이 없을 때 서기 모드 정책이 발을 디딘 채
# 0.15~0.5 °/s 로 몸통을 돌렸다 — 회전 추종 보상(허용 폭 0.25)·발 속도 벌점(0.5 m/s)으로는 이렇게 느린 흐름이 벌점이 안 됨.
# 서 있는 동안 몸통 회전·수평 속도를 허용 폭 STAND_DRIFT_SIGMA 로 좁게 본다.
# 기본은 꺼 둠(0): 4.0 으로 30 분 × 2 시드 학습해도 흐름이 0.21~0.35 °/s 로 조금만 줄었고 한 팔 carry 에서 넘어졌다
# → 배포는 biped_core 의 서기 중 방향 바로잡기(STAND_YAW_TOL)로 해결 (보행 학습 11절). 실험할 때 TRON1_STAND_DRIFT=4 로 켠다
STAND_DRIFT_SCALE = float(os.environ.get("TRON1_STAND_DRIFT", 0.0))
STAND_DRIFT_SIGMA = (0.02, 0.02)   # (회전 rad/s, 수평 속도 m/s)
# SF 서기 방향 유지 (stand_drift 를 대신함, 보행 학습 12절). 서기 정책은 발바닥을 디딘 채 몸통을 0.22 °/s(학습 환경)~0.45 °/s(배포) 로 돌린다.
# 다리에 수직축 관절이 없어 몸통이 돌면 두 발바닥이 바닥에서 비틀린다. 실패 원인 두 가지를 고침:
#  ① 학습 때 명령이 5 s 마다 바뀌어 서 있는 시간이 짧았다 (5 s 에 1.1° — 보상에 거의 안 드러남)
#     → 제자리 서기 명령의 STAND_LONG_P 비율은 10~20 s 유지
#  ② 몸통 속도 벌점은 서기 진입 감속·밀기에서 이미 포화돼 느린 비틀림에 기울기가 없었다
#     → 서기 시작 1 s 뒤부터 '기준 방향에서 돌아간 각도' 에 선형 벌점 (0.05 rad = 3° 에서 1, 상한 2)
#  ③ (v2) 기준 방향을 서기 시작 방향에 고정하면, 서 있는 동안 밀기·팔 반동으로 돈 몫(발을 못 떼 되돌릴 수 없음)이
#     계속 벌점으로 남아 학습 후반(교란이 커짐)에 벌점이 상한 근처(1.75/2)로 포화됐다 → 기준이 현재 방향을 시간상수
#     STAND_YAW_TAU 로 천천히 따라가게 함. 정책 자신의 느린 회전 r 은 r·τ 만큼 꾸준히 남아(0.22 °/s → 1.1°) 벌점이 되고,
#     밀려서 돈 몫은 τ 정도 뒤에 사라진다
STAND_YAW_SCALE = float(os.environ.get("TRON1_STAND_YAW", 3.0))
STAND_YAW_REF, STAND_YAW_SETTLE = 0.05, 1.0
STAND_YAW_TAU = float(os.environ.get("TRON1_STAND_YAW_TAU", 5.0))   # 0 이면 v1 (기준 고정)
STAND_LONG_P, STAND_LONG_STEPS = 0.5, (500, 1000)


# 속도 명령 범위 (환경 변수로 넓힐 수 있음, 기본은 v3 까지와 같음). 빠른 걸음 학습(v4): TRON1_VX=-0.5,0.8 TRON1_VY=0.35
def _rng(name, default):
    v = os.environ.get(name)
    if not v:
        return default
    a = [float(x) for x in v.split(",")]
    return (a[0], a[1]) if len(a) == 2 else (-a[0], a[0])


CMD_RANGES = dict(vx=_rng("TRON1_VX", (-0.35, 0.6)), vy=_rng("TRON1_VY", (-0.2, 0.2)), wz=(-0.9, 0.9))
VY_P = float(os.environ.get("TRON1_VY_P", 0.3))      # 일반 명령 중 옆걸음 성분을 넣는 비율
# 발 들기 (v4): 걸을 때 발을 거의 끌어(보상에 발 높이 항이 없음) 걸음이 느렸다 → 들어야 하는 구간(기대 접지 1 − des)에
# '디딘 발보다 걸음 설정 4번째 값(SF 0.12 m) 위' 에 못 미치는 만큼 제곱 벌점. 발 높이는 발 위치 센서(시뮬 정답)가 아니라
# 관절 각도 정기구학 + IMU 자세로 구한다 (실로봇에서도 얻을 수 있는 값)
CLEAR_SCALE = float(os.environ.get("TRON1_CLEAR", 0.0))
# 옆 방향 추종 (v5): tracking_lin_vel 은 앞뒤·옆 오차를 합쳐 보므로 빠른 전진에서 옆 오차가 묻힌다 — v3 는 앞으로 걸으며
# 걷는 속도의 16~19 % 만큼 옆으로 흘렀고 옆걸음은 0.3 m/s 명령에 0.1 m/s. 옆 속도 오차만 따로 좁은 폭으로 본다
TRACK_VY_SCALE = float(os.environ.get("TRON1_TRACK_VY", 0.0))
TRACK_VY_SIGMA = 0.02
# v6 (모두 기본 꺼짐, 사용자 요청: 일자로 걷기·팔을 움직여도 일자로·서 있을 때 발바닥 붙이기·옆걸음 보폭)
# 직진 방향 유지: 회전 명령 없이 걸을 때 걷기 시작한 방향(walk_yaw)에서 틀어진 각도에 벌점 (3° 에서 1, 상한 2).
#   팔은 걷는 중에도 3~6 s 마다 다른 자세로 움직이므로 '팔을 움직이며 일자로' 도 같이 배운다.
#   기준은 STAND_YAW_TAU 로 천천히 따라가 밀려서 돈 몫은 잊고, 정책 자신의 꾸준한 회전만 벌점으로 남긴다
WALK_HEADING_SCALE = float(os.environ.get("TRON1_WALK_HEADING", 0.0))
WALK_HEADING_REF = 0.05
# v8: 상한 2(=6°)에서 벌점이 더 안 커져 한 번 많이 돌면 되돌릴 이유가 없었다(v7 final: 직진 20 s 에 −66°) → 상한을 10(30°)으로
WALK_HEADING_CAP = float(os.environ.get("TRON1_WALK_HEADING_CAP", 2.0))
# v8 직진 중 회전 속도 벌점: 회전 추종 보상(허용 폭 0.25)은 직진 중 0.057 rad/s(3 °/s) 회전에 보상이 1.3 % 만 줄어 사실상 벌점이 없었다
# → 회전 명령 0 으로 걸을 때 (몸통 회전 속도 / 0.05 rad/s)² 벌점, 상한 4
WALK_YAWRATE_SCALE = float(os.environ.get("TRON1_WALK_YAWRATE", 0.0))
# 서 있을 때 발바닥: 서기 안정 구간에 한 발이라도 접촉력이 STAND_LIFT_F 아래면 벌점, 두 발 모두 붙고 미끄러지지 않으면 보상
STAND_LIFT_SCALE = float(os.environ.get("TRON1_STAND_LIFT", 0.0))
STAND_PLANTED_SCALE = float(os.environ.get("TRON1_STAND_PLANTED", 0.0))
STAND_LIFT_F = 5.0             # [N]
# 옆걸음 보폭: 두 발 사이 최소 거리 벌점(feet_distance −100)이 옆걸음에서 발을 모았다 벌리는 동작을 막아 보폭이 작았다 →
# 옆걸음 명령(|vy| > 0.05)일 때만 이 배율로 줄인다
FD_SIDE_SCALE = float(os.environ.get("TRON1_FD_SIDE", 1.0))
# v7 명령 분포 (TRON1_CMD_MIX=v7). 예전 분포는 일반 명령이 10 % 뿐이고 옆 성분(VY_P)은 그 안에서만 붙어, 0.3 m/s 급 옆걸음이
# 전체의 약 6 % 였다 → v6 에서 연습이 많은 직진은 방향 유지가 좋아졌지만(팔 움직이며 직진 13.5° → −1.8°) 드문 후진·옆걸음은
# 오히려 더 돌았다(−33°, −35°). 옆걸음·대각선을 따로 둔다:
#   서기 20 / 직진(앞·뒤) 15 / 옆걸음 15 / 대각선 10 / 느린 미세 이동 10 / 제자리 회전 15 / 일반 15 %
CMD_MIX = os.environ.get("TRON1_CMD_MIX", "")
# 발 들기 목표를 걸음 속도에 맞춘다 (TRON1_CLEAR_ADAPT=1): 고정 0.12 m 는 느린 걸음·제자리 걸음에서 닿을 수 없어 늘 벌점이
# 남았다(평가: 0.3 m/s 직진 7~8 cm, 0.6 m/s 10~12 cm) → 목표 = 0.06 + 0.07 × |명령 속도| (0.3 m/s 8.1 cm, 0.6 m/s 10.2 cm), 상한 0.12
CLEAR_ADAPT = os.environ.get("TRON1_CLEAR_ADAPT", "0") == "1"
# v9 서 있을 때 몸통 밀림: 서기 안정 구간에 기준 위치(서기 시작 위치, STAND_YAW_TAU 로 천천히 따라감)에서 수평으로 밀린 거리에 벌점
# (3 cm 에서 1, 상한 2). v7 best 는 팔을 움직일 때 몸통이 15.8 cm 밀렸고(v3 8.4 cm), 집기 중 방향 바로잡기 걸음이 31 s 이어졌다
STAND_SWAY_SCALE = float(os.environ.get("TRON1_STAND_SWAY", 0.0))
STAND_SWAY_REF = 0.03
# v10 느린 명령 추종: 작업대 앞 미세 이동·마커 기준 제자리 복귀는 0.02~0.06 m/s 명령을 쓴다. 절대 오차 추종 보상(폭 0.05)에서는
# 0.05 m/s 명령을 전혀 못 따라가도 보상이 95 % 남아, v9 best 는 전진 0.05 에 17 %, 후진 0.05 에 187 % 로 위치 맞추기에 45~80 s 걸렸다
# → 느린 명령(0 < |명령| < 0.15)에서 0.5 s 평균 속도(걸음마다 출렁임 제거)의 '명령 대비 비율 오차' 로 보상
TRACK_SLOW_SCALE = float(os.environ.get("TRON1_TRACK_SLOW", 0.0))
SLOW_TAU = 0.5

# ── SF 서기 버티기 (보행 학습 13절) ─────────────────────────────────────────────────────────────────────
# 실제 집기에서 손이 상자·작업대에 막혀 팔이 계속 밀면 손에 수십 N 이 걸리고, 서기 정책이 버티지 못해 한 발에 하중이 쏠리면
# 브리지가 서기를 풀어 디딤 걸음을 시작했다 (tools/stand_stress.py: 손 30 N·2 s 에서 12 회 중 7 회, 발 최대 66 cm 이동).
# 학습 때 팔 동작은 이름 자세 사이를 오가는 것뿐이었고 손에 힘이 걸린 적이 없다. 그래서 서 있는 동안
#   ① 팔 목표에 실제 집기 접근·파지·들기 자세(REACH_R, 상자 옆 5~15 cm, roll −90/−70°)를 섞고 (REACH_P)
#   ② 한 손(또는 두 손)에 0~HAND_F_MAX N 의 힘을 0.3~3 s 동안 무작위 방향으로 건다
#   ③ 보상: 발 구름 벌점 stand_feet (발 수평 속도, 0.05 m/s 폭) + 하중 쏠림 벌점 stand_load (한 발 하중 25 % 미만)
# 벌점은 '버틸 수 있으면 버티고, 정말 못 버티면 넘어지지 말고 디디게' 크기로: 한 걸음(약 0.4 s) 비용 ≈ 1.5 ≪ 넘어짐 20.
# 발 위치(서기 시작 자리)를 기준으로 벌점을 주지 않은 이유: 한 번 밀린 뒤 서 있는 내내 벌점이 남아 넘어지는 쪽이 싸진다.
# 기본값은 v3 설정(아래 값). 0 으로 두면 끔 (v2 이전 학습과 같음).
HAND_F_MAX = float(os.environ.get("TRON1_HAND_F", 25.0))           # [N]
REACH_P = float(os.environ.get("TRON1_REACH_P", 0.35))             # SF (PF 는 REACH_P_PF)
REACH_P_PF = float(os.environ.get("TRON1_REACH_P", 0.0))             # 팔 목표를 집기 자세로 뽑을 확률
STAND_FEET_SCALE = float(os.environ.get("TRON1_STAND_FEET", 5.0))
STAND_LOAD_SCALE = float(os.environ.get("TRON1_STAND_LOAD", 3.0))
STAND_TWIST_SCALE = float(os.environ.get("TRON1_STAND_TWIST", 5.0))
# v3 (서 있을 때 기울기·오래 서기): v2 정책은 손 외력에 대비해 서 있을 때 앞으로 6~8.5° 숙였다 (이전 3.8°).
# orientation(전체, −60·sin²)은 8° 에서도 작아 숙임을 막지 못했다 → 서기 1 s 뒤부터 STAND_TILT_FREE 를 넘는 기울기에 선형 벌점.
# 집기는 한 자리에 2~5 분 서 있는데 에피소드 20 s·서기 구간 10~20 s 였다 → 에피소드·긴 서기 구간을 늘릴 수 있게.
STAND_TILT_SCALE = float(os.environ.get("TRON1_STAND_TILT", 4.0))
STAND_TILT_FREE = math.sin(math.radians(3.0))     # 이만큼은 벌점 없음 (예전 정책 3.8°)
STAND_TILT_REF = math.sin(math.radians(5.0))      # 3 + 5 = 8° 에서 벌점 1 (상한 2)
EP_S = float(os.environ.get("TRON1_EP_S", 60.0))                    # SF 에피소드 [s] (PF 는 EP_S_PF)
EP_S_PF = float(os.environ.get("TRON1_EP_S", 20.0))
STAND_LONG_MAX_S = float(os.environ.get("TRON1_STAND_LONG_MAX_S", 50.0))
STAND_LONG_RANGE = (STAND_LONG_STEPS[0], int(round(STAND_LONG_MAX_S / POLICY_DT)))
STAND_FEET_SIGMA = 0.05        # 발 수평 속도 허용 폭 [m/s]
STAND_TWIST_SIGMA = 0.05       # 발바닥 수직축 회전 속도 허용 폭 [rad/s]
# stand_twist (v2): stand_feet 는 발의 직선 이동만 봐서, v1 학습(seed1)은 손 외력에 몸통이 돌 때 발바닥을 비틀어 받아냈다
# → 서 있는 동안 몸통이 천천히 돌아(5° 넘음) 브리지의 방향 바로잡기 걸음이 팔을 접는 동안 8 회 (stand_stress 25 N)
STAND_LOAD_MIN = 0.25          # 한 발 하중 비율 하한
# 오른팔 집기 자세 (tools/stand_stress.py 의 IK 로 구함: 상자 앞 0.53 m·옆 0.05/0.10/0.15 m, pitch 20°, roll −90°/−70°,
# 접근점·파지·10 cm 들기). 왼팔은 거울상
REACH_R = [[-0.538, 2.972, 0.732, 2.148, 0.282, 0.702, -0.799], [-1.201, 2.998, 0.667, 1.578, -0.18, 0.651, -0.746],
           [-0.812, 3.02, 0.631, 1.628, -0.107, 0.625, -0.45], [-0.554, 2.656, 0.658, 2.251, -0.032, 0.785, -0.702],
           [-1.191, 2.784, 0.644, 1.658, -0.385, 0.623, -0.678], [-0.737, 2.911, 0.476, 1.707, -0.188, 0.488, -0.423],
           [-0.464, 2.374, 0.496, 2.287, -0.441, 0.785, -0.44], [-1.111, 2.676, 0.482, 1.685, -0.472, 0.475, -0.639],
           [-0.711, 2.718, 0.409, 1.735, -0.393, 0.434, -0.348], [-1.225, 2.972, 0.705, 1.577, 0.064, 0.43, -0.893],
           [-0.907, 2.911, 0.785, 1.629, 0.118, 0.588, -0.657], [-0.75, 2.487, 0.881, 2.252, 0.211, 0.767, -0.974],
           [-1.167, 2.819, 0.603, 1.656, -0.056, 0.356, -0.824], [-0.805, 2.799, 0.607, 1.709, 0.037, 0.445, -0.57],
           [-0.469, 2.362, 0.509, 2.284, -0.07, 0.622, -0.673], [-1.178, 2.58, 0.615, 1.685, -0.293, 0.357, -0.715],
           [-0.781, 2.598, 0.557, 1.736, -0.173, 0.425, -0.456]]
# v9: 상자를 수평으로 들어 머리 카메라 시야 밖(손끝 앞 0.28 m·옆 0.20 m·높이 1.00 m)에 둔 자세 (robot_skills._hold_level).
# 실제 작업은 이 자세로 서 있고 걷는데 학습 팔 자세에 없었다. 집기 자세(REACH_R)에서 손 방향을 지키며 1 cm 씩 이어 푼 역기구학의 도착점
HOLD_R = [[0.843, 2.431, 0.285, 1.969, -0.62, 0.462, 1.056], [0.836, 2.399, 0.308, 1.97, -0.642, 0.492, 1.071],
          [0.828, 2.374, 0.329, 1.971, -0.667, 0.507, 1.087], [0.852, 2.434, 0.273, 1.975, -0.368, 0.754, 0.903],
          [0.837, 2.41, 0.302, 1.972, -0.405, 0.774, 0.927], [0.834, 2.388, 0.315, 1.97, -0.422, 0.785, 0.937]]
if os.environ.get("TRON1_HOLD", "0") == "1":
    REACH_R = REACH_R + HOLD_R * 2        # 집기 자세 17 + 들기 자세 12 (두 번 넣어 비율을 맞춤)



# 발 몸체 (발 높이·발바닥 비틀림 계산). PF 는 공 모양 발(foot_*), SF 는 발목+발바닥(ankle_*)
FEET_BODIES = {"pf": ("foot_L_Link", "foot_R_Link"), "sf": ("ankle_L_Link", "ankle_R_Link")}


def episode_s(foot):
    """에피소드 길이 [s] — SF v3 60 s, PF 20 s (환경 변수 TRON1_EP_S 가 있으면 둘 다 그 값)."""
    return EP_S if foot == "sf" else EP_S_PF


def reach_p(foot):
    """팔 목표를 집기 자세로 뽑을 확률 — SF v3 0.35, PF 0 (환경 변수 TRON1_REACH_P 가 있으면 둘 다 그 값)."""
    return REACH_P if foot == "sf" else REACH_P_PF


def reward_scales(foot):
    """발 형태별 보상 계수와 회전 추종 허용 폭 → (dict, ang_sigma). CPU·GPU 학습이 같은 값을 쓴다."""
    rs = dict(REWARD_SCALES)
    ang_sigma = 0.1
    if foot == "sf":
        # SF 4차(sf_v4): 회전 추종 보상을 키우고 허용 폭을 넓혀(0.25, LimX 기본) 제자리 회전을 배우게 (CPU 1~3차 기록은 보행 학습 문서)
        rs["tracking_ang_vel"] = 6.0
        ang_sigma = 0.25
        rs["stand_drift"] = -STAND_DRIFT_SCALE
        rs["stand_yaw"] = -STAND_YAW_SCALE
        rs["stand_feet"] = -STAND_FEET_SCALE
        rs["stand_load"] = -STAND_LOAD_SCALE
        rs["stand_twist"] = -STAND_TWIST_SCALE
        rs["stand_tilt"] = -STAND_TILT_SCALE
    else:
        rs["stand_feet"] = rs["stand_load"] = rs["stand_twist"] = rs["stand_tilt"] = 0.0
    rs["feet_clearance"] = -CLEAR_SCALE
    rs["tracking_vy"] = TRACK_VY_SCALE
    rs["walk_heading"] = -WALK_HEADING_SCALE
    rs["walk_yawrate"] = -WALK_YAWRATE_SCALE
    rs["stand_liftoff"] = -STAND_LIFT_SCALE
    rs["stand_sway"] = -STAND_SWAY_SCALE
    rs["tracking_slow"] = TRACK_SLOW_SCALE
    rs["stand_planted"] = STAND_PLANTED_SCALE
    return rs, ang_sigma
