"""이족(TRON1) + OpenArm 의 팔 이름 자세 — SRDF·시뮬 초기 자세·강화학습 환경이 모두 이 값을 쓴다.

어깨를 TRON1 몸통 윗면 바로 위(약 10 cm)에 붙인 설계(doc/tron1/design.md 설계 변경 이력 ⑦, validation-log.md 4절)에서는
바퀴형의 ready/carry(위팔을 아래로 내린 채 팔꿈치만 굽힘)가 TRON1 몸통 상자를 관통한다.
그래서 위팔을 앞으로 든(joint1) 자세로 다시 정했다. 몸통·다른 팔과 2 cm 이상 떨어지도록 격자 탐색으로 고름.

오른팔 기준. 왼팔은 거울상: [-j1, -j2, -j3, j4, -j5, j6, -j7]
  home     : 팔을 곧게 내리고 바깥으로 0.4 rad 벌림, 앞으로 0.1 rad (손끝 앞 0.19 m, 옆 ±0.41 m, 높이 0.30 m)
             예전 home(벌림 0.2 rad)은 팔꿈치가 TRON1 몸통 옆 손잡이(base 아래 0.10 m, 옆 0.17 m)와 겹쳤다.
             팔꿈치가 손잡이 높이에 오므로 옆으로 벌려 피한다 (손잡이·몸통·다리와 3.0 cm). 앞으로 0.3 rad 들어 피하는 자세는
             손이 몸 앞 0.29 m 로 나가 작업대 앞에서 상판 아래(MoveIt 작업대 상자 안)에 들어가 계획이 막혔다(−10).
             Nav2 footprint 는 이 손끝 폭(±0.41 m)에 맞춰 ±0.40 m + padding 0.02 (make_nav2_params_tron1.py)
  ready    : 손끝 ≈ 앞 0.45 m, 높이 0.80 m (작업대 위)
  carry    : 손끝 ≈ 앞 0.32 m, 높이 0.92 m, 안쪽 0.10 m (양손 동시에 들어도 서로 안 닿음)
  hands_up : 손끝 ≈ 앞 0.38 m, 높이 0.83 m
"""
RIGHT = {
    "home": [0.1, 0.40, 0.0, 0.0, 0.0, 0.0, 0.0],
    "ready": [0.2, 0.20, 0.0, 2.0, 0.0, 0.0, 0.0],
    "carry": [0.3, 0.35, -0.1, 2.4, 0.0, 0.0, 0.0],
    "hands_up": [0.0, 0.30, 0.0, 2.3, 0.0, 0.0, 0.0],
}


def mirror(q):
    return [-q[0], -q[1], -q[2], q[3], -q[4], q[5], -q[6]]


POSES = {name: (mirror(q), list(q)) for name, q in RIGHT.items()}      # name → (왼팔, 오른팔)
HOME_ARM = {"left": POSES["home"][0], "right": POSES["home"][1]}
