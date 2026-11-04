"""TRON1 Point-foot 보행 정책 실행기 (numpy 전용, ROS·onnxruntime 불필요).

LimX 공식 배포 코드(tron1-rl-deploy-python/controllers/PointfootController.py)와
같은 관측(observation)·행동(action) 규약을 그대로 따른다. 그래서
  * LimX 가 Isaac Gym 으로 학습해 공개한 정책(PF_TRON1A, 사전학습)과
  * 이 저장소의 rl/ 에서 OpenArm 상반신을 얹고 PPO 로 미세조정(fine-tune)한 정책
을 같은 코드로 돌릴 수 있다.

관측 30개 = 각속도(3)×0.25, 중력 방향(3), 관절각(6), 관절속도(6)×0.05, 직전 행동(6), 보행 시계 sin/cos(2), 보행 파라미터(4)
인코더 입력 = 최근 관측 10개(300) → 출력 3 = 추정 몸통 선속도×2 (학습 때 정답 속도로 지도학습된 추정기)
정책 입력 36 = 인코더 출력(3) + 관측(30) + 명령(3: vx×2, vy×2, wz×0.25)
정책 출력 6 = 관절 목표각 오프셋 / 0.25 → PD(kp 42, kd 3.5) 로 토크
"""
import numpy as np

JOINTS = ["abad_L_joint", "hip_L_joint", "knee_L_joint", "abad_R_joint", "hip_R_joint", "knee_R_joint"]   # xacro 이름 (LimX MJCF 는 *_Joint)
KP, KD = 42.0, 3.5
ACTION_SCALE = 0.25
TORQUE_LIMIT = 80.0
POLICY_DT = 0.02                     # 50 Hz
GAIT = np.array([2.0, 0.5, 0.5, 0.1], dtype=np.float32)   # 주파수 2Hz, 좌우 위상차 0.5, 접지비 0.5, 발 높이 0.1m
CMD_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
OBS_DIM, HIST = 30, 10


def elu(x):
    return np.where(x > 0, x, np.expm1(np.minimum(x, 0)))


def mlp(params, prefix, x):
    i = 0
    while f"{prefix}_W{i}" in params:
        x = x @ params[f"{prefix}_W{i}"] + params[f"{prefix}_b{i}"]
        if f"{prefix}_W{i + 1}" in params:
            x = elu(x)
        i += 1
    return x


def quat_rotate_inverse(q_wxyz, v):
    w, x, y, z = q_wxyz
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R.T @ v


class LocomotionPolicy:
    def __init__(self, npz_path):
        d = np.load(npz_path)
        self.p = {k: d[k] for k in d.files}
        self.reset()

    def reset(self):
        self.hist = None
        self.last_action = np.zeros(6, dtype=np.float32)
        self.gait_index = 0.0
        self.est_vel = np.zeros(3, dtype=np.float32)

    def observation(self, gyro, quat_wxyz, q, dq):
        grav = quat_rotate_inverse(np.asarray(quat_wxyz, dtype=np.float64), np.array([0.0, 0.0, -1.0]))
        self.gait_index = (self.gait_index + POLICY_DT * GAIT[0]) % 1.0
        clock = [np.sin(2 * np.pi * self.gait_index), np.cos(2 * np.pi * self.gait_index)]
        return np.concatenate([np.asarray(gyro) * 0.25, grav, q, np.asarray(dq) * 0.05,
                               self.last_action, clock, GAIT]).astype(np.float32)

    def act(self, gyro, quat_wxyz, q, dq, cmd):
        """50Hz 마다 호출. cmd = (vx m/s, vy m/s, wz rad/s). 반환: 관절 목표각 6개(rad)."""
        q = np.asarray(q, dtype=np.float32)
        dq = np.asarray(dq, dtype=np.float32)
        obs = np.clip(self.observation(gyro, quat_wxyz, q, dq), -100, 100)
        if self.hist is None:
            self.hist = np.tile(obs, HIST)
        else:
            self.hist = np.concatenate([self.hist[OBS_DIM:], obs])
        self.est_vel = mlp(self.p, "encoder", self.hist)
        x = np.concatenate([self.est_vel, obs, np.asarray(cmd, dtype=np.float32) * CMD_SCALE])
        a = np.clip(mlp(self.p, "policy", x), -100, 100)
        # 토크 한계를 넘지 않는 범위로 행동 제한 (LimX 배포 코드와 동일)
        lo = (q + (KD * dq - TORQUE_LIMIT) / KP) / ACTION_SCALE
        hi = (q + (KD * dq + TORQUE_LIMIT) / KP) / ACTION_SCALE
        a = np.clip(a, lo, hi).astype(np.float32)
        self.last_action = a
        return a * ACTION_SCALE

    @property
    def estimated_velocity(self):
        """인코더가 추정한 몸통 선속도 (m/s, 몸통 좌표)."""
        return self.est_vel / 2.0
