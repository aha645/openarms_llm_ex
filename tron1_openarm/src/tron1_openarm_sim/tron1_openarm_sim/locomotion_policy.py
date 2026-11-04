"""TRON1 보행 정책 실행기 (numpy 전용, ROS·onnxruntime 불필요). Point-foot(PF) 과 Sole-foot(SF) 를 모두 지원.

LimX 공식 배포 코드(tron1-rl-deploy-python/controllers/PointfootController.py, SolefootController.py)와
LimX 학습 코드(tron1-rl-isaacgym, pointfoot_flat / solefoot_flat)의 관측(observation)·행동(action) 규약을 그대로 따른다.
정책 파일(npz)의 첫 층 입력 크기로 PF(36) / SF(44) 를 자동 판별한다.

Point-foot (PF_TRON1A, 다리 6관절, 발목 없음)
  관측 30 = 각속도(3)×0.25, 중력 방향(3), 관절각(6), 관절속도(6)×0.05, 직전 행동(6), 보행 시계 sin/cos(2), 보행 파라미터(4)
  인코더 입력 = 최근 관측 10개(300) → 출력 3 = 추정 몸통 선속도×2
  정책 입력 36 = 인코더(3) + 관측(30) + 명령(3: vx×2, vy×2, wz×0.25)
  PD kp 42, kd 3.5, 토크 ±80 Nm. 보행 2 Hz — 명령 0 이어도 항상 제자리 걸음

Sole-foot (SF_TRON1A, 다리 8관절 = PF + 발목 2, 평발)
  관측 36 = 각속도(3)×0.25, 중력(3), (관절각−기본각)(8), 관절속도(8)×0.05, 직전 행동(8), 시계(2), 보행 파라미터(4)
  인코더 입력 = 최근 관측 5개(180) → 3
  정책 입력 44 = 인코더(3) + 관측(36) + 명령(5: vx×2, vy×2, wz×0.25, 목표 몸통 높이, 제자리 서기 0/1)
  PD kp 45, kd 3 (발목 1.5), 토크 ±80 (발목 ±20). 보행 1.3 Hz
  '제자리 서기(stand_still)=1' 이면 두 발을 다 디딘 위상(0 또는 0.5)에서 보행 시계를 멈추고 발을 떼지 않는다
  (LimX solefoot_flat.py _resample_stand_still_gait_clock 과 같은 규칙) → 발목 토크로 버티며 가만히 선다
"""
import numpy as np

POLICY_DT = 0.02                     # 50 Hz


class Spec:
    def __init__(self, name, joints, default_q, kp, kd, tau, gait, hist, obs_dim, n_cmd):
        self.name, self.joints = name, joints
        self.default_q = np.array(default_q, np.float32)
        self.kp, self.kd, self.tau = (np.array(v, np.float32) for v in (kp, kd, tau))
        self.gait = np.array(gait, np.float32)
        self.hist, self.obs_dim, self.n_cmd = hist, obs_dim, n_cmd


PF = Spec("pf", ["abad_L_joint", "hip_L_joint", "knee_L_joint", "abad_R_joint", "hip_R_joint", "knee_R_joint"],
          [0] * 6, [42.0] * 6, [3.5] * 6, [80.0] * 6, [2.0, 0.5, 0.5, 0.1], hist=10, obs_dim=30, n_cmd=3)
SF = Spec("sf", ["abad_L_joint", "hip_L_joint", "knee_L_joint", "ankle_L_joint",
                 "abad_R_joint", "hip_R_joint", "knee_R_joint", "ankle_R_joint"],
          [0, 0.13, 0, 0, 0, -0.13, 0, 0], [45.0] * 8, [3, 3, 3, 1.5] * 2, [80, 80, 80, 20] * 2,
          [1.3, 0.5, 0.5, 0.12], hist=5, obs_dim=36, n_cmd=5)
SPECS = {36: PF, 44: SF}             # 정책 첫 층 입력 크기 → 규약

JOINTS = PF.joints                   # 예전 코드 호환 (PF)
ACTION_SCALE = 0.25
SF_BASE_HEIGHT_CMD = 0.73            # SF 명령의 목표 몸통 높이 [m] (LimX 학습 범위 0.68~0.78)


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
        self.spec = SPECS[self.p["policy_W0"].shape[0]]
        self.joints = self.spec.joints
        self.stand_still = False         # SF 전용: True 면 발을 떼지 않고 선다
        self.reset()

    def reset(self):
        self.hist = None
        self.last_action = np.zeros(len(self.joints), dtype=np.float32)
        self.gait_index = 0.0
        self.est_vel = np.zeros(3, dtype=np.float32)
        self._standing = False

    def _advance_clock(self):
        s = self.spec
        if s.name == "sf" and self.stand_still:
            g = self.gait_index
            if self._standing or g < 0.05 or g >= 0.95 or 0.5 <= g < 0.55:
                # 두 발이 모두 디딘 위상에 왔으면 시계를 그 자리(0 또는 0.5)에 멈춘다
                if not self._standing:
                    self.gait_index = 0.5 if 0.5 <= g < 0.55 else 0.0
                self._standing = True
                return
        else:
            self._standing = False
        self.gait_index = (self.gait_index + POLICY_DT * s.gait[0]) % 1.0

    @property
    def standing(self):
        """SF: 지금 발을 떼지 않고 서 있는 중인지 (시계가 멈췄는지)."""
        return self._standing

    def observation(self, gyro, quat_wxyz, q, dq):
        grav = quat_rotate_inverse(np.asarray(quat_wxyz, dtype=np.float64), np.array([0.0, 0.0, -1.0]))
        self._advance_clock()
        clock = [np.sin(2 * np.pi * self.gait_index), np.cos(2 * np.pi * self.gait_index)]
        return np.concatenate([np.asarray(gyro) * 0.25, grav, q - self.spec.default_q, np.asarray(dq) * 0.05,
                               self.last_action, clock, self.spec.gait]).astype(np.float32)

    def command_vector(self, cmd):
        vx, vy, wz = (float(c) for c in cmd)
        if self.spec.n_cmd == 3:
            return np.array([vx * 2.0, vy * 2.0, wz * 0.25], np.float32)
        still = 1.0 if (self.stand_still and self._standing) else 0.0
        if still:
            vx = vy = wz = 0.0
        return np.array([vx * 2.0, vy * 2.0, wz * 0.25, SF_BASE_HEIGHT_CMD, still], np.float32)

    def act(self, gyro, quat_wxyz, q, dq, cmd):
        """50Hz 마다 호출. cmd = (vx m/s, vy m/s, wz rad/s). 반환: 관절 목표각 (rad, 관절 순서 = self.joints)."""
        s = self.spec
        q = np.asarray(q, dtype=np.float32)
        dq = np.asarray(dq, dtype=np.float32)
        obs = np.clip(self.observation(gyro, quat_wxyz, q, dq), -100, 100)
        if self.hist is None:
            self.hist = np.tile(obs, s.hist)
        else:
            self.hist = np.concatenate([self.hist[s.obs_dim:], obs])
        self.est_vel = mlp(self.p, "encoder", self.hist)
        x = np.concatenate([self.est_vel, obs, self.command_vector(cmd)])
        a = np.clip(mlp(self.p, "policy", x), -100, 100)
        # 토크 한계를 넘지 않는 범위로 행동 제한 (LimX 배포 코드와 동일)
        qd = q - s.default_q
        lo = (qd + (s.kd * dq - s.tau) / s.kp) / ACTION_SCALE
        hi = (qd + (s.kd * dq + s.tau) / s.kp) / ACTION_SCALE
        a = np.clip(a, lo, hi).astype(np.float32)
        self.last_action = a
        return a * ACTION_SCALE + s.default_q

    @property
    def estimated_velocity(self):
        """인코더가 추정한 몸통 선속도 (m/s, 몸통 좌표)."""
        return self.est_vel / 2.0
