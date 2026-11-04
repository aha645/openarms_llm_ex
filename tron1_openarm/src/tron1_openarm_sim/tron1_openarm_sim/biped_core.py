"""TRON1(Point-foot) + OpenArm 시뮬레이션 코어 (ROS 비의존).

바퀴형 MobileOpenArmSim 을 상속해 '하반신'에 관계된 부분만 바꾼다.
상반신(팔 궤적·그리퍼·파지 보조)·LiDAR·카메라·ArUco·사람 장애물 코드는 부모 클래스 것을 그대로 쓴다.

바뀐 것                     바퀴형 (mobile_openarm_sim)            이족 (이 파일)
-------------------------  ------------------------------------  ---------------------------------------------
/cmd_vel → 움직임            v, w → 좌우 바퀴 속도 (기구학 공식)     v, w → 보행 정책(신경망, 50Hz) → 다리 6관절 목표각 → PD
명령이 없을 때               바퀴 정지 = 완전히 멈춤                  Point-foot 은 가만히 설 수 없음 → 제자리 걸음
오도메트리                   바퀴 회전각 적분                        다리 기구학(디딤발 고정 가정) + IMU 자세
base_footprint → base_link  바퀴 반지름 0.08 m 고정                  서 있는 높이 0.68 m (기본값, 실제는 ±2 cm 출렁임)
초기 자세                    바퀴 위에 놓기                          다리 편 자세로 0.71 m 에 놓고 정책이 받아서 섬
"""
import math
import os

import mujoco
import numpy as np

from mobile_openarm_sim import sim_core as W
from mobile_openarm_sim.aruco_utils import ArucoDetector
from mobile_openarm_sim.sim_core import ARM_JOINTS, GRIPPER_JOINT, MIMIC_JOINT, yaw_of

from .locomotion_policy import JOINTS as LEG_JOINTS
from .locomotion_policy import POLICY_DT, LocomotionPolicy

NOMINAL_BASE_HEIGHT = 0.68
SPAWN_HEIGHT = NOMINAL_BASE_HEIGHT + 0.03
FOOT_RADIUS = 0.032            # LimX const.xacro foot_radius
PUBLISHED_JOINTS = (ARM_JOINTS["left"] + [GRIPPER_JOINT["left"], MIMIC_JOINT["left"]]
                    + ARM_JOINTS["right"] + [GRIPPER_JOINT["right"], MIMIC_JOINT["right"]] + LEG_JOINTS)
# 이족용 home: 팔을 바깥으로 0.2 rad 벌림 (곧게 내리면 손이 TRON1 몸통 옆면에 닿음). SRDF 의 home 과 같은 값
HOME_ARM = {"left": [0.0, -0.20, 0.0, 0.0, 0.0, 0.0, 0.0], "right": [0.0, 0.20, 0.0, 0.0, 0.0, 0.0, 0.0]}
CMD_LIMIT = (0.5, 1.0)          # 정책이 학습한 범위 안으로 (vx m/s, wz rad/s)
CMD_ACCEL = (0.8, 2.0)          # 명령 변화율 제한 (m/s², rad/s²) — 이족은 급가속에 약함
HOLD_KP, HOLD_KW, HOLD_V = 1.0, 1.5, 0.15   # 제자리 유지 이득 (m/s per m, rad/s per rad), 최대 보정 속도
VEL_KP, VEL_KI = np.array([0.3, 0.5, 0.3]), np.array([0.8, 1.0, 0.8])   # 속도 외곽 루프 이득 (vx, vy, wz)
VEL_I_MAX = np.array([0.25, 0.25, 0.4])
CORR_MAX = np.array([0.2, 0.2, 0.4])                    # 외곽 루프가 정책 명령에 더할 수 있는 최대 보정


def default_policy_path():
    env = os.environ.get("TRON1_POLICY")
    if env:
        return env
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("tron1_openarm_sim"), "policies", "tron1_openarm_policy.npz")
    except Exception:  # noqa: BLE001
        return os.path.join(os.path.dirname(__file__), "..", "policies", "tron1_openarm_policy.npz")


class BipedOpenArmSim(W.MobileOpenArmSim):
    def __init__(self, xml_path, camera_width=640, camera_height=480, camera_hz=10.0,
                 enable_camera=True, lidar_rays=360, lidar_range=(0.12, 12.0), lidar_hz=10.0,
                 lidar_noise=0.005, grasp_assist=True, move_person=True, person_speed=0.4,
                 aruco_dict="DICT_4X4_50", marker_size=0.04, use_depth=True, policy_path=None):
        # ---- 부모 __init__ 중 하반신과 무관한 부분을 그대로 재현 (부모는 바퀴 관절을 찾으므로 직접 호출 불가)
        import threading
        self.m = mujoco.MjModel.from_xml_path(xml_path)
        self.d = mujoco.MjData(self.m)
        self.lock = threading.RLock()
        m = self.m
        self.base_id = m.body("base_link").id
        self.base_qadr = m.jnt_qposadr[m.joint("base_free").id]
        self.arm_qadr = {s: [m.jnt_qposadr[m.joint(n).id] for n in ARM_JOINTS[s]] for s in ARM_JOINTS}
        self.arm_act = {s: [m.actuator(f"{s}_joint{i}_ctrl").id for i in range(1, 8)] for s in ARM_JOINTS}
        self.grip_qadr = {s: m.jnt_qposadr[m.joint(GRIPPER_JOINT[s]).id] for s in ARM_JOINTS}
        self.grip_act = {s: m.actuator(f"{s}_finger1_ctrl").id for s in ARM_JOINTS}
        self.pub_qadr = [m.jnt_qposadr[m.joint(n).id] for n in PUBLISHED_JOINTS]
        self.pub_dadr = [m.jnt_dofadr[m.joint(n).id] for n in PUBLISHED_JOINTS]
        self.pub_range = [tuple(m.jnt_range[m.joint(n).id]) if m.jnt_limited[m.joint(n).id] else (-1e9, 1e9)
                          for n in PUBLISHED_JOINTS]
        self.pub_eff_act = {}
        for s in ARM_JOINTS:
            for i, n in enumerate(ARM_JOINTS[s]):
                self.pub_eff_act[n] = self.arm_act[s][i]
        # ---- 다리 / 보행 정책
        self.leg_qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in LEG_JOINTS])
        self.leg_dadr = np.array([m.jnt_dofadr[m.joint(n).id] for n in LEG_JOINTS])
        self.leg_act = np.array([m.actuator(n + "_ctrl").id for n in LEG_JOINTS])
        for i, n in enumerate(LEG_JOINTS):
            self.pub_eff_act[n] = self.leg_act[i]
        self.policy = LocomotionPolicy(policy_path or default_policy_path())
        self.policy_every = max(1, int(round(POLICY_DT / m.opt.timestep)))
        self._substep = 0
        self.foot_body = [m.body("foot_L_Link").id, m.body("foot_R_Link").id]
        self.foot_touch = [m.sensor("foot_L_touch").id, m.sensor("foot_R_touch").id]
        self.gyro_adr = m.sensor_adr[m.sensor("imu_gyro").id]
        self.quat_adr = m.sensor_adr[m.sensor("imu_quat").id]
        self.fallen = False
        self.hold_position = True      # 명령 0 일 때 제자리 유지 (False: 정책 그대로)
        self._hold = None
        self.cmd_ramp = np.zeros(2)
        self._vi = np.zeros(3)
        self.odom_vy = 0.0

        self._init_arms()
        self.cmd = (0.0, 0.0)
        self.cmd_eff = np.zeros(2)
        self.cmd_stamp = -1e9
        self.cmd_timeout = 0.5
        self.odom = np.zeros(3)
        self.odom_vel = np.zeros(2)
        self.traj = {"left": None, "right": None}
        self.grip_traj = {"left": None, "right": None}
        self.lidar_site = m.site("lidar_site").id
        self.lidar_rays = lidar_rays
        self.lidar_min, self.lidar_max = lidar_range
        self.lidar_period = 1.0 / lidar_hz
        self.lidar_noise = lidar_noise
        self._angles = np.linspace(-math.pi, math.pi, lidar_rays, endpoint=False)
        self._geomgroup = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)
        self._ray_geomid = np.zeros(lidar_rays, dtype=np.int32)
        self._ray_dist = np.zeros(lidar_rays, dtype=np.float64)
        self._last_lidar_t = -1e9
        self.last_scan = None
        self.enable_camera = enable_camera
        self.cam_id = m.camera("head_camera").id
        self.cam_w, self.cam_h = camera_width, camera_height
        self.cam_period = 1.0 / camera_hz
        self._last_cam_t = -1e9
        self.renderer = None
        fovy = math.radians(m.cam_fovy[self.cam_id])
        fy = (camera_height / 2) / math.tan(fovy / 2)
        self.K = np.array([[fy, 0, camera_width / 2], [0, fy, camera_height / 2], [0, 0, 1]])
        self.aruco = ArucoDetector(aruco_dict, marker_size, self.K)
        self.use_depth = use_depth
        self.last_depth = None
        self.last_image = None
        self.last_detections = []
        self.grasp_assist = grasp_assist
        self.attached = {"left": None, "right": None}
        self.box_bodies = [i for i in range(m.nbody) if m.body(i).name.startswith("box_")]
        self.tcp = {s: m.body(f"openarm_{s}_hand_tcp").id for s in ARM_JOINTS}
        self.move_person = move_person
        self.person_speed = person_speed
        self.person_mocap = m.body_mocapid[m.body("person").id] if self._has_body("person") else -1
        self._person_dir = 1.0
        self.wall_t0 = None
        self.set_base_pose(0.0, 0.0, 0.0)

    def _init_arms(self):
        for s in ARM_JOINTS:
            for qa, a, v in zip(self.arm_qadr[s], self.arm_act[s], HOME_ARM[s]):
                self.d.qpos[qa] = v
                self.d.ctrl[a] = v
            self.d.ctrl[self.grip_act[s]] = 0.044
        mujoco.mj_forward(self.m, self.d)

    # ------------------------------------------------------------------ 자세 / 초기화
    def set_base_pose(self, x, y, yaw):
        with self.lock:
            a = self.base_qadr
            self.d.qpos[a:a + 3] = [x, y, SPAWN_HEIGHT]
            self.d.qpos[a + 3:a + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
            self.d.qpos[self.leg_qadr] = 0.0
            self.d.qvel[:] = 0
            self.d.ctrl[self.leg_act] = 0.0
            mujoco.mj_forward(self.m, self.d)
            self.policy.reset()
            self._substep = 0
            self.fallen = False
            self.cmd_eff[:] = 0
            self._hold = None
            self.cmd_ramp = np.zeros(2)
            self._vi = np.zeros(3)
            self._odom_reset()

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        with self.lock:
            mujoco.mj_resetData(self.m, self.d)
            self._init_arms()
            self.traj = {"left": None, "right": None}
            self.grip_traj = {"left": None, "right": None}
            self.attached = {"left": None, "right": None}
            self.cmd = (0.0, 0.0)
            self.set_base_pose(x, y, yaw)
            self._last_lidar_t = self._last_cam_t = -1e9

    def ground_truth_pose(self):
        q = self.d.qpos[self.base_qadr:self.base_qadr + 7]
        return np.array([q[0], q[1], yaw_of(q[3:7])])

    # ------------------------------------------------------------------ 하반신 제어
    def _apply_base(self):
        """물리 스텝마다 호출. policy_every 스텝(=20 ms)마다 보행 정책으로 다리 목표각을 새로 계산."""
        if self._substep % self.policy_every == 0:
            v, w = self.cmd
            if self.d.time - self.cmd_stamp > self.cmd_timeout:
                v, w = 0.0, 0.0
            v = float(np.clip(v, -CMD_LIMIT[0] * 0.7, CMD_LIMIT[0]))
            w = float(np.clip(w, -CMD_LIMIT[1], CMD_LIMIT[1]))
            dv = np.clip(np.array([v, w]) - self.cmd_ramp, -np.array(CMD_ACCEL) * POLICY_DT,
                         np.array(CMD_ACCEL) * POLICY_DT)
            self.cmd_ramp += dv
            d = self.d
            vy = 0.0
            # 자세 추종(제자리 유지 포함): 명령 속도를 적분한 '기준 자세'를 만들고, 오도메트리 자세가 그것을 따라가게 한다.
            #  - 명령 0 → 기준 자세가 멈춤 = 그 자리 유지 (Point-foot 은 명령 0 이어도 제자리 걸음을 하며 흐르므로)
            #  - 제자리 회전·느린 직진(도킹) → 옆으로 미끄러지지 않고 명령대로 이동
            #  - 주의: Nav2 MPPI 는 '측정 속도 + 가속 한계×0.05 s' 만큼씩만 명령을 키운다. 작은 명령(0.04 m/s)을 정지로
            #    무시하면 로봇이 안 움직이고 → 측정 속도 0 → 명령도 0.04 에 머무는 교착이 생긴다 (실제로 겪음) → 무시하지 않는다
            if self._hold is None:
                self._hold = self.odom.copy()
            ref = self._hold
            if self.hold_position:
                yr = ref[2]
                ref[0] += self.cmd_ramp[0] * math.cos(yr) * POLICY_DT
                ref[1] += self.cmd_ramp[0] * math.sin(yr) * POLICY_DT
                ref[2] = math.atan2(math.sin(yr + self.cmd_ramp[1] * POLICY_DT), math.cos(yr + self.cmd_ramp[1] * POLICY_DT))
                ex, ey = self.odom[0] - ref[0], self.odom[1] - ref[1]
                c, s = math.cos(self.odom[2]), math.sin(self.odom[2])
                bx, by = c * ex + s * ey, -s * ex + c * ey            # 몸통 좌표의 위치 오차
                eyaw = math.atan2(math.sin(self.odom[2] - ref[2]), math.cos(self.odom[2] - ref[2]))
                if math.hypot(bx, by) > 0.4 or abs(eyaw) > 0.6:    # 크게 벗어나면(부딪힘 등) 기준을 현재 자세로 다시 잡음
                    self._hold = self.odom.copy()
                    bx = by = eyaw = 0.0
                want = np.array([self.cmd_ramp[0] + np.clip(-HOLD_KP * bx, -HOLD_V, HOLD_V),
                                 np.clip(-HOLD_KP * by, -HOLD_V, HOLD_V),
                                 self.cmd_ramp[1] + np.clip(-HOLD_KW * eyaw, -0.3, 0.3)])
            else:
                want = np.array([self.cmd_ramp[0], 0.0, self.cmd_ramp[1]])
            # 속도 외곽 루프(PI): 목표 속도와 측정 속도(다리 오도메트리)의 차이로 정책 명령을 보정.
            # 정책 고유의 치우침(느림·옆 흐름·회전 흐름, 제자리에서의 흐름)을 적분항이 상쇄한다.
            meas = np.array([self.odom_vel[0], self.odom_vy, self.odom_vel[1]])
            err = want - meas
            self._vi = np.clip(self._vi + VEL_KI * err * POLICY_DT, -VEL_I_MAX, VEL_I_MAX)
            out = want + VEL_KP * err + self._vi
            # 보정량 상한: 팔 동작 반동 등으로 오차가 커져도 몸통이 달려 나가지 않도록
            out = np.clip(out, want - CORR_MAX, want + CORR_MAX)
            self.cmd_eff[:] = [float(np.clip(out[0], -0.45, 0.6)), float(np.clip(out[2], -1.2, 1.2))]
            vy = float(np.clip(out[1], -0.25, 0.25))
            gyro = d.sensordata[self.gyro_adr:self.gyro_adr + 3]
            quat = d.sensordata[self.quat_adr:self.quat_adr + 4]
            target = self.policy.act(gyro, quat, d.qpos[self.leg_qadr], d.qvel[self.leg_dadr],
                                     (self.cmd_eff[0], vy, self.cmd_eff[1]))
            d.ctrl[self.leg_act] = target
            if d.xpos[self.base_id][2] < 0.42 and not self.fallen:
                self.fallen = True
        self._substep += 1

    # ------------------------------------------------------------------ 다리 오도메트리
    def _odom_reset(self):
        d = self.d
        self._est_xy = d.xpos[self.base_id][:2].copy()          # 부팅 시점 위치 = odom 원점 (값 자체는 쓰지 않음)
        self._origin_xy = self._est_xy.copy()
        self._origin_yaw = yaw_of(d.xquat[self.base_id])
        self._stance = None
        self._anchor = None
        self._last_est = self._est_xy.copy()
        self._last_yaw = 0.0
        self.odom[:] = 0
        self.odom_vel[:] = 0
        self._vel_hist = None

    def _update_odom(self):
        """디딤발(접지력이 큰 발)은 땅에 고정돼 있다고 보고, 다리 기구학으로 몸통 위치를 거꾸로 계산.
        실로봇에서도 관절각(엔코더)·IMU 자세·발 접지 판정만으로 같은 계산을 한다 (여기선 MuJoCo 가 FK 를 계산)."""
        d = self.d
        # 몸통→발 벡터(세계 축) = R_imu · FK(관절각).  MuJoCo 가 이미 계산한 xpos 차이가 바로 그 값.
        # (실로봇 IMU 의 yaw 는 천천히 흐를 수 있으나 여기서는 IMU 자세를 정확하다고 둔다)
        rel = [d.xpos[b] - d.xpos[self.base_id] for b in self.foot_body]
        F = [d.sensordata[self.m.sensor_adr[s]] for s in self.foot_touch]
        st = int(np.argmax(F)) if max(F) > 20.0 else None
        t = d.time
        if st is not None:
            if st != self._stance or self._anchor is None:
                self._stance = st
                self._anchor = self._est_xy + rel[st][:2]        # 새로 디딘 발 '공 중심'의 (추정) 세계 위치
            else:
                # 공 모양 발은 디딘 채로 굴러간다: 접점은 그대로여도 공 중심은 r·(발 회전)만큼 앞으로 이동.
                # 이걸 무시하면 거리가 약 10% 짧게 나온다 (발 반지름 3.2 cm, 한 걸음에 정강이가 ~0.2 rad 회전)
                w = np.zeros(6)
                mujoco.mj_objectVelocity(self.m, d, mujoco.mjtObj.mjOBJ_BODY, self.foot_body[st], w, 0)
                dt_r = t - getattr(self, "_last_roll_t", t)
                self._anchor = self._anchor + FOOT_RADIUS * np.array([w[1], -w[0]]) * dt_r
            self._est_xy = self._anchor - rel[st][:2]
        self._last_roll_t = t
        yaw = yaw_of(d.xquat[self.base_id]) - self._origin_yaw
        c, s = math.cos(-self._origin_yaw), math.sin(-self._origin_yaw)
        dxy = self._est_xy - self._origin_xy
        self.odom[0] = c * dxy[0] - s * dxy[1]
        self.odom[1] = s * dxy[0] + c * dxy[1]
        self.odom[2] = math.atan2(math.sin(yaw), math.cos(yaw))
        # 속도: 보행 한 주기(0.5 s) 동안의 평균. 디딜 때마다 몸통이 앞뒤·좌우로 흔들려 짧은 구간 미분은
        # ±0.2 m/s·±0.5 rad/s 로 출렁이고, Nav2 MPPI 는 이 값을 현재 속도로 써서 제어를 거의 0 으로 낸다.
        hist = getattr(self, "_vel_hist", None)
        if hist is None or d.time < hist[-1][0]:
            hist = self._vel_hist = []
        hist.append((d.time, self.odom[0], self.odom[1], self.odom[2]))
        while len(hist) > 2 and d.time - hist[0][0] > 0.5:
            hist.pop(0)
        T = hist[-1][0] - hist[0][0]
        if T > 0.1:
            dx, dy = hist[-1][1] - hist[0][1], hist[-1][2] - hist[0][2]
            dyaw = math.atan2(math.sin(hist[-1][3] - hist[0][3]), math.cos(hist[-1][3] - hist[0][3]))
            c, s = math.cos(self.odom[2]), math.sin(self.odom[2])
            self.odom_vel[:] = [(c * dx + s * dy) / T, dyaw / T]
            self.odom_vy = (-s * dx + c * dy) / T

    # ------------------------------------------------------------------ 상태
    def joint_state(self):
        d = self.d
        pos = [float(min(max(d.qpos[a], lo), hi)) for a, (lo, hi) in zip(self.pub_qadr, self.pub_range)]
        vel = [float(d.qvel[a]) for a in self.pub_dadr]
        eff = [float(d.actuator_force[self.pub_eff_act[n]]) if n in self.pub_eff_act else 0.0
               for n in PUBLISHED_JOINTS]
        return list(PUBLISHED_JOINTS), pos, vel, eff

    def base_height_and_tilt(self):
        R = self.d.xmat[self.base_id].reshape(3, 3)
        return float(self.d.xpos[self.base_id][2]), float(math.degrees(math.acos(min(1.0, R[2, 2]))))
