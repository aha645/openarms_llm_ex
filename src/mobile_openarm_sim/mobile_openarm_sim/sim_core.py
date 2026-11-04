"""MuJoCo 시뮬레이션 코어 (ROS 비의존).

bridge_node.py 가 이 클래스를 감싸서 ROS 2 토픽/액션으로 노출한다.
ROS 없이도 tools/selftest.py 로 주행·LiDAR·카메라·ArUco·파지를 검증할 수 있다.

기능
  * 차동구동: set_cmd_vel(v, w) → 좌/우 바퀴 속도 actuator
  * 휠 오도메트리 (바퀴 각도 적분) + ground-truth 포즈
  * 2D LiDAR (mj_multiRay, 월드 geom group 0/1 만 감지 → 로봇 자기 몸체 무시)
  * 머리 RGB 카메라 오프스크린 렌더 + ArUco 검출/포즈 추정 (검출 결과만 밖으로)
  * 관절 궤적 실행 (FollowJointTrajectory 와 동일한 의미: 시간-위치 보간)
  * 그리퍼 + "파지 보조(grasp assist)" weld 구속 (교육용 옵션)
  * 움직이는 사람(mocap) 장애물
"""
import math
import threading
import time

import mujoco
import numpy as np

from .aruco_utils import ArucoDetector

ARM_JOINTS = {
    side: [f"openarm_{side}_joint{i}" for i in range(1, 8)] for side in ("left", "right")
}
GRIPPER_JOINT = {side: f"openarm_{side}_finger_joint1" for side in ("left", "right")}
MIMIC_JOINT = {side: f"openarm_{side}_finger_joint2" for side in ("left", "right")}
WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]
# ROS 로 내보낼 관절 순서 (URDF 와 동일 이름)
PUBLISHED_JOINTS = (ARM_JOINTS["left"] + [GRIPPER_JOINT["left"], MIMIC_JOINT["left"]]
                    + ARM_JOINTS["right"] + [GRIPPER_JOINT["right"], MIMIC_JOINT["right"]]
                    + WHEEL_JOINTS)

HOME_POSE = [0.0] * 7
WHEEL_RADIUS = 0.08
WHEEL_SEPARATION = 0.44


def yaw_of(quat_wxyz):
    w, x, y, z = quat_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Trajectory:
    """시간 기준 관절 궤적. points: [(t, positions[, velocities])]. 3차 Hermite 보간."""

    def __init__(self, joint_ids_qadr, actuator_ids, points, start_q, t0):
        self.qadr = joint_ids_qadr
        self.act = actuator_ids
        self.t0 = t0
        pts = list(points)
        if not pts or pts[0][0] > 1e-6:
            pts.insert(0, (0.0, list(start_q), [0.0] * len(start_q)))
        self.t = np.array([p[0] for p in pts])
        self.q = np.array([p[1] for p in pts], dtype=float)
        v = []
        for p in pts:
            v.append(p[2] if len(p) > 2 and p[2] is not None and len(p[2]) == len(p[1]) else None)
        if any(x is None for x in v):
            # 속도가 없으면 유한차분으로 추정, 끝점 0
            vv = np.zeros_like(self.q)
            for i in range(1, len(self.t) - 1):
                dt = self.t[i + 1] - self.t[i - 1]
                vv[i] = (self.q[i + 1] - self.q[i - 1]) / max(dt, 1e-6)
            self.v = vv
        else:
            self.v = np.array(v, dtype=float)
        self.done = False
        self.canceled = False
        self.error = None

    @property
    def duration(self):
        return float(self.t[-1])

    def sample(self, sim_t):
        tau = sim_t - self.t0
        if tau >= self.t[-1]:
            return self.q[-1], True
        i = int(np.searchsorted(self.t, tau, side="right") - 1)
        i = max(0, min(i, len(self.t) - 2))
        h = self.t[i + 1] - self.t[i]
        s = (tau - self.t[i]) / max(h, 1e-9)
        h00 = 2 * s ** 3 - 3 * s ** 2 + 1
        h10 = s ** 3 - 2 * s ** 2 + s
        h01 = -2 * s ** 3 + 3 * s ** 2
        h11 = s ** 3 - s ** 2
        q = h00 * self.q[i] + h10 * h * self.v[i] + h01 * self.q[i + 1] + h11 * h * self.v[i + 1]
        return q, False


class MobileOpenArmSim:
    def __init__(self, xml_path, camera_width=640, camera_height=480, camera_hz=10.0,
                 enable_camera=True, lidar_rays=360, lidar_range=(0.12, 12.0), lidar_hz=10.0,
                 lidar_noise=0.005, grasp_assist=True, move_person=True, person_speed=0.4,
                 aruco_dict="DICT_4X4_50", marker_size=0.04, use_depth=True):
        self.m = mujoco.MjModel.from_xml_path(xml_path)
        self.d = mujoco.MjData(self.m)
        self.lock = threading.RLock()
        m = self.m

        self.base_id = m.body("base_link").id
        self.base_qadr = m.jnt_qposadr[m.joint("base_free").id]
        self.wheel_qadr = [m.jnt_qposadr[m.joint(n).id] for n in WHEEL_JOINTS]
        self.wheel_dadr = [m.jnt_dofadr[m.joint(n).id] for n in WHEEL_JOINTS]
        self.wheel_act = [m.actuator("left_wheel_vel").id, m.actuator("right_wheel_vel").id]
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

        self._init_arms()

        # 차동구동/오도메트리
        self.cmd = (0.0, 0.0)
        self.cmd_stamp = -1e9
        self.cmd_timeout = 0.5
        self.odom = np.zeros(3)  # x, y, yaw
        self.odom_vel = np.zeros(2)
        self._last_wheel = np.array([self.d.qpos[a] for a in self.wheel_qadr])
        self._gt0 = self.ground_truth_pose()

        # 궤적
        self.traj = {"left": None, "right": None}
        self.grip_traj = {"left": None, "right": None}

        # LiDAR
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

        # 카메라
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

        # 파지 보조 (weld 구속은 런타임에 eq 를 쓰지 않고 '정적 부착'으로 구현: 상자 freejoint 를 손에 고정)
        self.grasp_assist = grasp_assist
        self.attached = {"left": None, "right": None}  # (box_body_id, rel_pos, rel_quat)
        self.box_bodies = [i for i in range(m.nbody) if m.body(i).name.startswith("box_")]
        self.tcp = {s: m.body(f"openarm_{s}_hand_tcp").id for s in ARM_JOINTS}

        # 사람
        self.move_person = move_person
        self.person_speed = person_speed
        self.person_mocap = m.body_mocapid[m.body("person").id] if self._has_body("person") else -1
        self._person_dir = 1.0

        self.wall_t0 = None

    def _init_arms(self):
        """초기 자세: 팔 내림(home), 그리퍼 열림."""
        for s in ARM_JOINTS:
            for qa, a, v in zip(self.arm_qadr[s], self.arm_act[s], HOME_POSE):
                self.d.qpos[qa] = v
                self.d.ctrl[a] = v
            self.d.ctrl[self.grip_act[s]] = 0.044
        mujoco.mj_forward(self.m, self.d)

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        """월드/로봇/상자 전체 초기화 (시뮬레이션 시간도 0 으로)."""
        with self.lock:
            mujoco.mj_resetData(self.m, self.d)
            self._init_arms()
            self.traj = {"left": None, "right": None}
            self.grip_traj = {"left": None, "right": None}
            self.attached = {"left": None, "right": None}
            self.cmd = (0.0, 0.0)
            self.set_base_pose(x, y, yaw)
            self._last_wheel = np.array([self.d.qpos[a] for a in self.wheel_qadr])
            self._last_lidar_t = self._last_cam_t = -1e9

    # ------------------------------------------------------------------ utils
    def _has_body(self, name):
        try:
            self.m.body(name)
            return True
        except KeyError:
            return False

    @property
    def time(self):
        return self.d.time

    def ground_truth_pose(self):
        q = self.d.qpos[self.base_qadr:self.base_qadr + 7]
        return np.array([q[0], q[1], yaw_of(q[3:7])])

    def set_base_pose(self, x, y, yaw):
        with self.lock:
            a = self.base_qadr
            self.d.qpos[a:a + 3] = [x, y, WHEEL_RADIUS + 0.002]
            self.d.qpos[a + 3:a + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
            self.d.qvel[:6] = 0
            mujoco.mj_forward(self.m, self.d)
            self._gt0 = self.ground_truth_pose()
            self.odom[:] = 0

    # ------------------------------------------------------------------ commands
    def set_cmd_vel(self, v, w):
        with self.lock:
            self.cmd = (float(v), float(w))
            self.cmd_stamp = self.d.time

    def arm_positions(self, side):
        return [float(self.d.qpos[a]) for a in self.arm_qadr[side]]

    def start_arm_trajectory(self, side, joint_names, points):
        """points: [(t, [q...], [v...] or None)] — joint_names 순서를 내부 순서로 재배열."""
        with self.lock:
            order = [joint_names.index(n) for n in ARM_JOINTS[side]]
            pts = []
            for t, q, v in points:
                pts.append((t, [q[i] for i in order], [v[i] for i in order] if v else None))
            tr = Trajectory(self.arm_qadr[side], self.arm_act[side], pts,
                            [self.d.ctrl[a] for a in self.arm_act[side]], self.d.time)
            old = self.traj[side]
            if old is not None and not old.done:
                old.canceled = True
            self.traj[side] = tr
            return tr

    def start_gripper_trajectory(self, side, points):
        """points: [(t, [finger_joint1], v)] 0.0(닫힘) ~ 0.044(열림)."""
        with self.lock:
            tr = Trajectory([self.grip_qadr[side]], [self.grip_act[side]],
                            [(t, [q[0]], None) for t, q, _ in points],
                            [self.d.ctrl[self.grip_act[side]]], self.d.time)
            self.grip_traj[side] = tr
            return tr

    def cancel_trajectory(self, side, gripper=False):
        with self.lock:
            tr = (self.grip_traj if gripper else self.traj)[side]
            if tr is not None:
                tr.canceled = True
                tr.done = True
                if not gripper:   # 현재 위치 유지
                    for qa, a in zip(self.arm_qadr[side], self.arm_act[side]):
                        self.d.ctrl[a] = self.d.qpos[qa]

    def gripper_position(self, side):
        return float(self.d.qpos[self.grip_qadr[side]])

    # ------------------------------------------------------------------ step
    def step(self, n=1):
        with self.lock:
            m, d = self.m, self.d
            for _ in range(n):
                self._apply_base()
                self._apply_trajectories()
                self._move_person()
                mujoco.mj_step(m, d)
                self._update_attached()
            self._update_odom()

    def _apply_base(self):
        v, w = self.cmd
        if self.d.time - self.cmd_stamp > self.cmd_timeout:
            v, w = 0.0, 0.0
        wl = (v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        wr = (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        self.d.ctrl[self.wheel_act[0]] = wl
        self.d.ctrl[self.wheel_act[1]] = wr

    def _apply_trajectories(self):
        t = self.d.time
        for side in ("left", "right"):
            tr = self.traj[side]
            if tr is not None and not tr.done:
                q, fin = tr.sample(t)
                for a, val in zip(tr.act, q):
                    lo, hi = self.m.actuator_ctrlrange[a]
                    self.d.ctrl[a] = min(max(val, lo), hi)
                if fin:
                    tr.done = True
            gt = self.grip_traj[side]
            if gt is not None and not gt.done:
                q, fin = gt.sample(t)
                self.d.ctrl[gt.act[0]] = float(np.clip(q[0], -0.005, 0.044))
                if fin:
                    gt.done = True

    def _move_person(self):
        if not self.move_person or self.person_mocap < 0:
            return
        p = self.d.mocap_pos[self.person_mocap]
        p[0] += self._person_dir * self.person_speed * self.m.opt.timestep
        if p[0] > 3.5:
            self._person_dir = -1.0
        elif p[0] < -3.5:
            self._person_dir = 1.0

    def _update_odom(self):
        wheel = np.array([self.d.qpos[a] for a in self.wheel_qadr])
        dl, dr = (wheel - self._last_wheel) * WHEEL_RADIUS
        self._last_wheel = wheel
        ds = (dl + dr) / 2
        dth = (dr - dl) / WHEEL_SEPARATION
        th = self.odom[2]
        self.odom[0] += ds * math.cos(th + dth / 2)
        self.odom[1] += ds * math.sin(th + dth / 2)
        self.odom[2] = math.atan2(math.sin(th + dth), math.cos(th + dth))
        wl, wr = (self.d.qvel[a] * WHEEL_RADIUS for a in self.wheel_dadr)
        self.odom_vel[:] = [(wl + wr) / 2, (wr - wl) / WHEEL_SEPARATION]

    # ------------------------------------------------------------------ grasp assist
    def _update_attached(self):
        """파지 보조: 그리퍼가 닫혀 있고 상자가 손가락 사이에 있으면 상자를 TCP 에 고정."""
        if not self.grasp_assist:
            return
        m, d = self.m, self.d
        for side in ("left", "right"):
            target = d.ctrl[self.grip_act[side]]
            tcp = self.tcp[side]
            att = self.attached[side]
            if att is None:
                if target < 0.03:  # 닫는 중
                    for b in self.box_bodies:
                        if any(a is not None and a[0] == b for a in self.attached.values()):
                            continue
                        rel = d.xmat[tcp].reshape(3, 3).T @ (d.xpos[b] - d.xpos[tcp])
                        # TCP 기준 상자 중심이 손가락 사이(±3cm)에 있고, 손가락이 상자에 닿았을 때
                        if np.all(np.abs(rel) < [0.035, 0.035, 0.04]) and self._fingers_touch(side, b):
                            qrel = np.zeros(4)
                            qinv = np.zeros(4)
                            mujoco.mju_negQuat(qinv, d.xquat[tcp])
                            mujoco.mju_mulQuat(qrel, qinv, d.xquat[b])
                            self.attached[side] = (b, rel, qrel)
                            break
            else:
                if target > 0.03:  # 열면 해제
                    self.attached[side] = None
                    continue
                b, rel, qrel = att
                jadr = m.jnt_qposadr[m.body_jntadr[b]]
                dadr = m.jnt_dofadr[m.body_jntadr[b]]
                pos = d.xpos[tcp] + d.xmat[tcp].reshape(3, 3) @ rel
                quat = np.zeros(4)
                mujoco.mju_mulQuat(quat, d.xquat[tcp], qrel)
                d.qpos[jadr:jadr + 3] = pos
                d.qpos[jadr + 3:jadr + 7] = quat
                d.qvel[dadr:dadr + 6] = 0

    def _fingers_touch(self, side, box_body):
        m, d = self.m, self.d
        names = {f"openarm_{side}_right_finger", f"openarm_{side}_left_finger"}
        touched = set()
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            for a, b in ((b1, b2), (b2, b1)):
                if b == box_body and m.body(a).name in names:
                    touched.add(m.body(a).name)
        return len(touched) >= 1

    def object_poses(self):
        """상자들의 world 위치 (정답값, 디버깅/채점용)."""
        return {self.m.body(b).name: [round(float(v), 4) for v in self.d.xpos[b]] for b in self.box_bodies}

    def attached_object(self, side):
        a = self.attached[side]
        return None if a is None else self.m.body(a[0]).name

    # ------------------------------------------------------------------ sensors
    def joint_state(self):
        d = self.d
        # 위치제어 오버슈트로 관절이 한계를 0.001rad 정도 넘으면 MoveIt 이 'Start state out of bounds' 로 거부 → 한계 안으로 clamp
        pos = [float(min(max(d.qpos[a], lo), hi)) for a, (lo, hi) in zip(self.pub_qadr, self.pub_range)]
        vel = [float(d.qvel[a]) for a in self.pub_dadr]
        eff = [float(d.actuator_force[self.pub_eff_act[n]]) if n in self.pub_eff_act else 0.0
               for n in PUBLISHED_JOINTS]
        return list(PUBLISHED_JOINTS), pos, vel, eff

    def imu(self):
        d, m = self.d, self.m
        g = d.sensor("imu_gyro").data.copy()
        a = d.sensor("imu_acc").data.copy()
        q = d.sensor("imu_quat").data.copy()
        return q, g, a

    def maybe_scan(self):
        """lidar 주기가 되면 (angle_min, angle_inc, ranges) 반환, 아니면 None."""
        if self.d.time - self._last_lidar_t < self.lidar_period:
            return None
        self._last_lidar_t = self.d.time
        with self.lock:
            d = self.d
            pnt = d.site_xpos[self.lidar_site].copy()
            R = d.site_xmat[self.lidar_site].reshape(3, 3)
            yaw = math.atan2(R[1, 0], R[0, 0])
            ang = self._angles + yaw
            vec = np.stack([np.cos(ang), np.sin(ang), np.zeros_like(ang)], 1).reshape(-1)
            self._ray_dist[:] = -1
            try:
                mujoco.mj_multiRay(self.m, d, pnt, vec, self._geomgroup, 1, self.base_id,
                                   self._ray_geomid, self._ray_dist, None, self.lidar_rays, self.lidar_max)
            except TypeError:  # MuJoCo < 3.3 (normal 인자 없음)
                mujoco.mj_multiRay(self.m, d, pnt, vec, self._geomgroup, 1, self.base_id,
                                   self._ray_geomid, self._ray_dist, self.lidar_rays, self.lidar_max)
        r = self._ray_dist.copy()
        r[(r < 0) | (r > self.lidar_max)] = np.inf
        if self.lidar_noise > 0:
            finite = np.isfinite(r)
            r[finite] += np.random.normal(0, self.lidar_noise, finite.sum())
        r[r < self.lidar_min] = np.inf
        self.last_scan = (-math.pi, 2 * math.pi / self.lidar_rays, r)
        return self.last_scan

    def maybe_camera(self):
        """카메라 주기가 되면 (rgb, detections) 반환. detections: list of dict."""
        if not self.enable_camera or self.d.time - self._last_cam_t < self.cam_period:
            return None
        self._last_cam_t = self.d.time
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.m, self.cam_h, self.cam_w)
        with self.lock:
            self.renderer.update_scene(self.d, camera=self.cam_id)
            self._camera_scene_hook(self.renderer.scene)
            T_base_opt = self.camera_optical_in_base()
        rgb = self.renderer.render().copy()
        dets = self.aruco.detect(rgb)
        if dets and self.use_depth:
            # RGB-D 카메라 흉내: 깊이 영상으로 마커 중심 3D 위치 보정
            # (640x480 에서 4cm 마커는 ~23px 라 PnP 깊이 오차가 5~15% 로 큼)
            self.renderer.enable_depth_rendering()
            self.renderer.update_scene(self.d, camera=self.cam_id)
            self._camera_scene_hook(self.renderer.scene)
            depth = self.renderer.render().copy()
            self.renderer.disable_depth_rendering()
            self.last_depth = depth
            Kinv = np.linalg.inv(self.K)
            for det in dets:
                u, v = det["corners"].mean(0)
                ui, vi = int(round(u)), int(round(v))
                if 0 <= vi < depth.shape[0] and 0 <= ui < depth.shape[1]:
                    z = float(np.median(depth[max(vi - 1, 0):vi + 2, max(ui - 1, 0):ui + 2]))
                    if 0.05 < z < 5.0:
                        det["position_pnp"] = det["position"]
                        det["position"] = z * (Kinv @ np.array([u, v, 1.0]))
        for det in dets:
            p = T_base_opt[:3, :3] @ det["position"] + T_base_opt[:3, 3]
            R = T_base_opt[:3, :3] @ det["rotation"]
            det["position_base"] = p
            det["rotation_base"] = R
        self.last_image = rgb
        self.last_detections = dets
        return rgb, dets

    def _camera_scene_hook(self, scene):
        """카메라 렌더 직전 장면 설정을 바꿀 자리 (기본: 그대로). 하위 클래스가 그림자 끄기 등에 사용."""

    def camera_optical_in_base(self):
        """base_link → head_camera_optical_frame (x 오른쪽, y 아래, z 전방) 변환 4x4."""
        d = self.d
        Rw_cam = d.cam_xmat[self.cam_id].reshape(3, 3) @ np.diag([1, -1, -1])
        pw = d.cam_xpos[self.cam_id]
        Rb = d.xmat[self.base_id].reshape(3, 3)
        pb = d.xpos[self.base_id]
        T = np.eye(4)
        T[:3, :3] = Rb.T @ Rw_cam
        T[:3, 3] = Rb.T @ (pw - pb)
        return T

    # ------------------------------------------------------------------ realtime loop helper
    def run_realtime(self, stop_event, rate_factor=1.0, on_tick=None, tick_hz=100.0, viewer=None):
        """wall clock 에 맞춰 시뮬레이션을 진행. on_tick(sim) 를 tick_hz 로 호출."""
        dt = self.m.opt.timestep
        steps_per_tick = max(1, int(round(1.0 / tick_hz / dt)))
        wall0 = time.monotonic()
        sim0 = self.d.time
        last_view = 0.0
        while not stop_event.is_set():
            self.step(steps_per_tick)
            if on_tick:
                on_tick(self)
            if viewer is not None:
                now = time.monotonic()
                if now - last_view > 1 / 30:
                    if not viewer.is_running():
                        stop_event.set()
                        break
                    with self.lock:
                        viewer.sync()
                    last_view = now
            target = wall0 + (self.d.time - sim0) / rate_factor
            sleep = target - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            elif sleep < -1.0:   # 너무 밀리면 기준 재설정 (실시간보다 느린 PC)
                wall0 = time.monotonic()
                sim0 = self.d.time
