"""MuJoCo 시뮬레이션 코어 중 하반신과 무관한 부분 (ROS 비의존).

OpenArmSimBase 는 팔·그리퍼·파지 보조·LiDAR·카메라·사람 장애물을 맡는다. 하반신(보행 정책·다리 오도메트리)과
초기화는 biped_core.BipedOpenArmSim 이 맡고, bridge_node.py 가 그것을 ROS 2 토픽/액션으로 노출한다.
ROS 없이도 tools/selftest_tron1.py 로 보행·LiDAR·카메라·ArUco·팔 동작을 검증할 수 있다.

기능
  * 2D LiDAR (mj_multiRay, 월드 geom group 0/1 만 감지 → 로봇 자기 몸체 무시)
  * 머리 RGB 카메라 오프스크린 렌더 + ArUco 검출/포즈 추정 (검출 결과만 밖으로)
  * 관절 궤적 실행 (FollowJointTrajectory 와 동일한 의미: 시간-위치 보간)
  * 그리퍼 + "파지 보조(grasp assist)" weld 구속 (교육용 옵션, load_model 이 손끝↔상자 weld 를 꺼진 채 넣어 둠)
  * 움직이는 사람(mocap) 장애물
"""
import math
import os
import time

import mujoco
import numpy as np

ARM_JOINTS = {
    side: [f"openarm_{side}_joint{i}" for i in range(1, 8)] for side in ("left", "right")
}
GRIPPER_JOINT = {side: f"openarm_{side}_finger_joint1" for side in ("left", "right")}
MIMIC_JOINT = {side: f"openarm_{side}_finger_joint2" for side in ("left", "right")}


def yaw_of(quat_wxyz):
    w, x, y, z = quat_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))



def load_model(xml_path):
    """MJCF 를 읽고 파지 보조용 weld 구속(손끝 ↔ 상자마다 하나, 꺼진 상태)을 더해 컴파일한다.
    상자를 쥐면 그때의 상대 자세로 켜고(_update_attached), 손가락을 열면 끈다."""
    spec = mujoco.MjSpec.from_file(xml_path)
    names = {b.name for b in spec.bodies}
    boxes = sorted(n for n in names if n.startswith("box_"))
    for side in ("left", "right"):
        tcp = f"openarm_{side}_hand_tcp"
        if tcp not in names:
            continue
        for b in boxes:
            e = spec.add_equality()
            e.type, e.objtype = mujoco.mjtEq.mjEQ_WELD, mujoco.mjtObj.mjOBJ_BODY
            e.name, e.name1, e.name2 = f"grasp_{side}_{b}", tcp, b
            e.active = False
            e.solref = [0.01, 1.0]
    return spec.compile()

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


class OpenArmSimBase:
    """상반신(OpenArm 양팔)·센서·파지 보조·사람 장애물. 생성자는 없다 — 하위 클래스(biped_core.BipedOpenArmSim)가
    모델을 읽고 아래 메서드가 쓰는 속성(m, d, lock, arm_qadr, arm_act, grip_qadr, grip_act, traj, grip_traj,
    LiDAR·카메라·파지 보조·사람 설정)을 모두 만든다. 하위 클래스는 _apply_base(물리 스텝마다 하반신 제어)와
    _update_odom(스텝 묶음마다 오도메트리)을 구현한다."""

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

    # ------------------------------------------------------------------ commands
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

    # ------------------------------------------------------------------ grasp assist
    def _init_grasp_welds(self):
        m = self.m
        self.grasp_eq = {}
        for s in ("left", "right"):
            for b in self.box_bodies:
                i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, f"grasp_{s}_{m.body(b).name}")
                if i >= 0:
                    self.grasp_eq[(s, b)] = i

    def _update_attached(self):
        """파지 보조: 그리퍼를 닫는 중 두 손가락이 모두 상자에 닿으면 그때의 상대 자세로 손끝↔상자 weld 를 켠다. 열면 끈다.
        예전에는 손가락 하나만 닿아도 '부착' 하고, 매 스텝 상자 위치를 손끝 기준으로 덮어쓰며 속도를 0 으로 만들었다.
        그러면 상자가 공중에 고정된 무한 질량 물체가 되어, 나머지 손가락이 닫히며 그 상자를 400 N 넘게 밀고 반작용이 팔→몸통→다리로
        그대로 전해졌다 (쥐고·놓을 때마다 하체가 떨리고 발을 디딤, 상자가 손가락 사이 1 cm 비켜 있으면 1 m 밀려 넘어짐 —
        오프라인 비교). weld 는 손과 상자 사이의 내력이라 쥐는 힘이 몸통을 밀지 않는다 (같은 조건 0.35 cm, 서기 풀림 0)."""
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
                        eq = self.grasp_eq.get((side, b))
                        if eq is None:
                            continue
                        Rt = d.xmat[tcp].reshape(3, 3)
                        rel = Rt.T @ (d.xpos[b] - d.xpos[tcp])
                        # TCP 기준 상자 중심이 손가락 사이(±3.5 cm)에 있고, 두 손가락이 모두 상자에 닿았을 때
                        if np.all(np.abs(rel) < [0.035, 0.035, 0.04]) and self._fingers_touch(side, b) >= 2:
                            qrel = np.zeros(4)
                            qinv = np.zeros(4)
                            mujoco.mju_negQuat(qinv, d.xquat[tcp])
                            mujoco.mju_mulQuat(qrel, qinv, d.xquat[b])
                            m.eq_data[eq, 0:3] = 0.0           # weld: anchor, body1(손끝) 기준 body2(상자) 상대 위치·자세
                            m.eq_data[eq, 3:6] = rel
                            m.eq_data[eq, 6:10] = qrel
                            d.eq_active[eq] = 1
                            self.attached[side] = (b, rel, qrel)
                            break
            elif target > 0.03:  # 열면 해제
                eq = self.grasp_eq.get((side, att[0]))
                if eq is not None:
                    d.eq_active[eq] = 0
                self.attached[side] = None

    def _fingers_touch(self, side, box_body):
        """상자에 닿은 손가락 수 (0~2)."""
        m, d = self.m, self.d
        names = {f"openarm_{side}_right_finger", f"openarm_{side}_left_finger"}
        touched = set()
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            for a, b in ((b1, b2), (b2, b1)):
                if b == box_body and m.body(a).name in names:
                    touched.add(m.body(a).name)
        return len(touched)

    def object_poses(self):
        """상자들의 world 위치 (정답값, 디버깅/채점용)."""
        return {self.m.body(b).name: [round(float(v), 4) for v in self.d.xpos[b]] for b in self.box_bodies}

    ROBOT_PREFIXES = ("openarm_", "base")   # arm_contacts 에서 로봇 몸으로 볼 이름 (하위 클래스가 다리를 더함)

    def arm_contacts(self):
        """디버그·채점용: 팔·손이 로봇 밖의 물체(작업대·상자·벽)와 닿아 있는 접촉 목록 ['팔 body-상대 body', ...].
        시뮬 스레드가 동시에 mj_step 을 돌리므로 잠금 안에서 접촉 쌍만 복사한다."""
        m, d = self.m, self.d
        with self.lock:
            pairs = [(int(d.contact[i].geom1), int(d.contact[i].geom2)) for i in range(d.ncon)]
        out = set()
        for g1, g2 in pairs:
            b1, b2 = m.body(m.geom_bodyid[g1]).name, m.body(m.geom_bodyid[g2]).name
            for a, b in ((b1, b2), (b2, b1)):
                if a.startswith("openarm_") and not b.startswith(self.ROBOT_PREFIXES):
                    out.add(f"{a}-{b or 'world'}")
        return sorted(out)

    def attached_object(self, side):
        a = self.attached[side]
        return None if a is None else self.m.body(a[0]).name

    # ------------------------------------------------------------------ sensors
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
        """카메라 렌더 직전 장면 설정을 바꿀 자리 (하위 클래스가 그림자 끄기 등에 사용)."""

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
        # 뷰어 갱신 주기. sync 는 뷰어가 한 장면을 다 그릴 때까지 기다리므로 그동안 물리가 멈춘다.
        # 전체 메시 모델·큰 창(2560x1440)에서 한 장면 약 26 ms → 30 Hz 면 1 초 중 0.78 s 를 기다려 실시간의 0.5 배로 돌았다
        view_period = 1.0 / float(os.environ.get("SIM_VIEW_HZ", 30.0))
        while not stop_event.is_set():
            self.step(steps_per_tick)
            if stop_event.is_set():
                break
            if on_tick:
                on_tick(self)
            if viewer is not None:
                now = time.monotonic()
                if now - last_view > view_period:
                    if not viewer.is_running():
                        stop_event.set()
                        break
                    with self.lock:
                        hook = getattr(self, "view_hook", None)
                        if hook is not None:
                            hook(viewer)        # 예: 관찰 카메라 시선 중심 갱신 (bridge_node._track_robot)
                        viewer.sync()
                    last_view = now
            target = wall0 + (self.d.time - sim0) / rate_factor
            sleep = target - time.monotonic()
            if sleep > 0:
                stop_event.wait(sleep)
            elif sleep < -1.0:   # 너무 밀리면 기준 재설정 (실시간보다 느린 PC)
                wall0 = time.monotonic()
                sim0 = self.d.time
