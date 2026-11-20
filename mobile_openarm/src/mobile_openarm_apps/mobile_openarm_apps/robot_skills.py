"""로봇 기본 동작(스킬) 라이브러리 — Nav2 / MoveIt 2 / 그리퍼 / ArUco 를 한 클래스로.

시뮬레이션(mujoco_bridge)과 실로봇(openarm_bringup + 바퀴 베이스 드라이버)이 같은 토픽·액션
이름을 쓰도록 만들었으므로 이 파일은 그대로 실로봇에도 사용할 수 있다.

    from mobile_openarm_apps.robot_skills import RobotSkills
    sk = RobotSkills.create()          # rclpy.init + 백그라운드 executor
    sk.navigate("pick_station")
    sk.pick("box_0")
"""
import json
import math
import os
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point32, Polygon, Pose, PoseStamped, Twist
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MoveItErrorCodes,
                             PlanningScene, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .geometry import approach_rotation, mat_to_quat_xyzw, quat_to_yaw, rot_axis, se2_inverse_apply, yaw_to_quat


ARM_JOINTS = {s: [f"openarm_{s}_joint{i}" for i in range(1, 8)] for s in ("left", "right")}
TCP = {s: f"openarm_{s}_hand_tcp" for s in ("left", "right")}
BASE = "base_footprint"


class SkillError(RuntimeError):
    pass


def _dur(sec):
    d = Duration()
    d.sec = int(sec)
    d.nanosec = int((sec - int(sec)) * 1e9)
    return d


class RobotSkills(Node):
    def __init__(self, name="robot_skills"):
        super().__init__(name, parameter_overrides=[Parameter("use_sim_time", value=True)])
        sim_share = get_package_share_directory("mobile_openarm_sim")
        mv_share = get_package_share_directory("mobile_openarm_moveit_config")
        with open(os.path.join(sim_share, "config", "locations.yaml"), encoding="utf-8") as f:
            self.world = yaml.safe_load(f)
        self.locations = self.world["locations"]
        self.objects = self.world["objects"]
        self.table_h = self.world["aruco"]["table_height"]
        self.box_size = self.world["aruco"]["box_size"]
        self.tables = self.world["tables"]                 # 이름 → center/size
        self.station_table = self.world["station_table"]   # 스테이션 → 작업대
        self.current_station = None
        # 이름 자세(home/ready/carry/hands_up)를 읽을 SRDF
        srdf = os.path.join(mv_share, "config", "mobile_openarm.srdf")
        self.named_states = self._load_srdf_states(srdf)
        self._table_top_seen = None      # 카메라로 잰 상판 높이 (손목 카메라 보정에서 높이가 안 맞는 측정을 버릴 때)

        cb = ReentrantCallbackGroup()
        self.cb = cb
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cb)
        self.move_group = ActionClient(self, MoveGroup, "move_action", callback_group=cb)
        self.exec_traj = ActionClient(self, ExecuteTrajectory, "execute_trajectory", callback_group=cb)
        self.grippers = {s: ActionClient(self, FollowJointTrajectory,
                                         f"/{s}_gripper_controller/follow_joint_trajectory", callback_group=cb)
                         for s in ("left", "right")}
        self.ik_srv = self.create_client(GetPositionIK, "compute_ik", callback_group=cb)
        self.cart_srv = self.create_client(GetCartesianPath, "compute_cartesian_path", callback_group=cb)
        self.scene_srv = self.create_client(ApplyPlanningScene, "apply_planning_scene", callback_group=cb)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._det_lock = threading.Lock()
        self._snaps = {}                # id → (그때 위치, 함께 보인 다른 마커 위치들) — 가려졌을 때 시각 오도메트리용
        self._marker_odom = {}          # id → odom 좌표 (가려졌을 때 몸통 흐름을 보정해 다시 쓰기)
        self._detections = {}           # id → (t_wall, position[3], yaw)
        self._attached = {"left": None, "right": None}
        self.create_subscription(String, "/aruco/detections", self._on_det, 10, callback_group=cb)
        # 손목 카메라 (카메라 서버 sim_cameras): 손끝(TCP) 기준 마커 위치. 카메라→손끝이 강체라 팔 처짐과
        # 상관없이 '손 기준 상자 위치' 가 정확하다. 머리 카메라는 손이 다가가면 손이 상자를 가린다
        self._wrist = {"left": {}, "right": {}}     # side → id → (t_wall, stamp, 손끝 기준 위치[3])
        self.wrist_cameras = False                  # 손목 카메라 메시지를 한 번이라도 받았으면 True
        for side in ("left", "right"):
            self.create_subscription(String, f"/wrist_{side}_camera/aruco/detections",
                                     lambda msg, s=side: self._on_wrist(s, msg), 10, callback_group=cb)
        self.create_subscription(String, "/sim/attached", self._on_att, 10, callback_group=cb)
        # 작업대 앞 모서리 (머리 카메라 깊이, 카메라 서버 table_edge.py): 도착 뒤 작업대에 대한 정밀 정렬.
        # 어느 작업대인지는 지도 좌표로 정하고(마커를 읽으러 다가갈 필요 없음), 거리·방향·상판 높이만 이것으로 맞춘다
        self._edge = None                # (받은 시각, dict)
        self.create_subscription(String, "/table/edge", self._on_edge, 10, callback_group=cb)
        # LiDAR: 작업대 앞 다리 두 개로 작업대 중심의 옆 위치를 잰다 (table_center_lidar) — AMCL 옆 오차(최대 0.2 m)를 쓰지 않으려고
        self._scan = None
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "_scan", m), 1, callback_group=cb)
        self._joints = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10, callback_group=cb)
        self.status_pub = self.create_publisher(String, "/task/log", 10)
        # Nav2 장애물 지도의 로봇 외곽을 바꾸는 토픽 (costmap 이 'footprint' 를 구독) — 상자를 몸 앞에 들고 다닐 때 늘림
        self._fp_pubs = [self.create_publisher(Polygon, f"/{c}/footprint", 10) for c in ("local_costmap", "global_costmap")]
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.holding = {"left": None, "right": None}
        self._grasp_tilt = {"left": None, "right": None}     # 쥘 때 손 (pitch, roll, yaw) — place 에서 수평 유지
        self._held_level = {"left": False, "right": False}   # 쥔 상자를 수평으로 들고 있음 (_hold_level, carry 로 접지 않음)
        # True: 집기 전후 팔을 carry(몸 쪽으로 접은 자세)로 옮긴다 — 상자를 든 채 걸어서 다른 곳으로 갈 때 필요.
        # 같은 작업대에서 집어 바로 놓을 때(pick_place)는 False: home → 접근 → 파지 → 살짝 들기 → 놓기 → home.
        # carry 를 거치면 팔 이동 4 번(약 3.5 분, 무작위 배치 r8 기록)이 더 들었다
        self.fold_carry = True
        self.cancel_event = threading.Event()
        self._executor = None

    # ------------------------------------------------------------------ 생성/정리
    @classmethod
    def create(cls, name="robot_skills"):
        if not rclpy.ok():
            rclpy.init()
        node = cls(name)
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        node._executor = ex
        return node

    def shutdown(self):
        if self._executor:
            self._executor.shutdown()
        self.destroy_node()
        rclpy.try_shutdown()

    def log(self, msg):
        self.get_logger().info(msg)
        m = String()
        m.data = msg
        self.status_pub.publish(m)

    # ------------------------------------------------------------------ 공통 대기
    def _wait(self, fut, timeout, what):
        t0 = time.time()
        while not fut.done():
            if self.cancel_event.is_set():
                raise SkillError(f"{what}: 취소됨")
            if time.time() - t0 > timeout:
                raise SkillError(f"{what}: {timeout:.0f}s 시간 초과")
            time.sleep(0.02)
        return fut.result()

    def _send_action(self, client, goal, what, timeout=60.0, server_timeout=10.0):
        if not client.wait_for_server(timeout_sec=server_timeout):
            raise SkillError(f"{what}: 액션 서버({client._action_name}) 없음 — 해당 노드를 실행했는지 확인")
        gh = self._wait(client.send_goal_async(goal), 10.0, what + " (goal 전송)")
        if not gh.accepted:
            raise SkillError(f"{what}: goal 거부됨")
        try:
            res = self._wait(gh.get_result_async(), timeout, what)
        except SkillError:
            gh.cancel_goal_async()
            raise
        return res

    def _call(self, client, req, what, timeout=10.0):
        if not client.wait_for_service(timeout_sec=10.0):
            raise SkillError(f"{what}: 서비스({client.srv_name}) 없음")
        return self._wait(client.call_async(req), timeout, what)

    # ------------------------------------------------------------------ 상태
    def _on_det(self, msg):
        data = json.loads(msg.data)
        now = time.time()
        with self._det_lock:
            for d in data.get("detections", []):
                self._detections[d["id"]] = (now, np.array(d["position"]), d.get("yaw", 0.0))

    def _on_wrist(self, side, msg):
        data = json.loads(msg.data)
        now = time.time()
        with self._det_lock:
            self.wrist_cameras = True
            for d in data.get("detections", []):
                if "position_tcp" in d:
                    self._wrist[side][d["id"]] = (now, data.get("stamp"), np.array(d["position_tcp"]))

    def _measure_box_wrist(self, side, box_id, samples=4, timeout=2.0):
        """손목 카메라로 상자 중심(base_footprint)을 잰다: 손끝(정기구학) + R_tcp · (카메라로 잰 손끝 기준 마커 위치).
        안 보이면 None. 반환: (손끝, 상자 중심) — 손끝↔상자 상대 위치는 카메라→손끝 강체 관계로 정확하다."""
        if not self.wrist_cameras:
            return None
        tcps, boxes, seen, t0 = [], [], set(), time.time()
        while len(boxes) < samples and time.time() - t0 < timeout:
            with self._det_lock:
                item = self._wrist[side].get(box_id)
            if item is not None and time.time() - item[0] < self.FRESH_S and item[1] not in seen:
                seen.add(item[1])
                tf = self._tf_matrix(TCP[side])
                if tf is not None:
                    top = tf[0] + tf[1] @ item[2]
                    tcps.append(tf[0])
                    boxes.append(top - np.array([0, 0, self.box_size / 2]))
            time.sleep(0.05)
        if not boxes:
            return None
        return np.median(np.array(tcps), axis=0), np.median(np.array(boxes), axis=0)

    def _on_att(self, msg):
        self._attached = json.loads(msg.data)

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self._joints[n] = p

    def gripper_opening(self, side):
        return self._joints.get(f"openarm_{side}_finger_joint1")

    def robot_pose(self, frame="map"):
        """map 기준 (x, y, yaw). TF 가 없으면 None."""
        try:
            t = self.tf_buffer.lookup_transform(frame, BASE, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, quat_to_yaw(q.x, q.y, q.z, q.w))

    def _on_edge(self, msg):
        try:
            e = json.loads(msg.data)
        except ValueError:
            return
        if e.get("found"):
            self._edge = (time.time(), e)

    def table_edge_seen(self, wait=1.5, settle=0.3):
        """지금(멈춘 뒤 settle 초 지나서) 잰 작업대 앞 모서리 {dist, yaw, height, ...} 또는 None (못 찾음·발행 안 함)."""
        after = time.time() + settle
        t_end = after + wait
        while time.time() < t_end:
            e = self._edge
            if e is not None and e[0] >= after:
                return e[1]
            time.sleep(0.05)
        return None

    def table_center_lidar(self, station=None, e=None):
        """작업대 중심이 로봇 기준 모서리 방향(왼쪽 +)으로 얼마나 떨어져 있나 [m] — LiDAR 로 본 앞 다리 두 개의 중간점. 못 찾으면 None.
        앞 모서리 거리·방향(e: /table/edge)으로 모서리 바로 안쪽 띠(0~9 cm)의 점만 모아, 모서리 방향으로 묶음을 나누고
        작업대 폭에서 나오는 다리 간격(폭 − 8 cm)과 맞는 두 묶음을 찾는다.
        왜: 깊이 카메라 시야(옆 ±0.3 m)로는 작업대 양 끝이 안 보여 옆 위치는 지도(AMCL)에 의존했는데, AMCL 이 0.22 m 틀리자
        도착 후 옆걸음이 반대로 어긋나고 MoveIt 작업대도 옆으로 밀려, 팔이 실제 작업대를 눌러 넘어졌다 (미션 D)."""
        station = station or self.current_station
        table = self.tables.get(self.station_table.get(station))
        e = e or self._edge_recent(2.0)
        msg = self._scan
        if table is None or e is None or msg is None:
            return None
        tf = self._tf_pair(BASE, msg.header.frame_id)
        if tf is None:
            return None
        r = np.asarray(msg.ranges, float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > msg.range_min) & (r < min(msg.range_max, 3.0))
        P = (tf[1] @ np.vstack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())])).T + tf[0]
        th = float(e["yaw"])
        n_in, e_dir = np.array([math.cos(th), math.sin(th)]), np.array([-math.sin(th), math.cos(th)])
        u = P[:, :2] @ n_in - float(e["dist"])                       # 모서리에서 안쪽 거리
        v = np.sort(P[(u > -0.01) & (u < 0.09), :2] @ e_dir)          # 모서리 방향 위치
        if len(v) < 4:
            return None
        groups = np.split(v, np.where(np.diff(v) > 0.04)[0] + 1)
        legs = [float(g.mean()) for g in groups if len(g) >= 2 and np.ptp(g) < 0.10]
        sep = float(table["size"][0]) - 0.08
        best = None
        for i in range(len(legs)):
            for j in range(i + 1, len(legs)):
                d = abs(abs(legs[j] - legs[i]) - sep)
                if d < 0.05 and (best is None or d < best[0]):
                    best = (d, 0.5 * (legs[i] + legs[j]))
        return None if best is None else best[1]

    def _edge_recent(self, max_age=1.5):
        e = self._edge
        return e[1] if e is not None and time.time() - e[0] < max_age else None

    def detections(self, max_age=1.0):
        now = time.time()
        with self._det_lock:
            return {i: (p, yaw) for i, (t, p, yaw) in self._detections.items() if now - t < max_age}

    FRESH_S = 0.5           # 카메라 검출을 '새 프레임' 으로 볼 최대 나이 [s]

    def find_marker(self, marker_id, samples=5, timeout=5.0):
        """여러 프레임 평균한 마커 위치 (base_footprint)."""
        pts = []
        time.sleep(0.3)                 # 베이스 정지 후의 프레임만 쓰도록 잠깐 대기 (카메라 10 Hz, 신선도는 FRESH_S 로 확인)
        t0 = time.time()
        last_t = None
        while len(pts) < samples and time.time() - t0 < timeout:
            with self._det_lock:
                item = self._detections.get(marker_id)
            if item and item[0] != last_t and time.time() - item[0] < self.FRESH_S:
                last_t = item[0]
                pts.append(item[1])
            time.sleep(0.05)
        if not pts:
            raise SkillError(f"마커 {marker_id} 를 카메라에서 찾지 못함")
        p = np.median(np.array(pts), axis=0)
        self._remember_marker(marker_id, p)
        self._snapshot(marker_id, p)
        return p

    def _other_markers(self, exclude, max_age=None):
        """지금 보이는 '다른' 물체 마커들 (id → base 위치). 손목 마커·exclude 는 뺀다."""
        known = {o["marker_id"] for o in self.objects.values()}
        now = time.time()
        max_age = self.FRESH_S if max_age is None else max_age
        with self._det_lock:
            return {i: v[1].copy() for i, v in self._detections.items()
                    if i in known and i != exclude and now - v[0] < max_age}

    def _snapshot(self, marker_id, p_top):
        """목표 마커를 잰 순간, 함께 보인 다른 마커들의 위치를 같이 저장 — 나중에 목표가 손에 가려지면
        다른 마커들이 몸통 기준으로 얼마나 움직였는지(= 몸통이 흐른 양)로 목표의 지금 위치를 추정한다 (시각 오도메트리)."""
        self._snaps[marker_id] = (np.asarray(p_top, float).copy(), self._other_markers(marker_id))

    def predict_hidden(self, marker_id):
        """가려진 목표 마커의 지금 위치(base) 추정: 마지막 측정 + 다른 마커들의 평균 이동. 공통 마커가 없으면 None."""
        snap = self._snaps.get(marker_id)
        if snap is None:
            return None, 0
        p0, others0 = snap
        now = self._other_markers(marker_id)
        common = [i for i in others0 if i in now]
        if not common:
            return None, 0
        shifts = np.array([now[i] - others0[i] for i in common])
        shift = np.median(shifts, axis=0)
        if np.linalg.norm(shift) > 0.15 or (len(common) > 1 and np.ptp(shifts, axis=0).max() > 0.05):
            return None, 0          # 너무 크게 움직였거나 마커끼리 안 맞으면(몸통이 크게 돎·오검출) 믿지 않음 (SF 시험 7: 53 cm)
        return p0 + shift, len(common)

    def _remember_marker(self, marker_id, p_base):
        """잰 마커 위치를 odom 좌표로도 기억 — 나중에 가려져 못 볼 때 odom 으로 현재 base 좌표를 다시 계산한다."""
        T = self._tf_pair("odom", BASE)
        if T is not None:
            self._marker_odom[marker_id] = T[1] @ np.asarray(p_base) + T[0]

    def recall_marker(self, marker_id):
        """기억한 마커(odom)를 지금의 base_footprint 좌표로. 없으면 None."""
        q = self._marker_odom.get(marker_id)
        T = self._tf_pair(BASE, "odom")
        if q is None or T is None:
            return None
        return T[1] @ q + T[0]

    # ------------------------------------------------------------------ Nav2
    def navigate(self, target, timeout=600.0, retries=1):
        """target: locations.yaml 이름 또는 (x, y, yaw_deg)."""
        if isinstance(target, str):
            if target not in self.locations:
                raise SkillError(f"알 수 없는 위치 '{target}' (가능: {list(self.locations)})")
            x, y, yaw_deg = self.locations[target]
        else:
            x, y, yaw_deg = target
        self.clear_table()
        if self.current_station in self.station_table:
            d = self.table_edge_distance()
            if d is not None and d < self.DOCK_GAP + 0.2:
                self.nudge_base(-(self.DOCK_GAP + 0.30 - d))     # 작업대에서 0.7m 까지 물러난 뒤 출발 (회전 공간 확보)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        q = yaw_to_quat(math.radians(yaw_deg))
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = q
        self.log(f"[nav] → {target} ({x:.2f}, {y:.2f}, {yaw_deg:.0f}°)")
        for attempt in range(1, retries + 2):
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            res = self._send_action(self.nav, goal, f"navigate({target})", timeout)
            if res.status == GoalStatus.STATUS_SUCCEEDED:
                break
            if attempt <= retries:
                self.log(f"[nav] 실패(status={res.status}) → 재시도 {attempt}/{retries}")
                time.sleep(1.0)
        else:
            raise SkillError(f"navigate({target}) 실패: status={res.status}")
        self.log(f"[nav] 도착: {target}")
        self.current_station = target if isinstance(target, str) else None
        return True

    def nudge_base(self, dx, speed=0.08, timeout=15.0):
        """Nav2 를 거치지 않고 짧게 전진/후진 (작업대 앞 미세 위치 조정, |dx| <= 0.40 m).
        odom TF 로 실제 이동거리를 확인하는 폐루프 — 시뮬이 실시간보다 느려도 정확히 움직인다."""
        dx = float(np.clip(dx, -0.40, 0.40))
        if abs(dx) < 0.01:
            return
        self.log(f"[base] 미세 이동 {dx:+.2f} m")
        p0 = self.robot_pose("odom")
        msg = Twist()
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.robot_pose("odom")
            moved = 0.0 if (p is None or p0 is None) else math.hypot(p[0] - p0[0], p[1] - p0[1])
            remain = abs(dx) - moved
            if remain < 0.005:
                break
            msg.linear.x = math.copysign(max(0.02, min(speed, remain * 1.5)), dx)
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.2)

    # ------------------------------------------------------------------ 손목 카메라 시각 피드백
    SERVO_ITERS = 4         # 보정 이동 최대 횟수
    SERVO_TOL = 0.010       # 손끝-목표 오차가 이보다 작으면 보정 끝 [m]
    SERVO_MAX_STEP = 0.05   # 한 번에 보정하는 최대 거리 [m] (잘못 잰 값으로 크게 튀지 않게)
    SERVO_DEPTH = 0.010     # 손끝(TCP)을 상자 중심보다 접근 방향으로 이만큼 더 넣는다 [m]

    def _tf_matrix(self, child):
        """BASE → child 변환 (p[3], R[3x3]). 없으면 None."""
        return self._tf_pair(BASE, child)

    def _tf_pair(self, target, source):
        """source 좌표의 점을 target 좌표로: p_target = R @ p_source + t → (t, R). 없으면 None."""
        try:
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        tr, q = t.transform.translation, t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return np.array([tr.x, tr.y, tr.z]), R

    def measure_hand_and_box(self, side, box_id, samples=4, timeout=5.0, allow_fk=True):
        """손끝과 상자 중심을 함께 잰다 (base_footprint). 손목 카메라로 상자가 보이면 그것(손끝 기준 상자 위치),
        안 보이면 측정된 관절각의 정기구학 손끝 + 머리 카메라 상자.
        반환: (손끝, box_center 또는 None, tcp_fk, 손끝 출처) — 둘 다 안 되면 SkillError"""
        time.sleep(0.3)
        w = self._measure_box_wrist(side, box_id)
        if w is not None:
            return w[0], w[1], w[0], "손목 카메라"
        if allow_fk:
            fk = self._tf_matrix(TCP[side])
            with self._det_lock:
                b = self._detections.get(box_id)
            box = None if b is None or time.time() - b[0] > 1.0 else b[1] - np.array([0, 0, self.box_size / 2])
            if fk is not None:
                return fk[0], box, fk[0], "정기구학"
        with self._det_lock:
            ages = {i: round(time.time() - v[0], 1) for i, v in self._detections.items()}
        raise SkillError(f"{side} 손목 카메라로 상자를 못 봄 (최근 검출 id:경과초 {ages}, 정기구학 손끝 TF 없음)")

    def visual_servo(self, side, box_id, R, goal, center, offset, floor_z=None, label="", peek=False):
        """손끝이 '상자 중심 + offset' 에 오도록 카메라로 재며 고친다 (closed-loop).
        goal: 지금까지 MoveIt 에 명령한 손끝 목표. 팔이 중력에 처지는 만큼 실제 손끝은 목표에 못 미치므로
        '명령 목표' 를 오차만큼 누적해서 옮긴다 (적분) → 처짐이 매번 같으면 2~3 번에 수렴.
        반환: (최종 명령 목표, 최신 상자 중심)"""
        peeked = False
        for it in range(1, self.SERVO_ITERS + 1):
            try:
                tcp, box, fk, src = self.measure_hand_and_box(side, box_id)
            except SkillError as e:
                self.log(f"[servo{label}] {e} → 보정 생략")
                break
            if box is None and peek and not peeked:
                # 파지 위치에서 상자 마커가 손에 가려졌으면 손을 잠깐 위·뒤로 빼서 상자를 직접 다시 잰다.
                # 다른 마커로만 추정하면 손이 상자를 민 것을 못 봐, 오차 0.6 cm 로 '맞췄는데' 빈손으로 닫혔다 (SF 미션 15: 상자 4.4 cm 밀림)
                peeked = True
                tf = self._tf_matrix(TCP[side])
                if tf is not None:
                    try:
                        self.cartesian(side, [tf[0] + np.array([0, 0, 0.06]) - R[:, 2] * 0.05], R, vel=0.1,
                                       min_fraction=0.0, avoid_collisions=False)
                        top = self.find_marker(box_id, samples=3)
                        center = top - np.array([0, 0, self.box_size / 2])
                        self._snapshot(box_id, top)
                        self.log(f"[servo{label}] 손을 빼고 다시 잰 상자 {np.round(center, 3)}")
                    except SkillError as e:
                        self.log(f"[servo{label}] 손을 빼고 다시 재기 실패: {e}")
                    try:
                        self.cartesian(side, [goal], R, vel=0.1, min_fraction=0.0, avoid_collisions=False)
                    except SkillError:
                        pass
                    continue
            stale = ""
            top_ok = self._table_top_seen is None or box is None or \
                abs(box[2] - self.box_size / 2 - self._table_top_seen) < 0.045
            if box is not None and top_ok:
                center = box                    # 같은 프레임에서 본 상자가 우선
                self._remember_marker(box_id, box + np.array([0, 0, self.box_size / 2]))
                self._snapshot(box_id, box + np.array([0, 0, self.box_size / 2]))
            else:
                # 카메라로 본 상자는 정답과 1 cm 안으로 맞았다(시험 11 정답 기록) — 손에 밀려 9 cm 움직인 것도 정확히 잡음.
                # 반대로 odom 으로 옮긴 옛 위치는 제자리 걸음 미끄러짐을 몰라 9 cm 틀렸다. 그래서 보이면 항상 카메라 값을 쓰고,
                # 높이가 상판과 4.5 cm 넘게 안 맞는 측정(손가락에 가려 깊이를 손가락에서 읽은 경우)만 버린다.
                # (2 cm 로 두었더니 SF 가 팔을 뻗어 몸통이 숙여질 때 생기는 3 cm 높이 차이로 올바른 측정까지 버렸다: SF 시험 8)
                # 상자가 가려졌으면 마지막 카메라 측정을 그대로 쓴다. odom 으로 옮기는 방법도 시험했지만(시험 11·12)
                # 다리 오도메트리가 제자리 걸음 중의 회전·미끄러짐을 못 봐서 오히려 7~9 cm 더 틀렸다.
                # 그래서 접근점(상자가 손에 가려지는 곳)보다 파지 위치(상자가 다시 보이는 곳)에서의 보정이 핵심이다.
                # 대신 함께 보이는 다른 상자 마커(작업대 위에 가만히 있음)가 몸통 기준으로 움직인 양만큼 옮긴다 (시각 오도메트리).
                # 시험 13: 목표 마커가 손에 가려진 1분 동안 몸통이 흘러, 옛 측정으로 '오차 0.5 cm' 라 판단했지만 실제로는
                # 손이 상자를 10 cm 밀고 손끝이 상자 2~5 cm 옆에 있었다.
                pred, n = self.predict_hidden(box_id)
                if pred is not None:
                    center = pred - np.array([0, 0, self.box_size / 2])
                why = "가려짐" if box is None else f"측정 {np.round(box, 3)} 높이가 상판과 안 맞음"
                stale = (f" [상자 {why}: 다른 마커 {n}개의 이동으로 추정]" if pred is not None
                         else f" [상자 {why}: 마지막 측정 사용]")
            want = center + offset
            err = want - tcp
            fk_s = "" if fk is None or src != "손목 마커" else f", 정기구학과 차이 {np.round((tcp - fk) * 100, 1)} cm"
            self.log(f"[servo{label}] {it}: 손끝({src}) {np.round(tcp, 3)} → 목표 {np.round(want, 3)}, "
                     f"오차 {np.round(err * 100, 1)} cm (|{np.linalg.norm(err) * 100:.1f}|){fk_s}"
                     + stale)
            if np.linalg.norm(err) < self.SERVO_TOL:
                break
            n = np.linalg.norm(err)
            step = err if n <= self.SERVO_MAX_STEP else err * (self.SERVO_MAX_STEP / n)
            lat = err - R[:, 2] * float(err @ R[:, 2])       # 접근 방향에 수직인 성분
            if np.linalg.norm(lat) > 0.012:
                # 옆·위아래로 어긋났으면 앞으로 더 들어가지 않고, 접근 방향 뒤로 2 cm 물러난 자리에서 옆만 맞춘다.
                # 앞으로 같이 가면 손가락이 상자 옆면을 밀어 상자가 계속 밀려나고 보정이 그것을 쫓는다
                # (PF 시험 14: 8 cm, SF 시험 12: 앞·옆으로 9 cm). 손가락 사이 여유는 ±1.9 cm (벌림 8.8 cm − 상자 5 cm)
                # (물러나기를 매번 2 cm 씩 더했더니 옆 오차가 1~2 cm 로 남는 동안 계속 누적돼 손이 상자 6 cm 뒤에서
                #  손가락을 닫았다: SF 미션 시험 8 → 앞으로는 안 가고, 너무 깊이 들어간 경우에만 뒤로)
                n_l = np.linalg.norm(lat)
                along = float(err @ R[:, 2])
                step = (lat if n_l <= self.SERVO_MAX_STEP else lat * (self.SERVO_MAX_STEP / n_l)) + R[:, 2] * min(along, 0.0)
            goal = goal + step
            if floor_z is not None:
                goal[2] = max(goal[2], floor_z)
            try:
                self.cartesian(side, [goal], R, vel=0.1, min_fraction=0.0, avoid_collisions=False)
            except SkillError as e:
                # 손가락이 상판에 걸려 팔이 목표를 못 따라감(−4) → 손을 조금 들고 목표도 그만큼 올려 다시
                self.log(f"[servo{label}] 보정 이동 실패: {e} → 2 cm 들고 다시")
                self._lift_clear(side, 0.02)
                goal = goal + np.array([0, 0, 0.02])
                if floor_z is not None:
                    floor_z += 0.01
        return goal, center

    def rotate_base(self, dyaw, speed=0.3, timeout=20.0):
        """제자리 회전 (odom TF 폐루프)."""
        if abs(dyaw) < math.radians(1.5):
            return
        self.log(f"[base] 제자리 회전 {math.degrees(dyaw):+.1f}°")
        p0 = self.robot_pose("odom")
        if p0 is None:
            return
        msg = Twist()
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.robot_pose("odom")
            if p is None:               # TF 가 잠깐 끊긴 경우 — 이번 주기는 건너뜀
                time.sleep(0.05)
                continue
            turned = math.atan2(math.sin(p[2] - p0[2]), math.cos(p[2] - p0[2]))
            remain = dyaw - turned
            if abs(remain) < math.radians(0.8):
                break
            msg.angular.z = math.copysign(max(0.05, min(speed, abs(remain) * 1.5)), remain)
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)

    def table_edge_distance(self, station=None):
        """현재 자세 기준 작업대 앞 모서리까지의 전방 거리. 머리 카메라 깊이로 잰 값이 있으면 그것(정확, ±0.1 cm),
        없으면 map TF 와 지도 좌표 (AMCL 오차 5~10 cm)."""
        station = station or self.current_station
        e = self.table_edge_seen()          # 카메라 서버가 /table/edge 를 낼 때 (camera_process)
        if e is not None:
            return float(e["dist"])
        table = self.station_table.get(station)
        pose = self.robot_pose()
        if table is None or pose is None:
            return None
        t = self.tables[table]
        local = se2_inverse_apply(pose, t["center"])
        return float(local[0] - t["size"][1] / 2)        # 작업대 깊이(y 방향 크기)의 절반

    def dock(self, station=None):
        """Nav2 도착 후 작업대 앞까지 직진 접근 (DOCK_GAP 맞추기)."""
        station = station or self.current_station
        pose = self.robot_pose()
        if pose is None or station not in self.locations:
            return
        # 1) 작업대와 정면으로 마주 보도록 yaw 보정 (Nav2 yaw 허용오차 ±7° 보정)
        want = math.radians(self.locations[station][2])
        self.rotate_base(math.atan2(math.sin(want - pose[2]), math.cos(want - pose[2])))
        # 지도 방향으로 돈 뒤 남은 어긋남(AMCL 방향 오차 수 °)은 깊이로 잰 작업대 모서리로 바로잡는다
        e = self.table_edge_seen()
        if e is not None:
            self.log(f"[dock] 작업대 모서리(깊이) 거리 {e['dist']:.3f} m, 방향 {math.degrees(e['yaw']):+.1f}°, "
                     f"상판 {e['height']:.3f} m")
            self.rotate_base(float(e["yaw"]))
        else:
            self.log("[dock] 작업대 모서리를 깊이로 못 찾음 → 지도 좌표로")
        # 2) 작업대 앞 모서리까지 DOCK_GAP 이 되도록 직진
        d = self.table_edge_distance(station)
        if d is not None:
            self.nudge_base(d - self.DOCK_GAP)

    # ------------------------------------------------------------------ 그리퍼
    def gripper(self, side, opening, duration=1.0):
        """opening: 0.0(닫힘) ~ 0.044(열림) [m, 손가락 하나 이동량]."""
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = [f"openarm_{side}_finger_joint1"]
        p = JointTrajectoryPoint()
        p.positions = [float(opening)]
        p.time_from_start = _dur(duration)
        g.trajectory.points = [p]
        self._send_action(self.grippers[side], g, f"gripper({side})", 30.0)   # 느린 PC(시뮬이 실시간보다 느림) 대비 여유

    def open_gripper(self, side):
        self.gripper(side, 0.044)

    def close_gripper(self, side):
        self.gripper(side, 0.0)

    # ------------------------------------------------------------------ MoveIt
    @staticmethod
    def _load_srdf_states(path):
        out = {}
        root = ET.parse(path).getroot()
        for gs in root.iter("group_state"):
            out[(gs.get("group"), gs.get("name"))] = {j.get("name"): float(j.get("value")) for j in gs.iter("joint")}
        return out

    def _plan_request(self, group, constraints, vel=0.3, acc=0.3, planning_time=5.0):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = group
        r.num_planning_attempts = 5
        r.allowed_planning_time = planning_time
        r.max_velocity_scaling_factor = vel
        r.max_acceleration_scaling_factor = acc
        r.pipeline_id = "ompl"
        r.planner_id = "RRTConnectkConfigDefault"
        r.start_state.is_diff = True
        r.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def move_joints(self, group, joint_values, vel=0.3):
        c = Constraints()
        for n, v in joint_values.items():
            jc = JointConstraint()
            jc.joint_name, jc.position = n, float(v)
            jc.tolerance_above = jc.tolerance_below = 0.005
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        res = self._send_action(self.move_group, self._plan_request(group, c, vel, vel), f"move_joints({group})", 60.0)
        code = res.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise SkillError(f"MoveIt {group} 실패 (error_code={code})")

    def move_named(self, group, name, vel=0.3, tuck=True):
        """tuck=False: 바로 이동을 먼저 시도하고, 실패하면 로그를 남긴 뒤 한 번 더 계획한다."""
        key = (group, name)
        if key not in self.named_states:
            raise SkillError(f"SRDF 에 {group}/{name} 자세가 없음")
        self.log(f"[arm] {group} → '{name}'" + ("" if tuck else " (바로)"))
        if not tuck:
            try:
                self.move_joints(group, self.named_states[key], vel)
                return
            except SkillError as e:
                self.log(f"[arm] 바로 이동 실패({e}) → 경유 자세로")
        self.move_joints(group, self.named_states[key], vel)

    def compute_ik(self, side, pos, R, avoid_collisions=True, seed=None):
        """seed: {관절: 값} — IK 반복 계산의 시작점. 주지 않으면 현재 자세에서 시작."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = f"{side}_arm"
        req.ik_request.robot_state.is_diff = True
        if seed:
            req.ik_request.robot_state.joint_state = JointState(name=list(seed), position=[float(v) for v in seed.values()])
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.ik_link_name = TCP[side]
        ps = PoseStamped()
        ps.header.frame_id = BASE
        ps.pose = self._pose(pos, R)
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout = _dur(0.3)
        res = self._call(self.ik_srv, req, "compute_ik")
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        js = res.solution.joint_state
        return {n: p for n, p in zip(js.name, js.position) if n in ARM_JOINTS[side]}

    @staticmethod
    def _pose(pos, R):
        p = Pose()
        p.position.x, p.position.y, p.position.z = map(float, pos)
        q = mat_to_quat_xyzw(R)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        return p

    def move_pose(self, side, pos, R, vel=0.3):
        """TCP 를 (pos, R) 로: IK → 관절 목표 계획."""
        q = self.compute_ik(side, pos, R)
        if q is None:
            raise SkillError(f"IK 해 없음 ({side}, pos={np.round(pos, 3)})")
        self.move_joints(f"{side}_arm", q, vel)

    def cartesian(self, side, waypoints, R, vel=0.15, min_fraction=0.9, avoid_collisions=True, start_q=None):
        """TCP 를 직선으로 이동 (waypoints: base_footprint 좌표 리스트, 자세 R 고정).
        start_q: 계획 시작 관절각 (주지 않으면 현재 관절각)"""
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE
        req.start_state.is_diff = True
        if start_q:
            req.start_state.joint_state = JointState(name=list(start_q), position=[float(v) for v in start_q.values()])
        req.group_name = f"{side}_arm"
        req.link_name = TCP[side]
        req.waypoints = [self._pose(p, R) for p in waypoints]
        req.max_step = 0.01
        if hasattr(req, "jump_threshold"):
            req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        for f in ("max_velocity_scaling_factor", "max_acceleration_scaling_factor"):
            if hasattr(req, f):
                setattr(req, f, vel)
        res = self._call(self.cart_srv, req, "compute_cartesian_path")
        if res.fraction < min_fraction:
            raise SkillError(f"직선 경로 {res.fraction*100:.0f}% 만 가능 ({side})")
        traj = res.solution
        pts = traj.joint_trajectory.points
        if pts and all(p.time_from_start.sec == 0 and p.time_from_start.nanosec == 0 for p in pts[1:]):
            for i, p in enumerate(pts):   # 시간 정보가 없으면 균등 배분
                p.time_from_start = _dur(0.05 * i / max(vel, 0.05))
        g = ExecuteTrajectory.Goal()
        g.trajectory = traj
        res = self._send_action(self.exec_traj, g, "execute_trajectory", 60.0)
        if res.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise SkillError(f"직선 이동 실행 실패 code={res.result.error_code.val}")

    # ------------------------------------------------------------------ 플래닝 씬
    def set_table(self, station=None, pad=0.0):
        """현재 작업대 위치를 MoveIt 충돌 객체로 추가 (base_footprint 기준).
        pad: 이동(transit) 중 여유 [m] — 상판을 그만큼 높이고 넓혀 손이 작업대 근처를 스치지 않게 한다."""
        station = station or self.current_station
        table = self.station_table.get(station)
        if table is None:
            return
        pose = self.robot_pose()
        if pose is None:   # map TF 가 없으면 스테이션 좌표를 사용
            x, y, yaw_deg = self.locations[station]
            pose = (x, y, math.radians(yaw_deg))
        t = self.tables[table]
        local = se2_inverse_apply(pose, t["center"])
        co = CollisionObject()
        co.header.frame_id = BASE
        co.id = "work_table"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        h = t["height"] - 0.012 + pad    # 손가락 끝이 상판에 닿는 것으로 판정되지 않도록 1.2cm 낮춤
        tbl_yaw = -pose[2]                 # 작업대 축은 map 축과 평행
        e = self._edge_recent()
        if e is not None and station in self.locations:
            # 머리 카메라 깊이로 잰 앞 모서리(거리·방향)로 작업대를 놓는다 — 지도·오도메트리 어긋남(5~10 cm, 수 °)을 없앰.
            # 지도(AMCL)로 놓은 작업대가 실제보다 2.5 cm 멀어 3 cm 여유로 겨우 덮였다 (바퀴형 회귀 시험 4 비교)
            # 모서리 위의 점 (dist, 0) 에서 작업대 안쪽 법선 n = (cos θ, sin θ) 으로 깊이/2, 모서리 방향 위치는 지도 값
            th = float(e["yaw"])
            n_in, e_dir = np.array([math.cos(th), math.sin(th)]), np.array([-math.sin(th), math.cos(th)])
            p_edge = np.array([float(e["dist"]), 0.0])
            c = self.table_center_lidar(station, e)                  # 옆 위치는 LiDAR 다리 (AMCL 옆 오차를 쓰지 않음, 미션 D)
            s_lat = c if c is not None else float((np.asarray(local) - p_edge) @ e_dir)
            local = p_edge + n_in * (t["size"][1] / 2) + e_dir * s_lat
            tbl_yaw = th - math.radians(self.locations[station][2])
        box.dimensions = [float(t["size"][0]) + 2 * pad, float(t["size"][1]) + 2 * pad, float(h)]
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(local[0]), float(local[1]), h / 2
        q = yaw_to_quat(tbl_yaw)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        co.primitives = [box]
        co.primitive_poses = [p]
        co.operation = CollisionObject.ADD
        self._apply_scene([co])

    def set_boxes(self, exclude_id=None):
        """보이는 상자들을 MoveIt 충돌 객체로 추가 (접근 경로가 다른/목표 상자를 치지 않도록).
        exclude_id 상자는 제거 → 직선 접근·파지 때는 손가락이 닿아도 되게."""
        objs = []
        known = {o["marker_id"] for o in self.objects.values()}
        for mid, (p, yaw) in self.detections(max_age=2.0).items():
            if mid not in known:
                continue        # 잘못 검출된 마커(예: id 37 — 팔·그리퍼 무늬를 마커로 오인)는 무시. 팔 위에 가짜 상자가 생겨 계획이 막혔다
            co = CollisionObject()
            co.header.frame_id = BASE
            co.id = f"box_{mid}"
            if mid == exclude_id:
                co.operation = CollisionObject.REMOVE
            else:
                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = [self.box_size + 0.01] * 3
                pose = Pose()
                pose.position.x, pose.position.y = float(p[0]), float(p[1])
                pose.position.z = float(p[2] - self.box_size / 2)
                q = yaw_to_quat(yaw)
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
                co.primitives = [box]
                co.primitive_poses = [pose]
                co.operation = CollisionObject.ADD
            objs.append(co)
        if exclude_id is not None and exclude_id not in self.detections(max_age=2.0):
            # 목표 상자가 지금 안 보여도(팔이 마커를 가림) 예전에 넣은 장애물은 반드시 지운다.
            # 남아 있으면 직선 접근이 상자에 '충돌' 해 중간에 끊긴다 (실제: 27% 에서 멈춤)
            co = CollisionObject()
            co.header.frame_id = BASE
            co.id = f"box_{exclude_id}"
            co.operation = CollisionObject.REMOVE
            objs.append(co)
        if objs:
            self._apply_scene(objs)

    def clear_boxes(self):
        objs = []
        for mid in range(10):
            co = CollisionObject()
            co.header.frame_id = BASE
            co.id = f"box_{mid}"
            co.operation = CollisionObject.REMOVE
            objs.append(co)
        try:
            self._apply_scene(objs, quiet=True)
        except SkillError:
            pass

    def clear_table(self):
        co = CollisionObject()
        co.header.frame_id = BASE
        co.id = "work_table"
        co.operation = CollisionObject.REMOVE
        try:
            self._apply_scene([co], quiet=True)
        except SkillError:
            pass

    def _apply_scene(self, objects, quiet=False):
        if not self.scene_srv.wait_for_service(timeout_sec=1.0 if quiet else 5.0):
            if quiet:
                return
            raise SkillError("apply_planning_scene 서비스 없음 (move_group 실행 확인)")
        req = ApplyPlanningScene.Request()
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.world.collision_objects = objects
        req.scene = ps
        self._call(self.scene_srv, req, "apply_planning_scene")

    # ------------------------------------------------------------------ 집기 / 놓기
    DOCK_GAP = 0.40         # 도킹 후 base_footprint 원점 ~ 작업대 앞 모서리 거리 [m] (범퍼 앞 여유 약 12cm)
    APPROACH_PITCH = 45.0
    GRASP_DEPTH = 0.01      # 상자 중심보다 접근방향으로 더 들어가는 거리 [m]
    FINGER_TIP_BELOW = 0.020   # 손가락 끝이 TCP 보다 접근 방향으로 나온 길이 [m] (시뮬 접촉 기록으로 맞춤)
    FINGER_HALF_T = 0.012      # 손가락 반두께 (접근 방향에 수직) [m]
    WRIST_MID_BACK = 0.05   # 손목 카메라가 있을 때 직선 접근 중간 보정 위치: 상자 중심 앞 [m] (손가락 끝이 상자에 닿기 전)
    IDEAL_REACH = 0.52      # base_footprint 기준 상자까지 이상적 전방 거리 [m]
    REACH_TOL = 0.03        # 이 안이면 미세 이동 안 함
    # approach_rotation(roll=0) 에서 손가락이 닫히는 축(TCP y)은 '위-아래' 방향 → roll ±70~90 이 손가락을 수평으로 만들어
    # 상자 옆면을 잡는다. MuJoCo 오프라인 실험: ±70 이 가장 안정(상자 밀림 1~2cm), 0 은 모서리 집기라 밀림 3~4cm, ±20 은 놓침.
    ROLL_CANDIDATES = [70, -70, 90, -90, 50, -50, 0]
    TRANSIT_PAD = 0.03      # 팔을 옮기는 동안 작업대 충돌 객체에 두는 여유 [m]

    def _cart_fraction(self, side, q_start, R, goal):
        """q_start 에서 goal 까지 TCP 직선 경로가 몇 % 가능한지 (실행하지 않고 계산만)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE
        req.start_state.is_diff = True
        req.start_state.joint_state = JointState(name=list(q_start), position=[float(v) for v in q_start.values()])
        req.group_name = f"{side}_arm"
        req.link_name = TCP[side]
        req.waypoints = [self._pose(goal, R)]
        req.max_step = 0.01
        if hasattr(req, "jump_threshold"):
            req.jump_threshold = 0.0
        req.avoid_collisions = True
        return self._call(self.cart_srv, req, "compute_cartesian_path(검사)").fraction

    def _need_fraction(self, back):
        """직선 접근에서 최소한 상자 중심 + 0.5 cm 까지는 가야 한다 → 필요한 경로 비율.
        GRASP_DEPTH 가 1 cm 면 0.95 이상 그대로, 더 깊이 넣으라고 한 몫은 다 못 가도 된다."""
        return min(0.95, (back + 0.005) / (back + self.GRASP_DEPTH))

    STAGE_BACK = 0.08       # 대기점: 접근점에서 접근 방향 반대로 [m]
    STAGE_UP = 0.04         # 대기점: 접근점보다 위로 [m]

    def _go_to_approach(self, side, R, pre, q_pre):
        """접근점으로: 상자에서 떨어진 대기점에서 손 방향(R)을 먼저 맞추고, 대기점 → 접근점은 손 방향을 고정한 직선.
        carry → 접근점을 관절 공간 직선으로 바로 가면 손끝이 곡선을 그리고 손목이 −90° 까지 돌며 들어가
        손목 카메라 화면에서 상자가 흔들리거나 벗어났다. 대기점부터는 카메라가 상자를 계속 같은 방향에서 본다."""
        group = f"{side}_arm"
        if not self.wrist_cameras:                          # 손목 카메라가 없으면 바로
            self.move_joints(group, q_pre)
            return
        stage = pre - R[:, 2] * self.STAGE_BACK + np.array([0.0, 0.0, self.STAGE_UP])
        q_st = self.compute_ik(side, stage, R, seed=q_pre)
        if q_st is not None and self._cart_fraction(side, q_st, R, pre) >= 0.95:
            if not self._orient_then_slide(side, R, stage, q_st):
                self.move_joints(group, q_st)
            try:
                self.cartesian(side, [pre], R, vel=0.1, min_fraction=0.9)
                self.log(f"[pick] 대기점(접근점 {self.STAGE_BACK * 100:.0f} cm 뒤)에서 손 방향을 맞추고 직선으로 접근점까지")
                return
            except SkillError as e:
                self.log(f"[pick] 대기점 → 접근점 직선 실패({e}) → 관절 이동")
        else:
            self.log("[pick] 대기점 IK/직선 불가 → 접근점으로 바로")
        self.move_joints(group, q_pre)

    ORIENT_AT = (0.0, 0.35, 0.7)    # 손 방향을 맞출 곳: 지금 손끝 → 대기점 선분의 이 비율 지점 (앞에서부터 시도)

    def _orient_then_slide(self, side, R, stage, q_st):
        """손 방향(R)을 지금 손 위치 근처에서 먼저 맞추고, 대기점까지는 방향을 고정한 직선으로 간다. 성공하면 True.
        carry → 대기점을 관절 이동으로 가면 손목이 이동 내내 돌아 손목 카메라 화면이 손목 궤적을 따라 휘둘렸다
        (집기 시험 관찰). 먼저 제자리에서 돌리면 그 뒤 이동 동안 카메라가 상자 쪽을 같은 방향에서 계속 본다."""
        tf = self._tf_matrix(TCP[side])
        if tf is None:
            return False
        for a in self.ORIENT_AT:
            p = tf[0] + a * (stage - tf[0])
            q_or = self.compute_ik(side, p, R, seed=q_st)
            if q_or is None or self._cart_fraction(side, q_or, R, stage) < 0.95:
                continue
            self.move_joints(f"{side}_arm", q_or)
            try:
                self.cartesian(side, [stage], R, vel=0.1, min_fraction=0.9)
            except SkillError as e:
                self.log(f"[pick] 방향 고정 직선(→대기점) 실패({e}) → 관절 이동")
                return False
            self.log(f"[pick] 손 방향을 먼저 맞추고(지금 위치에서 대기점 쪽 {a * 100:.0f} %) 대기점까지 방향 고정 직선")
            return True
        return False

    def _solve_approach(self, side, target, back=0.10, exclude_id=None, yaw=0.0, tilt=None, yaws=None):
        """접근 자세 후보(roll × 물러선 거리)들 중 '접근점 IK + 목표 IK + 접근점→목표 직선 경로' 가 모두 되는 것 → (R, pre, q_pre).
        직선 경로까지 미리 검사하는 이유: 두 끝점의 IK 가 풀려도 중간에 관절 한계에 걸려 직선이 끊길 수 있다
        그러면 같은 roll 로 더 가까이(0.07, 0.05 m)에서 시작하거나 다른 roll 을 쓴다.
        yaw: 접근 방향을 수평으로 이만큼 돌림 [rad] — 돌아간 상자의 옆면에 수직으로 들어가 두 손가락이 옆면에 함께 닿게.
        돌리지 않으면 손가락 하나가 상자 모서리에 먼저 닿아 닫히며 상자를 옆으로 10 cm 밀었다 (무작위 배치 시험 s1c, 상자 −8°)
        tilt=(pitch, roll): 이 기울기만 쓴다 (놓기: 집을 때와 같은 기울기 → 상자가 수평). yaws: 수평 방향 후보 [rad].
        고른 (pitch, roll, yaw) 는 self._last_approach 에 남긴다."""
        best = None
        pitches = [self.APPROACH_PITCH]
        # 45° 고정 접근에서 roll ±70° (손가락 벌림 방향이 수평에서 20° 기움)는 아래쪽 손가락이 손끝 하한 계산(수평 벌림 기준)보다
        # 더 내려가 상판을 긁고, 두 손가락이 상자에 함께 닿지 못했다 (바퀴형 회귀 시험 3·4: 첫 시도 파지 실패 — 접촉 기록).
        # roll 0° (손가락이 접근 방향과 같은 수직면에서 벌어져 상자 윗면·앞면을 잡음)는 시험 3 재시도·시험 5 에서 첫 시도에 성공.
        # 예전 파지 보조는 손가락 하나만 닿아도 상자를 붙여 이 문제를 가렸다 → 작은 roll 부터
        rolls = sorted(self.ROLL_CANDIDATES, key=abs)
        if self.wrist_cameras:
            # 손목 카메라: 카메라가 위에서 상자를 보려면 roll 이 음수여야 한다. −90° 는 손가락이 정확히 수평으로 벌어져
            # 손끝 하한 계산과 맞고(위 ±70° 문제 없음), 손가락이 상자 옆면을 잡는다. 안 되면 −70°, 그다음 위 순서
            rolls = [-90, -70] + [r for r in rolls if r not in (-90, -70)]
        if tilt is not None:
            pitches, rolls = [tilt[0]], [tilt[1]]
        cands = [(p, b, r, yw) for b in (back, 0.07, 0.05) for yw in (yaws or [yaw]) for p in pitches for r in rolls]
        # 직선 접근 검사는 실제 접근 때와 같은 장면(목표 상자만 뺀)에서 해야 한다.
        # 목표 상자를 장애물로 둔 채 검사하면 손이 상자에 닿는 마지막 구간이 '충돌'로 판정돼 항상 실패한다.
        if exclude_id is not None:
            self.set_boxes(exclude_id=exclude_id)
        try:
            for pitch, b, roll, yw in cands:
                R = approach_rotation(pitch, roll, forward=(math.cos(yw), math.sin(yw), 0.0))
                pre = target - R[:, 2] * b
                q_t = self.compute_ik(side, target, R, avoid_collisions=False)
                if not q_t:
                    continue
                # 접근점 IK 를 '목표점 IK' 에서 시작해 풀면 두 자세가 같은 팔 모양(가지)이 되어 직선 경로가 이어진다.
                # 현재 자세에서 시작하면 관절 한계에 붙은 다른 가지가 나와 직선이 중간에 끊길 수 있다.
                for seed in (q_t, None):
                    q_pre = self.compute_ik(side, pre, R, seed=seed)
                    if q_pre is None:
                        continue
                    f = self._cart_fraction(side, q_pre, R, target + R[:, 2] * self.GRASP_DEPTH)
                    if f >= self._need_fraction(b):
                        self.log(f"[ik] {side} 접근 자세 pitch={pitch:.0f}° roll={roll}°"
                                 + (f" yaw={math.degrees(yw):+.0f}°" if abs(yw) > 1e-6 else "")
                                 + f", {b * 100:.0f} cm 앞에서 직선 접근")
                        self._last_approach = (pitch, roll, yw)
                        return R, pre, q_pre
                    if best is None or f > best[0]:
                        best = (f, roll, b)
        finally:
            if exclude_id is not None:
                self.set_boxes()
        if best:
            raise SkillError(f"{side} 팔 직선 접근 불가 (최대 {best[0] * 100:.0f}%, roll={best[1]}°) — 로봇 위치를 조정하세요")
        raise SkillError(f"{side} 팔로 {np.round(target, 3)} 에 접근하는 IK 해 없음 — 로봇 위치를 조정하세요")

    def _lift_clear(self, side, dz=0.05):
        """손을 지금 자리에서 위로 dz 만큼 직선으로 든다 (충돌 검사 없이).
        손가락이 상판·상자에 닿은 채로는 MoveIt 이 '시작 자세 충돌(−10)' 로 계획을 거부한다."""
        tf = self._tf_matrix(TCP[side])
        if tf is None:
            return
        try:
            self.cartesian(side, [tf[0] + np.array([0, 0, dz])], tf[1], vel=0.1, min_fraction=0.0, avoid_collisions=False)
        except SkillError as e:
            self.log(f"[pick] 손 들어 올리기 일부만: {e}")

    def _refind_box(self, side, mid):
        """재시도용: 손을 위로 뺀 뒤 home 으로 내리고 상자를 다시 잰다.
        작업대 앞에서 carry 로 접으면 손가락이 상판·옆 상자를 쳤다 (바퀴형 회귀 시험 4)"""
        self._lift_clear(side)
        self.set_table(pad=self.TRANSIT_PAD)
        self.set_boxes()
        self.move_named(f"{side}_arm", "home")
        self.set_table()
        return self.find_marker(mid, timeout=10.0) - np.array([0, 0, self.box_size / 2])

    def pick(self, obj, side=None):
        if obj not in self.objects:
            raise SkillError(f"알 수 없는 물체 '{obj}' (가능: {list(self.objects)})")
        mid = self.objects[obj]["marker_id"]
        self.dock()
        self.set_table()
        top = self.find_marker(mid)                         # 상자 윗면 마커 (base_footprint)
        center = top - np.array([0, 0, self.box_size / 2])
        self._table_top_seen = float(top[2] - self.box_size)
        self.set_table()
        side = side or ("right" if center[1] < 0 else "left")
        if self.holding[side]:
            raise SkillError(f"{side} 손에 이미 {self.holding[side]} 를 들고 있음")
        self.log(f"[pick] {obj}(id {mid}) @ base {np.round(center, 3)} → {side} 팔")
        # 카메라로 잰 상자 거리로 베이스 위치를 정밀 보정 (AMCL 오차 ±5cm 를 지각 기반으로 제거)
        if abs(center[0] - self.IDEAL_REACH) > self.REACH_TOL:
            self.nudge_base(center[0] - self.IDEAL_REACH)
            self.set_table()
            center = self.find_marker(mid) - np.array([0, 0, self.box_size / 2])
            self._table_top_seen = float(center[2] - self.box_size / 2)
            self.set_table()
            self.log(f"[pick] 보정 후 상자 위치 base {np.round(center, 3)}")
        box_yaw = 0.0
        for attempt in (1, 2):
            self.open_gripper(side)
            # 몸 쪽 carry 자세에서 출발 → 접근점까지의 경로가 상자들 위쪽으로 지나감
            # 손을 옮기는 동안(home → carry → 접근점)은 작업대에 3 cm 여유를 둔다.
            # 상판을 1.2 cm 낮춘 MoveIt 작업대로 home→carry 를 계획하면 손가락이 상판을 긁었다 (바퀴형 회귀 시험 3)
            self.set_table(pad=self.TRANSIT_PAD)
            # 접근은 carry 를 거치지 않고 home 에서 바로 한다: 작업대 앞(모서리 0.38 m)에서 home→carry 는 팔을 앞으로 크게 휘둘러
            # 손목·손가락이 상판 모서리를 쳤다 (바퀴형 회귀 시험 3·4 접촉 기록, 시뮬 재현: 모서리 0.50 m 에서도 닿음).
            # 쥔 뒤에는 수평으로 들고(_hold_level), 놓은 뒤에는 바로 home
            self.set_boxes()                                   # 계획 중에는 모든 상자가 장애물
            self.set_table()                                   # 접근 자세 계산은 실제 상판 높이로
            R, pre, q_pre = self._solve_approach(side, center, exclude_id=mid, yaw=box_yaw)
            self.set_table(pad=self.TRANSIT_PAD)               # carry → 접근점 이동도 여유를 두고
            # 시각 피드백(visual_servo): 손목 카메라가 있으면 손끝 기준 상자 위치로 손을 맞추며 들어간다
            servo = self.wrist_cameras
            if servo:
                # 접근 자세를 푸는 동안 팔이 상자를 가리기 전에 상자를 한 번 더 잰다
                try:
                    c3 = self.find_marker(mid, samples=3) - np.array([0, 0, self.box_size / 2])
                    shift = c3 - center
                    self.log(f"[pick] 접근점으로 가기 전 상자 {np.round(c3, 3)} (계획 때와 차이 {np.round(shift * 100, 1)} cm)")
                    if np.linalg.norm(shift) > 0.02:
                        # 상자가 몸통 기준으로 옮겨졌으면 접근점도 같이 옮긴다 (그대로 가면 손이 상자를 친다: 시험 11)
                        q2 = self.compute_ik(side, pre + shift, R, seed=q_pre) or self.compute_ik(side, pre + shift, R)
                        if q2 is not None:
                            pre, q_pre = pre + shift, q2
                        else:
                            # 원래 접근점으로 가면 손이 상자 옆에 붙어 상자를 밀었다 (시험 14: 14 cm 어긋난 접근점) → 접근 자세를 새로 푼다
                            self.log("[pick] 옮긴 접근점 IK 실패 → 지금 상자 위치로 접근 자세를 다시 푼다")
                            R, pre, q_pre = self._solve_approach(side, c3, exclude_id=mid, yaw=box_yaw)
                    center = c3
                except SkillError:
                    pass
            self._go_to_approach(side, R, pre, q_pre)
            bias = np.zeros(3)
            self.set_table()                                   # 직선 접근은 실제 상판 높이로
            # 손끝(TCP)을 내릴 수 있는 가장 낮은 높이: 손가락 가장 낮은 곳 = 손가락 끝 길이·sin(pitch) + 반두께·cos(pitch) 아래.
            # 상자 중심 + 1 cm 깊이 목표(TCP 상판 + 1.8 cm)는 하한(45° 에서 2.9 cm)보다 낮아 손가락이 상판을 긁고,
            # 두 손가락이 상자에 닿지 못해 파지에 실패했다 (바퀴형 회귀 시험 1·3: 첫 시도 손가락 0.001 m)
            top = self.table_h
            s_p = max(0.0, -float(R[2, 2]))
            floor_z = top + self.FINGER_TIP_BELOW * s_p + self.FINGER_HALF_T * math.sqrt(1.0 - s_p * s_p) + 0.006
            if servo:
                # 시각 피드백 1단계 (접근점): 팔 처짐으로 손이 접근점에서 몇 cm 벗어나 있다.
                # 손목 마커와 상자 마커를 함께 재서 '상자 기준' 접근점에 손을 맞춘 뒤 직선으로 들어간다.
                self.set_boxes(exclude_id=mid)
                pre_goal, center = self.visual_servo(side, mid, R, pre.copy(), center, pre - center,
                                                     floor_z=floor_z, label=" 접근점")
                bias = pre_goal - pre                          # 명령을 이만큼 옮겨야 손이 제자리에 온다 (처짐 보정량)
            self.set_boxes(exclude_id=mid)                     # 직선 접근 전에 목표 상자만 제거
            goal = center + R[:, 2] * self.GRASP_DEPTH + bias
            need = self._need_fraction(float(np.linalg.norm(center - pre)))
            if servo:
                # 보정 이동으로 팔이 q_pre 에서 조금 벗어났으므로 현재 관절각에서 계획, 안 되면 q_pre 에서
                # 보정 이동 뒤 MoveIt 안의 손 위치(명령 관절각 기준)는 처짐을 보정한 만큼 상판 쪽으로 내려가 있어
                # 실제로는 닿지 않는데도 '작업대 충돌' 로 직선이 끊겼다 (실험: 75%·67%). 실제 손 높이는 카메라로 재서
                # floor_z 위로 지키고 있으므로, 충돌 검사로 안 되면 검사 없이 같은 직선을 간다.
                goal[2] = max(goal[2], floor_z)
                if self.wrist_cameras:
                    # 손목 카메라: 직선 접근 중간(상자 중심 WRIST_MID_BACK 앞)에서 한 번 더 보정한다. 10 cm 직선 동안 팔 처짐으로
                    # 손이 옆으로 3 cm 벗어나 손가락이 상자 옆면을 쳐 4 cm 밀 수 있다.
                    # 손목 카메라는 접근 내내 상자를 보므로 손가락이 상자에 닿기 전에 옆 오차를 지울 수 있다
                    off_mid = -R[:, 2] * self.WRIST_MID_BACK
                    wp = center + off_mid + bias
                    wp[2] = max(wp[2], floor_z)
                    try:
                        self.cartesian(side, [wp], R, vel=0.1, min_fraction=0.0, avoid_collisions=False)
                        wp_goal, center = self.visual_servo(side, mid, R, wp.copy(), center, off_mid,
                                                            floor_z=floor_z, label=" 중간")
                        bias = wp_goal - (center + off_mid)
                        goal = center + R[:, 2] * self.GRASP_DEPTH + bias
                        goal[2] = max(goal[2], floor_z)
                    except SkillError as e:
                        self.log(f"[pick] 중간 보정 생략: {e}")
                try:
                    self.cartesian(side, [goal], R, min_fraction=need)
                except SkillError as e:
                    self.log(f"[pick] 직선 접근 {e} → 충돌 검사 없이 (손 높이는 카메라로 확인한 값)")
                    try:
                        self.cartesian(side, [goal], R, min_fraction=need, avoid_collisions=False)
                    except SkillError as e2:
                        # 보정한 접근점에서는 팔 관절 한계로 직선이 끊길 수 있다(시험 12: 17%) → 이번 시도는 버리고
                        # carry 로 돌아가 상자를 다시 재서 접근 자세부터 다시 푼다
                        if attempt == 2:
                            raise
                        self.log(f"[pick] 직선 접근 불가({e2}) → carry 로 돌아가 다시 인식 {attempt}/2")
                        center = self._refind_box(side, mid)
                        continue
                # 시각 피드백 2단계 (파지 위치): 손끝이 상자 중심(+1 cm 안쪽)에 올 때까지 재고 고친다
                goal, center = self.visual_servo(side, mid, R, goal, center, R[:, 2] * self.SERVO_DEPTH,
                                                 floor_z=floor_z, label=" 파지", peek=True)
            else:
                goal[2] = max(goal[2], floor_z)
                self.cartesian(side, [goal], R,   # 더 깊이: 상자가 손가락 안쪽에 오도록
                               min_fraction=need)
            self.close_gripper(side)
            time.sleep(0.5)
            # 파지 확인: 손가락이 끝까지 닫혔으면(물체 없음) 실패 — 실로봇에서도 같은 방법으로 판정
            opening = self.gripper_opening(side)
            for _ in range(10):         # /joint_states 를 아직 못 받았으면 잠깐 기다림 (모르면 성공으로 치지 않음)
                if opening is not None:
                    break
                time.sleep(0.1)
                opening = self.gripper_opening(side)
            ok = opening is not None and opening > 0.006
            try:
                self.cartesian(side, [center + np.array([0, 0, 0.10])], R, avoid_collisions=False, min_fraction=0.5)
            except SkillError as e:
                # 브리지는 궤적 끝에서 관절 오차가 0.15 rad 넘으면 실패(−4)로 알린다. 상자를 쥔 팔이 무게로 처져도 그렇게 된다
                # (무작위 배치 r2: 상자 9.4 cm 들림, 그런데 실패 보고로 집기 전체 중단). 손이 실제로 올라갔으면 계속한다
                tf = self._tf_matrix(TCP[side])
                rose = None if tf is None else float(tf[0][2] - goal[2])
                if not ok or rose is None or rose < 0.05:
                    raise
                self.log(f"[pick] 들어 올림 끝 오차로 실패 보고({e}) — 손이 {rose * 100:.0f} cm 올라갔으므로 계속")
            if ok:
                break
            got = "값 없음" if opening is None else f"{opening:.3f} m"
            self.log(f"[pick] 파지 실패(손가락 {got}) → 다시 인식해서 재시도 {attempt}/2")
            self.open_gripper(side)
            center = self._refind_box(side, mid)
            other = "left" if side == "right" else "right"
            if (center[1] > 0.05 if side == "right" else center[1] < -0.05) and not self.holding[other]:
                # 상자가 밀려 반대쪽으로 갔으면 반대 팔로 바꾼다
                self.log(f"[pick] 상자가 반대쪽({center[1]:+.2f} m)으로 가서 {other} 팔로 바꿈")
                self.move_named(f"{side}_arm", "home")
                side = other
        else:
            raise SkillError(f"{obj} 파지 실패 (2회)")
        self.clear_boxes()
        self.holding[side] = obj
        # 쥘 때의 손 기울기 (상자는 수평이었다) → 놓을 때 같은 기울기로 내려놓으면 상자도 수평
        self._grasp_tilt[side] = getattr(self, "_last_approach", None)
        # carry 로 접지 않는다 — 관절 공간으로 접으면 손목이 돌아 상자가 회전·기울었다 (바퀴형 시험 7).
        # 쥔 손의 숙임·기울기를 지키며 들어 올리고 몸 쪽으로 당긴다 (_hold_level, 머리 카메라 시야 밖)
        self._hold_level(side)
        self._held_level[side] = True
        self.log(f"[pick] 완료: {obj}")
        return side

    # 쥔 상자를 들고 다니는 자세 — 상자가 쥘 때처럼 수평인 채 위로 들고 몸 쪽으로 당긴다.
    # 예전에는 정해진 관절각 carry 로 접어 손목이 돌며 상자가 기울고 가슴에 붙었다 (사용자 지적: 물이 든 그릇이면 쏟아진다).
    # 머리 카메라를 가리지 않는 높이·자리 (시뮬 분할 렌더로 잼: 놓기 작업대 앞 0.40·0.58 m 에서 작업대 상판·다른 상자 화소 중
    # 팔·쥔 상자에 가려지는 비율). 상판 + 12.5 cm·앞 0.30 m(시험 8 자세)는 16.5 % — 놓을 쪽(쥔 팔 쪽) 상판이 안 보였다.
    # 손끝 높이 1.00 m(상판 + 30 cm)에서는 카메라 시야 위로 빠져 앞 0.28 m·옆 0.20 m 에서 0.2~0.5 %, 몸 쪽으로 다 못 당겨
    # 앞 0.45~0.52 m 에 남아도 3~9 %. 몸·머리와 닿는 자세 없음.
    HOLD_UP = 0.30                  # 상판 위 손끝 높이 [m] (작업대 0.70 m → 1.00 m)
    HOLD_FWD = 0.28                 # 몸 쪽으로 당길 목표: 몸 앞 손끝 거리 [m]
    HOLD_SIDE = 0.20                # 바깥쪽으로: 몸 중심에서 손끝 옆 거리 최소 [m] (가운데 머리 카메라 시야를 비킴)
    HOLD_YAWS = (0, 15, -15, 30, -30, 45, -45)   # 당길 때 손 수평 방향(수직축 둘레) 후보 [°] — 상자 수평과 무관
    BASE_FOOTPRINT = [(0.28, 0.26), (0.28, -0.26), (-0.27, -0.26), (-0.27, 0.26)]   # config/nav2_params.yaml 과 같은 값

    def set_footprint(self, front=None):
        """Nav2 로봇 외곽 앞쪽을 front [m] 까지 늘린다 (None: 원래대로). 바퀴형 LiDAR(0.26 m)는 몸 앞에 든 상자(0.8 m 높이)를
        못 보고, Nav2 는 외곽 밖으로 나온 상자를 모른 채 벽·작업대 가까이 돌 수 있다."""
        pts = [((max(x, front) if (front is not None and x > 0) else x), y) for x, y in self.BASE_FOOTPRINT]
        msg = Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in pts])
        for pub in self._fp_pubs:
            pub.publish(msg)

    def _hold_level(self, side):
        """쥔 상자를 수평으로 유지한 채 들어 올리고 몸 쪽으로 당긴다. 앞뒤로 옮기거나 접을 때 손이 기울지 않는다.
        ① 손 방향 그대로 수직으로 들어 올림 (손끝 = 상판 + HOLD_UP, 머리 카메라 시야 위)
        ② 몸 쪽(앞 HOLD_FWD)·바깥쪽(옆 HOLD_SIDE)으로 직선. 상자가 수평이려면 손의 숙임·기울기만 지키면 되고 수직축 둘레 방향(yaw)은 돌아도 된다 →
           yaw 후보마다 직선이 얼마나 되는지 미리 재서 가장 멀리 가는 방향으로, 되는 데까지만 간다.
           (손 방향을 완전히 고정하면 팔 관절 한계로 76~92 % 만 풀렸다: 바퀴형 시험 7. 시작·끝 방향이 수직축 회전만 다르므로
            직선 경로 중간의 손 방향 보간도 수직축 회전 → 상자는 내내 수평)"""
        tf = self._tf_matrix(TCP[side])
        if tf is None:
            return
        p, R = tf
        vel = 0.15
        top = self.table_h
        z = max(float(p[2]), top + self.HOLD_UP)
        if z - p[2] > 0.005:
            try:
                self.cartesian(side, [np.array([p[0], p[1], z])], R, vel=vel, min_fraction=0.0)
            except SkillError as e:
                self.log(f"[pick] 더 들어 올리기 생략: {e}")
        tf = self._tf_matrix(TCP[side])
        if tf is not None and tf[0][0] > self.HOLD_FWD + 0.01:
            p, R = tf
            sgn = -1.0 if side == "right" else 1.0
            goal = np.array([self.HOLD_FWD, sgn * max(abs(float(p[1])), self.HOLD_SIDE), p[2]])
            q = {n: self._joints[n] for n in ARM_JOINTS[side] if n in self._joints}
            best = None
            for deg in self.HOLD_YAWS:
                Ry = rot_axis([0.0, 0.0, 1.0], math.radians(deg)) @ R
                try:
                    f = self._cart_fraction(side, q, Ry, goal)
                except SkillError:
                    continue
                if best is None or f > best[0] + 0.02:
                    best = (f, deg, Ry)
                if f >= 0.99:
                    break
            if best is not None and best[0] > 0.05:
                try:
                    self.cartesian(side, [goal], best[2], vel=vel, min_fraction=0.0)
                    self.log(f"[pick] 수평 유지하며 몸 쪽으로 (손 수평 방향 {best[1]:+d}°, 직선 {best[0] * 100:.0f} %)")
                except SkillError as e:
                    self.log(f"[pick] 몸 쪽으로 당기기 생략: {e}")
        tf = self._tf_matrix(TCP[side])
        tip = tf[0] if tf is not None else p
        front = float(tip[0]) + self.box_size * 0.75 + 0.02      # 손끝(상자 중심) + 상자 반 대각선 + 여유
        self.set_footprint(front)
        self.log(f"[pick] 쥔 상자 수평으로 들기: 손끝 base {np.round(tip, 3)}, Nav2 외곽 앞 {front:.2f} m")

    def _choose_place(self, side, fwd, lat, edge, top_h, obs):
        """빈자리 후보(원래 자리에서 가까운 순) 중 쥘 때와 같은 기울기로 IK·직선 접근이 되는 첫 자리 → (target, R, pre, q_pre).
        놓을 때 손 기울기(pitch, roll)를 쥘 때와 같게: 다르면 쥔 상자도 그만큼 기울어진 채 내려가 모서리부터 닿고
        넘어지거나 비틀어졌다 (예: 쥘 때 roll −90°, 놓을 때 첫 후보 roll −70° → 상자 20° 기울어짐, 사용자 지적).
        같은 기울기에서 수평 방향(yaw)만 바꾸면 상자는 수평을 유지한다 → yaw 0, ±10°, ±20° 만 찾는다.
        후보를 8 개까지 보고, 모두 안 되면 다른 기울기로 첫 후보에."""
        cands = self._place_candidates(fwd, lat, side, edge, obs)
        if not cands:
            raise SkillError(f"놓을 빈자리 없음 (작업대 위 물체 점 {len(obs)} 개)")
        if abs(cands[0][0] - fwd) > 1e-3 or abs(cands[0][1] - lat) > 1e-3:
            hit = self._place_blocked(fwd, lat, obs)
            why = f"물체 {np.round(hit, 2)}" if hit is not None else "범위 밖"
            self.log(f"[place] 원래 자리 ({fwd:.2f}, {lat:+.2f}) 막힘({why}) → 빈자리 후보 {len(cands)} 개")
        gt = self._grasp_tilt.get(side)
        yaws = [0.0] + [math.radians(v) for v in (10, -10, 20, -20)]
        for fx, ly in (cands[:8] if gt is not None else []):
            tgt = np.array([fx, ly, top_h + self.box_size / 2 + 0.01])
            try:
                return (tgt,) + self._solve_approach(side, tgt + np.array([0, 0, 0.05]), back=0.06, tilt=gt[:2], yaws=yaws)
            except SkillError:
                continue
        tgt = np.array([cands[0][0], cands[0][1], top_h + self.box_size / 2 + 0.01])
        self.log("[place] 쥘 때와 같은 기울기로 놓을 자세 없음 → 다른 기울기로 (상자가 기울어질 수 있음)")
        return (tgt,) + self._solve_approach(side, tgt + np.array([0, 0, 0.05]), back=0.06)

    def place(self, station=None, side=None, offset=(0.50, None)):
        side = side or next((s for s in ("right", "left") if self.holding[s]), None)
        if side is None:
            raise SkillError("들고 있는 물체가 없음")
        station = station or self.current_station
        self.dock(station)
        self.set_table(station)
        edge = self.table_edge_distance(station)
        fwd = offset[0] if edge is None else edge + 0.12    # 모서리에서 12cm 안쪽에 내려놓기
        lat = offset[1] if offset[1] is not None else (-0.12 if side == "right" else 0.12)
        top_h = self.table_h
        obs0 = self._place_obstacles(top_h)                # 놓을 자리를 정할 때 본 물체 (팔이 나중에 가려도 기억)
        target, R, pre, q_pre = self._choose_place(side, fwd, lat, edge, top_h, obs0)
        self.log(f"[place] {self.holding[side]} → base {np.round(target, 3)} ({side})")
        self.set_table(station, pad=self.TRANSIT_PAD)
        moved = False
        if self._held_level[side]:
            # 수평으로 들고 있던 손 방향 그대로 놓을 자리 위 접근점까지 직선 (손 방향은 수평 방향만 조금 돌 수 있음 → 상자 수평 유지)
            # 들고 있던 높이 그대로 놓을 자리 위로 → 접근점까지 내려감 (두 구간 직선, 손 방향은 수직축 회전만)
            tf = self._tf_matrix(TCP[side])
            hz = max(float(pre[2]), float(tf[0][2])) if tf is not None else float(pre[2])
            try:
                self.cartesian(side, [np.array([pre[0], pre[1], hz]), pre], R,
                               vel=0.15, min_fraction=0.95)
                moved = True
            except SkillError as e:
                self.log(f"[place] 접근점까지 직선 불가({e}) → 관절 이동")
        if not moved:
            self.move_joints(f"{side}_arm", q_pre, vel=0.3)
        self.set_table(station)
        self.cartesian(side, [target], R, avoid_collisions=False)
        # 놓기: 옆 물체에 닿지 않게 조금만 연다 — 지금 손가락 벌림 + 1.2 cm (최소 상자 반폭 + 1.2 cm, 최대 끝까지).
        # 고정값(상자 반폭 + 1.2 cm)만 열면, 가파르게(pitch 55°, roll −50°) 쥐어 상자를 대각선(7.1 cm)으로 문 경우 빠지지 않아
        # 상자가 손에 붙은 채 home 까지 따라가 바닥에 떨어졌다 (시험 p2)
        held = self._joints.get(f"openarm_{side}_finger_joint1")
        opening = max(self.box_size / 2, held if held is not None else 0.0) + 0.012
        self.gripper(side, min(0.044, opening))
        time.sleep(0.5)
        # 놓은 뒤: 접근 반대 방향으로 빼고 위로 들어 올려 작업대·물체와 충분히 떨어뜨림
        try:
            self.cartesian(side, [target - R[:, 2] * 0.08 + np.array([0, 0, 0.04]),
                                  target - R[:, 2] * 0.12 + np.array([0, 0, 0.12])], R,
                           avoid_collisions=False, min_fraction=0.3)
        except SkillError as e:   # 직선으로 다 못 빼도 이미 놓았으므로 계속 (아래 carry 는 충돌 회피 계획)
            self.log(f"[place] 후퇴 경로 일부만 가능: {e} → 위로 들어 올림")
            # 손가락이 상자를 감싼 채 carry 로 가면 상자가 손가락에 끼어 끌려간다 (왼팔 시험 l1: 상자가 작업대 밖으로 떨어짐).
            # 벌린 손가락은 위로 빼면 상자에 걸리지 않으므로 먼저 수직으로 든다
            try:
                self.cartesian(side, [target + np.array([0, 0, 0.10])], R, avoid_collisions=False, min_fraction=0.3)
            except SkillError as e2:
                self.log(f"[place] 위로 들기도 일부만 가능: {e2}")
        obj = self.holding[side]
        self.holding[side] = None
        if self._held_level[side]:
            self._held_level[side] = False
            if not any(self._held_level.values()):
                self.set_footprint(None)          # 들고 있던 상자를 놓았으면 Nav2 외곽을 원래대로
        self.set_table(station, pad=self.TRANSIT_PAD)
        # 놓은 뒤 바로 home. carry 를 거치면 손이 가슴 쪽으로 갔다가 내려와 불필요하다(사용자 지적)
        # → 방금 놓은 상자도 장애물로 두고 home 까지 한 번에
        self.set_boxes()
        self.move_named(f"{side}_arm", "home", tuck=False)
        self.log(f"[place] 완료: {obj}")
        return obj

    # 놓을 빈자리 찾기 (사용자 지적: 다른 상자가 있으면 피해서 빈 곳에). 상자가 차지할 자리뿐 아니라 벌린 손가락·손이
    # 접근하며 지나가는 띠(로봇 쪽으로 PLACE_HAND_BACK)까지 비어 있어야 한다. 예전에는 상자 중심끼리 10 cm 만 보고 옆으로만 옮겼다
    PLACE_FRONT = 0.040      # 놓을 자리 중심 → 앞(작업대 안쪽) 여유: 상자 반폭 2.5 + 1.5 cm
    PLACE_HAND_BACK = 0.14   # 놓을 자리 중심 → 뒤(로봇 쪽): 손바닥·손목이 지나가는 길이
    PLACE_HALF_W = 0.07      # 옆 반폭: 놓을 때 여는 손가락(상자 반폭 + 1.2 cm, place) + 손가락 두께 + 돌아간 옆 상자 모서리 여유
    # 놓을 때 여는 폭 (손가락 하나 이동량): 끝까지(0.044) 열면 벌어지는 손가락이 9 cm 옆 상자(15° 돌아감)에 닿았다 (v3 시험 r1·r7).
    # 상자 반폭 + 1.2 cm 면 손가락이 상자에서 떨어진다
    PLACE_FWD_RANGE = (0.08, 0.22)   # 앞 모서리에서 안쪽 거리 범위 [m]
    PLACE_LAT_MAX = 0.24
    PLACE_GRID = 0.02

    def _place_obstacles(self, top_h=None):
        """작업대 위 물체 점들 (base_footprint xy, N×2).
        ① 머리 카메라 깊이 점유 칸(/table/edge occ: 상자·모르는 물건 모두) — 손에 든 상자도 점유로 잡히므로 쥔 손끝 8 cm 안은 뺀다
        ② 색·마커로 아는 상자 윗면 5 cm (팔에 가려 깊이로 안 보이는 곳 보완)"""
        pts = []
        tcps = [tf[0][:2] for tf in (self._tf_matrix(TCP[s]) for s in ("left", "right") if self.holding[s]) if tf]
        e = self._edge_recent(2.0)
        for c in (e or {}).get("occ", []):
            c = np.asarray(c, float)
            if all(np.linalg.norm(c - t) > 0.08 for t in tcps):
                pts.append(c)
        held = {self.objects.get(o, {}).get("marker_id") for o in self.holding.values() if o}
        for i, (p, _y) in self.detections(max_age=2.0).items():
            if i in held or (top_h is not None and abs(p[2] - (top_h + self.box_size)) > 0.04):
                continue
            pts += [np.array([p[0] + dx, p[1] + dy]) for dx in (-0.02, 0.0, 0.02) for dy in (-0.02, 0.0, 0.02)]
        return np.array(pts, float).reshape(-1, 2)

    def _place_blocked(self, fwd, lat, obs):
        """(fwd, lat) 에 놓을 때 상자 자리 + 손이 지나가는 띠 안에 물체 점이 있으면 가장 가까운 점, 없으면 None."""
        if len(obs) == 0:
            return None
        m = (obs[:, 0] > fwd - self.PLACE_HAND_BACK) & (obs[:, 0] < fwd + self.PLACE_FRONT) & \
            (np.abs(obs[:, 1] - lat) < self.PLACE_HALF_W)
        if not m.any():
            return None
        return obs[m][np.argmin(np.hypot(obs[m, 0] - fwd, obs[m, 1] - lat))]

    def _place_candidates(self, fwd0, lat0, side, edge, obs):
        """빈 후보 자리 (원래 자리에서 가까운 순). 집은 팔 반대쪽으로 몸 앞을 가로지르지 않게 옆 위치를 제한."""
        lo, hi = ((edge + self.PLACE_FWD_RANGE[0], edge + self.PLACE_FWD_RANGE[1]) if edge is not None
                  else (fwd0 - 0.06, fwd0 + 0.08))
        sgn = -1.0 if side == "right" else 1.0
        out = []
        for fx in np.arange(lo, hi + 1e-6, self.PLACE_GRID):
            for ly in np.arange(-self.PLACE_LAT_MAX, self.PLACE_LAT_MAX + 1e-6, self.PLACE_GRID):
                if sgn * ly < -0.04:                       # 반대쪽으로 4 cm 넘게 가로지르지 않음
                    continue
                if self._place_blocked(fx, ly, obs) is None:
                    out.append((math.hypot(1.5 * (fx - fwd0), ly - lat0), float(fx), float(ly)))
        return [(f, l) for _c, f, l in sorted(out)]

    def home_arms(self):
        for s in ("left", "right"):
            if self.holding[s] and self._held_level.get(s):
                continue                        # 쥔 손 방향 그대로 들고 있음 (_hold_level) → 상자가 기울지 않게 그대로 둔다
            if self.holding[s]:
                self.move_named(f"{s}_arm", "carry")
            else:
                self.move_named(f"{s}_arm", "home")
