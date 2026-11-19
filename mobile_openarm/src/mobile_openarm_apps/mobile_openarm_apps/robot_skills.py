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
from geometry_msgs.msg import Pose, PoseStamped, Twist
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MoveItErrorCodes,
                             PlanningScene, RobotState, RobotTrajectory)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK, GetStateValidity
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

from .geometry import approach_rotation, mat_to_quat_xyzw, quat_to_yaw, se2_inverse_apply, yaw_to_quat

try:    # 이족(TRON1) 패키지가 있으면 손목 마커 정의를 쓴다 — 바퀴형 작업공간에는 없어도 된다
    from tron1_openarm_sim import wrist_markers as WRIST
except ImportError:  # noqa: BLE001
    WRIST = None

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
        # 이름 자세(home/ready/carry/hands_up)를 읽을 SRDF. 다른 하반신(예: tron1_openarm)은 환경 변수로 자기 SRDF 를 지정
        srdf = os.environ.get("OPENARM_SRDF") or os.path.join(mv_share, "config", "mobile_openarm.srdf")
        self.named_states = self._load_srdf_states(srdf)
        # 옆걸음이 되는 하반신(이족 tron1_openarm)이면 1. 바퀴형은 설정하지 않음 → 기존 동작 그대로
        self.can_strafe = os.environ.get("OPENARM_BIPED", "0") == "1"
        if self.can_strafe:
            self.FRESH_S = 2.0          # 이족 전체 시스템: 카메라 프레임 간격이 0.5 s 를 넘음
        self._table_top_seen = None      # 이족: 카메라로 잰 상판 높이 (base_footprint 명목 좌표)
        self._box_seen_fwd = None        # 이족: 카메라로 잰 목표 상자의 전방 거리 (작업대 앞 모서리 상한 계산용)
        if self.can_strafe:
            # 이족(TRON1) 작업대 거리. 예전 어깨 1.08 m 설계에서는 상자가 팔 길이 끝이라 0.32/0.41 로 붙어 섰지만,
            # 어깨를 몸통 위로 낮춘 지금은 팔을 앞으로 든 carry 자세의 팔뚝이 앞 0.36 m 까지 나와 붙어 서면 상판 모서리에 걸린다
            # (시뮬 접촉 기록). 어깨→상자 거리는 0.45 m 로 팔 길이(0.63 m) 안이므로 바퀴형과 비슷하게 떨어져 선다
            self.DOCK_GAP = float(os.environ.get("OPENARM_DOCK_GAP", 0.42))
            self.IDEAL_REACH = float(os.environ.get("OPENARM_IDEAL_REACH", 0.52))
            self.REACH_TOL = 0.04
            # 시뮬 정답 기록: 손끝(TCP)이 목표보다 접근 방향으로 3.5 cm 덜 들어가 상자 앞 모서리만 잡고 미끄러졌다
            # (팔 처짐 약 1.5 cm + 팔을 뻗을 때 몸통 반동). 접근 방향(45° 아래)으로 더 넣으면 손가락이 상판에 걸리므로
            # 접근은 1.5 cm 까지, 그다음 같은 높이에서 수평으로 3.5 cm 더 민다 (pick 의 GRASP_PUSH)
            self.GRASP_DEPTH = float(os.environ.get("OPENARM_GRASP_DEPTH", 0.015))
            self.GRASP_PUSH = float(os.environ.get("OPENARM_GRASP_PUSH", 0.035))   # 그다음 수평으로 더 미는 거리

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
        # 손목 카메라 (TRON1 카메라 서버 tron1_cameras): 손끝(TCP) 기준 마커 위치. 카메라→손끝이 강체라 팔 처짐·몸통 흔들림과
        # 상관없이 '손 기준 상자 위치' 가 정확하다. 머리 카메라는 손이 다가가면 손이 상자를 가린다 (TRON1 검증 기록 11절)
        self._wrist = {"left": {}, "right": {}}     # side → id → (t_wall, stamp, 손끝 기준 위치[3])
        self.wrist_cameras = False                  # 손목 카메라 메시지를 한 번이라도 받았으면 True
        for side in ("left", "right"):
            self.create_subscription(String, f"/wrist_{side}_camera/aruco/detections",
                                     lambda msg, s=side: self._on_wrist(s, msg), 10, callback_group=cb)
        self.create_subscription(String, "/sim/attached", self._on_att, 10, callback_group=cb)
        # 작업대 앞 모서리 (머리 카메라 깊이, TRON1 카메라 서버 table_edge.py): 도착 뒤 작업대에 대한 정밀 정렬.
        # 어느 작업대인지는 지도 좌표로 정하고(마커를 읽으러 다가갈 필요 없음), 거리·방향·상판 높이만 이것으로 맞춘다
        self._edge = None                # (받은 시각, dict)
        self.create_subscription(String, "/table/edge", self._on_edge, 10, callback_group=cb)
        # LiDAR: 작업대 앞 다리 두 개로 작업대 중심의 옆 위치를 잰다 (table_center_lidar) — AMCL 옆 오차(최대 0.2 m)를 쓰지 않으려고
        self._scan = None
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "_scan", m), 1, callback_group=cb)
        self._joints = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10, callback_group=cb)
        self.status_pub = self.create_publisher(String, "/task/log", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.holding = {"left": None, "right": None}
        self._grasp_tilt = {"left": None, "right": None}     # 쥘 때 손 (pitch, roll, yaw) — place 에서 수평 유지
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

    def _station_lateral_offset(self, station):
        """스테이션이 작업대 중심에서 모서리 방향(로봇 왼쪽 +)으로 떨어진 거리 (지도 좌표, 보통 0)."""
        t = self.tables.get(self.station_table.get(station))
        if t is None:
            return 0.0
        sx, sy, syaw = self.locations[station][0], self.locations[station][1], math.radians(self.locations[station][2])
        return -(sx - t["center"][0]) * math.sin(syaw) + (sy - t["center"][1]) * math.cos(syaw)

    def _edge_recent(self, max_age=1.5):
        e = self._edge
        return e[1] if e is not None and time.time() - e[0] < max_age else None

    def detections(self, max_age=1.0):
        now = time.time()
        with self._det_lock:
            return {i: (p, yaw) for i, (t, p, yaw) in self._detections.items() if now - t < max_age}

    # 카메라 검출을 '새 프레임' 으로 볼 최대 나이 [s]. 바퀴형 시뮬은 0.5 s 로 충분했지만 이족 전체 시스템(시뮬+Nav2+MoveIt)은
    # 실시간의 0.3 배까지 느려져 카메라 프레임 간격이 0.5 s 를 넘었고, 상자가 보이는데도 '마커를 찾지 못함' 이 났다 (SF 미션 15)
    FRESH_S = 0.5

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

    def station_keep(self, ref, tol=0.015, timeout=30.0, gain=0.8, vmax=0.05, label="", yaw_tol=math.radians(1.5)):
        """시각 제자리 유지: 작업대 위 마커들(ref: id → base 위치)이 처음 자리로 돌아오도록 몸통을 천천히 옮긴다.
        PF 는 팔을 뻗은 채 서 있으면 다리 오도메트리가 못 보는 흐름(1분에 수~수십 cm)이 생긴다(정답 기록·오프라인 측정).
        오도메트리 대신 카메라로 본 고정 마커를 기준으로 삼으면 실로봇에서도 같은 방식으로 쓸 수 있다.
        마커가 base 기준으로 s 만큼 옮겨 보이면 몸통이 −s 만큼 간 것 → +s 방향으로 걷는다."""
        if not self.can_strafe or not ref:
            return False
        t0, msg, last_s = time.time(), Twist(), None
        moved = False
        while time.time() - t0 < timeout:
            now = self._other_markers(exclude=None)
            common = [i for i in ref if i in now]
            if not common:
                time.sleep(0.1)
                continue
            sh = np.median(np.array([now[i] - ref[i] for i in common]), axis=0)
            # 방향(yaw): 마커 2개 이상이면 마커들을 잇는 선이 base 기준으로 돈 각도 = 몸통이 반대로 돈 각도.
            # SF 는 팔을 carry 로 옮길 때 몸통이 몇 도 돌아 상자가 옆으로 8 cm 옮겨 보였다 (SF 시험 8·9)
            dth = 0.0
            if len(common) >= 2:
                a, b = common[0], common[-1]
                v0, v1 = ref[b][:2] - ref[a][:2], now[b][:2] - now[a][:2]
                if np.linalg.norm(v0) > 0.08:
                    dth = math.atan2(v0[0] * v1[1] - v0[1] * v1[0], float(v0 @ v1))
                    # 회전으로 생긴 겉보기 이동은 빼고 남은 것만 평행 이동으로
                    c0 = np.mean([ref[i][:2] for i in common], axis=0)
                    c, s_ = math.cos(dth), math.sin(dth)
                    rot_c0 = np.array([c * c0[0] - s_ * c0[1], s_ * c0[0] + c * c0[1]])
                    sh = sh.copy()
                    sh[:2] = np.mean([now[i][:2] for i in common], axis=0) - rot_c0
            last_s = sh
            if np.linalg.norm(sh[:2]) < tol and abs(dth) < yaw_tol:
                break
            if np.linalg.norm(sh[:2]) > 0.25:
                self.log(f"[station{label}] 마커 이동 {np.round(sh * 100, 1)} cm 가 너무 커서 믿지 않음")
                break
            moved = True
            v = np.clip(gain * sh[:2], -vmax, vmax)
            msg.linear.x, msg.linear.y = float(v[0]), float(v[1])
            msg.angular.z = float(np.clip(1.0 * dth, -0.15, 0.15))     # 몸통이 +Δψ 돌면 마커는 −Δψ 돌아 보인다 → dth 방향으로 돌아 되돌림
            self.cmd_pub.publish(msg)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())
        if moved:
            self.log(f"[station{label}] 마커 기준 제자리 복귀 (남은 차이 {np.round(last_s * 100, 1) if last_s is not None else '?'} cm)")
            time.sleep(0.2)
            self.wait_still(timeout=8.0)
        return True

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
        """잰 마커 위치를 odom 좌표로도 기억 — 이족은 제자리에서도 몸통이 1분에 10 cm 넘게 흘러(시험 9: 12 cm)
        base_footprint 기준 옛 측정값은 금방 틀린다. 나중에 가려져 못 볼 때 odom 으로 현재 base 좌표를 다시 계산한다."""
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
        if self.can_strafe:
            # 이족 시뮬은 실시간보다 느리고(카메라 렌더 포함 0.3~0.5 배) 걸음도 느려, 6 m 이동이 600 s 를 넘겼다 (SF 미션 시험 10)
            timeout = max(timeout, 1500.0)
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

    def wait_still(self, tol=0.004, need=2, period=0.3, timeout=20.0, yaw_tol=0.004):
        """이족: 팔을 크게 움직이면 몸통이 균형을 잡느라 5~10 cm 출렁인다 (시뮬 측정). 오도메트리 위치가
        period 마다 tol 미만으로만 바뀌는 상태가 need 번 이어질 때까지 기다린다. 바퀴형은 바로 반환.
        (예전 0.5 s × 3 번, 6 mm: 최소 1.5 s. 같은 속도 기준(1.3 cm/s)으로 0.3 s × 2 번 — 집기 한 번에 20 번 넘게 불림)"""
        if not self.can_strafe:
            return
        last, ok, t0 = None, 0, time.time()
        while time.time() - t0 < timeout and ok < need:
            p = self.robot_pose("odom")
            if p is not None and last is not None:
                # 위치뿐 아니라 방향(yaw)도 멈춰야 한다: 팔을 뻗은 PF 몸통은 제자리에서 천천히 좌우로 돌며(±5°, 수십 초 주기)
                # 0.45 m 앞의 상자가 몸통 기준으로 10 cm 넘게 옆으로 움직였다 (시험 14 카메라 기록)
                dyaw = abs(math.atan2(math.sin(p[2] - last[2]), math.cos(p[2] - last[2])))
                ok = ok + 1 if (math.hypot(p[0] - last[0], p[1] - last[1]) < tol and dyaw < yaw_tol) else 0
            last = p
            time.sleep(period)

    # ------------------------------------------------------------------ 손목 마커 시각 피드백 (이족)
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
        """같은 카메라 프레임에서 '손목 마커로 잰 손끝' 과 '상자 중심' 을 함께 잰다 (base_footprint).
        둘 다 같은 카메라로 같은 순간에 재므로 카메라 위치 오차·몸통 출렁임이 상대 오차에서 거의 지워진다.
        반환: (손끝, box_center 또는 None, tcp_fk, 손끝 출처) — 손목 마커도 정기구학도 없으면 SkillError"""
        time.sleep(0.3)
        w = self._measure_box_wrist(side, box_id)
        if w is not None:
            return w[0], w[1], w[0], "손목 카메라"
        if WRIST is None:
            raise SkillError("손목 마커 정의(tron1_openarm_sim.wrist_markers) 없음")
        tcps, boxes, seen, t0 = [], [], set(), time.time()
        while len(tcps) < samples and time.time() - t0 < timeout:
            with self._det_lock:
                items = dict(self._detections)
            tf7 = self._tf_matrix(f"openarm_{side}_link7")
            fresh = [(i, items[i]) for i in WRIST.IDS[side] if i in items and time.time() - items[i][0] < self.FRESH_S]
            if tf7 is not None and fresh:
                stamp = max(v[0] for _, v in fresh)
                if stamp not in seen:
                    seen.add(stamp)
                    est = [WRIST.tcp_from_marker(i, v[1], tf7[1]) for i, v in fresh if v[0] == stamp]
                    tcps.append(np.mean(est, axis=0))
                    b = items.get(box_id)
                    if b is not None and b[0] == stamp:
                        boxes.append(b[1] - np.array([0, 0, self.box_size / 2]))
            time.sleep(0.05)
        if not tcps and allow_fk:
            # 손목 마커가 손·상자에 가려 안 보이면(파지 위치에서 자주) 측정된 관절각의 정기구학 손끝을 쓴다.
            # 시뮬에서 카메라로 잰 손끝과 정기구학 손끝 차이는 0.1~0.6 cm (시험 7·9) → 처짐 보정에는 충분.
            fk = self._tf_matrix(TCP[side])
            with self._det_lock:
                b = self._detections.get(box_id)
            box = None if b is None or time.time() - b[0] > 1.0 else b[1] - np.array([0, 0, self.box_size / 2])
            if fk is not None:
                return fk[0], box, fk[0], "정기구학"
        if not tcps:
            with self._det_lock:
                ages = {i: round(time.time() - v[0], 1) for i, v in self._detections.items()}
            raise SkillError(f"{side} 손목 마커가 카메라에 안 보임 (최근 검출 id:경과초 {ages}, link7 TF {'있음' if tf7 is not None else '없음'})")
        fk = self._tf_matrix(TCP[side])
        return (np.median(np.array(tcps), axis=0), np.median(np.array(boxes), axis=0) if boxes else None,
                None if fk is None else fk[0], "손목 마커")

    def visual_servo(self, side, box_id, R, goal, center, offset, floor_z=None, label="", peek=False):
        """손끝이 '상자 중심 + offset' 에 오도록 카메라로 재며 고친다 (closed-loop).
        goal: 지금까지 MoveIt 에 명령한 손끝 목표. 팔이 중력에 처지는 만큼 실제 손끝은 목표에 못 미치므로
        '명령 목표' 를 오차만큼 누적해서 옮긴다 (적분) → 처짐이 매번 같으면 2~3 번에 수렴.
        반환: (최종 명령 목표, 최신 상자 중심)"""
        peeked = False
        for it in range(1, self.SERVO_ITERS + 1):
            self.wait_still(timeout=8.0)
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

    def strafe_base(self, dy, speed=0.08, timeout=15.0):
        """옆걸음 (+왼쪽). 이족(OPENARM_BIPED=1)만 가능 — 바퀴형(차동 구동)은 옆으로 못 간다. odom TF 폐루프."""
        dy = float(np.clip(dy, -0.25, 0.25))
        if not self.can_strafe or abs(dy) < 0.01:
            return
        self.log(f"[base] 옆걸음 {dy:+.2f} m")
        p0 = self.robot_pose("odom")
        msg = Twist()
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.robot_pose("odom")
            moved = 0.0 if (p is None or p0 is None) else math.hypot(p[0] - p0[0], p[1] - p0[1])
            remain = abs(dy) - moved
            if remain < 0.005:
                break
            msg.linear.y = math.copysign(max(0.02, min(speed, remain * 1.5)), dy)
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.8)

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
        if self.can_strafe:
            e = self.table_edge_seen()
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
        self._box_seen_fwd = None       # 새 작업대 — 이전 작업대에서 잰 상자 거리는 쓰지 않음
        pose = self.robot_pose()
        if pose is None or station not in self.locations:
            return
        # 1) 작업대와 정면으로 마주 보도록 yaw 보정 (Nav2 yaw 허용오차 ±7° 보정)
        want = math.radians(self.locations[station][2])
        self.rotate_base(math.atan2(math.sin(want - pose[2]), math.cos(want - pose[2])))
        if self.can_strafe:
            # 지도 방향으로 돈 뒤 남은 어긋남(AMCL 방향 오차 수 °)은 깊이로 잰 작업대 모서리로 바로잡는다
            e = self.table_edge_seen()
            if e is not None:
                self.log(f"[dock] 작업대 모서리(깊이) 거리 {e['dist']:.3f} m, 방향 {math.degrees(e['yaw']):+.1f}°, "
                         f"상판 {e['height']:.3f} m")
                self.rotate_base(float(e["yaw"]))
            else:
                self.log("[dock] 작업대 모서리를 깊이로 못 찾음 → 지도 좌표로")
        if self.can_strafe:
            # 옆 위치: Nav2 '도착' 은 목표에서 0.25 m(xy 허용) + AMCL 오차만큼 옆으로 벗어날 수 있다. 미션 22 에서 스테이션보다
            # 35 cm 옆에 서서 집을 상자(로봇 오른쪽 0.47 m)가 머리 카메라 시야(옆 ±0.3 m) 밖 → '마커 못 찾음' 으로 실패.
            # LiDAR 로 작업대 앞 다리를 찾아 작업대 중심에 대해 맞춘다 (table_center_lidar). 못 찾을 때만 지도(AMCL) 값 —
            # AMCL 은 옆으로 0.2 m 틀릴 수 있어(미션 D: 0.22 m 반대로 옆걸음 → 넘어짐) 0.15 m 넘게 벗어난 것으로 보일 때만, 절반만 간다.
            # 옆걸음은 odom 폐루프, 한 번에 0.25 m 까지라 나눠서
            c = self.table_center_lidar(station, self.table_edge_seen())
            if c is not None:
                move = c + self._station_lateral_offset(station)            # +: 왼쪽으로
                src = f"LiDAR 작업대 다리: 중심 {c:+.2f} m"
            else:
                p = self.robot_pose()
                sx, sy, syaw = self.locations[station][0], self.locations[station][1], math.radians(self.locations[station][2])
                lat = 0.0 if p is None else -(p[0] - sx) * math.sin(syaw) + (p[1] - sy) * math.cos(syaw)
                move = -0.5 * lat if abs(lat) > 0.15 else 0.0
                src = f"지도(AMCL): 스테이션보다 옆으로 {lat:+.2f} m"
            if abs(move) >= 0.06:
                self.log(f"[dock] {src} → 옆걸음 {move:+.2f} m")
                rest = float(np.clip(move, -0.5, 0.5))
                while abs(rest) >= 0.01:
                    step = float(np.clip(rest, -0.25, 0.25))
                    self.strafe_base(step)
                    rest -= step
            else:
                self.log(f"[dock] {src} → 옆 위치 그대로")
            self.set_table(station, pad=self.TRANSIT_PAD)   # 이후 팔 이동 경로가 작업대를 피하도록 먼저 충돌 객체를 넣는다
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

    def _move_joints_direct(self, group, joint_values, vel=0.3, n=30):
        """이족 전용: 지금 관절각에서 목표까지 관절 공간 직선(모든 관절을 함께 같은 비율로)으로 움직인다.
        OMPL(RRTConnect)은 팔을 옆·앞으로 크게 돌려 가는 경로를 자주 만들었고(손이 몸 옆 0.6 m 까지 나감),
        어깨가 상판 바로 위인 이족에서는 그 경로가 작업대 모서리를 쓸거나 몸통을 크게 흔들었다.
        중간 자세 n 개를 MoveIt 충돌 검사(/check_state_validity)로 확인해 모두 안전할 때만 쓰고, 아니면 False."""
        names = list(joint_values)
        cur = [self._joints.get(nm) for nm in names]
        if any(v is None for v in cur):
            return False
        cur, goal = np.array(cur), np.array([joint_values[nm] for nm in names])
        if not hasattr(self, "_valid_srv"):
            self._valid_srv = self.create_client(GetStateValidity, "/check_state_validity")
        for s in np.linspace(0, 1, n)[1:]:
            req = GetStateValidity.Request()
            req.group_name = group
            req.robot_state.is_diff = True
            req.robot_state.joint_state = JointState(name=names, position=[float(v) for v in cur + (goal - cur) * s])
            if not self._call(self._valid_srv, req, "check_state_validity").valid:
                return False
        dur = max(1.5, float(np.max(np.abs(goal - cur))) / (1.0 * vel))     # 가장 많이 도는 관절 기준 1 rad/s × vel
        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = names
        for s in np.linspace(0, 1, n):
            pt = JointTrajectoryPoint()
            sm = s * s * (3 - 2 * s)                                          # 부드럽게 출발·정지
            pt.positions = [float(v) for v in cur + (goal - cur) * sm]
            pt.time_from_start = _dur(dur * s)
            traj.joint_trajectory.points.append(pt)
        g = ExecuteTrajectory.Goal()
        g.trajectory = traj
        res = self._send_action(self.exec_traj, g, f"move_direct({group})", 180.0)
        if res.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise SkillError(f"직접 관절 이동 {group} 실패 (error_code={res.result.error_code.val})")
        return True

    def move_joints(self, group, joint_values, vel=0.3):
        if self.can_strafe:
            # 이족: 팔을 빨리 옮기면 반동으로 PF 몸통이 옆으로 7~12 cm 밀렸다가 제자리 유지가 되돌린다(시험 17 정답 기록).
            # 06장 실험: 팔을 느리게 하면 흔들림이 준다 → 기본 속도를 절반으로
            vel = min(vel, self.BIPED_ARM_VEL)
        if self.can_strafe and self._move_joints_direct(group, joint_values, vel):
            return
        c = Constraints()
        for n, v in joint_values.items():
            jc = JointConstraint()
            jc.joint_name, jc.position = n, float(v)
            jc.tolerance_above = jc.tolerance_below = 0.005
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        res = self._send_action(self.move_group, self._plan_request(group, c, vel, vel), f"move_joints({group})", 60.0)
        code = res.result.error_code.val
        if code == MoveItErrorCodes.START_STATE_IN_COLLISION and self.can_strafe:
            # 이족: 몸통이 흔들려 MoveIt 안의 손이 부풀린 작업대·상자 장애물과 겹친 채로 시작하면 계획을 거부한다
            # (SF 미션 시험 14: carry → 접근점 −10) → 상자 장애물을 빼고 여유 없는 실제 상판으로 한 번 더
            self.log(f"[arm] 시작 자세가 장애물과 겹침(−10) → 상자 장애물·작업대 여유 없이 다시 계획")
            self.clear_boxes()
            self.set_table()
            res = self._send_action(self.move_group, self._plan_request(group, c, vel, vel), f"move_joints({group})", 60.0)
            code = res.result.error_code.val
            self.set_boxes()
        if code != MoveItErrorCodes.SUCCESS:
            raise SkillError(f"MoveIt {group} 실패 (error_code={code})")

    # 이족: home(팔 내림) ↔ carry/ready(팔 앞에 듦) 사이 경유 자세 (오른팔 기준, 왼팔은 거울상).
    # 바로 옮기면 아래팔이 앞으로 휘둘려 작업대 높이에서 몸 앞 0.475 m 까지 나가 상판 모서리(약 0.38 m)를 쳤다
    # (SF 미션 시험 9: carry 이동 시간 초과·기울기 22°). 위팔을 뒤로 젖히고 팔꿈치를 먼저 굽히면 앞으로 덜 나간다 (격자 탐색:
    # j1 −0.6 → 0.343 m, −0.4 → 0.371 m). −0.6 은 무게중심이 뒤로 가 SF 가 발을 디디며 20 cm 뒤로 밀렸고(오프라인 측정),
    # −0.5 는 1 cm 만 움직였다 → −0.5
    # 젖힐 때 팔꿈치가 TRON1 몸통 옆 손잡이(base 아래 0.10 m, 옆 0.17 m) 높이를 지나므로 어깨를 0.55 rad 벌린 채 젖히고 펴며,
    # 벌림은 젖힘을 다 되돌린 뒤(j1 0.1)에만 바꾼다. 예전 경로(벌림 0.2)는 팔꿈치·아래팔이 손잡이를 관통했다(틈 0 cm).
    # home→carry 순서: 더 벌림 → 젖히기 → 팔꿈치 굽힘 → 앞으로 → carry. 전 구간 몸통·손잡이·다리와 3.0 cm 이상,
    # 작업대 높이에서 몸 앞 0.39 m 이내 (격자 탐색, tron1_openarm_sim/tools/mech_fix.py 손잡이 치수)
    TUCK_RIGHT = [[0.1, 0.55, 0.0, 0.0, 0.0, 0.0, 0.0], [-0.5, 0.55, 0.5, 0.3, 0.0, 0.0, 0.0],
                  [-0.5, 0.55, 0.5, 2.4, 0.0, 0.0, 0.0], [0.3, 0.55, -0.1, 2.4, 0.0, 0.0, 0.0]]

    def _tuck_path(self, group, name):
        """경유 자세 목록 (관절 dict). 필요 없으면 []."""
        if not self.can_strafe or group not in ("left_arm", "right_arm"):
            return []
        side = group.split("_")[0]
        names = ARM_JOINTS[side]
        cur = [self._joints.get(n) for n in names]
        if any(v is None for v in cur):
            return []
        elbow_bent = cur[3] > 1.2
        sgn = 1 if side == "right" else -1
        near_home = abs(cur[0] - 0.1 * sgn) < 0.25 and cur[3] < 0.8 and abs(cur[1] - 0.4 * sgn) < 0.3   # home (arm_poses.py)
        if name in ("carry", "ready") and near_home:
            # 팔을 내린 home 근처에서만 경유. 잡기 자세(팔을 앞으로 뻗음)에서 carry 로 접을 때도 경유했더니 팔이 먼저 아래로 내려가
            # 쥔 상자가 LiDAR 높이(0.58 m)로 내려와 Nav2 가 그것을 바로 앞 장애물로 보고 뒷걸음질쳤다 (SF 미션 시험 12)
            seq = self.TUCK_RIGHT                       # home → 젖히기 → 팔꿈치 굽힘 → carry
        elif name == "home" and elbow_bent and cur[0] * (1 if side == "right" else -1) > -0.3:
            seq = self.TUCK_RIGHT[::-1]                 # carry → 팔꿈치 굽힌 채 젖히기 → 펴기 → home
        else:
            return []
        out = []
        for q in seq:
            qq = q if side == "right" else [-q[0], -q[1], -q[2], q[3], -q[4], q[5], -q[6]]
            out.append({n: float(v) for n, v in zip(names, qq)})
        return out

    def move_named(self, group, name, vel=0.3, tuck=True):
        """tuck=False: 경유 자세 없이 MoveIt 계획 한 번으로 (작업대·상자는 충돌 객체로 피함). 실패하면 경유 자세로 다시."""
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
        for wp in self._tuck_path(group, name):
            try:
                self.move_joints(group, wp, vel)
            except SkillError as e:
                self.log(f"[arm] 경유 자세 실패({e}) → 바로 이동")
                break
        try:
            self.move_joints(group, self.named_states[key], vel)
        except SkillError as e:
            if not self.can_strafe:
                raise
            # 이족: 이동용 여유(TRANSIT_PAD)로 부풀린 작업대 때문에 계획이 막히는 경우가 있다(시험 19: carry 계획 실패 −2)
            # → 여유 없이 실제 상판으로 한 번 더 계획
            self.log(f"[arm] {e} → 작업대 여유 없이 다시 계획")
            self.set_table()
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
        start_q: 계획 시작 관절각 (주지 않으면 현재 관절각). 이족은 팔이 중력에 1~3 cm 처져 현재 관절각에서 계획하면
        사전 검사(접근점 관절각 기준)와 다른 팔 모양이 나와 직선이 중간에 끊겼다(42%) → 접근점 관절각에서 계획"""
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
        res = self._send_action(self.exec_traj, g, "execute_trajectory", 180.0 if self.can_strafe else 60.0)   # 이족 시뮬은 실시간보다 느림
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
        h = t["height"] - 0.012          # 손가락 끝이 상판에 닿는 것으로 판정되지 않도록 1.2cm 낮춤
        if self.can_strafe and self._table_top_seen is not None:
            # 이족: base_link 가 명목값(0.68 m, 수평)과 달리 1~3 cm 높고 3~4° 앞으로 기울어 있다. 팔과 카메라는 base_link 에
            # 붙어 있으므로 카메라로 잰 상자 위치는 팔 기준으로 정확하지만, 지도에서 가져온 상판 높이는 그만큼 어긋나
            # 상자 중심이 상판 '안'에 있는 것처럼 보였다(IK 충돌). → 카메라로 잰 상자 바닥 높이를 상판 높이로 쓴다
            # 바퀴형은 손가락 끝이 상판에 닿아도 되게 1.2 cm 낮추지만, 이족은 상판을 스치면 몸통이 밀려 넘어질 뻔했다
            # (SF: 손가락-작업대 접촉 뒤 몸통이 45° 돌고 0.53 m 로 주저앉음) → 실제 높이 + 0.5 cm 로 두어 스치지 않는 접근만 고른다
            h = self._table_top_seen + 0.005
        h += pad
        if self.can_strafe and self._box_seen_fwd is not None:
            # 이족: 지도·오도메트리로 계산한 작업대 위치가 실제보다 9 cm 멀었다(시험 10: 실제 모서리 0.33 m, 계획 0.42 m —
            # 제자리 걸음 미끄러짐). 그래서 home→carry 경로의 손가락이 상판 모서리를 쳤다. 카메라로 잰 상자는 정확하므로
            # '상자는 모서리에서 최소 BOX_EDGE_MIN 안쪽에 있다' 는 가정으로 모서리 위치의 상한을 둔다 (가까운 쪽으로만 옮김).
            # 반대로 오도메트리가 실제보다 앞서 있으면(SF 미세 이동·서기 중 방향 바로잡기 걸음을 다리 오도메트리가 못 봄, 20~33 cm)
            # 작업대가 몸통 '안'에 놓여 다리·몸통·팔 전체가 충돌(MoveIt −10)했다 (검증 기록 10절) → 하한(BOX_EDGE_MAX)도 둔다
            edge_map = float(local[0] - t["size"][1] / 2)
            edge = float(np.clip(edge_map, self._box_seen_fwd - self.BOX_EDGE_MAX, self._box_seen_fwd - self.BOX_EDGE_MIN))
            if edge != edge_map:
                local = np.array([local[0] + edge - edge_map, local[1]])
        tbl_yaw = -pose[2]                 # 작업대 축은 map 축과 평행
        e = self._edge_recent() if self.can_strafe else None
        if e is not None and station in self.locations:
            # 이족: 머리 카메라 깊이로 잰 앞 모서리(거리·방향)로 작업대를 놓는다 — 지도·오도메트리 어긋남(5~10 cm, 수 °)을 없앰.
            # 모서리 위의 점 (dist, 0) 에서 작업대 안쪽 법선 n = (cos θ, sin θ) 으로 깊이/2, 모서리 방향 위치는 지도 값
            th = float(e["yaw"])
            n_in, e_dir = np.array([math.cos(th), math.sin(th)]), np.array([-math.sin(th), math.cos(th)])
            p_edge = np.array([float(e["dist"]), 0.0])
            c = self.table_center_lidar(station, e)                  # 옆 위치는 LiDAR 다리 (AMCL 옆 오차를 쓰지 않음, 미션 D)
            s_lat = c if c is not None else float((np.asarray(local) - p_edge) @ e_dir)
            local = p_edge + n_in * (t["size"][1] / 2) + e_dir * s_lat
            tbl_yaw = th - math.radians(self.locations[station][2])
            if self._table_top_seen is None:
                h = float(e["height"]) + 0.005 + pad
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
    REACH_TOL = 0.03        # 이 안이면 미세 이동 안 함 (이족은 생성자에서 0.04: 미세 이동 한 번에 7~10 s, 0.54~0.57 m 에서도 집기 성공)
    # approach_rotation(roll=0) 에서 손가락이 닫히는 축(TCP y)은 '위-아래' 방향 → roll ±70~90 이 손가락을 수평으로 만들어
    # 상자 옆면을 잡는다. MuJoCo 오프라인 실험: ±70 이 가장 안정(상자 밀림 1~2cm), 0 은 모서리 집기라 밀림 3~4cm, ±20 은 놓침.
    ROLL_CANDIDATES = [70, -70, 90, -90, 50, -50, 0]
    # 이족 전용: 상자가 옆으로 이만큼 넘게 치우쳐 있으면 옆걸음으로 LATERAL_IDEAL 에 맞춘다.
    # 바퀴형 도킹에서 상자는 보통 팔 쪽으로 0.10~0.15 m. 이족은 AMCL 옆 오차(±0.15 m)로 0.27~0.31 m 가 되어
    # 직선 접근이 64% 에서 막혔다 (실험). 바퀴형은 옆으로 못 가므로 이 단계를 건너뛴다.
    TRANSIT_PAD = 0.03
    BIPED_ARM_VEL = 0.15    # 이족: 팔 관절 이동 기본 속도 배율 (바퀴형 0.3)
    PRE_LIFT = 0.07         # 이족: 접근점을 이만큼 위에 둠 [m] (4 cm 로는 SF 시험 11 에서도 손가락이 상판에 닿음)
    HOLD_VEL = 0.1          # 이족: 물체를 쥔 팔을 움직이는 속도 배율 (보통 0.3)
    BOX_EDGE_MIN = 0.10     # 이족: 상자는 작업대 앞 모서리에서 적어도 이만큼 안쪽에 놓여 있다고 가정 [m] (시뮬 월드 0.12~0.20)
    BOX_EDGE_MAX = 0.25     # 이족: 상자는 작업대 앞 모서리에서 이보다 안쪽 깊이 있지 않다고 가정 [m]
    GRASP_PUSH = 0.0
    LATERAL_OK = 0.18
    LATERAL_IDEAL = 0.12

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
        GRASP_DEPTH 가 1 cm(바퀴형)면 0.95 이상 그대로, 이족처럼 더 깊이 넣으라고 한 몫은 다 못 가도 된다."""
        return min(0.95, (back + 0.005) / (back + self.GRASP_DEPTH))

    STAGE_BACK = 0.08       # 대기점: 접근점에서 접근 방향 반대로 [m]
    STAGE_UP = 0.04         # 대기점: 접근점보다 위로 [m]

    def _go_to_approach(self, side, R, pre, q_pre):
        """접근점으로: 상자에서 떨어진 대기점에서 손 방향(R)을 먼저 맞추고, 대기점 → 접근점은 손 방향을 고정한 직선.
        carry → 접근점을 관절 공간 직선으로 바로 가면 손끝이 곡선을 그리고 손목이 −90° 까지 돌며 들어가
        손목 카메라 화면에서 상자가 흔들리거나 벗어났다. 대기점부터는 카메라가 상자를 계속 같은 방향에서 본다."""
        group = f"{side}_arm"
        if not self.can_strafe:
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
        (TRON1 이족에서 실제로 64%·45% 에서 끊김). 그러면 같은 roll 로 더 가까이(0.07, 0.05 m)에서 시작하거나 다른 roll 을 쓴다.
        yaw: 접근 방향을 수평으로 이만큼 돌림 [rad] — 돌아간 상자의 옆면에 수직으로 들어가 두 손가락이 옆면에 함께 닿게.
        돌리지 않으면 손가락 하나가 상자 모서리에 먼저 닿아 닫히며 상자를 옆으로 10 cm 밀었다 (무작위 배치 시험 s1c, 상자 −8°)
        tilt=(pitch, roll): 이 기울기만 쓴다 (놓기: 집을 때와 같은 기울기 → 상자가 수평). yaws: 수평 방향 후보 [rad].
        고른 (pitch, roll, yaw) 는 self._last_approach 에 남긴다."""
        best = None
        # 이족: 어깨를 몸통 위로 낮춘 설계(어깨 0.78~0.83 m, 상자 0.69 m)에서는 거의 수평 접근이 자연스럽고,
        # 45° 로 내려오면 아래쪽 손가락이 상판에 걸린다 → 낮은 pitch 부터 시도
        pitches = ([20.0, 30.0, self.APPROACH_PITCH, 55.0] if self.can_strafe else [self.APPROACH_PITCH])
        rolls = self.ROLL_CANDIDATES
        if not self.can_strafe:
            # 바퀴형(45° 고정): roll ±70° (손가락 벌림 방향이 수평에서 20° 기움)는 아래쪽 손가락이 손끝 하한 계산(수평 벌림 기준)보다
            # 더 내려가 상판을 긁고, 두 손가락이 상자에 함께 닿지 못했다 (바퀴형 회귀 시험 3·4: 첫 시도 파지 실패 — 접촉 기록).
            # roll 0° (손가락이 접근 방향과 같은 수직면에서 벌어져 상자 윗면·앞면을 잡음)는 시험 3 재시도·시험 5 에서 첫 시도에 성공.
            # 예전 파지 보조는 손가락 하나만 닿아도 상자를 붙여 이 문제를 가렸다 → 작은 roll 부터
            rolls = sorted(rolls, key=abs)
        if self.wrist_cameras:
            # 손목 카메라는 손의 −x 쪽에 있어 roll 이 음수(−70~−90°)일 때 위에서 상자를 내려다본다 → 음수 roll 먼저
            rolls = [r for r in rolls if r < 0] + [r for r in rolls if r == 0] + [r for r in rolls if r > 0]
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
                if self.can_strafe:
                    # 이족: 팔을 뻗으면 몸통이 앞으로 숙여져(SF 최대 7°) 손이 3~5 cm 내려가 접근점에서 손가락이 상판에 닿았다
                    # (SF 시험 8). 접근점을 PRE_LIFT 만큼 위에 두고 비스듬히 내려가며 들어간다
                    pre = pre + np.array([0.0, 0.0, self.PRE_LIFT])
                q_t = self.compute_ik(side, target, R, avoid_collisions=False)
                if not q_t:
                    continue
                # 접근점 IK 를 '목표점 IK' 에서 시작해 풀면 두 자세가 같은 팔 모양(가지)이 되어 직선 경로가 이어진다.
                # 현재 자세에서 시작하면 관절 한계에 붙은 다른 가지가 나와 직선이 중간에 끊겼다 (이족에서 42~67%).
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
        손가락이 상판·상자에 닿은 채로는 MoveIt 이 '시작 자세 충돌(−10)' 로 계획을 거부한다 (SF 시험 11)."""
        tf = self._tf_matrix(TCP[side])
        if tf is None:
            return
        try:
            self.cartesian(side, [tf[0] + np.array([0, 0, dz])], tf[1], vel=0.1, min_fraction=0.0, avoid_collisions=False)
        except SkillError as e:
            self.log(f"[pick] 손 들어 올리기 일부만: {e}")

    def _box_yaw(self, mid, samples=5, timeout=5.0):
        """상자 마커의 yaw (base 기준). 상자는 작업대와 나란히 놓여 있으므로 90° 주기로 접어 ±45° 로."""
        vals, t0, last = [], time.time(), None
        while len(vals) < samples and time.time() - t0 < timeout:
            with self._det_lock:
                it = self._detections.get(mid)
            if it and it[0] != last and time.time() - it[0] < self.FRESH_S:
                last = it[0]
                y = (it[2] + math.pi / 4) % (math.pi / 2) - math.pi / 4
                vals.append(y)
            time.sleep(0.05)
        return float(np.median(vals)) if vals else None

    def _table_yaw(self, samples=5, timeout=5.0, agree=math.radians(6)):
        """작업대 방향 (base 기준 yaw) — 보이는 상자 마커들의 방향이 서로 맞을 때만.
        상자 하나의 방향은 상자가 놓인 방향일 뿐 작업대 방향이 아니다: 무작위 배치 시험(s1)에서 −8° 돌아간 상자에
        몸통을 맞추느라 −11° 돌고 옆걸음을 4 번 하며 11 cm 밀려나, 놓을 때 다른 상자를 쳤다.
        마커 2 개 이상의 방향이 agree 안에서 같으면 '작업대와 나란히 놓인 상자들' 로 보고 그 중앙값, 아니면 None."""
        per = {}
        t0 = time.time()
        seen = {}
        while time.time() - t0 < timeout and (not per or min(len(v) for v in per.values()) < samples):
            with self._det_lock:
                items = dict(self._detections)
            now = time.time()
            for i, (t, _p, yaw) in items.items():
                if now - t < self.FRESH_S and seen.get(i) != t:
                    seen[i] = t
                    per.setdefault(i, []).append((yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4)
            time.sleep(0.05)
        meds = [float(np.median(v)) for v in per.values() if len(v) >= 2]
        if len(meds) < 2:
            return None, meds
        if max(meds) - min(meds) > agree:
            return None, meds
        return float(np.median(meds)), meds

    def _visual_dock(self, mid, top):
        """이족: Nav2 도착 오차(SF 미션 시험 3: 옆 0.36 m, 방향 20°)를 카메라로 바로잡는 도킹.
        ① 상자 마커들의 방향이 서로 맞으면(작업대와 나란히 놓인 상자들) 그 방향으로 몸통을 작업대와 직각으로 맞춘다.
           상자들이 제각각 돌아가 있으면 돌리지 않는다 (_table_yaw)
        ② 상자가 옆으로 많이 벗어나 있으면 앞으로 다가가기 '전에' 옆걸음으로 맞춘다
           (먼저 다가가면 상자가 카메라 시야 밖으로 나가 '마커를 찾지 못함' 으로 실패했다)"""
        for _ in range(3):
            yaw, meds = self._table_yaw()
            if yaw is None:
                if len(meds) >= 2:
                    self.log(f"[dock] 상자 방향이 제각각 ({', '.join(f'{math.degrees(v):+.0f}' for v in meds)}°) → 몸통 방향은 그대로")
                break
            if abs(yaw) < math.radians(4):
                break
            self.log(f"[dock] 상자 마커들 기준 방향 오차 {math.degrees(yaw):+.1f}° → 회전")
            self.rotate_base(yaw)
            self.wait_still(timeout=8.0)
            top = self.find_marker(mid)
        for _ in range(4):
            if abs(top[1]) <= self.LATERAL_OK:
                break
            self.strafe_base(top[1] - math.copysign(self.LATERAL_IDEAL, top[1]))
            self.wait_still(timeout=8.0)
            top = self.find_marker(mid)
            self.log(f"[dock] 옆걸음 뒤 상자 base {np.round(top, 3)}")
        return top

    def _refind_box(self, side, mid):
        """재시도용: 팔을 carry 로 접고 상자를 다시 잰다. 안 보이면(팔·몸통 회전에 가려짐) home 으로 내려 한 번 더.
        바퀴형: 손을 위로 뺀 뒤 home 으로 내리고 잰다 — 작업대 앞에서 carry 로 접으면 손가락이 상판·옆 상자를 쳤다 (바퀴형 회귀 시험 4)"""
        self._lift_clear(side)
        if not self.can_strafe:
            self.set_table(pad=self.TRANSIT_PAD)
            self.set_boxes()
            self.move_named(f"{side}_arm", "home")
            self.set_table()
            return self.find_marker(mid, timeout=10.0) - np.array([0, 0, self.box_size / 2])
        self.move_named(f"{side}_arm", "carry")
        self.wait_still(timeout=8.0)
        try:
            top = self.find_marker(mid)
        except SkillError:
            self.log("[pick] carry 자세에서 상자가 안 보임 → 팔을 내리고 다시 찾음")
            self.move_named(f"{side}_arm", "home")
            self.wait_still(timeout=8.0)
            top = self.find_marker(mid, timeout=10.0)
            self.move_named(f"{side}_arm", "carry")
        center = top - np.array([0, 0, self.box_size / 2])
        if self.can_strafe:
            self._table_top_seen = float(center[2] - self.box_size / 2)
            self._box_seen_fwd = float(center[0])
            self.set_table()
        return center

    def pick(self, obj, side=None):
        if obj not in self.objects:
            raise SkillError(f"알 수 없는 물체 '{obj}' (가능: {list(self.objects)})")
        mid = self.objects[obj]["marker_id"]
        self.dock()
        self.set_table()
        top = self.find_marker(mid)                         # 상자 윗면 마커 (base_footprint)
        if self.can_strafe:
            top = self._visual_dock(mid, top)
        center = top - np.array([0, 0, self.box_size / 2])
        self._table_top_seen = float(top[2] - self.box_size)
        self._box_seen_fwd = float(top[0])
        self.set_table()                                    # 이족: 잰 상판 높이로 작업대 충돌 객체 갱신
        side = side or ("right" if center[1] < 0 else "left")
        if self.holding[side]:
            raise SkillError(f"{side} 손에 이미 {self.holding[side]} 를 들고 있음")
        self.log(f"[pick] {obj}(id {mid}) @ base {np.round(center, 3)} → {side} 팔")
        # 카메라로 잰 상자 거리로 베이스 위치를 정밀 보정 (AMCL 오차 ±5cm 를 지각 기반으로 제거)
        # 이족은 미세 이동이 명령보다 덜/더 가므로(SF 미션 시험 4: +0.40 m 명령에 실제 +0.22 m) 카메라로 재며 최대 4 번 반복.
        # 바퀴형은 한 번이면 충분하다 (그대로 1 번)
        for _ in range(4 if self.can_strafe else 1):
            if abs(center[0] - self.IDEAL_REACH) <= self.REACH_TOL:
                break
            dx = center[0] - self.IDEAL_REACH
            self.nudge_base(dx)
            if self.can_strafe:
                self.wait_still(timeout=8.0)
            self.set_table()
            prev_fwd = center[0]
            center = self.find_marker(mid) - np.array([0, 0, self.box_size / 2])
            if self.can_strafe and abs((prev_fwd - center[0]) - dx) > 0.06:
                # 명령한 이동과 6 cm 넘게 어긋난 측정 → 몸통이 아직 흔들리는 중에 잰 값일 수 있다 (집기 시험: +5 cm 이동 뒤
                # 상자가 18 cm 가까워진 것으로 재고 −12 cm 되돌아감 → 미세 이동 4 번). 멈추길 기다려 한 번 더 잰다
                self.wait_still(timeout=8.0)
                center = self.find_marker(mid) - np.array([0, 0, self.box_size / 2])
            self._table_top_seen = float(center[2] - self.box_size / 2)
            self._box_seen_fwd = float(center[0])
            self.set_table()
            self.log(f"[pick] 보정 후 상자 위치 base {np.round(center, 3)}")
        if self.can_strafe and abs(center[1]) > self.LATERAL_OK:
            self.strafe_base(center[1] - math.copysign(self.LATERAL_IDEAL, center[1]))
            self.set_table()
            center = self.find_marker(mid) - np.array([0, 0, self.box_size / 2])
            self._table_top_seen = float(center[2] - self.box_size / 2)
            self._box_seen_fwd = float(center[0])
            self.set_table()
            self.log(f"[pick] 옆걸음 후 상자 위치 base {np.round(center, 3)}")
        # 상자 방향 (base 기준, 90° 주기로 접어 ±45°): 접근 방향을 이만큼 돌려 손가락이 옆면에 수직으로 닿게 (_solve_approach)
        box_yaw = 0.0
        if self.can_strafe:
            y = self._box_yaw(mid)
            if y is not None and abs(y) > math.radians(3):
                box_yaw = float(np.clip(y, -math.radians(30), math.radians(30)))
                self.log(f"[pick] 상자 방향 {math.degrees(y):+.1f}° → 접근 방향을 {math.degrees(box_yaw):+.1f}° 돌림")
        for attempt in (1, 2):
            self.open_gripper(side)
            # 몸 쪽 carry 자세에서 출발 → 접근점까지의 경로가 상자들 위쪽으로 지나감
            # 손을 옮기는 동안(home → carry → 접근점)은 작업대에 3 cm 여유를 둔다.
            # 여유 없이 계획하면 손가락이 상판을 스쳐(시뮬 접촉 기록) 이족은 몸통이 밀리고 넘어질 뻔했다 (기울기 18°).
            # 바퀴형도 같다: 상판을 1.2 cm 낮춘 MoveIt 작업대로 home→carry 를 계획하면 손가락이 상판을 긁었다 (바퀴형 회귀 시험 3)
            self.set_table(pad=self.TRANSIT_PAD)
            # 접근은 carry 를 거치지 않고 home 에서 바로 한다. 이족: home 에서 바로 접근 (무작위 배치 r1~r8 8/8 로 검증된 경로).
            # 이족이 carry 에서 출발하면 접근 직선 경로가 막히고(0 %), home 으로 되돌리는 계획이 손을 작업대 위로 휘둘러
            # 손가락이 상판에 걸려 몸통이 22° 기울었다 (전체 미션 18). 상자를 쥔 뒤 carry 로 접는 것(fold_carry)은 그대로
            # 바퀴형도 같다: 작업대 앞(모서리 0.38 m)에서 home→carry 는 팔을 앞으로 크게 휘둘러 손목·손가락이 상판 모서리를 쳤다
            # (바퀴형 회귀 시험 3·4 접촉 기록, 시뮬 재현: 모서리 0.50 m 에서도 닿음) → 바퀴형도 home 에서 바로 접근
            # (그래서 fold_carry 는 이제 '쥔 뒤 carry 로 접기'·'놓은 뒤 carry 를 거쳐 home' 에만 쓰인다)
            self.set_boxes()                                   # 계획 중에는 모든 상자가 장애물
            self.set_table()                                   # 접근 자세 계산은 실제 상판 높이로
            try:
                R, pre, q_pre = self._solve_approach(side, center, exclude_id=mid, yaw=box_yaw)
            except SkillError as e:
                if not self.can_strafe:
                    raise
                # 이족: 몸통 자세가 조금만 달라도 직선 접근 가능 여부가 바뀐다 → 3 cm 다가서서 한 번 더
                self.log(f"[pick] {e} → 3 cm 다가서서 다시 시도")
                self.move_named(f"{side}_arm", "home")
                self.nudge_base(0.03)
                self.set_table()
                center = self.find_marker(mid) - np.array([0, 0, self.box_size / 2])
                self._table_top_seen = float(center[2] - self.box_size / 2)
                self._box_seen_fwd = float(center[0])
                self.set_table()
                self.set_boxes()
                R, pre, q_pre = self._solve_approach(side, center, exclude_id=mid, yaw=box_yaw)
            self.set_table(pad=self.TRANSIT_PAD)               # carry → 접근점 이동도 여유를 두고
            servo = self.can_strafe and (WRIST is not None or self.wrist_cameras)
            station_ref = None
            if servo:
                # 접근 자세를 푸는 동안(수십 초) 몸통이 흘러 있다 → 팔이 상자를 가리기 전에 상자를 한 번 더 잰다
                try:
                    c3 = self.find_marker(mid, samples=3) - np.array([0, 0, self.box_size / 2])
                    station_ref = self._other_markers(exclude=None)
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
            if self.can_strafe:
                self.wait_still()
            # 손끝(TCP)을 내릴 수 있는 가장 낮은 높이: 손가락 가장 낮은 곳 = 손가락 끝 길이·sin(pitch) + 반두께·cos(pitch) 아래.
            # 예전 고정값 '상판 + 2 cm' 는 거의 수평(pitch 20°) 접근 기준이라, 옆 상자에 막혀 가파르게(55°) 들어가면
            # 손가락 끝이 상판에 닿았다 (놓을 자리 막힘 시험 p2, TCP 상판 + 2.3 cm 에서 접촉). → pitch 20° 2.4 cm, 55° 2.9 cm.
            # 바퀴형(45° 고정)도 상자 중심 + 1 cm 깊이 목표(TCP 상판 + 1.8 cm)는 하한(2.9 cm)보다 낮아 손가락이 상판을 긁고,
            # 두 손가락이 상자에 닿지 못해 파지에 실패했다 (바퀴형 회귀 시험 1·3: 첫 시도 손가락 0.001 m)
            top = self._table_top_seen if (self.can_strafe and self._table_top_seen is not None) else self.table_h
            s_p = max(0.0, -float(R[2, 2]))
            floor_z = top + self.FINGER_TIP_BELOW * s_p + self.FINGER_HALF_T * math.sqrt(1.0 - s_p * s_p) + 0.006
            if servo:
                self.station_keep(station_ref, label=" 접근점")    # 팔을 뻗는 동안 흐른 몸통을 마커 기준으로 되돌림
                # 시각 피드백 1단계 (접근점): 팔 처짐·몸통 반동으로 손이 접근점에서 몇 cm 벗어나 있다.
                # 손목 마커와 상자 마커를 함께 재서 '상자 기준' 접근점에 손을 맞춘 뒤 직선으로 들어간다.
                self.set_boxes(exclude_id=mid)
                pre_goal, center = self.visual_servo(side, mid, R, pre.copy(), center, pre - center,
                                                     floor_z=floor_z, label=" 접근점")
                bias = pre_goal - pre                          # 명령을 이만큼 옮겨야 손이 제자리에 온다 (처짐 보정량)
            elif self.can_strafe:
                # 팔을 뻗으면 이족 몸통이 반동으로 몇 cm 밀린다 → 직선 접근 목표를 마지막으로 다시 잰 위치로
                try:
                    c2 = self.find_marker(mid, samples=3) - np.array([0, 0, self.box_size / 2])
                    if np.linalg.norm(c2 - center) < 0.06:
                        self.log(f"[pick] 팔을 뻗은 뒤 다시 잰 상자 {np.round(c2, 3)} (차이 {np.round((c2 - center) * 100, 1)} cm)")
                        center = c2
                    else:
                        self.log(f"[pick] 팔을 뻗은 뒤 측정 {np.round(c2, 3)} 은 너무 달라 버림")
                except SkillError:
                    self.log("[pick] 팔을 뻗은 뒤 마커가 안 보임 → 이전 측정값 사용")
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
                    # 손이 옆으로 3 cm 벗어나 손가락이 상자 옆면을 쳐 4 cm 밀었다 (TRON1 손목 카메라 시험 2).
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
                self.station_keep(station_ref, label=" 직선 접근 전")
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
                               min_fraction=need, start_q=q_pre if self.can_strafe else None)
            if self.can_strafe and not servo and self.GRASP_PUSH > 0:
                # 이족: 손끝이 상자 중심보다 3.5 cm 앞에서 멈췄다(시뮬 정답 기록: 팔 처짐 + 몸통 반동). 접근 방향으로 더 넣으면
                # 45° 아래로 내려가 손가락이 상판에 걸리므로, 같은 높이에서 수평으로만 더 민다 (일부만 가도 됨)
                hdir = np.array([R[0, 2], R[1, 2], 0.0])
                hdir /= max(np.linalg.norm(hdir), 1e-6)
                try:
                    self.cartesian(side, [goal + hdir * self.GRASP_PUSH], R,
                                   min_fraction=0.0, avoid_collisions=False)
                except SkillError:
                    pass
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
                # 상자가 밀리거나 몸통이 돌아 반대쪽으로 갔으면(SF 시험 12: 오른팔 쪽 −0.10 → +0.20 m) 반대 팔로 바꾼다
                self.log(f"[pick] 상자가 반대쪽({center[1]:+.2f} m)으로 가서 {other} 팔로 바꿈")
                self.move_named(f"{side}_arm", "home")
                side = other
        else:
            raise SkillError(f"{obj} 파지 실패 (2회)")
        self.clear_boxes()
        self.holding[side] = obj
        # 쥘 때의 손 기울기 (상자는 수평이었다) → 놓을 때 같은 기울기로 내려놓으면 상자도 수평
        self._grasp_tilt[side] = getattr(self, "_last_approach", None)
        if self.can_strafe:
            self.set_table(pad=self.TRANSIT_PAD)
            # 이족: 상자를 쥔 뒤 뻗은 팔을 carry 로 빠르게(0.3) 접자 반동으로 PF 몸통이 1 초 만에 21°→70° 넘어졌다 (시험 11,
            # 파지는 성공·상자 5 cm 들어 올림). 06장 실험에서 팔을 3배 느리게 하면 흔들림 −38% → 0.1 로 천천히 접는다
            self.wait_still(timeout=8.0)
            if self.fold_carry:
                self.move_named(f"{side}_arm", "carry", vel=self.HOLD_VEL)
                self.wait_still(timeout=8.0)
        else:
            self.move_named(f"{side}_arm", "carry")
        self.log(f"[pick] 완료: {obj}")
        return side

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
        top_h = self._table_top_seen if (self.can_strafe and self._table_top_seen is not None) else self.table_h
        obs0 = self._place_obstacles(top_h)                # 놓을 자리를 정할 때 본 물체 (팔이 나중에 가려도 기억)
        target, R, pre, q_pre = self._choose_place(side, fwd, lat, edge, top_h, obs0)
        self.log(f"[place] {self.holding[side]} → base {np.round(target, 3)} ({side})")
        # 놓을 자리를 정한 순간의 작업대 위 마커들 (쥔 상자 마커는 손과 같이 움직이므로 뺌)
        held_id = self.objects.get(self.holding[side], {}).get("marker_id")
        place_ref = self._other_markers(exclude=held_id) if self.can_strafe else None
        self.set_table(station, pad=self.TRANSIT_PAD)
        self.move_joints(f"{side}_arm", q_pre, vel=self.HOLD_VEL if self.can_strafe else 0.3)
        self.set_table(station)
        if self.can_strafe:
            # 상자를 쥔 팔을 천천히 옮기는 동안 몸통이 흐를 수 있다 (r6: 바로잡기 걸음으로 옆 0.5 m, 작업대 밖에서 놓음)
            # → 놓기 전에 마커 기준 제자리로 돌아온다
            self.station_keep(place_ref, label=" 놓기 전")
            # 내려놓기 바로 전에 다시 본다: 팔을 뻗고 기다리는 동안 몸통이 흐르거나(시험: 11 cm·10°) 물체가 옮겨졌으면,
            # 또는 자리를 정할 때 상자를 든 팔이 머리 카메라를 가려 못 본 물체가 있으면(v3 시험 r7: 옆 상자 가장자리) 정한 자리가
            # 비어 있지 않다 → 처음 본 물체 + 지금 본 물체로 자리를 다시 골라 팔을 옮긴다 (2 번까지), 그래도 막히면 놓지 않고 실패
            for k in range(3):
                self.table_edge_seen(wait=1.0)
                obs0 = np.vstack([obs0, self._place_obstacles(top_h)])
                hit = self._place_blocked(target[0], target[1], obs0)
                if hit is None:
                    break
                if k == 2:
                    raise SkillError(f"놓기 직전 확인: 놓을 자리 {np.round(target[:2], 3)} 옆 물체 {np.round(hit, 3)} → 놓지 않음")
                self.log(f"[place] 놓기 직전 확인: 자리 {np.round(target[:2], 3)} 옆 물체 {np.round(hit, 3)} → 자리 다시 고름 {k + 1}/2")
                target, R, pre, q_pre = self._choose_place(side, fwd, lat, edge, top_h, obs0)
                self.log(f"[place] {self.holding[side]} → base {np.round(target, 3)} ({side})")
                self.set_table(station, pad=self.TRANSIT_PAD)
                self.move_joints(f"{side}_arm", q_pre, vel=self.HOLD_VEL)
                self.set_table(station)
                self.station_keep(place_ref, label=" 놓기 전")
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
        # 'ready'(손이 작업대 위)를 거치면 방금 놓은 상자를 칠 수 있으므로, 몸 쪽 carry 를 거쳐 home 으로
        self.set_table(station, pad=self.TRANSIT_PAD)
        if self.fold_carry and not self.can_strafe:   # 이족은 놓은 뒤 바로 home (carry 를 거치는 계획이 작업대 위로 손을 휘두름, 미션 18)
            self.move_named(f"{side}_arm", "carry")
            self.move_named(f"{side}_arm", "home")
        else:
            self.set_boxes()                    # 방금 놓은 상자도 장애물로 두고 home 까지 한 번에
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
        e = self._edge_recent(2.0) if self.can_strafe else None
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
            if self.holding[s]:
                self.move_named(f"{s}_arm", "carry")
            else:
                self.move_named(f"{s}_arm", "home")
