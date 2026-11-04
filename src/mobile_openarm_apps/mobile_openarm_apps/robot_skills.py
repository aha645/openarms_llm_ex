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
                             PlanningScene, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .geometry import approach_rotation, mat_to_quat_xyzw, quat_to_yaw, se2_inverse_apply, yaw_to_quat

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
        self._detections = {}           # id → (t_wall, position[3], yaw)
        self._attached = {"left": None, "right": None}
        self.create_subscription(String, "/aruco/detections", self._on_det, 10, callback_group=cb)
        self.create_subscription(String, "/sim/attached", self._on_att, 10, callback_group=cb)
        self._joints = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10, callback_group=cb)
        self.status_pub = self.create_publisher(String, "/task/log", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.holding = {"left": None, "right": None}
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

    def detections(self, max_age=1.0):
        now = time.time()
        with self._det_lock:
            return {i: (p, yaw) for i, (t, p, yaw) in self._detections.items() if now - t < max_age}

    def find_marker(self, marker_id, samples=5, timeout=5.0):
        """여러 프레임 평균한 마커 위치 (base_footprint)."""
        pts = []
        time.sleep(1.0)                 # 베이스 정지 후의 프레임만 쓰도록 잠깐 대기
        t0 = time.time()
        last_t = None
        while len(pts) < samples and time.time() - t0 < timeout:
            with self._det_lock:
                item = self._detections.get(marker_id)
            if item and item[0] != last_t and time.time() - item[0] < 0.5:
                last_t = item[0]
                pts.append(item[1])
            time.sleep(0.05)
        if not pts:
            raise SkillError(f"마커 {marker_id} 를 카메라에서 찾지 못함")
        return np.median(np.array(pts), axis=0)

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
        time.sleep(0.5)

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
        """현재 자세 기준 작업대 앞 모서리까지의 전방 거리 (map TF 사용)."""
        station = station or self.current_station
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

    def move_named(self, group, name, vel=0.3):
        key = (group, name)
        if key not in self.named_states:
            raise SkillError(f"SRDF 에 {group}/{name} 자세가 없음")
        self.log(f"[arm] {group} → '{name}'")
        self.move_joints(group, self.named_states[key], vel)

    def compute_ik(self, side, pos, R, avoid_collisions=True):
        req = GetPositionIK.Request()
        req.ik_request.group_name = f"{side}_arm"
        req.ik_request.robot_state.is_diff = True
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

    def cartesian(self, side, waypoints, R, vel=0.15, min_fraction=0.9, avoid_collisions=True):
        """TCP 를 직선으로 이동 (waypoints: base_footprint 좌표 리스트, 자세 R 고정)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE
        req.start_state.is_diff = True
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
    def set_table(self, station=None):
        """현재 작업대 위치를 MoveIt 충돌 객체로 추가 (base_footprint 기준)."""
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
        box.dimensions = [float(t["size"][0]), float(t["size"][1]), float(h)]
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(local[0]), float(local[1]), h / 2
        q = yaw_to_quat(-pose[2])          # 작업대 축은 map 축과 평행
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        co.primitives = [box]
        co.primitive_poses = [p]
        co.operation = CollisionObject.ADD
        self._apply_scene([co])

    def set_boxes(self, exclude_id=None):
        """보이는 상자들을 MoveIt 충돌 객체로 추가 (접근 경로가 다른/목표 상자를 치지 않도록).
        exclude_id 상자는 제거 → 직선 접근·파지 때는 손가락이 닿아도 되게."""
        objs = []
        for mid, (p, yaw) in self.detections(max_age=2.0).items():
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
    IDEAL_REACH = 0.52      # base_footprint 기준 상자까지 이상적 전방 거리 [m]
    # approach_rotation(roll=0) 에서 손가락이 닫히는 축(TCP y)은 '위-아래' 방향 → roll ±70~90 이 손가락을 수평으로 만들어
    # 상자 옆면을 잡는다. MuJoCo 오프라인 실험: ±70 이 가장 안정(상자 밀림 1~2cm), 0 은 모서리 집기라 밀림 3~4cm, ±20 은 놓침.
    ROLL_CANDIDATES = [70, -70, 90, -90, 50, -50, 0]
    # 이족 전용: 상자가 옆으로 이만큼 넘게 치우쳐 있으면 옆걸음으로 LATERAL_IDEAL 에 맞춘다.
    # 바퀴형 도킹에서 상자는 보통 팔 쪽으로 0.10~0.15 m. 이족은 AMCL 옆 오차(±0.15 m)로 0.27~0.31 m 가 되어
    # 직선 접근이 64% 에서 막혔다 (실험). 바퀴형은 옆으로 못 가므로 이 단계를 건너뛴다.
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

    def _solve_approach(self, side, target, back=0.10):
        """접근 자세 후보(roll × 물러선 거리)들 중 '접근점 IK + 목표 IK + 접근점→목표 직선 경로' 가 모두 되는 것 → (R, pre, q_pre).
        직선 경로까지 미리 검사하는 이유: 두 끝점의 IK 가 풀려도 중간에 관절 한계에 걸려 직선이 끊길 수 있다
        (TRON1 이족에서 실제로 64%·45% 에서 끊김). 그러면 같은 roll 로 더 가까이(0.07, 0.05 m)에서 시작하거나 다른 roll 을 쓴다."""
        best = None
        for b in (back, 0.07, 0.05):
            for roll in self.ROLL_CANDIDATES:
                R = approach_rotation(self.APPROACH_PITCH, roll)
                pre = target - R[:, 2] * b
                q_pre = self.compute_ik(side, pre, R)
                if q_pre is None or not self.compute_ik(side, target, R, avoid_collisions=False):
                    continue
                f = self._cart_fraction(side, q_pre, R, target + R[:, 2] * self.GRASP_DEPTH)
                if f >= 0.95:
                    self.log(f"[ik] {side} 접근 자세 roll={roll}°, {b * 100:.0f} cm 앞에서 직선 접근")
                    return R, pre, q_pre
                if best is None or f > best[0]:
                    best = (f, roll, b)
        if best:
            raise SkillError(f"{side} 팔 직선 접근 불가 (최대 {best[0] * 100:.0f}%, roll={best[1]}°) — 로봇 위치를 조정하세요")
        raise SkillError(f"{side} 팔로 {np.round(target, 3)} 에 접근하는 IK 해 없음 — 로봇 위치를 조정하세요")

    def pick(self, obj, side=None):
        if obj not in self.objects:
            raise SkillError(f"알 수 없는 물체 '{obj}' (가능: {list(self.objects)})")
        mid = self.objects[obj]["marker_id"]
        self.dock()
        self.set_table()
        top = self.find_marker(mid)                         # 상자 윗면 마커 (base_footprint)
        center = top - np.array([0, 0, self.box_size / 2])
        side = side or ("right" if center[1] < 0 else "left")
        if self.holding[side]:
            raise SkillError(f"{side} 손에 이미 {self.holding[side]} 를 들고 있음")
        self.log(f"[pick] {obj}(id {mid}) @ base {np.round(center, 3)} → {side} 팔")
        # 카메라로 잰 상자 거리로 베이스 위치를 정밀 보정 (AMCL 오차 ±5cm 를 지각 기반으로 제거)
        if abs(center[0] - self.IDEAL_REACH) > 0.03:
            self.nudge_base(center[0] - self.IDEAL_REACH)
            self.set_table()
            center = self.find_marker(mid) - np.array([0, 0, self.box_size / 2])
            self.log(f"[pick] 보정 후 상자 위치 base {np.round(center, 3)}")
        if self.can_strafe and abs(center[1]) > self.LATERAL_OK:
            self.strafe_base(center[1] - math.copysign(self.LATERAL_IDEAL, center[1]))
            self.set_table()
            center = self.find_marker(mid) - np.array([0, 0, self.box_size / 2])
            self.log(f"[pick] 옆걸음 후 상자 위치 base {np.round(center, 3)}")
        for attempt in (1, 2):
            self.open_gripper(side)
            # 몸 쪽 carry 자세에서 출발 → 접근점까지의 경로가 상자들 위쪽으로 지나감
            self.move_named(f"{side}_arm", "carry")
            self.set_boxes()                                   # 계획 중에는 모든 상자가 장애물
            R, pre, q_pre = self._solve_approach(side, center)
            self.move_joints(f"{side}_arm", q_pre)
            if self.can_strafe:
                # 팔을 뻗으면 이족 몸통이 반동으로 몇 cm 밀린다 → 직선 접근 목표를 마지막으로 다시 잰 위치로
                try:
                    c2 = self.find_marker(mid, samples=3) - np.array([0, 0, self.box_size / 2])
                    if np.linalg.norm(c2 - center) < 0.06:
                        center = c2
                except SkillError:
                    pass                                       # 팔이 마커를 가리면 이전 측정값 사용
            self.set_boxes(exclude_id=mid)                     # 직선 접근 전에 목표 상자만 제거
            self.cartesian(side, [center + R[:, 2] * self.GRASP_DEPTH], R)   # 1cm 더 깊이: 상자가 손가락 안쪽에 오도록
            self.close_gripper(side)
            time.sleep(0.5)
            # 파지 확인: 손가락이 끝까지 닫혔으면(물체 없음) 실패 — 실로봇에서도 같은 방법으로 판정
            opening = self.gripper_opening(side)
            ok = opening is None or opening > 0.006
            self.cartesian(side, [center + np.array([0, 0, 0.10])], R, avoid_collisions=False, min_fraction=0.5)
            if ok:
                break
            self.log(f"[pick] 파지 실패(손가락 {opening:.3f} m) → 다시 인식해서 재시도 {attempt}/2")
            self.open_gripper(side)
            self.move_named(f"{side}_arm", "carry")
            center = self.find_marker(mid) - np.array([0, 0, self.box_size / 2])
        else:
            raise SkillError(f"{obj} 파지 실패 (2회)")
        self.clear_boxes()
        self.holding[side] = obj
        self.move_named(f"{side}_arm", "carry")
        self.log(f"[pick] 완료: {obj}")
        return side

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
        target = np.array([fwd, lat, self.table_h + self.box_size / 2 + 0.01])
        self.log(f"[place] {self.holding[side]} → base {np.round(target, 3)} ({side})")
        R, pre, q_pre = self._solve_approach(side, target + np.array([0, 0, 0.05]), back=0.06)
        self.move_joints(f"{side}_arm", q_pre)
        self.cartesian(side, [target], R, avoid_collisions=False)
        self.open_gripper(side)
        time.sleep(0.5)
        # 놓은 뒤: 접근 반대 방향으로 빼고 위로 들어 올려 작업대·물체와 충분히 떨어뜨림
        try:
            self.cartesian(side, [target - R[:, 2] * 0.08 + np.array([0, 0, 0.04]),
                                  target - R[:, 2] * 0.12 + np.array([0, 0, 0.12])], R,
                           avoid_collisions=False, min_fraction=0.3)
        except SkillError as e:   # 직선으로 다 못 빼도 이미 놓았으므로 계속 (아래 carry 는 충돌 회피 계획)
            self.log(f"[place] 후퇴 경로 일부만 가능: {e}")
        obj = self.holding[side]
        self.holding[side] = None
        # 'ready'(손이 작업대 위)를 거치면 방금 놓은 상자를 칠 수 있으므로, 몸 쪽 carry 를 거쳐 home 으로
        self.move_named(f"{side}_arm", "carry")
        self.move_named(f"{side}_arm", "home")
        self.log(f"[place] 완료: {obj}")
        return obj

    def home_arms(self):
        for s in ("left", "right"):
            if self.holding[s]:
                self.move_named(f"{s}_arm", "carry")
            else:
                self.move_named(f"{s}_arm", "home")
