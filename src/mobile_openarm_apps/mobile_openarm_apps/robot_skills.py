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
from sensor_msgs.msg import JointState
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
        self._table_top_seen = None      # 이족: 카메라로 잰 상판 높이 (base_footprint 명목 좌표)
        self._box_seen_fwd = None        # 이족: 카메라로 잰 목표 상자의 전방 거리 (작업대 앞 모서리 상한 계산용)
        if self.can_strafe:
            # 이족(TRON1) 작업대 거리. 예전 어깨 1.08 m 설계에서는 상자가 팔 길이 끝이라 0.32/0.41 로 붙어 섰지만,
            # 어깨를 몸통 위로 낮춘 지금은 팔을 앞으로 든 carry 자세의 팔뚝이 앞 0.36 m 까지 나와 붙어 서면 상판 모서리에 걸린다
            # (시뮬 접촉 기록). 어깨→상자 거리는 0.45 m 로 팔 길이(0.63 m) 안이므로 바퀴형과 비슷하게 떨어져 선다
            self.DOCK_GAP = float(os.environ.get("OPENARM_DOCK_GAP", 0.42))
            self.IDEAL_REACH = float(os.environ.get("OPENARM_IDEAL_REACH", 0.52))
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
        self._marker_odom = {}          # id → odom 좌표 (가려졌을 때 몸통 흐름을 보정해 다시 쓰기)
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
        p = np.median(np.array(pts), axis=0)
        self._remember_marker(marker_id, p)
        return p

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

    def wait_still(self, tol=0.006, need=3, period=0.5, timeout=20.0):
        """이족: 팔을 크게 움직이면 몸통이 균형을 잡느라 5~10 cm 출렁인다 (시뮬 측정). 오도메트리 위치가
        period 마다 tol 미만으로만 바뀌는 상태가 need 번 이어질 때까지 기다린다. 바퀴형은 바로 반환."""
        if not self.can_strafe:
            return
        last, ok, t0 = None, 0, time.time()
        while time.time() - t0 < timeout and ok < need:
            p = self.robot_pose("odom")
            if p is not None and last is not None:
                ok = ok + 1 if math.hypot(p[0] - last[0], p[1] - last[1]) < tol else 0
            last = p
            time.sleep(period)

    # ------------------------------------------------------------------ 손목 마커 시각 피드백 (이족)
    SERVO_ITERS = 3         # 보정 이동 최대 횟수
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
        if WRIST is None:
            raise SkillError("손목 마커 정의(tron1_openarm_sim.wrist_markers) 없음")
        time.sleep(0.6)
        tcps, boxes, seen, t0 = [], [], set(), time.time()
        while len(tcps) < samples and time.time() - t0 < timeout:
            with self._det_lock:
                items = dict(self._detections)
            tf7 = self._tf_matrix(f"openarm_{side}_link7")
            fresh = [(i, items[i]) for i in WRIST.IDS[side] if i in items and time.time() - items[i][0] < 0.5]
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

    def visual_servo(self, side, box_id, R, goal, center, offset, floor_z=None, label=""):
        """손끝이 '상자 중심 + offset' 에 오도록 카메라로 재며 고친다 (closed-loop).
        goal: 지금까지 MoveIt 에 명령한 손끝 목표. 팔이 중력에 처지는 만큼 실제 손끝은 목표에 못 미치므로
        '명령 목표' 를 오차만큼 누적해서 옮긴다 (적분) → 처짐이 매번 같으면 2~3 번에 수렴.
        반환: (최종 명령 목표, 최신 상자 중심)"""
        for it in range(1, self.SERVO_ITERS + 1):
            self.wait_still(timeout=8.0)
            try:
                tcp, box, fk, src = self.measure_hand_and_box(side, box_id)
            except SkillError as e:
                self.log(f"[servo{label}] {e} → 보정 생략")
                break
            stale = ""
            ref = self.recall_marker(box_id)    # 마지막으로 믿은 상자 위치를 지금 몸통 기준으로
            ref = center if ref is None else ref - np.array([0, 0, self.box_size / 2])
            top_ok = self._table_top_seen is None or box is None or \
                abs(box[2] - self.box_size / 2 - self._table_top_seen) < 0.02
            if box is not None and top_ok:
                center = box                    # 같은 프레임에서 본 상자가 우선
                self._remember_marker(box_id, box + np.array([0, 0, self.box_size / 2]))
            else:
                # 카메라로 본 상자는 정답과 1 cm 안으로 맞았다(시험 11 정답 기록) — 손에 밀려 9 cm 움직인 것도 정확히 잡음.
                # 반대로 odom 으로 옮긴 옛 위치는 제자리 걸음 미끄러짐을 몰라 9 cm 틀렸다. 그래서 보이면 항상 카메라 값을 쓰고,
                # 높이가 상판과 2 cm 넘게 안 맞는 측정(손가락에 가려 깊이를 손가락에서 읽은 경우)만 버린다.
                # 상자가 가려졌으면 마지막으로 본 위치(odom)를 지금 몸통 기준으로 옮겨 쓴다.
                # (시험 9: 옛 base 좌표를 그대로 써서 손을 상자 옆 10 cm 에 '정확히' 맞췄다)
                center = ref
                stale = " [상자 가려짐: 마지막 위치를 odom 으로 옮겨 사용]" if box is None else f" [상자 측정 {np.round(box, 3)} 이 너무 달라 버림]"
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
            goal = goal + step
            if floor_z is not None:
                goal[2] = max(goal[2], floor_z)
            try:
                self.cartesian(side, [goal], R, vel=0.1, min_fraction=0.0, avoid_collisions=False)
            except SkillError as e:
                self.log(f"[servo{label}] 보정 이동 실패: {e}")
                break
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
        self._box_seen_fwd = None       # 새 작업대 — 이전 작업대에서 잰 상자 거리는 쓰지 않음
        pose = self.robot_pose()
        if pose is None or station not in self.locations:
            return
        # 1) 작업대와 정면으로 마주 보도록 yaw 보정 (Nav2 yaw 허용오차 ±7° 보정)
        want = math.radians(self.locations[station][2])
        self.rotate_base(math.atan2(math.sin(want - pose[2]), math.cos(want - pose[2])))
        if self.can_strafe:
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
        if code != MoveItErrorCodes.SUCCESS:
            raise SkillError(f"MoveIt {group} 실패 (error_code={code})")

    def move_named(self, group, name, vel=0.3):
        key = (group, name)
        if key not in self.named_states:
            raise SkillError(f"SRDF 에 {group}/{name} 자세가 없음")
        self.log(f"[arm] {group} → '{name}'")
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
            edge_map = float(local[0] - t["size"][1] / 2)
            edge_cam = self._box_seen_fwd - self.BOX_EDGE_MIN
            if edge_cam < edge_map:
                local = np.array([local[0] + edge_cam - edge_map, local[1]])
        box.dimensions = [float(t["size"][0]) + 2 * pad, float(t["size"][1]) + 2 * pad, float(h)]
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
    IDEAL_REACH = 0.52      # base_footprint 기준 상자까지 이상적 전방 거리 [m]
    # approach_rotation(roll=0) 에서 손가락이 닫히는 축(TCP y)은 '위-아래' 방향 → roll ±70~90 이 손가락을 수평으로 만들어
    # 상자 옆면을 잡는다. MuJoCo 오프라인 실험: ±70 이 가장 안정(상자 밀림 1~2cm), 0 은 모서리 집기라 밀림 3~4cm, ±20 은 놓침.
    ROLL_CANDIDATES = [70, -70, 90, -90, 50, -50, 0]
    # 이족 전용: 상자가 옆으로 이만큼 넘게 치우쳐 있으면 옆걸음으로 LATERAL_IDEAL 에 맞춘다.
    # 바퀴형 도킹에서 상자는 보통 팔 쪽으로 0.10~0.15 m. 이족은 AMCL 옆 오차(±0.15 m)로 0.27~0.31 m 가 되어
    # 직선 접근이 64% 에서 막혔다 (실험). 바퀴형은 옆으로 못 가므로 이 단계를 건너뛴다.
    TRANSIT_PAD = 0.03
    BOX_EDGE_MIN = 0.10     # 이족: 상자는 작업대 앞 모서리에서 적어도 이만큼 안쪽에 놓여 있다고 가정 [m] (시뮬 월드 0.12~0.20)
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

    def _solve_approach(self, side, target, back=0.10, exclude_id=None):
        """접근 자세 후보(roll × 물러선 거리)들 중 '접근점 IK + 목표 IK + 접근점→목표 직선 경로' 가 모두 되는 것 → (R, pre, q_pre).
        직선 경로까지 미리 검사하는 이유: 두 끝점의 IK 가 풀려도 중간에 관절 한계에 걸려 직선이 끊길 수 있다
        (TRON1 이족에서 실제로 64%·45% 에서 끊김). 그러면 같은 roll 로 더 가까이(0.07, 0.05 m)에서 시작하거나 다른 roll 을 쓴다."""
        best = None
        # 이족: 어깨를 몸통 위로 낮춘 설계(어깨 0.78~0.83 m, 상자 0.69 m)에서는 거의 수평 접근이 자연스럽고,
        # 45° 로 내려오면 아래쪽 손가락이 상판에 걸린다 → 낮은 pitch 부터 시도
        pitches = ([20.0, 30.0, self.APPROACH_PITCH, 55.0] if self.can_strafe else [self.APPROACH_PITCH])
        cands = [(p, b, r) for b in (back, 0.07, 0.05) for p in pitches for r in self.ROLL_CANDIDATES]
        # 직선 접근 검사는 실제 접근 때와 같은 장면(목표 상자만 뺀)에서 해야 한다.
        # 목표 상자를 장애물로 둔 채 검사하면 손이 상자에 닿는 마지막 구간이 '충돌'로 판정돼 항상 실패한다.
        if exclude_id is not None:
            self.set_boxes(exclude_id=exclude_id)
        try:
            for pitch, b, roll in cands:
                R = approach_rotation(pitch, roll)
                pre = target - R[:, 2] * b
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
                        self.log(f"[ik] {side} 접근 자세 pitch={pitch:.0f}° roll={roll}°, {b * 100:.0f} cm 앞에서 직선 접근")
                        return R, pre, q_pre
                    if best is None or f > best[0]:
                        best = (f, roll, b)
        finally:
            if exclude_id is not None:
                self.set_boxes()
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
        self._table_top_seen = float(top[2] - self.box_size)
        self._box_seen_fwd = float(top[0])
        self.set_table()                                    # 이족: 잰 상판 높이로 작업대 충돌 객체 갱신
        side = side or ("right" if center[1] < 0 else "left")
        if self.holding[side]:
            raise SkillError(f"{side} 손에 이미 {self.holding[side]} 를 들고 있음")
        self.log(f"[pick] {obj}(id {mid}) @ base {np.round(center, 3)} → {side} 팔")
        # 카메라로 잰 상자 거리로 베이스 위치를 정밀 보정 (AMCL 오차 ±5cm 를 지각 기반으로 제거)
        if abs(center[0] - self.IDEAL_REACH) > 0.03:
            self.nudge_base(center[0] - self.IDEAL_REACH)
            self.set_table()
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
        for attempt in (1, 2):
            self.open_gripper(side)
            # 몸 쪽 carry 자세에서 출발 → 접근점까지의 경로가 상자들 위쪽으로 지나감
            # 이족: 손을 옮기는 동안(home → carry → 접근점)은 작업대에 3 cm 여유를 둔다.
            # 여유 없이 계획하면 손가락이 상판을 스쳐(시뮬 접촉 기록) 몸통이 밀리고 넘어질 뻔했다 (기울기 18°)
            if self.can_strafe:
                self.set_table(pad=self.TRANSIT_PAD)
            self.move_named(f"{side}_arm", "carry")
            self.set_boxes()                                   # 계획 중에는 모든 상자가 장애물
            if self.can_strafe:
                self.set_table()                               # 접근 자세 계산은 실제 상판 높이로
            try:
                R, pre, q_pre = self._solve_approach(side, center, exclude_id=mid)
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
                self.move_named(f"{side}_arm", "carry")
                self.set_boxes()
                R, pre, q_pre = self._solve_approach(side, center, exclude_id=mid)
            if self.can_strafe:
                self.set_table(pad=self.TRANSIT_PAD)           # carry → 접근점 이동도 여유를 두고
            servo = self.can_strafe and WRIST is not None
            if servo:
                # 접근 자세를 푸는 동안(수십 초) 몸통이 흘러 있다 → 팔이 상자를 가리기 전에 상자를 한 번 더 잰다
                try:
                    c3 = self.find_marker(mid, samples=3) - np.array([0, 0, self.box_size / 2])
                    shift = c3 - center
                    self.log(f"[pick] 접근점으로 가기 전 상자 {np.round(c3, 3)} (계획 때와 차이 {np.round(shift * 100, 1)} cm)")
                    if np.linalg.norm(shift) > 0.02:
                        # 상자가 몸통 기준으로 옮겨졌으면 접근점도 같이 옮긴다 (그대로 가면 손이 상자를 친다: 시험 11)
                        q2 = self.compute_ik(side, pre + shift, R, seed=q_pre)
                        if q2 is not None:
                            pre, q_pre = pre + shift, q2
                        else:
                            self.log("[pick] 옮긴 접근점 IK 실패 → 원래 접근점 사용 (시각 보정에 맡김)")
                    center = c3
                except SkillError:
                    pass
            self.move_joints(f"{side}_arm", q_pre)
            bias = np.zeros(3)
            floor_z = None
            if self.can_strafe:
                self.set_table()                               # 직선 접근은 실제 상판 높이로
                self.wait_still()
            if servo:
                # 시각 피드백 1단계 (접근점): 팔 처짐·몸통 반동으로 손이 접근점에서 몇 cm 벗어나 있다.
                # 손목 마커와 상자 마커를 함께 재서 '상자 기준' 접근점에 손을 맞춘 뒤 직선으로 들어간다.
                top = self._table_top_seen if self._table_top_seen is not None else self.table_h
                floor_z = top + 0.02                           # 손끝을 상판 2 cm 아래로는 내리지 않음 (손가락 두께)
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
                try:
                    self.cartesian(side, [goal], R, min_fraction=need)
                except SkillError as e:
                    self.log(f"[pick] 직선 접근 {e} → 충돌 검사 없이 (손 높이는 카메라로 확인한 값)")
                    self.cartesian(side, [goal], R, min_fraction=need, avoid_collisions=False)
                # 시각 피드백 2단계 (파지 위치): 손끝이 상자 중심(+1 cm 안쪽)에 올 때까지 재고 고친다
                goal, center = self.visual_servo(side, mid, R, goal, center, R[:, 2] * self.SERVO_DEPTH,
                                                 floor_z=floor_z, label=" 파지")
            else:
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
        if self.can_strafe:
            self.set_table(pad=self.TRANSIT_PAD)
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
        top_h = self._table_top_seen if (self.can_strafe and self._table_top_seen is not None) else self.table_h
        target = np.array([fwd, lat, top_h + self.box_size / 2 + 0.01])
        self.log(f"[place] {self.holding[side]} → base {np.round(target, 3)} ({side})")
        R, pre, q_pre = self._solve_approach(side, target + np.array([0, 0, 0.05]), back=0.06)
        if self.can_strafe:
            self.set_table(station, pad=self.TRANSIT_PAD)
        self.move_joints(f"{side}_arm", q_pre)
        if self.can_strafe:
            self.set_table(station)
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
        if self.can_strafe:
            self.set_table(station, pad=self.TRANSIT_PAD)
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
