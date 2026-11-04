#!/usr/bin/env python3
"""MuJoCo ↔ ROS 2 브리지 노드 (교육과정 Day1-02 "MuJoCo–ROS 2 브리지").

발행
  /clock                         rosgraph_msgs/Clock        (시뮬레이션 시간, 100Hz)
  /joint_states                  sensor_msgs/JointState     (50Hz)
  /odom  + TF odom→base_footprint nav_msgs/Odometry          (50Hz, 휠 오도메트리)
  /scan                          sensor_msgs/LaserScan      (10Hz, frame lidar_link)
  /imu                           sensor_msgs/Imu            (50Hz)
  /ground_truth/pose             geometry_msgs/PoseStamped  (world 기준 정답 위치, 디버깅용)
  /aruco/detections              std_msgs/String (JSON)     (검출 결과만 발행 — 카메라는 노드 내부 처리)
  /aruco/poses                   geometry_msgs/PoseArray    (base_footprint 기준)
  /aruco/markers                 visualization_msgs/MarkerArray
  /head_camera/image_raw/compressed sensor_msgs/CompressedImage (선택, 대시보드 MJPEG 용)
  /head_camera/camera_info       sensor_msgs/CameraInfo
  /sim/attached                  std_msgs/String            (파지 보조로 잡고 있는 물체)
  /sim/objects                   std_msgs/String (JSON)     (상자들의 정답 위치 — 디버깅/채점용)
구독
  /cmd_vel                       geometry_msgs/Twist
액션 서버 (openarm_bringup 의 컨트롤러 이름과 동일 → MoveIt 설정을 실로봇과 공유)
  /left_joint_trajectory_controller/follow_joint_trajectory   control_msgs/FollowJointTrajectory
  /right_joint_trajectory_controller/follow_joint_trajectory
  /left_gripper_controller/follow_joint_trajectory
  /right_gripper_controller/follow_joint_trajectory
  /left_gripper_controller/gripper_cmd                        control_msgs/GripperCommand
  /right_gripper_controller/gripper_cmd
서비스
  /sim/reset (std_srvs/Trigger), /sim/set_grasp_assist (std_srvs/SetBool),
  /sim/set_person_motion (std_srvs/SetBool)
"""
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .kinematics import mat_to_quat
from .sim_core import ARM_JOINTS, GRIPPER_JOINT, WHEEL_RADIUS, MobileOpenArmSim

BASE_LINK_Z = WHEEL_RADIUS   # base_footprint → base_link


def stamp(t):
    # 주의: t 는 0.002 를 누적한 float 라 107.99999999997 같은 값이 된다.
    # sec=int(t), nsec=round(frac) 로 나누면 nsec 가 1e9 로 반올림되어 '1초 뒤로 점프' → TF 버퍼가 매초 지워짐.
    ns = int(round(t * 1e9))
    msg = TimeMsg()
    msg.sec = ns // 1000000000
    msg.nanosec = ns % 1000000000
    return msg


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))  # x y z w


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")
        share = get_package_share_directory("mobile_openarm_sim")
        p = self.declare_parameter
        p("mjcf", os.path.join(share, "mjcf", "world.xml"))
        p("viewer", True)
        p("camera", True)
        p("camera_width", 640)
        p("camera_height", 480)
        p("camera_hz", 10.0)
        p("publish_image", True)
        p("image_hz", 5.0)
        p("lidar_rays", 360)
        p("lidar_hz", 10.0)
        p("grasp_assist", True)
        p("move_person", True)
        p("person_speed", 0.4)
        p("rate_factor", 1.0)          # 1.0 = 실시간, 0.5 = 절반 속도
        p("start_x", 0.0)
        p("start_y", 0.0)
        p("start_yaw", 0.0)
        p("publish_odom_tf", True)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.sim = MobileOpenArmSim(
            gp("mjcf"), camera_width=gp("camera_width"), camera_height=gp("camera_height"),
            camera_hz=gp("camera_hz"), enable_camera=gp("camera"), lidar_rays=gp("lidar_rays"),
            lidar_hz=gp("lidar_hz"), grasp_assist=gp("grasp_assist"), move_person=gp("move_person"),
            person_speed=gp("person_speed"))
        if gp("start_x") or gp("start_y") or gp("start_yaw"):
            self.sim.set_base_pose(gp("start_x"), gp("start_y"), gp("start_yaw"))
        self.viewer_enabled = gp("viewer")
        self.publish_image = gp("publish_image")
        self.image_period = 1.0 / max(gp("image_hz"), 0.1)
        self.rate_factor = gp("rate_factor")
        self.publish_odom_tf = gp("publish_odom_tf")
        self._last_img_t = -1e9
        self._last_js_t = -1e9

        qos_clock = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_clock = self.create_publisher(Clock, "/clock", qos_clock)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        self.pub_gt = self.create_publisher(PoseStamped, "/ground_truth/pose", 10)
        self.pub_det = self.create_publisher(String, "/aruco/detections", 10)
        self.pub_poses = self.create_publisher(PoseArray, "/aruco/poses", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/aruco/markers", 10)
        self.pub_img = self.create_publisher(CompressedImage, "/head_camera/image_raw/compressed", 2)
        self.pub_caminfo = self.create_publisher(CameraInfo, "/head_camera/camera_info", 2)
        self.pub_att = self.create_publisher(String, "/sim/attached", 10)
        self.pub_objs = self.create_publisher(String, "/sim/objects", 10)
        self.tfb = TransformBroadcaster(self)

        cb = ReentrantCallbackGroup()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10, callback_group=cb)
        self.create_service(Trigger, "/sim/reset", self.on_reset, callback_group=cb)
        self.create_service(SetBool, "/sim/set_grasp_assist", self.on_grasp_assist, callback_group=cb)
        self.create_service(SetBool, "/sim/set_person_motion", self.on_person, callback_group=cb)

        self._servers = []
        for side in ("left", "right"):
            self._servers.append(ActionServer(
                self, FollowJointTrajectory, f"/{side}_joint_trajectory_controller/follow_joint_trajectory",
                execute_callback=lambda gh, s=side: self.exec_arm(gh, s),
                goal_callback=lambda g, s=side: self.check_arm_goal(g, s),
                cancel_callback=lambda gh: CancelResponse.ACCEPT, callback_group=cb))
            self._servers.append(ActionServer(
                self, FollowJointTrajectory, f"/{side}_gripper_controller/follow_joint_trajectory",
                execute_callback=lambda gh, s=side: self.exec_gripper_traj(gh, s),
                cancel_callback=lambda gh: CancelResponse.ACCEPT, callback_group=cb))
            self._servers.append(ActionServer(
                self, GripperCommand, f"/{side}_gripper_controller/gripper_cmd",
                execute_callback=lambda gh, s=side: self.exec_gripper_cmd(gh, s),
                cancel_callback=lambda gh: CancelResponse.ACCEPT, callback_group=cb))

        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.physics_loop, daemon=True)
        self.thread.start()
        self.get_logger().info("MuJoCo bridge started (viewer=%s, camera=%s)" % (self.viewer_enabled, gp("camera")))

    # ------------------------------------------------------------ physics thread
    def physics_loop(self):
        viewer = None
        if self.viewer_enabled:
            try:
                import mujoco.viewer
                viewer = mujoco.viewer.launch_passive(self.sim.m, self.sim.d,
                                                      show_left_ui=False, show_right_ui=False)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"viewer 를 열 수 없음 → headless 로 계속: {e}")
                viewer = None
        try:
            self.sim.run_realtime(self.stop_event, rate_factor=self.rate_factor,
                                  on_tick=self.on_tick, tick_hz=100.0, viewer=viewer)
        finally:
            if viewer is not None:
                viewer.close()
            if self.stop_event.is_set() and rclpy.ok():
                self.get_logger().info("viewer closed → shutting down")
                rclpy.try_shutdown()

    def on_tick(self, sim):
        t = sim.time
        c = Clock()
        c.clock = stamp(t)
        self.pub_clock.publish(c)
        if t - self._last_js_t >= 0.02 - 1e-9:
            self._last_js_t = t
            self.publish_joint_states(t)
            self.publish_odom(t)
            self.publish_imu(t)
            self.publish_gt(t)
        scan = sim.maybe_scan()
        if scan is not None:
            self.publish_scan(t, scan)
        cam = sim.maybe_camera()
        if cam is not None:
            rgb, dets = cam
            self.publish_detections(t, dets)
            if self.publish_image and t - self._last_img_t >= self.image_period:
                self._last_img_t = t
                self.publish_image_msg(t, rgb, dets)

    # ------------------------------------------------------------ publishers
    def publish_joint_states(self, t):
        names, pos, vel, eff = self.sim.joint_state()
        js = JointState()
        js.header.stamp = stamp(t)
        js.name, js.position, js.velocity, js.effort = names, pos, vel, eff
        self.pub_js.publish(js)
        att = String()
        att.data = json.dumps({s: self.sim.attached_object(s) for s in ("left", "right")})
        self.pub_att.publish(att)
        objs = String()
        objs.data = json.dumps({"frame": "world", "objects": self.sim.object_poses()})
        self.pub_objs.publish(objs)

    def publish_odom(self, t):
        x, y, yaw = self.sim.odom
        q = yaw_quat(yaw)
        od = Odometry()
        od.header.stamp = stamp(t)
        od.header.frame_id = "odom"
        od.child_frame_id = "base_footprint"
        od.pose.pose.position.x, od.pose.pose.position.y = float(x), float(y)
        (od.pose.pose.orientation.x, od.pose.pose.orientation.y,
         od.pose.pose.orientation.z, od.pose.pose.orientation.w) = q
        od.twist.twist.linear.x = float(self.sim.odom_vel[0])
        od.twist.twist.angular.z = float(self.sim.odom_vel[1])
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.01
        cov[35] = 0.02
        od.pose.covariance = cov
        od.twist.covariance = cov
        self.pub_odom.publish(od)
        if self.publish_odom_tf:
            tf = TransformStamped()
            tf.header = od.header
            tf.child_frame_id = "base_footprint"
            tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
            (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w) = q
            self.tfb.sendTransform(tf)

    def publish_imu(self, t):
        q, g, a = self.sim.imu()
        m = Imu()
        m.header.stamp = stamp(t)
        m.header.frame_id = "imu_link"
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = map(float, q)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, g)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, a)
        self.pub_imu.publish(m)

    def publish_gt(self, t):
        x, y, yaw = self.sim.ground_truth_pose()
        ps = PoseStamped()
        ps.header.stamp = stamp(t)
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = yaw_quat(yaw)
        self.pub_gt.publish(ps)

    def publish_scan(self, t, scan):
        a0, inc, r = scan
        s = LaserScan()
        s.header.stamp = stamp(t)
        s.header.frame_id = "lidar_link"
        s.angle_min = float(a0)
        s.angle_increment = float(inc)
        s.angle_max = float(a0 + inc * (len(r) - 1))
        s.time_increment = 0.0
        s.scan_time = float(self.sim.lidar_period)
        s.range_min = float(self.sim.lidar_min)
        s.range_max = float(self.sim.lidar_max)
        s.ranges = [float(v) for v in r]
        self.pub_scan.publish(s)

    def publish_detections(self, t, dets):
        out = []
        pa = PoseArray()
        pa.header.stamp = stamp(t)
        pa.header.frame_id = "base_footprint"
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for det in dets:
            p = det["position_base"] + np.array([0, 0, BASE_LINK_Z])   # base_link → base_footprint
            R = det["rotation_base"]
            qw, qx, qy, qz = mat_to_quat(R)
            # 마커 z축(면 법선)으로부터 상자 yaw 추정 (상자 윗면 마커 → 마커 x축의 수평 방향)
            yaw = math.atan2(R[1, 0], R[0, 0])
            out.append({"id": det["id"], "frame": "base_footprint",
                        "position": [round(float(v), 4) for v in p],
                        "quaternion_xyzw": [float(qx), float(qy), float(qz), float(qw)],
                        "yaw": round(yaw, 4)})
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, p)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
                float(qx), float(qy), float(qz), float(qw))
            pa.poses.append(pose)
            mk = Marker()
            mk.header = pa.header
            mk.ns, mk.id, mk.type, mk.action = "aruco", det["id"], Marker.CUBE, Marker.ADD
            mk.pose = pose
            mk.scale.x = mk.scale.y = 0.05
            mk.scale.z = 0.005
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.9, 0.2, 0.8
            ma.markers.append(mk)
            tx = Marker()
            tx.header = pa.header
            tx.ns, tx.id, tx.type = "aruco_text", det["id"], Marker.TEXT_VIEW_FACING
            tx.pose = Pose()
            tx.pose.position.x, tx.pose.position.y, tx.pose.position.z = float(p[0]), float(p[1]), float(p[2]) + 0.06
            tx.scale.z = 0.04
            tx.color.r = tx.color.g = tx.color.b = tx.color.a = 1.0
            tx.text = f"id {det['id']}"
            ma.markers.append(tx)
        msg = String()
        msg.data = json.dumps({"stamp": t, "detections": out})
        self.pub_det.publish(msg)
        self.pub_poses.publish(pa)
        self.pub_markers.publish(ma)

    def publish_image_msg(self, t, rgb, dets):
        from .aruco_utils import ArucoDetector
        bgr = ArucoDetector.draw(rgb, dets)
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp(t)
        msg.header.frame_id = "head_camera_optical_frame"
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self.pub_img.publish(msg)
        ci = CameraInfo()
        ci.header = msg.header
        ci.width, ci.height = self.sim.cam_w, self.sim.cam_h
        K = self.sim.K
        ci.k = [float(v) for v in K.reshape(-1)]
        ci.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        ci.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        ci.d = [0.0] * 5
        ci.distortion_model = "plumb_bob"
        self.pub_caminfo.publish(ci)

    # ------------------------------------------------------------ subscriptions / services
    def on_cmd_vel(self, msg):
        self.sim.set_cmd_vel(msg.linear.x, msg.angular.z)

    def on_reset(self, req, res):
        # 주의: 시뮬레이션 시간이 0 으로 돌아가므로 Nav2/SLAM/MoveIt 은 재시작하는 것이 안전
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.sim.reset(gp("start_x"), gp("start_y"), gp("start_yaw"))
        self._last_js_t = self._last_img_t = -1e9
        res.success = True
        res.message = "simulation reset (다른 노드들은 재시작 권장)"
        return res

    def on_grasp_assist(self, req, res):
        self.sim.grasp_assist = bool(req.data)
        if not req.data:
            self.sim.attached = {"left": None, "right": None}
        res.success = True
        res.message = f"grasp_assist={req.data}"
        return res

    def on_person(self, req, res):
        self.sim.move_person = bool(req.data)
        res.success = True
        res.message = f"move_person={req.data}"
        return res

    # ------------------------------------------------------------ actions
    def check_arm_goal(self, goal, side):
        names = list(goal.trajectory.joint_names)
        if sorted(names) != sorted(ARM_JOINTS[side]):
            self.get_logger().error(f"{side} arm: 잘못된 joint_names {names}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _points(traj):
        pts = []
        for p in traj.points:
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            pts.append((t, list(p.positions), list(p.velocities) if len(p.velocities) else None))
        return pts

    def _wait(self, gh, tr, feedback_fn=None):
        while rclpy.ok():
            if gh.is_cancel_requested:
                return "canceled"
            if tr.canceled:
                return "aborted"
            if tr.done:
                return "done"
            if feedback_fn:
                feedback_fn()
            time.sleep(0.02)
        return "aborted"

    def exec_arm(self, gh, side):
        traj = gh.request.trajectory
        tr = self.sim.start_arm_trajectory(side, list(traj.joint_names), self._points(traj))
        names = list(traj.joint_names)

        def fb():
            f = FollowJointTrajectory.Feedback()
            f.joint_names = names
            cur = self.sim.arm_positions(side)
            idx = [ARM_JOINTS[side].index(n) for n in names]
            f.actual.positions = [cur[i] for i in idx]
            gh.publish_feedback(f)

        state = self._wait(gh, tr, fb)
        res = FollowJointTrajectory.Result()
        if state == "canceled":
            self.sim.cancel_trajectory(side)
            gh.canceled()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res
        if state == "aborted":
            gh.abort()
            res.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            res.error_string = "preempted by a newer trajectory"
            return res
        # 목표 도달 확인 (시뮬레이션 시간으로 최대 1.5초 대기 — 실시간보다 느린 PC 에서도 동일하게 동작)
        goal_q = np.array(tr.q[-1])
        t_end = self.sim.time + 1.5
        t_wall = time.time() + 30.0
        err = 1e9
        while self.sim.time < t_end and time.time() < t_wall:
            cur = np.array([self.sim.d.qpos[a] for a in self.sim.arm_qadr[side]])
            err = float(np.max(np.abs(cur - goal_q)))
            if err < 0.03:
                break
            time.sleep(0.02)
        if err > 0.15:
            gh.abort()
            res.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            res.error_string = f"final error {err:.3f} rad (충돌로 막혔을 수 있음)"
            return res
        gh.succeed()
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return res

    def exec_gripper_traj(self, gh, side):
        traj = gh.request.trajectory
        names = list(traj.joint_names)
        if GRIPPER_JOINT[side] not in names:
            gh.abort()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return res
        i = names.index(GRIPPER_JOINT[side])
        pts = [(t, [q[i]], None) for t, q, _ in self._points(traj)]
        tr = self.sim.start_gripper_trajectory(side, pts)
        state = self._wait(gh, tr)
        time.sleep(0.3)
        res = FollowJointTrajectory.Result()
        if state == "canceled":
            gh.canceled()
        else:
            gh.succeed()   # 물체를 잡으면 목표 위치에 못 가는 것이 정상 → 성공 처리
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return res

    def exec_gripper_cmd(self, gh, side):
        target = float(np.clip(gh.request.command.position, 0.0, 0.044))
        cur = self.sim.gripper_position(side)
        dur = max(0.3, abs(target - cur) / 0.05)
        tr = self.sim.start_gripper_trajectory(side, [(dur, [target], None)])
        self._wait(gh, tr)
        time.sleep(0.3)
        pos = self.sim.gripper_position(side)
        res = GripperCommand.Result()
        res.position = pos
        res.reached_goal = abs(pos - target) < 0.003
        res.stalled = not res.reached_goal
        res.effort = 0.0
        gh.succeed()
        return res

    def destroy_node(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = MujocoBridge()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
