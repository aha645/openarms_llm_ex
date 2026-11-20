#!/usr/bin/env python3
"""TRON1 + OpenArm 용 MuJoCo ↔ ROS 2 브리지 노드 (실행 파일 tron1_bridge).

보행 정책으로 걷는 시뮬레이션 코어(biped_core.BipedOpenArmSim)를 ROS 2 토픽·액션·서비스로 노출한다.
  * /cmd_vel 의 linear.y(옆걸음)도 받음 — 작업대 앞에서 상자 옆으로 비켜 서기 (robot_skills.strafe_base)
  * base_link → base_footprint: 추정한 몸통 높이·기울기(IMU + 다리 기구학)로 ArUco 위치를 바꾼다.
    같은 값을 가상 관절(base_height/pitch/roll_joint)로 /joint_states 에 내므로 TF·MoveIt 과 맞는다 (tools/base_tilt_urdf.py)
  * box_poses 파라미터: 상자 위치·방향 바꾸기 (반복 집기 시험)

발행
  /clock                         rosgraph_msgs/Clock        (시뮬레이션 시간, 100Hz)
  /joint_states                  sensor_msgs/JointState     (50Hz, 팔 + 다리 + 몸통 기울기 가상 관절)
  /odom  + TF odom→base_footprint nav_msgs/Odometry          (50Hz, 다리 오도메트리, linear.y = 옆걸음 속도)
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
  /biped/status                  std_msgs/String (JSON)     (5Hz: 몸통 높이·기울기·넘어짐 여부·정책이 받는 명령·손끝 정답 위치)
구독
  /cmd_vel                       geometry_msgs/Twist        (linear.x, linear.y, angular.z)
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
import sys
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
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .biped_core import BipedOpenArmSim
from .color_boxes import hide_box_markers
from .kinematics import mat_to_quat
from .sim_core import ARM_JOINTS, GRIPPER_JOINT
from .sim_state_shm import StateWriter


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


def parse_box_poses(text):
    """"box_0=x,y,yaw;box_1=..." → {name: (x, y, yaw_deg)}. yaw 는 생략 가능 (0)."""
    out = {}
    for item in filter(None, (t.strip() for t in text.split(";"))):
        name, vals = item.split("=")
        v = [float(t) for t in vals.split(",")]
        out[name.strip()] = (v[0], v[1], v[2] if len(v) > 2 else 0.0)
    return out


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")
        share = get_package_share_directory("tron1_openarm_sim")
        p = self.declare_parameter
        p("mjcf", os.path.join(share, "mjcf", "sf", "world.xml"))
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
        p("state_shm", "")             # 비어 있지 않으면 카메라 서버(camera_server.py)용 시뮬 상태 공유 메모리 이름
        p("box_markers", True)         # False: 상자 윗면 마커를 지움 (box_source:=color, 색으로 인식)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.sim = BipedOpenArmSim(
            gp("mjcf"), camera_width=gp("camera_width"), camera_height=gp("camera_height"),
            camera_hz=gp("camera_hz"), enable_camera=gp("camera"), lidar_rays=gp("lidar_rays"),
            lidar_hz=gp("lidar_hz"), grasp_assist=gp("grasp_assist"), move_person=gp("move_person"),
            person_speed=gp("person_speed"))
        if gp("start_x") or gp("start_y") or gp("start_yaw"):
            self.sim.set_base_pose(gp("start_x"), gp("start_y"), gp("start_yaw"))
        # 카메라 별도 프로세스 (camera_server.py): 브리지는 렌더링에 필요한 상태만 공유 메모리에 쓴다.
        # 이때 sim.launch.py 는 브리지의 camera 를 끈다 (머리·손목 카메라와 ArUco 를 카메라 서버가 대신 낸다).
        # 물리 스레드(on_tick)가 시작되기 전에 만들어 둔다
        self._state_writer = StateWriter(self.sim.m, gp("state_shm")) if gp("state_shm") else None
        if self._state_writer:
            self.get_logger().info(f"시뮬 상태 공유 메모리 '{gp('state_shm')}' → 카메라 서버")
        if not gp("box_markers"):
            # 상자 마커 없이: 뷰어에도 마커 없는 색 상자로 보이게 (카메라 서버도 같은 일을 한다)
            self.get_logger().info(f"상자 마커 {hide_box_markers(self.sim.m)} 개를 지움 (색으로 인식)")
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

        self._before_physics()
        self.stop_event = threading.Event()
        # SIGINT may invalidate the ROS context before main() reaches finally.
        self.context.on_shutdown(self.stop_event.set)
        self.thread = threading.Thread(target=self.physics_loop, daemon=True)
        self.thread.start()
        self.get_logger().info("MuJoCo bridge started (viewer=%s, camera=%s)" % (self.viewer_enabled, gp("camera")))
        self.pub_status = self.create_publisher(String, "/biped/status", 10)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info(f"TRON1 보행 정책 사용 (발 형태 {self.sim.foot}, 정책 50 Hz, 물리 {self.sim.m.opt.timestep * 1000:.0f} ms)")

    def _before_physics(self):
        """물리 스레드를 시작하기 전에 시뮬 상태를 바꾼다: 상자 위치 바꾸기 (반복 집기 시험).
        box_poses = "box_0=2.60,2.98,15;box_1=2.43,3.00,-10" = 월드 x, y [m], yaw [도].
        스레드가 돌기 시작한 뒤 메인 스레드에서 mj_forward 로 상태를 바꾸면, 물리 스레드가 뷰어를 만드는 중
        (launch_passive, 잠금 없이 데이터를 읽음)과 겹쳐 브리지가 세그멘테이션 오류(−11)로 죽었다 (연속 집기 시험, 두 번에 한 번꼴)."""
        self.declare_parameter("box_poses", "")
        self._box_poses = parse_box_poses(self.get_parameter("box_poses").value)
        self._apply_box_poses()

    def _apply_box_poses(self):
        for name, (x, y, yaw) in self._box_poses.items():
            self.sim.set_box_pose(name, x, y, math.radians(yaw))
            self.get_logger().info(f"상자 {name} → ({x:.3f}, {y:.3f}), {yaw:+.0f}°")

    # ------------------------------------------------------------ physics thread
    def physics_loop(self):
        viewer = None
        if self.viewer_enabled:
            try:
                import mujoco.viewer
                with self.sim.lock:         # 뷰어가 처음 장면을 만드는 동안 다른 스레드가 데이터를 바꾸지 않게
                    viewer = mujoco.viewer.launch_passive(self.sim.m, self.sim.d,
                                                          show_left_ui=False, show_right_ui=False)
                self._track_robot(viewer)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"viewer 를 열 수 없음 → headless 로 계속: {e}")
                viewer = None
        try:
            self.sim.run_realtime(self.stop_event, rate_factor=self.rate_factor,
                                  on_tick=self.on_tick, tick_hz=100.0, viewer=viewer)
        except _rclpy.RCLError:
            # A signal can invalidate the context between ok() and publish().
            # Do not conceal publication failures while ROS is still running.
            if self.context.ok():
                raise
        finally:
            self.stop_event.set()
            try:
                # Renderer GL resources must be released on the physics thread
                # that created them, before closing the passive GLFW viewer.
                if self.sim.renderer is not None:
                    self.sim.renderer.close()
                    self.sim.renderer = None
            finally:
                try:
                    if viewer is not None:
                        viewer.close()
                finally:
                    rclpy.try_shutdown(context=self.context)

    def _track_robot(self, viewer):
        """관찰 카메라가 로봇을 따라가며 화면 중앙에 두게 한다.
        MuJoCo 추적 카메라(mjCAMERA_TRACKING)는 몸통 위치를 그대로 따라가서, 걸을 때 걸음마다 몸통이
        위아래·앞뒤로 1~2 cm 출렁이는 것이 화면에서는 바닥·건물 전체가 흔들리는 것으로 보였다 (사용자 지적).
        → 자유 카메라로 두고, 시선 중심을 몸통의 바닥 위 위치(x, y)만 부드럽게(시간 상수 TRACK_TAU) 따라가게 하며
        높이는 고정한다. 마우스로 돌리기·확대는 그대로. 뷰어에서 다른 카메라를 고르면 따라가기를 멈춘다."""
        import mujoco
        m = self.sim.m
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if bid < 0:
            return
        TRACK_TAU, LOOK_Z = 0.7, 0.8
        state = {"t": None}

        def hook(v):
            if v.cam.type != mujoco.mjtCamera.mjCAMERA_FREE:
                return
            p = self.sim.d.xpos[bid]
            now = time.monotonic()
            a = 1.0 if state["t"] is None else min(1.0, (now - state["t"]) / TRACK_TAU)
            state["t"] = now
            v.cam.lookat[0] += a * (p[0] - v.cam.lookat[0])
            v.cam.lookat[1] += a * (p[1] - v.cam.lookat[1])
            v.cam.lookat[2] = LOOK_Z

        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.distance = 2.5          # 로봇과 앞 작업대가 함께 보이는 거리
            viewer.cam.elevation = -25.0
            # 시작 방향(start_yaw)의 뒤쪽 30° 비스듬히 위에서 → 로봇과 그 앞(작업대)이 함께 보임.
            # 시선 중심만 따라가고 방향은 월드 기준이라, 로봇이 돌면 마우스로 돌려 보면 된다
            viewer.cam.azimuth = math.degrees(self.get_parameter("start_yaw").value) - 30.0
            hook(viewer)
            self.sim.view_hook = hook
            # 그림자·반사 끔(모델의 조명·재질 값 — 그리기에만 쓰이고 물리와 무관, 카메라 서버는 자기 모델을 따로 씀):
            # 뷰어가 한 장면을 그리는 동안(sync 가 기다림) 물리가 멈춘다. 전체 메시 모델에서
            # sync 가 10 s 중 7.8 s 를 차지해 시뮬레이션이 실시간의 0.6 배로 돌았다 (팔 이동 1 회 40 s 넘게)
            m.light_castshadow[:] = 0
            m.mat_reflectance[:] = 0

    def _state_tilt(self):
        """카메라 서버에 넘기는 base_link → base_footprint (높이, pitch, roll): 몸통이 숙여진 만큼."""
        return self.sim.base_tilt

    def on_tick(self, sim):
        if self.stop_event.is_set() or not self.context.ok():
            return
        if self._state_writer is not None:
            self._state_writer.write(sim.d, self._state_tilt())
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
        objs.data = json.dumps({"frame": "world", "objects": self.sim.object_poses(),
                                "arm_contacts": self.sim.arm_contacts()})
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
        od.twist.twist.linear.y = float(self.sim.odom_vy)   # 옆걸음 — Nav2 Omni·EKF 가 씀
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
        # 공분산 (robot_localization 은 0 이면 측정을 거의 무한히 믿는다)
        m.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.003]
        m.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        m.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
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

    def _base_link_to_footprint(self, p, R):
        """base_link 좌표 → base_footprint 좌표: 높이와 몸통이 숙여진(최대 7°) 만큼의 회전 (TF 의 가상 관절과 같은 값)."""
        with self.sim.lock:
            return self.sim.base_link_to_footprint(p, R)

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
            p, R = self._base_link_to_footprint(det["position_base"], det["rotation_base"])
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
        self.sim.set_cmd_vel(msg.linear.x, msg.angular.z, msg.linear.y)

    def on_reset(self, req, res):
        # 주의: 시뮬레이션 시간이 0 으로 돌아가므로 Nav2/SLAM/MoveIt 은 재시작하는 것이 안전
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.sim.reset(gp("start_x"), gp("start_y"), gp("start_yaw"))
        self._last_js_t = self._last_img_t = -1e9
        self._apply_box_poses()
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

    def _tcp_world(self, side):
        d, m = self.sim.d, self.sim.m
        h = m.body(f"openarm_{side}_hand").id
        with self.sim.lock:
            return d.xpos[h] + d.xmat[h].reshape(3, 3) @ [0.0, 0.0, 0.09]

    def publish_status(self):
        h, tilt = self.sim.base_height_and_tilt()
        msg = String()
        msg.data = json.dumps({"foot": self.sim.foot, "standing": bool(self.sim.policy.standing),
                               "base_height": round(h, 3), "tilt_deg": round(tilt, 1), "fallen": self.sim.fallen,
                               "cmd": [round(float(x), 3) for x in self.sim.cmd_eff],
                               "est_vel": [round(float(x), 3) for x in self.sim.policy.estimated_velocity],
                               # 디버그·채점용 정답값(시뮬에서만): 손끝(TCP) 세계 위치 = 손 + 손 z축으로 0.09 m (URDF hand_tcp 와 같음)
                               "tcp_world": {s: [round(float(v), 4) for v in self._tcp_world(s)] for s in ("left", "right")},
                               # 디버그: 팔·손이 로봇 밖의 물체(작업대·상자·벽)와 닿아 있는 접촉 (다리는 로봇 몸)
                               "arm_contacts": self.sim.arm_contacts(),
                               # 디버그: 몸통 정답 자세(map=world 원점 기준)와 다리 오도메트리 (제자리 흐름 분석용)
                               "gt_pose": [round(float(v), 4) for v in self.sim.ground_truth_pose()],
                               "odom": [round(float(v), 4) for v in self.sim.odom]})
        self.pub_status.publish(msg)
        if self.sim.fallen:
            self.get_logger().error("로봇이 넘어졌습니다 → ros2 service call /sim/reset std_srvs/srv/Trigger",
                                    throttle_duration_sec=5.0)

    def destroy_node(self):
        self.stop_event.set()
        # Never destroy publishers while the physics thread can still use them.
        if threading.current_thread() is not self.thread:
            self.thread.join()
        return super().destroy_node()


def main():
    # GIL 전환 간격 5 ms → 0.5 ms. 물리 스레드는 스텝마다 짧은 파이썬 연산(보행 정책·제어기·중력 보상)을 수백 번 하고,
    # 그때마다 GIL 을 다시 받아야 한다. /cmd_vel 이 들어오는 동안 rclpy MultiThreadedExecutor 가 대기 집합을 파이썬으로
    # 다시 만들며 GIL 을 58~76 % 쥐어, 물리 스레드가 매번 최대 5 ms 씩 기다려 걷는 동안 실시간의 0.1~0.4 배로 느려지고
    # 몇 초씩 멈췄다 (Nav2 lifecycle heartbeat 끊김 → 주행 중단). 0.5 ms 로 줄이면 걷는 동안에도 실시간 1.00 (검증 기록 11절)
    sys.setswitchinterval(0.0005)
    rclpy.init()
    node = MujocoBridge()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_event.set()
        if node._state_writer:
            node._state_writer.close()
        ex.shutdown(wait_for_threads=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
