#!/usr/bin/env python3
"""TRON1 카메라 서버 — 머리 카메라 + 양 손목 카메라를 물리와 다른 프로세스에서 렌더링·검출·압축 발행.

    ros2 run tron1_openarm_sim tron1_cameras --ros-args -p mjcf:=<world.xml>
(sim.launch.py 가 camera_process:=true(기본)일 때 브리지와 함께 띄운다)

왜 별도 프로세스인가
  예전에는 브리지의 물리 스레드가 머리 카메라를 그리고 ArUco 를 검출했다. 그동안 GIL 을 쥐어 물리가 멈추고,
  카메라가 3 대가 되면 시뮬이 실시간을 못 따라간다. 여기서는 브리지가 공유 메모리에 쓰는 상태(sim_state_shm.py)를 읽어
  같은 MJCF 장면을 다시 만들고(운동학만), 카메라마다 렌더 → ArUco → JPEG 압축까지 한다.

발행
  머리   /aruco/detections, /aruco/poses, /aruco/markers   (브리지가 내던 것과 같은 형식, base_footprint)
         /head_camera/image_raw/compressed
  손목   /wrist_{left,right}_camera/aruco/detections  (JSON: id, 손끝(TCP) 기준 위치, base_footprint 기준 위치·yaw)
         /wrist_{left,right}_camera/image_raw/compressed
  손목 카메라는 손에 고정 → '손끝 기준 마커 위치' 는 팔 처짐·몸통 흔들림과 상관없이 정확하다 (카메라→손끝이 강체).
"""
import json
import math
import threading
import time

import cv2
import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from mobile_openarm_sim.aruco_utils import ArucoDetector
from mobile_openarm_sim.kinematics import mat_to_quat

from . import wrist_cameras as WC
from .sim_state_shm import DEFAULT_NAME, StateReader

OPT = np.diag([1.0, -1.0, -1.0])   # MuJoCo 카메라 축 → ROS 광학 축


def stamp(t):
    from builtin_interfaces.msg import Time
    s = Time()
    s.sec = int(t)
    s.nanosec = int((t - int(t)) * 1e9)
    return s


def footprint(p, R, tilt):
    """base_link → base_footprint (biped_core.base_link_to_footprint 와 같은 식, 기울기는 브리지가 추정한 값)."""
    h, pitch, roll = tilt
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    Rrp = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rrp @ p + np.array([0.0, 0.0, h]), Rrp @ R


class Cam:
    def __init__(self, m, name, width, height, hz, frame):
        self.name, self.id, self.frame = name, m.camera(name).id, frame
        self.w, self.h, self.period = width, height, 1.0 / hz
        fy = (height / 2) / math.tan(math.radians(m.cam_fovy[self.id]) / 2)
        self.K = np.array([[fy, 0, width / 2], [0, fy, height / 2], [0, 0, 1.0]])
        self.det = None
        self.last_t = -1e9
        self.last_img_t = -1e9


class CameraServer(Node):
    def __init__(self):
        super().__init__("tron1_cameras")
        p = self.declare_parameter
        p("mjcf", "")
        p("state_shm", DEFAULT_NAME)
        p("head_width", 960)
        p("head_height", 720)
        p("head_hz", 10.0)
        p("wrist_width", WC.WIDTH)
        p("wrist_height", WC.HEIGHT)
        p("wrist_hz", 10.0)
        p("image_hz", 5.0)
        p("jpeg_quality", 75)
        p("marker_size", 0.04)
        p("use_depth", True)    # RGB-D 카메라 흉내: 마커가 보이면 깊이 영상으로 3D 위치 보정 (브리지의 sim_core.maybe_camera 와 같음)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.m = mujoco.MjModel.from_xml_path(g("mjcf"))
        self.d = mujoco.MjData(self.m)
        self.base_id = self.m.body("base_link").id
        self.tcp_id = {s: self.m.body(f"openarm_{s}_hand_tcp").id for s in WC.SIDES}
        self.cams = [Cam(self.m, "head_camera", g("head_width"), g("head_height"), g("head_hz"), "head_camera_optical_frame")]
        for s in WC.SIDES:
            try:
                self.cams.append(Cam(self.m, WC.CAM_NAME[s], g("wrist_width"), g("wrist_height"), g("wrist_hz"),
                                     f"{WC.CAM_NAME[s]}_optical_frame"))
            except KeyError:
                self.get_logger().warn(f"{WC.CAM_NAME[s]} 가 MJCF 에 없음 (tools/wrist_cameras.py 로 추가)")
        for c in self.cams:
            # 손목 카메라는 파지 직전 마커가 크게(셀 25 px 넘게) 보여 원본에서 자주 놓친다 → 줄인 영상에서도 찾는다
            sc = (1.0,) if c.name == "head_camera" else (1.0, 0.5, 0.35)
            c.det = ArucoDetector("DICT_4X4_50", g("marker_size"), c.K, scales=sc)
        self.image_period = 1.0 / max(g("image_hz"), 0.1)
        self.jpeg_q = int(g("jpeg_quality"))
        self.use_depth = bool(g("use_depth"))
        self.shm_name = g("state_shm")
        # 발행
        self.pub_det = self.create_publisher(String, "/aruco/detections", 10)
        self.pub_poses = self.create_publisher(PoseArray, "/aruco/poses", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/aruco/markers", 10)
        self.pub_img = {c.name: self.create_publisher(CompressedImage, f"/{c.name}/image_raw/compressed", 2) for c in self.cams}
        self.pub_info = {c.name: self.create_publisher(CameraInfo, f"/{c.name}/camera_info", 2) for c in self.cams}
        self.pub_wdet = {c.name: self.create_publisher(String, f"/{c.name}/aruco/detections", 10)
                         for c in self.cams if c.name != "head_camera"}
        self.stats = {c.name: [0, 0.0] for c in self.cams}     # (프레임 수, 렌더+검출 시간 합)
        self.stop = threading.Event()
        threading.Thread(target=self.loop, daemon=True).start()
        self.create_timer(10.0, self.report)
        self.get_logger().info(f"카메라 서버: {[f'{c.name} {c.w}x{c.h}@{1 / c.period:.0f}Hz' for c in self.cams]}, 공유 메모리 {self.shm_name}")

    def report(self):
        s = ", ".join(f"{k} {n / 10:.1f} fps {1000 * t / max(n, 1):.0f} ms" for k, (n, t) in self.stats.items())
        self.get_logger().info(f"카메라 처리: {s}")
        self.stats = {k: [0, 0.0] for k in self.stats}

    # ------------------------------------------------------------------ 루프 (렌더러는 이 스레드에서 만들고 씀)
    def loop(self):
        reader = None
        while reader is None and not self.stop.is_set():
            try:
                reader = StateReader(self.m, self.shm_name)
            except FileNotFoundError:
                time.sleep(0.5)                     # 브리지가 아직 안 떴음
        renderers = {}
        for c in self.cams:
            key = (c.w, c.h)
            if key not in renderers:
                renderers[key] = mujoco.Renderer(self.m, c.h, c.w)
        tilt = (0.0, 0.0, 0.0)
        t = -1.0
        while not self.stop.is_set() and rclpy.ok():
            got = reader.read(self.d)
            if got is None:
                time.sleep(0.003)
                continue
            t, tilt = got
            due = [c for c in self.cams if t - c.last_t >= c.period - 1e-6]
            if not due:
                continue
            mujoco.mj_kinematics(self.m, self.d)
            mujoco.mj_camlight(self.m, self.d)
            for c in due:
                c.last_t = t
                t0 = time.perf_counter()
                r = renderers[(c.w, c.h)]
                r.update_scene(self.d, camera=c.id)
                r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                rgb = r.render().copy()
                dets = c.det.detect(rgb)
                if dets and self.use_depth:
                    self._depth_correct(r, c, dets)
                self.publish_dets(c, t, dets, tilt)
                if t - c.last_img_t >= self.image_period - 1e-6:
                    c.last_img_t = t
                    self.publish_image(c, t, rgb, dets)
                st = self.stats[c.name]
                st[0] += 1
                st[1] += time.perf_counter() - t0
        reader.close()
        for r in renderers.values():
            r.close()

    def _depth_correct(self, r, c, dets):
        """깊이 영상으로 마커 중심 3D 위치 보정. 960x720 머리 카메라에서 0.65 m 거리의 4 cm 마커는 약 23 px 라
        PnP 깊이 오차가 4 % (2.6 cm) — 깊이 보정 없이 내보냈더니 상자가 2~4 cm 높게 잡혀 파지가 실패했다."""
        r.enable_depth_rendering()
        r.update_scene(self.d, camera=c.id)
        depth = r.render().copy()
        r.disable_depth_rendering()
        Kinv = np.linalg.inv(c.K)
        for det in dets:
            u, v = det["corners"].mean(0)
            ui, vi = int(round(u)), int(round(v))
            if 0 <= vi < depth.shape[0] and 0 <= ui < depth.shape[1]:
                z = float(np.median(depth[max(vi - 1, 0):vi + 2, max(ui - 1, 0):ui + 2]))
                if 0.03 < z < 5.0:
                    det["position"] = z * (Kinv @ np.array([u, v, 1.0]))

    # ------------------------------------------------------------------ 발행
    def _cam_in_world(self, c):
        return self.d.cam_xmat[c.id].reshape(3, 3) @ OPT, self.d.cam_xpos[c.id].copy()

    def _flat_rotation(self, c, det, Rw, pw, pm_w, max_tilt=math.radians(35)):
        """마커 방향(세계 기준 회전). 마커가 거의 수평(PnP 법선이 위쪽 35° 안)이면 수평면 가정으로 다시 구한다:
        네 모서리 시선을 마커 중심 높이의 수평면과 교차시켜 모서리 0→1 방향으로 yaw.
        작은 마커를 비스듬히 볼 때 PnP 자세는 기울기를 헷갈려(평면 마커 모호성) yaw 가 틀렸다:
        머리 카메라 0.64 m 에서 −8° 상자를 −18° 로 재, 손을 10° 더 돌려 들어가 집기 실패 (집기 견고성 시험 s1d)."""
        Rm_w = Rw @ det["rotation"]
        if Rm_w[2, 2] < math.cos(max_tilt):
            return Rm_w
        Kinv = np.linalg.inv(c.K)
        pts = []
        for u, v in det["corners"]:
            ray = Rw @ (Kinv @ np.array([u, v, 1.0]))
            if abs(ray[2]) < 1e-6:
                return Rm_w
            pts.append(pw + ray * ((pm_w[2] - pw[2]) / ray[2]))
        pts = np.array(pts)
        ex = (pts[1] - pts[0]) + (pts[2] - pts[3])          # 마커 x축 (윗변·아랫변 평균)
        yaw = math.atan2(ex[1], ex[0])
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

    def publish_dets(self, c, t, dets, tilt):
        Rw, pw = self._cam_in_world(c)
        Rb, pb = self.d.xmat[self.base_id].reshape(3, 3), self.d.xpos[self.base_id]
        out = []
        for det in dets:
            pm_w = Rw @ det["position"] + pw
            Rm_w = self._flat_rotation(c, det, Rw, pw, pm_w)
            p_fp, R_fp = footprint(Rb.T @ (pm_w - pb), Rb.T @ Rm_w, tilt)
            item = {"id": det["id"], "frame": "base_footprint", "position": [round(float(v), 4) for v in p_fp],
                    "R": R_fp, "yaw": round(math.atan2(R_fp[1, 0], R_fp[0, 0]), 4),
                    "dist": round(float(np.linalg.norm(det["position"])), 4)}
            if c.name != "head_camera":
                side = "left" if "left" in c.name else "right"
                tid = self.tcp_id[side]
                Rt, pt = self.d.xmat[tid].reshape(3, 3), self.d.xpos[tid]
                item["position_tcp"] = [round(float(v), 5) for v in Rt.T @ (pm_w - pt)]
            out.append(item)
        if c.name == "head_camera":
            self._publish_head(t, out)
        else:
            msg = String()
            msg.data = json.dumps({"stamp": t, "camera": c.name,
                                   "detections": [{k: v for k, v in o.items() if k != "R"} for o in out]})
            self.pub_wdet[c.name].publish(msg)

    def _publish_head(self, t, out):
        """브리지(bridge_node.publish_detections)가 내던 것과 같은 형식 — 집기 앱은 바꾸지 않아도 된다."""
        pa = PoseArray()
        pa.header.stamp = stamp(t)
        pa.header.frame_id = "base_footprint"
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        js = []
        for o in out:
            qw, qx, qy, qz = mat_to_quat(o["R"])
            js.append({"id": o["id"], "frame": "base_footprint", "position": o["position"],
                       "quaternion_xyzw": [float(qx), float(qy), float(qz), float(qw)], "yaw": o["yaw"]})
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, o["position"])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = float(qx), float(qy), float(qz), float(qw)
            pa.poses.append(pose)
            mk = Marker()
            mk.header = pa.header
            mk.ns, mk.id, mk.type, mk.action = "aruco", o["id"], Marker.CUBE, Marker.ADD
            mk.pose = pose
            mk.scale.x = mk.scale.y = 0.05
            mk.scale.z = 0.005
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.9, 0.2, 0.8
            ma.markers.append(mk)
        msg = String()
        msg.data = json.dumps({"stamp": t, "detections": js})
        self.pub_det.publish(msg)
        self.pub_poses.publish(pa)
        self.pub_markers.publish(ma)

    def publish_image(self, c, t, rgb, dets):
        ok, buf = cv2.imencode(".jpg", ArucoDetector.draw(rgb, dets), [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_q])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp(t)
        msg.header.frame_id = c.frame
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self.pub_img[c.name].publish(msg)
        ci = CameraInfo()
        ci.header = msg.header
        ci.width, ci.height = c.w, c.h
        K = c.K
        ci.k = [float(v) for v in K.reshape(-1)]
        ci.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.d = [0.0] * 5
        ci.distortion_model = "plumb_bob"
        self.pub_info[c.name].publish(ci)


def main():
    rclpy.init()
    node = CameraServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop.set()
        time.sleep(0.2)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
