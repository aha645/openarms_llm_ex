#!/usr/bin/env python3
"""실로봇용 ArUco 검출 노드 — 시뮬레이션 브리지 내부 검출과 같은 /aruco/detections 를 발행.

시뮬레이션에서는 mujoco_bridge 가 카메라를 직접 렌더·검출하므로 이 노드가 필요 없다.
실로봇(예: RealSense)에서는 이 노드가 그 역할을 대신한다.

    ros2 run mobile_openarm_apps aruco_detector --ros-args \
        -p image_topic:=/camera/color/image_raw/compressed \
        -p info_topic:=/camera/color/camera_info \
        -p depth_topic:=/camera/aligned_depth_to_color/image_raw     # 없으면 PnP 만 사용

발행: /aruco/detections (std_msgs/String, base_footprint 기준 JSON — robot_skills.py 가 읽는 형식)
"""
import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from mobile_openarm_sim.aruco_utils import ArucoDetector



def tf_to_mat(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    p = t.transform.translation
    return R, np.array([p.x, p.y, p.z])


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.declare_parameter("image_topic", "/head_camera/image_raw/compressed")
        self.declare_parameter("info_topic", "/head_camera/camera_info")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_size", 0.04)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("output_topic", "/aruco/detections")
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base = g("base_frame")
        self.det = None
        self.dict_name, self.size = g("dictionary"), g("marker_size")
        self.depth = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(String, g("output_topic"), 10)
        self.create_subscription(CameraInfo, g("info_topic"), self.on_info, 2)
        self.create_subscription(CompressedImage, g("image_topic"), self.on_image, qos_profile_sensor_data)
        if g("depth_topic"):
            self.create_subscription(Image, g("depth_topic"), self.on_depth, qos_profile_sensor_data)

    def on_info(self, msg):
        if self.det is None:
            K = np.array(msg.k).reshape(3, 3)
            self.det = ArucoDetector(self.dict_name, self.size, K, np.array(msg.d) if len(msg.d) else None)
            self.K = K
            self.get_logger().info(f"camera_info 수신: fx={K[0, 0]:.1f}")

    def on_depth(self, msg):
        dt = np.uint16 if msg.encoding in ("16UC1", "mono16") else np.float32
        d = np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.width)
        self.depth = d.astype(np.float32) * (0.001 if dt == np.uint16 else 1.0)

    def on_image(self, msg):
        if self.det is None:
            return
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        dets = self.det.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        try:
            t = self.tf_buffer.lookup_transform(self.base, msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp),
                                                rclpy.duration.Duration(seconds=0.2))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"TF {self.base}<-{msg.header.frame_id} 없음: {e}", throttle_duration_sec=5.0)
            return
        R, p = tf_to_mat(t)
        out = []
        for d in dets:
            pos = d["position"]
            if self.depth is not None:
                u, v = d["corners"].mean(0)
                vi, ui = int(round(v)), int(round(u))
                if 0 <= vi < self.depth.shape[0] and 0 <= ui < self.depth.shape[1]:
                    z = float(np.median(self.depth[max(vi - 1, 0):vi + 2, max(ui - 1, 0):ui + 2]))
                    if 0.05 < z < 5.0:
                        pos = z * (np.linalg.inv(self.K) @ np.array([u, v, 1.0]))
            pb = R @ pos + p
            Rb = R @ d["rotation"]
            out.append({"id": d["id"], "frame": self.base, "position": [round(float(x), 4) for x in pb],
                        "yaw": round(float(np.arctan2(Rb[1, 0], Rb[0, 0])), 4)})
        s = String()
        s.data = json.dumps({"stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, "detections": out})
        self.pub.publish(s)


def main():
    rclpy.init()
    rclpy.spin(ArucoNode())


if __name__ == "__main__":
    main()
