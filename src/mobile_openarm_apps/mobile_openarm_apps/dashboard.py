#!/usr/bin/env python3
"""Day2-06 웹 대시보드 (Flask · SSE · MJPEG).

    ros2 run mobile_openarm_apps dashboard        → 브라우저 http://localhost:8080

  /            지도·로봇 위치·미션 상태·카메라·명령 입력 화면
  /events      Server-Sent Events: 0.5초마다 상태 JSON 푸시
  /camera.mjpg MJPEG 스트림 (/head_camera/image_raw/compressed 를 그대로 중계)
  /map.png     /map(OccupancyGrid) 를 PNG 로
  POST /api/command {"text": "..."}  → /task/command
  POST /api/cancel                    → /task/cancel
"""
import json
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from flask import Flask, Response, jsonify, request, send_from_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .geometry import quat_to_yaw


class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.declare_parameter("port", 8080)
        self.jpeg = None
        self.jpeg_seq = 0
        self.jpeg_cv = threading.Condition()
        self.task = {"state": "unknown", "message": "task_manager 미실행", "steps": []}
        self.logs = deque(maxlen=40)
        self.detections = []
        self.attached = {}
        self.map_png = None
        self.map_meta = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(String, "/task/command", 10)
        self.cancel_cli = self.create_client(Trigger, "/task/cancel")
        self.create_subscription(CompressedImage, "/head_camera/image_raw/compressed", self.on_img, 2)
        self.create_subscription(String, "/task/status", self.on_task, 10)
        self.create_subscription(String, "/task/log", lambda m: self.logs.append(
            {"t": time.strftime("%H:%M:%S"), "msg": m.data}), 50)
        self.create_subscription(String, "/aruco/detections", self.on_det, 10)
        self.create_subscription(String, "/sim/attached", lambda m: setattr(self, "attached", json.loads(m.data)), 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)

    def on_img(self, msg):
        with self.jpeg_cv:
            self.jpeg = bytes(msg.data)
            self.jpeg_seq += 1
            self.jpeg_cv.notify_all()

    def on_task(self, msg):
        self.task = json.loads(msg.data)

    def on_det(self, msg):
        self.detections = json.loads(msg.data).get("detections", [])

    def on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 205, np.uint8)
        img[data == 0] = 250
        img[data > 50] = 30
        img = img[::-1]                     # 이미지 위쪽 = +y
        ok, buf = cv2.imencode(".png", img)
        if ok:
            self.map_png = buf.tobytes()
            o = msg.info.origin.position
            self.map_meta = {"width": w, "height": h, "resolution": msg.info.resolution,
                             "origin": [o.x, o.y], "stamp": time.time()}

    def pose(self):
        for frame in ("map", "odom"):
            try:
                t = self.tf_buffer.lookup_transform(frame, "base_footprint", rclpy.time.Time())
                q = t.transform.rotation
                return {"frame": frame, "x": t.transform.translation.x, "y": t.transform.translation.y,
                        "yaw": quat_to_yaw(q.x, q.y, q.z, q.w)}
            except Exception:  # noqa: BLE001
                continue
        return None


def make_app(node):
    web = os.path.join(get_package_share_directory("mobile_openarm_apps"), "web")
    sim_share = get_package_share_directory("mobile_openarm_sim")
    import yaml
    world = yaml.safe_load(open(os.path.join(sim_share, "config", "locations.yaml"), encoding="utf-8"))
    app = Flask(__name__, static_folder=None)

    @app.route("/")
    def index():
        return send_from_directory(web, "index.html")

    @app.route("/events")
    def events():
        def gen():
            while True:
                payload = {"pose": node.pose(), "task": node.task, "logs": list(node.logs)[-15:],
                           "detections": node.detections, "attached": node.attached,
                           "map": node.map_meta, "locations": world["locations"],
                           "camera": node.jpeg is not None}
                yield f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"
                time.sleep(0.5)
        return Response(gen(), mimetype="text/event-stream",
                        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    @app.route("/camera.mjpg")
    def camera():
        def gen():
            last = -1
            while True:
                with node.jpeg_cv:
                    node.jpeg_cv.wait_for(lambda: node.jpeg_seq != last, timeout=2.0)
                    frame, last = node.jpeg, node.jpeg_seq
                if frame is None:
                    continue
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode()
                       + b"\r\n\r\n" + frame + b"\r\n")
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/map.png")
    def map_png():
        if node.map_png is None:
            return Response(status=404)
        return Response(node.map_png, mimetype="image/png", headers={"Cache-Control": "no-cache"})

    @app.route("/api/command", methods=["POST"])
    def command():
        text = (request.get_json(silent=True) or {}).get("text", "").strip()
        if not text:
            return jsonify(ok=False, error="빈 명령"), 400
        m = String()
        m.data = text
        node.cmd_pub.publish(m)
        node.logs.append({"t": time.strftime("%H:%M:%S"), "msg": f"[web] 명령: {text}"})
        return jsonify(ok=True)

    @app.route("/api/cancel", methods=["POST"])
    def cancel():
        if node.cancel_cli.service_is_ready():
            node.cancel_cli.call_async(Trigger.Request())
            return jsonify(ok=True)
        return jsonify(ok=False, error="task_manager 없음"), 503

    return app


def main():
    rclpy.init()
    node = DashboardNode()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    port = node.get_parameter("port").value
    node.get_logger().info(f"dashboard: http://localhost:{port}")
    try:
        make_app(node).run(host="0.0.0.0", port=port, threaded=True, use_reloader=False)
    finally:
        ex.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
