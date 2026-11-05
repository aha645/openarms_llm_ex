"""검증용: 정답 위치를 보며 경유지를 차례로 따라가는 단순 주행기 (SLAM 지도 만들기 테스트)."""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String

WP = [(2.5, -0.5), (3.5, -2.6), (0.0, -2.5), (-3.4, -2.6), (-3.4, 0.6), (0.0, 0.6), (0.0, 2.2),
      (2.4, 2.25), (-2.4, 2.25), (0.0, 2.2), (0.0, 0.5), (0.0, 0.0)]


class D(Node):
    def __init__(self):
        super().__init__("auto_drive", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose = None
        self.status = {}
        self.create_subscription(PoseStamped, "/ground_truth/pose", self.on_pose, 10)
        self.create_subscription(String, "/biped/status", lambda m: setattr(self, "status", json.loads(m.data)), 10)


def main():
    rclpy.init()
    n = D()
    t0 = time.time()
    i = 0
    while rclpy.ok() and i < len(WP):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.pose is None:
            continue
        x, y, yaw = n.pose
        gx, gy = WP[i]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < 0.2:
            print(f"wp {i} {WP[i]} reached t={time.time() - t0:.0f}s status={n.status}", flush=True)
            i += 1
            continue
        err = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
        t = Twist()
        t.angular.z = max(-0.5, min(0.5, 1.2 * err))
        t.linear.x = 0.25 if abs(err) < 0.35 else 0.0
        n.pub.publish(t)
        if n.status.get("fallen"):
            print("FALLEN", flush=True)
            break
    n.pub.publish(Twist())
    print("done", i, "of", len(WP), f"{time.time() - t0:.0f}s", flush=True)


def on_pose(self, m):
    q = m.pose.orientation
    self.pose = (m.pose.position.x, m.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


D.on_pose = on_pose
if __name__ == "__main__":
    main()
