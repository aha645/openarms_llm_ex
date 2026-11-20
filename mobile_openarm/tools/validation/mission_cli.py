"""검증용: /task/command 로 자연어 명령을 보내고 /task/status·/task/log 를 끝까지 기록.
    python3 mission_cli.py "빨간 상자를 왼쪽 작업대로 옮기고 돌아와" 1500     (두 번째 인자: 최대 대기 초)
    python3 mission_cli.py @명령.txt                                       (UTF-8 파일에서 명령 읽기)
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

cmd = sys.argv[1]
if cmd.startswith("@"):
    cmd = open(cmd[1:], encoding="utf-8").read().strip()
rclpy.init()
n = Node("mission_test")
pub = n.create_publisher(String, "/task/command", 10)
st = {"v": None}
logs = []
n.create_subscription(String, "/task/status", lambda m: st.__setitem__("v", json.loads(m.data)), 10)
n.create_subscription(String, "/task/log", lambda m: (logs.append(m.data), print("LOG", m.data, flush=True)), 50)
t0 = time.time()
while time.time() - t0 < 30 and st["v"] is None:
    rclpy.spin_once(n, timeout_sec=0.2)
# 명령 토픽 구독자(작업 관리자)와 연결될 때까지 기다렸다 보낸다. 상태 토픽이 먼저 연결돼도 명령 토픽은 아직일 수 있어
# 명령이 버려지고 미션이 '대기 중' 에 멈췄다. 5 s 안에 대기 중을 벗어나지 않으면 다시 보낸다 (최대 3 번)
while time.time() - t0 < 30 and pub.get_subscription_count() == 0:
    rclpy.spin_once(n, timeout_sec=0.2)
time.sleep(1)
for k in range(3):
    pub.publish(String(data=cmd))
    print("sent" if k == 0 else f"다시 보냄 {k}", cmd, flush=True)
    t_sent = time.time()
    while time.time() - t_sent < 5:
        rclpy.spin_once(n, timeout_sec=0.2)
        if st["v"] and st["v"]["state"] != "idle":
            break
    if st["v"] and st["v"]["state"] != "idle":
        break
last = None
while time.time() - t0 < float(sys.argv[2] if len(sys.argv) > 2 else 2400):
    rclpy.spin_once(n, timeout_sec=0.2)
    v = st["v"]
    if v and (v["state"], v["current"]) != last:
        last = (v["state"], v["current"])
        print(f"[{time.time() - t0:5.0f}s] state={v['state']} step={v['current']} msg={v['message']}", flush=True)
        if v["state"] in ("done", "failed", "rejected") and time.time() - t0 > 5:
            break
print("FINAL", json.dumps(st["v"], ensure_ascii=False)[:600], flush=True)
