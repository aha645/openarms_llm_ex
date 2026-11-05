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
n.create_subscription(String, "/biped/status", lambda m: st.__setitem__("b", json.loads(m.data)), 10)
t0 = time.time()
while time.time() - t0 < 30 and st["v"] is None:
    rclpy.spin_once(n, timeout_sec=0.2)
time.sleep(1)
pub.publish(String(data=cmd))
print("sent", cmd, flush=True)
last = None
while time.time() - t0 < float(sys.argv[2] if len(sys.argv) > 2 else 2400):
    rclpy.spin_once(n, timeout_sec=0.2)
    v = st["v"]
    if v and (v["state"], v["current"]) != last:
        last = (v["state"], v["current"])
        print(f"[{time.time() - t0:5.0f}s] state={v['state']} step={v['current']} msg={v['message']} biped={st.get('b')}", flush=True)
        if v["state"] in ("done", "failed", "rejected") and time.time() - t0 > 5:
            break
print("FINAL", json.dumps(st["v"], ensure_ascii=False)[:600], flush=True)
