"""검증용 (시뮬 전용): 팔·손이 로봇 밖 물체(작업대·상자·벽)에 닿는 순간을 그때의 작업 로그와 함께 기록.
    python3 contact_recorder.py 기록.log
/sim/objects 의 arm_contacts 가 바뀔 때마다 한 줄: '[경과 s] 접촉 목록 | 마지막 /task/log'.
쥐고 있는 상자와 손가락의 접촉(정상)은 뺀다.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
n = Node("contact_recorder")
out = open(sys.argv[1], "w", encoding="utf-8")
st = {"log": "", "att": {}, "prev": None, "t0": time.time()}


def on_objs(m):
    held = {b for b in st["att"].values() if b}
    c = [x for x in json.loads(m.data).get("arm_contacts", [])
         if not (x.split("-", 1)[1] in held and "finger" in x)]
    if c != st["prev"]:
        st["prev"] = c
        out.write(f"[{time.time() - st['t0']:6.1f}] {', '.join(c) if c else '(없음)'} | {st['log']}\n")
        out.flush()


n.create_subscription(String, "/sim/objects", on_objs, 10)
n.create_subscription(String, "/task/log", lambda m: st.__setitem__("log", m.data), 50)
n.create_subscription(String, "/sim/attached", lambda m: st.__setitem__("att", json.loads(m.data)), 10)
rclpy.spin(n)
