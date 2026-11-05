"""검증용 정답 기록기 (시뮬 전용): /biped/status 의 손끝 정답 위치·팔 접촉·기울기, /sim/objects 의 상자 정답 위치,
MoveIt 이 생각하는 손끝(TF base_footprint→openarm_right_hand_tcp), 카메라가 잰 상자(/aruco/detections) 를 0.2 s 마다 JSON 한 줄씩 기록.
    python3 grasp_recorder.py 기록.jsonl
"""
import rclpy, json, time, sys
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("rec2"); out = open(sys.argv[1], "w"); st = {}
buf = Buffer(); TransformListener(buf, n)
def ob(m): st["obj"] = json.loads(m.data)["objects"]
def det(m):
    for d in json.loads(m.data).get("detections", []):
        if d["id"] == 0: st["det0"] = d["position"]
def bs(m):
    b = json.loads(m.data); o = st.get("obj")
    try:
        tf = buf.lookup_transform("base_footprint", "openarm_right_hand_tcp", rclpy.time.Time())
        tcp_b = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
    except Exception:
        tcp_b = None
    if o: out.write(json.dumps({"t": time.time(), "tcp": b["tcp_world"]["right"], "box0": o["box_0"], "tcp_b": tcp_b, "det0": st.get("det0"),
                                "tilt": b["tilt_deg"], "h": b["base_height"], "ac": b.get("arm_contacts"), "st": b.get("standing"),
                                "gt": b.get("gt_pose"), "odom": b.get("odom"), "cmd": b.get("cmd")}) + "\n"); out.flush()
n.create_subscription(String, "/sim/objects", ob, 10); n.create_subscription(String, "/biped/status", bs, 10)
n.create_subscription(String, "/aruco/detections", det, 10)
rclpy.spin(n)
