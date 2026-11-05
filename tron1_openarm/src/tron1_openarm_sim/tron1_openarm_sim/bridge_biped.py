#!/usr/bin/env python3
"""TRON1 + OpenArm 용 MuJoCo ↔ ROS 2 브리지.

바퀴형 브리지(mobile_openarm_sim/bridge_node.py)를 그대로 쓰고, 시뮬레이션 코어만 이족용으로 바꾼다.
→ 토픽·액션·서비스 이름(/cmd_vel, /odom, /scan, /joint_states, 팔 FollowJointTrajectory, /aruco/*, /sim/*)이
  바퀴형과 100% 같으므로 SLAM·Nav2·MoveIt·DAY2 앱이 코드 수정 없이 동작한다.

달라진 점
  * MobileOpenArmSim → BipedOpenArmSim (보행 정책·다리 오도메트리)
  * BASE_LINK_Z: 0.08(바퀴 반지름) → 0.68(TRON1 서 있는 높이). ArUco 위치를 base_footprint 기준으로 바꿀 때 사용
  * /cmd_vel 의 linear.y(옆걸음)도 받음 — 작업대 앞에서 상자 옆으로 비켜 서기 (robot_skills.strafe_base)
  * /biped/status (std_msgs/String JSON): 몸통 높이·기울기·넘어짐 여부·정책이 받는 명령
"""
import json

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from mobile_openarm_sim import bridge_node as B

from .biped_core import NOMINAL_BASE_HEIGHT, BipedOpenArmSim

B.MobileOpenArmSim = BipedOpenArmSim
B.BASE_LINK_Z = NOMINAL_BASE_HEIGHT        # 시뮬 생성 뒤 발 형태(pf 0.68 / sf 0.73)에 맞춰 다시 설정


class BipedBridge(B.MujocoBridge):
    def __init__(self):
        super().__init__()
        B.BASE_LINK_Z = self.sim.nominal_height
        self.pub_status = self.create_publisher(String, "/biped/status", 10)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info(f"TRON1 보행 정책 사용 (발 형태 {self.sim.foot}, 정책 50 Hz, 물리 {self.sim.m.opt.timestep * 1000:.0f} ms)")

    def on_cmd_vel(self, msg):
        # 바퀴형은 linear.x·angular.z 만 쓴다. 이족은 linear.y(옆걸음)도 받는다
        self.sim.set_cmd_vel(msg.linear.x, msg.angular.z, msg.linear.y)

    def _tcp_world(self, side):
        d, m = self.sim.d, self.sim.m
        h = m.body(f"openarm_{side}_hand").id
        with self.sim.lock:
            return d.xpos[h] + d.xmat[h].reshape(3, 3) @ [0.0, 0.0, 0.09]

    def _arm_contacts(self):
        """디버그: 팔·손이 로봇 밖의 물체(작업대·상자·벽)와 닿아 있는 접촉 목록."""
        m, d = self.sim.m, self.sim.d
        out = set()
        # 시뮬 스레드가 동시에 mj_step 을 돌리므로 잠금 안에서 접촉 쌍만 복사한다.
        # 잠금 없이 d.contact[i] 를 읽다가 그사이 접촉 수가 줄어 IndexError 로 브리지가 죽었다 (집기 시험 8)
        with self.sim.lock:
            pairs = [(int(d.contact[i].geom1), int(d.contact[i].geom2)) for i in range(d.ncon)]
        for g1, g2 in pairs:
            b1, b2 = m.body(m.geom_bodyid[g1]).name, m.body(m.geom_bodyid[g2]).name
            for a, b in ((b1, b2), (b2, b1)):
                if a.startswith("openarm_") and not b.startswith(("openarm_", "base", "abad", "hip", "knee", "ankle", "foot")):
                    out.add(f"{a}-{b or 'world'}")
        return sorted(out)

    def publish_status(self):
        h, tilt = self.sim.base_height_and_tilt()
        msg = String()
        msg.data = json.dumps({"foot": self.sim.foot, "standing": bool(self.sim.policy.standing),
                               "base_height": round(h, 3), "tilt_deg": round(tilt, 1), "fallen": self.sim.fallen,
                               "cmd": [round(float(x), 3) for x in self.sim.cmd_eff],
                               "est_vel": [round(float(x), 3) for x in self.sim.policy.estimated_velocity],
                               # 디버그·채점용 정답값(시뮬에서만): 손끝(TCP) 세계 위치 = 손 + 손 z축으로 0.09 m (URDF hand_tcp 와 같음)
                               "tcp_world": {s: [round(float(v), 4) for v in self._tcp_world(s)] for s in ("left", "right")},
                               "arm_contacts": self._arm_contacts()})
        self.pub_status.publish(msg)
        if self.sim.fallen:
            self.get_logger().error("로봇이 넘어졌습니다 → ros2 service call /sim/reset std_srvs/srv/Trigger",
                                    throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = BipedBridge()
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
