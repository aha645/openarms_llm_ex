#!/usr/bin/env python3
"""TRON1 + OpenArm 용 MuJoCo ↔ ROS 2 브리지.

바퀴형 브리지(mobile_openarm_sim/bridge_node.py)를 그대로 쓰고, 시뮬레이션 코어만 이족용으로 바꾼다.
→ 토픽·액션·서비스 이름(/cmd_vel, /odom, /scan, /joint_states, 팔 FollowJointTrajectory, /aruco/*, /sim/*)이
  바퀴형과 100% 같으므로 SLAM·Nav2·MoveIt·DAY2 앱이 코드 수정 없이 동작한다.

달라진 점
  * MobileOpenArmSim → BipedOpenArmSim (보행 정책·다리 오도메트리)
  * BASE_LINK_Z: 0.08(바퀴 반지름) → 0.68(TRON1 서 있는 높이). ArUco 위치를 base_footprint 기준으로 바꿀 때 사용
  * /biped/status (std_msgs/String JSON): 몸통 높이·기울기·넘어짐 여부·정책이 받는 명령
"""
import json

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from mobile_openarm_sim import bridge_node as B

from .biped_core import NOMINAL_BASE_HEIGHT, BipedOpenArmSim

B.MobileOpenArmSim = BipedOpenArmSim
B.BASE_LINK_Z = NOMINAL_BASE_HEIGHT


class BipedBridge(B.MujocoBridge):
    def __init__(self):
        super().__init__()
        self.pub_status = self.create_publisher(String, "/biped/status", 10)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(f"TRON1 보행 정책 사용 (정책 50 Hz, 물리 {self.sim.m.opt.timestep * 1000:.0f} ms)")

    def publish_status(self):
        h, tilt = self.sim.base_height_and_tilt()
        msg = String()
        msg.data = json.dumps({"base_height": round(h, 3), "tilt_deg": round(tilt, 1), "fallen": self.sim.fallen,
                               "cmd": [round(float(x), 3) for x in self.sim.cmd_eff],
                               "est_vel": [round(float(x), 3) for x in self.sim.policy.estimated_velocity]})
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
