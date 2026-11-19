#!/usr/bin/env python3
"""TRON1 + OpenArm 용 MuJoCo ↔ ROS 2 브리지.

바퀴형 브리지(mobile_openarm_sim/bridge_node.py)를 그대로 쓰고, 시뮬레이션 코어만 이족용으로 바꾼다.
→ 토픽·액션·서비스 이름(/cmd_vel, /odom, /scan, /joint_states, 팔 FollowJointTrajectory, /aruco/*, /sim/*)이
  바퀴형과 100% 같으므로 SLAM·Nav2·MoveIt·DAY2 앱이 코드 수정 없이 동작한다.

달라진 점
  * MobileOpenArmSim → BipedOpenArmSim (보행 정책·다리 오도메트리)
  * base_link → base_footprint: 고정 높이 대신 추정한 몸통 높이·기울기(IMU + 다리 기구학)로 ArUco 위치를 바꾼다.
    같은 값을 가상 관절(base_height/pitch/roll_joint)로 /joint_states 에 내므로 TF·MoveIt 과 맞는다 (tools/base_tilt_urdf.py)
  * /cmd_vel 의 linear.y(옆걸음)도 받음 — 작업대 앞에서 상자 옆으로 비켜 서기 (robot_skills.strafe_base)
  * /biped/status (std_msgs/String JSON): 몸통 높이·기울기·넘어짐 여부·정책이 받는 명령
"""
import json
import math
import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from mobile_openarm_sim import bridge_node as B

from .biped_core import NOMINAL_BASE_HEIGHT, BipedOpenArmSim

B.MobileOpenArmSim = BipedOpenArmSim
B.BASE_LINK_Z = NOMINAL_BASE_HEIGHT        # 시뮬 생성 뒤 발 형태(pf 0.68 / sf 0.73)에 맞춰 다시 설정


def parse_box_poses(text):
    """"box_0=x,y,yaw;box_1=..." → {name: (x, y, yaw_deg)}. yaw 는 생략 가능 (0)."""
    out = {}
    for item in filter(None, (t.strip() for t in text.split(";"))):
        name, vals = item.split("=")
        v = [float(t) for t in vals.split(",")]
        out[name.strip()] = (v[0], v[1], v[2] if len(v) > 2 else 0.0)
    return out


class BipedBridge(B.MujocoBridge):
    def __init__(self):
        super().__init__()
        B.BASE_LINK_Z = self.sim.nominal_height
        # 카메라 서버용 공유 메모리(state_shm)·상자 마커 숨기기(box_markers)는 바퀴형 브리지에 있다 (공통)
        # 상자 위치 바꾸기 (반복 집기 시험): "box_0=2.60,2.98,15;box_1=2.43,3.00,-10" = 월드 x, y [m], yaw [도]
        self.declare_parameter("box_poses", "")
        self._box_poses = parse_box_poses(self.get_parameter("box_poses").value)
        self._apply_box_poses()
        self.pub_status = self.create_publisher(String, "/biped/status", 10)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info(f"TRON1 보행 정책 사용 (발 형태 {self.sim.foot}, 정책 50 Hz, 물리 {self.sim.m.opt.timestep * 1000:.0f} ms)")

    def _apply_box_poses(self):
        for name, (x, y, yaw) in self._box_poses.items():
            self.sim.set_box_pose(name, x, y, math.radians(yaw))
            self.get_logger().info(f"상자 {name} → ({x:.3f}, {y:.3f}), {yaw:+.0f}°")

    def on_reset(self, req, res):
        res = super().on_reset(req, res)
        self._apply_box_poses()
        return res

    def _base_link_to_footprint(self, p, R):
        # 바퀴형: 높이(BASE_LINK_Z)만 더함 → 이족: 몸통이 숙여진(최대 7°) 만큼 회전까지 (TF 의 가상 관절과 같은 값)
        with self.sim.lock:
            return self.sim.base_link_to_footprint(p, R)

    def _state_tilt(self):
        return self.sim.base_tilt          # 몸통이 숙여진 만큼 (카메라 서버가 base_footprint 로 바꿀 때)

    def on_cmd_vel(self, msg):
        # 바퀴형은 linear.x·angular.z 만 쓴다. 이족은 linear.y(옆걸음)도 받는다
        self.sim.set_cmd_vel(msg.linear.x, msg.angular.z, msg.linear.y)

    def _tcp_world(self, side):
        d, m = self.sim.d, self.sim.m
        h = m.body(f"openarm_{side}_hand").id
        with self.sim.lock:
            return d.xpos[h] + d.xmat[h].reshape(3, 3) @ [0.0, 0.0, 0.09]

    def _arm_contacts(self):
        """디버그: 팔·손이 로봇 밖의 물체(작업대·상자·벽)와 닿아 있는 접촉 목록 (sim_core.arm_contacts, 다리는 로봇 몸)."""
        return self.sim.arm_contacts()

    def publish_status(self):
        h, tilt = self.sim.base_height_and_tilt()
        msg = String()
        msg.data = json.dumps({"foot": self.sim.foot, "standing": bool(self.sim.policy.standing),
                               "base_height": round(h, 3), "tilt_deg": round(tilt, 1), "fallen": self.sim.fallen,
                               "cmd": [round(float(x), 3) for x in self.sim.cmd_eff],
                               "est_vel": [round(float(x), 3) for x in self.sim.policy.estimated_velocity],
                               # 디버그·채점용 정답값(시뮬에서만): 손끝(TCP) 세계 위치 = 손 + 손 z축으로 0.09 m (URDF hand_tcp 와 같음)
                               "tcp_world": {s: [round(float(v), 4) for v in self._tcp_world(s)] for s in ("left", "right")},
                               "arm_contacts": self._arm_contacts(),
                               # 디버그: 몸통 정답 자세(map=world 원점 기준)와 다리 오도메트리 (제자리 흐름 분석용)
                               "gt_pose": [round(float(v), 4) for v in self.sim.ground_truth_pose()],
                               "odom": [round(float(v), 4) for v in self.sim.odom]})
        self.pub_status.publish(msg)
        if self.sim.fallen:
            self.get_logger().error("로봇이 넘어졌습니다 → ros2 service call /sim/reset std_srvs/srv/Trigger",
                                    throttle_duration_sec=5.0)


def main():
    # GIL 전환 간격 5 ms → 0.5 ms. 물리 스레드는 스텝마다 짧은 파이썬 연산(보행 정책·제어기·중력 보상)을 수백 번 하고,
    # 그때마다 GIL 을 다시 받아야 한다. /cmd_vel 이 들어오는 동안 rclpy MultiThreadedExecutor 가 대기 집합을 파이썬으로
    # 다시 만들며 GIL 을 58~76 % 쥐어, 물리 스레드가 매번 최대 5 ms 씩 기다려 걷는 동안 실시간의 0.1~0.4 배로 느려지고
    # 몇 초씩 멈췄다 (Nav2 lifecycle heartbeat 끊김 → 주행 중단). 0.5 ms 로 줄이면 걷는 동안에도 실시간 1.00 (검증 기록 11절)
    sys.setswitchinterval(0.0005)
    rclpy.init()
    node = BipedBridge()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_event.set()
        if node._state_writer:
            node._state_writer.close()
        ex.shutdown(wait_for_threads=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
