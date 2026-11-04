#!/usr/bin/env python3
"""Day2-01 MoveIt 2 양팔 제어 실습.

  1) 이름 자세(SRDF group_state): home → ready → hands_up (양팔 동시: both_arms 그룹)
  2) 관절 목표(joint goal): 오른팔만
  3) 말단 자세 목표(pose goal): TCP 를 몸 앞 좌표로 (IK → 관절 목표)
  4) 직선(Cartesian) 경로: TCP 를 10cm 위/아래로

    ros2 run mobile_openarm_apps arm_demo
"""
import numpy as np

from .geometry import approach_rotation
from .robot_skills import RobotSkills, SkillError


def main():
    sk = RobotSkills.create("arm_demo")
    try:
        for name in ("ready", "hands_up", "home"):
            sk.move_named("both_arms", name)
        sk.log("관절 목표: 오른팔 joint4 = 1.2 rad")
        q = dict(sk.named_states[("right_arm", "ready")])
        q["openarm_right_joint4"] = 1.2
        sk.move_joints("right_arm", q)
        for side, y in (("right", -0.15), ("left", 0.15)):
            R = approach_rotation(30)                       # 30도 숙인 접근 자세
            p = np.array([0.50, y, 0.95])
            sk.log(f"pose goal: {side} TCP → {p}")
            sk.move_pose(side, p, R)
            sk.log("직선 이동: 아래로 10cm → 다시 위로")
            sk.cartesian(side, [p - [0, 0, 0.10]], R)
            sk.cartesian(side, [p], R)
            sk.move_named(f"{side}_arm", "home")
        sk.open_gripper("right")
        sk.close_gripper("right")
        sk.log("arm_demo 완료")
    except SkillError as e:
        sk.get_logger().error(str(e))
    finally:
        sk.shutdown()


if __name__ == "__main__":
    main()
