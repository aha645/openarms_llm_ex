#!/usr/bin/env python3
"""Day2-02 ArUco 기반 Pick & Place (로봇이 작업대 앞에 있다고 가정).

    ros2 run mobile_openarm_apps pick_place --ros-args -p object:=box_0
  - 상자를 집어서 같은 작업대의 다른 위치(로봇 정면 중앙)에 내려놓는다.
"""

import math
import time

from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from .geometry import yaw_to_quat
from .robot_skills import RobotSkills, SkillError


def ensure_map_frame(sk, x, y, yaw):
    """Nav2(AMCL) 없이 실행할 때: 시뮬 시작 위치를 map→odom 정적 TF 로 알려준다."""
    time.sleep(1.0)
    if sk.robot_pose("map") is not None:
        return None
    br = StaticTransformBroadcaster(sk)
    t = TransformStamped()
    t.header.stamp = sk.get_clock().now().to_msg()
    t.header.frame_id, t.child_frame_id = "map", "odom"
    t.transform.translation.x, t.transform.translation.y = float(x), float(y)
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = yaw_to_quat(yaw)
    br.sendTransform(t)
    sk.log(f"map 프레임 없음 → map→odom 정적 TF 발행 (시작 위치 {x}, {y}, {math.degrees(yaw):.0f}°)")
    time.sleep(0.5)
    return br


def main():
    sk = RobotSkills.create("pick_place")
    sk.declare_parameter("object", "box_0")
    sk.declare_parameter("station", "pick_station")
    sk.declare_parameter("start_x", 2.5)        # sim.launch.py 의 start_x/y/yaw 와 같게
    sk.declare_parameter("start_y", 2.45)
    sk.declare_parameter("start_yaw", 1.5708)
    obj = sk.get_parameter("object").value
    sk.current_station = sk.get_parameter("station").value
    g = lambda n: sk.get_parameter(n).value  # noqa: E731
    _br = ensure_map_frame(sk, g("start_x"), g("start_y"), g("start_yaw"))  # noqa: F841 (TF 유지용 참조)
    try:
        sk.home_arms()
        dets = sk.detections(max_age=2.0)
        sk.log(f"보이는 마커: {sorted(dets)}")
        side = sk.pick(obj)
        sk.place(side=side, offset=(0.48, -0.04))   # 같은 작업대, 로봇 정면(초록 상자 옆)에 내려놓기
        sk.log("pick_place 완료")
    except SkillError as e:
        sk.get_logger().error(str(e))
    finally:
        sk.shutdown()


if __name__ == "__main__":
    main()
