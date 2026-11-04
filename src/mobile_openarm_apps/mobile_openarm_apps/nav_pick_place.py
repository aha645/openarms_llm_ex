#!/usr/bin/env python3
"""Day2-03 이동 조작: 작업대 A 로 주행 → 집기 → 작업대 B 로 운반 → 놓기 → 복귀.

    ros2 run mobile_openarm_apps nav_pick_place --ros-args -p object:=box_0
"""
from .robot_skills import RobotSkills, SkillError


def main():
    sk = RobotSkills.create("nav_pick_place")
    sk.declare_parameter("object", "box_0")
    sk.declare_parameter("from_station", "pick_station")
    sk.declare_parameter("to_station", "place_station")
    obj = sk.get_parameter("object").value
    a = sk.get_parameter("from_station").value
    b = sk.get_parameter("to_station").value
    try:
        sk.home_arms()                 # 주행 전 팔을 footprint 안으로
        sk.navigate(a)
        side = sk.pick(obj)            # 주행 → 조작 전환: 작업대 충돌객체 추가, 마커 좌표는 base_footprint
        sk.navigate(b)                 # 운반 중 팔은 carry 자세
        sk.place(station=b, side=side)
        sk.navigate("home")
        sk.log("nav_pick_place 완료")
    except SkillError as e:
        sk.get_logger().error(str(e))
    finally:
        sk.shutdown()


if __name__ == "__main__":
    main()
