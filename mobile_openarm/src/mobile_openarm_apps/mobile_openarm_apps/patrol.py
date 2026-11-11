#!/usr/bin/env python3
"""Day1-06 통합 데모: 복수 목적지 순찰 (nav2_simple_commander).

    ros2 run mobile_openarm_apps patrol --ros-args -p loops:=2
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter


def make_pose(nav, x, y, yaw_deg):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
    p.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter("use_sim_time", value=True)])
    nav.declare_parameter("loops", 1)
    nav.declare_parameter("mode", "waypoints")      # waypoints | through
    loops = nav.get_parameter("loops").value
    mode = nav.get_parameter("mode").value
    cfg = yaml.safe_load(open(os.path.join(get_package_share_directory("mobile_openarm_sim"),
                                           "config", "locations.yaml"), encoding="utf-8"))
    route = cfg["patrol_route"]
    nav.waitUntilNav2Active()          # AMCL + Nav2 lifecycle 활성화 대기
    for k in range(loops):
        poses = [make_pose(nav, *cfg["locations"][n]) for n in route]
        nav.get_logger().info(f"순찰 {k+1}/{loops}: {route}")
        if mode == "through":
            nav.goThroughPoses(poses)
        else:
            nav.followWaypoints(poses)
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None and hasattr(fb, "current_waypoint"):
                nav.get_logger().info(f"  waypoint {fb.current_waypoint + 1}/{len(poses)}",
                                      throttle_duration_sec=3.0)
        res = nav.getResult()
        nav.get_logger().info(f"  결과: {'성공' if res == TaskResult.SUCCEEDED else res}")
        if res != TaskResult.SUCCEEDED:
            break
    nav.goToPose(make_pose(nav, *cfg["locations"]["home"]))
    while not nav.isTaskComplete():
        pass
    nav.get_logger().info("순찰 종료 → home 복귀")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
