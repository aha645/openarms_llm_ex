"""[TRON1 이족] Day1-04/05/06: 저장된 지도 + AMCL + Nav2 자율주행.

    ros2 launch tron1_openarm_sim nav.launch.py                               # 기본: 정답 지도(maps/world_map.yaml)
    ros2 launch tron1_openarm_sim nav.launch.py map:=$HOME/maps/my_map.yaml   # 직접 만든 SLAM 지도
    ros2 launch tron1_openarm_sim nav.launch.py slam:=True                    # 지도 없이 SLAM+Nav2 동시
    ros2 launch tron1_openarm_sim nav.launch.py start_x:=-3.5 start_y:=-2.8 start_yaw:=1.5708   # 다른 곳에서 출발 (지도 좌표)
start_x/y/yaw: 시뮬레이터의 로봇 시작 위치와 AMCL 초기 위치를 같이 바꾼다 (기본 0 = 지도 원점, 예전과 같음).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import math
import tempfile

import yaml
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _nav(context, share, nav2, lc):
    """AMCL 초기 위치를 시작 위치(start_x/y/yaw)로 바꾼 Nav2 설정으로 bringup. 0 이면 설정 파일 그대로."""
    params = lc("params_file").perform(context)
    x, y, yaw = (float(lc(k).perform(context)) for k in ("start_x", "start_y", "start_yaw"))
    if abs(x) > 1e-6 or abs(y) > 1e-6 or abs(yaw) > 1e-6:
        p = yaml.safe_load(open(params, encoding="utf-8"))
        a = p["amcl"]["ros__parameters"]
        a["set_initial_pose"] = True
        a["initial_pose"] = {"x": x, "y": y, "z": 0.0, "yaw": math.atan2(math.sin(yaw), math.cos(yaw))}
        f = tempfile.NamedTemporaryFile("w", suffix="_nav2_params_tron1.yaml", delete=False, encoding="utf-8")
        yaml.safe_dump(p, f, sort_keys=False)
        f.close()
        params = f.name
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "bringup_launch.py")),
        launch_arguments={"map": lc("map"), "params_file": params, "use_sim_time": "True",
                          "slam": lc("slam"), "autostart": "True", "use_composition": "True"}.items())]


def generate_launch_description():
    share = get_package_share_directory("tron1_openarm_sim")
    nav2 = get_package_share_directory("nav2_bringup")
    lc = LaunchConfiguration
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "sim.launch.py")),
        launch_arguments={"foot": lc("foot"), "policy": lc("policy"), "viewer": lc("viewer"), "world": lc("world"), "camera": lc("camera"), "move_person": lc("move_person"), "box_source": lc("box_source"),
                          "rviz_config": os.path.join(share, "rviz", "nav.rviz"), "rviz": lc("rviz"),
                          "camera_hz": lc("camera_hz"), "lidar_rays": lc("lidar_rays"),
                          "rate_factor": lc("rate_factor"), "ekf": lc("ekf"),
                          "start_x": lc("start_x"), "start_y": lc("start_y"), "start_yaw": lc("start_yaw")}.items())
    nav = OpaqueFunction(function=_nav, args=[share, nav2, lc])
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("policy", default_value="", description="비우면 발 형태별 기본 정책"),
        DeclareLaunchArgument("foot", default_value="sf", description="sf = Sole-foot(교육 기준), pf = Point-foot"),
        DeclareLaunchArgument("world", default_value="world.xml"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("box_source", default_value="color", description="상자 인식: aruco | color (sim.launch.py)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("move_person", default_value="true"),
        DeclareLaunchArgument("camera_hz", default_value="10.0", description="GPU 없는 환경은 5.0"),
        DeclareLaunchArgument("lidar_rays", default_value="360"),
        DeclareLaunchArgument("rate_factor", default_value="1.0", description="1.0=실시간, 느린 PC 는 0.5"),
        DeclareLaunchArgument("ekf", default_value="false", description="true = EKF(다리 오도메트리 + IMU)가 odom TF 를 냄"),
        DeclareLaunchArgument("slam", default_value="False"),
        DeclareLaunchArgument("start_x", default_value="0.0", description="시작 위치 (지도 좌표) — 시뮬레이터·AMCL 초기 위치"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0", description="[rad]"),
        DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "world_map.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "nav2_params_tron1.yaml")),
        sim,
        TimerAction(period=3.0, actions=[nav]),   # /clock·TF 가 먼저 나오도록 잠깐 대기
    ])
