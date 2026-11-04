"""[TRON1 이족] Day1-04/05/06: 저장된 지도 + AMCL + Nav2 자율주행.

    ros2 launch tron1_openarm_sim nav.launch.py                               # 기본: 정답 지도(maps/world_map.yaml)
    ros2 launch tron1_openarm_sim nav.launch.py map:=$HOME/maps/my_map.yaml   # 직접 만든 SLAM 지도
    ros2 launch tron1_openarm_sim nav.launch.py slam:=True                    # 지도 없이 SLAM+Nav2 동시
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("tron1_openarm_sim")
    nav2 = get_package_share_directory("nav2_bringup")
    lc = LaunchConfiguration
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "sim.launch.py")),
        launch_arguments={"foot": lc("foot"), "policy": lc("policy"), "viewer": lc("viewer"), "world": lc("world"), "camera": lc("camera"), "move_person": lc("move_person"),
                          "rviz_config": os.path.join(share, "rviz", "nav.rviz"), "rviz": lc("rviz")}.items())
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "bringup_launch.py")),
        launch_arguments={"map": lc("map"), "params_file": lc("params_file"), "use_sim_time": "True",
                          "slam": lc("slam"), "autostart": "True", "use_composition": "True"}.items())
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("policy", default_value="", description="비우면 발 형태별 기본 정책"),
        DeclareLaunchArgument("foot", default_value="pf", description="pf = Point-foot, sf = Sole-foot"),
        DeclareLaunchArgument("world", default_value="world.xml"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("move_person", default_value="true"),
        DeclareLaunchArgument("slam", default_value="False"),
        DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "world_map.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "nav2_params_tron1.yaml")),
        sim,
        TimerAction(period=3.0, actions=[nav]),   # /clock·TF 가 먼저 나오도록 잠깐 대기
    ])
