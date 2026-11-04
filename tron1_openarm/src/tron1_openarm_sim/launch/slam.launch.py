"""[TRON1 이족] Day1-04: SLAM(slam_toolbox) 으로 지도 작성.

    ros2 launch tron1_openarm_sim slam.launch.py
    # 다른 터미널: ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # 저장:       ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("tron1_openarm_sim")
    slam_share = get_package_share_directory("slam_toolbox")
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("policy", default_value="", description="비우면 발 형태별 기본 정책"),
        DeclareLaunchArgument("foot", default_value="pf", description="pf = Point-foot, sf = Sole-foot"),
        DeclareLaunchArgument("world", default_value="world.xml"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("slam_params", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "sim.launch.py")),
            launch_arguments={"foot": lc("foot"), "policy": lc("policy"), "viewer": lc("viewer"), "world": lc("world"), "rviz": lc("rviz"), "rviz_config": os.path.join(share, "rviz", "nav.rviz"),
                              "camera": "false", "move_person": "false"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": lc("slam_params"), "use_sim_time": "true"}.items()),
    ])
