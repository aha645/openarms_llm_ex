"""Day2-06 최종 통합: 시뮬 + Nav2 + MoveIt 2 + Task Manager + 웹 대시보드.

    ros2 launch mobile_openarm_apps full_system.launch.py planner:=offline   # API 키 없이
    ros2 launch mobile_openarm_apps full_system.launch.py planner:=llm       # OPENAI_API_KEY 필요
    → 브라우저 http://localhost:8080
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory("mobile_openarm_sim")
    mv = get_package_share_directory("mobile_openarm_moveit_config")
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("planner", default_value="offline"),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("world", default_value="world.xml", description="world_light.xml = 저사양 모드"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim, "launch", "nav.launch.py")),
                                 launch_arguments={"viewer": lc("viewer"), "rviz": lc("rviz"), "world": lc("world")}.items()),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mv, "launch", "move_group.launch.py")),
            launch_arguments={"rviz": "false"}.items())]),
        TimerAction(period=12.0, actions=[
            Node(package="mobile_openarm_apps", executable="task_manager", output="screen",
                 parameters=[{"planner": lc("planner")}]),
            Node(package="mobile_openarm_apps", executable="dashboard", output="screen"),
        ]),
    ])
