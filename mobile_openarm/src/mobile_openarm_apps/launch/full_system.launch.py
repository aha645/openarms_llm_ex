"""Day2-06 최종 통합: 시뮬 + Nav2 + MoveIt 2 + Task Manager + 웹 대시보드.

    ros2 launch mobile_openarm_apps full_system.launch.py planner:=offline   # API 키 없이
    ros2 launch mobile_openarm_apps full_system.launch.py planner:=llm       # OPENAI_API_KEY 필요
    → 브라우저 http://localhost:8080
    ... dashboard_host:=0.0.0.0      # 같은 네트워크의 휴대폰에서도 접속 (인증 없음 주의)
    ... camera_hz:=5.0 lidar_rays:=180 rate_factor:=0.5   # 느린 PC / GPU 없는 Docker
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
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
        DeclareLaunchArgument("camera_hz", default_value="10.0", description="GPU 없는 환경은 5.0"),
        DeclareLaunchArgument("lidar_rays", default_value="360"),
        DeclareLaunchArgument("rate_factor", default_value="1.0", description="1.0=실시간, 느린 PC 는 0.5"),
        DeclareLaunchArgument("camera_process", default_value="true",
                              description="true = 머리 RGB-D·양 손목 카메라를 별도 프로세스로 (손목 카메라로 상자를 보며 잡기)"),
        DeclareLaunchArgument("box_source", default_value="color", description="aruco = 윗면 마커, color = 마커 없이 색+깊이"),
        DeclareLaunchArgument("dashboard_host",
                              default_value=EnvironmentVariable("OPENARM_DASHBOARD_HOST", default_value="127.0.0.1"),
                              description="127.0.0.1=이 PC 에서만, 0.0.0.0=같은 네트워크 전체(인증 없음)"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim, "launch", "nav.launch.py")),
                                 launch_arguments={"viewer": lc("viewer"), "rviz": lc("rviz"), "world": lc("world"),
                                                   "camera_hz": lc("camera_hz"), "lidar_rays": lc("lidar_rays"),
                                                   "rate_factor": lc("rate_factor"), "camera_process": lc("camera_process"),
                                                   "box_source": lc("box_source")}.items()),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mv, "launch", "move_group.launch.py")),
            launch_arguments={"rviz": "false"}.items())]),
        TimerAction(period=12.0, actions=[
            Node(package="mobile_openarm_apps", executable="task_manager", output="screen",
                 parameters=[{"planner": lc("planner")}]),
            Node(package="mobile_openarm_apps", executable="dashboard", output="screen",
                 parameters=[{"host": lc("dashboard_host")}]),
        ]),
    ])
