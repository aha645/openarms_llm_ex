"""[TRON1 이족] Day2-06 최종 통합: 시뮬(보행 정책) + Nav2 + MoveIt 2 + Task Manager + 웹 대시보드.
    ros2 launch tron1_openarm_sim full_system.launch.py planner:=offline
    → 브라우저 http://localhost:8080

바퀴형 mobile_openarm_apps/full_system.launch.py 와 같은 구성. 하반신 관련 두 줄만 다르다:
  nav.launch.py      : mobile_openarm_sim → tron1_openarm_sim (보행 브리지, 이족용 Nav2 파라미터)
  move_group.launch  : mobile_openarm_moveit_config → tron1_openarm_moveit_config (URDF 에 다리 포함)
Task Manager·대시보드·LLM 계획기·robot_skills 는 mobile_openarm_apps 의 것을 코드 수정 없이 그대로 쓴다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory("tron1_openarm_sim")
    mv = get_package_share_directory("tron1_openarm_moveit_config")
    lc = LaunchConfiguration
    return LaunchDescription([
        # robot_skills(DAY2 앱)가 이족용 이름 자세(팔을 벌린 home)를 쓰도록
        SetEnvironmentVariable("OPENARM_SRDF", os.path.join(mv, "config", "tron1_openarm.srdf")),
        SetEnvironmentVariable("OPENARM_BIPED", "1"),      # robot_skills: 옆걸음(strafe) 사용
        DeclareLaunchArgument("planner", default_value="offline"),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("world", default_value="world.xml"),
        DeclareLaunchArgument("policy", default_value=os.path.join(sim, "policies", "tron1_openarm_policy.npz")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim, "launch", "nav.launch.py")),
                                 launch_arguments={"viewer": lc("viewer"), "rviz": lc("rviz"), "world": lc("world"),
                                                   "policy": lc("policy")}.items()),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mv, "launch", "move_group.launch.py")),
            launch_arguments={"rviz": "false"}.items())]),
        TimerAction(period=12.0, actions=[
            Node(package="mobile_openarm_apps", executable="task_manager", output="screen",
                 parameters=[{"planner": lc("planner")}]),
            Node(package="mobile_openarm_apps", executable="dashboard", output="screen"),
        ]),
    ])
