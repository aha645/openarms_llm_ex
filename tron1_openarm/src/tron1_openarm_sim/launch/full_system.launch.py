"""[TRON1 이족] Day2-06 최종 통합: 시뮬(보행 정책) + Nav2 + MoveIt 2 + Task Manager + 웹 대시보드.
    ros2 launch tron1_openarm_sim full_system.launch.py planner:=offline        # Sole-foot (기본, 교육 기준)
    → 브라우저 http://localhost:8080
    ... foot:=pf                                    # Point-foot 비교
    ... camera_hz:=5.0 lidar_rays:=180 rate_factor:=0.5   # 느린 PC / GPU 없는 Docker
    ... dashboard_host:=0.0.0.0                     # 같은 네트워크의 휴대폰에서도 접속 (인증 없음 주의)

구성: nav.launch.py(시뮬 + 보행 브리지 + 카메라 서버 + Nav2) → 5 s 뒤 move_group.launch.py(MoveIt, URDF 에 다리 포함)
→ 12 s 뒤 tron1_openarm_apps 의 task_manager(LLM 계획기 + robot_skills)·dashboard.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _check_foot(context):
    foot = LaunchConfiguration("foot").perform(context)
    if foot not in ("pf", "sf"):
        raise RuntimeError(f"foot:={foot!r} 은(는) 잘못된 값입니다. foot 는 pf(Point-foot) 또는 sf(Sole-foot, 교육 기준) 입니다")
    return []


def generate_launch_description():
    sim = get_package_share_directory("tron1_openarm_sim")
    mv = get_package_share_directory("tron1_openarm_moveit_config")
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("foot", default_value="sf", description="sf = Sole-foot(발목 있음, 교육 기준), pf = Point-foot"),
        OpaqueFunction(function=_check_foot),
        DeclareLaunchArgument("planner", default_value="offline"),
        # 계획 최대 단계 수 (llm_planner.MAX_STEPS 기본 20 = 상자 3 개 운반 13 단계 + 순찰)
        DeclareLaunchArgument("max_steps", default_value="20"),
        SetEnvironmentVariable("OPENARM_MAX_STEPS", LaunchConfiguration("max_steps")),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("world", default_value="world.xml"),
        DeclareLaunchArgument("policy", default_value="", description="비우면 발 형태별 기본 정책"),
        DeclareLaunchArgument("camera_hz", default_value="10.0", description="GPU 없는 환경은 5.0"),
        DeclareLaunchArgument("lidar_rays", default_value="360"),
        DeclareLaunchArgument("box_source", default_value="color", description="상자 인식: aruco | color (sim.launch.py)"),
        DeclareLaunchArgument("start_x", default_value="0.0", description="시작 위치 (지도 좌표, nav.launch.py)"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0", description="[rad]"),
        DeclareLaunchArgument("rate_factor", default_value="1.0", description="1.0=실시간, 느린 PC 는 0.5"),
        DeclareLaunchArgument("ekf", default_value="false", description="true = EKF(다리 오도메트리 + IMU)가 odom TF 를 냄"),
        DeclareLaunchArgument("dashboard_host",
                              default_value=EnvironmentVariable("OPENARM_DASHBOARD_HOST", default_value="127.0.0.1"),
                              description="127.0.0.1=이 PC 에서만, 0.0.0.0=같은 네트워크 전체(인증 없음)"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim, "launch", "nav.launch.py")),
                                 launch_arguments={"foot": lc("foot"), "viewer": lc("viewer"), "rviz": lc("rviz"),
                                                   "world": lc("world"), "policy": lc("policy"),
                                                   "camera_hz": lc("camera_hz"), "lidar_rays": lc("lidar_rays"),
                                                   "rate_factor": lc("rate_factor"), "ekf": lc("ekf"),
                                                   "box_source": lc("box_source"), "start_x": lc("start_x"),
                                                   "start_y": lc("start_y"), "start_yaw": lc("start_yaw")}.items()),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mv, "launch", "move_group.launch.py")),
            launch_arguments={"rviz": "false", "foot": lc("foot")}.items())]),
        TimerAction(period=12.0, actions=[
            Node(package="tron1_openarm_apps", executable="task_manager", output="screen",
                 parameters=[{"planner": lc("planner")}]),
            Node(package="tron1_openarm_apps", executable="dashboard", output="screen",
                 parameters=[{"host": lc("dashboard_host")}]),
        ]),
    ])
