"""Day1-02/03: MuJoCo 시뮬레이터 + ROS 2 브리지 + robot_state_publisher (+ RViz2).

    ros2 launch mobile_openarm_sim sim.launch.py                 # 뷰어 + RViz
    ros2 launch mobile_openarm_sim sim.launch.py viewer:=false   # 헤드리스 (가벼움)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mobile_openarm_sim")
    urdf = os.path.join(share, "urdf", "mobile_openarm.urdf")
    with open(urdf, encoding="utf-8") as f:
        robot_description = f.read()

    args = [
        DeclareLaunchArgument("viewer", default_value="true", description="MuJoCo 뷰어 창"),
        DeclareLaunchArgument("world", default_value="world.xml",
                              description="world_light.xml = 저사양 모드(시각 메시 제거, 메모리 1.9GB→0.2GB)"),
        DeclareLaunchArgument("camera", default_value="true", description="머리 카메라 렌더 + ArUco"),
        DeclareLaunchArgument("publish_image", default_value="true", description="압축 이미지 발행(대시보드용)"),
        DeclareLaunchArgument("move_person", default_value="true", description="움직이는 사람 장애물"),
        DeclareLaunchArgument("grasp_assist", default_value="true", description="파지 보조(교육용)"),
        DeclareLaunchArgument("rate_factor", default_value="1.0", description="1.0=실시간, 느린 PC 는 0.5"),
        DeclareLaunchArgument("lidar_rays", default_value="360"),
        DeclareLaunchArgument("camera_hz", default_value="10.0", description="ArUco 카메라 렌더 주기 (GPU 없는 환경은 5.0 권장)"),
        DeclareLaunchArgument("image_hz", default_value="5.0", description="대시보드용 압축 이미지 발행 주기"),
        DeclareLaunchArgument("start_x", default_value="0.0", description="시작 위치(월드). Nav2 와 같이 쓸 땐 0 유지"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0", description="rad"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(share, "rviz", "sim.rviz")),
    ]
    lc = LaunchConfiguration
    bridge = Node(
        package="mobile_openarm_sim", executable="mujoco_bridge", name="mujoco_bridge", output="screen",
        parameters=[{
            "mjcf": PathJoinSubstitution([share, "mjcf", lc("world")]),
            "viewer": lc("viewer"), "camera": lc("camera"), "publish_image": lc("publish_image"),
            "move_person": lc("move_person"), "grasp_assist": lc("grasp_assist"),
            "rate_factor": lc("rate_factor"), "lidar_rays": lc("lidar_rays"),
            "camera_hz": lc("camera_hz"), "image_hz": lc("image_hz"),
            "start_x": lc("start_x"), "start_y": lc("start_y"), "start_yaw": lc("start_yaw"),
        }],
    )
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", arguments=["-d", lc("rviz_config")],
        parameters=[{"use_sim_time": True}], condition=IfCondition(lc("rviz")), output="log",
    )
    return LaunchDescription(args + [bridge, rsp, rviz])
