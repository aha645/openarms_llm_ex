"""TRON1(Point-foot) + OpenArm: MuJoCo 시뮬레이터 + ROS 2 브리지 + robot_state_publisher (+ RViz2).
    ros2 launch tron1_openarm_sim sim.launch.py
    ros2 launch tron1_openarm_sim sim.launch.py world:=world_light.xml viewer:=false
바퀴형 mobile_openarm_sim/sim.launch.py 와 인자·토픽이 같다. 다른 점: 브리지 실행파일(tron1_bridge), URDF, 보행 정책 파일(policy).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("tron1_openarm_sim")
    with open(os.path.join(share, "urdf", "tron1_openarm.urdf"), encoding="utf-8") as f:
        robot_description = f.read()
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("world", default_value="world.xml", description="world_light.xml = 저사양"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("publish_image", default_value="true"),
        DeclareLaunchArgument("move_person", default_value="true"),
        DeclareLaunchArgument("grasp_assist", default_value="true"),
        DeclareLaunchArgument("rate_factor", default_value="1.0"),
        DeclareLaunchArgument("lidar_rays", default_value="360"),
        DeclareLaunchArgument("camera_hz", default_value="10.0"),
        DeclareLaunchArgument("image_hz", default_value="5.0"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(share, "rviz", "sim.rviz")),
        DeclareLaunchArgument("policy", default_value=os.path.join(share, "policies", "tron1_openarm_policy.npz"),
                              description="보행 정책 가중치(npz). LimX 사전학습: policies/limx_pf_tron1a_pretrained.npz"),
    ]
    bridge = Node(
        package="tron1_openarm_sim", executable="tron1_bridge", name="mujoco_bridge", output="screen",
        parameters=[{
            "mjcf": PathJoinSubstitution([share, "mjcf", lc("world")]),
            "viewer": lc("viewer"), "camera": lc("camera"), "publish_image": lc("publish_image"),
            "move_person": lc("move_person"), "grasp_assist": lc("grasp_assist"),
            "rate_factor": lc("rate_factor"), "lidar_rays": lc("lidar_rays"),
            "camera_hz": lc("camera_hz"), "image_hz": lc("image_hz"),
            # 머리 카메라가 바퀴형보다 8 cm 높아(1.24 m) 4 cm 마커가 640x480 에서 너무 작게 보임 → 960x720
            "camera_width": 960, "camera_height": 720,
            "start_x": lc("start_x"), "start_y": lc("start_y"), "start_yaw": lc("start_yaw"),
        }],
    )
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}])
    rviz = Node(package="rviz2", executable="rviz2", arguments=["-d", lc("rviz_config")],
                parameters=[{"use_sim_time": True}], condition=IfCondition(lc("rviz")), output="log")
    return LaunchDescription(args + [SetEnvironmentVariable("TRON1_POLICY", lc("policy")), bridge, rsp, rviz])
