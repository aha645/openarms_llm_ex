"""Day2-01: MoveIt 2 move_group (+ RViz MotionPlanning).

    ros2 launch mobile_openarm_moveit_config move_group.launch.py            # sim.launch.py 가 먼저 실행돼 있어야 함
    ros2 launch mobile_openarm_moveit_config move_group.launch.py rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    sim_share = get_package_share_directory("mobile_openarm_sim")
    urdf = os.path.join(sim_share, "urdf", "mobile_openarm.urdf")
    mc = (
        MoveItConfigsBuilder("mobile_openarm", package_name="mobile_openarm_moveit_config")
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path="config/mobile_openarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[mc.to_dict(), {"use_sim_time": True},
                    {"publish_planning_scene": True, "publish_geometry_updates": True,
                     "publish_state_updates": True, "publish_transforms_updates": True}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", output="log",
        arguments=["-d", os.path.join(get_package_share_directory("mobile_openarm_moveit_config"),
                                      "config", "moveit.rviz")],
        parameters=[mc.robot_description, mc.robot_description_semantic, mc.robot_description_kinematics,
                    mc.planning_pipelines, mc.joint_limits, {"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription([DeclareLaunchArgument("rviz", default_value="true"), move_group, rviz])
