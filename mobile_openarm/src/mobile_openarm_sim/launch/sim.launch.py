"""Day1-02/03: MuJoCo 시뮬레이터 + ROS 2 브리지 + robot_state_publisher (+ RViz2).

    ros2 launch mobile_openarm_sim sim.launch.py                 # 뷰어 + RViz
    ros2 launch mobile_openarm_sim sim.launch.py viewer:=false   # 헤드리스 (가벼움)
    ros2 launch mobile_openarm_sim sim.launch.py camera_process:=false   # 예전 방식: 브리지의 물리 스레드가 머리 카메라만 렌더링
    ros2 launch mobile_openarm_sim sim.launch.py box_source:=color       # 상자 마커 없이 색(빨강·초록·파랑)+깊이로 인식

카메라 (camera_process:=true, 기본): 머리 RGB-D 카메라와 양 손목 카메라를 별도 프로세스(sim_cameras = camera_server.py)가
렌더링·검출·JPEG 압축해 발행한다 (TRON1 과 같은 코드). 손목 카메라 검출(/wrist_*_camera/aruco/detections)이 들어오면
집기 앱이 손끝 기준 상자 위치로 손을 맞추며(visual_servo) 잡는다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SHM = "mobile_sim_state"


def _setup(context):
    """브리지 (+ camera_process 면 카메라 서버). 카메라 서버가 있으면 브리지는 자기 카메라를 끄고 렌더링에 필요한 상태만
    공유 메모리로 넘긴다 (TRON1 sim.launch.py 와 같은 방식)."""
    lc = LaunchConfiguration
    on = lambda n: lc(n).perform(context).lower() in ("true", "1")  # noqa: E731
    share = get_package_share_directory("mobile_openarm_sim")
    mjcf = os.path.join(share, "mjcf", lc("world").perform(context))
    cam = on("camera") and on("camera_process")
    nodes = [Node(
        package="mobile_openarm_sim", executable="mujoco_bridge", name="mujoco_bridge", output="screen",
        parameters=[{
            "mjcf": mjcf,
            "viewer": lc("viewer"), "publish_image": lc("publish_image"),
            "camera": on("camera") and not cam, "state_shm": SHM if cam else "",
            "box_markers": lc("box_source").perform(context) != "color",
            "move_person": lc("move_person"), "grasp_assist": lc("grasp_assist"),
            "rate_factor": lc("rate_factor"), "lidar_rays": lc("lidar_rays"),
            "camera_hz": lc("camera_hz"), "image_hz": lc("image_hz"),
            "start_x": lc("start_x"), "start_y": lc("start_y"), "start_yaw": lc("start_yaw"),
        }])]
    if cam:
        nodes.append(Node(package="mobile_openarm_sim", executable="sim_cameras", name="sim_cameras", output="screen",
                          parameters=[{"mjcf": mjcf, "state_shm": SHM,
                                       "head_width": 640, "head_height": 480,      # 예전 브리지 머리 카메라와 같은 해상도
                                       "head_hz": lc("camera_hz"), "wrist_hz": lc("camera_hz"), "image_hz": lc("image_hz"),
                                       "use_sim_time": True, "box_source": lc("box_source")}]))
    return nodes


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
        DeclareLaunchArgument("camera_process", default_value="true",
                              description="true = 머리 RGB-D·양 손목 카메라를 별도 프로세스(sim_cameras)가 렌더링·검출·JPEG 압축 발행"),
        DeclareLaunchArgument("box_source", default_value="aruco",
                              description="상자 인식: aruco = 윗면 마커, color = 마커 없이 색(빨강·초록·파랑)+깊이 (camera_process 필요)"),
    ]
    lc = LaunchConfiguration
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", arguments=["-d", lc("rviz_config")],
        parameters=[{"use_sim_time": True}], condition=IfCondition(lc("rviz")), output="log",
    )
    return LaunchDescription(args + [rsp, rviz, OpaqueFunction(function=_setup)])
