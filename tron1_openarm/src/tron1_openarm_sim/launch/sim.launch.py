"""TRON1 + OpenArm: MuJoCo 시뮬레이터 + ROS 2 브리지 + robot_state_publisher (+ RViz2).
    ros2 launch tron1_openarm_sim sim.launch.py                    # Sole-foot (기본, 교육 기준. 발목 있음, 제자리에 가만히 섬)
    ros2 launch tron1_openarm_sim sim.launch.py foot:=pf           # Point-foot (PF 정책·기록 비교용)
    ros2 launch tron1_openarm_sim sim.launch.py world:=world_light.xml viewer:=false
    ros2 launch tron1_openarm_sim sim.launch.py camera_process:=false  # 예전 방식: 브리지의 물리 스레드가 머리 카메라만 렌더링
    ros2 launch tron1_openarm_sim sim.launch.py ekf:=true          # odom→base_footprint 를 EKF(다리 오도메트리 + IMU, config/ekf_tron1.yaml)가 냄
  policy:= 로 정책을 바꿀 때는 그 정책의 발 형태와 foot 를 맞춘다 (다르면 브리지가 오류로 멈춤)
브리지(tron1_bridge, 보행 정책 policy)·카메라 서버(tron1_cameras)·robot_state_publisher(URDF)·RViz 를 띄운다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


FOOT_FILES = {   # 발 형태 → (mjcf 하위 폴더, URDF, 기본 정책)
    "pf": ("", "tron1_openarm.urdf", "tron1_openarm_policy.npz"),
    "sf": ("sf", "tron1_openarm_sf.urdf", "tron1sf_openarm_policy.npz"),
}


def check_foot(foot):
    """foot 인자는 pf / sf 만 허용 (대소문자 구분). 틀리면 알아보기 쉬운 오류로 중단."""
    if foot not in ("pf", "sf"):
        raise RuntimeError(f"foot:={foot!r} 은(는) 잘못된 값입니다. foot 는 pf(Point-foot) 또는 sf(Sole-foot, 교육 기준) 입니다")
    return foot


def _setup(context):
    share = get_package_share_directory("tron1_openarm_sim")
    lc = LaunchConfiguration
    foot = check_foot(lc("foot").perform(context))
    sub, urdf, pol = FOOT_FILES[foot]
    with open(os.path.join(share, "urdf", urdf), encoding="utf-8") as f:
        robot_description = f.read()
    policy = lc("policy").perform(context) or os.path.join(share, "deployed_policies", pol)
    ekf = lc("ekf").perform(context).lower() in ("true", "1")
    cam_proc = lc("camera_process").perform(context).lower() in ("true", "1")
    camera_on = lc("camera").perform(context).lower() in ("true", "1")
    mjcf = os.path.join(share, "mjcf", sub, lc("world").perform(context))
    bridge = Node(
        package="tron1_openarm_sim", executable="tron1_bridge", name="mujoco_bridge", output="screen",
        parameters=[{
            "mjcf": mjcf,
            # camera_process: 카메라 3 대(머리·양 손목)는 별도 프로세스(tron1_cameras)가 렌더링 → 브리지는 상태만 공유 메모리로
            "viewer": lc("viewer"), "camera": camera_on and not cam_proc, "publish_image": lc("publish_image"),
            "state_shm": "tron1_sim_state" if (cam_proc and camera_on) else "",
            "move_person": lc("move_person"), "grasp_assist": lc("grasp_assist"),
            "rate_factor": lc("rate_factor"), "lidar_rays": lc("lidar_rays"),
            "camera_hz": lc("camera_hz"), "image_hz": lc("image_hz"),
            # 머리 카메라가 높아(1.24 m) 4 cm 마커가 640x480 에서 너무 작게 보임 → 960x720
            "camera_width": 960, "camera_height": 720,
            "start_x": lc("start_x"), "start_y": lc("start_y"), "start_yaw": lc("start_yaw"),
            "box_poses": lc("box_poses").perform(context),
            "box_markers": lc("box_source").perform(context) != "color",
            "publish_odom_tf": not ekf,      # EKF 를 쓰면 TF 는 EKF 가 낸다 (둘이 같이 내면 TF 가 튐)
        }],
    )
    nodes = []
    if cam_proc and camera_on:
        nodes.append(Node(package="tron1_openarm_sim", executable="tron1_cameras", name="tron1_cameras", output="screen",
                          parameters=[{"mjcf": mjcf, "state_shm": "tron1_sim_state", "head_hz": lc("camera_hz"),
                                       "wrist_hz": lc("camera_hz"), "image_hz": lc("image_hz"), "use_sim_time": True,
                                       "box_source": lc("box_source")}]))
    if ekf:
        nodes.append(Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
                          parameters=[os.path.join(share, "config", "ekf_tron1.yaml")],
                          remappings=[("odometry/filtered", "/odometry/filtered")]))
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}])
    rviz = Node(package="rviz2", executable="rviz2", arguments=["-d", lc("rviz_config")],
                parameters=[{"use_sim_time": True}], condition=IfCondition(lc("rviz")), output="log")
    return [SetEnvironmentVariable("TRON1_POLICY", policy), bridge, rsp, rviz] + nodes


def generate_launch_description():
    share = get_package_share_directory("tron1_openarm_sim")
    args = [
        DeclareLaunchArgument("foot", default_value="sf", description="sf = Sole-foot(발목 있음, 교육 기준), pf = Point-foot"),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("world", default_value="world.xml", description="world_light.xml = 저사양"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("publish_image", default_value="true"),
        DeclareLaunchArgument("move_person", default_value="true"),
        DeclareLaunchArgument("grasp_assist", default_value="true"),
        DeclareLaunchArgument("rate_factor", default_value="1.0", description="1.0=실시간, 느린 PC 는 0.5"),
        DeclareLaunchArgument("lidar_rays", default_value="360"),
        DeclareLaunchArgument("camera_hz", default_value="10.0", description="GPU 없는 환경은 5.0"),
        DeclareLaunchArgument("image_hz", default_value="5.0"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(share, "rviz", "sim.rviz")),
        DeclareLaunchArgument("camera_process", default_value="true",
                              description="true = 머리·양 손목 카메라를 별도 프로세스(tron1_cameras)가 렌더링·검출·JPEG 압축 발행"),
        DeclareLaunchArgument("ekf", default_value="false",
                              description="true = robot_localization EKF 가 odom TF 를 냄 (다리 오도메트리 + IMU)"),
        DeclareLaunchArgument("box_poses", default_value="",
                              description='상자 위치 바꾸기 (반복 집기 시험) "box_0=x,y,yaw도;box_1=..." 월드 좌표. 비우면 MJCF 그대로'),
        DeclareLaunchArgument("box_source", default_value="color",
                              description="상자 인식: aruco = 윗면 마커, color = 마커 없이 색(빨강·초록·파랑)+깊이 (camera_server.py)"),
        DeclareLaunchArgument("policy", default_value="",
                              description="보행 정책 가중치(npz). 비우면 발 형태별 기본값. LimX 사전학습: deployed_policies/limx_pf_tron1a_pretrained.npz"),
    ]
    # MuJoCo 뷰어 갱신 20 Hz (기본 30): 뷰어가 한 장면(전체 메시, 큰 창)을 그리는 약 26 ms 동안 물리가 멈춰 30 Hz 에서는
    # 실시간의 0.5 배로 돌았다 (팔 이동·집기 전체가 2 배 느림). 20 Hz 에서 1.0 배. 따로 정하면 그 값 (SIM_VIEW_HZ)
    view_hz = [SetEnvironmentVariable("SIM_VIEW_HZ", os.environ.get("SIM_VIEW_HZ", "20"))]
    return LaunchDescription(args + view_hz + [OpaqueFunction(function=_setup)])
