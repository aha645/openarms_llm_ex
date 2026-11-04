# MuJoCo + ROS 2 주행형 OpenArm 실습 가이드

Nov 4, 2026

> 이 저장소(openarms_llm_ex) 자체가 워크스페이스(`openarm_mobile_ws`)입니다. 문서의 `~/openarm_mobile_ws` 는 `git clone https://github.com/aha645/openarms_llm_ex.git ~/openarm_mobile_ws` 로 받은 폴더로 읽으세요.


**결론: 교육과정 14개 모듈은 이 노트북에서 시뮬레이션으로 모두 실습할 수 있습니다. 단, GTX 1050은 드라이버 580·CUDA 12.x·PyTorch 2.14에 고정해야 하고, 실로봇 제어는 하드웨어가 없어 지금은 불가능합니다.** OpenArm v1 양팔 상반신 아래에 바퀴 베이스를 붙인 로봇, ROS 2 브리지, Nav2·MoveIt 2 설정, LLM 작업 계획, 웹 대시보드까지 새로 작성했고, 최종 미션 "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"가 끝까지 성공하는 것을 확인했습니다.

| 구분 | 가능 여부 | 조건 / 이유 |
| --- | --- | --- |
| Ubuntu 24.04 + ROS 2 Jazzy + Nav2 + MoveIt 2 설치 | 가능 | apt 공식 저장소 그대로 |
| NVIDIA 드라이버 | 조건부 가능 | 580 브랜치 고정(590+는 GTX 10 시리즈 미지원) |
| CUDA / PyTorch | 조건부 가능 (과정에는 불필요) | CUDA 12.6, PyTorch 2.14 + cu126 가 마지막. 최신 버전은 동작 안 함 |
| DAY 1 자율주행 (브리지·TF·SLAM·AMCL·Nav2·순찰) | 가능 | 사람 장애물 포함. CPU가 바쁘면 실시간보다 느려짐 |
| DAY 2 양팔·Pick & Place·이동조작·LLM·대시보드 | 가능 | LLM은 OpenAI API 키(또는 다른 PC의 로컬 LLM) 필요, 키 없으면 오프라인 규칙 계획기 |
| 로컬 LLM을 이 노트북에서 실행 | 사실상 불가 | VRAM 2GB |
| 실로봇(OpenArm + 바퀴 베이스) 제어 | 불가(하드웨어 필요) | 로봇·CAN-FD 어댑터·베이스·센서 필요. 소프트웨어는 같은 인터페이스로 준비됨(미검증) |

만든 것: 워크스페이스 `openarm_mobile_ws.zip` (ROS 2 패키지 3개 + 설치 스크립트 5개). OpenArm 메시(48MB)는 설치 스크립트가 enactic/openarm\_mujoco에서 받습니다.

## 문제점과 제약 (먼저 읽기)

대상 노트북은 Samsung Notebook Odyssey NP800G5M(2017): i7-7700HQ 4코어/8스레드, GTX 1050(Pascal, 2GB 또는 4GB), RAM 8GB 또는 16GB, 128GB NVMe SSD + 1TB 5400rpm HDD입니다. 교육과정 권장 사양은 i5급, RAM 16GB, **GTX 1650 이상**이므로 GPU 한 등급이 모자랍니다.

| 항목 | 판정 | 무엇이 문제인가 | 대응 |
| --- | --- | --- | --- |
| NVIDIA 드라이버 | 조건부 가능 | 580 브랜치가 GTX 10 시리즈(Pascal)를 지원하는 마지막 드라이버. 590부터 Pascal 지원 중단 | `nvidia-driver-580` 설치 후 `apt-mark hold`. `-open` 드라이버와 `cuda-drivers` 패키지는 설치 금지(590+로 올라감) |
| CUDA Toolkit | 12.x까지 | CUDA 13.0부터 Maxwell·Pascal·Volta(5.x\~7.0) 컴파일 지원 제거 | 필요하면 `cuda-toolkit-12-6`. 이 교육과정 자체에는 CUDA 불필요 |
| PyTorch | 2.14 + cu126까지 | 2.8부터 cu128/cu129 휠에서 sm\_61 제거, 2.15부터 cu126 휠도 미배포 → 최신 PyTorch는 GTX 1050에서 "no kernel image" 오류 | `torch==2.14.0 torchvision==0.29.0 --index-url .../whl/cu126` 고정. 과정에는 불필요(선택) |
| VRAM 2GB | 주의 | MuJoCo 뷰어 + 오프스크린 카메라 + RViz2는 가능. 로컬 LLM·딥러닝 학습은 사실상 불가 | LLM은 OpenAI API 또는 다른 PC(예: RTX 3090)의 OpenAI 호환 서버 사용 |
| RAM 8GB 모델 | 위험 | 시뮬+Nav2+MoveIt+RViz 동시 실행 시 6\~8GB 사용 | 16GB로 증설 권장. 안 되면 swap 8GB + `viewer:=false` + RViz 하나만 |
| 128GB SSD | 빠듯함 | Ubuntu+ROS desktop+Nav2+MoveIt+워크스페이스 약 35\~45GB. HDD에 설치하면 빌드·실행이 매우 느림 | Ubuntu는 SSD에, 데이터(rosbag·지도)는 HDD에 |
| 하이브리드 그래픽(Optimus) | 주의 | 기본 내장 GPU로 렌더링되면 뷰어가 끊김 | `sudo prime-select nvidia`, 로그인 화면에서 **Ubuntu on Xorg** 선택(Wayland에서 RViz2 불안정) |
| Secure Boot | 주의 | 드라이버 모듈 로드 실패(nvidia-smi 오류) | 설치 중 MOK 비밀번호 설정 → 재부팅 시 Enroll MOK, 또는 BIOS에서 Secure Boot 끄기 |
| CPU 7700HQ | 가능 | 물리(2ms 스텝)+LiDAR 360선+카메라 10Hz+MPPI 제어가 겹치면 실시간보다 느려질 수 있음 | 전원 연결·성능 모드, `rate_factor:=0.5`, `lidar_rays:=180`, Nav2 MPPI batch 1000으로 축소(적용됨) |

### 교육과정과 다른 점 (이 문서가 해결한 것과 못 한 것)

- **교육과정 원본 코드는 공개되어 있지 않습니다.** PinkLAB 워크샵은 자사 주행형 양팔 로봇(PL-2 M) 기준이며 소스는 현장에서 배포됩니다. 이 문서의 코드는 공개 커리큘럼 14개 모듈을 같은 기술 스택(MuJoCo, ROS 2 Jazzy, Nav2, MoveIt 2, ArUco, OpenAI Structured Outputs, Flask·SSE·MJPEG)으로 새로 구현한 것입니다.
- **OpenArm에는 공식 바퀴 베이스가 없습니다.** OpenArm v1 상반신 MJCF(enactic/openarm\_mujoco)를 그대로 쓰고, 아래에 차동구동 베이스(바퀴 2 + 캐스터 2, LiDAR, IMU, 머리 RGB-D 카메라)를 직접 설계해 붙였습니다.
- **OpenArm 공식 ROS 2 패키지는 ROS 인덱스상 Humble 릴리스만 있습니다.** Jazzy에서는 소스 빌드가 필요하고, 공식 MoveIt 설정은 바뀐 URDF 경로를 참조하고 있어 그대로 쓰기 어렵습니다. 그래서 MJCF에서 URDF·SRDF를 자동 생성하는 방식으로 대체했습니다. 컨트롤러 이름은 공식 `openarm_bringup`과 같게 맞췄습니다.
- **시뮬레이션의 "파지 보조(grasp\_assist)"는 실제와 다릅니다.** 손가락 마찰만으로 집기는 MuJoCo에서 자주 미끄러지므로, 손가락이 상자에 닿은 채 닫히면 상자를 손에 고정합니다. `grasp_assist:=false`로 끄고 순수 마찰 파지를 실험할 수 있습니다.
- **카메라는 RGB-D로 흉내 냅니다.** 640×480에서 4cm 마커는 약 23픽셀이라 PnP 깊이 오차가 5\~15%(45\~113mm)였습니다. 깊이 영상으로 보정해 오차를 1\~3mm로 줄였습니다. 실로봇에서는 RGB-D 카메라(예: RealSense)나 더 큰 마커가 필요합니다.
- **실로봇 제어는 이 노트북만으로는 불가능합니다.** OpenArm 양팔(공식 가격 $6,500), 팔마다 CAN-FD USB 어댑터, 24V 전원, 바퀴 베이스와 그 드라이버, LiDAR, 카메라, 비상정지가 필요합니다. 소프트웨어 경로는 "실로봇 제어로 넘어가기" 절에 정리했습니다.

## 구성

```text
 ┌─ 4. 응용 ─────────────────────────────────────────────────────┐
 │  웹 대시보드(Flask·SSE·MJPEG) → task_manager → llm_planner     │
 │  (OpenAI Structured Outputs / 오프라인 규칙) → validate_plan   │
 ├─ 3. 스킬 (robot_skills) ──────────────────────────────────────┤
 │  navigate · dock · pick · place · home_arms · ArUco 인식       │
 ├─ 2. ROS 2 표준 스택 ──────────────────────────────────────────┤
 │  Nav2(AMCL·MPPI) · slam_toolbox · MoveIt 2(KDL·OMPL) · TF2     │
 ├─ 1. 브리지 (교체 지점) ───────────────────────────────────────┤
 │  시뮬레이션: bridge_node.py (MuJoCo ↔ ROS 2)                   │
 │  실로봇:     openarm_bringup(CAN-FD) + 베이스 드라이버 + LiDAR │
 │             + RGB-D 카메라 + aruco_detector                    │
 └───────────────────────────────────────────────────────────────┘
```

위 세 층은 시뮬과 실로봇에서 그대로 쓰고, 강조한 브리지 층만 교체합니다.

| ROS 2 패키지 | 내용 | 교육과정 모듈 |
| --- | --- | --- |
| `mobile_openarm_sim` | MJCF 생성기(OpenArm v1 + 바퀴 베이스), 월드, MJCF→URDF 변환, 정답 지도, 브리지 노드, sim/slam/nav launch, Nav2·SLAM 파라미터, selftest | DAY1 01\~05 |
| `mobile_openarm_moveit_config` | SRDF(그룹·이름자세·자기충돌 자동 생성), KDL, OMPL(Jazzy 형식), 컨트롤러 설정, move\_group launch | DAY2 01 |
| `mobile_openarm_apps` | `robot_skills`(Nav2·MoveIt·그리퍼·ArUco 스킬), patrol, arm\_demo, pick\_place, nav\_pick\_place, llm\_planner, task\_manager, dashboard, aruco\_detector(실로봇용), full\_system launch | DAY1 06, DAY2 02\~06 |

## 설치: 드라이버부터 빌드까지

워크스페이스 압축파일(`openarm_mobile_ws.zip`)을 홈 폴더에 풀고 `scripts/` 의 스크립트를 순서대로 실행합니다. 각 단계 끝의 확인 명령이 통과해야 다음으로 넘어갑니다.

```bash
cd ~ && unzip openarm_mobile_ws.zip && cd ~/openarm_mobile_ws
bash scripts/00_check_system.sh        # 사양·Secure Boot·세션(Xorg/Wayland) 점검
```

### 1단계 — NVIDIA 드라이버 580 (Pascal 마지막 지원)

```bash
bash scripts/01_install_nvidia_driver.sh
#   내부: apt install nvidia-driver-580 → apt-mark hold → prime-select nvidia
sudo reboot
nvidia-smi                               # GeForce GTX 1050, Driver 580.xx
glxinfo -B | grep "OpenGL renderer"     # NVIDIA GeForce GTX 1050/PCIe/SSE2 이어야 함
```

- 로그인 화면 오른쪽 아래 톱니바퀴에서 **Ubuntu on Xorg** 를 선택합니다.
- renderer 가 `llvmpipe` 나 `Mesa Intel` 이면 dGPU가 안 쓰이는 것입니다. `sudo prime-select nvidia` 후 재부팅하거나, 실행 시 `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` 를 앞에 붙입니다.

### 2단계 (선택) — CUDA / PyTorch

이 교육과정(물리 시뮬, Nav2, MoveIt, OpenAI API)은 CUDA를 쓰지 않습니다. GPU 딥러닝 실습을 따로 할 때만 설치합니다. ROS와 충돌하지 않도록 별도 venv에 설치합니다.

```bash
bash scripts/02_install_cuda_pytorch_optional.sh
#   pip install torch==2.14.0 torchvision==0.29.0 --index-url https://download.pytorch.org/whl/cu126
#   확인: cuda available True, capability (6, 1), arch_list 에 sm_61 포함
```

- PyTorch 휠에 CUDA 런타임이 들어 있어 CUDA Toolkit은 필요 없습니다. nvcc가 꼭 필요하면 `cuda-toolkit-12-6` 만 설치하고 `cuda-drivers` 는 설치하지 마세요.
- `pip install torch` (버전 미지정)는 최신 cu13x 휠을 받아 GTX 1050에서 `no kernel image is available` 오류가 납니다.

### 3단계 — ROS 2 Jazzy + Nav2 + SLAM Toolbox + MoveIt 2

```bash
bash scripts/03_install_ros2_jazzy.sh
#   ros2-apt-source 로 저장소 등록 → ros-jazzy-desktop, ros-dev-tools,
#   ros-jazzy-navigation2, nav2-bringup, slam-toolbox, moveit, moveit-configs-utils,
#   teleop-twist-keyboard, python3-opencv, python3-flask
source /opt/ros/jazzy/setup.bash
ros2 doctor --report | head -20
```

### 4단계 — 워크스페이스 (OpenArm 에셋 + 모델 생성 + 빌드)

```bash
bash scripts/04_setup_workspace.sh
#  1) git clone enactic/openarm_mujoco → v1 메시(48MB) 복사
#  2) pip --user: mujoco==3.5.0, openai, pydantic>=2  (numpy<2 제약으로 ROS 의 numpy 1.26 보호)
#  3) build_mjcf → build_world → mjcf_to_urdf → make_map → make_srdf
#  4) tools/selftest.py (ROS 없이 물리·센서·집기 14개 항목 점검)
#  5) rosdep install && colcon build --symlink-install
source ~/openarm_mobile_ws/install/setup.bash
```

- **numpy를 2.x로 올리면 안 됩니다.** ROS Jazzy apt 패키지(cv\_bridge 등)와 Ubuntu의 python3-opencv 4.6은 numpy 1.26에 맞춰 빌드되어 `numpy.core.multiarray failed to import` 가 납니다. 다른 pip 설치 시에도 `-c` 제약파일(`numpy<2`)을 함께 쓰세요.
- selftest 끝에 `총 14개 중 실패 0개` 가 나오면 시뮬레이션은 정상입니다. 카메라 항목만 실패하면 OpenGL(드라이버) 문제입니다.

## DAY 1 — MuJoCo + ROS 2 자율주행

모든 터미널에서 먼저 `source ~/openarm_mobile_ws/install/setup.bash` 를 실행합니다. 메모리가 8GB이거나 뷰어가 끊기면 모든 launch에 `world:=world_light.xml` 을 붙입니다(시각 메시 제거, 모델 메모리 1.9GB → 0.2GB).

### 01 Gazebo와 MuJoCo 이해

|  | Gazebo (gz-sim) | MuJoCo |
| --- | --- | --- |
| 모델 형식 | SDF / URDF | MJCF (XML) |
| 강점 | ROS 연동 생태계, 센서 플러그인, 대규모 환경 | 접촉·마찰 물리가 정확하고 빠름, 강화학습·매니퓰레이션 표준 |
| ROS 연결 | ros\_gz\_bridge | 직접 작성한 브리지 또는 mujoco\_ros2\_control |

```bash
# ROS 없이 월드만 열어 보기 (더블클릭=물체 선택, Ctrl+우클릭 드래그=힘 가하기)
python3 -m mujoco.viewer --mjcf=$(ros2 pkg prefix --share mobile_openarm_sim)/mjcf/world.xml
```

로봇 구조: 차동구동 베이스(바퀴 반지름 0.08m, 간격 0.44m, 섬시 0.50×0.36m) 위에 OpenArm v1 몸통 기둥과 7자유도 팔 2개, 평행 그리퍼 2개. 총 44.5kg. 월드는 10×8m 방 2개(문 2m), 작업대 2개, ArUco 상자 3개(빨강 id0, 초록 id1, 파랑 id2), 복도를 왕복하는 사람 1명입니다.

### 02 MuJoCo–ROS 2 브리지

```bash
ros2 launch mobile_openarm_sim sim.launch.py          # MuJoCo 뷰어 + RViz2
ros2 topic list                                        # /clock /joint_states /odom /scan /imu /aruco/* ...
ros2 topic hz /scan                                    # 약 10Hz
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # i/j/l/, 키로 주행
ros2 action list                                       # *_joint_trajectory_controller/follow_joint_trajectory
```

브리지(`mobile_openarm_sim/bridge_node.py`)는 물리 스레드에서 2ms 스텝을 돌리며 시뮬레이션 시간을 `/clock` 으로 내보냅니다. 다른 모든 노드는 `use_sim_time:=true` 로 동작합니다. 팔 제어는 실로봇 `openarm_bringup` 과 같은 이름의 FollowJointTrajectory 액션으로 받습니다.

```bash
# 오른팔을 관절 목표로 직접 움직이기 (MoveIt 없이)
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
 "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
```

### 03 Launch·Description·센서·TF

MJCF를 정본으로 두고 URDF를 자동 생성합니다. 그래서 시뮬과 RViz·MoveIt의 기구학이 항상 같습니다(무작위 자세 20개 FK 비교 최대 오차 7×10⁻⁶ m).

```bash
cd ~/openarm_mobile_ws/src/mobile_openarm_sim
python3 tools/mjcf_to_urdf.py --mjcf mjcf/mobile_openarm.xml --out urdf/mobile_openarm.urdf
check_urdf urdf/mobile_openarm.urdf | head          # sudo apt install liburdfdom-tools
ros2 run tf2_tools view_frames                       # frames.pdf: map→odom→base_footprint→base_link→...
ros2 run tf2_ros tf2_echo base_footprint head_camera_optical_frame
```

TF 트리: `map → odom`(AMCL/SLAM) → `base_footprint`(브리지 휠 오도메트리) → `base_link` → `lidar_link`, `imu_link`, `openarm_body_link0` → `head_camera_link` → `head_camera_optical_frame`, `openarm_{left,right}_link0..7` → `hand` → `hand_tcp`.

### 04 SLAM과 RViz2

```bash
ros2 launch mobile_openarm_sim slam.launch.py                 # slam_toolbox online_async
ros2 run teleop_twist_keyboard teleop_twist_keyboard          # 방 두 개를 천천히 한 바퀴
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p use_sim_time:=true
# AMCL 위치추정: 저장한 지도로 Nav2 실행 → RViz '2D Pose Estimate' 로 초기 위치 보정
ros2 launch mobile_openarm_sim nav.launch.py map:=$HOME/maps/my_map.yaml
```

- **반드시 `home`(월드 원점)에서 매핑을 시작합니다.** SLAM 지도 원점 = 시작 위치이고, `config/locations.yaml` 의 작업장 좌표가 이 원점 기준이기 때문입니다.
- SLAM을 건너뛰려면 월드에서 바로 만든 정답 지도 `maps/world_map.yaml` 을 씁니다(nav.launch 기본값).

### 05 Nav2 자율주행

```bash
ros2 launch mobile_openarm_sim nav.launch.py      # RViz 상단 '2D Goal Pose' 로 목표 클릭
# CLI 로 목표 주기
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
 "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: 2.45}, orientation: {z: 0.7071, w: 0.7071}}}}"
# 사람 장애물 켜기/끄기
ros2 service call /sim/set_person_motion std_srvs/srv/SetBool "{data: true}"
```

주요 파라미터(`config/nav2_params.yaml`, Nav2 Jazzy 기본값에서 변경): footprint 0.55×0.52m, inflation 0.55m, MPPI batch 1000·time\_steps 48·vx\_max 0.4m/s, goal tolerance 0.08m/0.10rad. `tools/make_nav2_params.py` 를 고쳐 다시 생성하며 실험합니다.

### 06 통합 데모: 순찰

```bash
ros2 launch mobile_openarm_sim nav.launch.py
ros2 run mobile_openarm_apps patrol --ros-args -p loops:=2 -p mode:=waypoints   # patrol_1→2→3→4 → home
```

## DAY 2 — 양팔 매니퓰레이션 + LLM 작업 자동화

### 01 MoveIt 2 양팔 제어

Planning Group은 `left_arm`, `right_arm`(체인 `openarm_*_link0` → `openarm_*_hand_tcp`, KDL IK), `both_arms`(두 그룹의 합), `left_gripper`, `right_gripper` 입니다. 이름 자세는 `home`, `ready`, `carry`(손이 footprint 안으로 들어오는 운반 자세), `hands_up` 입니다. SRDF의 자기충돌 비활성 쌍 233개는 MuJoCo에서 무작위 자세 3,000개를 샘플링해 만들었습니다(Setup Assistant의 Self-Collisions 단계와 같은 원리).

```bash
ros2 launch mobile_openarm_sim sim.launch.py rviz:=false
ros2 launch mobile_openarm_moveit_config move_group.launch.py         # RViz MotionPlanning 패널 포함
ros2 run mobile_openarm_apps arm_demo   # 이름자세 → 관절목표 → 말단 자세목표(IK) → 직선(Cartesian) 경로
```

### 02 ArUco 기반 Pick & Place

카메라 영상은 브리지 안에서 직접 처리하고 검출 결과만 `/aruco/detections`(base\_footprint 기준 JSON)로 발행합니다. 교육과정의 "Python 직접 수신, 검출 결과만 ROS 2 발행" 구조와 같습니다.

```bash
# 로봇을 작업대 A 앞에서 시작 (Nav2 없이 조작만 연습)
ros2 launch mobile_openarm_sim sim.launch.py start_x:=2.5 start_y:=2.45 start_yaw:=1.5708
ros2 launch mobile_openarm_moveit_config move_group.launch.py
ros2 topic echo /aruco/detections --once
ros2 run mobile_openarm_apps pick_place --ros-args -p object:=box_0
```

동작 순서(`robot_skills.py` 의 `pick()` / `place()`):

1. **도킹** — 작업대와 정면이 되도록 제자리 회전(odom 폐루프) 후, 작업대 앞 모서리까지 0.40m가 되도록 직진. Nav2는 작업대 0.70m 앞까지만 보냅니다(팽창 영역 안에서는 MPPI가 제자리걸음을 반복했음).
2. **인식** — 정지 1초 뒤 마커 5프레임 중앙값, 상자 중심 = 마커 − 2.5cm. 상자까지 전방 거리가 0.52m에서 3cm 이상 벗어나면 베이스를 그만큼 움직이고 다시 인식(AMCL 오차 대신 카메라 거리 기준).
3. **접근 자세** — 45° 숙인 접근에서 접근축 회전(roll)을 70·−70·±90·±50·0° 순으로 `compute_ik` 시도. roll 0은 손가락이 위아래로 닫혀 모서리를 집고, ±70이 옆면을 가장 안정적으로 잡습니다(MuJoCo 실험: ±70은 상자 밀림 1\~2cm, ±20은 놓침).
4. **집기** — 10cm 뒤 접근점까지 OMPL 계획 → 상자 중심보다 1cm 더 깊이 직선 접근 → 닫기 → **손가락 간격으로 파지 확인**(끝까지 닫히면 실패, 다시 인식해 1회 재시도) → 직선 10cm 들기 → `carry`.
5. **놓기** — 도킹 → 작업대 모서리 12cm 안쪽 위로 접근 → 내려놓고 열기 → 뒤·위로 빼기 → `carry` → `home`(손이 방금 놓은 상자 위를 지나는 `ready` 는 거치지 않음).

작업대는 도착 후 현재 TF 기준으로 MoveIt 충돌 객체에 넣고, 주행 전에 지웁니다.

### 03 Nav + Pick & Place (이동 조작)

```bash
ros2 launch mobile_openarm_sim nav.launch.py
ros2 launch mobile_openarm_moveit_config move_group.launch.py rviz:=false
ros2 run mobile_openarm_apps nav_pick_place --ros-args -p object:=box_0
#   home 자세 → pick_station 주행 → 집기 → carry 자세로 place_station 주행 → 놓기 → home
```

주행↔조작 전환 규칙: 주행 중에는 팔을 footprint 안(`home`/`carry`)에 두고, 작업대 충돌 객체는 주행 전에 지우고 도착 후 현재 TF(map→base\_footprint)로 다시 넣습니다. 마커 좌표는 항상 base\_footprint 기준이라 지도 오차와 무관합니다.

### 04 LLM과 OpenAI API

```bash
export OPENAI_API_KEY=sk-...            # 본인 키
export OPENAI_MODEL=gpt-4o-mini         # 계정에서 쓸 수 있는 모델명으로
ros2 run mobile_openarm_apps llm_planner "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
ros2 run mobile_openarm_apps llm_planner --offline "파란 상자랑 초록 상자 옮겨줘"   # 키 없이 규칙 기반
# 다른 PC의 로컬 LLM (LM Studio / Ollama / vLLM, OpenAI 호환)
export OPENAI_BASE_URL=http://<서버IP>:1234/v1 OPENAI_API_KEY=local OPENAI_MODEL=<모델명>
```

`RobotPlan` 스키마(pydantic)가 안전 범위의 핵심입니다. 행동은 `navigate / pick / place / home_arms / say / wait` 6개, 위치와 물체는 Enum으로만 고를 수 있어 LLM이 임의 좌표나 명령을 만들 수 없습니다. OpenAI Structured Outputs(Responses API `text_format=`)를 먼저 쓰고, 지원하지 않는 로컬 서버는 Chat Completions json\_schema로 자동 전환합니다.

### 05 LLM → ROS 2 작업 실행

`validate_plan()` 이 상태를 따라가며 전제조건을 검사합니다. 이미 든 채 또 집기, 빈손으로 놓기, 작업대 없는 곳에 놓기, 12단계 초과는 거부합니다. 빠진 이동 단계는 자동으로 넣고 경고로 남깁니다.

```bash
ros2 run mobile_openarm_apps task_manager --ros-args -p planner:=llm    # 또는 planner:=offline
ros2 topic pub --once /task/command std_msgs/String "{data: '빨간 상자를 왼쪽 작업대로 옮기고 돌아와'}"
ros2 topic echo /task/status            # planning → running(단계별 pending/running/done) → done
ros2 service call /task/cancel std_srvs/srv/Trigger
```

### 06 웹 대시보드 & Final

```bash
ros2 launch mobile_openarm_apps full_system.launch.py planner:=offline   # 또는 planner:=llm
# 브라우저: http://localhost:8080  (같은 네트워크의 휴대폰에서도 http://<노트북IP>:8080)
```

화면 구성: 지도(/map PNG + 로봇 위치·작업장 표시), 머리 카메라 MJPEG(`/camera.mjpg`), 자연어 명령 입력과 빠른 버튼, 미션 단계별 상태, 로그. 상태는 SSE(`/events`)로 0.5초마다 푸시됩니다. 최종 미션은 "빨간 상자를 왼쪽 작업대로 옮기고 돌아와" (5단계: 이동 → 집기 → 이동 → 놓기 → 복귀)입니다.

## 실로봇 제어로 넘어가기

소프트웨어는 실로봇과 같은 토픽·액션 이름을 쓰도록 만들어, 시뮬 브리지 하나를 실제 드라이버 4개로 바꾸면 됩니다. 단, 실제 하드웨어에서는 검증하지 못했습니다.

| 시뮬에서 `mujoco_bridge` 가 하던 일 | 실로봇에서 대신할 것 | 필요한 하드웨어 |
| --- | --- | --- |
| 팔·그리퍼 FollowJointTrajectory, /joint\_states | `openarm_bringup` (ros2\_control + `openarm_hardware`, Damiao 모터 CAN-FD) | OpenArm 양팔(공식 가 $6,500), 팔당 CAN-FD USB 어댑터 1개(can0/can1), 24V 전원 |
| /cmd\_vel → 바퀴, /odom + TF odom→base\_footprint | 베이스 드라이버 (상용 베이스의 ROS 2 드라이버 또는 ros2\_control `diff_drive_controller`) | 차동구동 베이스(양팔 상반신 약 16kg + 배터리를 싣을 내하중) |
| /scan | LiDAR 드라이버 (frame 이름을 `lidar_link` 로) | 2D LiDAR |
| /aruco/detections | `ros2 run mobile_openarm_apps aruco_detector` (이번에 추가한 노드, 깊이 토픽 지원) | RGB-D 카메라(예: RealSense), 머리 거치대 |
| /clock | 필요 없음 — 모든 노드 `use_sim_time:=false` | 비상정지 스위치(모터 전원 차단) |

```bash
# OpenArm ROS 2 (Jazzy 는 소스 빌드). openarm_can 은 OpenArm 문서의 PPA 안내로 설치
cd ~/openarm_mobile_ws/src && git clone https://github.com/enactic/openarm_ros2
sudo apt install -y libopenarm-can-dev openarm-can-utils
cd .. && rosdep install --from-paths src --ignore-src -r -y && colcon build
# 1) 가짜 하드웨어로 먼저 확인
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v1.0
# 2) CAN-FD 설정 (1Mbps nominal / 5Mbps data) → 실제 팔
openarm-can-configure-socketcan can0 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v1.0 use_fake_hardware:=false \
    right_can_interface:=can0 left_can_interface:=can1
```

바뀌어야 할 것:

1. **robot\_description 통일.** 실로봇 런치는 openarm\_description의 URDF를, Nav2·MoveIt은 이 워크스페이스의 베이스 포함 URDF를 씁니다. 실제 베이스 치수·센서 위치로 `tools/build_mjcf.py` 상수를 고치고 URDF를 다시 만든 뒤, 한 쪽 robot\_state\_publisher만 띄웁니다.
2. **그리퍼 단위 확인.** 시뮬은 `openarm_*_finger_joint1` 0\~0.044m(손가락 1개 이동량)입니다. 실제 컨트롤러의 단위·범위를 `openarm_bringup` 설정에서 확인해 `robot_skills.py` 의 0.044 값을 맞춥니다.
3. **grasp\_assist 없음.** 실제로는 마찰로 잡으므로 닫는 힘(Damiao 전류 제한)과 접근 자세를 다시 튜닝합니다.
4. **안전.** MoveIt 속도 배율 0.1부터 시작, 작업 반경에 사람 금지, 비상정지를 손에 들고 시험합니다. 주행 중 팔 자세는 반드시 `home`/`carry`.
5. **노트북 역할.** 2017년 노트북을 로봇 위 컴퓨터로 올리면 USB 포트(CAN 2 + LiDAR + 카메라)가 부족하므로 전원 공급형 USB 3.0 허브가 필요합니다. CAN 레이턴시가 80ms를 넘으면 포트를 바꾸거나 백그라운드 부하를 줄입니다.

## 검증 결과와 트러블슈팅

노트북 실물이 아닌, 클라우드 Linux(CPU 2코어, RAM 8GB, GPU 없음·소프트웨어 렌더)에 ROS 2 Jazzy 패키지(RoboStack: Nav2 1.3.13, MoveIt 2.12.4)를 설치해 헤드리스로 검증했습니다. RViz와 MuJoCo 뷰어 창, 실제 Ubuntu apt 패키지, GTX 1050 드라이버는 직접 확인하지 못했습니다.

| 항목 | 결과 |
| --- | --- |
| selftest 14항목 (MuJoCo 3.5 + numpy 1.26 + OpenCV 4.6 / MuJoCo 3.17 + OpenCV 5.0) | 모두 통과: 1m 직진 오도메트리 오차 1.6cm, ArUco 위치 오차 1\~3mm, IK·집기·운반·놓기 |
| URDF ↔ MJCF 기구학 | 무작위 20자세 FK 최대 오차 7×10⁻⁶ m, 카메라 프레임 일치 |
| 브리지 토픽·액션 | /clock·/scan·/odom·/aruco/\* 발행, 팔·그리퍼 액션 6개 동작 |
| Nav2 (정답 지도 + AMCL + MPPI) | home → 문 → 작업대 A 주행 성공(사람 장애물 이동 중) |
| MoveIt + ArUco Pick & Place (DAY2-02) | 성공: 상자 인식 → 집기 → 같은 작업대에 놓기 → home |
| 자연어 최종 미션 (대시보드 → task\_manager → Nav2·MoveIt) | 5단계 전부 성공(590초): 작업대 A 주행 → 도킹 → 집기 → 작업대 B 주행 → 놓기(상자 최종 위치 −2.22, 3.00, 0.725) → home 복귀. 불가능한 명령("커피 한 잔 타줘")은 rejected. SLAM(/map 생성)과 arm\_demo(양팔 both\_arms 계획 포함)도 통과. patrol·OpenAI 온라인 호출·aruco\_detector(실로봇용)는 미실행 |
| 실시간 배속 | 전체 시스템 동시 실행 시 약 0.5배(소프트웨어 카메라 렌더가 CPU 1코어 사용). 모든 노드가 시뮬 시간을 쓰므로 느려질 뿐 동작은 같음 |

### 검증 중 발견해 고친 문제 (같은 증상이 보이면 참고)

| 증상 | 원인 | 조치 |
| --- | --- | --- |
| Nav2가 매초 "Detected jump back in time", TF extrapolation 오류 | 시뮬 시간 107.99999 같은 float를 sec/nanosec로 나눌 때 nanosec이 1e9로 반올림돼 1초 뒤로 감 | 전체 ns로 바꾼 뒤 나눔 (`bridge_node.py` `stamp()`) |
| controller\_server 활성화 실패 "Controller period more then model dt" | MPPI는 controller\_frequency ≥ 1/model\_dt(20Hz) 필요 | 20Hz 유지, 대신 batch·time\_steps 축소 |
| "Timed out while waiting for action server ... compute\_path\_to\_pose" | CPU가 바빠 BT가 플래너 응답을 20ms만 기다림 | `default_server_timeout: 500`, planner 5Hz |
| MoveIt "Start state out of bounds" (error -26) | 위치제어 오버슈트로 joint5가 한계를 0.001rad 초과 | /joint\_states 를 한계 안으로 clamp |
| 작업대 앞에서 "Failed to make progress" 반복 | 목표가 팽창 영역 안 | 스테이션을 0.70m 앞으로, 나머지는 도킹 |
| 처음 집기에서 자주 놓침 | home→접근점 경로가 상자를 침 + 손가락이 위아래로 닫히는 roll | carry에서 출발, 상자를 충돌 객체로 등록, roll ±70 우선, 파지 확인 후 재시도 |
| `numpy.core.multiarray failed to import` | pip가 numpy 2.x를 설치 | `pip install "numpy<2"` 로 되돌리고 항상 `-c` 제약파일 사용 |
| `colcon build --symlink-install` 에서 "option --editable not recognized" | setuptools 80 이상(특히 conda/venv) | `--symlink-install` 없이 빌드. Ubuntu apt setuptools 68에서는 정상 |
| MuJoCo 뷰어가 열리지 않거나 검은 화면 | Wayland 세션 또는 내장 GPU | Ubuntu on Xorg, `prime-select nvidia`, 안 되면 `viewer:=false` |
| 모델 로드에 RAM 1.9GB | 시각 메시 47개(약 160만 면) | `world:=world_light.xml` (0.2GB) |

### 노트북에서 직접 재보세요

```bash
ros2 launch mobile_openarm_sim sim.launch.py rviz:=false viewer:=false &
ros2 topic hz /clock     # 100Hz 가까우면 실시간. 크게 낮으면 lidar_rays:=180, camera_hz 낮추기, rate_factor:=0.5
nvidia-smi dmon -s u     # 뷰어·카메라가 GPU 를 쓰는지 확인
```

## Sources

- [PinkLAB — MuJoCo + ROS2 2일 워크샵 1차 (커리큘럼·준비물)](https://pinklab.art/academy/mujoco-ros2-workshop-1/)
- [enactic/openarm](https://github.com/enactic/OpenArm) · [openarm\_mujoco](https://github.com/enactic/openarm_mujoco) · [openarm\_ros2](https://github.com/enactic/openarm_ros2)
- [ROS Index — openarm\_bringup (Humble 릴리스, launch 인자)](https://index.ros.org/p/openarm_bringup/)
- [OpenArm ROS2 Control 문서 (bringup·MoveIt 명령)](https://tokirobot.github.io/OpenArm/api-reference/ros2/control)
- [OpenArm CAN 설정 (openarm-can-configure-socketcan)](https://github.com/enactic/openarm_can/wiki)
- [Samsung Odyssey NP800G5M 사양 (Notebookcheck)](https://www.notebookcheck.net/Samsung-Odyssey-NP800G5M-7700HQ-FHD-GTX-1050-Laptop-Review.213840.0.html)
- [NVIDIA 580이 Maxwell·Pascal·Volta 마지막 지원 드라이버 (Notebookcheck)](https://www.notebookcheck.net/Nvidia-confirms-upcoming-end-of-driver-support-for-GTX-700-900-and-10-series-GPUs.1048442.0.html)
- [PyTorch: CUDA 12.8/12.9 빌드에서 Maxwell·Pascal 제거](https://dev-discuss.pytorch.org/t/cuda-toolkit-version-and-architecture-support-update-maxwell-and-pascal-architecture-support-removed-in-cuda-12-8-and-12-9-builds/3128)
- [PyTorch: 2.15부터 CUDA 12.6 휠 중단 (Pascal 마지막 = 2.14)](https://dev-discuss.pytorch.org/t/notice-cuda-12-6-wheels-will-no-longer-be-published-from-pytorch-2-15-drops-maxwell-pascal-volta/3432)
- [GTX 1050 + PyTorch 2.8 cu128 "no kernel image" 사례](https://github.com/bnsreenu/digitalsreeni-image-annotator/issues/57)
- [mujoco\_ros2\_control (ros-controls, Jazzy 지원) — 이 워크스페이스의 브리지 대신 쓸 수 있는 대안](https://control.ros.org/jazzy/doc/mujoco_ros2_control/doc/index.html)
- [Nav2 Jazzy 기본 파라미터](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_bringup/params/nav2_params.yaml)
