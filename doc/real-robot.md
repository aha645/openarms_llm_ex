# 실로봇으로 넘어가기 — 휠형·TRON1 공통 한계와 전환 순서

[README](../README.md) · [휠형 교육](wheel/README.md) · [TRON1 교육](tron1/README.md) · [용어집](glossary.md)

이 저장소는 **MuJoCo 시뮬레이션**을 대상으로 합니다. 소프트웨어는 실로봇과 같은 토픽·액션 이름을 쓰도록 만들어 브리지만 바꾸면 되도록 설계했지만, **실제 하드웨어에서는 검증하지 못했습니다.**

## 1. 시뮬레이션과 실제의 차이 (휠형)


- **교육과정 원본 코드는 공개되어 있지 않습니다.** PinkLAB 워크샵은 자사 주행형 양팔 로봇(PL-2 M) 기준이며 소스는 현장에서 배포됩니다. 이 문서의 코드는 공개 커리큘럼 14개 모듈을 같은 기술 스택(MuJoCo, ROS 2 Jazzy, Nav2, MoveIt 2, ArUco, OpenAI Structured Outputs, Flask·SSE·MJPEG)으로 새로 구현한 것입니다.
- **OpenArm에는 공식 바퀴 베이스가 없습니다.** OpenArm v1 상반신 MJCF(enactic/openarm\_mujoco)를 그대로 쓰고, 아래에 차동구동 베이스(바퀴 2 + 캐스터 2, LiDAR, IMU, 머리 RGB-D 카메라)를 직접 설계해 붙였습니다.
- **OpenArm 공식 ROS 2 패키지는 ROS 인덱스상 Humble 릴리스만 있습니다.** Jazzy에서는 소스 빌드가 필요하고, 공식 MoveIt 설정은 바뀐 URDF 경로를 참조하고 있어 그대로 쓰기 어렵습니다. 그래서 MJCF에서 URDF·SRDF를 자동 생성하는 방식으로 대체했습니다. 컨트롤러 이름은 공식 `openarm_bringup`과 같게 맞췄습니다.
- **시뮬레이션의 "파지 보조(grasp\_assist)"는 실제와 다릅니다.** 손가락 마찰만으로 집기는 MuJoCo에서 자주 미끄러지므로, 손가락이 상자에 닿은 채 닫히면 상자를 손에 고정합니다. `grasp_assist:=false`로 끄고 순수 마찰 파지를 실험할 수 있습니다.
- **카메라는 RGB-D로 흉내 냅니다.** 640×480에서 4cm 마커는 약 23픽셀이라 PnP 깊이 오차가 5\~15%(45\~113mm)였습니다. 깊이 영상으로 보정해 오차를 1\~3mm로 줄였습니다. 실로봇에서는 RGB-D 카메라(예: RealSense)나 더 큰 마커가 필요합니다.
- **실로봇 제어는 이 노트북만으로는 불가능합니다.** OpenArm 양팔(공식 가격 $6,500), 팔마다 CAN-FD USB 어댑터, 24V 전원, 바퀴 베이스와 그 드라이버, LiDAR, 카메라, 비상정지가 필요합니다. 소프트웨어 경로는 이 문서 2절에 정리했습니다.

## 2. 휠형: 시뮬 브리지를 실제 드라이버로 바꾸기


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
cd ~/openarm_mobile_ws/mobile_openarm/src && git clone https://github.com/enactic/openarm_ros2
sudo apt install -y libopenarm-can-dev openarm-can-utils
cd .. && rosdep install --from-paths src --ignore-src -r -y && .venv-mobile/bin/python -m colcon build
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

## 3. TRON1 이족: 알고 써야 하는 한계

용어

- **sim-to-real** — 시뮬레이터에서 만든 정책을 실제 로봇에 옮기는 것. 질량·마찰·지연·센서 잡음 차이 때문에 그대로는 잘 안 된다
- **limxsdk** — LimX 로봇과 통신하는 SDK (RobotState/ImuData 받기, RobotCmd 보내기)
- **e-stop** (emergency stop, 비상 정지) — 누르면 모터 전원을 끊는 스위치


| 한계 | 내용 | 대처 |
|---|---|---|
| 상반신 기둥은 가정치 | 짧은 기둥의 질량(2.21 kg)과 브래킷 몫(0.9 kg)은 가정. 실제 장착 기구는 아직 없다 | 기구를 만들면 실측 질량·무게중심을 `build_tron1_openarm.py` 상단 상수에 넣고 모델 재생성 + 재학습 |
| TRON1 탑재 한계 | 상반신 14.4 kg 는 TRON1 자체(18.6 kg)의 77%. LimX 공식 탑재 하중 사양은 이 저장소에서 확인하지 않았다 | 실기 전 LimX 사양·토크 여유 확인. 시뮬에서 무릎 토크 최대값을 `/joint_states` effort 로 확인 (한계 80 Nm) |
| 정책은 팔 위치를 모름 | 관측에 팔 관절이 없다. 몸통 반동으로만 알아챈다 | 팔을 빠르게 움직이지 않기(MoveIt 속도 배율 0.3 유지). 더 좋게는 팔 관절을 관측에 넣어 처음부터 학습 ([보행 학습 7절](tron1/locomotion-rl.md#7-더-좋은-정책을-원할-때-gpu-pc)) |
| 시뮬 LiDAR 는 자기 몸을 무시 | 실로봇에선 다리·팔이 스캔 일부를 가린다 | 실기에서는 `laser_filters` 의 각도 마스크로 가려지는 구간 제거 |
| IMU 자세를 정확하다고 가정 | 다리 오도메트리의 yaw 를 시뮬 정답 자세에서 가져온다. 실제 IMU yaw 는 천천히 흐른다 | AMCL/SLAM 이 보정. 실기에서는 robot_localization(EKF)으로 IMU+다리 오도메트리 결합 권장 |
| 넘어짐 복구 없음 | 넘어지면 `/sim/reset` 으로 다시 세워야 한다 | 실로봇은 넘어지기 전 감지(기울기 > 30°) 시 정지·e-stop |
| 시뮬 파지 보조 | 상자를 손에 고정하는 교육용 기능 (`grasp_assist`) | 실로봇에는 없음. 바퀴형과 같은 한계 |

## 4. TRON1 이족: 실로봇으로 가는 순서 (권장)


> 실제 이족 로봇에 무거운 상반신을 얹고 RL 정책을 돌리는 것은 넘어짐·부상 위험이 큽니다. 아래 순서를 건너뛰지 마세요.

1. **TRON1 단독 + LimX 공식 정책**으로 먼저 걷기 확인 (`tron1-rl-deploy-python`, LimX 매뉴얼). 안전 줄(갠트리)에 매단 상태에서
2. **장착 기구 제작 → 실측 질량·무게중심을 모델에 반영 → 재학습**. 이 저장소의 정책은 가정 질량으로 학습됨
3. 더미 무게(같은 질량·높이의 추)를 얹고 매단 채 시험 → 팔 없이 걷기
4. Isaac Gym/Lab 에서 처음부터 학습하거나 이 저장소 방식으로 미세조정한 정책을 ONNX 로 내보내(`rl/export_onnx.py`) LimX 배포 코드에서 실행. 입출력 규약이 같음
5. OpenArm 장착, 팔 고정 상태로 걷기 → 팔 천천히 움직이기 → 서서 조작
6. ROS 2 연결: 이 저장소의 브리지 자리에 **실로봇 드라이버**가 들어간다
   - 다리: limxsdk 로 RobotState/ImuData 를 받아 정책 실행 (LimX 배포 코드) + `/joint_states`(다리), `/odom`(다리 오도메트리), TF 발행 노드
   - 팔: `openarm_bringup` (CAN-FD) — 바퀴형 실로봇 안내와 같음
   - LiDAR·RGB-D 카메라 드라이버, `aruco_detector` (바퀴형과 같음)
7. Nav2 는 `nav2_params_tron1.yaml` 에서 속도를 더 낮춰(0.2 m/s) 시작

## 5. 바퀴형 vs 이족, 언제 무엇을 쓰나


| | 바퀴형 | TRON1 이족 |
|---|---|---|
| 교육 목적 | SLAM·Nav2·MoveIt·LLM 파이프라인 학습 (안정적) | + 보행 RL, sim-to-sim, 이족 오도메트리 |
| 시뮬 속도 | 빠름 | 정책 계산이 더해지지만 비슷 (numpy MLP, 50 Hz) |
| 실기 난이도 | 낮음 | 높음 (위 4절) |
| 계단·턱 | 못 감 | 원리상 가능 (이 저장소 정책은 평지만 학습) |

## 참고 자료

- [enactic/openarm](https://github.com/enactic/OpenArm) · [openarm\_mujoco](https://github.com/enactic/openarm_mujoco) · [openarm\_ros2](https://github.com/enactic/openarm_ros2)
- [ROS Index — openarm\_bringup (Humble 릴리스, launch 인자)](https://index.ros.org/p/openarm_bringup/)
- [OpenArm ROS2 Control 문서 (bringup·MoveIt 명령)](https://tokirobot.github.io/OpenArm/api-reference/ros2/control)
- [OpenArm CAN 설정 (openarm-can-configure-socketcan)](https://github.com/enactic/openarm_can/wiki)
- [mujoco\_ros2\_control (ros-controls, Jazzy 지원) — 이 워크스페이스의 브리지 대신 쓸 수 있는 대안](https://control.ros.org/jazzy/doc/mujoco_ros2_control/doc/index.html)
