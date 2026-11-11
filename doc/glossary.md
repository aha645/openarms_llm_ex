# 용어집 — 약어와 뜻

[README](../README.md) · [휠형 교육](wheel/README.md) · [TRON1 교육](tron1/README.md)

교육 문서에 나오는 약어를 **약어 (영문 전체 이름, 우리말 뜻)** 순서로 정리했습니다. 각 챕터 맨 앞의 "이 장에 나오는 용어" 표에도 같은 설명이 있습니다.

## 로봇·ROS 기본

| 약어 / 용어 | 전체 이름 | 뜻 |
|---|---|---|
| **ROS 2** | Robot Operating System 2 | 로봇 프로그램끼리 데이터를 주고받게 해 주는 소프트웨어 플랫폼. 운영체제(Windows 같은)가 아니라 Ubuntu 위에서 도는 통신·도구 모음 |
| Jazzy | ROS 2 Jazzy Jalisco | 2024년에 나온 ROS 2 버전 이름. Ubuntu 24.04용 |
| 노드 (node) | — | ROS 2에서 실행되는 프로그램 하나 (예: 브리지 노드, SLAM 노드) |
| 토픽 (topic) | — | 노드끼리 데이터를 계속 흘려보내는 이름 붙은 통로. 라디오 채널과 비슷 |
| 서비스 (service) | — | 한 번 요청하고 한 번 답을 받는 통신. 전화와 비슷 |
| 액션 (action) | — | 오래 걸리는 작업을 맡기고 진행 상황과 결과를 받는 통신. 중간 취소 가능 |
| launch | launch file | 여러 노드를 정해진 옵션으로 한 번에 실행하는 스크립트 (`*.launch.py`) |
| colcon | collective construction | ROS 2 패키지를 빌드(설치 가능한 형태로 만들기)하는 도구 |
| QoS | Quality of Service, 서비스 품질 | 토픽 전송 방식 설정 (꼭 전달 / 최신 것만 등) |
| DDS | Data Distribution Service, 데이터 분배 서비스 | ROS 2가 내부에서 쓰는 통신 표준 |

## 시뮬레이션·모델

| 약어 / 용어 | 전체 이름 | 뜻 |
|---|---|---|
| **MuJoCo** | Multi-Joint dynamics with Contact, 접촉이 있는 다관절 동역학 | Google DeepMind의 물리 시뮬레이터. 접촉·마찰 계산이 정확해 로봇 조작 연구에 많이 씀 |
| **MJCF** | MuJoCo Format (MuJoCo XML 모델 형식) | MuJoCo가 읽는 로봇·환경 설명 XML 파일 |
| Gazebo | Gazebo (gz-sim) | ROS와 오래 함께 써 온 다른 로봇 시뮬레이터 |
| SDF | Simulation Description Format, 시뮬레이션 설명 형식 | Gazebo가 쓰는 모델 XML 형식 |
| **URDF** | Unified Robot Description Format, 통합 로봇 설명 형식 | ROS가 쓰는 로봇 구조 XML. 링크(부품)와 조인트(관절) 나무 구조 |
| SRDF | Semantic Robot Description Format, 의미 로봇 설명 형식 | MoveIt용 추가 설명: 팔 그룹, 이름 붙은 자세, 서로 부딪혀도 되는 부품 쌍 |
| XML | eXtensible Markup Language | `<태그>` 로 구조를 적는 텍스트 형식 |
| 브리지 (bridge) | — | 서로 다른 두 시스템(MuJoCo ↔ ROS 2)의 데이터를 통역해 주는 프로그램 |
| sim time | simulation time, 시뮬레이션 시간 | 시뮬레이터 안의 시계. `/clock` 토픽으로 발행, 노드는 `use_sim_time` 으로 사용 |
| mocap | motion capture body | MuJoCo에서 물리 계산 없이 코드로 위치를 정하는 몸체 (움직이는 사람에 사용) |

## 좌표·센서

| 약어 / 용어 | 전체 이름 | 뜻 |
|---|---|---|
| **TF** | Transform, 좌표 변환 (ROS 패키지 tf2) | 좌표계 사이의 위치·방향 관계를 시간과 함께 알려 주는 ROS 시스템 |
| frame | coordinate frame, 좌표계 | `map`, `odom`, `base_footprint` 처럼 이름 붙은 기준 좌표 |
| odom | odometry, 주행 거리계 | 바퀴 회전량으로 계산한 로봇 이동 추정. 시간이 지나면 오차가 쌓임 |
| **LiDAR** | Light Detection And Ranging, 레이저 거리 측정기 | 레이저를 360° 돌려 쏴서 주변 벽·물체까지의 거리를 재는 센서. 결과 토픽 `/scan` |
| IMU | Inertial Measurement Unit, 관성 측정 장치 | 가속도·회전 속도를 재는 센서 |
| RGB-D | Red Green Blue + Depth, 컬러 + 깊이 카메라 | 색 영상과 픽셀별 거리를 함께 주는 카메라 (예: RealSense) |
| FK | Forward Kinematics, 정기구학 | 관절 각도 → 손끝 위치 계산 |
| IK | Inverse Kinematics, 역기구학 | 원하는 손끝 위치 → 필요한 관절 각도 계산 |
| TCP | Tool Center Point, 도구 중심점 | 그리퍼(손) 끝의 기준점. 여기서는 `openarm_*_hand_tcp` |
| rpy | roll, pitch, yaw | 방향 표현: x축 회전(roll), y축 회전(pitch, 숙이기), z축 회전(yaw, 좌우 돌기) |
| 쿼터니언 (quaternion) | — | 방향을 숫자 4개(x, y, z, w)로 표현하는 방법. ROS 메시지가 이것을 씀 |

## 지도·주행 (DAY 1)

| 약어 / 용어 | 전체 이름 | 뜻 |
|---|---|---|
| **SLAM** | Simultaneous Localization And Mapping, 동시적 위치추정 및 지도작성 | 처음 보는 공간에서 지도를 만들면서 동시에 그 지도 안의 내 위치를 알아내는 기술 |
| slam_toolbox | — | ROS 2 표준 SLAM 패키지 |
| online_async | online asynchronous | slam_toolbox 실행 방식: 실시간 센서(online)를, 밀리면 최신 것만(async) 처리 |
| loop closure | 루프 클로저 | 한 바퀴 돌아 아는 곳에 오면 쌓인 오차를 한꺼번에 고치는 SLAM 기능 |
| occupancy grid | 점유 격자 지도 | 칸마다 빈 곳/벽/모름을 표시한 2D 지도 (`.pgm` 그림 + `.yaml` 설명) |
| **AMCL** | Adaptive Monte Carlo Localization, 적응형 몬테카를로 위치추정 | 이미 있는 지도 위에서 파티클(위치 후보) 수백 개로 로봇 위치를 추정하는 방법 |
| particle | 파티클, 입자 | AMCL의 위치 후보 하나. RViz에 작은 화살표로 보임 |
| **RViz2** | ROS Visualization 2, ROS 시각화 도구 | 로봇이 보고 생각하는 것(지도, 센서, 경로, TF)을 3D로 보여 주는 화면 |
| **Nav2** | Navigation 2 | ROS 2의 자율주행 패키지 모음: 경로 계획, 장애물 회피, 위치 추정, 복구 동작 |
| costmap | 비용 지도 | 벽 근처일수록 "비싸게" 표시한 지도. Nav2가 이것을 보고 경로를 짠다 |
| inflation | 팽창 | 벽 주변에 로봇 크기만큼 위험 영역을 넓혀 칠하는 것 |
| footprint | 로봇 바닥 윤곽 | 충돌 검사에 쓰는 로봇 바닥 사각형 |
| MPPI | Model Predictive Path Integral, 모델 예측 경로 적분 | Nav2의 주행 제어기. 가능한 움직임 수백 개를 시뮬레이션해 가장 좋은 속도를 고른다 |
| BT | Behavior Tree, 행동 트리 | Nav2가 "경로 계획 → 따라가기 → 실패하면 복구"를 조합하는 방식 |
| waypoint | 경유지 | 순서대로 들르는 지점 |
| teleop | teleoperation, 원격 조종 | 사람이 키보드·조이스틱으로 직접 운전 |
| `cmd_vel` | command velocity, 속도 명령 | 로봇에 주는 선속도(m/s)·각속도(rad/s) 토픽 |
| pgm | Portable Gray Map | 흑백 그림 파일 형식 (지도 그림) |

## 팔·인식 (DAY 2)

| 약어 / 용어 | 전체 이름 | 뜻 |
|---|---|---|
| **MoveIt 2** | — | ROS 2의 로봇 팔 동작 계획 프레임워크: IK, 충돌 없는 경로 찾기, 실행 |
| move_group | — | MoveIt의 중심 노드. 계획·실행 요청을 받는다 |
| planning group | 계획 그룹 | 함께 계획하는 관절 묶음 (`left_arm`, `right_arm`, `both_arms` …) |
| planning scene | 계획 장면 | MoveIt이 아는 주변 물체(작업대, 상자). 충돌 검사에 사용 |
| OMPL | Open Motion Planning Library, 공개 동작 계획 라이브러리 | 충돌 없는 관절 경로를 찾는 알고리즘 모음 (RRTConnect 등) |
| KDL | Kinematics and Dynamics Library, 기구학·동역학 라이브러리 | IK 계산에 쓰는 라이브러리 |
| Cartesian path | 직교 좌표 경로 | 손끝을 공간에서 직선으로 움직이는 경로 |
| **ArUco** | Augmented Reality University of Córdoba (코르도바 대학 증강현실 마커) | 카메라로 ID와 위치·방향을 알아낼 수 있는 흑백 사각 마커 |
| PnP | Perspective-n-Point | 마커 모서리 4점의 영상 좌표로 카메라 기준 3D 위치를 계산하는 방법 |
| OpenCV | Open Source Computer Vision Library | 영상 처리 라이브러리. ArUco 검출에 사용 |
| pick & place | 집어서 놓기 | 물체를 집어 다른 곳에 놓는 조작 작업 |
| grasp | 파지 | 손으로 물체를 잡는 것 |
| CAN-FD | Controller Area Network with Flexible Data-rate | 실제 OpenArm 모터와 통신하는 산업용 통신 방식 |

## LLM·웹 (DAY 2)

| 약어 / 용어 | 전체 이름 | 뜻 |
|---|---|---|
| **LLM** | Large Language Model, 대규모 언어 모델 | GPT 같은 언어 AI. 여기서는 자연어 명령을 작업 계획으로 바꾼다 |
| API | Application Programming Interface | 프로그램이 다른 서비스(OpenAI)를 호출하는 약속된 방법 |
| API 키 | API key | API를 쓸 권한을 증명하는 비밀 문자열. 코드·저장소에 넣으면 안 됨 |
| JSON | JavaScript Object Notation | `{"이름": 값}` 형태의 데이터 텍스트 형식 |
| Structured Outputs | 구조화 출력 | LLM이 정해진 JSON 형식(스키마)대로만 답하게 하는 OpenAI 기능 |
| schema | 스키마 | 데이터가 가져야 할 형식 정의 |
| pydantic | — | Python에서 스키마를 클래스로 정의·검증하는 라이브러리 |
| enum | enumeration, 열거형 | 정해진 값 몇 개만 허용하는 형식 (위치 이름 8개 등) |
| Flask | — | Python 웹 서버 라이브러리 |
| SSE | Server-Sent Events, 서버 전송 이벤트 | 서버가 브라우저로 데이터를 계속 밀어 보내는 웹 기술 |
| MJPEG | Motion JPEG | JPEG 그림을 연속으로 보내 영상처럼 보이게 하는 방식 |
| HTTP | HyperText Transfer Protocol | 웹 통신 규약 |

## 설치·환경

| 약어 / 용어 | 전체 이름 | 뜻 |
|---|---|---|
| GPU | Graphics Processing Unit, 그래픽 처리 장치 | 그래픽 카드 (GTX 1050 등) |
| CUDA | Compute Unified Device Architecture | NVIDIA GPU로 계산하는 플랫폼. 이 실습 기본 과정에는 필요 없음 |
| OpenGL | Open Graphics Library | 3D 화면을 그리는 표준. MuJoCo·RViz가 사용 |
| Mesa llvmpipe | — | GPU 없이 CPU로 OpenGL을 그리는 소프트웨어 렌더러 (Docker에서 사용) |
| VNC | Virtual Network Computing | 다른 컴퓨터 화면을 원격으로 보는 방식 |
| noVNC | — | 브라우저에서 VNC 화면을 보는 프로그램 |
| Xvfb | X virtual framebuffer, X 가상 화면 | 모니터 없이 메모리 안에 만드는 가상 화면 |
