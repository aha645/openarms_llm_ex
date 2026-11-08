# 휠 + OpenArm — 2일차: 팔로 집고, 운반하고, 자연어로 명령하기

[README](../../README.md) · 이전: [1일차](wheel-day1.md)

오늘은 **팔 동작 → 작업대 앞에서 집기 → 작업대 사이 운반 → 자연어 계획 → 실제 미션 실행 → 웹 통합** 순서로 실습합니다.

## 시작 전에

Ubuntu 24.04, ROS 2 Jazzy, 이 저장소의 에셋 생성·빌드가 완료된 환경을 사용합니다. 설치는 [노트북](../setup/ubuntu24-native.md) 또는 [iMac Docker](../setup/imac-docker.md)를 보세요.

- T1/T2/T3는 별도 터미널입니다. Docker에서는 모두 컨테이너 안에서 실행합니다.
- 각 절의 시작에 종료하라고 적혀 있으면 이전 프로그램을 `Ctrl+C`로 종료합니다.
- `sim`, `nav`, `full_system`은 각각 시뮬을 포함합니다. 동시에 실행하면 안 됩니다.
- 아래의 명령은 모두 실제로 **한 줄**입니다. 화면에서 길게 보이더라도 중간에 명령이 나뉜 것은 아닙니다.

## 0. 새 터미널마다 준비하기

### 저장소로 이동

```bash
cd /home/liky/Documents/openarms_llm_ex
```

**상세 설명:** `cd` 뒤는 저장소 경로입니다. 다른 곳에 받았다면 바꾸세요. Docker 안에서는 대신 `cd /root/openarm_mobile_ws`를 실행합니다.

**이 한 줄이 하는 일:** 상대 경로 `install/...`가 올바른 파일을 가리키도록 작업 폴더를 맞춥니다.

### 기본 ROS 환경 적용

```bash
source /opt/ros/jazzy/setup.bash
```

**상세 설명:** `source`는 지정한 환경 설정을 현재 터미널에 적용합니다. 이 파일은 기본 ROS 2 Jazzy의 검색 경로를 설정합니다.

**이 한 줄이 하는 일:** `ros2` 명령과 기본 패키지를 사용할 수 있게 합니다. 노드를 실행하지는 않습니다.

### 우리 로봇 패키지 환경 적용

```bash
source install/setup.bash
```

**상세 설명:** 이 저장소를 빌드하면서 생성한 환경 설정입니다. 우리 패키지의 실행 프로그램과 설정을 찾게 합니다.

**이 한 줄이 하는 일:** `mobile_openarm_apps` 등 이 프로젝트의 패키지를 현재 터미널에서 사용할 수 있게 합니다.

이전에 TRON1을 사용한 터미널이라면 실행합니다.

```bash
unset OPENARM_BIPED OPENARM_SRDF
```

**상세 설명:** 이족 모드와 SRDF 파일 강제 선택 환경 변수를 해제합니다. SRDF는 팔 그룹·이름 자세 등을 정의하는 설정입니다.

**이 한 줄이 하는 일:** 휠형 기본 설정을 사용하게 합니다. 파일을 삭제하지 않습니다.

### 명령 구조 다시 보기

```text
ros2 run    패키지  실행프로그램  그 프로그램에 줄 입력값/옵션
ros2 launch 패키지  launch파일   launch에 줄 설정값
```

`run`의 실행 이름은 반드시 Python 파일명이 아닙니다. 우리 앱은 `setup.py`에서 실행 이름을 Python의 `main()` 함수에 연결해 둡니다. 실행 이름 **뒤의 문법은 프로그램마다 다릅니다.** 아래에서 필요한 입력값을 각각 설명합니다.

## 1. MoveIt으로 팔 움직이기

기존 1일차 프로그램을 모두 종료합니다. 이 절에서는 **T1의 로봇, T2의 팔 계획 서버, T3의 시험 앱**을 함께 사용합니다. MoveIt은 장애물과 관절 조건을 고려해 팔 경로를 계획하고 실행을 요청하는 소프트웨어입니다.

### T1 — 움직일 로봇 준비

```bash
ros2 launch mobile_openarm_sim sim.launch.py world:=world_light.xml rviz:=false
```

**상세 설명:** `mobile_openarm_sim` 패키지의 `sim.launch.py`를 실행합니다. `world:=world_light.xml`은 가벼운 월드 선택, `rviz:=false`는 sim 측 RViz 창 끄기입니다. MuJoCo 뷰어는 기본으로 켜집니다.

**이 한 줄이 하는 일:** 팔 명령을 받아 실제로 움직일 시뮬 로봇과 ROS 브리지를 시작합니다. 팔 계획 서버는 다음 명령으로 따로 실행합니다.

### T2 — 팔 계획 서버 준비

```bash
ros2 launch mobile_openarm_moveit_config move_group.launch.py
```

**상세 설명:** `mobile_openarm_moveit_config`는 휠형 로봇의 MoveIt 설정 패키지입니다. `move_group.launch.py`는 팔 계획·실행 요청을 처리하는 서버와 기본 RViz를 시작합니다. 추가 설정값을 주지 않아 기본 설정을 사용합니다.

**이 한 줄이 하는 일:** 앱이 팔 목표를 보내면 경로를 계산하고 브리지에 실행을 요청할 준비를 합니다. 이 명령만으로 데모 동작을 시작하지는 않습니다.

### T3 — 팔 데모 실행

```bash
ros2 run mobile_openarm_apps arm_demo
```

**상세 설명:** `mobile_openarm_apps`는 작업 앱 패키지, `arm_demo`는 팔 시험 프로그램의 등록된 실행 이름입니다. 추가 입력값은 없습니다. 프로그램 안에 정의된 이름 자세·손끝 목표를 순서대로 요청합니다.

**이 한 줄이 하는 일:** T2의 MoveIt에 목표를 보내고 T1의 팔이 실제로 움직이는지 시험합니다.

**확인:** RViz의 계획 결과와 MuJoCo의 팔 움직임을 함께 확인합니다. 계획 성공 로그만으로 실제 실행까지 성공했다고 판단하지 마세요.

## 2. 작업대 앞에서 상자를 집어 같은 작업대에 놓기

**앞 절의 모든 프로그램을 종료**합니다. 주행을 제외하고 집기만 시험하기 위해 로봇을 작업대 앞에서 시작합니다.

### T1 — 작업대 앞 위치에서 로봇 시작

```bash
ros2 launch mobile_openarm_sim sim.launch.py world:=world_light.xml rviz:=false start_x:=2.5 start_y:=2.45 start_yaw:=1.5708
```

**상세 설명**

- `sim.launch.py`: 로봇과 ROS 연결을 실행합니다.
- `world:=world_light.xml`: 가벼운 물리 월드 선택입니다.
- `rviz:=false`: sim 측 RViz 창을 끕니다.
- `start_x:=2.5`, `start_y:=2.45`: 로봇 시작 위치를 월드 좌표 x=2.5m, y=2.45m로 정합니다. 물체나 손의 목표 좌표가 아닙니다.
- `start_yaw:=1.5708`: 로봇 시작 방향입니다. 단위는 rad이며 약 +90°입니다. +x 방향에서 +y 방향으로 돌아선 자세입니다.

**이 한 줄이 하는 일:** 로봇을 작업대 앞에 배치해 이동 과정 없이 카메라 인식과 팔 집기부터 시험할 수 있게 합니다.

### T2 — 창 없이 MoveIt 시작

```bash
ros2 launch mobile_openarm_moveit_config move_group.launch.py rviz:=false
```

**상세 설명:** 1절과 같은 팔 계획 서버입니다. 이번에는 `rviz:=false`를 주어 MoveIt의 RViz도 열지 않습니다. 창을 끄는 것과 서버를 끄는 것은 다릅니다.

**이 한 줄이 하는 일:** 화면 렌더링 부담을 줄이면서 팔 계획·실행 기능을 준비합니다.

### T3 — 카메라 검출 결과 하나 확인

```bash
ros2 topic echo /aruco/detections --once
```

**상세 설명:** `ros2 topic echo`는 토픽의 메시지를 읽어 출력합니다. `/aruco/detections`는 카메라 영상에서 찾은 ArUco 마커 검출 정보입니다. ArUco는 물체에 붙인 사각형 식별 마커입니다. `--once`는 메시지 하나를 받은 뒤 종료한다는 뜻이지 물체 하나만 찾으라는 뜻은 아닙니다.

**이 한 줄이 하는 일:** 집기 전에 카메라에서 상자 마커를 찾고 있는지 확인합니다. 내용이 비어 있다면 메시지 수신은 됐지만 상자는 아직 인식하지 못한 것입니다.

### T3 — 빨간 상자 집기/놓기

```bash
ros2 run mobile_openarm_apps pick_place --ros-args -p object:=box_0
```

**상세 설명**

- `mobile_openarm_apps`: 앱 패키지입니다.
- `pick_place`: 작업대 앞에서 상자를 집고 놓는 실행 프로그램입니다.
- `--ros-args`: 이후 값을 ROS 설정으로 전달하라는 구분자입니다.
- `-p object:=box_0`: 노드 파라미터 `object`를 `box_0`로 설정합니다. `box_0`는 빨간 상자, `box_1`은 초록, `box_2`는 파란 상자입니다.

**이 한 줄이 하는 일:** 빨간 상자를 카메라로 확인하고 집어 **같은 작업대의 다른 위치**에 놓습니다. 다른 작업대로 운반하는 명령은 아닙니다.

**확인:** 검출 → 접근 → 그리퍼 닫기 → 상자 들림 → 놓기를 관찰합니다. 기본 파지 보조가 켜진 교육용 시뮬이므로 현실 로봇의 파지 성능으로 해석하지 마세요.

위 시작 좌표는 `pick_place` 앱의 기본 좌표와 맞춘 값입니다. 처음에는 바꾸지 마세요. 변경하려면 sim뿐 아니라 앱의 시작 좌표 파라미터도 맞춰야 합니다.

## 3. 작업대 A에서 B로 운반하기

**앞 절의 모든 프로그램을 종료**합니다. 이번에는 home에서 출발해 주행과 집기를 함께 실행합니다.

### T1 — 자율주행 준비

```bash
ros2 launch mobile_openarm_sim nav.launch.py world:=world_light.xml rviz:=false
```

**상세 설명:** `nav.launch.py`는 시뮬, 기본 지도, AMCL 위치 추정, Nav2 자율주행을 함께 실행합니다. `world`는 가벼운 물리 월드 선택이고 `rviz:=false`는 주행 RViz 창 끄기입니다. `map`을 지정하지 않아 패키지의 기본 지도를 사용합니다.

**이 한 줄이 하는 일:** 로봇이 home에서 작업대까지 스스로 이동할 준비를 합니다. 앞 절의 작업대 시작 좌표는 이 명령에 추가하지 않습니다.

### T2 — 팔 계획 준비

```bash
ros2 launch mobile_openarm_moveit_config move_group.launch.py rviz:=false
```

**상세 설명:** 앞 절과 같은 MoveIt 서버입니다. `rviz:=false`는 창만 끄며 팔 계획 기능은 유지합니다.

**이 한 줄이 하는 일:** 주행으로 작업대에 도착한 뒤 사용할 집기·놓기 기능을 준비합니다.

### T3 — 운반 시나리오 실행

```bash
ros2 run mobile_openarm_apps nav_pick_place --ros-args -p object:=box_0
```

**상세 설명:** 실행 이름이 `pick_place`가 아닌 **`nav_pick_place`**입니다. 주행까지 포함한 앱입니다. `--ros-args -p object:=box_0`는 ROS 파라미터로 운반할 빨간 상자를 선택합니다. 목적지는 앱의 기본 작업대 설정을 사용합니다.

**이 한 줄이 하는 일:** 작업대 A로 이동 → 빨간 상자 집기 → 작업대 B로 이동 → 놓기 → home 복귀를 순서대로 요청합니다.

**확인:** 앱 종료 여부뿐 아니라 상자가 왼쪽 작업대에 놓이고 로봇이 복귀했는지 봅니다. 같은 시험을 반복하려면 전체를 재시작해 상자 배치를 초기화하세요.

## 4. 자연어를 계획으로 바꾸기 — 아직 로봇은 움직이지 않음

앞 절 프로그램은 종료해도 됩니다. 여기서는 계획기만 시험합니다.

```bash
ros2 run mobile_openarm_apps llm_planner --offline "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
```

**상세 설명**

- `llm_planner`: 문장을 작업 단계 목록으로 바꾸는 실행 프로그램입니다.
- `--offline`: API 대신 키워드 규칙을 사용합니다. 실제 LLM 호출이 아닙니다. 이 프로그램이 직접 받는 옵션이므로 `--ros-args -p`를 붙이지 않습니다.
- `"빨간 ... 돌아와"`: 프로그램에 전달하는 입력 문장 하나입니다. 공백이 있어 큰따옴표로 묶습니다. Python 파일이나 ROS 파라미터 이름이 아닙니다.

**이 한 줄이 하는 일:** 문장을 이동·집기·이동·놓기·복귀 계획으로 바꾸고 검증 결과를 출력합니다. **계획 출력만 하며 로봇을 움직이지 않습니다.**

수행할 수 없는 요청도 시험합니다.

```bash
ros2 run mobile_openarm_apps llm_planner --offline "커피 한 잔 타줘"
```

**상세 설명:** 같은 계획기에 입력 문장만 바꿉니다. 구현된 이동·상자 조작 범위를 벗어난 요청입니다.

**이 한 줄이 하는 일:** 할 수 없는 요청을 거부하는지 확인합니다. 이 CLI는 검증 거부를 오류 출력(Traceback)으로 표시할 수 있습니다.

실제 API 연결은 필수가 아닙니다. 먼저 아래 offline 통합 실습까지 완료하세요. API 설정은 [LLM 상세 문서](../labs/day2-04-llm-planner.md)를 참고합니다.

## 5. Task Manager에 자연어를 보내 실제로 실행하기

**3절의 T1 Nav2와 T2 MoveIt을 새로 실행**하고 초기화를 기다립니다. 앞 운반으로 상자가 옮겨진 상태라면 반드시 재시작하세요. 이 절은 계획을 실제 로봇 기능 호출로 연결합니다.

### T3 — 작업 관리자 시작

```bash
ros2 run mobile_openarm_apps task_manager --ros-args -p planner:=offline
```

**상세 설명:** `task_manager`는 자연어 명령을 기다렸다가 계획·검증·단계별 실행을 관리하는 프로그램입니다. `--ros-args -p planner:=offline`은 관리자 노드의 `planner` 파라미터를 규칙 기반으로 설정합니다. 앞 절 계획기의 `--offline`과 목적은 비슷하지만 서로 다른 프로그램이므로 문법이 다릅니다.

**이 한 줄이 하는 일:** `/task/command`로 명령이 오면 실제 주행과 팔 동작을 실행할 준비를 합니다. 준비 로그가 나올 때까지 기다립니다.

### T4 — 진행 상태 관찰

```bash
ros2 topic echo /task/status --field data
```

**상세 설명:** `/task/status`는 관리자가 보내는 상태 토픽입니다. `--field data`는 메시지 전체 대신 `data` 필드만 출력합니다. 그 안에는 JSON 형태의 상태 문자열이 들어 있습니다. `--once`가 없으므로 계속 출력합니다.

**이 한 줄이 하는 일:** 대기 중(`idle`), 계획 중(`planning`), 실행 중(`running`), 완료(`done`), 실패(`failed`) 등을 관찰합니다. 명령을 보내거나 로봇을 움직이지는 않습니다. 이 터미널은 유지하세요.

### T5 — 자연어 명령 한 번 보내기

```bash
ros2 topic pub --once /task/command std_msgs/msg/String "{data: '빨간 상자를 왼쪽 작업대로 옮기고 돌아와'}"
```

**상세 설명**

- `ros2 topic pub`: 토픽으로 메시지를 보냅니다.
- `--once`: 같은 요청을 반복하지 않고 한 번만 발행합니다.
- `/task/command`: 명령을 받을 토픽 이름입니다.
- `std_msgs/msg/String`: 메시지 타입입니다. 문자열을 담는 `data` 필드 하나를 갖습니다.
- `"{data: '...'}"`: 메시지 내용을 적은 YAML입니다. `data`가 필드 이름, 작은따옴표 안 문장이 값입니다. 바깥 큰따옴표는 전체를 셸 입력 하나로 묶습니다.

**이 한 줄이 하는 일:** Task Manager에 실제 운반 작업 시작을 요청합니다. 발행 명령이 끝나도 작업은 관리자가 계속 실행합니다.

**확인:** T4에서 단계가 바뀌고, 최종 `done`과 실제 상자 위치·로봇 복귀를 확인합니다. 실패했다면 어느 단계에서 무슨 이유로 실패했는지 기록합니다.

### 필요할 때 — 취소 요청

```bash
ros2 service call /task/cancel std_srvs/srv/Trigger
```

**상세 설명:** `ros2 service call`은 한 번 요청하고 응답을 받습니다. `/task/cancel`은 취소 서비스 이름, `std_srvs/srv/Trigger`는 요청 필드가 없는 서비스 타입입니다. 따라서 뒤에 추가 입력값이 없습니다.

**이 한 줄이 하는 일:** 진행 중인 작업에 취소를 요청합니다. **현재 단계 사이에서 적용되며 비상정지가 아닙니다.** 응답이 왔다고 현재 주행이나 팔이 즉시 멈췄다고 판단하지 마세요.

## 6. 하나의 명령으로 모두 실행하고 웹에서 미션 보내기

**지금까지의 T1~T5 프로그램을 모두 종료**합니다. 아래 launch가 앞의 시뮬·Nav2·MoveIt·관리자를 포함하므로 중복 실행하면 안 됩니다.

### T1 — 전체 시스템 시작

```bash
ros2 launch mobile_openarm_apps full_system.launch.py planner:=offline world:=world_light.xml rviz:=false
```

**상세 설명**

- `mobile_openarm_apps`: 통합 실행 파일이 들어 있는 패키지입니다.
- `full_system.launch.py`: 시뮬·Nav2·MoveIt·Task Manager·웹 대시보드를 함께 시작하는 launch 파일입니다.
- `planner:=offline`: launch 설정값으로 규칙 기반 계획기를 선택하여 관리자에 전달합니다. 여기서는 `--ros-args -p`가 아니라 launch의 `이름:=값` 문법을 씁니다.
- `world:=world_light.xml`: 하위 시뮬이 가벼운 월드를 사용하게 합니다.
- `rviz:=false`: 주행 RViz를 열지 않습니다. 이 통합 launch는 MoveIt RViz도 끕니다. MuJoCo 뷰어는 기본으로 켜집니다.

**이 한 줄이 하는 일:** 앞에서 여러 터미널로 준비했던 기능과 웹 화면을 한꺼번에 시작합니다. 아직 미션은 시작하지 않습니다.

**다음 행동:** 브라우저에서 **http://localhost:8080**을 열고, Task Manager 준비 로그와 대기 상태를 확인한 뒤 빨간 상자 운반 명령을 입력합니다. 웹이 12초 뒤 시작되도록 되어 있지만 PC에 따라 전체 초기화는 더 걸릴 수 있습니다.

iMac에서 뷰어가 느리다면 위 실행을 종료한 뒤 **대신** 다음을 실행합니다.

```bash
ros2 launch mobile_openarm_apps full_system.launch.py planner:=offline world:=world_light.xml rviz:=false viewer:=false
```

**상세 설명:** 이전 명령에 `viewer:=false`만 추가했습니다. MuJoCo 별도 창을 끄는 옵션입니다. 시뮬과 대시보드 카메라는 계속 동작합니다.

**이 한 줄이 하는 일:** 같은 전체 시스템을 화면 부하가 적은 방식으로 실행합니다. 두 통합 명령을 동시에 실행하지 마세요.

## 오늘 실행한 기능은 소스에서 어디에 있는가?

| 기능 | 주요 소스 | 역할 |
|---|---|---|
| 팔 계획 구성 | `src/mobile_openarm_moveit_config/launch/move_group.launch.py`, `config/` | MoveIt 서버, 팔 그룹·자세·계획기·컨트롤러 설정 |
| 로봇 기능 | `src/mobile_openarm_apps/mobile_openarm_apps/robot_skills.py` | 이동, 도킹, 팔·그리퍼, 집기·놓기 구현 |
| 개별 시험 앱 | 같은 폴더의 `arm_demo.py`, `pick_place.py`, `nav_pick_place.py` | 위 기능을 실습 순서대로 호출 |
| 자연어 계획 | 같은 폴더의 `llm_planner.py` | 입력 문장으로 계획 생성·검증 |
| 실제 실행 관리 | 같은 폴더의 `task_manager.py` | 명령 수신, 계획 호출, 기능 실행, 상태 보고 |
| 웹 | 같은 폴더의 `dashboard.py`, 패키지 `web/index.html` | 명령 입력·상태·지도·카메라 표시 |
| 통합 구성 | `src/mobile_openarm_apps/launch/full_system.launch.py` | 필요한 프로그램을 함께 실행 |

전체 흐름은 **자연어 → 계획기 → 계획 검증 → 작업 관리자 → 로봇 기능 → Nav2/MoveIt → 브리지 → MuJoCo**입니다. 계획기는 바퀴·팔 관절을 직접 제어하지 않습니다.

**완료 기준:** 팔 동작, 같은 작업대 집기, 작업대 간 운반, 계획만 출력하기, 자연어로 실제 실행하기를 구분할 수 있고 웹에서 결과까지 확인했습니다.

API·추가 옵션·문제 해결이 필요할 때만 [기존 상세 실습서](../labs/README.md)를 참고하세요. 실제 LLM 모드에서는 API 실패 시 offline 대체가 가능하므로 사용된 계획기를 로그로 확인해야 합니다.
