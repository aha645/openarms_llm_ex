# 휠 + OpenArm — 교육 2일차

[README](../../README.md) · [1일차](wheel-day1.md)

## 전제·터미널 규칙

Ubuntu 24.04 / ROS 2 Jazzy / 소스·에셋·빌드 완료를 전제합니다. 각 터미널에서 저장소 루트로 이동하고 다음을 실행합니다. Docker에서는 컨테이너 내부 `/root/openarm_mobile_ws`를 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset OPENARM_BIPED OPENARM_SRDF
```

**준비 명령의 목적**

| 명령 | 설명 |
|---|---|
| `source /opt/ros/jazzy/setup.bash` | 현재 셸에 기본 ROS 2 Jazzy 실행·패키지 검색 환경을 등록합니다. 설치나 노드 실행은 하지 않습니다. |
| `source install/setup.bash` | 현재 저장소에서 빌드한 패키지의 overlay 환경을 등록합니다. 저장소 루트 기준 상대 경로이므로 먼저 해당 폴더로 이동해야 합니다. |
| `unset OPENARM_BIPED OPENARM_SRDF` | 이전 이족 실습의 모드와 SRDF 강제 선택을 해제합니다. 휠형 기본 자세·설정을 선택하려는 것이며 파일 삭제가 아닙니다. |

### 명령과 옵션을 구분해서 읽기

- `ros2 launch 패키지 파일.launch.py 이름:=값`: 여러 노드를 묶어 실행합니다. `이름:=값`은 해당 launch가 선언한 **launch 인자**입니다.
- `ros2 run 패키지 실행이름 --ros-args -p 이름:=값`: 실행 프로그램 하나를 시작하고 **ROS 노드 파라미터**를 지정합니다. `-p`는 각 파라미터 앞에 붙입니다.
- `--offline`, `--once`, `--field` 등은 각 프로그램/하위 명령의 옵션입니다. 모든 ROS 명령에서 공통으로 사용할 수 있는 옵션은 아닙니다.
- 줄 끝 `\`는 다음 줄까지 이어지는 한 명령입니다. `#`으로 시작한 줄은 주석이며 실행되지 않습니다.
- `world:=world_light.xml`은 시각 메시를 줄인 물리 월드 선택, `rviz:=false`는 RViz 창 끄기, `viewer:=false`는 MuJoCo 뷰어 창 끄기입니다. 뷰어를 꺼도 물리 시뮬·카메라·ROS 연결은 계속 동작합니다.

T1/T2/T3는 별도 터미널입니다. 각 절 전환 시 앞 절의 노드들을 Ctrl+C로 종료합니다. 단, ‘유지’라고 표시한 경우만 유지합니다. full_system과 수동 sim/nav/move_group/task_manager는 중복 실행하지 않습니다.

## 소스 구조: 오늘의 실행 계층

| 계층 | 소스 (`src/` 기준) | 역할 |
|---|---|---|
| 팔 계획 설정 | `mobile_openarm_moveit_config/launch/move_group.launch.py`, `config/` | SRDF 그룹·이름 자세, KDL IK, OMPL, 컨트롤러 |
| 기능 계층 | `mobile_openarm_apps/mobile_openarm_apps/robot_skills.py` | Nav2 이동, 카메라 기반 도킹, MoveIt 팔·그리퍼, 집기·놓기 |
| 개별 앱 | 같은 폴더의 `arm_demo.py`, `pick_place.py`, `nav_pick_place.py` | 기능별 시험 시나리오 |
| 계획 | 같은 폴더의 `llm_planner.py` | offline/API 계획, Pydantic 스키마, 전제조건 검증 |
| 실행 관리 | 같은 폴더의 `task_manager.py` | 명령 수신, 단계 실행, 상태·로그·취소 |
| 웹 | 같은 폴더의 `dashboard.py`, 패키지 `web/index.html` | 상태·카메라 표시와 명령 입력 |
| 통합 | `mobile_openarm_apps/launch/full_system.launch.py` | Nav2 포함 시뮬 + MoveIt + Task Manager + 웹 |

## 2-01. MoveIt 양팔 동작

T1:

```bash
ros2 launch mobile_openarm_sim sim.launch.py world:=world_light.xml rviz:=false
```

T2:

```bash
ros2 launch mobile_openarm_moveit_config move_group.launch.py
```

T3 (move_group 준비 로그 후):

```bash
ros2 run mobile_openarm_apps arm_demo
```

**명령 한 줄씩 해설**

| 명령 | 실행 목적과 옵션 의미 |
|---|---|
| `ros2 launch mobile_openarm_sim sim.launch.py world:=world_light.xml rviz:=false` | light 월드의 시뮬·브리지·TF를 띄워 팔 명령을 실행할 로봇을 준비합니다. `rviz:=false`는 T2에서 켤 RViz와 중복되는 것을 피합니다. MuJoCo 뷰어는 기본값으로 켜집니다. |
| `ros2 launch mobile_openarm_moveit_config move_group.launch.py` | 휠형 MoveIt 설정으로 `move_group`과 기본 RViz를 시작합니다. IK·충돌 검사·경로 계획과 실행 연결을 준비하는 명령이며, 이 한 줄만으로 데모 동작을 시작하지는 않습니다. |
| `ros2 run mobile_openarm_apps arm_demo` | 준비된 MoveIt과 브리지를 이용해 데모에 정의된 이름 자세·손끝 목표를 순서대로 시험합니다. `mobile_openarm_apps`는 패키지, `arm_demo`는 등록된 실행이름이며 별도 인자는 없습니다. |

T1과 T2는 계속 실행해 두고 T3를 실행합니다. T3의 데모가 종료되어도 시뮬과 MoveIt은 계속 켜져 있습니다.

**테스트:** 이름 자세와 손끝 목표에 대한 계획·실행을 확인합니다. RViz MotionPlanning과 실제 시뮬의 팔 움직임을 비교합니다. ‘계획 성공’과 ‘실행 성공’을 구분합니다.

**소스:** `arm_demo.py → robot_skills.py → MoveIt → 브리지의 FollowJointTrajectory`. SRDF는 양팔과 그리퍼 그룹·자세, YAML은 IK·계획기·컨트롤러를 정의합니다.

상세: [MoveIt 실습](../labs/day2-01-moveit.md).

## 2-02. 카메라·ArUco 집기/놓기

앞 절 노드 종료 후 T1:

```bash
ros2 launch mobile_openarm_sim sim.launch.py world:=world_light.xml rviz:=false \
  start_x:=2.5 start_y:=2.45 start_yaw:=1.5708
```

T2:

```bash
ros2 launch mobile_openarm_moveit_config move_group.launch.py rviz:=false
```

T3:

```bash
ros2 topic echo /aruco/detections --once
ros2 run mobile_openarm_apps pick_place --ros-args -p object:=box_0
```

**명령 한 줄씩 해설**

| 명령/옵션 | 목적과 의미 |
|---|---|
| `ros2 launch mobile_openarm_sim sim.launch.py ... start_x:=2.5 start_y:=2.45 start_yaw:=1.5708` | 주행 단계를 생략하고 작업대 앞에서 바로 집기를 시험하도록 로봇 시작 위치·방향을 지정합니다. Nav2까지 실행하는 명령은 아닙니다. |
| `world:=world_light.xml`, `rviz:=false` | light 월드를 선택하고 sim 측 RViz를 끕니다. MuJoCo 뷰어와 카메라는 유지됩니다. |
| `start_x:=2.5`, `start_y:=2.45` | 월드 좌표계에서 초기 로봇 위치를 x=2.5 m, y=2.45 m로 설정합니다. +x는 동쪽, +y는 북쪽입니다. 물체 좌표나 손끝 목표가 아닙니다. |
| `start_yaw:=1.5708` | 초기 수평 회전각을 약 π/2 rad, 즉 +90°로 설정합니다. +x를 향한 방향에서 +y 방향으로 돌린 자세입니다. 단위는 도(degree)가 아닌 rad입니다. |
| `ros2 launch mobile_openarm_moveit_config move_group.launch.py rviz:=false` | 팔 계획·실행 서버는 켜되 MoveIt 측 RViz도 끕니다. 창 없이도 T3의 프로그램이 계획·실행을 요청할 수 있습니다. |
| `ros2 topic echo /aruco/detections --once` | 마커 검출 메시지를 하나 받아 내용과 검출된 물체를 확인한 뒤 종료합니다. `--once`는 물체 하나만 검출한다는 뜻이 아닙니다. 검출 메시지가 비어 있다면 수신 성공만으로 상자를 인식했다고 보지 않습니다. |
| `ros2 run mobile_openarm_apps pick_place --ros-args -p object:=box_0` | 로봇이 작업대 앞에 있다는 전제로 상자를 집어 **같은 작업대의 다른 위치**에 놓습니다. 두 작업대 사이 운반은 다음 절입니다. |
| `--ros-args -p object:=box_0` | 앱의 ROS 파라미터 `object`를 빨간 상자 식별자 `box_0`로 지정합니다. `box_1`은 초록, `box_2`는 파란 상자입니다. 임의의 물체 이름은 사용할 수 없습니다. |

`pick_place` 앱의 기본 시작 좌표도 위 sim 좌표와 같습니다. 시작 좌표를 바꿀 때는 앱의 `start_x/start_y/start_yaw` 파라미터도 맞춰야 합니다. 이 앱은 Nav2 없이 사용할 `map→odom` 정적 변환을 시작 좌표로 준비하기 때문입니다.

**테스트:** 작업대 앞에서 빨간 상자를 인식하고, 접근·그리퍼 닫기·들어올리기·놓기를 관찰합니다. `/aruco/detections` 수신과 실제 상자 이동을 모두 확인합니다. 다른 색 실습 전에는 월드를 재시작해 초기 조건을 맞춥니다.

**소스:** 시뮬 패키지의 `aruco_utils.py`, `bridge_node.py`, 앱의 `pick_place.py`, `robot_skills.py`. 기본 `grasp_assist:=true`는 교육용 파지 보조입니다. 성공을 현실 접촉 파지 성능으로 해석하지 않습니다. 실로봇용 `aruco_detector.py`를 시뮬에서 중복 실행하지 않습니다.

상세: [ArUco·파지 보조·집기 과정](../labs/day2-02-aruco-pick-place.md).

## 2-03. 주행 + 운반

앞 절 노드 종료 후 T1 (시작 좌표는 home 기본값 유지):

```bash
ros2 launch mobile_openarm_sim nav.launch.py world:=world_light.xml rviz:=false
```

T2:

```bash
ros2 launch mobile_openarm_moveit_config move_group.launch.py rviz:=false
```

T3:

```bash
ros2 run mobile_openarm_apps nav_pick_place --ros-args -p object:=box_0
```

**명령 한 줄씩 해설**

| 명령 | 실행 목적과 옵션 의미 |
|---|---|
| `ros2 launch mobile_openarm_sim nav.launch.py world:=world_light.xml rviz:=false` | home에서 시뮬·기본 지도·AMCL·Nav2를 함께 시작하여 작업대까지 스스로 이동할 준비를 합니다. `world`는 시뮬 월드 선택이고 `rviz:=false`는 주행 RViz만 끕니다. 기본 지도 파일을 바꾸는 옵션은 `map:=...`입니다. |
| `ros2 launch mobile_openarm_moveit_config move_group.launch.py rviz:=false` | 주행과 별도로 팔 계획·실행 기능을 준비합니다. RViz를 끄더라도 계획 서버는 동작합니다. |
| `ros2 run mobile_openarm_apps nav_pick_place --ros-args -p object:=box_0` | 준비된 Nav2와 MoveIt을 함께 사용하여 빨간 상자 운반 시나리오를 실행합니다. 앱 실행이름 `nav_pick_place`가 앞 절의 `pick_place`와 다르다는 점이 중요합니다. |
| `--ros-args -p object:=box_0` | 운반 대상 물체를 선택합니다. 작업대 위치나 목적지 좌표를 직접 입력하는 옵션은 아니며 앱 기본 시나리오와 위치 설정을 사용합니다. |

앞 절의 `start_x/start_y/start_yaw`를 여기에 그대로 복사하지 않습니다. 이 단계는 home 시작·지도 위치 추정·작업대까지 이동하는 전체 흐름을 시험합니다.

**테스트:** 오른쪽 작업대 이동 → 집기 → 왼쪽 작업대 이동 → 놓기 → 복귀. 주행 도착 후 카메라로 다시 도킹하는 이유를 설명해 봅니다. 명령 종료만 보지 말고 상자가 목적 작업대에 놓였는지 확인합니다.

**소스:** `nav_pick_place.py → robot_skills.py`; 위치는 `mobile_openarm_sim/config/locations.yaml`의 이름으로 관리합니다.

상세: [이동 조작](../labs/day2-03-nav-pick-place.md).

## 2-04. 자연어 계획만 단독 시험

시뮬을 종료해도 가능한 실습입니다.

```bash
ros2 run mobile_openarm_apps llm_planner --offline \
  "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
ros2 run mobile_openarm_apps llm_planner --offline "커피 한 잔 타줘"
```

**명령 한 줄씩 해설**

| 명령/구성 | 목적과 의미 |
|---|---|
| `ros2 run mobile_openarm_apps llm_planner --offline "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"` | 자연어를 실행 단계 목록으로 바꾸고 검증한 결과를 터미널에 출력합니다. ROS 명령으로 실행하지만 이 계획기 자체는 로봇 제어 노드가 아니며 시뮬을 움직이지 않습니다. |
| `--offline` | 계획기 프로그램의 옵션으로, API를 호출하지 않고 키워드 규칙으로 계획합니다. `--ros-args -p planner:=offline`과 다른 문법입니다. |
| `"...자연어..."` | 계획기 입력인 위치 인자 하나입니다. 공백을 포함한 한국어 문장을 한 인자로 전달하기 위해 큰따옴표로 감쌉니다. |
| `ros2 run mobile_openarm_apps llm_planner --offline "커피 한 잔 타줘"` | 구현된 행동 목록 밖의 요청을 거절하는지 시험합니다. 로봇을 실행하지 않고 계획 생성·검증의 한계를 확인하려는 명령입니다. |

**확인 기준:** 첫 명령은 이동·집기·이동·놓기·복귀 계획, 두 번째는 수행 불가로 거부됩니다. 단독 CLI는 검증 실패 시 Traceback으로 끝날 수 있으며 로봇을 움직이지 않습니다.

**소스:** `llm_planner.py`의 `RobotPlan`, `Step`, `plan_offline()`, `validate_plan()`. 행동/장소/물체를 제한하고 집기·놓기 전제조건을 검증합니다. 규칙 기반은 자연어 의미 전체를 이해하지 않습니다. ‘상자 모두 옮겨’는 기본 12단계 제한을 초과할 수 있습니다.

### 선택: 실제 LLM API

```bash
export OPENAI_API_KEY='자신의_API_키'
export OPENAI_MODEL='gpt-4o-mini'
ros2 run mobile_openarm_apps llm_planner "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
```

**각 줄의 목적**

| 명령 | 의미 |
|---|---|
| `export OPENAI_API_KEY='자신의_API_키'` | 현재 셸과 이후 시작할 자식 프로세스에 API 인증 정보를 전달합니다. 따옴표 안 예시를 실제 키로 바꾸되 문서·코드·대화에 키를 기록하지 않습니다. 다른 터미널에는 자동 적용되지 않습니다. |
| `export OPENAI_MODEL='gpt-4o-mini'` | 사용할 API 모델 이름을 지정합니다. 계정에서 해당 모델을 사용할 수 있어야 하며 요청에 따라 비용이 발생할 수 있습니다. |
| `ros2 run mobile_openarm_apps llm_planner "..."` | `--offline`을 생략하여 실제 API로 계획을 생성합니다. 이 명령도 계획 출력만 하며 로봇 미션 실행은 하지 않습니다. |

OpenAI 호환 로컬 서버는 `OPENAI_BASE_URL`, `OPENAI_MODEL`, `OPENAI_API_KEY`를 지정합니다. 키를 소스에 기록하지 마세요. 자세한 설정·Structured Outputs 제약은 [LLM 상세 문서](../labs/day2-04-llm-planner.md)를 참고합니다.

## 2-05. Task Manager의 단계 실행과 상태

2-03의 nav + MoveIt을 새로 시작하거나, 종료하지 않았다면 초기 상자 배치를 확인합니다. T3:

```bash
ros2 run mobile_openarm_apps task_manager --ros-args -p planner:=offline
```

T4:

```bash
ros2 topic echo /task/status --field data
```

T5:

```bash
ros2 topic pub --once /task/command std_msgs/msg/String \
  "{data: '빨간 상자를 왼쪽 작업대로 옮기고 돌아와'}"
```

**명령 한 줄씩 해설**

| 명령/구성 | 목적과 옵션 의미 |
|---|---|
| `ros2 run mobile_openarm_apps task_manager --ros-args -p planner:=offline` | 자연어를 받아 계획·검증 후 실제 로봇 기능을 순서대로 호출하는 관리 노드를 시작합니다. 단독 계획기와 달리 Nav2·MoveIt이 실행 중이어야 동작을 수행할 수 있습니다. |
| `--ros-args -p planner:=offline` | Task Manager의 ROS 파라미터 `planner`를 규칙 기반으로 설정합니다. 입력 문장을 명령줄 인자로 바로 넘기는 것이 아니라 `/task/command`로 기다립니다. |
| `ros2 topic echo /task/status --field data` | 상태 토픽을 계속 구독하며 메시지의 `data` 필드만 출력합니다. `--field data`는 문자열 안의 JSON을 별도로 해석하는 옵션은 아니며 메시지 바깥 구조를 생략합니다. Ctrl+C로 모니터만 종료할 수 있습니다. |
| `ros2 topic pub --once /task/command std_msgs/msg/String "{data: '...'}"` | 자연어 명령 문자열을 Task Manager에 한 번 발행합니다. 이 한 줄이 실제 작업 시작을 요청합니다. 발행 프로그램이 끝나도 관리 노드의 작업은 계속될 수 있습니다. |
| `--once` | 기본 반복 발행 대신 메시지 한 번만 보냅니다. 동일 미션을 반복 요청하지 않도록 합니다. |
| `/task/command` | 명령을 보낼 토픽 이름입니다. 상태 출력용 `/task/status`와 다릅니다. |
| `std_msgs/msg/String` | 보낼 ROS 메시지 타입입니다. 문자열 필드 하나인 `data`를 갖습니다. |
| `"{data: '...'}"` | 메시지 필드를 지정하는 YAML입니다. 바깥 큰따옴표는 셸 인자 전체를 묶고 안쪽 작은따옴표는 YAML 문자열을 묶습니다. 명령 문장만 바꾸고 구조는 유지합니다. |

T3의 준비 로그와 T4의 `idle`을 확인한 뒤 T5를 실행합니다. T4는 상태를 보는 터미널이고 T5는 명령을 보내는 터미널이므로 역할이 다릅니다.

**확인 기준:** `idle → planning → running → done` 또는 명확한 `failed/rejected` 이유와 단계 로그. `task_manager.py`가 계획 검증 후 `robot_skills.py`의 기능을 순서대로 부릅니다.

취소 실습:

```bash
ros2 service call /task/cancel std_srvs/srv/Trigger
```

**취소 명령 해설:** `ros2 service call`은 지속 발행 토픽 대신 한 번의 요청·응답을 수행합니다. `/task/cancel`은 서비스 이름, `std_srvs/srv/Trigger`는 입력 필드가 없는 서비스 타입입니다. 따라서 여기서는 추가 요청 YAML 없이 호출할 수 있습니다. 응답을 받았다고 현재 주행·팔 동작이 즉시 멈췄다는 의미는 아닙니다.

취소는 **현재 단계 사이에서 적용되는 요청이며 비상정지가 아닙니다.** 실행 중 새 명령은 무시됩니다. 재실습은 전체 종료·재시작으로 상자와 위치를 초기화하는 것이 가장 단순합니다.

상세: [Task Manager 인터페이스·상태](../labs/day2-05-task-manager.md).

## 2-06. full_system과 웹으로 최종 통합

**기존 수동 launch와 앱을 전부 종료**합니다. T1:

```bash
ros2 launch mobile_openarm_apps full_system.launch.py \
  planner:=offline world:=world_light.xml rviz:=false
# iMac에서 뷰어 부하가 크면 viewer:=false 추가
```

**통합 명령 한 줄 해설**

| 구성 요소 | 목적과 의미 |
|---|---|
| `ros2 launch mobile_openarm_apps full_system.launch.py` | 앞 절에서 별도 터미널로 띄운 시뮬·Nav2·MoveIt·Task Manager·웹을 하나의 launch로 묶어 시작합니다. 실행 직후 자동으로 상자를 옮기는 것이 아니라 사용자 명령을 받을 준비를 합니다. |
| `planner:=offline` | 통합 launch 인자로 규칙 기반 계획기를 선택하고 Task Manager 파라미터로 전달합니다. API 키 없이 교육을 진행하기 위한 값입니다. |
| `world:=world_light.xml` | 하위 Nav2/sim launch에 light 물리 월드 선택을 전달합니다. 지도 파일이나 LLM 모델을 바꾸는 설정이 아닙니다. |
| `rviz:=false` | 하위 주행 RViz를 띄우지 않습니다. 통합 launch는 MoveIt RViz도 끈 상태로 포함합니다. 웹·MuJoCo 뷰어로 관찰할 수 있습니다. |
| 선택 옵션 `viewer:=false` | MuJoCo 별도 창을 끄고 대시보드 카메라 중심으로 관찰합니다. iMac의 CPU 렌더 부하를 줄이려는 옵션이며 head camera 발행까지 끄지는 않습니다. |
| 대체 값 `planner:=llm` | 규칙 대신 API 계획을 요청합니다. 같은 터미널에 API 환경 변수를 설정하고 기존 launch 종료 후 재실행합니다. |

예를 들어 뷰어까지 끄는 명령은 다음과 같습니다. 위 기본 명령과 **동시에 실행하지 말고 하나만 선택**하세요.

```bash
ros2 launch mobile_openarm_apps full_system.launch.py \
  planner:=offline world:=world_light.xml rviz:=false viewer:=false
```

API 통합 실행도 명령으로 쓰면 다음과 같습니다. 2-04에서 설명한 `export` 설정을 같은 터미널에 적용한 뒤 실행합니다.

```bash
ros2 launch mobile_openarm_apps full_system.launch.py \
  planner:=llm world:=world_light.xml rviz:=false
```

두 예시 모두 기존과 같은 전체 시스템을 시작하며, 각각 뷰어 사용 여부 또는 계획기만 다릅니다. `camera_hz` 등 sim 전용 인자가 이 상위 launch에서도 전달된다고 가정하지 마세요.

브라우저 **http://localhost:8080**에서 같은 빨간 상자 운반 명령을 입력합니다. 전체 초기화 후 실행하고, 단계별 상태·카메라·실제 상자 위치를 확인합니다.

실제 LLM 통합은 이 launch를 종료하고, API 변수를 설정한 **동일 터미널**에서 `planner:=llm`으로 재실행합니다. Task Manager는 기본적으로 API 실패 시 offline 대체가 가능하므로 로그에서 실제 계획기를 확인합니다. Docker의 API 설정 전달 방법은 [기존 LLM 상세 문서](../labs/day2-04-llm-planner.md)를 참고합니다.

**소스:** full_system은 nav launch를 포함하고 5초 뒤 MoveIt, 12초 뒤 Task Manager·dashboard를 올립니다. 이 지연은 readiness 검사나 성공 보장이 아닙니다.

상세: [대시보드](../labs/day2-06-dashboard.md).

## 2일차 완료 체크

- [ ] 팔 계획·실행과 실제 상자 이동을 확인했다.
- [ ] 주행과 조작을 결합한 운반을 실행했다.
- [ ] offline과 LLM API를 구분하고 계획 검증 거부를 확인했다.
- [ ] Task Manager의 단계 상태·실패·취소 제한을 이해했다.
- [ ] full_system을 중복 노드 없이 실행하고 결과·로그를 기록했다.
