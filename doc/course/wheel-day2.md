# 휠 + OpenArm — 교육 2일차

[README](../../README.md) · [1일차](wheel-day1.md)

## 전제·터미널 규칙

Ubuntu 24.04 / ROS 2 Jazzy / 소스·에셋·빌드 완료를 전제합니다. 각 터미널에서 저장소 루트로 이동하고 다음을 실행합니다. Docker에서는 컨테이너 내부 `/root/openarm_mobile_ws`를 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset OPENARM_BIPED OPENARM_SRDF
```

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

**확인 기준:** 첫 명령은 이동·집기·이동·놓기·복귀 계획, 두 번째는 수행 불가로 거부됩니다. 단독 CLI는 검증 실패 시 Traceback으로 끝날 수 있으며 로봇을 움직이지 않습니다.

**소스:** `llm_planner.py`의 `RobotPlan`, `Step`, `plan_offline()`, `validate_plan()`. 행동/장소/물체를 제한하고 집기·놓기 전제조건을 검증합니다. 규칙 기반은 자연어 의미 전체를 이해하지 않습니다. ‘상자 모두 옮겨’는 기본 12단계 제한을 초과할 수 있습니다.

### 선택: 실제 LLM API

```bash
export OPENAI_API_KEY='자신의_API_키'
export OPENAI_MODEL='gpt-4o-mini'
ros2 run mobile_openarm_apps llm_planner "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
```

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

**확인 기준:** `idle → planning → running → done` 또는 명확한 `failed/rejected` 이유와 단계 로그. `task_manager.py`가 계획 검증 후 `robot_skills.py`의 기능을 순서대로 부릅니다.

취소 실습:

```bash
ros2 service call /task/cancel std_srvs/srv/Trigger
```

취소는 **현재 단계 사이에서 적용되는 요청이며 비상정지가 아닙니다.** 실행 중 새 명령은 무시됩니다. 재실습은 전체 종료·재시작으로 상자와 위치를 초기화하는 것이 가장 단순합니다.

상세: [Task Manager 인터페이스·상태](../labs/day2-05-task-manager.md).

## 2-06. full_system과 웹으로 최종 통합

**기존 수동 launch와 앱을 전부 종료**합니다. T1:

```bash
ros2 launch mobile_openarm_apps full_system.launch.py \
  planner:=offline world:=world_light.xml rviz:=false
# iMac에서 뷰어 부하가 크면 viewer:=false 추가
```

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
