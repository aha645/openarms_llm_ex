# TRON1 + OpenArm — 교육 2일차

[TRON1 목차](README.md) · [1일차 및 추가 빌드 전제](day1.md) · [용어집](../glossary.md)

## 전제·오늘의 목표

Ubuntu 24.04 / ROS 2 Jazzy / 휠형·TRON1 에셋 생성과 두 워크스페이스(`mobile_openarm/`, 그 위에 overlay한 `tron1_openarm/`)의 빌드 완료를 전제로 합니다.

기준은 **Sole-foot(SF)**입니다. launch 기본값은 PF이므로 `foot:=sf`를 명시합니다. 공통 앱·LLM 원리는 [휠형 2일차](../wheel/README.md)와 같지만 아래에 이족형 실행 명령을 따로 제공합니다. 기존 SF 완주는 1회 기록이며 실제 노트북·iMac의 반복 성공을 보장하지 않습니다.

각 새 터미널에서 저장소 루트로 이동 후:

```bash
source /opt/ros/jazzy/setup.bash
source tron1_openarm/install/setup.bash         # 휠형 mobile_openarm underlay 포함
source mobile_openarm/.venv-mobile/bin/activate # python3 를 직접 실행할 때 필요
export OPENARM_BIPED=1
export OPENARM_SRDF="$(ros2 pkg prefix --share tron1_openarm_moveit_config)/config/tron1_openarm_sf.srdf"
```

Docker에서는 컨테이너 내부 `/root/openarm_mobile_ws`에서 실행합니다. 수동 앱을 실행하는 터미널에도 위 환경 변수가 필요합니다. `full_system.launch.py`는 발 형태에 맞춰 자동 설정합니다. PF로 비교할 때 SRDF는 `tron1_openarm.srdf`로 바꿔야 합니다.

각 절 사이 기존 launch·앱을 Ctrl+C로 종료합니다. full_system과 수동 nav/sim/move_group/task_manager를 동시에 실행하지 않습니다.

## 소스 구성과 이족 전용 동작

| 소스 | 역할 |
|---|---|
| `tron1_openarm/src/tron1_openarm_moveit_config/` | 다리 포함 URDF/SRDF, 이족 몸통 충돌을 피하는 home, PF/SF MoveIt 설정 |
| `mobile_openarm/src/mobile_openarm_apps/mobile_openarm_apps/robot_skills.py` | 공통 기능 + `OPENARM_BIPED`에 따른 옆걸음·재측정·시각 도킹/보정 |
| 같은 폴더의 `arm_demo.py`, `pick_place.py`, `nav_pick_place.py` | 휠형과 같은 시험 앱 |
| 같은 폴더의 `llm_planner.py`, `task_manager.py` | 하반신과 무관한 계획·검증·순서 실행 |
| `tron1_openarm/src/tron1_openarm_sim/launch/full_system.launch.py` | TRON1 Nav2 + TRON1 MoveIt + 공통 Task Manager/웹, PF/SF 환경 설정 |
| `tron1_openarm/tools/validation/` | 정답 기록·집기/미션 시험 도구 (선택적 검증) |

다리 자체는 MoveIt 팔 계획 그룹이 아닙니다. 보행 정책이 균형을 유지하고, MoveIt은 팔 경로를 계획합니다. 팔 이동에 따른 몸통 반동·표류가 집기 오차를 만들 수 있습니다.

## 2-01. MoveIt 팔 계획·실행

T1:

```bash
ros2 launch tron1_openarm_sim sim.launch.py foot:=sf world:=world_light.xml rviz:=false
```

T2:

```bash
ros2 launch tron1_openarm_moveit_config move_group.launch.py foot:=sf
```

T3 (공통 환경 변수 설정 후):

```bash
ros2 run mobile_openarm_apps arm_demo
```

**테스트:** 이름 자세와 손끝 목표 실행, 팔–몸통/다리 충돌, 팔 이동 중 몸통 반응을 관찰합니다. 충돌 오류를 무조건 검사 비활성화로 우회하지 말고 SRDF·발 형태·현재 자세를 먼저 확인합니다.

**소스:** 공통 `arm_demo.py`/`robot_skills.py` + TRON1 MoveIt의 `config/tron1_openarm_sf.srdf`. 휠형 home을 그대로 쓰지 않는 이유를 설명해 봅니다.

**휠형과 다른 점** (통합 전 PF 기준 기록. 수치는 SF 와 다를 수 있음)

- `home` 자세가 바퀴형과 조금 다릅니다: 팔을 내릴 때 어깨 joint2 를 ±0.2 rad 벌려 손이 TRON1 몸통 상자 바깥으로 나옵니다. 그대로 내리면 손이 몸통과 겹쳐 MoveIt 이 `START_STATE_IN_COLLISION` 으로 거부합니다. TRON1 SRDF 에 반영되어 있고, 앱은 `OPENARM_SRDF` 환경변수로 이 SRDF 를 읽습니다 (`full_system.launch.py` 가 자동 설정. 따로 실행할 때는 `export OPENARM_SRDF=$(ros2 pkg prefix --share tron1_openarm_moveit_config)/config/tron1_openarm.srdf`).

```bash
ros2 launch tron1_openarm_sim sim.launch.py rviz:=false
ros2 launch tron1_openarm_moveit_config move_group.launch.py
ros2 run mobile_openarm_apps arm_demo
```

- RViz MotionPlanning 에 다리가 보입니다. 다리 관절은 계획 그룹에 없고, MoveIt 은 현재 다리 자세(`/joint_states`)로 **팔–다리 충돌**을 검사합니다.
- 팔을 크게 움직이면 몸통이 반동으로 조금 움직입니다(제자리 걸음으로 균형). `arm_demo` 의 `pose goal` 은 base_footprint 기준이라 그 사이 몸이 움직이면 목표가 1~2 cm 어긋날 수 있습니다.

## 2-02. 카메라 기반 집기·놓기

앞 절 노드 종료. T1:

```bash
ros2 launch tron1_openarm_sim sim.launch.py foot:=sf world:=world_light.xml rviz:=false \
  start_x:=2.5 start_y:=2.45 start_yaw:=1.5708
```

T2:

```bash
ros2 launch tron1_openarm_moveit_config move_group.launch.py foot:=sf rviz:=false
```

T3 (환경 변수 설정 후):

```bash
ros2 topic echo /aruco/detections --once
ros2 run mobile_openarm_apps pick_place --ros-args -p object:=box_0
```

**확인 기준:** 상자 마커 검출 → 방향/옆/앞뒤 도킹 → 접근·집기 → 상자 들림 → 놓기. 상태 로그 외에 실제 상자 높이·목적 위치·넘어짐 여부도 확인합니다.

**소스:** 공통 `robot_skills.py`의 이족 분기, 시뮬 `bridge_biped.py`와 정책 코어. 이족은 `/cmd_vel`의 `linear.y`로 옆걸음이 가능하지만 휠형은 그렇지 않습니다. 몸통 흔들림과 마커 가림 때문에 재측정·시각 피드백이 중요합니다.

**휠형과 다른 점** (통합 전 PF 기준 기록. 수치는 SF 와 다를 수 있음)

```bash
ros2 launch tron1_openarm_sim sim.launch.py start_x:=2.5 start_y:=2.45 start_yaw:=1.5708
ros2 launch tron1_openarm_moveit_config move_group.launch.py rviz:=false
export OPENARM_BIPED=1     # 옆걸음 + 접근 직전 재측정 켜기
export OPENARM_SRDF=$(ros2 pkg prefix --share tron1_openarm_moveit_config)/config/tron1_openarm.srdf
ros2 run mobile_openarm_apps pick_place --ros-args -p object:=box_0
```

- 머리 카메라는 960×720 입니다 (바퀴형 640×480). 카메라가 높아져 마커가 작게 보이기 때문입니다.

- 이족은 조작 중에도 제자리 걸음을 하므로 몸통이 조금씩 흐릅니다. `robot_skills.pick()` 은 그래서 원래부터 **집기 직전에 카메라로 다시 재고(find_marker 5회 평균), 상자가 0.52 m 앞에 오도록 미세 이동**합니다. 이족에서 이 단계가 더 중요해집니다.
- **옆걸음 (이족만):** 작업대 앞에서 상자가 옆으로 0.18 m 넘게 치우쳐 있으면 `[base] 옆걸음 -0.15 m` 처럼 옆으로 걸어 0.12 m 에 맞춥니다. 바퀴형은 옆으로 못 가므로 Nav2 로 다시 가야 하지만, 이족은 `/cmd_vel` 의 `linear.y` 로 바로 비켜 섭니다. 직접 해 보기: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.08}}"`
- 이족은 팔을 `carry` 로 올리는 동안에도 몸이 1~3 cm 움직이므로, 접근 직전에 상자를 **한 번 더 잽니다.**
- 실패해도 `파지 실패 → 다시 인식해서 재시도` 가 한 번 더 돕니다.

기본 파지 보조가 켜진 교육 시뮬입니다. 현실 로봇 파지 성능을 보장하지 않습니다. PF 드리프트와 시각 피드백 한계는 [시각 피드백 시험 기록](validation-log.md#5-시각-피드백-집기-손목-마커-visual-servoing)을 참고합니다.

## 2-03. 주행 + 집기 + 운반

앞 절 노드 종료. T1:

```bash
ros2 launch tron1_openarm_sim nav.launch.py foot:=sf world:=world_light.xml rviz:=false
```

T2:

```bash
ros2 launch tron1_openarm_moveit_config move_group.launch.py foot:=sf rviz:=false
```

T3:

```bash
ros2 run mobile_openarm_apps nav_pick_place --ros-args -p object:=box_0
```

**테스트:** home → 작업대 A → 집기 → 작업대 B → 놓기 → 복귀. Nav2의 느슨한 도착 오차를 카메라 도킹이 보완하는지, 운반 중 상자와 로봇이 안정적인지 관찰합니다.

**소스:** `nav_pick_place.py → robot_skills.py`, TRON1 Nav2 파라미터·정책·오도메트리. 실패는 이동 실패/마커 미검출/계획 실패/파지 실패/넘어짐으로 구분해 기록합니다. 재시험은 시뮬을 재시작해 상자 배치를 초기화합니다.

## 2-04. offline 계획과 선택적 LLM API

로봇을 종료해도 가능한 시험:

```bash
ros2 run mobile_openarm_apps llm_planner --offline \
  "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
ros2 run mobile_openarm_apps llm_planner --offline "커피 한 잔 타줘"
```

첫 명령은 계획 생성, 두 번째는 거부되어야 합니다. 단독 계획기는 로봇을 움직이지 않습니다. 하반신이 바뀌어도 가능한 행동·위치·물체 스키마는 같습니다.

실제 API 사용은 선택입니다.

```bash
export OPENAI_API_KEY='자신의_API_키'
export OPENAI_MODEL='gpt-4o-mini'
ros2 run mobile_openarm_apps llm_planner "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
```

**소스:** 공통 `llm_planner.py`. 로컬 서버·키 관리·구조화 출력은 [휠형 2-04 LLM 장](../wheel/day2-04-llm-planner.md)를 참고합니다.

## 2-05. Task Manager로 실행·상태 관찰

2-03의 TRON1 nav + MoveIt을 새로 시작하고, 초기 상자 배치를 확인합니다. T3 (이족/SRDF 환경 변수 필요):

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

**테스트:** planning → running → done/failed/rejected의 단계 상태와 실패 이유. 취소는 `ros2 service call /task/cancel std_srvs/srv/Trigger`로 요청하지만 현재 단계 사이에서 반영되므로 비상정지가 아닙니다.

**소스:** 공통 `task_manager.py`, `validate_plan()`, `robot_skills.py`. LLM은 다리 관절을 직접 제어하지 않습니다.

## 2-06. 웹 대시보드·통합 미션

기존 수동 launch와 앱을 **전부 종료**한 뒤 T1:

```bash
ros2 launch tron1_openarm_sim full_system.launch.py \
  foot:=sf planner:=offline world:=world_light.xml rviz:=false
# iMac/CPU 환경은 viewer:=false 추가 가능
```

브라우저 **http://localhost:8080**에서 빨간 상자 운반 명령을 입력합니다. 이 launch는 `OPENARM_BIPED`와 SF SRDF를 자식 프로세스에 자동 설정합니다. 5초/12초 지연 후에도 각 노드 준비를 기다려야 합니다.

**완료 기준:** Task Manager done + 상자가 왼쪽 작업대 위 + 로봇 home 복귀 + 넘어짐 없음. 실패해도 각 단계 결과를 기록하면 원인 분석 실습이 됩니다. 느린 CPU 렌더링에서는 수십 분 걸릴 수 있으며 성공·실행 시간은 현재 장비에서 다시 측정해야 합니다.

실제 LLM 통합은 종료 후 API 변수 설정 터미널에서 `planner:=llm`으로 재실행합니다. API 실패 시 기본 offline fallback을 로그로 구분하세요.

선택적 자동 검증 (별도 full_system을 종료한 상태):

```bash
LOG_DIR=/tmp/tron1_logs bash tron1_openarm/tools/validation/mission_test.sh sf 1
```

**주의:** 이 스크립트는 마지막에 이름 패턴으로 찾은 ROS 관련 프로세스를 `kill -9`로 종료합니다. 별개의 ROS 작업까지 종료할 수 있으므로 **전용 컨테이너/격리된 실습 세션에서만**, 코드를 먼저 확인한 뒤 사용하세요. 시스템 실행·정답 기록을 포함하므로 기존 수동 시스템과 겹쳐 실행하지 않습니다. 교육 기본 절차가 아닌 시험 기록 재현 도구입니다. 두 번째 인자 `1`은 반복 횟수가 아닌 기록 이름이며, 내부 시간 제한·로그 경로도 실행 전 확인합니다.

## 2일차 완료 체크·해석의 한계

- [ ] SF SRDF와 이족 환경 변수가 올바르게 적용됐다.
- [ ] 팔·카메라 도킹·집기·이동 운반을 구분해 시험했다.
- [ ] Task Manager 상태와 물리적 최종 결과를 함께 기록했다.
- [ ] offline은 실제 LLM이 아님을 설명할 수 있다.
- [ ] 1회 성공 기록과 반복 성공률·실로봇 안전성을 구분했다.

[SF 완주 변경 이력](validation-log.md#6-sole-footsf-로-전체-미션-완주) · [흔들림 분석](validation-log.md#4-몸통-흔들림-원인-분석) · [실로봇 한계](../real-robot.md)
