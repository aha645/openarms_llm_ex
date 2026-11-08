# 휠 + OpenArm — 교육 1일차

[README](../../README.md) · [2일차](wheel-day2.md)

## 전제와 공통 실행 규칙

Ubuntu **24.04** + ROS 2 Jazzy + 이 저장소 clone + `04_setup_workspace.sh`에 의한 에셋 생성·colcon 빌드가 완료되어 있다고 가정합니다. 단순 clone과 colcon만으로 외부 메시가 준비되지는 않습니다. 설치는 [노트북](../setup/ubuntu24-native.md) 또는 [iMac](../setup/imac-docker.md)을 보세요.

각 새 터미널에서 저장소 루트로 이동한 뒤:

```bash
# 노트북
cd /home/liky/Documents/openarms_llm_ex   # 자신의 clone 경로로 변경
# Docker는 위 명령 대신: cd /root/openarm_mobile_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset OPENARM_BIPED OPENARM_SRDF
```

**준비 명령 한 줄씩 설명**

| 명령 | 목적과 실행 결과 |
|---|---|
| `cd /home/liky/Documents/openarms_llm_ex` | 현재 작업 폴더를 저장소 루트로 옮깁니다. 이후 `src/...`, `install/...` 상대 경로의 기준입니다. Docker에서는 `/root/openarm_mobile_ws`를 사용합니다. |
| `source /opt/ros/jazzy/setup.bash` | 현재 셸에 Jazzy의 실행 파일·라이브러리·패키지 검색 경로를 등록합니다. ROS 설치 명령이 아닙니다. |
| `source install/setup.bash` | 빌드한 이 저장소의 패키지를 현재 셸에서 찾도록 overlay 환경을 불러옵니다. 빌드를 수행하지는 않습니다. |
| `unset OPENARM_BIPED OPENARM_SRDF` | 앞선 TRON1 실습에서 남은 이족 모드·SRDF 강제 선택을 해제하여 휠형 기본 설정을 사용합니다. 파일을 삭제하지 않습니다. |

T1/T2/T3는 별도 터미널이며, Docker는 모두 컨테이너 안입니다. **단계 전환 시 앞 단계의 launch·teleop·앱을 Ctrl+C로 종료**합니다. sim/slam/nav/full_system은 각각 시뮬레이터를 포함하므로 동시에 실행하지 않습니다. 아래는 CPU 부하를 줄인 light 월드 기준입니다. RViz가 필요 없는 단계는 `rviz:=false`, iMac은 필요 시 `viewer:=false`를 추가합니다.

## 명령을 읽는 공통 규칙

| 표기 | 의미 |
|---|---|
| `ros2 launch 패키지 파일.launch.py` | 패키지에 설치된 launch 파일을 실행하여 여러 노드를 함께 시작합니다. 파일은 현재 폴더가 아니라 ROS 패키지 설치 경로에서 찾습니다. |
| `world:=world_light.xml` 같은 `이름:=값` | launch 인자입니다. launch 파일이 이 값을 받아 하위 노드 설정으로 전달합니다. 해당 launch가 지원하는 인자만 사용합니다. |
| `ros2 run 패키지 실행이름` | 패키지에 등록된 실행 프로그램 하나를 시작합니다. 실행이름이 반드시 Python 파일명과 같은 것은 아닙니다. |
| `--ros-args -p 이름:=값` | 실행 노드에 ROS 파라미터를 전달합니다. `-p`는 parameter이며 파라미터마다 반복합니다. launch 인자와는 다른 전달 방식입니다. |
| `--once` | 해당 하위 명령의 동작을 한 번만 수행합니다. `topic echo`에서는 메시지 하나를 받은 뒤 종료합니다. |
| 줄 끝 `\` | 다음 줄까지 이어지는 **하나의 명령**입니다. `\` 뒤에 공백이나 주석을 넣지 않습니다. |
| `# ...` | 셸 주석입니다. 실행되지 않습니다. 주석 처리된 대체 명령은 필요할 때만 `#`을 빼고 실행합니다. |
| `~`, `$HOME` | 현재 사용자의 홈입니다. 노트북 사용자 홈과 Docker의 `/root`는 서로 다른 위치입니다. |

`world:=world_light.xml`은 패키지 `mjcf/` 안의 저사양 월드를 선택합니다. 시각용 메시를 줄여 렌더링 부담을 낮추는 것이지 로봇의 하반신 종류를 바꾸는 옵션은 아닙니다. `rviz:=false`는 RViz만, `viewer:=false`는 MuJoCo 뷰어 창만 끕니다. 시뮬레이션과 카메라를 모두 중단한다는 의미는 아닙니다. `true`로 설정하면 해당 창을 켭니다.

## 소스 구조: 오늘의 데이터 흐름

```text
build_mjcf.py → mobile_openarm.xml → build_world.py → world_light.xml
sim.launch.py
  ├─ bridge_node.py → sim_core.py (MuJoCo 물리·바퀴 제어·센서)
  ├─ robot_state_publisher ← urdf/mobile_openarm.urdf
  └─ RViz
slam.launch.py → 시뮬 + slam_toolbox
nav.launch.py  → 시뮬 + 지도/AMCL + Nav2
patrol.py      → Nav2 액션 → /cmd_vel → 브리지 → 좌/우 바퀴
```

파일은 `src/mobile_openarm_sim/` 기준이며, 순찰은 `src/mobile_openarm_apps/mobile_openarm_apps/patrol.py`입니다. 모델 생성 도구는 `tools/`, ROS 실행 코드는 `mobile_openarm_sim/`, launch는 `launch/`, 파라미터는 `config/`에 있습니다.

## 1-01. MuJoCo 모델과 월드

ROS launch를 종료한 상태에서:

```bash
python3 -m mujoco.viewer --mjcf=src/mobile_openarm_sim/mjcf/world_light.xml
# 뷰어 종료 후
python3 src/mobile_openarm_sim/tools/selftest.py world_light.xml
```

**명령별 목적과 옵션**

| 명령/구성 요소 | 설명 |
|---|---|
| `python3 -m mujoco.viewer --mjcf=...` | Python 모듈 `mujoco.viewer`를 실행하여 지정한 월드를 직접 엽니다. ROS 없이 모델의 모양과 물리를 관찰하려는 명령입니다. |
| `-m mujoco.viewer` | 파일 경로 대신 설치된 Python 모듈을 실행합니다. |
| `--mjcf=src/.../world_light.xml` | 뷰어가 읽을 MJCF 파일의 경로입니다. 여기서는 저장소 루트 기준 상대 경로이며, ROS launch의 `world:=...`와 문법이 다릅니다. |
| `python3 src/.../selftest.py world_light.xml` | 자체 점검 스크립트를 실행하여 물리·센서 등 항목별 통과 여부를 확인합니다. 마지막 `world_light.xml`은 스크립트가 해석하는 위치 인자이며 `--` 옵션이 아닙니다. 패키지 월드 중 light 버전을 검사합니다. |

첫 명령의 뷰어를 닫은 후 두 번째 명령을 실행합니다. 뷰어가 열린 상태에서 같은 터미널에 다음 명령을 입력하는 방식이 아닙니다.

**테스트:** 상반신 양팔, 좌우 구동 바퀴·캐스터, 작업대와 상자를 구분합니다. 자체 점검의 항목별 결과를 확인합니다. 뷰어 단독 실행은 ROS·Nav2 연결 시험이 아닙니다.

**소스:** `tools/build_mjcf.py`가 휠 베이스와 OpenArm을 결합하고 `tools/build_world.py`가 실습 환경을 구성합니다. MJCF는 물리 모델, URDF는 ROS의 링크·관절 표현입니다.

상세: [MuJoCo 개념과 점검](../labs/day1-01-mujoco-basics.md).

## 1-02. 브리지와 직접 운전

T1:

```bash
ros2 launch mobile_openarm_sim sim.launch.py world:=world_light.xml rviz:=false
```

T2:

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**명령별 목적과 옵션**

| 명령 | 이 한 줄로 하려는 일 / 옵션 의미 |
|---|---|
| `ros2 launch mobile_openarm_sim sim.launch.py world:=world_light.xml rviz:=false` | `mobile_openarm_sim`의 launch로 시뮬·ROS 브리지·로봇 TF 발행을 시작합니다. `world`는 light 월드 선택, `rviz:=false`는 RViz 중복 렌더링 방지입니다. MuJoCo 뷰어는 기본값으로 켜집니다. 이 명령만으로 SLAM/Nav2는 시작하지 않습니다. |
| `ros2 topic list` | 현재 발견되는 토픽 이름을 나열합니다. 메시지 내용이나 발행 정상 여부까지 검증하는 명령은 아닙니다. |
| `ros2 topic echo /odom --once` | `/odom`의 메시지 한 개를 받아 위치·방향·속도를 확인한 뒤 종료합니다. 발행자가 없으면 기다릴 수 있습니다. |
| `ros2 topic echo /joint_states --once` | 관절 이름·위치·속도 등 메시지 한 개를 확인하여 로봇 상태가 ROS로 전달되는지 검사합니다. |
| `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | 앞 이름은 패키지, 뒤 이름은 실행 프로그램입니다. 키 입력을 `/cmd_vel` 속도 명령으로 보내 직접 운전합니다. 자동 경로 계획은 하지 않습니다. |
| `ros2 topic echo /cmd_vel` | 별도 T3에서 속도 명령을 계속 관찰합니다. `/cmd_vel`의 `linear.x`는 전후 속도(m/s), `angular.z`는 회전 속도(rad/s)입니다. `--once`가 없어 Ctrl+C까지 계속 출력합니다. 관찰용 echo를 종료해도 시뮬은 꺼지지 않습니다. |

teleop 터미널을 선택하고 안내에 따라 이동합니다. `z`로 속도를 낮추고 짧게 전진·회전한 뒤 정지합니다. T3에서 필요 시 `ros2 topic echo /cmd_vel`로 명령을 관찰합니다.

**확인 기준:** 명령에 따라 이동하고 `/odom`·관절 값이 변합니다. `/clock`, `/scan`, 카메라 관련 토픽도 발행되는지 확인합니다.

**소스:** `bridge_node.py`는 ROS 토픽·액션·서비스를 연결합니다. `sim_core.py`는 선속도/각속도를 좌우 바퀴 속도로 바꾸고 물리와 센서를 계산합니다. 상위 앱은 바퀴 actuator를 직접 제어하지 않습니다.

상세: [브리지·액션·리셋](../labs/day1-02-bridge.md).

## 1-03. Description·TF·센서

앞 단계의 sim만 유지하고 teleop는 종료합니다. T2:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_footprint head_camera_link
# Ctrl+C 후
ros2 run tf2_ros tf2_echo base_footprint openarm_right_hand_tcp
```

**명령별 목적과 인자**

| 명령 | 설명과 기대 결과 |
|---|---|
| `ros2 run tf2_tools view_frames` | `tf2_tools` 패키지의 도구로 잠시 TF를 수집하여 현재 폴더에 프레임 관계 그래프 파일을 생성합니다. 링크들이 하나의 좌표계 트리로 연결되는지 확인하려는 명령이며 로봇을 움직이지 않습니다. |
| `ros2 run tf2_ros tf2_echo base_footprint head_camera_link` | `base_footprint` 기준에서 `head_camera_link`의 위치·회전을 계속 출력합니다. 뒤의 두 이름은 옵션이 아니라 기준 프레임과 대상 프레임의 위치 인자입니다. 거리 단위는 m입니다. |
| `ros2 run tf2_ros tf2_echo base_footprint openarm_right_hand_tcp` | 같은 기준에서 오른손 TCP(작업 기준점)의 위치·회전을 확인합니다. 팔 자세에 따라 값이 달라집니다. 첫 echo를 Ctrl+C로 끊은 뒤 실행합니다. |
| `rviz:=true`로 sim 재실행 | 위 1-02의 sim 명령에서 `rviz:=false`를 `true`로 바꾸면 센서/로봇을 시각화하는 RViz를 함께 켭니다. 기존 sim을 먼저 종료해야 합니다. |

`tf2_echo`의 출력 중 RPY는 roll/pitch/yaw이며 rad와 degree 표기를 구분해서 읽습니다. 좌표가 있다고 해서 센서 측정 자체의 정확성이 검증되는 것은 아닙니다.

**테스트:** `odom → base_footprint → base_link → 팔/카메라/라이다` 경로를 확인합니다. `map`은 sim 단독 단계에서는 아직 없어도 정상입니다. RViz가 필요하면 sim을 종료 후 `rviz:=true`로 재실행합니다.

**소스:** `tools/mjcf_to_urdf.py`, `urdf/mobile_openarm.urdf`, sim launch의 `robot_state_publisher`, `rviz/sim.rviz`.

상세: [TF·URDF·센서](../labs/day1-03-description-tf.md).

## 1-04. SLAM 지도 작성과 저장

기존 sim 종료. T1:

```bash
ros2 launch mobile_openarm_sim slam.launch.py world:=world_light.xml
```

T2:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

천천히 방을 돌고 출발점으로 돌아옵니다. T3:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/wheel_map \
  --ros-args -p use_sim_time:=true
```

**명령별 목적과 옵션**

| 명령 | 설명 |
|---|---|
| `ros2 launch mobile_openarm_sim slam.launch.py world:=world_light.xml` | light 월드의 시뮬과 SLAM을 함께 시작합니다. 라이다와 오도메트리로 새 지도를 만드는 단계입니다. 앞의 sim launch를 함께 실행하지 않습니다. |
| `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | SLAM이 여러 방향의 벽을 관측하도록 사람이 속도를 입력해 로봇을 이동시킵니다. 키 입력은 이 명령을 실행한 터미널에서 합니다. |
| `mkdir -p ~/maps` | 지도 저장 폴더를 만듭니다. `-p`는 상위 폴더도 생성하고 이미 존재해도 오류를 내지 않는 옵션입니다. ROS의 `-p` 파라미터 옵션과는 다릅니다. |
| `ros2 run nav2_map_server map_saver_cli -f ~/maps/wheel_map --ros-args -p use_sim_time:=true` | 현재 `/map`을 파일로 저장합니다. `nav2_map_server`는 패키지, `map_saver_cli`는 저장 도구입니다. 저장 후 종료하며 SLAM은 계속 실행됩니다. |
| `-f ~/maps/wheel_map` | 출력 **파일명 접두사**입니다. 디렉터리만 지정하는 것이 아니며, 이 접두사에 `.yaml`과 지도 이미지 확장자가 붙습니다. |
| `--ros-args -p use_sim_time:=true` | 저장 노드가 실제 PC 시간이 아니라 시뮬레이터의 `/clock`을 사용하도록 합니다. `--ros-args`부터 ROS용 설정이고 `-p` 뒤가 파라미터 지정입니다. |

지도 저장은 운전·SLAM을 유지한 상태에서 별도 T3로 실행합니다. 마지막 줄은 `\`로 이어져 있어도 하나의 저장 명령입니다.

**확인 기준:** RViz에서 `/map`이 확장되고 `wheel_map.yaml`과 이미지가 생성됩니다. Docker의 `~/maps`는 컨테이너 root 홈이므로 컨테이너 삭제 후에도 보관하려면 공유 소스 폴더나 별도 볼륨으로 복사하세요.

**소스:** `launch/slam.launch.py`, `config/slam_toolbox.yaml`; 센서 `/scan`과 오도메트리 TF를 slam_toolbox에 제공합니다.

상세: [SLAM·AMCL](../labs/day1-04-slam-rviz.md).

## 1-05. Nav2 자율주행

SLAM과 teleop 모두 종료. T1:

```bash
ros2 launch mobile_openarm_sim nav.launch.py world:=world_light.xml
# 직접 작성한 지도로 비교하려면 위 명령 대신:
# ros2 launch mobile_openarm_sim nav.launch.py world:=world_light.xml map:=$HOME/maps/wheel_map.yaml
```

**명령별 목적과 옵션**

| 명령/인자 | 설명 |
|---|---|
| `ros2 launch mobile_openarm_sim nav.launch.py world:=world_light.xml` | 시뮬과 저장 지도 기반 위치 추정(AMCL), Nav2 경로 계획·제어를 함께 시작합니다. 목표를 지정할 준비만 하며 즉시 특정 위치로 출발하는 명령은 아닙니다. RViz는 기본으로 켜집니다. |
| `world:=world_light.xml` | 시뮬레이터가 읽을 물리 월드입니다. Nav2가 읽는 지도 파일을 지정하는 옵션은 아닙니다. |
| `map:=$HOME/maps/wheel_map.yaml` | Nav2가 읽을 지도 메타데이터 YAML을 1-04에서 만든 파일로 교체합니다. `$HOME`은 셸이 홈 경로로 확장합니다. YAML은 해당 지도 이미지·해상도·원점 등을 설명합니다. |

두 launch 예시는 **택일**합니다. 먼저 기본 지도로 시작한 뒤 비교하려면 종료하고 `map:=...`을 추가한 명령을 실행합니다. 물리 월드와 지도 모양이 다르면 위치 추정·주행이 실패할 수 있습니다.

기본 지도는 `maps/world_map.yaml`의 생성된 정답 지도입니다. RViz의 **Nav2 Goal**로 빈 공간의 목표를 찍고 경로·장애물 회피·도착을 관찰합니다. 위치가 어긋나면 **2D Pose Estimate**로 초기 위치를 맞춥니다.

**확인 기준:** Nav2 lifecycle 활성화, 경로 생성, 목표 도착. SLAM 지도 사용과 기본 정답 지도 사용을 혼동하지 않습니다.

**소스:** `launch/nav.launch.py`, `config/nav2_params.yaml`, `maps/world_map.*`. 경로 계획과 제어는 Nav2가 담당하고 브리지는 `/cmd_vel`을 실행합니다.

상세: [Nav2 액션 명령·문제 해결](../labs/day1-05-nav2.md).

## 1-06. 순찰 앱

앞 단계의 Nav2만 유지하고 목표 실행이 끝난 뒤 T2:

```bash
ros2 run mobile_openarm_apps patrol --ros-args -p loops:=1 -p mode:=waypoints
# 종료 후 비교 실습:
# ros2 run mobile_openarm_apps patrol --ros-args -p loops:=1 -p mode:=through
```

**명령별 목적과 옵션**

| 구성 요소 | 의미 |
|---|---|
| `ros2 run mobile_openarm_apps patrol` | 공통 앱 패키지의 순찰 프로그램을 실행하여, 이미 실행 중인 Nav2에 이름으로 정의한 여러 목표를 보냅니다. 이 앱 자체가 시뮬이나 Nav2를 새로 띄우지는 않습니다. |
| `--ros-args` | 이후 인자를 ROS 설정으로 해석하도록 하는 구분자입니다. |
| `-p loops:=1` | 순찰 경로를 1회 반복합니다. 마지막 home 복귀와는 별개이며 숫자를 늘리면 경로를 그 횟수만큼 반복합니다. |
| `-p mode:=waypoints` | `followWaypoints()`로 지점별 방문을 요청합니다. 각 목표를 순서대로 방문하는 동작을 시험합니다. |
| `-p mode:=through` | 대체 명령에서는 `goThroughPoses()`로 여러 자세를 통과하는 주행을 요청합니다. 같은 순찰 경로를 다른 Nav2 액션 방식으로 비교합니다. |

첫 순찰이 종료된 뒤 비교 명령을 실행합니다. 단순히 `Ctrl+C`로 순찰 클라이언트만 끊었다고 Nav2의 진행 중 목표가 반드시 취소됐다고 가정하지 마세요. 앱 로그·로봇 정지를 확인하고 필요하면 시스템을 재시작합니다.

**테스트:** `locations.yaml`의 순찰 지점을 차례로 방문하고 home으로 복귀하는지 확인합니다. 실패 시 앱 로그와 Nav2 결과를 함께 기록합니다.

**소스:** `patrol.py`의 `BasicNavigator`, `followWaypoints()` / `goThroughPoses()`, `src/mobile_openarm_sim/config/locations.yaml`. 앱은 공통 위치 파일을 읽습니다.

상세: [순찰](../labs/day1-06-patrol.md).

## 1일차 완료 체크

- [ ] 모델과 ROS 브리지 역할을 구분했다.
- [ ] 직접 운전·오도메트리·TF·센서 토픽을 확인했다.
- [ ] SLAM 지도 저장과 저장 지도 기반 Nav2 실행을 해봤다.
- [ ] 순찰 결과와 실패 로그를 기록했다.

모든 launch·앱을 종료한 뒤 [2일차](wheel-day2.md)로 이동합니다.
