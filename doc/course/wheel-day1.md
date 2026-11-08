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

T1/T2/T3는 별도 터미널이며, Docker는 모두 컨테이너 안입니다. **단계 전환 시 앞 단계의 launch·teleop·앱을 Ctrl+C로 종료**합니다. sim/slam/nav/full_system은 각각 시뮬레이터를 포함하므로 동시에 실행하지 않습니다. 아래는 CPU 부하를 줄인 light 월드 기준입니다. RViz가 필요 없는 단계는 `rviz:=false`, iMac은 필요 시 `viewer:=false`를 추가합니다.

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

**테스트:** `locations.yaml`의 순찰 지점을 차례로 방문하고 home으로 복귀하는지 확인합니다. 실패 시 앱 로그와 Nav2 결과를 함께 기록합니다.

**소스:** `patrol.py`의 `BasicNavigator`, `followWaypoints()` / `goThroughPoses()`, `src/mobile_openarm_sim/config/locations.yaml`. 앱은 공통 위치 파일을 읽습니다.

상세: [순찰](../labs/day1-06-patrol.md).

## 1일차 완료 체크

- [ ] 모델과 ROS 브리지 역할을 구분했다.
- [ ] 직접 운전·오도메트리·TF·센서 토픽을 확인했다.
- [ ] SLAM 지도 저장과 저장 지도 기반 Nav2 실행을 해봤다.
- [ ] 순찰 결과와 실패 로그를 기록했다.

모든 launch·앱을 종료한 뒤 [2일차](wheel-day2.md)로 이동합니다.
