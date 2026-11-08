# TRON1 + OpenArm — 교육 1일차

[README](../../README.md) · [2일차](tron1-day2.md) · [휠형 1일차](wheel-day1.md)

## 전제: 환경은 같지만 TRON1 빌드는 별도

Ubuntu 24.04 / ROS 2 Jazzy / 저장소 clone / 휠형 에셋·colcon 빌드까지 완료되어 있다고 가정합니다. **TRON1 교육은 추가로 TRON1 모델·정책·패키지 준비까지 완료한 상태**에서 시작합니다. 이미 완료했다면 아래 설치 명령은 반복하지 않습니다.

준비 담당자가 한 번 실행할 명령 (저장소 루트, Docker는 컨테이너 안):

```bash
source /opt/ros/jazzy/setup.bash
# 네이티브는 sudo apt-get, root Docker에서는 apt-get 사용
# 필요한 패키지: ros-jazzy-xacro liburdfdom-tools
bash tron1_openarm/scripts/10_setup_tron1.sh
colcon build --symlink-install --base-paths src tron1_openarm/src
source install/setup.bash
ros2 pkg prefix tron1_openarm_sim
ros2 pkg prefix tron1_openarm_moveit_config
```

`10`은 외부 TRON1 description을 clone하고 PF/SF 모델·월드·URDF/SRDF·Nav2 파라미터를 생성한 뒤 두 트리를 빌드합니다. **스크립트의 패키지 설치는 `sudo`를 사용하며 실패를 무시**합니다. root Docker에 sudo가 없다면 먼저 아래를 실행한 뒤 `10`을 실행하세요.

```bash
# root Docker에만 해당
apt-get update
apt-get install -y ros-jazzy-xacro liburdfdom-tools
```

`10`의 자체 점검 실패도 무시될 수 있습니다. 빌드 성공과 보행 성공은 별도로 확인합니다. 기본 실습에는 PPO 재학습이나 CUDA/JAX 설치가 필요하지 않습니다.

### PF와 SF를 혼동하지 않기

| 구분 | Point-foot (PF) | Sole-foot (SF) |
|---|---|---|
| 실행 인자 | `foot:=pf` (launch 기본값) | `foot:=sf` (이 교육의 기준) |
| 특징 | 점 접촉 발, 제자리 걸음·드리프트 | 발목 있는 발, 정지 시 서기 모드 |
| 기존 기록 | 집기 중 흔들림·표류 한계 | 전체 운반 미션 1회 완주 기록 |

**명령을 생략하면 PF가 선택됩니다. 이 문서는 모든 TRON1 launch에 SF를 명시**합니다. 한 번의 성공은 반복 성공률 보장이 아닙니다. [기존 SF 시험 기록](../tron1/09-sf-mission.md).

## 공통 터미널 규칙

각 터미널은 저장소 루트에서 시작합니다. 노트북은 현재 clone 경로, Docker는 `/root/openarm_mobile_ws`입니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

T1/T2/T3는 별도 터미널입니다. 단계 전환 시 기존 launch·teleop를 Ctrl+C로 종료합니다. 휠형과 이족형, sim/slam/nav/full_system을 동시에 올리지 않습니다. 같은 노드·토픽·액션을 공유하기 때문입니다.

## 소스 구성과 휠형과의 차이

```text
tron1_openarm/src/tron1_openarm_sim/
  tools/build_tron1_openarm.py       # TRON1 description + OpenArm 결합, PF/SF
  tools/build_world_tron1.py         # 공통 월드 생성 도구 재사용
  tools/make_urdf_tron1.py           # ROS용 URDF 생성
  tron1_openarm_sim/
    locomotion_policy.py            # npz 정책 추론
    biped_core.py                   # 다리 제어·보행·오도메트리
    bridge_biped.py                 # 공통 ROS 인터페이스에 이족 코어 연결
  launch/                           # sim, slam, nav, full_system
  policies/                         # PF/SF 배포 정책
  mjcf/                             # PF, sf/ 아래 SF 월드
```

`/cmd_vel → 보행 정책 → 다리 관절 제어`가 하반신의 핵심 차이입니다. 휠형처럼 좌우 바퀴 속도로 변환하지 않습니다. 상위 앱은 `src/mobile_openarm_apps`를 재사용합니다. 자세한 설계: [모델 결합](../tron1/02-model.md), [보행 정책](../tron1/03-locomotion-rl.md).

## 1-01. 정책을 포함한 모델 점검

```bash
python3 tron1_openarm/src/tron1_openarm_sim/tools/selftest_tron1.py \
  sf/world_light.xml policies/tron1sf_openarm_policy.npz
```

**테스트:** 서기·걷기·회전·센서·팔 관련 항목의 결과를 확인합니다. TRON1 월드를 단순 MuJoCo viewer로만 열면 보행 정책이 제어하지 않아 주저앉을 수 있습니다. 첫 시각화는 아래 sim launch로 합니다.

## 1-02. 보행 브리지와 직접 운전

T1:

```bash
ros2 launch tron1_openarm_sim sim.launch.py \
  foot:=sf world:=world_light.xml rviz:=false
```

T2:

```bash
ros2 topic echo /biped/status --field data
```

T3:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

안내에 따라 속도를 낮춰 약 0.2~0.3 m/s로 짧게 전진·회전하고 정지합니다. PF 비교는 현재 launch 종료 후 `foot:=pf`로 바꿔 재실행합니다.

**확인 기준:** 넘어짐 여부, 몸통 높이·기울기, 명령·추정속도, 정지 시 동작을 기록합니다. `/joint_states`에는 바퀴 대신 다리 관절이 포함됩니다. `/odom`과 `/ground_truth/pose`를 비교할 수 있지만 정답 위치를 실제 로봇에서 얻을 수 있는 센서값으로 혼동하지 않습니다.

**소스:** `bridge_biped.py → biped_core.py → locomotion_policy.py`. `sim.launch.py`가 발 형태에 맞는 URDF·월드·정책을 선택하고 `TRON1_POLICY`를 설정합니다.

## 1-03. TF와 이족 좌표계

sim은 유지하고 teleop는 종료. T2:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_footprint head_camera_link
```

**테스트:** 다리 링크와 OpenArm/센서가 같은 트리에 연결되는지 확인합니다. 몸통이 흔들리는 실제 물리 모델과 ROS용 명목 높이 표현의 차이를 설명합니다. PF의 높이 수치를 SF에 그대로 적용하지 않습니다.

**소스:** `tools/make_urdf_tron1.py`, `urdf/tron1_openarm_sf.urdf`, `biped_core.py`의 odom 처리. 원리 설명은 [기존 ROS 실습의 TF 절](../tron1/04-ros2-labs.md)에 있지만 해당 문서는 PF 기준 수치가 섞여 있습니다.

## 1-04. SLAM 지도 작성

기존 sim 종료. T1:

```bash
ros2 launch tron1_openarm_sim slam.launch.py foot:=sf world:=world_light.xml
```

T2:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

천천히 이동·회전하고 출발점으로 돌아옵니다. T3:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/tron1_sf_map \
  --ros-args -p use_sim_time:=true
```

**확인 기준:** 지도와 파일 생성, 벽 겹침·왜곡 정도. Docker의 `~/maps`는 컨테이너 안이므로 영구 보관이 필요하면 공유 폴더로 복사합니다. SF와 PF는 센서 높이·보행 특성이 다르므로 지도 시험의 발 형태를 기록하세요.

**소스:** TRON1 `launch/slam.launch.py`, SLAM 설정, 보행 오도메트리. 정책 추종 오차·스캔 흔들림이 SLAM에 미치는 영향을 휠형과 비교합니다.

## 1-05. Nav2 주행

SLAM·teleop 종료. T1:

```bash
ros2 launch tron1_openarm_sim nav.launch.py foot:=sf world:=world_light.xml
# 직접 만든 지도와 비교할 때는 위 명령 대신:
# ros2 launch tron1_openarm_sim nav.launch.py foot:=sf world:=world_light.xml map:=$HOME/maps/tron1_sf_map.yaml
```

RViz Nav2 Goal로 빈 공간 목표를 지정합니다. 기본값은 생성된 `maps/world_map.yaml`이며 자체 작성 SLAM 지도가 아닙니다.

**확인 기준:** lifecycle 활성화·목표 도착·기울기·넘어짐·도착 오차. 휠형보다 느릴 수 있으며 시간 초과와 정책 추종 실패를 구분합니다.

**소스:** `launch/nav.launch.py`, `config/nav2_params_tron1.yaml`, `tools/make_nav2_params_tron1.py`. 상위 Nav2가 보낸 속도를 보행 정책이 얼마나 따르는지가 결과에 영향을 줍니다. 기존 문서의 속도·허용오차 표보다 현재 YAML을 기준으로 확인하세요.

## 1-06. 공통 순찰 앱 재사용

Nav2 유지, T2:

```bash
ros2 run mobile_openarm_apps patrol --ros-args -p loops:=1 -p mode:=waypoints
```

**테스트:** 하반신이 바뀌어도 같은 Nav2 앱으로 순찰·home 복귀가 가능한지 확인합니다. 앱은 `mobile_openarm_sim/config/locations.yaml`을 읽으므로 휠 패키지도 설치되어 있어야 합니다.

**소스:** `src/mobile_openarm_apps/mobile_openarm_apps/patrol.py`는 그대로 사용하고 TRON1 Nav2/브리지만 교체합니다.

## 1일차 완료 체크

- [ ] PF/SF와 정책 파일의 대응을 확인했다.
- [ ] 보행·정지·기울기·오도메트리를 관찰했다.
- [ ] TF·SLAM·Nav2·순찰을 실행하고 발 형태·실패 로그를 기록했다.

모두 종료한 뒤 [TRON1 2일차](tron1-day2.md)로 이동합니다.
