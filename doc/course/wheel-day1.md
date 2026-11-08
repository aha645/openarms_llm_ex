# 휠 + OpenArm — 1일차: 로봇을 움직이고 지도로 길 찾기

[README](../../README.md) · 다음: [2일차](wheel-day2.md)

오늘은 **로봇 화면 확인 → 직접 운전 → 카메라 좌표 확인 → 지도 만들기 → 자율주행 → 순찰** 순서로 실습합니다. 로봇은 차동구동 바퀴 하반신과 OpenArm 양팔 상반신으로 구성됩니다.

## 시작 전에

Ubuntu 24.04, ROS 2 Jazzy, 저장소 다운로드, 에셋 생성과 빌드가 완료되어 있어야 합니다. 아직 준비하지 않았다면 [노트북 설치](../setup/ubuntu24-native.md) 또는 [iMac Docker 설치](../setup/imac-docker.md)를 먼저 진행하세요.

- **T1, T2, T3는 서로 다른 터미널**입니다. 실행 중인 프로그램이 로그를 출력하고 있다면 다음 명령은 새 터미널에서 실행합니다.
- iMac에서는 ROS 명령을 모두 **컨테이너 안**에서 실행합니다.
- 프로그램을 종료하려면 그 터미널에서 `Ctrl+C`를 누릅니다.
- `sim`, `slam`, `nav` launch는 모두 시뮬레이터를 포함합니다. 문서에서 종료하라고 하면 이전 것을 종료하고 다음 것을 실행하세요.

## 0. 새 터미널마다 준비하기

### 0-1. 저장소 폴더로 이동

```bash
cd /home/liky/Documents/openarms_llm_ex
```

**상세 설명:** `cd`는 현재 작업 폴더를 바꾸는 셸 명령입니다. 뒤의 경로는 이 노트북의 저장소 위치입니다. 다른 위치에 받았다면 해당 경로로 바꾸세요. Docker 안에서는 이 명령 대신 `cd /root/openarm_mobile_ws`를 실행합니다.

**이 한 줄이 하는 일:** 이후 `install/...`, `src/...`가 이 저장소 안의 파일을 가리키도록 기준 폴더를 맞춥니다. 로봇을 실행하지는 않습니다.

### 0-2. 기본 ROS 환경 불러오기

```bash
source /opt/ros/jazzy/setup.bash
```

**상세 설명:** `source`는 설정 파일을 **현재 터미널에 적용**합니다. `/opt/ros/jazzy/setup.bash`는 설치된 Jazzy의 환경 설정 파일입니다.

**이 한 줄이 하는 일:** 현재 터미널에서 `ros2` 명령과 기본 ROS 패키지를 사용할 수 있게 합니다. 설치나 빌드를 수행하는 명령은 아닙니다.

### 0-3. 우리 로봇 패키지 환경 불러오기

```bash
source install/setup.bash
```

**상세 설명:** `install/setup.bash`는 이 저장소를 빌드하면서 생성한 설정 파일입니다. ROS가 `mobile_openarm_sim` 같은 우리 패키지를 찾는 경로를 등록합니다.

**이 한 줄이 하는 일:** 현재 터미널에서 이 프로젝트의 프로그램을 실행할 수 있게 합니다. 파일이 없다면 경로 또는 빌드 완료 여부를 확인하세요.

이전에 TRON1을 실행했던 터미널이라면 아래도 실행합니다.

```bash
unset OPENARM_BIPED OPENARM_SRDF
```

**상세 설명:** `unset`은 환경 변수 설정을 지웁니다. `OPENARM_BIPED`는 이족 모드, `OPENARM_SRDF`는 팔 자세 설정 파일을 강제로 선택할 때 쓰는 변수입니다.

**이 한 줄이 하는 일:** TRON1 설정이 휠형 실습에 남아 영향을 주지 않게 합니다. 파일을 삭제하는 명령이 아닙니다.

## 명령을 읽는 데 필요한 규칙 두 가지

```text
ros2 run    패키지이름  실행프로그램이름  그 프로그램에 줄 입력값/옵션
ros2 launch 패키지이름  launch파일이름    launch에 줄 설정값
```

- **패키지**는 관련 프로그램·설정을 묶은 단위입니다.
- **실행 프로그램**은 패키지에 설치·등록된 이름입니다. Python 파일명이라고 단정하면 안 됩니다. C++ 프로그램일 수도 있습니다.
- **run 뒤의 입력값은 프로그램마다 다릅니다.** 아래에서 각 명령에 맞춰 설명합니다.
- **launch**는 여러 프로그램을 함께 시작하는 실행 구성입니다. 설정값은 보통 `이름:=값`으로 전달합니다.

## 1. 로봇과 월드만 화면으로 보기

다른 ROS launch가 꺼진 상태에서 실행합니다.

```bash
python3 -m mujoco.viewer --mjcf=src/mobile_openarm_sim/mjcf/world_light.xml
```

**상세 설명**

- `python3`: Python을 실행합니다.
- `-m mujoco.viewer`: 설치된 `mujoco.viewer` 모듈을 프로그램으로 실행합니다.
- `--mjcf=...`: 뷰어에서 열 로봇·환경 파일을 지정합니다. MJCF는 MuJoCo의 물리 모델 파일 형식입니다.
- `world_light.xml`: 시각용 메시를 줄인 가벼운 월드입니다. 작업대·상자와 휠+OpenArm 로봇이 포함됩니다.

**이 한 줄이 하는 일:** ROS 연결 없이 MuJoCo 화면을 열어 로봇과 실습 공간을 관찰합니다. 자동 주행이나 순찰을 시작하지 않습니다.

**확인:** 바퀴, 양팔, 작업대와 상자가 보이는지 확인한 뒤 뷰어를 종료합니다.

## 2. 시뮬레이터를 ROS에 연결하고 직접 운전하기

### T1 — 로봇 실행

```bash
ros2 launch mobile_openarm_sim sim.launch.py world:=world_light.xml rviz:=false
```

**상세 설명**

- `ros2 launch`: 여러 노드를 함께 실행합니다. 노드는 실행 중인 ROS 프로그램입니다.
- `mobile_openarm_sim`: 시뮬레이션 패키지입니다.
- `sim.launch.py`: MuJoCo, ROS 브리지, 로봇 좌표계 발행을 시작하는 launch 파일입니다. 브리지는 시뮬레이터와 ROS 사이에서 명령·센서값을 전달합니다.
- `world:=world_light.xml`: 가벼운 월드를 선택합니다. 이 launch에서는 패키지의 `mjcf/` 안에서 월드를 찾습니다.
- `rviz:=false`: RViz 시각화 창은 열지 않습니다. MuJoCo 뷰어는 기본값으로 열립니다. `false`는 끄기, `true`는 켜기입니다.

**이 한 줄이 하는 일:** ROS 명령으로 움직일 수 있는 로봇을 준비합니다. 아직 지도 작성이나 자율주행 기능은 실행하지 않습니다.

**확인:** MuJoCo 창과 브리지 시작 로그를 확인하고 T1은 그대로 둡니다.

### T2 — 로봇 위치 메시지 하나 확인

T2에서도 먼저 0절의 환경 설정을 실행합니다.

```bash
ros2 topic echo /odom --once
```

**상세 설명**

- `ros2 topic echo`: 토픽으로 전달되는 메시지를 화면에 출력합니다. 토픽은 프로그램 간 데이터가 흐르는 통로입니다.
- `/odom`: 바퀴 이동량으로 추정한 로봇 위치·속도 데이터의 토픽 이름입니다.
- `--once`: 메시지를 하나 받으면 종료합니다. 메시지가 오지 않으면 기다릴 수 있습니다.

**이 한 줄이 하는 일:** 브리지가 로봇의 이동 상태를 ROS로 보내는지 확인합니다. 로봇을 움직이지는 않습니다. 정지 중이라면 위치·속도 값이 거의 변하지 않는 것이 정상입니다.

### T2 — 키보드 운전

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**상세 설명**

- `ros2 run`: 설치된 실행 프로그램 하나를 시작합니다.
- 첫 번째 `teleop_twist_keyboard`: 패키지 이름입니다.
- 두 번째 `teleop_twist_keyboard`: 그 패키지의 실행 프로그램 이름입니다. 두 이름이 같은 경우입니다.
- 추가 입력값은 없습니다. 실행 후 터미널에 나타나는 안내에 따라 키를 누릅니다.

**이 한 줄이 하는 일:** 키 입력을 `/cmd_vel` 속도 명령으로 보내 로봇을 직접 운전합니다. 이 터미널을 선택한 상태에서 조작하세요.

**확인:** `z`로 속도를 낮추고 짧게 전진·회전해 봅니다. 정지 키로 로봇을 멈춘 뒤 `Ctrl+C`로 teleop를 종료합니다. T1의 시뮬은 유지합니다.

## 3. 로봇 바닥을 기준으로 카메라 위치 확인하기

T1의 시뮬이 켜진 상태에서 T2로 실행합니다.

```bash
ros2 run tf2_ros tf2_echo base_footprint head_camera_link
```

**상세 설명**

- `tf2_ros`: 좌표계 사이의 관계를 다루는 도구가 들어 있는 패키지입니다.
- `tf2_echo`: 두 좌표계 사이의 위치·방향을 계속 출력하는 실행 프로그램입니다. Python 파일 이름이나 좌표계 이름이 아닙니다.
- `base_footprint`: **첫 번째 입력값**, 기준 좌표계 이름입니다. 이 로봇의 바닥 쪽 이동 기준점과 축을 뜻합니다.
- `head_camera_link`: **두 번째 입력값**, 확인할 대상 좌표계 이름입니다. 머리 카메라 장착 프레임입니다.
- 뒤의 두 단어는 파일이 아니라, 실행 중인 로봇의 TF 트리에 등록된 이름입니다. TF는 좌표계 간 위치·방향 관계를 전달하는 시스템입니다.

**이 한 줄이 하는 일:** “로봇 바닥 기준으로 머리 카메라가 어디에, 어떤 방향으로 붙어 있는가?”를 묻습니다. 로봇을 움직이거나 카메라로 거리를 새로 측정하지 않습니다.

**출력 읽기:** `Translation [x, y, z]`는 기준 좌표계 축 방향의 거리(m), `Rotation`은 상대 회전입니다. 예를 들어 z가 1.16이면 기준 원점보다 +Z 방향으로 1.16m에 있다는 뜻입니다. 이 숫자는 해석 예시이며 실제 출력값을 확인하세요. `head_camera_link`가 영상 처리용 optical frame과 같다고 가정하지 마세요.

카메라가 고정 장착되어 있으면 출력이 반복돼도 값은 같을 수 있습니다. 확인 후 `Ctrl+C`로 조회만 종료합니다.

## 4. 직접 운전하면서 지도 만들기 — SLAM

**T1의 기존 sim을 종료**하고 시작합니다. SLAM은 로봇 위치를 추정하면서 주변 지도를 만드는 기능입니다.

### T1 — 시뮬과 지도 작성 기능 시작

```bash
ros2 launch mobile_openarm_sim slam.launch.py world:=world_light.xml
```

**상세 설명:** `mobile_openarm_sim`은 앞과 같은 패키지입니다. 이번에는 `sim.launch.py`가 아니라 **`slam.launch.py`**를 선택합니다. 이 파일은 시뮬과 SLAM을 함께 실행합니다. `world:=world_light.xml`은 가벼운 물리 월드 선택입니다. RViz는 기본으로 열립니다.

**이 한 줄이 하는 일:** 라이다 거리 측정과 로봇 이동 정보를 이용해 새 지도를 만들 준비를 합니다. 로봇을 자동으로 탐색시키지는 않습니다.

### T2 — 지도를 만들기 위해 운전

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**상세 설명:** 2절과 같은 키보드 운전 프로그램입니다. 추가 옵션은 없습니다. 이번에는 로봇이 이동하면서 벽과 장애물을 여러 방향에서 보도록 조작합니다.

**이 한 줄이 하는 일:** SLAM에 필요한 관측이 쌓이도록 사람이 로봇을 움직입니다. 천천히 방을 돌고 출발점으로 돌아오세요.

**확인:** RViz의 지도(`/map`)가 이동에 따라 넓어지는지 봅니다. 지도 저장 전에는 정지 키로 로봇을 멈춥니다.

### T3 — 저장 폴더 만들기

```bash
mkdir -p ~/maps
```

**상세 설명:** `mkdir`은 폴더 생성, `-p`는 필요한 상위 폴더도 만들고 이미 있으면 그대로 사용하라는 옵션입니다. `~/maps`는 현재 사용자의 홈 아래 `maps` 폴더입니다.

**이 한 줄이 하는 일:** 지도 파일을 저장할 위치를 준비합니다. 지도 자체를 저장하지는 않습니다.

### T3 — 현재 지도 저장

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/wheel_map --ros-args -p use_sim_time:=true
```

**상세 설명**

- `nav2_map_server`: 지도 관련 프로그램을 제공하는 패키지입니다.
- `map_saver_cli`: 현재 지도를 파일로 저장하는 실행 프로그램입니다.
- `-f ~/maps/wheel_map`: 저장할 파일의 경로와 이름 접두사입니다. `wheel_map.yaml`과 지도 이미지가 생성됩니다.
- `--ros-args`: 여기부터 ROS용 설정을 전달한다는 구분자입니다.
- `-p use_sim_time:=true`: 노드 파라미터 `use_sim_time`을 켭니다. PC 시계 대신 시뮬레이터의 `/clock`을 사용합니다. 여기의 `-p`는 위 `mkdir -p`와 다른 의미입니다.

**이 한 줄이 하는 일:** 실행 중인 SLAM의 `/map`을 파일로 보관합니다. 저장 명령이 끝나도 SLAM은 계속 실행됩니다.

**확인:** 저장 성공 로그와 `~/maps/wheel_map.yaml`·지도 이미지 생성을 확인합니다. Docker의 `~/maps`는 컨테이너 안의 `/root/maps`입니다. 컨테이너 삭제 후에도 보관하려면 공유 폴더로 복사하세요.

## 5. 저장된 지도로 자율주행하기 — Nav2

**SLAM과 teleop를 모두 종료**합니다.

### T1 — 위치 추정과 자율주행 기능 시작

```bash
ros2 launch mobile_openarm_sim nav.launch.py world:=world_light.xml map:=$HOME/maps/wheel_map.yaml
```

**상세 설명**

- `nav.launch.py`: 시뮬, 지도 읽기, AMCL 위치 추정, Nav2 자율주행을 함께 시작합니다. AMCL은 관측한 벽과 저장 지도를 비교해 현재 위치를 추정합니다.
- `world:=world_light.xml`: 시뮬레이터가 읽을 **물리 환경**입니다.
- `map:=$HOME/maps/wheel_map.yaml`: Nav2가 읽을 **지도 파일**입니다. `$HOME`은 홈 경로로 바뀌므로 4절에서 저장한 지도를 선택합니다.
- `world`와 `map`은 다릅니다. 앞은 로봇이 실제로 움직이는 가상 공간, 뒤는 길 찾기와 위치 추정에 사용할 지도입니다.

**이 한 줄이 하는 일:** 직접 만든 지도를 읽고, 사용자가 지정할 목표까지 스스로 이동할 준비를 합니다. 명령 실행만으로 로봇이 출발하지는 않습니다.

**실제로 이동시키기:** RViz의 **Nav2 Goal**을 선택하고 빈 공간에 목표 위치·방향을 지정합니다. 현재 위치가 어긋나면 **2D Pose Estimate**로 초기 위치를 맞춥니다.

**확인:** 경로가 표시되고 로봇이 목표까지 도착하는지 관찰합니다. 직접 만든 지도가 잘못됐을 때 비교하려면 launch를 종료하고 위 명령에서 `map:=...`만 빼고 실행하세요. 그러면 패키지에 포함된 기본 지도를 사용합니다.

## 6. 프로그램으로 여러 지점을 순찰시키기

T1의 Nav2는 유지합니다. 앞 목표 주행이 끝난 뒤 T2에서 실행합니다.

```bash
ros2 run mobile_openarm_apps patrol --ros-args -p loops:=1 -p mode:=waypoints
```

**상세 설명**

- `mobile_openarm_apps`: 로봇의 작업 앱이 들어 있는 패키지입니다.
- `patrol`: 미리 정의한 위치들을 순서대로 방문하는 실행 프로그램입니다. 이 저장소에서는 Python 함수에 연결해 등록한 이름이며, 명령에 `patrol.py`라고 적지 않습니다.
- `--ros-args`: ROS 노드 설정을 전달합니다.
- `-p loops:=1`: `loops` 파라미터를 1로 설정하여 순찰 경로를 한 번 반복합니다.
- `-p mode:=waypoints`: 각 목표 지점을 순서대로 방문하는 액션 방식을 선택합니다.

**이 한 줄이 하는 일:** 이미 켜진 Nav2에 여러 이동 목표를 보내 순찰하고, 마지막에 home으로 복귀합니다. 시뮬레이터나 Nav2를 새로 실행하는 명령은 아닙니다.

**확인:** 앱 로그의 방문 진행과 결과, 실제 로봇의 home 복귀를 확인합니다. 직접 만든 지도와 위치 정의가 맞지 않으면 주행이 실패할 수 있습니다.

## 오늘 실행한 기능은 소스에서 어디에 있는가?

| 기능 | 주요 소스 | 역할 |
|---|---|---|
| 휠+양팔 모델 | `src/mobile_openarm_sim/tools/build_mjcf.py` | OpenArm 모델에 바퀴 베이스 결합 |
| 물리·바퀴·센서 | `src/mobile_openarm_sim/mobile_openarm_sim/sim_core.py` | 속도 명령을 좌우 바퀴 제어로 바꾸고 센서 계산 |
| ROS 연결 | `src/mobile_openarm_sim/mobile_openarm_sim/bridge_node.py` | ROS 명령을 받고 상태·센서·TF 발행 |
| 좌표계 모델 | `src/mobile_openarm_sim/urdf/mobile_openarm.urdf` | 링크와 관절의 연결·장착 위치 정의 |
| 실행 구성 | `src/mobile_openarm_sim/launch/` | sim/slam/nav 단계별 노드 실행 |
| 순찰 | `src/mobile_openarm_apps/mobile_openarm_apps/patrol.py` | Nav2에 순찰 목표 요청 |
| 위치 정의 | `src/mobile_openarm_sim/config/locations.yaml` | home·작업대·순찰 지점 정의 |

**완료 기준:** 직접 운전, 카메라 상대 좌표 조회, 지도 저장, 목표 주행, 순찰 결과를 확인했습니다. 모두 종료한 뒤 [2일차](wheel-day2.md)로 이동하세요.

추가 개념·문제 해결이 필요할 때만 [기존 상세 실습서](../labs/README.md)를 참고하세요.
