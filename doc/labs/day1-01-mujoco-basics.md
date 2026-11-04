# DAY 1-01 Gazebo vs MuJoCo — 시뮬레이터 이해하기

## 이 장의 미션

> ROS 2 없이 **MuJoCo 뷰어만으로** 로봇과 실습 월드를 열고, 물체를 밀어 보고, 접촉점과 관절을 눈으로 확인한다.
> 그리고 자체 점검 프로그램(selftest)으로 시뮬레이션 모델이 정상인지 확인한다.

## 이 장에 나오는 용어

- **MuJoCo** (Multi-Joint dynamics with Contact, 접촉이 있는 다관절 동역학) — 물리 시뮬레이터
- **MJCF** (MuJoCo Format, MuJoCo XML 모델 형식) — MuJoCo가 읽는 로봇·환경 설명 파일
- **Gazebo** — ROS와 오래 함께 써 온 다른 로봇 시뮬레이터
- **URDF** (Unified Robot Description Format, 통합 로봇 설명 형식) — ROS가 쓰는 로봇 구조 XML. 링크(부품)와 조인트(관절)의 나무 구조

전체 목록은 [용어집](glossary.md)에 있습니다.

## 1. 개념

### 로봇 시뮬레이터가 하는 일

시뮬레이터는 **가상의 물리 세계**입니다. 로봇의 질량, 관절, 모터, 바닥 마찰, 충돌을 계산해서 "명령을 주면 실제로 어떻게 움직일지"를 보여 줍니다. 진짜 로봇을 부수지 않고 코드를 시험할 수 있습니다.

### Gazebo와 MuJoCo 비교

| | Gazebo (gz-sim) | MuJoCo |
|---|---|---|
| 모델 파일 | SDF / URDF | **MJCF** (MuJoCo용 XML) |
| 장점 | ROS와 연동 생태계, 센서 플러그인, 큰 환경 | **접촉·마찰 계산이 정확하고 빠름**, 강화학습·조작(manipulation) 연구 표준 |
| ROS 연결 | `ros_gz_bridge` | 직접 브리지를 짜거나 `mujoco_ros2_control` |
| 이 실습 | — | **MuJoCo 3.5** + 직접 작성한 Python 브리지(02장) |

이 실습은 상자를 손가락으로 집는 **조작**이 핵심이라 접촉 계산이 좋은 MuJoCo를 씁니다.

### MJCF 파일 구조

MuJoCo 모델은 XML 한 파일(또는 `include` 로 나눈 여러 파일)입니다.

| 태그 | 뜻 |
|---|---|
| `<body>` | 강체 하나 (로봇 링크, 상자, 작업대) |
| `<joint>` | 몸체 사이 관절 (회전 `hinge`, 직선 `slide`, 자유 `freejoint`) |
| `<geom>` | 충돌·표시용 모양 (box, cylinder, mesh …) |
| `<actuator>` | 모터. 관절에 힘이나 목표 위치·속도를 준다 |
| `<sensor>`, `<site>`, `<camera>` | 센서 부착 위치, 카메라 |

## 2. 실제로 쓰는 파일

| 파일 | 하는 일 |
|---|---|
| `src/mobile_openarm_sim/tools/build_mjcf.py` | enactic의 OpenArm v1 양팔 MJCF를 가져와 **바퀴 베이스·LiDAR·카메라를 붙여** `mjcf/mobile_openarm.xml` 을 만든다 (`04_setup_workspace.sh` 가 이미 실행함) |
| `src/mobile_openarm_sim/tools/build_world.py` | 방·벽·작업대·ArUco 상자·사람을 넣은 `mjcf/world.xml` 을 만든다. 저사양용 `world_light.xml` 도 만든다 |
| `src/mobile_openarm_sim/mjcf/world.xml` | 이 장에서 여는 파일. 첫 줄에서 `mobile_openarm.xml`(로봇)을 `include` 한다 |
| `python3 -m mujoco.viewer` | MuJoCo 패키지에 들어 있는 기본 뷰어 프로그램 |
| `src/mobile_openarm_sim/tools/selftest.py` | ROS 없이 모델을 직접 돌려 14개 항목(바퀴 주행, LiDAR, 카메라·ArUco, 팔 관절, 집기 등)을 점검 |

## 3. 따라하기

### 단계 1 — T1: 월드 파일 열어 보기

```bash
cd ~/openarm_mobile_ws
python3 -m mujoco.viewer --mjcf=src/mobile_openarm_sim/mjcf/world.xml
# 저사양: --mjcf=src/mobile_openarm_sim/mjcf/world_light.xml
```

- **언제**: ROS 없이 모델만 볼 때. 이 명령은 ROS 2 노드가 아니므로 `/scan` 같은 토픽은 나오지 않습니다.
- `--mjcf=` 뒤의 경로가 열 파일입니다. 빌드 후 설치본 경로는 `$(ros2 pkg prefix --share mobile_openarm_sim)/mjcf/world.xml` 이고 내용은 같습니다.
- 창이 뜨면 물리 시뮬레이션이 **바로 시작**됩니다. 로봇은 모터 명령이 없으니 제자리에 서 있고, 팔은 처음 자세를 유지합니다.

### 단계 2 — 뷰어 조작 익히기

| 조작 | 동작 |
|---|---|
| 왼쪽 드래그 | 시점 회전 |
| 오른쪽 드래그 | 시점 이동 |
| 휠 | 확대·축소 |
| `Space` | 일시정지 / 재개 |
| 일시정지 중 `→` | 한 단계(timestep)씩 진행 |
| `Backspace` | 처음 상태로 리셋 |
| 몸체 **더블클릭** | 그 몸체 선택 (선택되면 밝게 표시) |
| 선택 후 `Ctrl` + **오른쪽 드래그** | 선택한 몸체를 **잡아 끌기** (힘 가하기) |
| 선택 후 `Ctrl` + **왼쪽 드래그** | 선택한 몸체를 **돌리기** (토크 가하기) |
| `Tab` / `Shift+Tab` | 왼쪽 / 오른쪽 설정 패널 열고 닫기 |
| 왼쪽 패널 Rendering | Contact Point, Contact Force, Joint 등 표시 켜기 |
| `F1` | 단축키 도움말 |

### 단계 3 — 미션 수행

1. **작업대 A 찾기**: 오른쪽 드래그와 휠로 북동쪽 작업대로 이동합니다. 빨강·초록·파랑 상자 위에 흑백 ArUco 마커가 붙어 있습니다.
2. **상자 밀기**: 빨간 상자를 더블클릭 → `Ctrl` + 오른쪽 드래그로 끌어 작업대 아래로 떨어뜨려 봅니다. 떨어지며 튕기고 구르는 것이 MuJoCo의 접촉 계산입니다.
3. **접촉점 보기**: `Tab` 으로 왼쪽 패널을 열고 Rendering에서 **Contact Point** 와 **Contact Force** 를 켭니다. 바퀴·캐스터와 바닥이 닿는 점, 상자와 작업대가 닿는 점에 표시가 나타납니다.
4. **관절 보기**: Rendering에서 **Joint** 를 켜면 팔의 회전축(관절 7개 × 2팔), 바퀴 축이 화살표로 보입니다.
5. **팔 밀어 보기**: 팔 링크 하나를 더블클릭해 `Ctrl` + 오른쪽 드래그 → 모터가 원래 자세로 되돌리려고 버팁니다(위치 제어 액추에이터).
6. `Backspace` 로 리셋하면 상자가 제자리로 돌아옵니다.

창을 닫으면(또는 T1에서 Ctrl+C) 끝납니다.

### 단계 4 — T1: 자체 점검(selftest)

```bash
cd ~/openarm_mobile_ws
python3 src/mobile_openarm_sim/tools/selftest.py                 # 기본 world.xml
python3 src/mobile_openarm_sim/tools/selftest.py world_light.xml # 저사양 월드
```

- **무엇을 하나**: 창 없이 시뮬레이션을 빠르게 돌리며 항목마다 `[PASS]` / `[FAIL]` 을 출력합니다. 1 m 직진 시 오도메트리 오차, LiDAR 거리, 카메라로 본 ArUco 위치 오차, 팔 관절 추종, 상자 집기·놓기 등을 봅니다.
- **기대 결과**: 마지막 줄 `총 14개 중 실패 0개`. 카메라 항목만 실패하면 OpenGL(그래픽 드라이버) 문제입니다. 설치 가이드의 트러블슈팅을 보세요.

## 4. 미션 체크리스트

- [ ] 뷰어로 world.xml을 열어 방, 작업대 2개, 상자 3개, 로봇을 찾았다
- [ ] 상자를 끌어서 떨어뜨리고, `Backspace` 로 되돌렸다
- [ ] Contact Point / Joint 표시를 켜서 접촉점과 관절축을 확인했다
- [ ] selftest 가 `실패 0개` 로 끝났다

## 5. 확인 질문

1. MuJoCo 모델 파일 형식의 이름은? → MJCF
2. `world.xml` 은 로봇 모델을 어떻게 불러오나? → `<include file="mobile_openarm.xml"/>`
3. 이 실습에서 Gazebo 대신 MuJoCo를 쓴 가장 큰 이유는? → 집기(접촉·마찰) 계산이 정확하고 빠르다

## 6. 잘 안 될 때

| 증상 | 해결 |
|---|---|
| `No module named mujoco` | `pip install --user --break-system-packages mujoco==3.5.0` (Docker는 이미 설치됨) |
| 창이 검거나 바로 꺼짐 | 노트북: `glxinfo -B` 로 NVIDIA가 쓰이는지 확인, Wayland면 Xorg 세션으로 로그인. Docker: VNC 화면 안에서 실행했는지 확인 |
| 매우 느림 | `world_light.xml` 사용 |

## 7. 더 해보기

- `mjcf/world.xml` 을 텍스트 편집기로 열어 `pick_table` body의 `pos` 를 찾아보기
- `tools/build_world.py` 의 `BOXES` 에 상자를 하나 더 추가하고 `python3 tools/build_world.py --out mjcf --config config` 로 월드를 다시 만들어 보기 (패키지 폴더 `src/mobile_openarm_sim` 에서 실행)
