# 02. 모델 만들기 — 로봇 설명 가져오기 · xacro · MJCF 생성 · 상반신 고정

## 이 장의 미션

> LimX TRON1 Point-foot 의 xacro 를 가져와 **xacro → URDF → MJCF** 로 바꾸고, 그 위에 OpenArm 양팔을 **바퀴형과 같은 어깨 높이(1.0 m)** 로 고정한 MuJoCo 모델을 만든다. 왜 그 높이·위치인지 숫자로 확인한다.

## 이 장에 나오는 용어

- **xacro** (XML Macros) — URDF 를 변수·매크로·include 로 짧게 쓰는 도구. `xacro 파일 → (처리) → URDF`
- **URDF** (Unified Robot Description Format) — ROS 의 로봇 구조 XML. 링크(부품)와 조인트(관절)
- **MJCF** (MuJoCo Format) — MuJoCo 의 모델 XML. body(몸체)·joint·geom(충돌/표시 형상)·actuator(구동기)
- **관성 (inertia)** — 회전시키기 어려운 정도. 질량과 함께 넘어짐·보행에 직접 영향
- **무게중심 (COM, Center Of Mass)** — 전체 질량이 모였다고 볼 수 있는 점. 두 발 위에 있어야 넘어지지 않는다
- **PD 제어** (Proportional–Derivative) — 토크 = kp×(목표각−현재각) − kd×각속도. 관절을 목표각으로 끌어당기는 스프링+댐퍼
- **REP-105** (ROS Enhancement Proposal 105) — ROS 좌표계 이름 규칙 (`base_link`, `odom`, `map`)

## 1. 로봇 설명 파일 가져오기

```bash
cd ~/openarm_mobile_ws
bash tron1_openarm/scripts/10_setup_tron1.sh      # 아래 과정을 모두 자동으로
```

스크립트가 하는 첫 일은 LimX 공개 저장소를 받는 것입니다.

```bash
git clone --depth 1 https://github.com/limxdynamics/tron1-robot-description.git tron1_openarm/third_party/tron1-robot-description
```

이 실습에서 쓰는 것은 `pointfoot/PF_TRON1A/` 입니다.

| 폴더 | 내용 | 이 실습에서 |
|---|---|---|
| `xacro/robot.xacro` | 최상위: base_Link, IMU, 다리 매크로 2번 호출 | **include 해서 그대로 사용** |
| `xacro/const.xacro` | 치수·관절 한계(도)·질량·관성 (총 18.5 kg) | 정본 (수정 안 함) |
| `xacro/leg.xacro` | 다리 매크로: abad → hip → knee → foot (+contact) | 그대로 |
| `meshes/*.STL` | 외형 메시 9개 | `mjcf/meshes/tron1/` 로 복사 |
| `xml/robot.xml` | LimX 가 MuJoCo 용으로 손본 MJCF | **충돌 형상·관절 armature 값만** 가져옴 |
| `urdf/robot.urdf` | xacro 처리 결과 | 참고용 |

`tron1-mujoco-sim`(LimX 시뮬레이터)은 비교·참고용으로만 함께 받습니다. 그 시뮬레이터는 limxsdk 통신 규약으로 동작하고, 우리 실습은 ROS 2 브리지를 쓰기 때문입니다.

## 2. 최상위 xacro (새로 만든 파일)

`tron1_openarm/src/tron1_openarm_sim/urdf/tron1_openarm.urdf.xacro`

```xml
<robot name="tron1_openarm" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="tron1_dir" default=""/>
  <xacro:include filename="$(arg tron1_dir)/xacro/robot.xacro"/>     <!-- LimX 원본 그대로 -->
  <xacro:arg name="mount_x" default="0.08"/>                            <!-- 상반신 앞뒤 위치 (인자로 바꿀 수 있음) -->
  <xacro:property name="mount_z" value="0.030"/>                        <!-- TRON1 몸통 윗면 -->
  <joint name="base_Link_to_openarm_mount_link" type="fixed"> ... </joint>
  <link name="openarm_mount_link"> 0.20×0.24×0.014 m 알루미늄 판, 0.6 kg </link>
</robot>
```

**왜 LimX 파일을 고치지 않고 include 하나?** LimX 가 치수·질량을 고쳐 배포하면 `git pull` 만으로 반영되게 하기 위해서입니다. 우리가 추가한 것(마운트 판)은 이 파일 하나에만 있습니다.

**`$(find robot_description)` 처리:** LimX xacro 는 ROS 패키지 `robot_description` 경로를 찾습니다. 빌더는 임시 폴더에 xacro 를 복사해 이 부분을 실제 경로로 바꾼 뒤 처리합니다. 그래서 LimX 저장소를 ROS 패키지로 빌드하지 않아도 됩니다. ROS 워크스페이스에 넣어 빌드해도 동작합니다(LimX 저장소에 `package.xml` 이 있음).

직접 처리해 보기:

```bash
cd ~/openarm_mobile_ws/tron1_openarm/src/tron1_openarm_sim
cat mjcf/tron1_openarm.urdf.generated | head -40      # 빌더가 저장한 xacro 처리 결과
check_urdf mjcf/tron1_openarm.urdf.generated            # base_Link → abad/hip/knee/foot ×2 + openarm_mount_link
```

## 3. xacro → URDF → MJCF (`tools/build_tron1_openarm.py`)

```bash
python3 tools/build_tron1_openarm.py \
    --tron1 ../../third_party/tron1-robot-description/pointfoot/PF_TRON1A \
    --openarm ../../../third_party/openarm_mujoco/v1/openarm_bimanual.xml --out mjcf
```

출력 예:

```
[1-2] xacro → URDF → MJCF: TRON1 질량 19.184 kg, 몸체 11개, 관절 6개
[5] 상반신 접목: 기둥 0.347 m (원본 0.75 m), 기둥+브래킷 2.21 kg (원본 3.74 kg)
    전체 질량 33.58 kg = TRON1 18.58 + 어댑터 판 0.6 + 상반신 14.39
    어깨 높이 1.029 m (목표 0.999), 머리 카메라 1.192 m, LiDAR 0.580 m (base 높이 0.710 m 기준)
```

(출력의 높이는 내려놓기 전 시작 높이 0.71 m 기준입니다. 서 있는 높이 0.68 m에서는 어깨 0.999 m, 카메라 1.162 m, LiDAR 0.55 m 입니다.)

단계별로 하는 일과 근거:

| 단계 | 하는 일 | 근거 / 이유 |
|---|---|---|
| 1. xacro | 위 최상위 xacro → URDF 문자열 | 기구학·관성 정본 = LimX `const.xacro` |
| 2. URDF 정리 | `contact_L/R_Link` 제거 | 발끝 표시용 링크(질량 1e-6 kg, 관성 0). MuJoCo 가 "관성 고유값이 양수가 아님" 오류로 거부함. 실제 접촉은 `foot_*_Link` 의 반지름 3.2 cm 구가 담당 |
| 3. URDF → MJCF | `mujoco.MjModel.from_xml_path(urdf)` → `mj_saveLastXML` | MuJoCo 는 URDF 를 직접 읽는다. `<mujoco><compiler fusestatic="false" discardvisual="false" balanceinertia="true"/>` 를 넣어 링크를 합치지 않고, 외형 메시도 남기게 함 |
| 4. 충돌 형상 이식 | URDF 의 무릎 구 → LimX `robot.xml` 의 원통·상자 (`base_collision`, `abad/hip/knee_*_collision`) | URDF 에는 다리 충돌이 구 2개뿐이라 정강이·허벅지가 벽·작업대를 뚫는다. LimX 가 MuJoCo 에서 검증한 형상을 같은 몸체 이름으로 옮김 |
| 5. 관절 동특성 | 다리 관절 `armature=0.01`, `damping=0.01` | LimX `robot.xml` 기본값 (모터 회전자 관성). URDF 에는 0 |
| 6. 이름 | `base_Link` → `base_link`, 관절은 xacro 이름(`abad_L_joint`) | `base_link` 는 ROS REP-105 표준이고, 바퀴형 브리지·LiDAR·카메라 코드가 이 이름을 씀. 관절 이름은 xacro(=URDF) 쪽을 따름 (LimX MJCF 는 `abad_L_Joint`) |
| 7. 떠 있는 몸통 | `base_link` 에 `<freejoint name="base_free"/>` | URDF 루트는 땅에 고정됨 → 걸어 다니려면 6자유도 자유 관절 필요 |
| 8. 구동기 | 다리 6관절 `<position kp="42" kv="3.5" forcerange="-80 80"/>` | LimX 배포 설정 `params.yaml` (stiffness 42, damping 3.5, user_torque_limit 80). 정책이 이 PD 값으로 학습됨 |
| 9. 물리 간격 | 배포 2 ms, 학습 4 ms | 정책 50 Hz = 10 / 5 스텝. 학습은 속도를 위해 4 ms, 배포 2 ms 에서 다시 평가 (03장) |

## 4. 상반신 고정 — 어디에, 얼마나 높이

### 4-1. 높이: 어깨 1.08 m (바퀴형 1.0 m 보다 8 cm 높게)

| 값 | 바퀴형 | TRON1 | 계산 |
|---|---|---|---|
| 몸통(base_link) 높이 | 0.08 m (바퀴축) | **0.68 m** (서 있을 때) | LimX 학습 목표 `base_height_target 0.68` |
| OpenArm 원점(body_link0) 높이 | 0.302 m | 0.383 m | = 어깨 − 0.697 (OpenArm 모델의 `arm_origin` site 높이) |
| 어깨(openarm_*_link0) 높이 | 0.999 m | **1.08 m** | `SHOULDER_HEIGHT` |
| 어깨 앞뒤 위치 (base_footprint 기준) | +0.12 m | **+0.08 m** | `MOUNT_X` (4-3) |
| 머리 카메라 | 1.162 m, 50° 숙임 | 1.243 m, 50° 숙임 | body_link0 + 0.86 |

**처음 설계(어깨 0.999 m, 바퀴형과 똑같이)와 바꾼 이유**

처음에는 DAY 2 조작 코드의 수치(작업대 0.70 m, 상자 앞 거리 `IDEAL_REACH 0.52 m`, 도킹 거리 `DOCK_GAP 0.40 m`, `carry` 자세)를 그대로 쓰려고 어깨를 바퀴형과 같은 0.999 m 에 맞췄습니다. 통합 시험에서 팔을 내린 `home` 자세의 손가락이 TRON1 **엉덩이 모터(abad 링크, 윗면 약 0.47 m)** 에 닿는 것이 드러났습니다 (4-3). 손끝 높이는 어깨 − 0.63 m 이므로, 어깨를 1.08 m 로 올려 손끝(0.45 m)이 모터 위로 지나가게 했습니다. 앞으로 8 cm 옮긴 것과 합쳐 손–다리 최소 거리는 **6.0 cm** 입니다 (걷는 동안 측정).

8 cm 높아져도 앱 코드는 그대로입니다. 작업대 상자까지 어깨에서의 거리는 √(0.44² + 0.355²) ≈ 0.57 m 로, OpenArm 팔 길이(약 0.63 m) 안입니다. 바퀴형은 0.48 m 였습니다. 무게중심이 높아지는 만큼 보행은 어려워지므로 이 모델로 정책을 다시 미세조정했습니다 (03장).

OpenArm 원본(75 cm 기둥)을 TRON1 위에 그대로 세우면 어깨가 1.40 m 가 됩니다. 그러면 작업대 상자까지 거리가 팔 길이를 넘고, 무게중심도 더 높아집니다. 그래서 기둥은 짧게 바꿉니다 (4-2).

### 4-2. 기둥 교체

OpenArm 원점(body_link0)은 기둥 밑동에 있습니다. 어깨를 1.08 m 에 두면 원점은 TRON1 몸통 안쪽(0.383 m)에 오게 됩니다. 그래서 기둥 메시(75 cm, 충돌 메시 포함)를 빼고 **TRON1 윗면 판(0.717 m)부터 어깨 브래킷 윗면(1.145 m)까지 약 0.43 m 짜리 상자 기둥**으로 바꿨습니다. 팔·그리퍼·어깨 관절은 원본 그대로입니다.

| 질량 가정 | 값 | 근거 |
|---|---|---|
| 원본 body_link0 | 3.74 kg | openarm_bimanual.xml |
| 그중 어깨 브래킷·배선 몫 (남김) | 0.9 kg | 가정 (`BRACKET_MASS`) |
| 기둥 몫 | 2.84 kg × 0.347/0.75 = 1.31 kg | 길이에 비례한다고 가정 |
| 새 body_link0 | **2.21 kg** | 1.31 + 0.9 |
| 상반신 전체 (기둥+양팔+그리퍼) | **14.39 kg** | 팔 하나 6.09 kg × 2 |

실제 기구를 만들 때는 기둥을 실측해 이 두 숫자(`BRACKET_MASS`, `ORIG_COLUMN_LEN`)를 고치면 됩니다.

### 4-3. 앞뒤 위치: 무게중심 vs 내린 손의 틈 (`mount_x = 0.08 m`)

처음에는 무게중심만 보고 정했습니다. TRON1 만 세워 두고 측정한 값입니다 (LimX 사전학습 정책, 명령 0):

```
두 발 평균 위치 (base_link 기준) x = -0.017 m,  TRON1 무게중심 x = -0.003 m
```

상반신 무게중심은 기둥 축에서 x = −0.005 m 이므로, `mount_x = 0.0` 이면 전체 무게중심이 두 발 사이에 옵니다. **그런데 통합 시험에서 문제가 드러났습니다.** 팔을 내린 `home` 자세에서 **손가락이 TRON1 엉덩이 모터(abad 링크)에 닿았습니다.**

- MoveIt 이 시작 자세를 충돌로 판정해 모든 팔 계획을 거부했습니다 (`START_STATE_IN_COLLISION: abad_L_Link - openarm_left_left_finger`).
- 시뮬레이터에서도 손이 엉덩이를 눌러 몸통이 8° 기울었습니다.
- 학습 모델은 상반신 충돌을 꺼 두었기 때문에 학습 중에는 드러나지 않았습니다.

그래서 걷는 동안 팔–다리 최소 거리를 재며 `mount_x` 를 바꿔 봤습니다 (`--mount-x` 인자):

| mount_x | 손–엉덩이 최소 거리 | 몸통 기울기 | 판단 |
|---|---|---|---|
| 0.00 m | 0 (닿음) | 8.1° | ✗ |
| 0.05 m | 0 (닿음) | 6.6° | ✗ |
| 0.08 m | 2.1 cm | 4.6° → 재학습 후 몸을 젖혀 11° 기울며 다시 닿음 | ✗ |
| **0.08 m + 어깨 1.08 m** | **6.0 cm** | (재학습 정책, 03장) | **채택** |
| 0.10 m | 0 (기울어진 채 다시 닿음) | 3.3° | ✗ |

최종 위치(0.08 m)에서는 전체 무게중심이 발보다 약 3.4 cm 앞에 옵니다 (14.4 kg × 0.08 / 33.6 kg). 정책은 몸을 약간 뒤로 기울여 균형을 잡고, 이 모델로 다시 미세조정했습니다 (03장). 바퀴형도 팔 도달거리를 위해 몸통을 12 cm 앞에 두었으므로(`TORSO_X 0.12`), 팔과 작업대의 관계는 오히려 바퀴형에 가까워집니다.

**교훈:** 장착 위치는 무게중심만이 아니라 **팔의 기본 자세가 다리와 부딪히지 않는지**도 함께 봐야 합니다. 학습용 모델에서 충돌을 끄더라도 배포 모델로 반드시 다시 확인하세요.

### 4-4. 센서 위치

| 센서 | 위치 | 이유 |
|---|---|---|
| IMU | base_link 원점 | TRON1 실제 IMU 위치 (LimX `imu_Link`) |
| LiDAR | base_link 기준 (0.17, 0, −0.13) → 지면 0.55 m | 월드의 가장 낮은 장애물인 **상자 더미가 0.6 m** 라 그보다 낮아야 보인다. 0.13 m 아래·몸통 앞쪽이라 내린 팔(어깨 아래 0.6 m 까지)·다리를 대부분 피한다 |
| 머리 카메라 | body_link0 기준 (0.10, 0, 0.86), 50° 숙임 | 바퀴형과 같은 높이·각도 → ArUco 거리·각도 조건이 같음 |

시뮬레이터의 LiDAR 는 로봇 자기 몸을 무시합니다(월드 geom 그룹만 감지). 실제 로봇에서는 뒤쪽 다리가 스캔 일부(뒤쪽 약 ±40°)를 가리므로 각도 마스크를 넣어야 합니다 → [05장](05-limits-real-robot.md)

### 4-5. 충돌 제외

기둥·마운트 판·어깨 첫 링크는 TRON1 몸통 상자와 겹치게 붙어 있어 `<contact><exclude>` 로 서로 충돌하지 않게 했습니다. 팔 ↔ 다리 충돌은 **켜져 있습니다** (팔을 내린 채 걸으면 실제로 부딪힐 수 있으므로).

## 5. 생성되는 파일

| 파일 | 용도 |
|---|---|
| `mjcf/tron1_openarm.xml` | 로봇 (시각 메시 포함) |
| `mjcf/tron1_openarm_light.xml` | 저사양 (시각 메시 제거, 충돌 형상 표시) |
| `mjcf/tron1_openarm_train.xml` | 강화학습용: 저사양 + 상반신 충돌 끔 + 바닥 + 4 ms |
| `mjcf/world.xml`, `world_light.xml` | 실습 월드 (`tools/build_world_tron1.py`: 바퀴형 build_world 재사용, include 만 교체) |
| `urdf/tron1_openarm.urdf` | ROS 용 (`tools/make_urdf_tron1.py`: 최종 MJCF → URDF, base_footprint→base_link 0.68 m) |
| `../tron1_openarm_moveit_config/config/tron1_openarm.srdf` | MoveIt (바퀴형 make_srdf.py 재사용, 다리 포함 자기충돌 샘플링) |

### ROS 용 URDF 를 xacro 결과로 바로 쓰지 않는 이유

ROS 쪽 URDF 에는 TRON1 다리뿐 아니라 OpenArm 팔·그리퍼, 카메라·LiDAR 프레임까지 들어가야 하고, 시뮬레이터와 **기구학이 1:1 로 같아야** 합니다. 그래서 바퀴형과 같은 원칙으로 **최종 MJCF 를 정본**으로 두고 URDF 를 다시 뽑습니다. xacro 는 "하반신 정본 → MJCF" 단계에서 쓰입니다.

## 6. 확인하기

```bash
cd ~/openarm_mobile_ws/tron1_openarm/src/tron1_openarm_sim
python3 -m mujoco.viewer --mjcf=mjcf/world.xml          # 로봇 확인 (정책이 없어 다리 힘이 0 → 곧 주저앉음, 정상)
check_urdf urdf/tron1_openarm.urdf | head -20          # base_footprint → base_link → 다리 / openarm_mount_link → 팔
python3 tools/selftest_tron1.py                         # 정책으로 서기·걷기·회전·LiDAR·ArUco·팔 동작 점검
```

뷰어에서 다리에 힘이 없는 것이 정상입니다. 다리는 보행 정책이 매 20 ms 마다 목표각을 줘야 서기 때문입니다 (03장).

## 확인 질문

1. LimX xacro 를 수정하지 않고 include 한 이유는? → 원본 업데이트를 그대로 받고, 우리 추가분을 분리하기 위해
2. 어깨 높이를 1.0 m 로 맞춘 두 가지 이유는? → DAY2 조작 수치를 그대로 쓰기 위해, 무게중심을 낮추기 위해
3. LiDAR 를 0.55 m 에 둔 이유는? → 0.6 m 높이 상자 더미를 보기 위해
4. URDF → MJCF 변환 후 충돌 형상을 LimX robot.xml 에서 가져온 이유는? → URDF 의 충돌이 구 2개뿐이라서
