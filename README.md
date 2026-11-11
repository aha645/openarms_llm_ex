# OpenArm 이동 조작 실습 — MuJoCo + ROS 2 Jazzy

**Ubuntu 24.04 + ROS 2 Jazzy**에서 하반신(차동구동 휠 또는 TRON1 이족)과 OpenArm v1 양팔 상반신을 결합해, 주행·팔 제어·물체 운반·자연어 미션을 실습하는 저장소입니다. 저장소 아래 `mobile_openarm/`(휠형)과 `tron1_openarm/`(TRON1 이족)이 각각 colcon 워크스페이스이며, TRON1은 휠형 위에 겹쳐(overlay) 빌드합니다.

> `planner:=offline`은 LLM이 아니라 **키워드 규칙 기반 계획기**입니다. 실제 LLM API 연결은 2일차의 선택 실습입니다. 실로봇이 아닌 **MuJoCo 시뮬레이션**을 대상으로 합니다.

## 1. 처음 시작: 환경 설정은 하나만 선택

| 환경 | 설치부터 최초 통합 실행까지 |
|---|---|
| 노트북: Ubuntu 24.04 네이티브 (Intel 내장 / GTX 1050 급) | **[노트북 환경 설정](doc/setup/ubuntu24-native.md)** |
| NVIDIA DGX Spark: DGX OS(Ubuntu 24.04), ARM64, GB10 | **[DGX Spark 환경 설정](doc/setup/dgx-spark.md)** — `apt upgrade` 주의, 명령 한 줄씩 진행, TRON1 JAX GPU 학습 포함 |
| Intel iMac: macOS + Docker Desktop, Ubuntu 24.04 컨테이너 | **[iMac Docker 환경 설정](doc/setup/imac-docker.md)** |

세 문서는 ROS 2 Jazzy 준비 → Git clone → 모델 에셋 준비 → colcon build → **휠형 `full_system.launch.py planner:=offline` 실행**까지 다룹니다. 환경 설정 후에는 아래 교육 문서로 이동하세요. Docker에서는 이후 모든 ROS 명령을 컨테이너 안에서 실행합니다.

Python 의존성은 가상환경 두 개로 나눕니다. 설치 스크립트가 자동으로 만들며, 구성과 이유는 [Python 가상환경](doc/setup/python-venv.md)에 정리했습니다.

| 가상환경 | 만드는 스크립트 | 용도 |
|---|---|---|
| `mobile_openarm/.venv-mobile` | `mobile_openarm/scripts/setup_venv_mobile.sh` (`04_setup_workspace.sh`가 호출) | MuJoCo·OpenAI·Pydantic v2·Flask — SLAM/Nav2/MoveIt 연동, LLM 시뮬레이션, 웹 대시보드 (휠형·TRON1 ROS 실행 공용). NumPy는 ROS apt 버전 고정 |
| `tron1_openarm/.venv-biped` | `tron1_openarm/scripts/setup_venv_biped.sh` (`11_train_policy.sh`가 호출) | MuJoCo + JAX(GPU) — 팔을 움직이는 상태의 TRON1+OpenArm 보행 PPO 학습. ROS와 분리, NumPy 2.x |

## 2. 교육: 사용할 하반신 선택

환경 설정이 끝났다는 전제에서 개념 → 실행 명령(명령 풀이 포함) → 확인 기준 → 소스 구조를 설명합니다.

| 시스템 | 1일차 | 2일차 |
|---|---|---|
| 차동구동 휠 + OpenArm ([목차·공통 규칙](doc/wheel/README.md)) | [01 MuJoCo](doc/wheel/day1-01-mujoco-basics.md) · [02 브리지](doc/wheel/day1-02-bridge.md) · [03 TF](doc/wheel/day1-03-description-tf.md) · [04 SLAM](doc/wheel/day1-04-slam-rviz.md) · [05 Nav2](doc/wheel/day1-05-nav2.md) · [06 순찰](doc/wheel/day1-06-patrol.md) | [01 MoveIt](doc/wheel/day2-01-moveit.md) · [02 ArUco 집기](doc/wheel/day2-02-aruco-pick-place.md) · [03 운반](doc/wheel/day2-03-nav-pick-place.md) · [04 LLM](doc/wheel/day2-04-llm-planner.md) · [05 Task Manager](doc/wheel/day2-05-task-manager.md) · [06 대시보드](doc/wheel/day2-06-dashboard.md) |
| TRON1 + OpenArm ([목차](doc/tron1/README.md)) | **[보행·브리지·TF·SLAM·Nav2·순찰](doc/tron1/day1.md)** | **[이족 팔 제어·집기·운반·통합 미션](doc/tron1/day2.md)** |

**TRON1은 별도 모델 생성과 두 소스 트리를 포함한 빌드가 필요합니다.** 휠형 `mobile_openarm/` 빌드만으로는 준비되지 않으며 `tron1_openarm/scripts/10_setup_tron1.sh`로 `tron1_openarm/` 워크스페이스를 추가로 빌드합니다. [TRON1 1일차의 사전 준비 확인](doc/tron1/day1.md)을 먼저 읽으세요. 교육 명령은 완주 기록이 있는 **Sole-foot(`foot:=sf`)**를 기준으로 제시하며, launch 기본값은 Point-foot(`pf`)임을 구분합니다. 기존 SF 성공 기록도 1회이며 반복 성공률을 보장하지 않습니다.

## 3. 소스 위치

```text
docker/                               # Intel iMac용 이미지·가상 데스크톱·Compose
doc/                                  # 설치·교육·설계 문서
mobile_openarm/                       # 휠형 colcon 워크스페이스
  scripts/                            # 노트북 설치(00~03), 휠형 모델 생성·빌드(04), .venv-mobile 생성
  src/
    mobile_openarm_sim/               # 휠 모델, 센서·물리, ROS 브리지, SLAM/Nav2 launch
    mobile_openarm_moveit_config/     # 휠형 URDF/SRDF 기반 MoveIt 설정
    mobile_openarm_apps/              # 공통 주행/팔 스킬, LLM 계획, 작업 관리자, 웹
  third_party/openarm_mujoco/         # (스크립트가 clone, git 제외) OpenArm 에셋
  .venv-mobile/  build/ install/ log/ # (생성물, git 제외)
tron1_openarm/                        # TRON1 colcon 워크스페이스 (mobile_openarm/install 위에 overlay)
  scripts/                            # TRON1 모델 준비·빌드(10), PPO 학습(11), .venv-biped 생성
  src/tron1_openarm_sim/              # 이족 모델, 정책 추론·물리·브리지, launch
  src/tron1_openarm_moveit_config/    # 이족용 MoveIt 설정
  rl/                                 # 선택적 PPO 학습 (기본 교육 범위 밖)
  tools/validation/                   # 시험·정답 기록 도구
  .venv-biped/  build/ install/ log/  # (생성물, git 제외)
```

자연어 실행 흐름:

```text
대시보드 또는 /task/command
  → task_manager → llm_planner(offline 또는 API) → 계획 검증
  → robot_skills → Nav2 / MoveIt → ROS 브리지 → MuJoCo
```

## 4. 심화 자료

- [TRON1 설계](doc/tron1/design.md) · [보행 PPO 학습](doc/tron1/locomotion-rl.md) · [시험·검증 기록](doc/tron1/validation-log.md): 모델 결합, 정책 학습, 흔들림 분석, 시각 피드백, SF 완주 이력
- [실로봇으로 넘어가기](doc/real-robot.md): 시뮬과 실제의 차이, 휠형 드라이버 교체, TRON1 한계와 전환 순서
- [Python 가상환경](doc/setup/python-venv.md): `.venv-mobile` / `.venv-biped` 구성과 빌드 방식
- [용어집](doc/glossary.md)

```text
doc/
  setup/      ubuntu24-native.md · dgx-spark.md · imac-docker.md · python-venv.md
  wheel/      README.md(공통 규칙) + 12장 (day1-01 … day2-06)
  tron1/      README.md · day1.md · day2.md · design.md · locomotion-rl.md · validation-log.md
  real-robot.md · glossary.md · images/
```

## 라이선스·출처

OpenArm 에셋: [enactic/openarm_mujoco](https://github.com/enactic/openarm_mujoco), TRON1 모델: [limxdynamics/tron1-robot-description](https://github.com/limxdynamics/tron1-robot-description). 기존 문서에서 프로젝트 코드와 해당 에셋의 라이선스를 Apache-2.0으로 안내합니다. 외부 에셋 재배포 시 각 upstream 라이선스를 확인하세요.
