# OpenArm 이동 조작 실습 — MuJoCo + ROS 2 Jazzy

**Ubuntu 24.04 + ROS 2 Jazzy**에서 하반신(차동구동 휠 또는 TRON1 이족)과 OpenArm v1 양팔 상반신을 결합해, 주행·팔 제어·물체 운반·자연어 미션을 실습하는 저장소입니다. 저장소 자체가 colcon 워크스페이스입니다.

> `planner:=offline`은 LLM이 아니라 **키워드 규칙 기반 계획기**입니다. 실제 LLM API 연결은 2일차의 선택 실습입니다. 실로봇이 아닌 **MuJoCo 시뮬레이션**을 대상으로 합니다.

## 1. 처음 시작: 환경 설정은 하나만 선택

| 환경 | 설치부터 최초 통합 실행까지 |
|---|---|
| 현재 노트북: Ubuntu 24.04 네이티브, Intel HD 630 / GTX 1050 | **[노트북 환경 설정](doc/setup/ubuntu24-native.md)** |
| Intel iMac: macOS + Docker Desktop, Ubuntu 24.04 컨테이너 | **[iMac Docker 환경 설정](doc/setup/imac-docker.md)** |

두 문서는 ROS 2 Jazzy 준비 → Git clone → 모델 에셋 준비 → colcon build → **휠형 `full_system.launch.py planner:=offline` 실행**까지만 다룹니다. 환경 설정 후에는 아래 교육 문서로 이동하세요. Docker에서는 이후 모든 ROS 명령을 컨테이너 안에서 실행합니다.

## 2. 교육: 사용할 하반신 선택

환경 설정이 끝났다는 전제에서 **실행 명령 → 테스트할 기능 → 확인 기준 → 소스 구조**를 일차별로 설명합니다.

| 시스템 | 1일차 | 2일차 |
|---|---|---|
| 차동구동 휠 + OpenArm | **[시뮬·브리지·TF·SLAM·Nav2·순찰](doc/course/wheel-day1.md)** | **[MoveIt·집기·운반·LLM·통합 미션](doc/course/wheel-day2.md)** |
| TRON1 + OpenArm | **[보행·브리지·TF·SLAM·Nav2·순찰](doc/course/tron1-day1.md)** | **[이족 팔 제어·집기·운반·통합 미션](doc/course/tron1-day2.md)** |

**TRON1은 별도 모델 생성과 두 소스 트리를 포함한 빌드가 필요합니다.** 휠형의 `colcon build --base-paths src`만으로는 준비되지 않습니다. [TRON1 1일차의 사전 준비 확인](doc/course/tron1-day1.md)을 먼저 읽으세요. 교육 명령은 완주 기록이 있는 **Sole-foot(`foot:=sf`)**를 기준으로 제시하며, launch 기본값은 Point-foot(`pf`)임을 구분합니다. 기존 SF 성공 기록도 1회이며 반복 성공률을 보장하지 않습니다.

## 3. 소스 위치

```text
scripts/                              # 노트북 설치·휠형 모델 생성·빌드
docker/                               # Intel iMac용 이미지·가상 데스크톱·Compose
src/
  mobile_openarm_sim/                 # 휠 모델, 센서·물리, ROS 브리지, SLAM/Nav2 launch
  mobile_openarm_moveit_config/       # 휠형 URDF/SRDF 기반 MoveIt 설정
  mobile_openarm_apps/                # 공통 주행/팔 스킬, LLM 계획, 작업 관리자, 웹
tron1_openarm/
  scripts/                           # TRON1 모델 준비·빌드, 선택적 정책 학습
  src/tron1_openarm_sim/              # 이족 모델, 정책 추론·물리·브리지, launch
  src/tron1_openarm_moveit_config/    # 이족용 MoveIt 설정
  rl/                                # 선택적 PPO 학습 (기본 교육 범위 밖)
  tools/validation/                  # 시험·정답 기록 도구
```

자연어 실행 흐름:

```text
대시보드 또는 /task/command
  → task_manager → llm_planner(offline 또는 API) → 계획 검증
  → robot_skills → Nav2 / MoveIt → ROS 브리지 → MuJoCo
```

## 4. 상세·심화 자료 (필수 읽기 아님)

- [기존 챕터별 상세 실습서](doc/labs/README.md): 개념, 개별 명령 풀이, 추가 미션, 문제 해결
- [용어집](doc/labs/glossary.md)
- [TRON1 설계·학습·검증 기록](doc/tron1/README.md): 모델 결합, 정책 학습, 흔들림, 시각 피드백, SF 완주 이력
- [문서 정리 결과와 확인이 필요한 사항](doc/document-review.md): 기존 문서 보존 정책, 중복과 불일치, 미검증 범위

기존 종합 가이드와 상세 실습서는 삭제하지 않았습니다. **설치·교육의 기본 동선은 위의 환경 설정 2개와 일차별 교육 4개**를 따르세요.

## 라이선스·출처

OpenArm 에셋: [enactic/openarm_mujoco](https://github.com/enactic/openarm_mujoco), TRON1 모델: [limxdynamics/tron1-robot-description](https://github.com/limxdynamics/tron1-robot-description). 기존 문서에서 프로젝트 코드와 해당 에셋의 라이선스를 Apache-2.0으로 안내합니다. 외부 에셋 재배포 시 각 upstream 라이선스를 확인하세요.
