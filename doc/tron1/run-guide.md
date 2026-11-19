# TRON1 + OpenArm 실행 안내 — 다른 컴퓨터에서 받아서 미션 돌리기

[TRON1 목차](README.md) · [1일차](day1.md) · [2일차](day2.md) · [검증 기록](validation-log.md) · [보행 학습](locomotion-rl.md)

이 문서는 저장소를 새 컴퓨터에 받아 **마지막으로 검증한 상태 그대로** 집기 시험과 전체 미션(걸어가 집기 → 들고 걷기 → 다른 작업대에 놓기)을 돌리는 방법입니다.
교육 절차의 설명은 [1일차](day1.md)·[2일차](day2.md)에 있고, 여기서는 실행 명령과 확인 방법만 다룹니다.

## 1. 저장소에 무엇이 들어 있나

새 컴퓨터에서 **인터넷으로 다른 원본을 받을 필요가 없습니다**. 실행에 필요한 것은 모두 저장소에 있습니다.

| 무엇 | 위치 | 비고 |
|---|---|---|
| 보행 정책 (Sole-foot, 교육 기준) | `tron1_openarm/src/tron1_openarm_sim/policies/tron1sf_openarm_policy.npz` | v3, 서 있을 때 기울기 2.3° ([보행 학습 13-5](locomotion-rl.md)) |
| 보행 정책 (Point-foot, 비교용) | `.../policies/tron1_openarm_policy.npz` | `foot:=pf` |
| 로봇·월드 모델 (MJCF) | `.../mjcf/`, `.../mjcf/sf/` | 생성 도구로 만든 결과 그대로 |
| 메시 (OpenArm·TRON1) | `.../mjcf/meshes/`, `.../mjcf/sf/meshes/`, `mobile_openarm/src/mobile_openarm_sim/mjcf/meshes/` | Apache-2.0, 출처: 각 `mjcf/MESHES_NOTICE.md` |
| ROS 용 URDF·SRDF | `.../urdf/`, `tron1_openarm_moveit_config/config/` | |
| 지도·Nav2 설정·위치 | `.../maps/`, `.../config/` | |

배포 정책이 맞는지 확인 (마지막 검증 때의 파일):

```bash
md5sum tron1_openarm/src/tron1_openarm_sim/policies/tron1sf_openarm_policy.npz
# e9593d270975860f634091cdd5fd289c  ← v3 (= runs/upright/seed1/final.npz)
```

## 2. 새 컴퓨터 준비

전제: Ubuntu 24.04 + ROS 2 Jazzy. 설치는 [Ubuntu 24.04 설치](../setup/ubuntu24-native.md) (Mac 은 [Docker](../setup/imac-docker.md), DGX Spark 는 [DGX](../setup/dgx-spark.md)).
GPU 가 없어도 돌지만 느립니다 (아래 7절).

```bash
git clone https://github.com/aha645/openarms_llm_ex.git
cd openarms_llm_ex
bash mobile_openarm/scripts/04_setup_workspace.sh      # .venv-mobile, 자체 점검, 바퀴형 빌드 (5~15 분)
bash tron1_openarm/scripts/10_setup_tron1.sh           # 모델·정책 확인, 자체 점검(PF·SF), TRON1 빌드
source tron1_openarm/install/setup.bash                # 새 터미널마다 (바퀴형 underlay 포함)
```

- 두 스크립트는 저장소의 모델을 **덮어쓰지 않습니다**. 자체 점검이 `총 10개 중 실패 0개`(PF·SF 각각)면 정상입니다.
- `third_party/` 폴더는 생기지 않습니다. 원본에서 모델을 다시 만드는 일은 8절(선택)입니다.
- 이미 설치한 컴퓨터에서 최신으로 바꿀 때: `git pull` 뒤 `bash tron1_openarm/scripts/10_setup_tron1.sh` (또는 `cd tron1_openarm && colcon build --symlink-install --base-paths src`).

## 3. 전체 미션 — 시험 스크립트로 한 번에

시뮬레이터 + Nav2 + MoveIt + 작업 관리자 + 오프라인 계획기를 띄우고, 자연어 명령 하나를 끝까지 실행한 뒤 **상자가 왼쪽 작업대 위에 있는지 정답으로 판정**하고 모두 끕니다.

```bash
VIEW=1 BOX_SOURCE=color bash tron1_openarm/tools/validation/mission_test.sh sf _m1
#   VIEW=1        MuJoCo 뷰어 + RViz 를 띄워 눈으로 확인 (빼면 화면 없이)
#   BOX_SOURCE    color = 상자를 색으로 인식 (마지막 검증 기준), aruco = 상자 윗면 마커
#   _m1           기록 이름 (로그 파일 이름에 붙음)
#   세 번째 인자로 명령을 바꿀 수 있음 (기본 "빨간 상자를 왼쪽 작업대로 옮기고 돌아와")
```

끝나면 마지막 줄에 판정이 나옵니다.

```
sf final box0 [-2.303  2.95   0.725] | on place table (...): True | max tilt 2.9 | final gt [...]
```

`on place table: True` 면 성공입니다. 마지막 검증(미션 24)은 458 s 에 완주했습니다 ([검증 기록 17절](validation-log.md)).
로그는 `/tmp/tron1_logs/` (`LOG_DIR=` 로 바꿈): `sf_full_m1.log`(전체 시스템), `sf_mission_m1.log`(명령 진행), `mrec_sf_m1.jsonl`(정답 기록 0.2 s 마다).

## 4. 전체 미션 — 직접 띄워서 명령 보내기

```bash
# 터미널 1: 전체 시스템 (Ctrl+C 로 끔)
ros2 launch tron1_openarm_sim full_system.launch.py foot:=sf planner:=offline box_source:=color rviz:=true
```

준비되면(약 20 s, 터미널에 `task_manager 준비 완료`) 명령을 보냅니다. 셋 중 하나:

```bash
# (a) 웹 대시보드: 브라우저 http://localhost:8080 → 명령 입력
# (b) 터미널 2:
source tron1_openarm/install/setup.bash
ros2 topic pub --once /task/command std_msgs/msg/String "data: '빨간 상자를 왼쪽 작업대로 옮기고 돌아와'"
# (c) 진행 기록까지 보기:
python3 tron1_openarm/tools/validation/mission_cli.py "초록 상자를 오른쪽 작업대로 옮기고 돌아와" 1500
```

| launch 인자 | 값 | 뜻 |
|---|---|---|
| `foot` | `sf` (기본) / `pf` | 발 형태 |
| `planner` | `offline` (기본) / `llm` | 자연어 → 단계 계획. `llm` 은 `OPENAI_API_KEY` (로컬 LLM 은 `OPENAI_BASE_URL`, `OPENAI_MODEL`) |
| `box_source` | `aruco` (기본) / `color` | 상자 인식 방식 |
| `world` | `world.xml` / `world_light.xml` | 저사양은 light |
| `viewer`, `rviz` | `true` / `false` | 화면 |
| `policy` | npz 경로 | 다른 보행 정책으로 (5절) |
| `camera_hz`, `rate_factor` | `10.0`, `1.0` | 느린 PC 는 `5.0`, `0.5` |

명령 예: "빨간 상자를 왼쪽 작업대로 옮기고 돌아와", "파란 상자를 왼쪽 작업대로 옮겨", "상자 모두 옮기고 돌아와".
'왼쪽/오른쪽 작업대' 는 **출발 지점에서 작업대들 쪽을 바라볼 때** 기준입니다 (시작할 때 작업 관리자가 `출발 지점 (x, y) 기준 작업대: 오른쪽=…, 왼쪽=…` 을 출력).

## 5. 집기 시험 — 작업대 앞에서 집고 놓기만

전체 미션보다 짧고(약 2 분) 반복 시험에 씁니다. 로봇을 집기 작업대 앞에 세운 채 시작합니다.

```bash
VIEW=1 BOX_SOURCE=color bash tron1_openarm/tools/validation/pick_test.sh sf _p1
# 상자 배치 바꾸기 (월드 x, y, yaw도) — 집을 상자 OBJ 가 로봇 오른쪽이면 오른팔, 왼쪽이면 왼팔
OBJ=box_1 BOXES="box_0=2.590,2.989,-8;box_1=2.438,2.988,-9" VIEW=1 BOX_SOURCE=color bash tron1_openarm/tools/validation/pick_test.sh sf _p2
# 검증 배치 여러 개 연속 (r1~r8 무작위 배치, p1·p2 놓을 자리 막힘): 결과 /tmp/tron1_logs/series_<접두어>.txt
BOX_SOURCE=color bash tron1_openarm/tools/validation/series_run.sh s1 r1 r2 r3 r4 r5 r6 r7 r8 p1 p2
```

마지막 줄의 `contacts` 에 집은 상자와 손가락만 있고(`openarm_right_left_finger-box_0` 등), `final box` 높이가 0.725 면 정상입니다.
마지막 검증: r1~r8·p1·p2 10/10, 평균 99 s ([검증 기록 17-1](validation-log.md)).

## 6. 다른 보행 정책으로 시험하기

```bash
ros2 launch tron1_openarm_sim full_system.launch.py foot:=sf policy:=/절대/경로/새정책.npz box_source:=color
# 시험 스크립트는 설치된 기본 정책을 쓰므로, 바꿔 보려면 파일을 바꿔 넣고 되돌린다
cp tron1_openarm/src/tron1_openarm_sim/policies/tron1sf_openarm_policy.npz /tmp/policy_backup.npz
cp 새정책.npz tron1_openarm/src/tron1_openarm_sim/policies/tron1sf_openarm_policy.npz   # symlink 설치라 다시 빌드 불필요
# ... 시험 ...
git checkout tron1_openarm/src/tron1_openarm_sim/policies/tron1sf_openarm_policy.npz     # 저장소 버전으로 되돌리기
```

정책 학습·평가는 [보행 학습](locomotion-rl.md) (`tron1_openarm/scripts/12_train_mjx.sh`, `rl/eval_policy.py`).

## 7. 느린 컴퓨터 · 문제 해결

| 증상 | 조치 |
|---|---|
| 시뮬레이터가 실시간보다 많이 느림 | `world:=world_light.xml camera_hz:=5.0`, 뷰어 끄기(`viewer:=false`), 그래도 느리면 `rate_factor:=0.5` |
| GPU 없음 (자체 점검의 렌더링 오류) | 스크립트가 `MUJOCO_GL=egl` → `osmesa` 순으로 자동 시도 |
| 이전 실행이 남아 새 실행이 이상함 (카메라 영상 멈춤 등) | `pkill -f tron1_bridge; pkill -f move_group; pkill -f tron1_cameras` 뒤 다시 실행 (시험 스크립트는 끝날 때 자동 정리) |
| `모델 파일이 없습니다` (10_setup) | 저장소를 다시 받거나 8절 재생성 |
| 미션에서 '마커/상자를 찾지 못함' | `sf_full_*.log` 의 `[dock]` 줄로 도착 위치 확인 ([검증 기록 17절](validation-log.md) 미션 22·23 사례) |

## 8. (선택) 원본에서 모델 다시 만들기

평소에는 필요 없습니다. OpenArm·LimX TRON1 원본을 새 버전으로 바꾸거나 생성 도구(`tools/*.py`)를 고쳤을 때만 씁니다.
원본은 `third_party/` 에 받고(git 에 안 올라감), 생성 파일을 덮어씁니다.

```bash
bash mobile_openarm/scripts/05_regen_models.sh          # 바퀴형: enactic/openarm_mujoco → MJCF·URDF·SRDF·지도
bash tron1_openarm/scripts/13_regen_tron1_models.sh     # TRON1: limxdynamics/tron1-robot-description + openarm_mujoco
git diff --stat                                          # 바뀐 생성 파일 확인
# 특정 원본 커밋으로: TRON1_DESC_REF=<커밋> OPENARM_MUJOCO_REF=<커밋> bash tron1_openarm/scripts/13_regen_tron1_models.sh
```

현재 저장소 모델은 아래 원본 커밋으로 다시 만들면 **바이트 단위로 같게** 나옵니다 (`MESHES_NOTICE.md`):
`enactic/openarm_mujoco` `6c455d3`, `limxdynamics/tron1-robot-description` `ac417d4`.
재생성 뒤에는 5절 집기 시험과 3절 미션으로 검증하고 커밋합니다.
