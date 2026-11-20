# PPO 학습 경로와 배포 v3 설정 조사

## 파일명과 실행 장치

기존 작업 트리에서 이미 아래 이름으로 변경되어 있었다. CPU/GPU는 **물리 rollout 장치**를 구분한다.

| 이전 이름 | 현재 실행 파일 | 물리 | PPO 업데이트 |
|---|---|---|---|
| `11_train_policy.sh` (`train_policy.sh`) | `tron1_openarm/scripts/train_ppo_cpu.sh` | MuJoCo C 스레드, CPU | JAX CPU (스크립트가 `JAX_PLATFORMS=cpu` 로 고정) |
| `12_train_mjx.sh` (`train_mjx.sh`) | `tron1_openarm/scripts/train_ppo_gpu.sh` | MJX + Warp, GPU | JAX GPU |

Python 진입점도 각각 `rl/train_ppo_cpu.py`, `rl/train_ppo_gpu.py`, 환경은 `rl/tron1_env_cpu.py`, `rl/tron1_env_gpu.py`다. 두 경로 모두 PPO + GAE, 별도 critic, 선속도 추정 encoder 지도학습을 사용한다. 두 판 모두 기본 SF·30분이고 시작 정책은 발 형태별 배포 정책이다. 환경 수만 CPU 128 / GPU 4096 으로 다르다. 보상·명령·팔 자세·손 외력 설정은 `rl/train_config.py` 를 함께 쓴다 (예전 CPU 판은 PF·LimX 원본에서 시작·90분이었다).

## v3의 정확한 의미

여기서 배포 v3는 초기 SF 실험 `sf_v3`가 아니라 **`runs/upright/seed1/final.npz`**다. 배포 대상은 `src/tron1_openarm_sim/deployed_policies/tron1sf_openarm_policy.npz`.

`locomotion-rl.md` 13-5절에 따르면 v2의 서기 기울기 7~8°를 줄이기 위해 아래 설정으로 시드 2개를 각각 35분 학습했다.

| 변수 | v3 값 | 의미 |
|---|---:|---|
| `TRON1_HAND_F` | 25 | 손 외력 최대 N |
| `TRON1_REACH_P` | 0.35 | 집기 팔 자세 선택 확률 |
| `TRON1_STAND_FEET` | 5 | 서기 발 이동 벌점 크기 |
| `TRON1_STAND_LOAD` | 3 | 발 하중 쏠림 벌점 크기 |
| `TRON1_STAND_TWIST` | 5 | 서기 발 비틀림 벌점 크기 |
| `TRON1_STAND_TILT` | 4 | 서기 기울기 벌점 크기 |
| `TRON1_EP_S` | 60 | 에피소드 길이 초 |
| `TRON1_STAND_LONG_MAX_S` | 50 | 긴 서기 최대 초 |

이 값은 `rl/train_config.py` 의 **SF 기본값**이라 셸 스크립트로 실행하든 Python 을 직접 실행하든 적용된다. PF 에는 적용하지 않는다(손 외력·서기 벌점 없음, 집기 자세 0 %, 에피소드 20 s). 명시적으로 지정한 환경 변수는 발 형태와 관계없이 우선하며, 두 스크립트 모두 실제 `TRON1_*` 값을 실행 폴더 `training_env.txt`에 기록한다. v4~v10의 추가 보상은 환경 기본값인 꺼짐 상태로 두어야 한다. 이전 실험에서 export한 변수가 남아 있으면 설정이 달라지므로 기록을 확인한다.

**이것은 v3 보상 조건으로 추가 미세조정하는 것이지 원래 v3 생성의 완전 재현은 아니다.** 기본 초기 정책은 현재 배포 v3다. 원래 과거 학습을 재현하려면 당시 초기 정책/코드/의존성/시드도 복원해야 한다.

CPU 환경에는 손 외력·집기 자세·추가 서기 항목과 v4~v10 실험 항목을 GPU 판과 같은 식으로 이식했다(2026-11-20). 발 높이·발바닥 비틀림에 필요한 발 몸체 위치·각속도는 CPU 모델을 읽을 때 센서로 더한다(`mjx_model.build_cpu_model`). 두 환경의 보상 항·계수(29개)·에피소드 길이·집기 자세 확률이 SF·PF 모두 같은지 확인했다. 물리 차이(GPU 판의 발만 닿는 단순화 모델·솔버 반복 16, 랜덤화 방식)는 남으며 `rl/mjx_parity.py` 로 비교한다. 이 이식 과정에서 GPU 판이 발 몸체 이름을 SF 것(`ankle_*`)으로 고정해 PF 학습이 실패하던 문제도 고쳤다.

## 이후 진행과 최종 판정

- v3 seed1 final은 기울기 약 3°로 개선. 처음 집기 실패는 파지 보조 구현 문제였고 weld 보조 수정 뒤 r1~r8 **8/8**, 평균 99초로 채택됐다. seed2는 장시간 서기에서 탈락했다.
- 이후 v4~v10은 옆걸음·직진 방향·팔 자세·느린 이동을 개선하려는 추가 미세조정이었다. 일부 지표는 좋아졌지만 다른 지표가 악화됐다.
- 최종 비교에서 v9 best도 서기 기울기와 느린 미세 이동이 나빠 **배포 v3 유지**로 결론 났다 (`locomotion-rl.md` 14-9절).
- 근거 기록: `rl/eval_records/eval_gait_history.json`, `rl/eval_records/gait_pick_tests.json`. 학습 보상만으로 배포하지 않는다. v3는 best가 아니라 final이 채택된 사례다.

## 실행과 평가

```bash
# SF, v3 보상 조건으로 배포 정책에서 추가 학습
bash tron1_openarm/scripts/train_ppo_gpu.sh 35 sf --seed 1
# 같은 조건을 CPU 물리로 (GPU 가 없는 PC, 학습량은 수십 배 적음)
bash tron1_openarm/scripts/train_ppo_cpu.sh 30 sf
```

두 실행 모두 학습 후 best/final을 배포 정책과 비교하여 실행 폴더 `eval.json`에 저장한다. 버전 관리 중인 `rl/eval_records/eval_results.json`은 덮어쓰지 않는다.

후보 채택 전에는 같은 코드에서 배포 v3와 나란히 오프라인 평가, 집기 r1~r8, 미션을 시험한다. 기록상의 기준은 집기 8/8, 걸음 모드 약 6초 이하, 기울기 4° 이하와 미션 성공이다. 이번 조사는 실제 재학습이나 로봇 시험을 실행하지 않았다.
