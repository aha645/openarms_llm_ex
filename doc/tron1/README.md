# TRON1 + OpenArm 교육·설계·검증

[README](../../README.md) · [휠형 교육](../wheel/README.md) · [용어집](../glossary.md)

바퀴형 실습 환경(`mobile_openarm/`)은 그대로 두고, `tron1_openarm/` 폴더에 **하반신만 LimX TRON1(이족)** 으로 바꾼 로봇을 따로 만들었습니다. 상반신 OpenArm 양팔, 실습 월드, SLAM·Nav2·MoveIt·ArUco·LLM 미션·웹 대시보드는 바퀴형과 같습니다. 교육은 전체 미션 완주 기록이 있는 **Sole-foot(`foot:=sf`)** 기준이며, launch 기본값은 Point-foot(`pf`)입니다.

![TRON1 + OpenArm](../images/tron1_openarm.png)

## 문서

| 문서 | 내용 |
|---|---|
| [day1.md](day1.md) | **교육 1일차** — 사전 준비(모델 생성·overlay 빌드), 보행 브리지·TF·SLAM·Nav2·순찰, 휠형 명령 대응표와 다른 점 |
| [day2.md](day2.md) | **교육 2일차** — 이족 MoveIt·카메라 집기·운반·LLM·Task Manager·통합 미션 |
| [design.md](design.md) | 무엇을 새로 구성했고 왜 그랬나, LimX description → xacro → URDF → MJCF, 상반신 장착 높이·위치와 설계 변경 이력 |
| [locomotion-rl.md](locomotion-rl.md) | 두 발로 서고 걷기: LimX 사전학습 정책 + PPO 미세조정(팔을 움직이는 상태), 보상·버전 이력, 브리지 제어 흐름 |
| [validation-log.md](validation-log.md) | 시험·검증 기록(날짜순): SLAM·Nav2·집기 시험, 흔들림 원인 분석, 손목 마커 시각 피드백, SF 전체 미션 완주 |
| [../real-robot.md](../real-robot.md) | 검증 범위·한계, 실로봇으로 갈 때의 순서 (휠형과 공통) |

정책 학습용 가상환경(`tron1_openarm/.venv-biped`, JAX GPU)은 [Python 가상환경](../setup/python-venv.md)을 참고하세요.

## 빠른 시작

```bash
cd ~/openarm_mobile_ws
bash mobile_openarm/scripts/04_setup_workspace.sh               # 바퀴형 (OpenArm 에셋·의존성)
bash tron1_openarm/scripts/10_setup_tron1.sh     # TRON1 description 받기 → 모델 생성 → 자체점검 → 빌드(overlay)
source tron1_openarm/install/setup.bash          # 바퀴형 mobile_openarm underlay 포함
ros2 launch tron1_openarm_sim full_system.launch.py foot:=sf planner:=offline    # → http://localhost:8080
```

(선택) 보행 정책 직접 학습: `bash tron1_openarm/scripts/11_train_policy.sh 90` — ROS 와 분리된 `tron1_openarm/.venv-biped`(JAX GPU)를 자동으로 만들어 씁니다. [Python 가상환경](../setup/python-venv.md)

## 폴더 구조

```
tron1_openarm/
├── scripts/10_setup_tron1.sh, 11_train_policy.sh, setup_venv_biped.sh
├── third_party/                    (스크립트가 clone, git 에 안 올라감) tron1-robot-description, tron1-mujoco-sim
├── .venv-biped/                    (setup_venv_biped.sh 가 생성, git 제외) PPO 학습용 JAX·MuJoCo
├── build/ install/ log/            (colcon 산출물, mobile_openarm/install 위 overlay)
├── src/
│   ├── tron1_openarm_sim/          ROS 2 패키지: 모델·월드·브리지·launch·Nav2 설정·보행 정책
│   │   ├── urdf/tron1_openarm.urdf.xacro   ← 새로 작성 (LimX xacro include + 상반신 마운트)
│   │   ├── tools/build_tron1_openarm.py    ← xacro→URDF→MJCF + OpenArm 접목
│   │   ├── tools/build_world_tron1.py, make_urdf_tron1.py, make_nav2_params_tron1.py, selftest_tron1.py
│   │   ├── tron1_openarm_sim/locomotion_policy.py, biped_core.py, bridge_biped.py
│   │   ├── policies/tron1_openarm_policy.npz (미세조정), limx_pf_tron1a_pretrained.npz (LimX 원본)
│   │   └── launch/sim, slam, nav, full_system
│   └── tron1_openarm_moveit_config/  MoveIt (다리 포함 URDF·SRDF, 팔 설정은 바퀴형과 동일)
├── rl/                             강화학습: tron1_env.py, train_ppo.py, eval_policy.py, export_onnx.py
├── policies/                       평가 결과(eval_results.json), ONNX
└── runs/                           학습 로그 (git 에는 요약만)
```

## 출처·라이선스

- TRON1 모델·사전학습 정책: [limxdynamics/tron1-robot-description](https://github.com/limxdynamics/tron1-robot-description), [tron1-mujoco-sim](https://github.com/limxdynamics/tron1-mujoco-sim), [tron1-rl-deploy-python](https://github.com/limxdynamics/tron1-rl-deploy-python), [tron1-rl-isaacgym](https://github.com/limxdynamics/tron1-rl-isaacgym) (Apache-2.0)
- OpenArm: [enactic/openarm_mujoco](https://github.com/enactic/openarm_mujoco) (Apache-2.0)
