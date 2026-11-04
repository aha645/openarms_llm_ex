# 04. 같은 실습을 TRON1 로 — SLAM · Nav2 · MoveIt · ArUco · LLM 미션

## 이 장의 미션

> 바퀴형으로 했던 DAY 1·DAY 2 실습을 이족 로봇으로 똑같이 한다. 명령은 패키지 이름만 바뀌고, 달라지는 점(제자리 걸음, 느린 속도, 다리 오도메트리)을 직접 관찰한다.
> 마지막에 웹 대시보드에서 **"빨간 상자를 왼쪽 작업대로 옮기고 돌아와"** 를 실행해 끝까지 완료한다.

각 장의 개념·명령 설명은 바퀴형 실습서([../labs/README.md](../labs/README.md))와 같습니다. 이 문서는 **바뀐 부분만** 적습니다.

## 이 장에 나오는 용어

- **SLAM** (Simultaneous Localization And Mapping, 동시적 위치추정 및 지도작성) · **AMCL** (Adaptive Monte Carlo Localization, 적응형 몬테카를로 위치추정) · **Nav2** (Navigation 2) · **MoveIt 2** · **TF** (Transform, 좌표 변환) — 바퀴형 실습서 [용어집](../labs/glossary.md) 참고
- **제자리 걸음 (stepping in place)** — 명령 0 일 때 Point-foot 이족이 균형을 위해 계속 발을 구르는 상태
- **명목 높이 (nominal height)** — URDF 에 고정으로 적은 base_footprint→base_link 높이 0.68 m. 실제 몸통은 ±2 cm 출렁인다

## 0. 준비 (한 번)

```bash
cd ~/openarm_mobile_ws
bash scripts/04_setup_workspace.sh                 # 바퀴형 (아직 안 했다면)
bash tron1_openarm/scripts/10_setup_tron1.sh       # TRON1 모델·월드·URDF·SRDF 생성 + 빌드
source install/setup.bash
```

빌드는 두 폴더를 함께 합니다: `colcon build --symlink-install --base-paths src tron1_openarm/src`. DAY 2 앱(`mobile_openarm_apps`)은 바퀴형 것을 그대로 씁니다.

## 1. 명령 대응표

| 장 | 바퀴형 | TRON1 이족 |
|---|---|---|
| 1-01 MuJoCo | `python3 -m mujoco.viewer --mjcf=src/mobile_openarm_sim/mjcf/world.xml` | `python3 tron1_openarm/src/tron1_openarm_sim/tools/selftest_tron1.py` (뷰어만 열면 정책이 없어 주저앉음) |
| 1-02 브리지 | `ros2 launch mobile_openarm_sim sim.launch.py` | `ros2 launch tron1_openarm_sim sim.launch.py` |
| 1-03 TF | `tools/mjcf_to_urdf.py` | `tools/make_urdf_tron1.py`, `view_frames` 에 다리 링크 추가 |
| 1-04 SLAM | `ros2 launch mobile_openarm_sim slam.launch.py` | `ros2 launch tron1_openarm_sim slam.launch.py` |
| 1-05 Nav2 | `ros2 launch mobile_openarm_sim nav.launch.py` | `ros2 launch tron1_openarm_sim nav.launch.py` |
| 1-06 순찰 | `ros2 run mobile_openarm_apps patrol` | 같음 (Nav2 만 TRON1 것을 켜 두면 됨) |
| 2-01 MoveIt | `ros2 launch mobile_openarm_moveit_config move_group.launch.py` | `ros2 launch tron1_openarm_moveit_config move_group.launch.py` |
| 2-02 Pick&Place | `sim.launch.py start_x:=2.5 start_y:=2.45 start_yaw:=1.5708` + `pick_place` | `tron1_openarm_sim sim.launch.py` 같은 인자 + 같은 `pick_place` |
| 2-03 Nav+Pick | nav + move_group + `nav_pick_place` | TRON1 nav + TRON1 move_group + 같은 `nav_pick_place` |
| 2-04 LLM | `llm_planner` | 같음 (로봇과 무관) |
| 2-05 Task Manager | `task_manager` | 같음 |
| 2-06 최종 | `ros2 launch mobile_openarm_apps full_system.launch.py` | `ros2 launch tron1_openarm_sim full_system.launch.py` |

모든 TRON1 launch 공통 인자: `world:=world_light.xml`(저사양), `viewer:=false`, `rviz:=false`, `policy:=<npz>`(보행 정책 바꾸기).

## 2. 장별로 달라지는 점

### DAY 1-02 브리지

```bash
ros2 launch tron1_openarm_sim sim.launch.py
ros2 topic echo /biped/status --field data        # 새 토픽: 몸통 높이·기울기·넘어짐·정책이 받은 명령·추정속도
```

- 실행 직후 로봇이 0.71 m 에서 다리를 편 채 놓이고, 정책이 받아서 0.68 m 로 섭니다. 명령이 없어도 **제자리 걸음**을 합니다.
- `/joint_states` 에 바퀴 관절 대신 다리 6관절(`abad/hip/knee_L/R_joint`)이 나옵니다.
- teleop: 바퀴형처럼 `z` 로 속도를 0.3 m/s 근처로 낮추세요. 0.5 m/s 이상은 브리지가 잘라냅니다.
- 미션 "1 m 직진 후 `/odom` 과 `/ground_truth/pose` 비교": 다리 오도메트리 오차는 바퀴보다 큽니다 (수 cm/m).

### DAY 1-03 TF

```
map → odom → base_footprint → base_link → abad/hip/knee/foot_L,R
                                        → openarm_mount_link → openarm_body_link0 → 양팔·머리 카메라
                                        → lidar_link, imu_link
```

- `base_footprint → base_link` 는 URDF 에 **고정 0.68 m** 입니다. 실제 몸통은 ±2 cm 출렁이고 1~3° 기웁니다. 그런데도 고정으로 둔 이유:
  - MoveIt 은 팔 경로를 base_footprint 기준 로봇 모델로 계산합니다. 이 관절이 움직이면 MoveIt 이 그 값을 알 방법이 없습니다(관절 상태에 없음).
  - 대신 브리지는 ArUco 위치를 **실제 base_link 기준으로 잰 뒤 고정 0.68 m 를 더해** base_footprint 기준으로 냅니다. 팔·카메라가 모두 base_link 에 붙어 있으므로, 몸통이 출렁여도 "카메라가 본 상자 ↔ 손" 의 상대 위치는 정확합니다.
  - 오도메트리 TF(`odom → base_footprint`)의 x, y, yaw 는 실제 움직임을 따릅니다.
- 확인: `ros2 run tf2_ros tf2_echo base_footprint head_camera_link` → z ≈ 1.16 m (바퀴형과 같음)

### DAY 1-04 SLAM

```bash
ros2 launch tron1_openarm_sim slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # 속도 0.25~0.3 m/s, 회전 0.5 rad/s 정도
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/tron1_map --ros-args -p use_sim_time:=true
ros2 launch tron1_openarm_sim nav.launch.py map:=$HOME/maps/tron1_map.yaml
```

- LiDAR 가 0.55 m 높이 (바퀴형 0.34 m) 라 작업대 상판은 여전히 안 보이고 다리만 보입니다. 상자 더미(0.6 m)는 보입니다.
- 제자리 걸음 때문에 스캔이 조금 흔들립니다. 그래서 바퀴형보다 **천천히, 회전은 더 천천히** 하세요. 루프 클로저(출발점으로 돌아오기)가 더 중요합니다.
- 다리 오도메트리 잡음 때문에 지도가 살짝 휠 수 있습니다. 한 바퀴 돈 뒤 home 근처에서 잠시 서 있으면 slam_toolbox 가 맞춥니다.

### DAY 1-05 Nav2

| 파라미터 | 바퀴형 | TRON1 |
|---|---|---|
| 최대 속도 vx / wz | 0.40 m/s / 1.2 rad/s | 0.35 m/s / 0.8 rad/s |
| 가속 ax / az | 2.0 / 2.5 | 0.8 / 1.5 |
| 도착 허용 xy / yaw | 0.12 m / 0.15 rad | 0.15 m / 0.20 rad |
| footprint | 0.55 × 0.52 m | 0.46 × 0.44 m |
| AMCL alpha1~4 | 0.2 | 0.4 |

- 도착한 뒤에도 로봇이 제자리 걸음을 하므로 RViz 에서 몸이 조금씩 움찔합니다. 정상입니다.
- `Failed to make progress` 가 바퀴형보다 잘 나면 `config/nav2_params_tron1.yaml` 의 `movement_time_allowance` 를 늘리세요.

### DAY 2-01 MoveIt

```bash
ros2 launch tron1_openarm_sim sim.launch.py rviz:=false
ros2 launch tron1_openarm_moveit_config move_group.launch.py
ros2 run mobile_openarm_apps arm_demo
```

- RViz MotionPlanning 에 다리가 보입니다. 다리 관절은 계획 그룹에 없고, MoveIt 은 현재 다리 자세(`/joint_states`)로 **팔–다리 충돌**을 검사합니다.
- 팔을 크게 움직이면 몸통이 반동으로 조금 움직입니다(제자리 걸음으로 균형). `arm_demo` 의 `pose goal` 은 base_footprint 기준이라 그 사이 몸이 움직이면 목표가 1~2 cm 어긋날 수 있습니다.

### DAY 2-02 ArUco Pick & Place

```bash
ros2 launch tron1_openarm_sim sim.launch.py start_x:=2.5 start_y:=2.45 start_yaw:=1.5708
ros2 launch tron1_openarm_moveit_config move_group.launch.py rviz:=false
ros2 run mobile_openarm_apps pick_place --ros-args -p object:=box_0
```

- 이족은 조작 중에도 제자리 걸음을 하므로 몸통이 조금씩 흐릅니다. `robot_skills.pick()` 은 그래서 원래부터 **집기 직전에 카메라로 다시 재고(find_marker 5회 평균), 상자가 0.52 m 앞에 오도록 미세 이동**합니다. 이족에서 이 단계가 더 중요해집니다.
- 실패해도 `파지 실패 → 다시 인식해서 재시도` 가 한 번 더 돕니다.

### DAY 2-03 ~ 2-06

```bash
ros2 launch tron1_openarm_sim full_system.launch.py planner:=offline      # → http://localhost:8080
```

바퀴형과 같은 대시보드·같은 예시 버튼입니다. 주행이 느려 미션 시간이 더 깁니다.

> 통합 검증(SLAM 지도 작성, Nav2 주행, LLM 미션) 결과는 검증이 끝나면 이 자리에 추가합니다.

## 3. 확인 질문

1. TRON1 브리지가 바퀴형 브리지 코드를 재사용할 수 있는 이유는? → 토픽·액션·서비스 이름과 형식이 같고, 하반신 부분만 시뮬레이션 코어에서 바뀌므로
2. base_footprint→base_link 를 고정 0.68 m 로 둔 이유는? → MoveIt 이 알 수 없는 관절을 만들지 않기 위해. ArUco 위치는 실제 base_link 기준으로 재서 정확도를 유지
3. 도착 허용 오차를 넓힌 이유는? → 제자리 걸음 중 몸통이 ±2~3 cm 흔들려서
