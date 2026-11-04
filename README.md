# openarm_mobile_ws — MuJoCo + ROS 2 Jazzy 주행형 OpenArm 실습 워크스페이스

PinkLAB "MuJoCo + ROS2 2일 워크샵" 커리큘럼(14개 모듈)을 **OpenArm v1 양팔 상반신 + 차동구동 바퀴 베이스**로
따라 할 수 있도록 새로 구현한 워크스페이스입니다. (원본 교육과정 소스는 비공개라 같은 기술 스택으로 재구성)

대상 PC: Ubuntu 24.04, Samsung Notebook Odyssey(2017, i7-7700HQ, GTX 1050) 기준. 자세한 설명은 함께 드린 가이드 문서를 보세요.

## 빠른 시작
```bash
cd ~/openarm_mobile_ws
bash scripts/00_check_system.sh
bash scripts/01_install_nvidia_driver.sh && sudo reboot      # 580 고정 (Pascal 마지막 지원)
bash scripts/03_install_ros2_jazzy.sh
bash scripts/04_setup_workspace.sh                            # 에셋 받기 → 모델 생성 → selftest → colcon build
source install/setup.bash
ros2 launch mobile_openarm_apps full_system.launch.py planner:=offline   # → http://localhost:8080
```
RAM 8GB 이거나 화면이 끊기면 모든 launch 에 `world:=world_light.xml` 을 붙이세요.

## Mac(Intel) / GPU 없는 PC 에서 Docker 로 실행
```bash
cd ~/openarm_mobile_ws/docker
bash mac_start.sh          # 이미지 빌드 → 컨테이너 시작 → 04 스크립트 → 브라우저(noVNC) 열기
```
컨테이너 안에는 GPU 가 없으므로 Mesa llvmpipe(CPU)로 그립니다. NVIDIA 드라이버·CUDA·PyTorch 는 설치하지 않습니다.
자세한 명령과 옵션 설명은 별도 문서(Docker 가이드)를 보세요.

## 패키지
| 패키지 | 내용 |
|---|---|
| `mobile_openarm_sim` | MJCF 생성(tools/build_mjcf.py), 월드(build_world.py), MJCF→URDF(mjcf_to_urdf.py), 정답 지도(make_map.py), Nav2 파라미터(make_nav2_params.py), MuJoCo–ROS 2 브리지(bridge_node.py), selftest, sim/slam/nav launch |
| `mobile_openarm_moveit_config` | SRDF 자동 생성(tools/make_srdf.py), KDL·OMPL·컨트롤러 설정, move_group launch |
| `mobile_openarm_apps` | robot_skills, patrol, arm_demo, pick_place, nav_pick_place, llm_planner, task_manager, dashboard, aruco_detector(실로봇용), full_system launch |

## 교육과정 대응
| DAY | 모듈 | 실행 |
|---|---|---|
| 1 | 01 Gazebo vs MuJoCo | `python3 -m mujoco.viewer --mjcf=<share>/mjcf/world.xml` |
| 1 | 02 브리지 | `ros2 launch mobile_openarm_sim sim.launch.py` |
| 1 | 03 Description·TF | `tools/mjcf_to_urdf.py`, `ros2 run tf2_tools view_frames` |
| 1 | 04 SLAM·AMCL | `ros2 launch mobile_openarm_sim slam.launch.py` → map_saver → `nav.launch.py map:=...` |
| 1 | 05 Nav2 | `ros2 launch mobile_openarm_sim nav.launch.py` |
| 1 | 06 순찰 | `ros2 run mobile_openarm_apps patrol` |
| 2 | 01 MoveIt 양팔 | `ros2 launch mobile_openarm_moveit_config move_group.launch.py` + `ros2 run mobile_openarm_apps arm_demo` |
| 2 | 02 ArUco Pick&Place | `sim.launch.py start_x:=2.5 start_y:=2.45 start_yaw:=1.5708` + move_group + `pick_place` |
| 2 | 03 Nav+Pick&Place | nav.launch + move_group + `nav_pick_place` |
| 2 | 04 LLM | `ros2 run mobile_openarm_apps llm_planner "..."` |
| 2 | 05 LLM→ROS 2 | `task_manager` + `ros2 topic pub /task/command ...` |
| 2 | 06 대시보드 | `ros2 launch mobile_openarm_apps full_system.launch.py` |

## 라이선스
OpenArm 모델/메시: enactic/openarm_mujoco (Apache-2.0). 이 워크스페이스 코드: Apache-2.0.
