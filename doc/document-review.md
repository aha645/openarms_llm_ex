# 문서 재구성 결과와 확인이 필요한 사항

[README](../README.md)

## 1. 기본 동선과 이번 처리

환경 설정 2개 + 로봇/일차별 교육 4개를 새 기본 문서로 작성했습니다.

- [노트북 Ubuntu 24.04 네이티브](setup/ubuntu24-native.md)
- [Intel iMac Docker](setup/imac-docker.md)
- [휠형 1일차](course/wheel-day1.md) / [2일차](course/wheel-day2.md)
- [TRON1 1일차](course/tron1-day1.md) / [2일차](course/tron1-day2.md)

설치 문서는 offline 통합 실행까지만, 교육 문서는 준비 완료를 전제로 기능·명령·소스를 설명합니다. 기존 파일은 링크·개념 설명·시험 이력 유실을 막기 위해 삭제하거나 이동하지 않았습니다. 기존 종합 가이드와 목차에는 새 기본 문서 안내를 추가했습니다. **소스 동작은 변경하지 않았습니다.**

## 2. 기존 문서는 어떤 내용이며 어떻게 처리했나

| 기존 문서 | 성격 | 현재 처리 / 권장 후속 조치 |
|---|---|---|
| `mujoco_ros2_mobile_openarm_guide.md` | 노트북 설치, 커리큘럼 연결, 실로봇 전환, 검증 기록이 섞인 종합 가이드 | 보존하되 새 설치 문서를 우선 안내. 실로봇 부분은 향후 독립 심화 문서로 분리 권장 |
| `imac_docker_openarm_guide.md` | Docker 상세 옵션·noVNC·트러블슈팅·시험 범위 | 상세 참고로 보존. 새 문서는 clone 기반 기본 경로만 제공 |
| `labs/README.md`, `labs/day1-*`, `labs/day2-*` | 12개 장의 개념·명령 설명·미션·문제 해결 | 선택적 상세 교재로 보존. 새 교육 문서에서 필요한 장만 링크 |
| `labs/glossary.md` | 약어 풀이 | 공통 참고로 계속 사용 |
| `tron1/01-what-changed.md`, `02-model.md` | 하반신 교체 이유·모델 생성 | 설계 심화 자료로 보존 |
| `tron1/03-locomotion-rl.md` | PPO 정책·학습 과정 | 선택적 심화. 기본 교육 설치에 JAX/PyTorch를 섞지 않음 |
| `tron1/04-ros2-labs.md` | 휠형 실습을 PF로 바꾸는 대응표·차이 | 과거 상세 참고로 보존. 최신 기본 명령은 새 SF 교육 문서를 우선 |
| `tron1/05-limits-real-robot.md` | 검증 범위·실로봇 전환 한계 | 안전·한계 참고로 보존 |
| `tron1/06-sway-analysis.md` | 흔들림 원인 분해 측정 | 연구/분석 기록으로 보존 |
| `tron1/07-implementation-walkthrough.md` | 모델·학습·SLAM·집기·미션 구현 전체 이력 | 개발자 심화 자료로 보존. 날짜별 기록이지 최신 설치 절차로 사용하지 않음 |
| `tron1/08-visual-servo.md` | 시각 피드백 집기 개발·실패 시험 | 개발·실험 이력으로 보존 |
| `tron1/09-sf-mission.md` | SF 전체 미션 1회 성공과 수정 사항 | 현재 SF 교육 기준의 근거 자료로 연결. 반복 성공 보장으로 표현하지 않음 |
| `images/` | 월드·로봇·SLAM 비교 그림 | 기존 링크가 있으므로 유지 |

## 3. 검토 중 발견한 불일치·주의점

### A. Ubuntu 버전과 경로

- 요청의 `Ubuntu34.04`는 문맥상 **Ubuntu 24.04**로 해석했습니다.
- 저장소 이름은 `openarms_llm_ex`, 기존 문서의 워크스페이스 이름은 `openarm_mobile_ws`입니다. 네이티브 새 문서는 현재 clone 경로를 사용합니다.
- Docker Compose는 Mac 폴더 이름과 무관하게 내부를 `/root/openarm_mobile_ws`로 마운트합니다. 이 경로는 코드에 맞춰 유지했습니다.

### B. ‘colcon build 완료’의 의미

- 휠형 `04_setup_workspace.sh`는 `--base-paths src`로만 빌드합니다. 이것만으로 TRON1 패키지는 준비되지 않습니다.
- clone·일반 colcon만으로 외부 메시가 내려오지 않습니다. 모델/에셋 생성이 먼저 필요합니다.
- TRON1은 `10_setup_tron1.sh`와 `--base-paths src tron1_openarm/src`가 필요합니다. 새 TRON1 1일차에서 이 전제를 명시했습니다.

### C. PF 기본값과 SF 성공 기록

- 실제 launch 기본값은 PF입니다. 기존 TRON1 목차·초기 실습서는 PF 중심입니다.
- 이후 `09-sf-mission.md`의 완주 기록은 SF 1회이며 반복 성공률은 미측정입니다.
- 새 교육은 SF를 명시하고 PF를 비교 실습으로 분리했습니다. **launch 기본값 자체는 변경하지 않았습니다.**

### D. 모든 launch에 붙일 수 없는 옵션

- `camera_hz`, `rate_factor`, `lidar_rays`는 sim launch에 있지만 nav/full_system에서 모두 선언·전달하지 않습니다.
- 기존 문서의 ‘모든 launch에 camera_hz를 붙이기’ 같은 포괄 안내는 그대로 따르지 않도록 새 문서에 경고했습니다.
- `world:=world_light.xml`도 모델을 띄우는 launch에 적용하는 옵션이지 MoveIt launch 공통 옵션이 아닙니다.
- 향후 상위 launch가 해당 옵션을 전달하도록 소스를 확장할지 결정이 필요합니다.

### E. 설치 스크립트의 영향과 실패 처리

- `03`은 `sudo apt upgrade -y`를 수행합니다. 새 문서에 시스템 전체 업그레이드 주의를 명시했습니다.
- `01`은 NVIDIA 580 설치·hold·PRIME 변경을 전제로 합니다. 현재 배포 저장소와 장치 지원을 재확인하지 않고 필수 실행하도록 안내하지 않았습니다.
- `04`/`10`은 일부 rosdep·selftest·apt 실패를 무시합니다. ‘빌드 완료 = 렌더링·보행 전체 검증 완료’로 보지 않도록 안내했습니다.
- root Docker에서 `10`의 sudo 설치가 실패할 수 있어 root용 apt 사전 설치를 명시했습니다.
- Dockerfile은 베이스 이미지에서 개발 도구가 제공되는지에 영향을 받으므로 새 Docker 절차는 `colcon`/`rosdep` 확인과 필요 시 `ros-dev-tools` 설치를 포함합니다.

### F. 검증 도구의 강제 종료

`tron1_openarm/tools/validation/mission_test.sh`는 마지막에 이름 패턴으로 ROS 프로세스를 찾아 **`kill -9`** 합니다. 다른 ROS 작업까지 종료할 수 있습니다. 새 교육 문서에서는 격리된 세션에서만 사용하는 선택적 검증 도구로 경고했습니다. 향후 자신이 시작한 PID만 종료하도록 수정하는 것이 좋지만 이번 문서 작업에서는 코드를 변경하지 않았습니다.

### G. offline·취소·안전

- offline은 실제 LLM이 아닌 키워드 규칙입니다.
- `planner:=llm`도 API 오류 시 기본 offline fallback이 가능하므로 실제 사용 계획기는 로그로 확인해야 합니다.
- `/task/cancel`은 단계 사이 취소이며 비상정지가 아닙니다.
- 기본 파지 보조와 정답 기반 시험을 실로봇 성능·안전성으로 해석하면 안 됩니다.

## 4. 사용자 판단이 필요한 문서 정책

이번에는 보수적으로 **기존 자료를 모두 보존**했습니다. 다음은 추가 판단이 필요합니다.

1. **기존 12개 상세 실습서를 계속 제공할지?**
   - 권장: 새 4개 일차별 문서를 기본 교재로, 기존 12개는 개념·문제 해결 참고로 유지.
   - 완전 축약을 원하면 상세 실습서를 archive로 이동할 수 있지만 기존 상대 링크를 전부 수정해야 합니다.
2. **TRON1 교육을 SF로 확정할지?**
   - 새 문서는 완주 기록에 따라 SF 기준. PF 보행 비교 중심 교육을 원하면 별도 경로로 분리하는 것이 좋습니다.
   - launch 기본값을 SF로 바꾸는 것은 별도 소스 변경·재검증 사항입니다.
3. **실로봇 전환·CUDA/PyTorch·정책 학습을 교육 범위에 넣을지?**
   - 권장: 이번 2일 과정에서 제외하고 심화로 유지. 특히 학습용 NumPy/JAX 환경과 ROS 환경을 섞지 않기.
4. **실험 기록의 날짜·성공 표현을 어떻게 유지할지?**
   - 권장: 과거 기록은 그대로 두되 최신 지원표와 구분. 과거 성공을 현재 노트북·iMac의 재현 성공으로 바꾸지 않기.

## 5. 이번 작업의 검증 범위

- 설치 스크립트, Dockerfile/Compose, 휠/TRON1 launch, 앱 계획기·순찰, 검증 스크립트 및 기존 교육·실험 문서의 구조/관련 명령을 대조했습니다.
- 새 기본 문서의 로컬 링크·소스 경로를 정적 점검합니다.
- **실제 apt 설치, Docker build, colcon build, 보행·집기·미션 재실행은 하지 않았습니다.** 현재 노트북에는 조사 시 ROS가 없었고 iMac 환경도 제공되지 않았습니다. 교육 배포 전 두 장비에서 처음부터 따라 하는 검증이 필요합니다.
