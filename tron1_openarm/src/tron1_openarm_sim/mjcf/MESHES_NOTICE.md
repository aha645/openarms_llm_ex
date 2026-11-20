# 메시 출처

`meshes/` (Point-foot) 와 `sf/meshes/` (Sole-foot) 는 아래 원본을 그대로 복사한 것입니다 (`tools/build_tron1_openarm.py` 가 복사).

| 폴더 | 원본 | 커밋 | 라이선스 |
|---|---|---|---|
| `meshes/visual`, `meshes/collision`, `sf/meshes/visual`, `sf/meshes/collision` | https://github.com/enactic/openarm_mujoco `v1/meshes/` | `6c455d32854e8ad5af2c8a483db8d84b1ca49402` (2026-11-16) | Apache-2.0 — [전문](../../../../doc/third_party_licenses/openarm_mujoco-LICENSE.txt) |
| `meshes/tron1` | https://github.com/limxdynamics/tron1-robot-description `pointfoot/PF_TRON1A/meshes/` | `ac417d44656d3c7494a095cecd62928e777a0f3a` (2026-11-05) | Apache-2.0 — [전문](../../../../doc/third_party_licenses/tron1-robot-description-LICENSE.txt) |
| `sf/meshes/tron1` | 같은 저장소 `pointfoot/SF_TRON1A/meshes/` | 같음 | 같음 |

MJCF·URDF·SRDF 도 이 원본과 저장소의 생성 도구(`tools/`)로 만든 결과를 그대로 넣어 두었습니다.
평소 설치(`setup_workspace.sh`)는 이 파일들을 그대로 씁니다. 원본에서 다시 만들려면 `bash tron1_openarm/scripts/regen_models.sh`.
