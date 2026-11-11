# DAY 2-04 LLM과 OpenAI API — 자연어를 작업 계획으로

← [이전](day2-03-nav-pick-place.md) · [목차·공통 규칙](README.md) · [다음](day2-05-task-manager.md) →

## 이 장의 미션

> "빨간 상자를 왼쪽 작업대로 옮기고 돌아와" 같은 **한국어 명령을 LLM에 주고, 로봇이 실행할 수 있는 단계 목록(JSON)을 받는다.**
> 로봇이 할 수 없는 명령("커피 타 줘")은 LLM이 **거절**하는지, 위험한 계획은 **검증기가 막는지** 확인한다.

이 장은 **로봇을 움직이지 않습니다.** 계획만 만들어 화면에 출력합니다. 실제 실행은 다음 장(05)입니다. 시뮬레이터를 켜지 않아도 됩니다.

## 이 장에 나오는 용어

- **LLM** (Large Language Model, 대규모 언어 모델) — GPT 같은 언어 AI
- **API** (Application Programming Interface) — 프로그램이 OpenAI 같은 서비스를 호출하는 약속. **API 키**는 사용 권한 비밀 문자열
- **Structured Outputs** (구조화 출력) — LLM이 정해진 JSON 형식(**스키마, schema**)대로만 답하게 하는 OpenAI 기능
- **JSON** (JavaScript Object Notation) — `{"이름": 값}` 형태의 데이터 텍스트
- **enum** (enumeration, 열거형) — 정해진 값 몇 개만 허용하는 형식
- **pydantic** — Python에서 데이터 형식을 클래스로 정의·검증하는 라이브러리

전체 목록은 [용어집](../glossary.md)에 있습니다.

## 1. 개념

### LLM을 로봇에 쓸 때의 문제

**LLM(Large Language Model, 대규모 언어 모델)** 은 자연어를 잘 이해하지만 그대로 로봇에 연결하면 위험합니다.

- 아무 형식으로나 답함 → 프로그램이 읽을 수 없음
- 없는 장소·물체를 지어냄("부엌으로 가서…")
- 순서가 틀림(집지도 않았는데 놓기)

그래서 세 겹으로 막습니다.

| 겹 | 방법 | 이 실습 코드 |
|---|---|---|
| ① 형식 강제 | **구조화 출력(Structured Outputs)**: LLM이 정해진 JSON 스키마(형식)대로만 답하게 함 | `RobotPlan` 클래스(pydantic) |
| ② 값 제한 | 행동은 6가지, 위치는 8개 이름, 물체는 3개 이름만 허용(**enum**, 열거형) → 좌표나 새 명령을 만들 수 없음 | `Step.action`(Literal), `Location`, `ObjectName` |
| ③ 실행 전 검증 | 계획을 **가상으로 한 단계씩 따라가며** 전제조건 검사: 빈손인데 놓기, 이미 들었는데 또 집기, 작업대 없는 곳에 놓기, 12단계 초과 → 거부. 빠진 이동은 자동 추가 | `validate_plan()` |

### 계획 형식 (`RobotPlan`)

```json
{
  "feasible": true,
  "reason": "빨간 상자를 작업대 B로 옮긴 뒤 복귀",
  "steps": [
    {"action": "navigate", "location": "pick_station", "object": null, "text": null, "seconds": null},
    {"action": "pick",     "location": null, "object": "box_0", "text": null, "seconds": null},
    {"action": "navigate", "location": "place_station", ...},
    {"action": "place",    "location": "place_station", ...},
    {"action": "navigate", "location": "home", ...}
  ]
}
```

| 행동 | 필요한 값 | 의미 |
|---|---|---|
| `navigate` | `location` | 그 위치로 자율주행 (DAY 1-05) |
| `pick` | `object` | 지금 작업대에서 물체 집기 (DAY 2-02) |
| `place` | `location` | 그 작업대에 놓기 |
| `home_arms` | — | 팔 기본 자세 |
| `say` | `text` | 로그로 보고 |
| `wait` | `seconds` | 대기 (최대 30초) |

위치: `home`, `pick_station`(작업대 A, 동쪽), `place_station`(작업대 B, 서쪽), `door`, `patrol_1~4`.
물체: `box_0` 빨강, `box_1` 초록, `box_2` 파랑 — 처음엔 모두 작업대 A 위.

### 시스템 프롬프트

LLM에게 "너는 이 로봇의 작업 계획기다. 행동·위치·물체는 이것뿐이다. 목록에 없는 일은 `feasible=false`로 답하라"고 알려 주는 지시문입니다. `llm_planner.py` 의 `SYSTEM_PROMPT` 에 있습니다.

### 오프라인 모드

API 키가 없어도 실습할 수 있도록 **키워드 규칙 기반 계획기**(`plan_offline()`)를 함께 넣었습니다. "빨간", "파란", "모두", "순찰", "돌아와" 같은 단어만 보고 계획을 만듭니다. LLM 결과와 비교해 보세요.

## 2. 실제로 실행되는 소스

`mobile_openarm/src/mobile_openarm_apps/mobile_openarm_apps/llm_planner.py` 한 파일입니다. ROS 노드가 아니라 일반 Python 모듈이라 단독 실행됩니다.

| 함수 | 하는 일 |
|---|---|
| `main()` | 명령줄 인자 읽기 → 계획 → `validate_plan()` → 결과 출력 |
| `plan_with_llm(command)` | OpenAI 호출. ① **Responses API** `client.responses.parse(text_format=RobotPlan)` 시도 → 안 되면 ② **Chat Completions** `client.chat.completions.parse(response_format=RobotPlan)` (LM Studio·Ollama 같은 로컬 서버용) |
| `plan_offline(command)` | 키워드 규칙 계획 |
| `validate_plan(plan)` | 전제조건 검사·자동 보정. 반환: (보정된 단계, 경고 목록). 문제면 `ValueError` |

환경 변수:

| 변수 | 의미 | 예 |
|---|---|---|
| `OPENAI_API_KEY` | **API 키(비밀)**. 저장소·코드에 절대 넣지 말 것 | `sk-...` |
| `OPENAI_MODEL` | 모델 이름 (기본 `gpt-4o-mini`) | 계정에서 쓸 수 있는 모델 |
| `OPENAI_BASE_URL` | 다른 서버 주소 (로컬 LLM) | `http://192.168.0.10:1234/v1` |

## 3. 따라하기

### 단계 1 — T1: 오프라인으로 먼저 해 보기 (키 없이)

```bash
ros2 run mobile_openarm_apps llm_planner --offline "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
```

**명령 풀이**

- `llm_planner`: 문장을 작업 단계 목록으로 바꾸는 실행 프로그램입니다.
- `--offline`: API 대신 키워드 규칙을 씁니다. **실제 LLM 호출이 아닙니다.** 이 프로그램이 직접 받는 옵션이므로 `--ros-args -p` 를 붙이지 않습니다.
- `"빨간 ... 돌아와"`: 프로그램에 넘기는 입력 문장 하나입니다. 공백이 있어 큰따옴표로 묶습니다.

**계획을 출력만 하며 로봇은 움직이지 않습니다.** 할 수 없는 요청은 검증에서 거부되며, 이 CLI 는 거부를 오류 출력(Traceback)으로 보여 줄 수 있습니다.

출력:

```
RobotPlan: { "feasible": true, "reason": "오프라인 규칙 계획: 5단계", "steps": [ ... ] }
실행 단계:
  1. navigate pick_station
  2. pick box_0
  3. navigate place_station
  4. place place_station
  5. navigate home
```

다른 명령도 넣어 보세요.

```bash
ros2 run mobile_openarm_apps llm_planner --offline "상자 전부 옮겨"          # 3번 왕복 → 13단계 > 12 → 거부 (검증기)
ros2 run mobile_openarm_apps llm_planner --offline "순찰 한 바퀴 돌고 와"     # patrol_1~4 네 단계
ros2 run mobile_openarm_apps llm_planner --offline "커피 한 잔 타줘"         # 이해 못 함 → 거부
```

| 명령 | 끝 줄 | 의미 |
|---|---|---|
| 상자 전부 옮겨 | `ValueError: 단계가 너무 많음 (13 > 12)` | 3번 왕복 12단계 + home = 13 → **검증기가 막음** |
| 순찰 한 바퀴 돌고 와 | `4. navigate patrol_4` | "돌아/복귀/home"이 없어 home 복귀 단계가 빠짐 → 키워드 규칙의 한계 |
| 커피 한 잔 타줘 | `ValueError: 계획기 판단: 수행 불가 — 이해할 수 없는 명령` | `feasible: false` 라 검증기가 실행을 막음 |

오류(Traceback)로 끝나는 것이 **정상 동작**입니다. 05장의 Task Manager는 이 오류를 받아 "거부" 상태로 표시합니다.

### 단계 2 — API 키 준비

1. platform.openai.com 에서 API 키를 만듭니다(유료 크레딧 필요).
2. **이 터미널에서만** 쓰도록 환경 변수로 넣습니다.

```bash
export OPENAI_API_KEY='sk-여기에_내_키'
export OPENAI_MODEL=gpt-4o-mini
```

- `export` 는 이 터미널을 닫으면 사라집니다. 매번 넣기 싫으면 `~/.bashrc` 끝에 추가하되, **그 파일을 git에 올리지 마세요.**
- Docker는 Mac에서 `export OPENAI_API_KEY=...` 후 `docker compose up -d` 하면 컨테이너로 전달됩니다(`docker-compose.yml` 의 environment).

### 단계 3 — T1: LLM으로 계획하기 (미션)

```bash
ros2 run mobile_openarm_apps llm_planner "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
ros2 run mobile_openarm_apps llm_planner "파란 상자랑 초록 상자를 서쪽 작업대에 갖다 놓고 문 앞에서 기다려"
ros2 run mobile_openarm_apps llm_planner "커피 한 잔 타줘"
ros2 run mobile_openarm_apps llm_planner "사람을 따라가"
```

**관찰할 것**
- 같은 명령에 오프라인과 LLM 결과가 어떻게 다른지 (LLM은 "왼쪽 = 서쪽 = place_station"을 이해)
- 불가능한 명령에서 `"feasible": false` 와 `reason`
- `경고:` 줄 — 검증기가 빠진 이동을 자동으로 넣었을 때

### 단계 4 — (선택) 로컬 LLM 쓰기

다른 PC에서 LM Studio, Ollama, vLLM 같은 **OpenAI 호환 서버**를 돌리고 있다면:

```bash
export OPENAI_BASE_URL=http://<서버IP>:1234/v1     # LM Studio 기본 포트 1234, Ollama는 11434
export OPENAI_API_KEY=local                         # 로컬 서버는 아무 글자나
export OPENAI_MODEL=<서버에 올린 모델 이름>
ros2 run mobile_openarm_apps llm_planner "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
```

로컬 서버는 Responses API가 없을 수 있어 자동으로 Chat Completions로 넘어갑니다. 작은 모델은 JSON 스키마를 잘 못 지켜 실패할 수 있습니다.

## 4. 미션 체크리스트

- [ ] 오프라인 모드로 계획 3가지를 만들고, "상자 전부 옮겨"가 검증기에 막히는 것을 봤다
- [ ] LLM 모드로 같은 명령을 넣어 결과를 비교했다
- [ ] 불가능한 명령이 `feasible: false` 로 거절됐다

## 5. 확인 질문

1. LLM이 임의의 좌표를 만들어 내지 못하게 막는 장치는? → 위치·물체를 enum으로 제한한 스키마
2. `validate_plan()` 이 잡아내는 오류 두 가지는? → 빈손으로 놓기, 이미 들고 있는데 또 집기 (그 외: 작업대 없는 곳에 놓기, 12단계 초과)
3. API 키를 코드에 쓰면 안 되는 이유는? → 저장소에 올라가면 누구나 내 키로 요금을 쓸 수 있음

## 6. 잘 안 될 때

| 증상 | 해결 |
|---|---|
| `LLM 계획 실패: … api_key` | `OPENAI_API_KEY` 가 이 터미널에 없음. `echo ${OPENAI_API_KEY:0:5}` 로 앞 5글자 확인 |
| `model … does not exist` | `OPENAI_MODEL` 을 내 계정에서 쓸 수 있는 모델로 |
| `No module named openai` | `python3` 로 직접 실행했다면 가상환경 켜기: `source ~/openarm_mobile_ws/mobile_openarm/.venv-mobile/bin/activate`. 없으면 `bash mobile_openarm/scripts/setup_venv_mobile.sh` (openai·pydantic v2 설치) |
| 로컬 LLM에서 JSON 오류 | 더 큰 모델 사용, 또는 오프라인 모드 |

## 7. 더 해보기

- `SYSTEM_PROMPT` 에 "한 번에 두 물체까지 양손으로 운반 가능"을 넣으면 계획이 바뀌는지 (검증기는 여전히 한 손만 허용 → 거부되는지)
- `MAX_STEPS` 를 16으로 늘려 "상자 전부 옮겨"가 통과하게 하기
