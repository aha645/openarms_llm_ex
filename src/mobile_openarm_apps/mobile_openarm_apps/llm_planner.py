"""Day2-04 LLM 작업 계획: 자연어 → RobotPlan (OpenAI Structured Outputs).

ROS 비의존 모듈 — 단독 실행으로 테스트 가능:
    export OPENAI_API_KEY=sk-...
    python3 -m mobile_openarm_apps.llm_planner "빨간 상자를 왼쪽 작업대로 옮기고 돌아와"
    python3 -m mobile_openarm_apps.llm_planner --offline "파란 상자 가져다 놔"   # API 없이 규칙 기반

로컬 LLM(LM Studio / Ollama / vLLM 등 OpenAI 호환 서버)도 사용 가능:
    export OPENAI_BASE_URL=http://localhost:1234/v1  OPENAI_API_KEY=local  OPENAI_MODEL=<모델명>

안전 설계 (교육과정 "로봇이 수행 가능한 명령과 안전한 실행 범위")
  1) 스키마: action 은 Literal 화이트리스트, target/object 는 Enum 으로 제한 → LLM 이 임의 좌표/명령을 못 만듦
  2) validate_plan(): 상태 시뮬레이션으로 전제조건 검사 (집기 전 손이 비었는지, 작업대 앞인지 등),
     빠진 이동 단계는 자동 삽입, 최대 단계 수 제한
"""
import argparse
import enum
import json
import os
import re
from typing import List, Literal, Optional

from pydantic import BaseModel, Field

LOCATIONS = ["home", "pick_station", "place_station", "door",
             "patrol_1", "patrol_2", "patrol_3", "patrol_4"]
TABLE_STATIONS = ["pick_station", "place_station"]
OBJECTS = {"box_0": "빨간 상자(red)", "box_1": "초록 상자(green)", "box_2": "파란 상자(blue)"}
MAX_STEPS = 12


class Location(str, enum.Enum):
    home = "home"
    pick_station = "pick_station"
    place_station = "place_station"
    door = "door"
    patrol_1 = "patrol_1"
    patrol_2 = "patrol_2"
    patrol_3 = "patrol_3"
    patrol_4 = "patrol_4"


class ObjectName(str, enum.Enum):
    box_0 = "box_0"
    box_1 = "box_1"
    box_2 = "box_2"


class Step(BaseModel):
    action: Literal["navigate", "pick", "place", "home_arms", "say", "wait"]
    location: Optional[Location] = Field(description="navigate/place 대상 위치, 그 외 null")
    object: Optional[ObjectName] = Field(description="pick 대상 물체, 그 외 null")
    text: Optional[str] = Field(description="say 의 문장, 그 외 null")
    seconds: Optional[float] = Field(description="wait 시간(초), 그 외 null")


class RobotPlan(BaseModel):
    feasible: bool = Field(description="요청을 이 로봇이 수행 가능한가")
    reason: str = Field(description="불가능하면 이유, 가능하면 계획 요약(한국어 한 문장)")
    steps: List[Step]


SYSTEM_PROMPT = f"""너는 주행형 양팔 로봇(OpenArm 상반신 + 바퀴 베이스)의 작업 계획기다.
사용자의 자연어 명령을 로봇이 실행할 단계 목록(RobotPlan)으로 바꿔라.

[사용 가능한 행동]
- navigate(location): 지정 위치로 자율주행
- pick(object): 현재 위치의 작업대에서 물체를 집는다 (pick_station 또는 place_station 앞에서만 가능)
- place(location): 들고 있는 물체를 그 위치의 작업대에 놓는다 (먼저 그 위치로 navigate 해야 함)
- home_arms(): 팔을 기본 자세로
- say(text): 상태를 말/로그로 보고
- wait(seconds): 대기 (최대 30초)

[위치] {LOCATIONS}  (pick_station=오른쪽(동쪽) 작업대 A, place_station=왼쪽(서쪽) 작업대 B, home=출발점)
[물체] {json.dumps(OBJECTS, ensure_ascii=False)} — 처음에는 모두 pick_station 작업대 위에 있다.

[규칙]
- 한 번에 한 물체만 운반한다. 여러 개면 왕복을 반복하라.
- 위 목록에 없는 위치·물체·행동(예: 문 열기, 사람 따라가기, 물 붓기)은 feasible=false 로 답하라.
- 사람에게 위험하거나 무관한 요청도 feasible=false.
- 단계는 최대 {MAX_STEPS}개.
"""


# ------------------------------------------------------------------ LLM 호출
def plan_with_llm(command, model=None, base_url=None, api_key=None, timeout=30.0):
    from openai import OpenAI   # 지연 import: 오프라인 모드에선 openai 없이도 동작

    client = OpenAI(base_url=base_url or os.environ.get("OPENAI_BASE_URL") or None,
                    api_key=api_key or os.environ.get("OPENAI_API_KEY"), timeout=timeout)
    model = model or os.environ.get("OPENAI_MODEL", "gpt-4o-mini")
    # 1) Responses API + Structured Outputs (OpenAI 공식 서버)
    try:
        resp = client.responses.parse(model=model, instructions=SYSTEM_PROMPT, input=command,
                                      text_format=RobotPlan)
        if resp.output_parsed is not None:
            return resp.output_parsed
    except Exception as e:  # noqa: BLE001  (로컬 서버는 Responses API 미지원일 수 있음)
        last = e
    # 2) Chat Completions + json_schema (대부분의 OpenAI 호환 로컬 서버)
    try:
        resp = client.chat.completions.parse(
            model=model, messages=[{"role": "system", "content": SYSTEM_PROMPT},
                                   {"role": "user", "content": command}],
            response_format=RobotPlan)
        parsed = resp.choices[0].message.parsed
        if parsed is not None:
            return parsed
    except Exception as e:  # noqa: BLE001
        last = e
    raise RuntimeError(f"LLM 계획 실패: {last}")


# ------------------------------------------------------------------ 규칙 기반(오프라인) 계획
COLOR_WORDS = {"box_0": ["빨간", "빨강", "red", "0번"], "box_1": ["초록", "녹색", "green", "1번"],
               "box_2": ["파란", "파랑", "blue", "2번"]}


def plan_offline(command):
    """API 키 없이 실습할 수 있는 간단한 키워드 파서 (LLM 과 결과 비교용)."""
    c = command.lower()
    objs = [o for o, ws in COLOR_WORDS.items() if any(w in c for w in ws)]
    if re.search(r"(모두|전부|all)", c):
        objs = list(OBJECTS)
    steps = []
    if "순찰" in c or "patrol" in c:
        for p in ("patrol_1", "patrol_2", "patrol_3", "patrol_4"):
            steps.append(Step(action="navigate", location=p, object=None, text=None, seconds=None))
    dest = "place_station"     # 물체는 처음에 모두 작업대 A(pick_station)에 있으므로 목적지는 B
    for o in objs:
        steps += [Step(action="navigate", location="pick_station", object=None, text=None, seconds=None),
                  Step(action="pick", location=None, object=o, text=None, seconds=None),
                  Step(action="navigate", location=dest, object=None, text=None, seconds=None),
                  Step(action="place", location=dest, object=None, text=None, seconds=None)]
    if any(w in c for w in ("돌아", "복귀", "home", "집으로")) or (objs and "그대로" not in c):
        steps.append(Step(action="navigate", location="home", object=None, text=None, seconds=None))
    if not steps:
        return RobotPlan(feasible=False, reason="이해할 수 없는 명령 (오프라인 파서)", steps=[])
    return RobotPlan(feasible=True, reason=f"오프라인 규칙 계획: {len(steps)}단계", steps=steps)


# ------------------------------------------------------------------ 검증 (안전한 실행 범위)
def validate_plan(plan: RobotPlan, start_location="home"):
    """전제조건 검사 + 자동 보정. (보정된 단계 리스트, 경고 리스트) 반환. 실패 시 ValueError."""
    if not plan.feasible:
        raise ValueError(f"계획기 판단: 수행 불가 — {plan.reason}")
    if len(plan.steps) > MAX_STEPS:
        raise ValueError(f"단계가 너무 많음 ({len(plan.steps)} > {MAX_STEPS})")
    loc = start_location
    holding = None
    out, warns = [], []
    for i, s in enumerate(plan.steps):
        a = s.action
        if a == "navigate":
            if s.location is None:
                raise ValueError(f"{i+1}단계 navigate 에 location 없음")
            loc = s.location.value if isinstance(s.location, enum.Enum) else s.location
        elif a == "pick":
            if s.object is None:
                raise ValueError(f"{i+1}단계 pick 에 object 없음")
            if holding:
                raise ValueError(f"{i+1}단계: 이미 {holding} 을(를) 들고 있어 집을 수 없음")
            if loc != "pick_station":
                warns.append(f"{i+1}단계 pick 전에 pick_station 이동을 자동 추가")
                out.append(Step(action="navigate", location="pick_station", object=None, text=None, seconds=None))
                loc = "pick_station"
            holding = s.object.value if isinstance(s.object, enum.Enum) else s.object
        elif a == "place":
            target = s.location.value if isinstance(s.location, enum.Enum) else (s.location or loc)
            if target not in TABLE_STATIONS:
                raise ValueError(f"{i+1}단계: '{target}' 에는 작업대가 없어 놓을 수 없음")
            if not holding:
                raise ValueError(f"{i+1}단계: 들고 있는 물체가 없음")
            if loc != target:
                warns.append(f"{i+1}단계 place 전에 {target} 이동을 자동 추가")
                out.append(Step(action="navigate", location=target, object=None, text=None, seconds=None))
                loc = target
            holding = None
        elif a == "wait":
            s = s.model_copy(update={"seconds": min(max(s.seconds or 1.0, 0.0), 30.0)})
        out.append(s)
    if holding:
        warns.append(f"계획 종료 시 {holding} 을(를) 들고 있음")
    return out, warns


def plan_to_json(steps):
    return [json.loads(s.model_dump_json()) for s in steps]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("command")
    ap.add_argument("--offline", action="store_true")
    ap.add_argument("--model", default=None)
    a = ap.parse_args()
    plan = plan_offline(a.command) if a.offline else plan_with_llm(a.command, a.model)
    print("RobotPlan:", plan.model_dump_json(indent=2))
    steps, warns = validate_plan(plan)
    for w in warns:
        print("경고:", w)
    print("실행 단계:")
    for i, s in enumerate(steps, 1):
        print(f"  {i}. {s.action} {s.location.value if s.location else ''}"
              f"{s.object.value if s.object else ''} {s.text or ''}")


if __name__ == "__main__":
    main()
