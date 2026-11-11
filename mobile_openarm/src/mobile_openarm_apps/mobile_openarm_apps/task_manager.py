#!/usr/bin/env python3
"""Day2-05 LLM → ROS 2 작업 실행: Task Manager.

  /task/command (std_msgs/String)  ← 자연어 명령 (대시보드/터미널)
  /task/status  (std_msgs/String)  → JSON 상태 (대시보드 SSE 로 전달)
  /task/cancel  (std_srvs/Trigger)

    ros2 run mobile_openarm_apps task_manager --ros-args -p planner:=llm      # OPENAI_API_KEY 필요
    ros2 run mobile_openarm_apps task_manager --ros-args -p planner:=offline  # 키 없이
    ros2 topic pub --once /task/command std_msgs/String "{data: '빨간 상자를 왼쪽 작업대로 옮기고 돌아와'}"
"""
import json
import threading
import time
import traceback

from std_msgs.msg import String
from std_srvs.srv import Trigger

from .llm_planner import plan_offline, plan_to_json, plan_with_llm, validate_plan
from .robot_skills import RobotSkills, SkillError


class TaskManager:
    def __init__(self):
        self.sk = RobotSkills.create("task_manager")
        n = self.sk
        n.declare_parameter("planner", "llm")
        n.declare_parameter("model", "")
        n.declare_parameter("fallback_offline", True)
        self.pub = n.create_publisher(String, "/task/status", 10)
        n.create_subscription(String, "/task/command", self.on_command, 10, callback_group=n.cb)
        n.create_service(Trigger, "/task/cancel", self.on_cancel, callback_group=n.cb)
        self.lock = threading.Lock()
        self.worker = None
        self.location = "home"
        self.status = {"state": "idle", "command": "", "steps": [], "current": -1, "message": "대기 중",
                       "warnings": [], "planner": n.get_parameter("planner").value, "history": []}
        n.create_timer(1.0, self.publish, callback_group=n.cb)
        n.log("task_manager 준비 완료 — /task/command 로 자연어 명령을 보내세요")

    def publish(self):
        m = String()
        m.data = json.dumps(self.status, ensure_ascii=False)
        self.pub.publish(m)

    def set(self, **kw):
        self.status.update(kw)
        self.publish()

    def on_cancel(self, req, res):
        self.sk.cancel_event.set()
        res.success = True
        res.message = "취소 요청됨"
        return res

    def on_command(self, msg):
        with self.lock:
            if self.worker and self.worker.is_alive():
                self.sk.log("이미 작업 실행 중 — 무시 (먼저 /task/cancel)")
                return
            self.sk.cancel_event.clear()
            self.worker = threading.Thread(target=self.run, args=(msg.data,), daemon=True)
            self.worker.start()

    # ------------------------------------------------------------------
    def make_plan(self, command):
        planner = self.sk.get_parameter("planner").value
        if planner == "offline":
            return plan_offline(command), "offline"
        try:
            model = self.sk.get_parameter("model").value or None
            return plan_with_llm(command, model=model), "llm"
        except Exception as e:  # noqa: BLE001
            if self.sk.get_parameter("fallback_offline").value:
                self.sk.log(f"LLM 실패 → 오프라인 계획 사용: {e}")
                return plan_offline(command), "offline(fallback)"
            raise

    def run(self, command):
        t0 = time.time()
        self.set(state="planning", command=command, steps=[], current=-1, message="계획 중...", warnings=[])
        try:
            plan, used = self.make_plan(command)
            steps, warns = validate_plan(plan, start_location=self.location)
        except Exception as e:  # noqa: BLE001
            self.set(state="rejected", message=str(e))
            self.sk.log(f"[task] 거부: {e}")
            return
        js = plan_to_json(steps)
        for s in js:
            s["status"] = "pending"
        self.set(state="running", steps=js, planner=used, message=plan.reason, warnings=warns)
        self.sk.log(f"[task] 계획({used}): {plan.reason}")
        try:
            for i, s in enumerate(steps):
                if self.sk.cancel_event.is_set():
                    raise SkillError("사용자 취소")
                js[i]["status"] = "running"
                self.set(current=i, steps=js)
                self.execute(s)
                js[i]["status"] = "done"
                self.set(steps=js)
            self.set(state="done", message=f"완료 ({time.time() - t0:.0f}s)")
        except Exception as e:  # noqa: BLE001
            if 0 <= self.status["current"] < len(js):
                js[self.status["current"]]["status"] = "failed"
            self.set(state="failed", steps=js, message=str(e))
            self.sk.get_logger().error(traceback.format_exc())
        finally:
            hist = self.status["history"][-9:] + [{"command": command, "state": self.status["state"],
                                                   "message": self.status["message"]}]
            self.set(history=hist)

    def execute(self, s):
        a = s.action
        if a == "navigate":
            loc = s.location.value
            self.sk.navigate(loc)
            self.location = loc
        elif a == "pick":
            self.sk.pick(s.object.value)
        elif a == "place":
            self.sk.place(station=(s.location.value if s.location else self.location))
        elif a == "home_arms":
            self.sk.home_arms()
        elif a == "say":
            self.sk.log(f"[say] {s.text}")
        elif a == "wait":
            time.sleep(float(s.seconds or 1.0))


def main():
    tm = TaskManager()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        tm.sk.shutdown()


if __name__ == "__main__":
    main()
