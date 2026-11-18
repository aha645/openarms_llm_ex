"""검증용: Nav2 목표 하나를 보내고 도착할 때까지 정답 자세·오도메트리·EKF·AMCL·명령을 기록한다.
    python3 nav_goal_test.py <x> <y> <yaw_deg> <기록.jsonl> [제한 초]
끝에 한 줄 요약(JSON): 결과, 걸린 시간, 도착 위치 오차(정답 기준), 오도메트리·EKF 의 흐름(정답 대비 이동량 차이), 옆 명령을 쓴 비율."""
import json, math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    gx, gy, gyaw = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
    out = open(sys.argv[4], "w")
    limit = float(sys.argv[5]) if len(sys.argv) > 5 else 400.0
    rclpy.init()
    n = Node("nav_goal_test", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    S = {}
    pose = lambda p: [p.position.x, p.position.y, yaw(p.orientation)]
    n.create_subscription(PoseStamped, "/ground_truth/pose", lambda m: S.__setitem__("gt", pose(m.pose)), 10)
    n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: S.__setitem__("amcl", pose(m.pose.pose)), 10)
    n.create_subscription(Odometry, "/odom", lambda m: S.__setitem__("odom", pose(m.pose.pose)), 10)
    n.create_subscription(Odometry, "/odometry/filtered", lambda m: S.__setitem__("ekf", pose(m.pose.pose)), 10)
    n.create_subscription(Twist, "/cmd_vel", lambda m: S.__setitem__("cmd", [m.linear.x, m.linear.y, m.angular.z]), 10)
    ac = ActionClient(n, NavigateToPose, "navigate_to_pose")
    if not ac.wait_for_server(timeout_sec=60):
        print(json.dumps({"result": "no_server"})); return
    t_end = time.time() + 20
    while "gt" not in S and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = "map"
    g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    fut = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=30)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print(json.dumps({"result": "rejected"})); return
    res = gh.get_result_async()
    t0 = time.time(); last = 0; rec = []
    while not res.done() and time.time() - t0 < limit:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - last > 0.5:
            last = time.time()
            r = {"t": round(last - t0, 2), **{k: [round(v, 4) for v in S[k]] for k in ("gt", "odom", "ekf", "amcl", "cmd") if k in S}}
            rec.append(r); out.write(json.dumps(r) + "\n"); out.flush()
    status = res.result().status if res.done() else -1
    gt = S.get("gt", [math.nan] * 3)
    summ = {"result": {4: "succeeded", 5: "canceled", 6: "aborted"}.get(status, str(status)), "time_s": round(time.time() - t0, 1),
            "goal_err_m": round(math.hypot(gt[0] - gx, gt[1] - gy), 3),
            "goal_err_deg": round(math.degrees(math.atan2(math.sin(gt[2] - gyaw), math.cos(gt[2] - gyaw))), 1)}
    if rec and "gt" in rec[0]:
        a, b = rec[0], rec[-1]
        dg = [b["gt"][0] - a["gt"][0], b["gt"][1] - a["gt"][1]]
        L = math.hypot(*dg)
        summ["true_path_m"] = round(sum(math.hypot(r2["gt"][0] - r1["gt"][0], r2["gt"][1] - r1["gt"][1]) for r1, r2 in zip(rec, rec[1:])), 2)
        for k in ("odom", "ekf"):
            if k in a and k in b:
                # odom 프레임은 시작 방향만큼 돌아가 있으므로 이동 거리·회전량으로 비교
                dk = math.hypot(b[k][0] - a[k][0], b[k][1] - a[k][1])
                summ[f"{k}_dist_err_m"] = round(dk - L, 3)
        if "amcl" in b:
            summ["amcl_err_m"] = round(math.hypot(b["amcl"][0] - b["gt"][0], b["amcl"][1] - b["gt"][1]), 3)
        cm = [r["cmd"] for r in rec if "cmd" in r]
        if cm:
            summ["vy_used_frac"] = round(sum(abs(c[1]) > 0.01 for c in cm) / len(cm), 2)
    print(json.dumps(summ), flush=True)


if __name__ == "__main__":
    main()
