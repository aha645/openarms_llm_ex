#!/usr/bin/env python3
"""Nav2 Jazzy 기본 nav2_params.yaml → 본 로봇(차동구동 0.54x0.50m, base_footprint) 용으로 변환.

    python3 make_nav2_params.py --src ../config/nav2_params_upstream_jazzy.yaml --out ../config/nav2_params.yaml
"""
import argparse
import yaml

FOOTPRINT = "[[0.28, 0.26], [0.28, -0.26], [-0.27, -0.26], [-0.27, 0.26]]"


def walk(node, fn):
    if isinstance(node, dict):
        for k in list(node.keys()):
            node[k] = fn(k, walk(node[k], fn))
    elif isinstance(node, list):
        return [walk(x, fn) for x in node]
    return node


def main(src, out):
    p = yaml.safe_load(open(src))

    def fix(k, v):
        if k in ("robot_base_frame", "base_frame", "base_frame_id") and v in ("base_link", "base_footprint"):
            return "base_footprint"
        return v
    walk(p, fix)

    amcl = p["amcl"]["ros__parameters"]
    amcl.update({"laser_max_range": 12.0, "laser_min_range": 0.12, "max_beams": 90,
                 "set_initial_pose": True,
                 "initial_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}})

    # CPU 가 바쁠 때(시뮬+Nav2+MoveIt 동시) BT 가 planner 응답을 20ms 만 기다리다 goal 을 포기하는 문제 방지
    p["bt_navigator"]["ros__parameters"].update({"default_server_timeout": 500, "wait_for_service_timeout": 5000})
    p["planner_server"]["ros__parameters"]["expected_planner_frequency"] = 5.0

    ctrl = p["controller_server"]["ros__parameters"]
    ctrl["controller_frequency"] = 20.0           # MPPI 는 1/주기 >= 1/model_dt(0.05s) 이어야 함 → 20Hz 미만 불가
    mppi = ctrl["FollowPath"]
    mppi.update({"batch_size": 1000, "time_steps": 48, "vx_max": 0.4, "vx_min": -0.25,
                 "wz_max": 1.2, "ax_max": 2.0, "az_max": 2.5})
    gc = ctrl["general_goal_checker"]
    gc.update({"xy_goal_tolerance": 0.12, "yaw_goal_tolerance": 0.15})   # 남는 오차는 도킹 단계가 보정
    ctrl["progress_checker"].update({"required_movement_radius": 0.3, "movement_time_allowance": 20.0})

    for cm in ("local_costmap", "global_costmap"):
        c = p[cm][cm]["ros__parameters"]
        c.pop("robot_radius", None)
        c["footprint"] = FOOTPRINT
        c["footprint_padding"] = 0.02
        c["inflation_layer"]["inflation_radius"] = 0.45
        c["inflation_layer"]["cost_scaling_factor"] = 5.0
        for layer in ("voxel_layer", "obstacle_layer"):
            if layer in c:
                c[layer]["scan"].update({"raytrace_max_range": 6.0, "obstacle_max_range": 5.5,
                                         "max_obstacle_height": 2.0})
    p["local_costmap"]["local_costmap"]["ros__parameters"].update({"width": 4, "height": 4})
    vs = p["velocity_smoother"]["ros__parameters"]
    vs["max_velocity"] = [0.4, 0.0, 1.2]
    vs["min_velocity"] = [-0.25, 0.0, -1.2]

    with open(out, "w") as f:
        f.write("# 자동 생성: tools/make_nav2_params.py (원본: nav2_bringup jazzy params/nav2_params.yaml)\n")
        yaml.safe_dump(p, f, sort_keys=False)
    print("wrote", out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    main(a.src, a.out)
