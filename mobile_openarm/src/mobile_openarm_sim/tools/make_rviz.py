#!/usr/bin/env python3
"""RViz2 설정 파일 생성 (sim.rviz: odom 고정프레임, nav.rviz: map 고정프레임)."""
import os
import sys
import yaml

def display(cls, name, **kw):
    d = {"Class": cls, "Name": name, "Enabled": True}
    d.update(kw)
    return d

def topic(t, reliability="Reliable", durability="Volatile"):
    return {"Value": t, "Depth": 5, "History Policy": "Keep Last",
            "Reliability Policy": reliability, "Durability Policy": durability}

def config(fixed, with_map):
    ds = [
        display("rviz_default_plugins/Grid", "Grid", **{"Cell Size": 1, "Plane Cell Count": 20,
                "Color": "160; 160; 164", "Reference Frame": "<Fixed Frame>"}),
        display("rviz_default_plugins/RobotModel", "RobotModel", **{
            "Description Source": "Topic", "Description Topic": topic("/robot_description", durability="Transient Local"),
            "Visual Enabled": True, "Collision Enabled": False, "Alpha": 1}),
        display("rviz_default_plugins/TF", "TF", **{"Show Names": False, "Marker Scale": 0.3, "Enabled": False}),
        display("rviz_default_plugins/LaserScan", "LaserScan", **{"Topic": topic("/scan", "Best Effort"),
                "Size (m)": 0.04, "Style": "Points", "Color Transformer": "FlatColor", "Color": "255; 0; 0"}),
        display("rviz_default_plugins/MarkerArray", "ArUco", **{"Topic": topic("/aruco/markers")}),
        display("rviz_default_plugins/Image", "HeadCamera", **{"Topic": topic("/head_camera/image_raw", "Best Effort"),
                "Enabled": False}),
    ]
    if with_map:
        ds += [
            display("rviz_default_plugins/Map", "Map", **{"Topic": topic("/map", durability="Transient Local"),
                    "Color Scheme": "map", "Alpha": 0.8}),
            display("rviz_default_plugins/Map", "GlobalCostmap", **{"Topic": topic("/global_costmap/costmap"),
                    "Color Scheme": "costmap", "Alpha": 0.3}),
            display("rviz_default_plugins/Map", "LocalCostmap", **{"Topic": topic("/local_costmap/costmap"),
                    "Color Scheme": "costmap", "Alpha": 0.4}),
            display("rviz_default_plugins/Path", "GlobalPlan", **{"Topic": topic("/plan"), "Color": "25; 255; 0"}),
            display("rviz_default_plugins/Path", "LocalPlan", **{"Topic": topic("/local_plan"), "Color": "0; 12; 255"}),
            display("rviz_default_plugins/PoseArray", "AmclParticles", **{"Topic": topic("/particle_cloud", "Best Effort"),
                    "Enabled": False}),
            display("rviz_default_plugins/Polygon", "Footprint", **{"Topic": topic("/local_costmap/published_footprint")}),
        ]
    return {
        "Panels": [{"Class": "rviz_common/Displays", "Name": "Displays"},
                   {"Class": "rviz_common/Tool Properties", "Name": "Tool Properties"},
                   {"Class": "rviz_common/Time", "Name": "Time"}],
        "Visualization Manager": {
            "Class": "", "Displays": ds, "Enabled": True,
            "Global Options": {"Background Color": "48; 48; 48", "Fixed Frame": fixed, "Frame Rate": 30},
            "Name": "root",
            "Tools": [{"Class": "rviz_default_plugins/Interact"}, {"Class": "rviz_default_plugins/MoveCamera"},
                      {"Class": "rviz_default_plugins/Select"}, {"Class": "rviz_default_plugins/Measure"},
                      {"Class": "rviz_default_plugins/SetInitialPose", "Topic": topic("/initialpose")},
                      {"Class": "rviz_default_plugins/SetGoal", "Topic": topic("/goal_pose")},
                      {"Class": "rviz_default_plugins/PublishPoint", "Topic": topic("/clicked_point")}],
            "Value": True,
            "Views": {"Current": {"Class": "rviz_default_plugins/Orbit", "Distance": 9.0, "Pitch": 0.9,
                                  "Yaw": 4.0, "Focal Point": {"X": 0, "Y": 0, "Z": 0}, "Target Frame": "<Fixed Frame>",
                                  "Name": "Current View"}, "Saved": None},
        },
        "Window Geometry": {"Height": 900, "Width": 1400, "Hide Left Dock": False, "Hide Right Dock": True},
    }

out = sys.argv[1]
os.makedirs(out, exist_ok=True)
yaml.safe_dump(config("odom", False), open(os.path.join(out, "sim.rviz"), "w"), sort_keys=False)
yaml.safe_dump(config("map", True), open(os.path.join(out, "nav.rviz"), "w"), sort_keys=False)
print("wrote rviz configs")
