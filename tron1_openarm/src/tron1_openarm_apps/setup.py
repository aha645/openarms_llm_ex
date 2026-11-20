from glob import glob

from setuptools import setup

package_name = "tron1_openarm_apps"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="user@example.com",
    description="TRON1 + OpenArm apps (skills, LLM planner, task manager, dashboard)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "patrol = tron1_openarm_apps.patrol:main",
            "arm_demo = tron1_openarm_apps.arm_demo:main",
            "pick_place = tron1_openarm_apps.pick_place:main",
            "nav_pick_place = tron1_openarm_apps.nav_pick_place:main",
            "llm_planner = tron1_openarm_apps.llm_planner:main",
            "task_manager = tron1_openarm_apps.task_manager:main",
            "dashboard = tron1_openarm_apps.dashboard:main",
            "aruco_detector = tron1_openarm_apps.aruco_detector:main",
        ],
    },
)
