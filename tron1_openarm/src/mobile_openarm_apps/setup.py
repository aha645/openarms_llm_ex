from glob import glob

from setuptools import setup

package_name = "mobile_openarm_apps"
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
    description="Mobile OpenArm curriculum apps",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "patrol = mobile_openarm_apps.patrol:main",
            "arm_demo = mobile_openarm_apps.arm_demo:main",
            "pick_place = mobile_openarm_apps.pick_place:main",
            "nav_pick_place = mobile_openarm_apps.nav_pick_place:main",
            "llm_planner = mobile_openarm_apps.llm_planner:main",
            "task_manager = mobile_openarm_apps.task_manager:main",
            "dashboard = mobile_openarm_apps.dashboard:main",
            "aruco_detector = mobile_openarm_apps.aruco_detector:main",
        ],
    },
)
