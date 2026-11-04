import os
from glob import glob

from setuptools import setup

package_name = "tron1_openarm_sim"


def tree(src, dst_root):
    out = []
    for dirpath, _, files in os.walk(src):
        if not files:
            continue
        rel = os.path.relpath(dirpath, src)
        dst = os.path.join("share", package_name, dst_root, rel) if rel != "." else os.path.join("share", package_name, dst_root)
        out.append((dst, [os.path.join(dirpath, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/maps", glob("maps/*")),
        ("share/" + package_name + "/policies", glob("policies/*")),
        ("share/" + package_name + "/tools", glob("tools/*.py")),
    ] + tree("mjcf", "mjcf"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="user@example.com",
    description="TRON1 point-foot + OpenArm MuJoCo simulation and ROS 2 bridge",
    license="Apache-2.0",
    entry_points={"console_scripts": ["tron1_bridge = tron1_openarm_sim.bridge_biped:main"]},
)
