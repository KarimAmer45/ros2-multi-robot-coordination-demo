from glob import glob
import os

from setuptools import find_packages, setup


package_name = "multi_robot_coordination_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KarimAmer45",
    maintainer_email="karimamer456@gmail.com",
    description="A mini ROS 2 demo for coordinating multiple simulated robot agents.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "coordinator = multi_robot_coordination_demo.coordinator_node:main",
            "robot_agent = multi_robot_coordination_demo.robot_agent_node:main",
        ],
    },
)

