import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "multi_robot_coordination_demo"


def _load_robot_configs(config_file):
    with open(config_file, "r", encoding="utf-8") as stream:
        raw_config = yaml.safe_load(stream) or {}
    return raw_config.get("robots", [])


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    robots = _load_robot_configs(config_file)

    nodes = [
        Node(
            package=PACKAGE_NAME,
            executable="coordinator",
            name="coordinator",
            output="screen",
            parameters=[{"config_file": config_file}],
        )
    ]

    for robot in robots:
        name = str(robot["name"])
        initial_pose = robot.get("initial_pose", [0.0, 0.0, 0.0])
        nodes.append(
            Node(
                package=PACKAGE_NAME,
                executable="robot_agent",
                namespace=name,
                name="agent",
                output="screen",
                parameters=[
                    {
                        "robot_name": name,
                        "initial_x": float(initial_pose[0]),
                        "initial_y": float(initial_pose[1]),
                        "initial_yaw": float(initial_pose[2]),
                    }
                ],
            )
        )

    return nodes


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "config",
        "robots.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the robot and task scenario YAML file.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

