import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "multi_robot_coordination_demo"
WORLD_NAME = "coordination_arena"


def _load_robot_configs(config_file):
    with open(config_file, "r", encoding="utf-8") as stream:
        raw_config = yaml.safe_load(stream) or {}
    return raw_config.get("robots", [])


def _launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory(PACKAGE_NAME)
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    config_file = LaunchConfiguration("config_file").perform(context)
    world_file = LaunchConfiguration("world_file").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    gz_msg_prefix = LaunchConfiguration("gz_msg_prefix").perform(context)
    robots = _load_robot_configs(config_file)

    bridge_arguments = [f"/clock@rosgraph_msgs/msg/Clock[{gz_msg_prefix}.Clock"]
    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": f"-r {world_file}"}.items(),
        ),
        Node(
            package=PACKAGE_NAME,
            executable="coordinator",
            name="coordinator",
            output="screen",
            parameters=[{"config_file": config_file}, {"use_sim_time": True}],
        ),
    ]

    for robot in robots:
        name = str(robot["name"])
        initial_pose = robot.get("initial_pose", [0.0, 0.0, 0.0])

        nodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{name}",
                output="screen",
                arguments=[
                    "-world",
                    WORLD_NAME,
                    "-file",
                    robot_model,
                    "-name",
                    name,
                    "-x",
                    str(float(initial_pose[0])),
                    "-y",
                    str(float(initial_pose[1])),
                    "-z",
                    "0.05",
                    "-Y",
                    str(float(initial_pose[2])),
                ],
            )
        )
        nodes.append(
            Node(
                package=PACKAGE_NAME,
                executable="gazebo_robot_agent",
                namespace=name,
                name="gazebo_agent",
                output="screen",
                parameters=[
                    {
                        "robot_name": name,
                        "odom_topic": f"/model/{name}/odometry",
                        "cmd_vel_topic": f"/model/{name}/cmd_vel",
                        "use_sim_time": True,
                    }
                ],
            )
        )

        bridge_arguments.extend(
            [
                f"/model/{name}/odometry@nav_msgs/msg/Odometry[{gz_msg_prefix}.Odometry",
                f"/model/{name}/cmd_vel@geometry_msgs/msg/Twist]{gz_msg_prefix}.Twist",
            ]
        )

    nodes.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gazebo_bridge",
            output="screen",
            arguments=bridge_arguments,
        )
    )

    return nodes


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)
    default_config = os.path.join(package_share, "config", "robots.yaml")
    default_world = os.path.join(package_share, "worlds", "coordination_arena.sdf")
    default_robot_model = os.path.join(
        package_share,
        "models",
        "coordination_diffbot",
        "model.sdf",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the robot and task scenario YAML file.",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=default_world,
                description="Path to the Gazebo Sim world SDF.",
            ),
            DeclareLaunchArgument(
                "robot_model",
                default_value=default_robot_model,
                description="Path to the Gazebo robot model SDF.",
            ),
            DeclareLaunchArgument(
                "gz_msg_prefix",
                default_value="gz.msgs",
                description=(
                    "Gazebo message namespace for ros_gz_bridge. "
                    "Use ignition.msgs on older ROS/Gazebo pairings."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
