# ROS 2 Multi-Robot Coordination Mini Demo

A compact ROS 2 demo that coordinates multiple simulated robots with waypoint assignments, simple goal reservations, and namespaced robot topics.

The demo uses only standard ROS 2 message packages, so it is easy to build, inspect, and extend without creating custom interfaces first.

## Features

- Coordinator node assigns queued waypoint tasks to idle robots.
- Multiple robot agent nodes run in separate namespaces.
- Robot agents simulate simple 2D movement toward assigned goals.
- Goal reservation radius avoids assigning robots to conflicting targets.
- JSON payloads over `std_msgs/String` keep the package lightweight.
- Pure helper tests cover assignment payloads and reservation behavior.

## Project Layout

```text
.
├── config/
│   └── robots.yaml
├── launch/
│   └── demo.launch.py
├── multi_robot_coordination_demo/
│   ├── coordination.py
│   ├── coordinator_node.py
│   └── robot_agent_node.py
├── test/
│   └── test_coordination.py
├── package.xml
├── setup.cfg
└── setup.py
```

## Quick Start

Clone this repository into a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/KarimAmer45/ros2-multi-robot-coordination-demo.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select multi_robot_coordination_demo
source install/setup.bash
ros2 launch multi_robot_coordination_demo demo.launch.py
```

Watch the coordination event stream in another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /coordination/events
```

## Topics

Each robot is launched in its own namespace:

```text
/robot_1/assignment
/robot_1/pose
/robot_1/status
/robot_2/assignment
/robot_2/pose
/robot_2/status
```

The coordinator publishes:

```text
/coordination/events
```

## Customize The Scenario

Edit `config/robots.yaml` to change the robot fleet, starting poses, task queue, assignment period, or reservation radius.

You can also launch with a custom config file:

```bash
ros2 launch multi_robot_coordination_demo demo.launch.py config_file:=/absolute/path/to/robots.yaml
```

## Run Tests

```bash
colcon test --packages-select multi_robot_coordination_demo
colcon test-result --verbose
```

The included tests exercise pure Python coordination helpers, so they run quickly and do not need live ROS nodes.

## Next Steps

Good extensions for this mini demo:

- Add Gazebo Sim robots with `ros_gz`.
- Replace JSON payloads with typed ROS 2 interfaces.
- Add Nav2 for map-aware multi-robot navigation.
- Add task priorities, cancellation, and charging behavior.

