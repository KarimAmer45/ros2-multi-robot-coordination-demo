# Gazebo Sim Integration

This package includes an optional Gazebo Sim launch path for the same coordinator/task flow used by the lightweight ROS-only demo.

## Why Gazebo Sim

Gazebo Sim with `ros_gz` is the current ROS 2 integration path. The older Gazebo Classic stack is not the best fit for a new demo. This package keeps Gazebo optional so the core coordination logic remains easy to run in environments without a simulator.

## Launch

Install the ROS-Gazebo integration packages for your ROS 2 distribution, then build the workspace:

```bash
sudo apt install ros-$ROS_DISTRO-ros-gz ros-$ROS_DISTRO-ros-gz-sim ros-$ROS_DISTRO-ros-gz-bridge
colcon build --packages-select multi_robot_coordination_demo
source install/setup.bash
```

Start the Gazebo-backed demo:

```bash
ros2 launch multi_robot_coordination_demo gazebo_demo.launch.py
```

If your `ros_gz_bridge` version expects old Ignition message type names, switch the bridge prefix:

```bash
ros2 launch multi_robot_coordination_demo gazebo_demo.launch.py gz_msg_prefix:=ignition.msgs
```

Watch coordination events:

```bash
ros2 topic echo /coordination/events
```

## Architecture

```text
Coordinator
  ├── publishes /robot_N/assignment
  ├── subscribes /robot_N/pose
  └── subscribes /robot_N/status

Gazebo robot agent
  ├── subscribes /robot_N/assignment
  ├── subscribes /model/robot_N/odometry
  ├── publishes /model/robot_N/cmd_vel
  ├── republishes /robot_N/pose
  └── publishes /robot_N/status

ros_gz_bridge
  ├── bridges /clock from Gazebo to ROS
  ├── bridges /model/robot_N/odometry from Gazebo to ROS
  └── bridges /model/robot_N/cmd_vel from ROS to Gazebo
```

The Gazebo robot uses the built-in `gz::sim::systems::DiffDrive` system. The coordinator is unchanged; only the robot agent changes from an in-process kinematic simulator to a closed-loop waypoint follower using Gazebo odometry.

## Notes

- The model is intentionally simple: two driven wheels, a box chassis, and a front marker.
- The controller is a proportional waypoint follower, not a full navigation stack.
- For obstacle-aware navigation, the next better step is Nav2 with maps and per-robot navigation namespaces.
