"""ROS 2 node that assigns waypoint tasks to simulated robot agents."""

from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Dict, List, Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from multi_robot_coordination_demo.coordination import (
    Pose2D,
    TaskConfig,
    build_assignment_payload,
    collides_with_reserved_goal,
    distance_2d,
    load_demo_config,
)


@dataclass
class RobotRuntimeState:
    """State the coordinator tracks for one robot."""

    pose: Optional[Pose2D]
    state: str = "idle"
    active_goal: Optional[Pose2D] = None


class CoordinatorNode(Node):
    """Assign queued waypoint tasks to idle robots."""

    def __init__(self) -> None:
        super().__init__("coordinator")
        self.declare_parameter("config_file", "")

        config_file = self.get_parameter("config_file").value
        self.config = load_demo_config(str(config_file) if config_file else "")
        self.pending_tasks: List[TaskConfig] = list(self.config.tasks)
        self.robot_states: Dict[str, RobotRuntimeState] = {
            robot.name: RobotRuntimeState(pose=robot.initial_pose)
            for robot in self.config.robots
        }
        self.robot_assignments: Dict[str, TaskConfig] = {}
        self.assignment_publishers: Dict[str, object] = {}
        self._subscriptions = []

        self.event_publisher = self.create_publisher(String, "/coordination/events", 10)

        for robot in self.config.robots:
            self.assignment_publishers[robot.name] = self.create_publisher(
                String,
                f"/{robot.name}/assignment",
                10,
            )
            self._subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    f"/{robot.name}/pose",
                    lambda msg, robot_name=robot.name: self._pose_callback(robot_name, msg),
                    10,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    String,
                    f"/{robot.name}/status",
                    lambda msg, robot_name=robot.name: self._status_callback(robot_name, msg),
                    10,
                )
            )

        self.timer = self.create_timer(
            self.config.assignment_period_sec,
            self._assignment_tick,
        )

        self.get_logger().info(
            "Coordinator ready for "
            f"{len(self.robot_states)} robots and {len(self.pending_tasks)} tasks."
        )

    def _pose_callback(self, robot_name: str, message: PoseStamped) -> None:
        pose = message.pose
        self.robot_states[robot_name].pose = (
            float(pose.position.x),
            float(pose.position.y),
            0.0,
        )

    def _status_callback(self, robot_name: str, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Ignoring invalid status from {robot_name}: {message.data}")
            return

        robot_state = self.robot_states[robot_name]
        robot_state.state = str(status.get("state", "unknown"))
        completed_task_id = status.get("completed_task_id")

        if completed_task_id:
            self._complete_task(robot_name, str(completed_task_id))

    def _assignment_tick(self) -> None:
        if not self.pending_tasks:
            return

        reserved_goals = [
            robot_state.active_goal
            for robot_state in self.robot_states.values()
            if robot_state.active_goal is not None
        ]

        idle_robot_names = [
            robot_name
            for robot_name, state in self.robot_states.items()
            if robot_name not in self.robot_assignments and state.pose is not None
        ]
        idle_robot_names.sort()

        for robot_name in idle_robot_names:
            if not self.pending_tasks:
                return

            task = self._select_task_for_robot(robot_name, reserved_goals)
            if task is None:
                continue

            self._assign_task(robot_name, task)
            reserved_goals.append(task.pose)

    def _select_task_for_robot(
        self,
        robot_name: str,
        reserved_goals: List[Pose2D],
    ) -> Optional[TaskConfig]:
        robot_pose = self.robot_states[robot_name].pose
        if robot_pose is None:
            return None

        best_index: Optional[int] = None
        best_distance = float("inf")

        for index, task in enumerate(self.pending_tasks):
            if collides_with_reserved_goal(
                task.pose,
                reserved_goals,
                self.config.reservation_radius,
            ):
                continue

            task_distance = distance_2d(robot_pose, task.pose)
            if task_distance < best_distance:
                best_distance = task_distance
                best_index = index

        if best_index is None:
            return None

        return self.pending_tasks.pop(best_index)

    def _assign_task(self, robot_name: str, task: TaskConfig) -> None:
        message = String()
        message.data = build_assignment_payload(task)
        self.assignment_publishers[robot_name].publish(message)

        self.robot_assignments[robot_name] = task
        self.robot_states[robot_name].active_goal = task.pose
        self.robot_states[robot_name].state = "assigned"

        self._publish_event(
            {
                "event": "task_assigned",
                "robot": robot_name,
                "task_id": task.task_id,
                "target": {"x": task.pose[0], "y": task.pose[1], "yaw": task.pose[2]},
            }
        )
        self.get_logger().info(f"Assigned {task.task_id} to {robot_name}.")

    def _complete_task(self, robot_name: str, task_id: str) -> None:
        assigned_task = self.robot_assignments.get(robot_name)
        if assigned_task is None or assigned_task.task_id != task_id:
            return

        self.robot_assignments.pop(robot_name)
        self.robot_states[robot_name].active_goal = None
        self.robot_states[robot_name].state = "idle"

        self._publish_event(
            {
                "event": "task_completed",
                "robot": robot_name,
                "task_id": task_id,
            }
        )
        self.get_logger().info(f"{robot_name} completed {task_id}.")

    def _publish_event(self, payload: Dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.event_publisher.publish(message)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

