"""ROS 2 node that simulates one robot moving toward assigned waypoints."""

from __future__ import annotations

import json
import math
from typing import List, Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from multi_robot_coordination_demo.coordination import (
    Pose2D,
    distance_2d,
    parse_assignment_payload,
)


class RobotAgentNode(Node):
    """A tiny 2D robot simulator controlled by assignment messages."""

    def __init__(self) -> None:
        super().__init__("robot_agent")
        self.declare_parameter("robot_name", self.get_namespace().strip("/") or "robot")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("speed", 0.6)
        self.declare_parameter("goal_tolerance", 0.08)
        self.declare_parameter("control_period_sec", 0.1)
        self.declare_parameter("frame_id", "map")

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.pose = [
            float(self.get_parameter("initial_x").value),
            float(self.get_parameter("initial_y").value),
            float(self.get_parameter("initial_yaw").value),
        ]
        self.speed = float(self.get_parameter("speed").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.control_period_sec = float(self.get_parameter("control_period_sec").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.state = "idle"
        self.current_task_id: Optional[str] = None
        self.completed_task_id: Optional[str] = None
        self.goal: Optional[Pose2D] = None

        self.assignment_subscription = self.create_subscription(
            String,
            "assignment",
            self._assignment_callback,
            10,
        )
        self.pose_publisher = self.create_publisher(PoseStamped, "pose", 10)
        self.status_publisher = self.create_publisher(String, "status", 10)
        self.timer = self.create_timer(self.control_period_sec, self._control_tick)

        self.get_logger().info(
            f"{self.robot_name} ready at ({self.pose[0]:.2f}, {self.pose[1]:.2f})."
        )

    def _assignment_callback(self, message: String) -> None:
        try:
            task_id, target_pose = parse_assignment_payload(message.data)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"Ignoring invalid assignment: {error}")
            return

        self.current_task_id = task_id
        self.completed_task_id = None
        self.goal = target_pose
        self.state = "moving"
        self.get_logger().info(
            f"{self.robot_name} received {task_id} -> "
            f"({target_pose[0]:.2f}, {target_pose[1]:.2f})."
        )

    def _control_tick(self) -> None:
        if self.state == "moving" and self.goal is not None:
            self._move_toward_goal()

        self._publish_pose()
        self._publish_status()

        if self.state == "arrived":
            self.state = "idle"

    def _move_toward_goal(self) -> None:
        if self.goal is None:
            return

        if distance_2d(self.pose, self.goal) <= self.goal_tolerance:
            self._complete_current_task()
            return

        dx = self.goal[0] - self.pose[0]
        dy = self.goal[1] - self.pose[1]
        distance = math.hypot(dx, dy)
        step = min(self.speed * self.control_period_sec, distance)

        if distance > 0.0:
            self.pose[0] += step * dx / distance
            self.pose[1] += step * dy / distance
            self.pose[2] = math.atan2(dy, dx)

        if distance_2d(self.pose, self.goal) <= self.goal_tolerance:
            self.pose[0] = self.goal[0]
            self.pose[1] = self.goal[1]
            self.pose[2] = self.goal[2]
            self._complete_current_task()

    def _complete_current_task(self) -> None:
        if self.current_task_id is None:
            return

        self.completed_task_id = self.current_task_id
        self.current_task_id = None
        self.goal = None
        self.state = "arrived"
        self.get_logger().info(f"{self.robot_name} completed {self.completed_task_id}.")

    def _publish_pose(self) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.pose.position.x = float(self.pose[0])
        message.pose.position.y = float(self.pose[1])
        message.pose.position.z = 0.0

        half_yaw = self.pose[2] * 0.5
        message.pose.orientation.z = math.sin(half_yaw)
        message.pose.orientation.w = math.cos(half_yaw)
        self.pose_publisher.publish(message)

    def _publish_status(self) -> None:
        payload = {
            "robot": self.robot_name,
            "state": self.state,
        }
        if self.current_task_id is not None:
            payload["task_id"] = self.current_task_id
        if self.completed_task_id is not None:
            payload["completed_task_id"] = self.completed_task_id

        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.status_publisher.publish(message)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RobotAgentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

