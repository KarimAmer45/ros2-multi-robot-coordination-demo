"""ROS 2 node that drives a Gazebo Sim robot toward assigned waypoints."""

from __future__ import annotations

import json
import math
from typing import List, Optional

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from multi_robot_coordination_demo.coordination import (
    Pose2D,
    distance_2d,
    parse_assignment_payload,
)


class GazeboRobotAgentNode(Node):
    """Waypoint follower that closes the loop with Gazebo odometry."""

    def __init__(self) -> None:
        super().__init__("gazebo_robot_agent")
        self.declare_parameter("robot_name", self.get_namespace().strip("/") or "robot")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("goal_tolerance", 0.12)
        self.declare_parameter("heading_tolerance", 0.25)
        self.declare_parameter("control_period_sec", 0.1)
        self.declare_parameter("max_linear_speed", 0.65)
        self.declare_parameter("max_angular_speed", 1.6)
        self.declare_parameter("linear_gain", 0.9)
        self.declare_parameter("angular_gain", 2.2)
        self.declare_parameter("frame_id", "map")

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.heading_tolerance = float(self.get_parameter("heading_tolerance").value)
        self.control_period_sec = float(self.get_parameter("control_period_sec").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.pose: Optional[Pose2D] = None
        self.state = "waiting_for_odom"
        self.current_task_id: Optional[str] = None
        self.completed_task_id: Optional[str] = None
        self.goal: Optional[Pose2D] = None

        self.assignment_subscription = self.create_subscription(
            String,
            "assignment",
            self._assignment_callback,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, "pose", 10)
        self.status_publisher = self.create_publisher(String, "status", 10)
        self.timer = self.create_timer(self.control_period_sec, self._control_tick)

        self.get_logger().info(
            f"{self.robot_name} Gazebo agent ready. "
            f"odom={self.odom_topic}, cmd_vel={self.cmd_vel_topic}"
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
        self.state = "moving" if self.pose is not None else "waiting_for_odom"
        self.get_logger().info(
            f"{self.robot_name} received {task_id} -> "
            f"({target_pose[0]:.2f}, {target_pose[1]:.2f})."
        )

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = _yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.pose = (float(position.x), float(position.y), yaw)

        if self.state == "waiting_for_odom":
            self.state = "moving" if self.goal is not None else "idle"

        self._publish_pose(message.header.stamp)

    def _control_tick(self) -> None:
        if self.pose is None:
            self._publish_stop()
            self._publish_status()
            return

        if self.state == "moving" and self.goal is not None:
            self._drive_toward_goal()
        else:
            self._publish_stop()

        self._publish_status()

        if self.state == "arrived":
            self.state = "idle"

    def _drive_toward_goal(self) -> None:
        if self.pose is None or self.goal is None:
            return

        distance = distance_2d(self.pose, self.goal)
        if distance <= self.goal_tolerance:
            self._complete_current_task()
            return

        target_heading = math.atan2(self.goal[1] - self.pose[1], self.goal[0] - self.pose[0])
        heading_error = _normalize_angle(target_heading - self.pose[2])

        command = Twist()
        command.angular.z = _clamp(
            self.angular_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        if abs(heading_error) < self.heading_tolerance:
            command.linear.x = _clamp(
                self.linear_gain * distance,
                0.0,
                self.max_linear_speed,
            )

        self.cmd_vel_publisher.publish(command)

    def _complete_current_task(self) -> None:
        if self.current_task_id is None:
            return

        self.completed_task_id = self.current_task_id
        self.current_task_id = None
        self.goal = None
        self.state = "arrived"
        self._publish_stop()
        self.get_logger().info(f"{self.robot_name} completed {self.completed_task_id}.")

    def _publish_pose(self, stamp) -> None:
        if self.pose is None:
            return

        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.pose.position.x = self.pose[0]
        message.pose.position.y = self.pose[1]
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
        self.completed_task_id = None

    def _publish_stop(self) -> None:
        self.cmd_vel_publisher.publish(Twist())


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GazeboRobotAgentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

