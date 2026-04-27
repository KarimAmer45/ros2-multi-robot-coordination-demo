"""Shared configuration and assignment helpers for the coordination demo."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Iterable, Mapping, Optional, Tuple


Pose2D = Tuple[float, float, float]


DEFAULT_CONFIG = {
    "coordinator": {
        "assignment_period_sec": 1.0,
        "reservation_radius": 0.75,
    },
    "robots": [
        {"name": "robot_1", "initial_pose": [0.0, 0.0, 0.0]},
        {"name": "robot_2", "initial_pose": [0.0, 2.0, 0.0]},
    ],
    "tasks": [
        {"id": "inspect_a", "pose": [2.0, 0.0, 0.0]},
        {"id": "inspect_b", "pose": [2.0, 2.0, 0.0]},
    ],
}


@dataclass(frozen=True)
class RobotConfig:
    """Static configuration for one robot agent."""

    name: str
    initial_pose: Pose2D


@dataclass(frozen=True)
class TaskConfig:
    """A waypoint task the coordinator can assign to a robot."""

    task_id: str
    pose: Pose2D


@dataclass(frozen=True)
class DemoConfig:
    """Complete scenario configuration."""

    assignment_period_sec: float
    reservation_radius: float
    robots: Tuple[RobotConfig, ...]
    tasks: Tuple[TaskConfig, ...]


def load_demo_config(config_file: Optional[str]) -> DemoConfig:
    """Load a scenario YAML file or return the built-in fallback scenario."""

    if not config_file:
        return parse_demo_config(DEFAULT_CONFIG)

    path = Path(config_file).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"Config file does not exist: {path}")

    import yaml

    with path.open("r", encoding="utf-8") as stream:
        raw_config = yaml.safe_load(stream) or {}

    return parse_demo_config(raw_config)


def parse_demo_config(raw_config: Optional[Mapping[str, object]]) -> DemoConfig:
    """Parse a raw YAML mapping into typed demo configuration."""

    raw_config = raw_config or DEFAULT_CONFIG
    coordinator = {
        **DEFAULT_CONFIG["coordinator"],
        **dict(raw_config.get("coordinator", {}) or {}),
    }

    robots_raw = raw_config.get("robots", DEFAULT_CONFIG["robots"])
    tasks_raw = raw_config.get("tasks", DEFAULT_CONFIG["tasks"])

    robots = tuple(_parse_robot(robot, index) for index, robot in enumerate(robots_raw))
    tasks = tuple(_parse_task(task, index) for index, task in enumerate(tasks_raw))

    if not robots:
        raise ValueError("Scenario must define at least one robot.")
    if not tasks:
        raise ValueError("Scenario must define at least one task.")

    robot_names = [robot.name for robot in robots]
    if len(robot_names) != len(set(robot_names)):
        raise ValueError("Robot names must be unique.")

    task_ids = [task.task_id for task in tasks]
    if len(task_ids) != len(set(task_ids)):
        raise ValueError("Task ids must be unique.")

    return DemoConfig(
        assignment_period_sec=float(coordinator["assignment_period_sec"]),
        reservation_radius=float(coordinator["reservation_radius"]),
        robots=robots,
        tasks=tasks,
    )


def build_assignment_payload(task: TaskConfig) -> str:
    """Create the JSON payload sent to a robot assignment topic."""

    return json.dumps(
        {
            "task_id": task.task_id,
            "target": {
                "x": task.pose[0],
                "y": task.pose[1],
                "yaw": task.pose[2],
            },
        },
        separators=(",", ":"),
    )


def parse_assignment_payload(payload: str) -> Tuple[str, Pose2D]:
    """Parse a robot assignment JSON payload."""

    data = json.loads(payload)
    target = data["target"]
    return (
        str(data["task_id"]),
        (float(target["x"]), float(target["y"]), float(target.get("yaw", 0.0))),
    )


def distance_2d(first: Sequence[float], second: Sequence[float]) -> float:
    """Return planar distance between two pose-like sequences."""

    return math.hypot(float(first[0]) - float(second[0]), float(first[1]) - float(second[1]))


def collides_with_reserved_goal(
    target_pose: Sequence[float],
    reserved_goals: Iterable[Sequence[float]],
    reservation_radius: float,
) -> bool:
    """Return true when a target is too close to any active goal reservation."""

    return any(
        distance_2d(target_pose, reserved_goal) < reservation_radius
        for reserved_goal in reserved_goals
    )


def _parse_robot(raw_robot: Mapping[str, object], index: int) -> RobotConfig:
    name = str(raw_robot.get("name", f"robot_{index + 1}"))
    return RobotConfig(name=name, initial_pose=_parse_pose(raw_robot.get("initial_pose")))


def _parse_task(raw_task: Mapping[str, object], index: int) -> TaskConfig:
    task_id = str(raw_task.get("id", f"task_{index + 1}"))
    return TaskConfig(task_id=task_id, pose=_parse_pose(raw_task.get("pose")))


def _parse_pose(raw_pose: object) -> Pose2D:
    if not isinstance(raw_pose, Sequence) or isinstance(raw_pose, (str, bytes)):
        raise ValueError("Pose must be a sequence: [x, y, yaw].")
    if len(raw_pose) != 3:
        raise ValueError("Pose must contain exactly three values: [x, y, yaw].")
    return (float(raw_pose[0]), float(raw_pose[1]), float(raw_pose[2]))

