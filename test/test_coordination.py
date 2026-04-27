import json

import pytest

from multi_robot_coordination_demo.coordination import (
    TaskConfig,
    build_assignment_payload,
    collides_with_reserved_goal,
    parse_assignment_payload,
    parse_demo_config,
)


def test_parse_demo_config_rejects_duplicate_robot_names():
    with pytest.raises(ValueError, match="Robot names"):
        parse_demo_config(
            {
                "robots": [
                    {"name": "robot_1", "initial_pose": [0.0, 0.0, 0.0]},
                    {"name": "robot_1", "initial_pose": [1.0, 0.0, 0.0]},
                ],
                "tasks": [{"id": "task_1", "pose": [2.0, 0.0, 0.0]}],
            }
        )


def test_assignment_payload_round_trip():
    payload = build_assignment_payload(TaskConfig("inspect", (1.25, -0.5, 0.2)))
    task_id, pose = parse_assignment_payload(payload)

    assert json.loads(payload)["task_id"] == "inspect"
    assert task_id == "inspect"
    assert pose == (1.25, -0.5, 0.2)


def test_reserved_goal_collision_uses_radius():
    assert collides_with_reserved_goal((0.2, 0.0, 0.0), [(0.0, 0.0, 0.0)], 0.5)
    assert not collides_with_reserved_goal((0.8, 0.0, 0.0), [(0.0, 0.0, 0.0)], 0.5)

