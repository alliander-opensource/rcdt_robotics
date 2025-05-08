# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import subprocess

import pytest
import rclpy
import rclpy.node
from rcdt_utilities.launch_utils import (
    assert_for_message,
)
from rcdt_utilities.test_utils import (
    call_move_gripper_service,
    follow_joint_trajectory_goal,
    get_joint_position,
)
from sensor_msgs.msg import JointState


class FrankaCoreTests:
    """Base class: override LAUNCH_FILE in subclasses to point at any launch."""

    def test_joint_states_published(self) -> None:
        assert_for_message(JointState, "/joint_states", 60)

    def test_controller_is_active(self) -> None:
        result = subprocess.run(
            ["ros2", "control", "list_controllers"],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        assert "joint_state_broadcaster" in result.stdout
        assert "fr3_arm_controller" in result.stdout
        assert "gripper_action_controller" in result.stdout

    def test_can_move_fingers(self, singleton_node: rclpy.node.Node) -> None:
        assert call_move_gripper_service(singleton_node, width=0.08, speed=0.03) is True
        assert get_joint_position("fr3_finger_joint1") * 2 == pytest.approx(
            0.08, abs=2.5e-3
        )

    def test_follow_joint_trajectory_goal(
        self, singleton_node: rclpy.node.Node
    ) -> None:
        follow_joint_trajectory_goal(
            singleton_node,
            positions=[0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0],
        )
        assert get_joint_position("fr3_joint2") == pytest.approx(-0.5, abs=0.01)
        assert get_joint_position("fr3_joint3") == pytest.approx(0.5, abs=0.01)
