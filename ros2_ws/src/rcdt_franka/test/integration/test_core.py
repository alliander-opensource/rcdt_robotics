# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import subprocess

import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_utilities.launch_utils import assert_for_message, get_file_path
from rcdt_utilities.test_utils import (
    call_move_gripper_service,
    follow_joint_trajectory_goal,
    get_joint_position,
)
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def core_launch() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                get_file_path("rcdt_franka", ["launch"], "core.launch.py")
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=core_launch)
def test_joint_states_published() -> None:
    assert_for_message(JointState, "/joint_states", 60)


@pytest.mark.launch(fixture=core_launch)
def test_controller_is_active() -> None:
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


@pytest.mark.launch(fixture=core_launch)
def test_can_move_fingers(singleton_node: rclpy.node.Node) -> None:
    call_move_gripper_service(
        singleton_node,
        width=0.08,
        speed=0.03,
    )
    result_joint = get_joint_position("fr3_finger_joint1")
    # The gripper is closed when the joint position is 0.00 with a tolerance of 0.0025
    assert result_joint * 2 == pytest.approx(0.08, abs=2.5e-3)


@pytest.mark.launch(fixture=core_launch)
def test_follow_joint_trajectory_goal(singleton_node: rclpy.node.Node) -> None:
    """Test sending a joint trajectory goal to the arm controller."""

    follow_joint_trajectory_goal(
        singleton_node, positions=[0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0]
    )

    fr3_joint2_result = get_joint_position("fr3_joint2")
    fr3_joint3_result = get_joint_position("fr3_joint3")
    assert fr3_joint2_result == pytest.approx(-0.5, abs=0.01)
    assert fr3_joint3_result == pytest.approx(0.5, abs=0.01)
