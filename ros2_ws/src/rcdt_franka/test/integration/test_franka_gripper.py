# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros import actions
from rcdt_utilities.launch_utils import (
    assert_for_message,
)
from rcdt_utilities.test_utils import get_joint_position
from rclpy.node import Node
from sensor_msgs.msg import JointState
from utils import call_trigger_service


@launch_pytest.fixture(scope="module")
def franka_and_gripper_launch(
    core_launch: IncludeLaunchDescription,
    controllers_launch: IncludeLaunchDescription,
    open_gripper: actions.Node,
    close_gripper: actions.Node,
) -> LaunchDescription:
    return LaunchDescription(
        [
            core_launch,
            controllers_launch,
            open_gripper,
            close_gripper,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=franka_and_gripper_launch)
def test_joint_states_published() -> None:
    assert_for_message(JointState, "franka/joint_states", 60)


@pytest.mark.launch(fixture=franka_and_gripper_launch)
@pytest.mark.parametrize(
    "service, expected_value",
    [
        ("/close_gripper", 0.00),
        ("/open_gripper", 0.08),
    ],
)
def test_gripper_action(
    service: str, expected_value: float, singleton_node: Node
) -> None:
    """Test gripper open/close action and verify joint state."""
    assert call_trigger_service(singleton_node, service) is True
    joint_value = get_joint_position(name_space="franka", joint="fr3_finger_joint1") * 2
    assert joint_value == pytest.approx(expected_value, abs=0.005), (
        f"The joint value is {joint_value}"
    )
