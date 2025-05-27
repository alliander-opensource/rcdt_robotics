# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_franka.test.utils import call_move_to_configuration_service
from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import get_joint_position, wait_for_register
from rclpy.node import Node
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def franka_and_moveit_launch(
    core_launch: RegisteredLaunchDescription,
    controllers_launch: RegisteredLaunchDescription,
    moveit_launch: RegisteredLaunchDescription,
) -> LaunchDescription:
    """Fixture to create launch file for the franka core, controllers, and MoveIt.

    Args:
        core_launch (RegisteredLaunchDescription): The launch description for the core.
        controllers_launch (RegisteredLaunchDescription): The launch description for the controllers.
        moveit_launch (RegisteredLaunchDescription): The launch description for MoveIt.

    Returns:
        LaunchDescription: The launch description for the franka core, controllers, and MoveIt.
    """
    return Register.connect_context([core_launch, controllers_launch, moveit_launch])


@pytest.mark.launch(fixture=franka_and_moveit_launch)
def test_wait_for_register(timeout: int) -> None:
    """Test that the robot is registered in the system to start the tests.

    Args:
        timeout (int): The timeout in seconds before stopping the test.
    """
    wait_for_register(timeout=timeout)


@pytest.mark.launch(fixture=franka_and_moveit_launch)
def test_joint_states_published(timeout: int) -> None:
    """Test that joint states are published.

    Args:
        timeout (int): The timeout in seconds before stopping the test.
    """
    assert_for_message(JointState, "franka/joint_states", timeout=timeout)


@pytest.mark.launch(fixture=franka_and_moveit_launch)
def test_move_to_drop_configuration(
    test_node: Node, joint_movement_tolerance: float, timeout: int
) -> None:
    """Test that MoveIt can move to a configuration.

    Args:
        test_node (Node): The test node to use for the test.
        joint_movement_tolerance (float): The tolerance for joint movement.
        timeout (int): The timeout in seconds before stopping the test.
    """
    assert (
        call_move_to_configuration_service(test_node, "drop", timeout=timeout) is True
    )
    drop_values = [-1.57079632679, -0.65, 0, -2.4, 0, 1.75, 0.78539816339]
    for i in range(7):
        joint_value = get_joint_position(
            namespace="franka", joint=f"fr3_joint{i + 1}", timeout=timeout
        )
        assert joint_value == pytest.approx(
            drop_values[i], abs=joint_movement_tolerance
        ), f"The joint value is {joint_value}"
