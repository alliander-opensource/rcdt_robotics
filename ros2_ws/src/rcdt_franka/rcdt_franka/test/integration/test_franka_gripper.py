# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import (
    call_trigger_action,
    get_joint_position,
    wait_for_register,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def franka_and_gripper_launch(
    core_launch: RegisteredLaunchDescription,
    controllers_launch: RegisteredLaunchDescription,
    gripper_services_launch: RegisteredLaunchDescription,
) -> LaunchDescription:
    """Fixture to create launch file for the franka core, controllers, and gripper services.

    Args:
        core_launch (RegisteredLaunchDescription): The launch description for the core.
        controllers_launch (RegisteredLaunchDescription): The launch description for the controllers.
        gripper_services_launch (RegisteredLaunchDescription): The launch description for the gripper services.

    Returns:
        LaunchDescription: The launch description for the franka core, controllers, and gripper services.
    """
    return Register.connect_context(
        [core_launch, controllers_launch, gripper_services_launch]
    )


@pytest.mark.launch(fixture=franka_and_gripper_launch)
def test_wait_for_register(timeout: int) -> None:
    """Test that the robot is registered in the system to start the tests.

    Args:
        timeout (int): The timeout in seconds before stopping the test.
    """
    wait_for_register(timeout=timeout)


@pytest.mark.launch(fixture=franka_and_gripper_launch)
def test_joint_states_published(timeout: int) -> None:
    """Test that joint states are published.

    Args:
        timeout (int): The timeout in seconds before stopping the test.
    """
    assert_for_message(JointState, "franka/joint_states", timeout=timeout)


@pytest.mark.launch(fixture=franka_and_gripper_launch)
@pytest.mark.parametrize(
    "action, expected_value",
    [
        ("/franka/gripper/close", 0.00),
        ("/franka/gripper/open", 0.04),
    ],
)
def test_gripper_action(
    action: str,
    expected_value: float,
    test_node: Node,
    finger_joint_fault_tolerance: float,
    timeout: int,
) -> None:
    """Test gripper open/close action and verify joint state.

    Args:
        action (str): The action to call.
        expected_value (float): The expected joint value after the action.
        test_node (Node): The test node to use for the test.
        finger_joint_fault_tolerance (float): The tolerance for the finger joint position.
        timeout (int): The timeout in seconds before stopping the test.
    """
    assert call_trigger_action(test_node, action, timeout=timeout) is True
    joint_value = get_joint_position(
        namespace="franka", joint="fr3_finger_joint1", timeout=timeout
    )
    assert joint_value == pytest.approx(
        expected_value, abs=finger_joint_fault_tolerance
    ), f"The joint value is {joint_value}"
