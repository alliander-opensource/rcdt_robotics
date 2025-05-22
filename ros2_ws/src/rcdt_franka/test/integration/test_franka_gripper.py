# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import (
    call_trigger_service,
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
    return Register.connect_context(
        [core_launch, controllers_launch, gripper_services_launch]
    )


@pytest.mark.launch(fixture=franka_and_gripper_launch)
def test_wait_for_register(pytestconfig: pytest.Config) -> None:
    wait_for_register(pytestconfig)


@pytest.mark.launch(fixture=franka_and_gripper_launch)
def test_joint_states_published() -> None:
    assert_for_message(JointState, "franka/joint_states", 60)


@pytest.mark.launch(fixture=franka_and_gripper_launch)
@pytest.mark.parametrize(
    "service, expected_value",
    [
        ("/franka/close_gripper", 0.00),
        ("/franka/open_gripper", 0.04),
    ],
)
def test_gripper_action(
    service: str,
    expected_value: float,
    test_node: Node,
    finger_joint_fault_tolerance: float,
) -> None:
    """Test gripper open/close action and verify joint state."""
    assert call_trigger_service(test_node, service) is True
    joint_value = get_joint_position(namespace="franka", joint="fr3_finger_joint1")
    assert joint_value == pytest.approx(
        expected_value, abs=finger_joint_fault_tolerance
    ), f"The joint value is {joint_value}"
