# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_franka.test.utils import follow_joint_trajectory_goal
from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import get_joint_position, wait_for_register
from rclpy.node import Node
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def franka_core_launch(
    core_launch: RegisteredLaunchDescription,
    controllers_launch: RegisteredLaunchDescription,
) -> LaunchDescription:
    """Fixture to create launch file for the franka core and controllers.

    Args:
        core_launch (RegisteredLaunchDescription): The launch description for the core.
        controllers_launch (RegisteredLaunchDescription): The launch description for the controllers.

    Returns:
        LaunchDescription: The launch description for the franka core and controllers.
    """
    return Register.connect_context([core_launch, controllers_launch])


@pytest.mark.launch(fixture=franka_core_launch)
def test_wait_for_register(timeout: int) -> None:
    """Test that the robot is registered in the system to start the tests.

    Args:
        timeout (int): The timeout in seconds to wait for the robot to register.
    """
    wait_for_register(timeout=timeout)


@pytest.mark.launch(fixture=franka_core_launch)
def test_joint_states_published(timeout: int) -> None:
    """Test that joint states are published.

    Args:
        timeout (int): The timeout in seconds to wait for the joint states to be published.
    """
    assert_for_message(JointState, "franka/joint_states", timeout=timeout)


@pytest.mark.launch(fixture=franka_core_launch)
def test_follow_joint_trajectory_goal(
    test_node: Node, joint_movement_tolerance: float, timeout: int
) -> None:
    """Test following a joint trajectory goal.

    Args:
        test_node (Node): The test node to use for the test.
        joint_movement_tolerance (float): The tolerance for joint movement.
        timeout (int): The timeout in seconds to wait for the joint trajectory goal to be followed.
    """
    positions = [0.15, -0.39, 0.1, -2.06, 0.0, 1.68, 1.01]

    follow_joint_trajectory_goal(
        test_node,
        positions=positions,
        controller="franka/fr3_arm_controller",
        timeout=timeout,
    )
    for i in range(7):
        joint_value = get_joint_position(
            namespace="franka", joint=f"fr3_joint{i + 1}", timeout=timeout
        )
        assert joint_value == pytest.approx(
            positions[i], abs=joint_movement_tolerance
        ), f"The joint value is {joint_value}"
