# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_franka.test.utils import call_move_to_configuration_service
from rcdt_utilities.launch_utils import (
    assert_for_message,
    assert_for_node,
)
from rcdt_utilities.register import Register
from rcdt_utilities.test_utils import get_joint_position, wait_for_register
from rclpy.node import Node
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def franka_and_moveit_launch(
    core_launch: IncludeLaunchDescription,
    controllers_launch: IncludeLaunchDescription,
    moveit_launch: IncludeLaunchDescription,
) -> LaunchDescription:
    return Register.connect_context([core_launch, controllers_launch, moveit_launch])


@pytest.mark.launch(fixture=franka_and_moveit_launch)
def test_wait_for_register(timeout: int) -> None:
    wait_for_register(timeout=timeout)


@pytest.mark.launch(fixture=franka_and_moveit_launch)
def test_joint_states_published(timeout: int) -> None:
    assert_for_message(JointState, "franka/joint_states", timeout=timeout)


@pytest.mark.launch(fixture=franka_and_moveit_launch)
def test_moveit_node_activated(test_node: Node) -> None:
    assert_for_node(
        fully_qualified_node_name="franka/move_group",
        test_node=test_node,
        timeout=30,
    )
    assert_for_node(
        fully_qualified_node_name="franka/moveit_manager",
        test_node=test_node,
        timeout=30,
    )


@pytest.mark.launch(fixture=franka_and_moveit_launch)
def test_move_to_drop_configuration(
    test_node: Node, joint_movement_tolerance: float, timeout: int
) -> None:
    """Test that MoveIt can move to a configuration."""

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
