# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import time

import launch_pytest
import pytest
import rclpy
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.test_utils import (
    call_trigger_service,
    get_joint_position,
    wait_until_active,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def core(
    core_launch: IncludeLaunchDescription, controllers_launch: IncludeLaunchDescription
) -> LaunchDescription:
    return LaunchDescription(
        [
            core_launch,
            controllers_launch,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=core)
def test_joint_states_published() -> None:
    assert_for_message(JointState, "/panther/joint_states", 60)


def test_ready_to_start(test_node: Node) -> None:
    """This test will ensure the tests are ready to start by waiting for the move_group and moveit_manager node.
    Also waits until the gripper_action_controller is active."""

    assert (
        wait_until_active(
            node=test_node,
            controller_name="drive_controller",
            controller_manager_name="/panther/controller_manager",
        )
        is True
    )


@pytest.mark.launch(fixture=core)
def test_e_stop_request(test_node: Node) -> None:
    assert (
        call_trigger_service(
            node=test_node, service_name="/panther/hardware/e_stop_reset"
        )
        is True
    )


@pytest.mark.launch(fixture=core)
def test_driving(test_node: Node) -> None:
    """Test that the controllers work and the wheels have turned."""
    pub = test_node.create_publisher(Twist, "/panther/cmd_vel", 10)

    msg = Twist()
    msg.linear.x = 1.0

    pub.publish(msg)
    rclpy.spin_once(test_node, timeout_sec=0.1)
    time.sleep(1)  # give the panther some time to move

    joint_value = get_joint_position("panther", "fl_wheel_joint")
    assert joint_value != pytest.approx(0, abs=0.5), f"The joint value is {joint_value}"
