# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import time

import launch_pytest
import pytest
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import (
    call_trigger_service,
    get_joint_position,
    publish_for_duration,
    wait_for_register,
    wait_for_subscriber,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def panther_core_launch(
    core_launch: RegisteredLaunchDescription,
    controllers_launch: RegisteredLaunchDescription,
) -> LaunchDescription:
    return Register.connect_context([core_launch, controllers_launch])


@pytest.mark.launch(fixture=panther_core_launch)
def test_wait_for_register(timeout: int) -> None:
    wait_for_register(timeout=timeout)


@pytest.mark.launch(fixture=panther_core_launch)
def test_joint_states_published(timeout: int) -> None:
    assert_for_message(JointState, "/panther/joint_states", timeout=timeout)


@pytest.mark.launch(fixture=panther_core_launch)
def test_e_stop_request(test_node: Node, timeout: int) -> None:
    assert (
        call_trigger_service(
            node=test_node,
            service_name="/panther/hardware/e_stop_reset",
            timeout=timeout,
        )
        is True
    )


@pytest.mark.launch(fixture=panther_core_launch)
def test_driving(test_node: Node, timeout: int) -> None:
    """Test that the controllers work and the wheels have turned."""

    joint_value_before_driving = get_joint_position(
        "panther", "fl_wheel_joint", timeout=timeout
    )

    pub = test_node.create_publisher(Twist, "/panther/cmd_vel", 10)
    wait_for_subscriber(pub, timeout)

    msg = Twist()
    msg.linear.x = 1.0

    publish_for_duration(
        node=test_node, publisher=pub, msg=msg, publish_duration=1, rate_sec=0.1
    )

    time.sleep(1)  # give the panther some time to move

    joint_value_after_driving = get_joint_position(
        "panther", "fl_wheel_joint", timeout=timeout
    )

    delta = joint_value_after_driving - joint_value_before_driving

    assert delta != pytest.approx(0, abs=0.5), (
        f"The current joint value is {joint_value_after_driving}, but it should be different from {joint_value_before_driving}."
    )
