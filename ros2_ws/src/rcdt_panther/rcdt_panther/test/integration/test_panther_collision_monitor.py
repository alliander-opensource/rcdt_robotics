# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import time

import launch_pytest
import pytest
import rclpy
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from rcdt_utilities.launch_utils import assert_for_message, get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import (
    call_trigger_service,
    wait_for_register,
    wait_for_subscriber,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def panther_launch() -> LaunchDescription:
    """Fixture to create launch file for panther robot.

    Returns:
        LaunchDescription: The launch description for the panther robot.
    """
    panther = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "panther.launch.py"),
        launch_arguments={
            "rviz": "False",
            "collision_monitor": "True",
            "positions": "3.8-0-0",
        },
    )
    return Register.connect_context([panther])


@pytest.mark.launch(fixture=panther_launch)
def test_wait_for_register(timeout: int) -> None:
    """Test that the panther core is registered in the RCDT.

    Args:
        timeout (int): The timeout in seconds to wait for the panther core to register.
    """
    wait_for_register(timeout=timeout)


@pytest.mark.launch(fixture=panther_launch)
def test_joint_states_published(timeout: int) -> None:
    """Test that the joint states are published.

    Args:
        timeout (int): The timeout in seconds to wait for the joint states to be published.
    """
    assert_for_message(JointState, "/panther/joint_states", timeout=timeout)


@pytest.mark.launch(fixture=panther_launch)
def test_e_stop_request(test_node: Node, timeout: int) -> None:
    """Test that the E-Stop request service can be called.

    Args:
        test_node (Node): The ROS 2 node to use for the test.
        timeout (int): The timeout in seconds to wait for the service to be called.
    """
    assert (
        call_trigger_service(
            node=test_node,
            service_name="/panther/hardware/e_stop_reset",
            timeout=timeout,
        )
        is True
    )


@pytest.mark.launch(fixture=panther_launch)
def test_collision_monitoring(test_node: Node, timeout: int) -> None:
    """Test that cmd_vel is reduced to 30% by the collision monitor.

    Args:
        test_node (Node): The ROS 2 node to use for the test.
        timeout (int): The timeout in seconds to wait for the wheels to turn.
    """
    input_velocity = 0.0001
    expected_output = input_velocity * 0.3

    publisher = test_node.create_publisher(Twist, "/panther/cmd_vel_raw", 10)
    result = {}

    def callback_function_cmd_vel(msg: Twist) -> None:
        """Callback function to handle messages from the state topic.

        Args:
            msg (Twist): The message received from the state topic.
        """
        result["output_velocity"] = msg.linear.x

    test_node.create_subscription(
        msg_type=Twist,
        topic="/panther/cmd_vel",
        callback=callback_function_cmd_vel,
        qos_profile=10,
    )

    wait_for_subscriber(publisher, timeout)

    msg = Twist()
    msg.linear.x = input_velocity

    publish_duration = 1  # seconds
    publish_rate_sec = 0.1  # seconds
    deadline = time.monotonic() + publish_duration

    while (
        time.monotonic() < deadline and result.get("output_velocity") != expected_output
    ):
        publisher.publish(msg)
        rclpy.spin_once(test_node, timeout_sec=publish_rate_sec)

    assert result.get("output_velocity") == expected_output, (
        f"Expected output velocity to be ~{expected_output}, got {result.get('output_velocity')}"
    )
