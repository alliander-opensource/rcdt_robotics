# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import call_trigger_service, wait_for_register
from rclpy.node import Node


@launch_pytest.fixture(scope="module")
def grasping_launch() -> LaunchDescription:
    """Fixture for grasping launch file.

    Returns:
        LaunchDescription: The launch description for the franka core, controllers, and gripper services.
    """
    franka_core_launch = RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "core.launch.py"),
        launch_arguments={
            "realsense": "True",
            "world": "table_with_1_brick.sdf",
        },
    )

    grasping_launch = RegisteredLaunchDescription(
        get_file_path("rcdt_grasping", ["launch"], "grasping.launch.py"),
    )
    return Register.connect_context([franka_core_launch, grasping_launch])


@pytest.mark.launch(fixture=grasping_launch)
def test_wait_for_register(timeout: int) -> None:
    """Test that the robot is registered in the system to start the tests.

    Args:
        timeout (int): The timeout in seconds before stopping the test.
    """
    wait_for_register(timeout=timeout)


@pytest.mark.launch(fixture=grasping_launch)
def test_trigger_request(test_node: Node, timeout: int = 100) -> None:
    """Test that the robot is registered in the system to start the tests.

    Args:
        test_node (Node): The ROS2 node to use for the service call.
        timeout (int): The timeout in seconds before stopping the test.
    """
    assert (
        call_trigger_service(
            node=test_node, service_name="/grasp/generate", timeout=timeout
        )
        is True
    )
