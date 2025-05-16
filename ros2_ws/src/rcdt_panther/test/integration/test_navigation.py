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
from rcdt_utilities.test_utils import call_trigger_service, get_joint_position
from rclpy.node import Node
from sensor_msgs.msg import JointState
from utils import EndToEndUtils
from typing import Iterator

import pytest
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros import actions
from rcdt_utilities.launch_utils import get_file_path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource

@launch_pytest.fixture(scope="module")
def navigation(navigation_launch) -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_file_path("rcdt_panther", ["launch"], "panther.launch.py")
                ),
                launch_arguments={
                    "nav2": "True"
                }.items(),
            ),
            navigation_launch,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=navigation)
def test_joint_states_published() -> None:
    time.sleep(20)
    assert_for_message(JointState, "/panther/joint_states", 60)


def test_ready_to_start(singleton_node: Node) -> None:
    """This test will ensure the tests are ready to start by waiting for the move_group and moveit_manager node.
    Also waits until the gripper_action_controller is active."""

    assert (
        EndToEndUtils().wait_until_active(
            node=singleton_node,
            controller_name="drive_controller",
            controller_manager_name="/panther/controller_manager",
        )
        is True
    )
@pytest.mark.launch(fixture=navigation)
def test_e_stop_request(singleton_node) -> None:
    assert (
        call_trigger_service(
            node=singleton_node, service_name="/panther/hardware/e_stop_reset"
        )
        is True
    )