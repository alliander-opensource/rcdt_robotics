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
from rcdt_utilities.launch_utils import assert_for_message, get_file_path
from rcdt_utilities.test_utils import call_trigger_service, get_joint_position
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def panther() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                get_file_path("rcdt_panther", ["launch"], "panther.launch.py"),
                launch_arguments={
                    "rviz": "False",
                }.items(),
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=panther)
def test_joint_states_published() -> None:
    assert_for_message(JointState, "/panther/joint_states", 60)


@pytest.mark.launch(fixture=panther)
def test_e_stop_request(singleton_node) -> None:
    assert (
        call_trigger_service(
            node=singleton_node, service_name="/panther/hardware/e_stop_reset"
        )
        is True
    )


@pytest.mark.launch(fixture=panther)
def test_driving(singleton_node) -> None:
    pub = singleton_node.create_publisher(Twist, "/panther/cmd_vel", 10)

    msg = Twist()
    msg.linear.x = 1.0

    pub.publish(msg)
    rclpy.spin_once(singleton_node, timeout_sec=0.1)

    # Wait for the gripper to move
    time.sleep(0.5)
    # Check the gripper position
    joint_value = get_joint_position("fl_wheel_joint")
    assert joint_value != pytest.approx(0, abs=0.5), f"The joint value is {joint_value}"
