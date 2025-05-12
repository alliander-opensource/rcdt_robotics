# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import launch_pytest
import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_utilities.launch_utils import assert_for_message, get_file_path
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def core() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                get_file_path("rcdt_mobile_manipulator", ["launch"], "core.launch.py")
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=core)
def test_joint_states_franka() -> None:
    assert_for_message(JointState, "/franka/joint_states", 60)


@pytest.mark.launch(fixture=core)
def test_joint_states_panther() -> None:
    assert_for_message(JointState, "/panther/joint_states", 60)
