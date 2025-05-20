# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_franka.test.base_franka import FrankaFullTests
from rcdt_utilities.launch_utils import (
    get_file_path,
)


@launch_pytest.fixture(scope="class")
def franka_launch() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                get_file_path("rcdt_franka", ["launch"], "franka.launch.py"),
                launch_arguments={
                    "rviz": "False",
                    "world": "empty_camera.sdf",
                    "realsense": "False",
                }.items(),
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=franka_launch)
class TestCoreLaunch(FrankaFullTests):
    """Run all the FrankaLaunchTests under franka.launch.py"""
