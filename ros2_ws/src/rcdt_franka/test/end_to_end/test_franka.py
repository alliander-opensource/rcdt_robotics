# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from end_to_end.base_franka import FrankaFullTests
from launch import LaunchDescription
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription


@launch_pytest.fixture(scope="class")
def franka_launch() -> LaunchDescription:
    franka = RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "franka.launch.py"),
        launch_arguments={
            "rviz": "False",
            "world": "empty_camera.sdf",
            "realsense": "False",
        },
    )
    return Register.connect_context([franka])


@pytest.mark.launch(fixture=franka_launch)
class TestCoreLaunch(FrankaFullTests):
    """Run all the FrankaLaunchTests under franka.launch.py"""
