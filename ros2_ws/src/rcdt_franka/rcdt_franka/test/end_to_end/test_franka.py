# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_franka.test.end_to_end.base_franka import get_tests
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import add_tests_to_class


@launch_pytest.fixture(scope="class")
def franka() -> LaunchDescription:
    franka_launch = RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "franka.launch.py"),
        launch_arguments={
            "rviz": "False",
            "world": "empty_camera.sdf",
            "realsense": "False",
        },
    )
    return Register.connect_context([franka_launch])


@pytest.mark.launch(fixture=franka)
class TestCoreLaunch:
    """Run all the FrankaLaunchTests under franka.launch.py"""


add_tests_to_class(TestCoreLaunch, get_tests())
