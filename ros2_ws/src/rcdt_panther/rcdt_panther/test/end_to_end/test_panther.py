# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from _pytest.fixtures import SubRequest
from launch import LaunchDescription
from rcdt_panther.test.end_to_end.base_panther import get_tests
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import add_tests_to_class


@launch_pytest.fixture(scope="module")
def panther_launch(request: SubRequest) -> LaunchDescription:
    """Fixture to create launch file for panther robot.

    Args:
        request (SubRequest): The pytest request object, used to access command line options

    Returns:
        LaunchDescription: The launch description for the panther robot.
    """
    panther = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "panther.launch.py"),
        launch_arguments={
            "rviz": "False",
            "simulation": request.config.getoption("simulation"),
        },
    )
    utils_launch = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "utils.launch.py"),
    )
    return Register.connect_context([panther, utils_launch])


@pytest.mark.launch(fixture=panther_launch)
class TestPantherFullSuite:
    """Re-run all tests from PantherFullTests using panther.launch.py."""


add_tests_to_class(TestPantherFullSuite, get_tests())
