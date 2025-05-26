# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_franka.test.end_to_end.base_franka import FrankaTestSuite
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription


@launch_pytest.fixture(scope="class")
def franka() -> LaunchDescription:
    """Fixture to create a launch description for the Franka robot.

    This fixture sets up the Franka robot with the necessary configurations and
    launches the required nodes for testing.

    Returns:
        LaunchDescription: The launch description containing the Franka robot setup.

    """
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
class TestCoreLaunch(FrankaTestSuite()):
    """Test suite for the Franka robot using the core launch file.

    This class inherits from FrankaTestSuite and runs all tests defined in the base class.
    It uses the `franka` fixture to set up the launch description for the Franka robot.
    """
