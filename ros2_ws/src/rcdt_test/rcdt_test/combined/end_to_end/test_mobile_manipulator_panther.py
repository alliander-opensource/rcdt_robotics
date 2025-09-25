# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_launch.robot import Arm, Vehicle
from rcdt_test.panther.end_to_end.base_panther import get_tests
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import add_tests_to_class

namespace_vehicle = "panther"
namespace_arm = "franka"


@launch_pytest.fixture(scope="class")
def mobile_manipulator() -> LaunchDescription:
    """Fixture to launch the mobile manipulator.

    Returns:
        LaunchDescription: The launch description for the mobile manipulator.

    Returns:
        LaunchDescription: The launch description for the mobile manipulator.
    """
    vehicle = Vehicle(
        platform="panther",
        position=[0, 0, 0.2],
        namespace=namespace_vehicle,
    )
    Arm(
        platform="franka",
        position=[0, 0, 0],
        namespace=namespace_arm,
        parent=vehicle,
        moveit=True,
    )

    launch = RegisteredLaunchDescription(
        get_file_path("rcdt_launch", ["launch"], "robots.launch.py"),
        launch_arguments={"rviz": "False"},
    )
    return Register.connect_context([launch])


@pytest.mark.launch(fixture=mobile_manipulator)
class TestPantherMMLaunch:
    """Run all the PantherFullTests under mobile_manipulator.launch.py."""


add_tests_to_class(TestPantherMMLaunch, get_tests(namespace_vehicle))
