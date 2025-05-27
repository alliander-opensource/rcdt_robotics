# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_franka.test.end_to_end.base_franka import get_tests
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import add_tests_to_class


@launch_pytest.fixture(scope="class")
def mobile_manipulator(
    mobile_manipulator_launch: RegisteredLaunchDescription,
) -> LaunchDescription:
    """Fixture to launch the mobile manipulator.

    Args:
        mobile_manipulator_launch (RegisteredLaunchDescription): The launch description for the mobile manipulator.

    Returns:
        LaunchDescription: The launch description for the mobile manipulator.
    """
    return Register.connect_context(
        [
            mobile_manipulator_launch,
        ]
    )


@pytest.mark.launch(fixture=mobile_manipulator)
class TestFrankaMMLaunch:
    """Run all the FrankaLaunchTests under mobile_manipulator.launch.py."""


add_tests_to_class(TestFrankaMMLaunch, get_tests())
