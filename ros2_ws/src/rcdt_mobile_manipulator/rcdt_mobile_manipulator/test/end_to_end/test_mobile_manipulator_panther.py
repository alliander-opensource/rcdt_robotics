# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_panther.test.end_to_end.base_panther import get_tests
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import add_tests_to_class


@launch_pytest.fixture(scope="class")
def mobile_manipulator(
    mobile_manipulator_launch: RegisteredLaunchDescription,
) -> LaunchDescription:
    return Register.connect_context(
        [
            mobile_manipulator_launch,
        ]
    )


@pytest.mark.launch(fixture=mobile_manipulator)
class TestPantherMMLaunch:
    """Run all the PantherFullTests under mobile_manipulator.launch.py"""


add_tests_to_class(TestPantherMMLaunch, get_tests())
