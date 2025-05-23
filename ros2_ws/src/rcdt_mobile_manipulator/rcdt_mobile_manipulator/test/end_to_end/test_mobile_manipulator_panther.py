# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_panther.test.end_to_end.base_panther import PantherTestSuite
from rcdt_utilities.register import Register, RegisteredLaunchDescription


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
class TestPantherMMLaunch(PantherTestSuite()):
    """Run all the PantherFullTests under mobile_manipulator.launch.py"""
