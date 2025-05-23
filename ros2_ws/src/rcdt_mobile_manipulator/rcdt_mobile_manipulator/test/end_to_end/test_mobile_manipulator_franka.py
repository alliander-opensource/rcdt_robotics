# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_franka.test.end_to_end.base_franka import FrankaTestSuite
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
class TestFrankaMMLaunch(FrankaTestSuite()):
    """Run all the FrankaLaunchTests under mobile_manipulator.launch.py"""
