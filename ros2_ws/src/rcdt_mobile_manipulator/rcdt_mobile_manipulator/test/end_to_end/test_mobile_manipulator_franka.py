# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_franka.test.end_to_end.base_franka import FrankaTestSuite


@launch_pytest.fixture(scope="class")
def mobile_manipulator(
    mobile_manipulator_launch: IncludeLaunchDescription,
) -> LaunchDescription:
    return LaunchDescription(
        [
            mobile_manipulator_launch,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=mobile_manipulator)
class TestCoreLaunch(FrankaTestSuite()):
    """Run all the FrankaLaunchTests under franka.launch.py"""
