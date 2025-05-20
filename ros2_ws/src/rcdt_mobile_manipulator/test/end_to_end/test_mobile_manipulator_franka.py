# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_franka.test.base_franka import FrankaFullTests


@launch_pytest.fixture(scope="class")
def mobile_manipulator_launch(
    mobile_manipulator_launch: IncludeLaunchDescription,
) -> LaunchDescription:
    return LaunchDescription(
        [
            mobile_manipulator_launch,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=mobile_manipulator_launch)
class TestCoreLaunch(FrankaFullTests):
    """Run all the FrankaLaunchTests under mobile_manipulator.launch.py"""
