# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from base_franka_core import FrankaCoreTests
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_utilities.launch_utils import get_file_path


@launch_pytest.fixture(scope="module")
def core_launch() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                get_file_path("rcdt_franka", ["launch"], "core.launch.py")
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=core_launch)
class TestCoreLaunch(FrankaCoreTests):
    """Run all the FrankaLaunchTests under core.launch.py"""
