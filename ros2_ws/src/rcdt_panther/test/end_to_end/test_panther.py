# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from end_to_end.base_panther import PantherFullTests
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_utilities.launch_utils import get_file_path


@launch_pytest.fixture(scope="module")
def panther() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                get_file_path("rcdt_panther", ["launch"], "panther.launch.py"),
                launch_arguments={
                    "rviz": "False",
                }.items(),
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=panther)
class TestEndToEndLaunch(PantherFullTests):
    """Run all the PantherFullTests under panther.launch.py"""
