# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from end_to_end.base_panther import PantherFullTests
from launch import LaunchDescription
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription


@launch_pytest.fixture(scope="module")
def panther_launch() -> LaunchDescription:
    panther = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "panther.launch.py"),
        launch_arguments={
            "rviz": "False",
        },
    )
    utils_launch = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "utils.launch.py"),
    )
    return Register.connect_context([panther, utils_launch])


@pytest.mark.launch(fixture=panther_launch)
class TestEndToEndLaunch(PantherFullTests):
    """Run all the PantherFullTests under panther.launch.py"""
