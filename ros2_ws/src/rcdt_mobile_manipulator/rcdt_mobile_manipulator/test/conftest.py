# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import pytest
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import RegisteredLaunchDescription
from rcdt_utilities.test_utils import *  # noqa


@pytest.fixture(scope="module")
def mobile_manipulator_launch() -> RegisteredLaunchDescription:
    """Fixture to create launch file for controllers."""
    return RegisteredLaunchDescription(
        get_file_path(
            "rcdt_mobile_manipulator", ["launch"], "mobile_manipulator.launch.py"
        ),
        launch_arguments={
            "rviz": "False",
        },
    )


@pytest.fixture(scope="session")
def finger_joint_fault_tolerance() -> float:
    """Tolerance of testing finger joint movements."""
    return 0.025


@pytest.fixture(scope="session")
def joint_movement_tolerance() -> float:
    """Tolerance of testing joint movements."""
    return 0.01
