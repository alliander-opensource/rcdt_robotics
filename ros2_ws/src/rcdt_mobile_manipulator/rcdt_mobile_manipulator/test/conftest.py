# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import pytest
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import RegisteredLaunchDescription


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
