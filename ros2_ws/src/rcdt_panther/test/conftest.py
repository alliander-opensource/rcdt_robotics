# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import pytest
from launch.actions import IncludeLaunchDescription
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.test_utils import *  # noqa


@pytest.fixture(scope="module")
def core_launch() -> IncludeLaunchDescription:
    """Fixture to create launch file for the panther core."""
    return IncludeLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "core.launch.py")
    )


@pytest.fixture(scope="module")
def controllers_launch() -> IncludeLaunchDescription:
    """Fixture to create launch file for the panther controllers."""
    return IncludeLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "controllers.launch.py")
    )


@pytest.fixture(scope="module")
def navigation_launch() -> IncludeLaunchDescription:
    """Fixture to create launch file for the panther navigation."""
    return IncludeLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "navigation.launch.py")
    )
