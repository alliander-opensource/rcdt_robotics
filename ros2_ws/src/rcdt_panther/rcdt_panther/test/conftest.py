# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import pytest
from _pytest.fixtures import SubRequest
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import RegisteredLaunchDescription


@pytest.fixture(scope="module")
def core_launch(request: SubRequest) -> RegisteredLaunchDescription:
    """Fixture to create launch file for the panther core.

    Args:
        request (SubRequest): The pytest request object.

    Returns:
        RegisteredLaunchDescription: The launch description for the panther core.
    """
    return RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "core.launch.py"),
        launch_arguments={"simulation": request.config.getoption("simulation")},
    )


@pytest.fixture(scope="module")
def controllers_launch(request: SubRequest) -> RegisteredLaunchDescription:
    """Fixture to create launch file for the panther controllers.

    Args:
        request (SubRequest): The pytest request object.

    Returns:
        RegisteredLaunchDescription: The launch description for the panther controllers.
    """
    return RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "controllers.launch.py"),
        launch_arguments={"simulation": request.config.getoption("simulation")},
    )
