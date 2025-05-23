# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import pytest
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import RegisteredLaunchDescription


@pytest.fixture(scope="module")
def controllers_launch(request: pytest.FixtureRequest) -> RegisteredLaunchDescription:
    """Fixture to create launch file for controllers."""
    return RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "controllers.launch.py"),
        launch_arguments={"simulation": request.config.getoption("simulation")},
    )


@pytest.fixture(scope="module")
def core_launch(request: pytest.FixtureRequest) -> RegisteredLaunchDescription:
    """Fixture to create launch file for the franka core."""
    return RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "core.launch.py"),
        launch_arguments={"simulation": request.config.getoption("simulation")},
    )


@pytest.fixture(scope="module")
def moveit_launch() -> RegisteredLaunchDescription:
    """Fixture to launch moveit."""
    return RegisteredLaunchDescription(
        get_file_path("rcdt_moveit", ["launch"], "moveit.launch.py"),
        launch_arguments={
            "robot_name": "fr3",
            "moveit_package_name": "rcdt_franka_moveit_config",
            "servo_params_package": "rcdt_franka",
            "namespace": "franka",
        },
    )


@pytest.fixture(scope="module")
def gripper_services_launch(
    request: pytest.FixtureRequest,
) -> RegisteredLaunchDescription:
    """Fixture to create launch file for controllers."""
    return RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "gripper_services.launch.py"),
        launch_arguments={"simulation": request.config.getoption("simulation")},
    )
