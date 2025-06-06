# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import pytest
from _pytest.fixtures import SubRequest
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import RegisteredLaunchDescription


@pytest.fixture(scope="module")
def controllers_launch() -> RegisteredLaunchDescription:
    """Fixture to create launch file for controllers.

    Returns:
        RegisteredLaunchDescription: The launch description for the controllers.
    """
    return RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "controllers.launch.py")
    )


@pytest.fixture(scope="module")
def core_launch(request: SubRequest) -> RegisteredLaunchDescription:
    """Fixture to create launch file for the franka core.

    Args:
        request (SubRequest): The pytest request object, used to access the module name.

    Returns:
        RegisteredLaunchDescription: The launch description for the franka core.
    """
    use_realsense = request.module.__name__ == "test_franka_realsense"

    return RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "core.launch.py"),
        launch_arguments={"realsense": str(use_realsense)},
    )


@pytest.fixture(scope="module")
def realsense_launch() -> RegisteredLaunchDescription:
    """Fixture to launch realsense.

    Returns:
        RegisteredLaunchDescription: The launch description for realsense.
    """
    return RegisteredLaunchDescription(
        get_file_path("rcdt_detection", ["launch"], "realsense.launch.py")
    )


@pytest.fixture(scope="module")
def moveit_launch() -> RegisteredLaunchDescription:
    """Fixture to launch moveit.

    Returns:
        RegisteredLaunchDescription: The launch description for MoveIt.
    """
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
def gripper_services_launch() -> RegisteredLaunchDescription:
    """Fixture to create launch file for controllers.

    Returns:
        RegisteredLaunchDescription: The launch description for the gripper services.
    """
    return RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "gripper_services.launch.py")
    )
