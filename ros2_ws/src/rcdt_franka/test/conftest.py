# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import Iterator

import pytest
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros import actions
from rcdt_utilities.launch_utils import get_file_path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


@pytest.fixture(scope="module")
def singleton_node() -> Iterator[Node]:
    """Fixture to create a singleton node for testing."""
    rclpy.init()
    node = Node("test_node_singleton")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    yield node
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture(scope="module")
def controllers_launch() -> IncludeLaunchDescription:
    """Fixture to create launch file for controllers."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_file_path("rcdt_franka", ["launch"], "controllers.launch.py")
        )
    )


@pytest.fixture(scope="module")
def core_launch() -> IncludeLaunchDescription:
    """Fixture to create launch file for the franka core."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_file_path("rcdt_franka", ["launch"], "core.launch.py")
        )
    )


@pytest.fixture(scope="module")
def moveit_launch() -> IncludeLaunchDescription:
    """Fixture to launch moveit."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_file_path("rcdt_moveit", ["launch"], "moveit.launch.py")
        ),
        launch_arguments={
            "robot_name": "fr3",
            "moveit_package_name": "rcdt_franka_moveit_config",
            "servo_params_package": "rcdt_franka",
            "namespace": "franka",
        }.items(),
    )


@pytest.fixture(scope="module")
def open_gripper() -> actions.Node:
    """Fixture for launching the open_gripper node."""
    return actions.Node(
        package="rcdt_franka",
        executable="open_gripper.py",
    )


@pytest.fixture(scope="module")
def close_gripper() -> actions.Node:
    """Fixture for launching the close_gripper node."""
    return actions.Node(
        package="rcdt_franka",
        executable="close_gripper.py",
    )
