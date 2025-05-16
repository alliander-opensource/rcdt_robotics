# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import Iterator

import pytest
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
def core_launch() -> IncludeLaunchDescription:
    """Fixture to create launch file for the panther core."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_file_path("rcdt_panther", ["launch"], "core.launch.py")
        )
    )


@pytest.fixture(scope="module")
def controllers_launch() -> IncludeLaunchDescription:
    """Fixture to create launch file for the panther controllers."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_file_path("rcdt_panther", ["launch"], "controllers.launch.py")
        )
    )
    

@pytest.fixture(scope="module")
def navigation_launch() -> IncludeLaunchDescription:
    """Fixture to create launch file for the panther navigation."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_file_path("rcdt_panther", ["launch"], "navigation.launch.py")
        )
    )