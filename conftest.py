# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

"""
Global pytest fixtures for ROS 2 integration testing.
"""

import pytest
from typing import Iterator
import rclpy
from rclpy.node import Node


@pytest.fixture(scope="module")
def test_node() -> Iterator[Node]:
    """Fixture to create a singleton node for testing."""
    rclpy.init()
    node = Node("test_node")
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture(scope="module")
def timeout(pytestconfig: pytest.Config) -> int:
    """Fixture to get the timeout value from pytest config."""
    return int(pytestconfig.getini("timeout"))


@pytest.fixture(scope="session")
def finger_joint_fault_tolerance() -> float:
    """Tolerance of testing finger joint movements."""
    return 0.025


@pytest.fixture(scope="session")
def joint_movement_tolerance() -> float:
    """Tolerance of testing joint movements."""
    return 0.01
