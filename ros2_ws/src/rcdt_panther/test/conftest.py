# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import Iterator

import pytest
import rclpy
from rclpy.node import Node


@pytest.fixture(scope="module")
def singleton_node() -> Iterator[rclpy.node.Node]:
    """Fixture to create a singleton node for testing."""
    rclpy.init()
    node = Node("test_node_singleton")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    yield node
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()