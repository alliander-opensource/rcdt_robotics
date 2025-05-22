# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import time
from typing import Type

import pytest
import rclpy
from launch_testing_ros.wait_for_topics import WaitForTopics
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.service import Service
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from termcolor import colored

from rcdt_utilities.register import Register


def get_joint_position(namespace: str, joint: str) -> float:
    """Get the joint position of a joint by name. This is done by calling the
    /joint_states topic and parsing the output. This is a workaround for the fact
    that the joint states are not published in a format that can be easily parsed.
    Also --field does not work with the /joint_states topic.

    Args:
        namespace (str); The name space of the robot.
        joint (str): The name of the joint.
    Returns:
        float: The position of the joint.
    """
    topic_list = [(f"{namespace}/joint_states", JointState)]
    wait_for_topics = WaitForTopics(topic_list, timeout=10.0)
    assert wait_for_topics.wait()
    msg: JointState = wait_for_topics.received_messages(f"{namespace}/joint_states")[0]
    idx = msg.name.index(joint)
    position = msg.position[idx]
    wait_for_topics.shutdown()
    return position


def create_ready_service_client(
    node: Node, srv_type: Service, service_name: str, timeout_sec: float = 20.0
) -> Client:
    """
    Create and wait for a service client to become available.

    Args:
        node (Node): The rclpy node to use for client creation.
        srv_type: The service type (e.g., `ListControllers`).
        service_name (str): Fully qualified name of the service.
        timeout_sec (float): Timeout to wait for the service.

    Returns:
        Client: Ready service client.

    Raises:
        RuntimeError: If the service is not available within timeout.
    """
    client = node.create_client(srv_type, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"Service {service_name} not available")
    return client


def call_trigger_service(node: Node, service_name: str) -> bool:
    """Call a trigger service and return True if the service was called successfully."""
    client = create_ready_service_client(node, Trigger, service_name)

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future=future)
    return future.result() is not None


def create_ready_action_client(
    node: Node, action_type: Type, action_name: str, timeout_sec: float = 20.0
) -> ActionClient:
    """
    Create and wait for an ActionClient to become ready.

    Args:
        node (Node): The rclpy node used to create the action client.
        action_type (Type): The action type class
        action_name (str): The fully qualified action name.
        timeout_sec (float): Timeout in seconds to wait for the server.

    Returns:
        ActionClient: A ready ActionClient instance.

    Raises:
        RuntimeError: If the action server is not available within the timeout.
    """
    client = ActionClient(node, action_type, action_name)
    if not client.wait_for_server(timeout_sec=timeout_sec):
        raise RuntimeError(f"Action server {action_name} not available")
    return client


def wait_for_register(pytestconfig: pytest.Config) -> None:
    """
    Waits till all registerd actions are started.

    This function should be called in every first test of a test file.
    This ensures that all other tests are started after all actions are launched correctly.

    If not all actions start correctly, pytest-timeout will cancel the test, but this does not stop the while loop.
    Therefore, the while loop has it's own timeout which also uses the defined pytest-timeout variable.
    """
    logger = get_logger("wait_for_register")
    timeout = int(pytestconfig.getini("timeout"))
    start = time.time()
    while not Register.all_started and time.time() - start < timeout:
        time.sleep(1)
    if Register.all_started:
        logger.info(colored("Register is ready, start testing!", "green"))
    assert Register.all_started
