# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


from typing import Iterator, Type

import pytest
from typing import Type
import rclpy
from launch_testing_ros.wait_for_topics import WaitForTopics
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.node import Node
from rclpy.service import Service
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import time

from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String


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


@pytest.fixture(scope="session")
def singleton_node() -> Iterator[Node]:
    """Fixture to create a singleton node for testing."""
    rclpy.init()
    node = Node("test_node")
    yield node
    node.destroy_node()
    rclpy.shutdown()


def assert_joy_topic_switch(
    node: Node,
    expected_topic: str,
    button_config: list[int],
    timeout_sec: float = 3.0,
    state_topic: str = "/joy_topic_manager/state",
) -> None:
    """
    Publishes a Joy message and asserts that the expected topic is published on state_topic.

    Args:
        node (Node): rclpy test node.
        expected_topic (str): Expected topic that should be published by the JoyTopicManager.
        button_config (list[int]): Joy message buttons to trigger the topic change.
        timeout_sec (float): Max time to wait for the result.
        state_topic (str): Topic to listen for state updates from joy_topic_manager.
    """
    qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

    result = {}

    def callback(msg: String) -> None:
        result["state"] = msg.data

    node.create_subscription(
        String,
        state_topic,
        callback,
        qos_profile=qos,
    )

    pub = node.create_publisher(Joy, "/joy", 10)
    msg = Joy()
    msg.buttons = button_config
    pub.publish(msg)

    start_time = time.time()
    while "state" not in result and time.time() - start_time < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    assert result.get("state") == expected_topic, (
        f"Expected state '{expected_topic}', but got '{result.get('state')}'"
    )
