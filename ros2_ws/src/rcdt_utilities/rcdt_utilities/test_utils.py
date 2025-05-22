# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import time
from typing import Callable, Iterator, Type

import pytest
import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from launch_testing_ros.wait_for_topics import WaitForTopics
from rcdt_messages.srv import ExpressPoseInOtherFrame
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from rclpy.service import Service
from rclpy.task import Future
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from termcolor import colored

from rcdt_utilities.register import Register


@pytest.fixture(scope="module")
def test_node() -> Iterator[Node]:
    """Fixture to create a singleton node for testing."""
    rclpy.init()
    node = Node("test_node")
    yield node
    node.destroy_node()
    rclpy.shutdown()


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
    rclpy.spin_until_future_complete(node, future=future, timeout_sec=90)
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


def assert_joy_topic_switch(
    node: Node,
    expected_topic: str,
    button_config: list[int],
    timeout_sec: float = 90.0,
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

    def callback_function(msg: String) -> None:
        result["state"] = msg.data

    node.create_subscription(
        msg_type=String,
        topic=state_topic,
        callback=callback_function,
        qos_profile=qos,
    )

    pub = node.create_publisher(Joy, "/joy", 10)
    msg = Joy()
    msg.buttons = button_config
    pub.publish(msg)

    start_time = time.time()
    while (
        result.get("state") != expected_topic and time.time() - start_time < timeout_sec
    ):
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    assert result.get("state") == expected_topic, (
        f"Expected state '{expected_topic}', but got '{result.get('state')}'"
    )


def call_express_pose_in_other_frame(
    node: Node, pose: PoseStamped, target_frame: str, timeout_sec: float = 90
) -> ExpressPoseInOtherFrame.Response:
    """
    Calls the /express_pose_in_other_frame service.

    Args:
        node (Node): An active rclpy Node.
        pose (PoseStamped): The pose to transform.
        target_frame (str): The frame to express the pose in.
        timeout_sec (float): Timeout for waiting on service and result.

    Returns:
        ExpressPoseInOtherFrame.Response: The response containing the transformed pose.
    """

    client = create_ready_service_client(
        node, ExpressPoseInOtherFrame, "/express_pose_in_other_frame"
    )

    request = ExpressPoseInOtherFrame.Request()
    request.pose = pose
    request.target_frame = target_frame

    future: Future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    response = future.result()
    if response is None:
        raise RuntimeError("Service call failed or timed out")

    return response


def assert_movements_with_joy(  # noqa: PLR0913
    node: Node,
    joy_axes: list[float],
    compare_fn: Callable[[PoseStamped, PoseStamped], float],
    threshold: float,
    description: str,
    frame_base: str,
    frame_target: str,
) -> None:
    """Publishes a joystick message and asserts that movement occurs above a threshold.

    Args:
        test_node: The ROS node used for publishing/subscribing.
        joy_axes: A list of axes for the Joy message.
        compare_fn: A function taking (first_pose, moved_pose) and returning a float delta.
        threshold: The minimum delta required to pass the test.
        description: Description of what is being tested (used in assertion message).
        frame_base: The frame in which the original pose is defined.
        frame_target: The frame to which the pose should be transformed.
    """
    pose = PoseStamped()
    pose.header.frame_id = frame_base
    first_pose = call_express_pose_in_other_frame(
        node=node, pose=pose, target_frame=frame_target
    ).pose.pose

    pub = node.create_publisher(Joy, "/joy", 10)

    msg = Joy()
    msg.axes = joy_axes
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(1)  # let the robot move

    pose = PoseStamped()
    pose.header.frame_id = frame_base
    moved_pose = call_express_pose_in_other_frame(
        node=node, pose=pose, target_frame=frame_target
    ).pose.pose
    delta = compare_fn(first_pose, moved_pose)

    assert abs(delta) > threshold, (
        f"{description} did not change after input. Δ = {delta:.4f}"
    )


def list_controllers(node: Node, controller_manager_name: str) -> list[ControllerState]:
    """Query the controller manager for all currently loaded controllers.

    Args:
        node (Node): The rclpy node used to create the service client.
        controller_manager_name (str): Name or namespace of the controller manager.

    Returns:
        List[ControllerState]: List of current controller states."""
    client = create_ready_service_client(
        node, ListControllers, f"{controller_manager_name}/list_controllers"
    )
    request = ListControllers.Request()
    future: Future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=90)

    response: ListControllers.Response = future.result()
    if response is None:
        raise RuntimeError("Failed to get response from list_controllers")

    return response.controller


def get_controller_state(
    controllers: list[ControllerState], controller_name: str
) -> str:
    """Retrieve the state of a controller by name.

    Args:
        controllers (List[ControllerState]): List of controllers.
        controller_name (str): Name of the controller to find.

    Returns:
        str: The current state of the controller.

    Raises:
        ValueError: If the controller is not found."""
    for controller in controllers:
        if controller.name == controller_name:
            return controller.state
    raise ValueError(f"Controller '{controller_name}' not found")


def wait_until_active(
    node: Node,
    controller_name: str,
    controller_manager_name: str,
    timeout_sec: float = 90.0,
    poll_interval: float = 0.5,
) -> bool:
    """Poll until a controller becomes 'active'.

    Args:
        node (Node): rclpy node used to call the service.
        controller_name (str): Name of the controller to wait for.
        timeout_sec (float): Timeout duration in seconds.
        poll_interval (float): Interval between polls.
        controller_manager_name (str): Controller manager service name.

    Returns:
        bool: True if the controller became active, False on timeout."""
    end_time = time.time() + timeout_sec
    while time.time() < end_time:
        controllers = list_controllers(node, controller_manager_name)
        try:
            state = get_controller_state(controllers, controller_name)
            if state == "active":
                print("The controller is active!")
                time.sleep(1)
                return True
        except ValueError:
            pass

        time.sleep(poll_interval)

    return False


def wait_until_reached_joint(
    namespace: str,
    joint: str,
    expected_value: float,
    tolerance: float = 0.025,
    timeout_sec: int = 30,
) -> tuple[bool, float]:
    """Wait until a joint reaches the expected value within a tolerance.

    Args:
        namespace (str): Namespace of the robot (e.g., 'franka').
        joint (str): Name of the joint to check.
        expected_value (float): Target joint value in radians.
        tolerance (float): Acceptable deviation from the expected value.
        timeout_sec (int): Timeout duration in seconds.
        poll_interval (float): Interval between joint state checks.

    Returns:
        Tuple[bool, float]: (True, joint_value) if target reached; otherwise (False"""
    end_time = time.time() + timeout_sec
    while time.time() < end_time:
        try:
            joint_value = get_joint_position(namespace=namespace, joint=joint)
            if joint_value == pytest.approx(expected_value, abs=tolerance):
                print("The joint reached its expected value!")
                time.sleep(0.25)
                return (True, joint_value)
        except ValueError:
            pass

        time.sleep(0.25)
    return (False, joint_value)


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
