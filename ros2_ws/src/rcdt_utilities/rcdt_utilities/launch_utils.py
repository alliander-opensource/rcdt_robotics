# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import ast
import os
import time
from typing import List

import rclpy
import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import Action, LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.substitutions import LaunchConfiguration
from launch_testing_ros.wait_for_topics import WaitForTopics
from rclpy.executors import Executor
from rclpy.logging import get_logger
from rclpy.node import Node

SKIP = LaunchDescriptionEntity()
WAIT = 3


class LaunchArgument:
    def __init__(
        self,
        name: str,
        default_value: str | bool | int | float,
        choices: List = None,
    ) -> None:
        self.configuration = LaunchConfiguration(name)
        if choices is not None:
            choices = [str(choice) for choice in choices]
        self.declaration = DeclareLaunchArgument(
            name=name, default_value=str(default_value), choices=choices
        )

    def value(self, context: LaunchContext) -> str | bool | int | float:
        string_value = self.configuration.perform(context)
        try:
            return ast.literal_eval(string_value)
        except Exception:
            return string_value


def get_package_path(package: str) -> str:
    return get_package_share_directory(package)


def get_lib_path(package: str) -> str:
    package_prefix = get_package_prefix(package) + ""
    return os.path.join(package_prefix, "lib", package)


def get_file_path(package: str, folders: List[str], file: str) -> str:
    package_path = get_package_path(package)
    return os.path.join(package_path, *folders, file)


def get_yaml(file_path: str) -> yaml.YAMLObject:
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def get_robot_description(xacro_path: str, xacro_arguments: dict = None) -> str:
    if xacro_arguments is None:
        xacro_arguments = {}
    robot_description_config = xacro.process_file(xacro_path, mappings=xacro_arguments)
    return {"robot_description": robot_description_config.toxml()}


def spin_node(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        raise e


def spin_executor(executor: Executor) -> None:
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        raise e


def start_on_exit(
    action_to_exit: Action, action_to_start: Action
) -> RegisterEventHandler:
    logger = get_logger(start_on_exit.__name__)

    def on_exit(event: ProcessExited, _context: LaunchContext) -> Action:
        if event.returncode == 0:
            return action_to_start
        else:
            logger.error("Target did not start successfully. Please restart.")

    return RegisterEventHandler(
        OnProcessExit(target_action=action_to_exit, on_exit=on_exit)
    )


def start_actions_in_sequence(actions: list[Action]) -> LaunchDescription:
    sequence = [actions[0]]
    for n in range(len(actions) - 1):
        sequence.append(start_on_exit(actions[n], actions[n + 1]))
    return LaunchDescription(sequence)


def assert_for_message(message_type: type, topic: str, timeout: int) -> bool:
    wait_for_topics = WaitForTopics([(topic, message_type)], timeout)
    received = wait_for_topics.wait()
    wait_for_topics.shutdown()
    assert received, (
        f"No message received of type {message_type.__name__} on topic {topic} within {timeout} seconds."
    )


# TODO this is already in Rolling, but not in Humble. See https://github.com/ros2/rclpy/pull/930
def assert_for_node(
    fully_qualified_node_name: str, singleton_node: Node, timeout: int
) -> bool:
    """
    Wait until node name is present in the system or timeout.
    The node name should be the full name with namespace.

    :param node_name: fully qualified name of the node to wait for.
    :param timeout: seconds to wait for the node to be present. If negative, the function won't timeout.

    :return: True if the node was found, False if timeout.
    """
    if not fully_qualified_node_name.startswith("/"):
        fully_qualified_node_name = f"/{fully_qualified_node_name}"

    start = time.time()
    flag = False
    while time.time() - start < timeout and not flag:
        names_and_namespaces = singleton_node.get_node_names_and_namespaces()
        fully_qualified_node_names = [
            ns + ("" if ns.endswith("/") else "/") + name
            for name, ns in names_and_namespaces
        ]

        flag = fully_qualified_node_name in fully_qualified_node_names
        time.sleep(0.1)
    return flag
