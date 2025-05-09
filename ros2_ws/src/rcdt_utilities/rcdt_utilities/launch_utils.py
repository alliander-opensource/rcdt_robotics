# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import ast
import os
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
from launch_testing_examples.check_multiple_nodes_launch_test import WaitForNodes
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


def assert_for_nodes(node_list: List[str], timeout: int) -> bool:
    """
    Assert that all specified ROS 2 nodes are available within the given timeout.

    Args:
        node_list (List[str]): A list of node names to wait for.
        timeout (int): Timeout duration in seconds.

    Returns:
        bool: True if all nodes were found, otherwise raises AssertionError.
    """
    wait_for_nodes_1 = WaitForNodes(node_list, timeout=timeout)
    assert wait_for_nodes_1.wait()
    assert wait_for_nodes_1.get_nodes_not_found() == set()
    wait_for_nodes_1.shutdown()
