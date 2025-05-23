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
from launch import LaunchContext, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_testing_ros.wait_for_topics import WaitForTopics
from rclpy.executors import Executor
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


def assert_for_message(message_type: type, topic: str, timeout: int) -> bool:
    wait_for_topics = WaitForTopics([(topic, message_type)], timeout)
    received = wait_for_topics.wait()
    wait_for_topics.shutdown()
    assert received, (
        f"No message received of type {message_type.__name__} on topic {topic} within {timeout} seconds."
    )
