# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import contextlib
import os

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
WAIT: int = 3


class LaunchArgument:
    """A class to handle launch arguments in ROS 2 launch files.

    This class allows you to declare a launch argument with a default value and optional choices.
    It also provides a method to retrieve the value of the argument in a launch context.
    """

    def __init__(
        self,
        name: str,
        default_value: str | bool | int | float,
        choices: list | None = None,
        min_value: float | None = None,
        max_value: float | None = None,
    ) -> None:
        """Initializes a LaunchArgument instance.

        Args:
            name (str): The name of the launch argument.
            default_value (str | bool | int | float): The default value of the launch argument.
            choices (list | None): A list of valid choices for the launch argument. Defaults to None.
            min_value (float | None): The minimum value for the launch argument if it is a number. Defaults to None.
            max_value (float | None): The maximum value for the launch argument if it is a number. Defaults to None.
        """
        self.configuration = LaunchConfiguration(name)
        self.min_value = min_value
        self.max_value = max_value
        if choices is not None:
            choices = [str(choice) for choice in choices]
            self.declaration = DeclareLaunchArgument(
                name=name, default_value=str(default_value), choices=choices
            )
        else:
            self.declaration = DeclareLaunchArgument(
                name=name, default_value=str(default_value)
            )

    def string_value(self, context: LaunchContext) -> str:
        """Retrieve the string value of the launch argument in a given context.

        Args:
            context (LaunchContext): The launch context in which to evaluate the argument.

        Returns:
            str: The string value of the launch argument.
        """
        return self.configuration.perform(context)

    def bool_value(self, context: LaunchContext) -> bool:
        """Retrieve the boolean value of the launch argument in a given context.

        Args:
            context (LaunchContext): The launch context in which to evaluate the argument.

        Raises:
            TypeError: If the string value cannot be interpreted as a boolean.

        Returns:
            bool: The boolean value of the launch argument.
        """
        string_value = self.string_value(context)
        if string_value in {"True", "true"}:
            return True
        elif string_value in {"False", "false"}:
            return False
        else:
            raise TypeError

    def int_value(self, context: LaunchContext) -> int:
        """Retrieve the integer value of the launch argument in a given context.

        Args:
            context (LaunchContext): The launch context in which to evaluate the argument.

        Returns:
            int: The integer value of the launch argument.
        """
        string_value = self.string_value(context)
        return int(string_value)

    def float_value(self, context: LaunchContext) -> float:
        """Retrieve the float value of the launch argument in a given context.

        Args:
            context (LaunchContext): The launch context in which to evaluate the argument.

        Raises:
            RuntimeError: If the float value is outside the specified min or max range.

        Returns:
            float: The float value of the launch argument.
        """
        string_value = self.string_value(context)
        float_value = float(string_value)
        if self.min_value is not None and float_value < self.min_value:
            raise RuntimeError(
                f"'Value must be ≥ {self.min_value}, but got {float_value}"
            )
        if self.max_value is not None and float_value > self.max_value:
            raise RuntimeError(
                f"'Value must be ≤ {self.max_value}, but got {float_value}"
            )
        return float_value


class LaunchPosition:
    """Provide conversions between position strings (used in launch files) and positions lists for calculations."""

    def __init__(self, position: list | str) -> None:
        """Initialize the LaunchPosition with a position.

        Args:
            position (list | str): The position as a list of floats or a comma-separated string.
        """
        if isinstance(position, str):
            position = [float(axis) for axis in position.split(",")]
        self.position = position

    @property
    def string(self) -> str:
        """Get the position as a comma-separated string, suitable to pass as launch argument.

        Returns:
            str: The position as a comma-separated string.
        """
        return ",".join(map(str, self.position))

    def absolute(self, relative_position: list) -> str:
        """Convert a relative position to an absolute position.

        Args:
            relative_position (list): The relative position to convert.

        Returns:
            str: The absolute position as a comma-separated string.
        """
        absolute = [
            sum(axis) for axis in zip(self.position, relative_position, strict=False)
        ]
        return ",".join(map(str, absolute))


def get_package_path(package: str) -> str:
    """Retrieve the share directory path of a ROS 2 package.

    Args:
        package (str): The name of the ROS 2 package.

    Returns:
        str: The path to the package's share directory.
    """
    return get_package_share_directory(package)


def get_lib_path(package: str) -> str:
    """Retrieve the library directory path of a ROS 2 package.

    Args:
        package (str): The name of the ROS 2 package.

    Returns:
        str: The path to the package's library directory.
    """
    package_prefix = get_package_prefix(package) + ""
    return os.path.join(package_prefix, "lib", package)


def get_file_path(package: str, folders: list[str], file: str) -> str:
    """Construct a file path within a ROS 2 package.

    Args:
        package (str): The name of the ROS 2 package.
        folders (list[str]): A list of folder names leading to the file.
        file (str): The name of the file.

    Returns:
        str: The full path to the specified file within the package.
    """
    package_path = get_package_path(package)
    return os.path.join(package_path, *folders, file)


def get_yaml(file_path: str) -> dict:
    """Load a YAML file and return its contents as a dictionary.

    Args:
        file_path (str): The path to the YAML file.

    Returns:
        dict: The contents of the YAML file as a dictionary.
    """
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return {}


def get_robot_description(xacro_path: str, xacro_arguments: dict | None = None) -> dict:
    """Process a Xacro file to generate the robot description.

    Args:
        xacro_path (str): The path to the Xacro file.
        xacro_arguments (dict | None): A dictionary of arguments to pass to the Xacro processor. Defaults to None.

    Returns:
        dict: A dictionary containing the robot description in XML format.
    """
    if xacro_arguments is None:
        xacro_arguments = {}
    robot_description_config = xacro.process_file(xacro_path, mappings=xacro_arguments)
    return {"robot_description": robot_description_config.toxml()}


def spin_node(node: Node) -> None:
    """Spin a ROS 2 node to process callbacks and keep it alive.

    Args:
        node (Node): The ROS 2 node to spin.
    """
    with contextlib.suppress(KeyboardInterrupt):
        rclpy.spin(node)


def spin_executor(executor: Executor) -> None:
    """Spin a ROS 2 executor to process callbacks from multiple nodes.

    Args:
        executor (Executor): The ROS 2 executor to spin.
    """
    with contextlib.suppress(KeyboardInterrupt):
        executor.spin()


def assert_for_message(message_type: type, topic: str, timeout: int) -> None:
    """Assert that a message of a specific type is received on a given topic within a timeout period.

    Args:
        message_type (type): The type of the message to wait for.
        topic (str): The topic to listen to.
        timeout (int): The maximum time in seconds to wait for the message.
    """
    wait_for_topics = WaitForTopics([(topic, message_type)], timeout)
    received = wait_for_topics.wait()
    wait_for_topics.shutdown()
    assert received, (
        f"No message received of type {message_type.__name__} on topic {topic} within {timeout} seconds."
    )
