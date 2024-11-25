# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List, Literal
import os
import yaml
import xacro
import ast

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder


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


def get_moveit_parameters(
    robot_name: str,
    package_name: str,
    mode: Literal["off", "rviz", "sevo", "node"] = "off",
) -> dict:
    moveit_config = MoveItConfigsBuilder(robot_name, package_name=package_name)
    match mode:
        case "node":
            moveit_config.trajectory_execution(
                get_file_path(package_name, ["config"], "moveit_controllers.yaml")
            )
            moveit_config.moveit_cpp(
                get_file_path(package_name, ["config"], "planning_pipeline.yaml")
            )
    return moveit_config.to_dict()
