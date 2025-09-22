# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import os.path

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from rcdt_launch.rviz import Rviz
from rcdt_utilities.register import Register


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for RViz.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    arguments = []
    parameters = []
    remappings = []

    file_name = "/tmp/rviz.rviz"
    if os.path.isfile(file_name):
        arguments.extend(["--display-config", file_name])

    if Rviz.load_motion_planning_plugin:
        for ns in Rviz.moveit_namespaces:
            moveit_config = MoveItConfigsBuilder(
                "franka", package_name="rcdt_franka_moveit_config"
            )
            moveit_config = moveit_config.to_moveit_configs()

            kinematics = moveit_config.robot_description_kinematics[
                "robot_description_kinematics"
            ]
            parameters.append({f"{ns}_robot_description_kinematics": kinematics})
            parameters.append(moveit_config.planning_pipelines)
            parameters.append(moveit_config.pilz_cartesian_limits)

            remappings.extend(
                [
                    (
                        f"/{ns}_robot_description",
                        f"/{ns}/robot_description",
                    ),
                    (
                        f"/{ns}_robot_description_semantic",
                        f"/{ns}/robot_description_semantic",
                    ),
                ]
            )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=arguments,
        parameters=parameters,
        remappings=remappings,
    )

    return [
        Register.on_log(rviz, "OpenGl version:", context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for RViz.

    Returns:
        LaunchDescription: The launch description for RViz.
    """
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
