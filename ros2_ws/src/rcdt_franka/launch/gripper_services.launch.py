# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.register import Register


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Franka gripper services.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    namespace = "franka"
    gripper = Node(package="rcdt_franka", executable="gripper", namespace=namespace)

    return [
        Register.on_start(gripper, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Franka gripper services.

    Returns:
        LaunchDescription: The launch description containing the gripper services.
    """
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
