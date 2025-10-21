# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.register import Register


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the node.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    graspnet_node = Node(
        package="rcdt_grasping",
        executable="generate_grasp.py",
    )

    return [
        Register.on_log(
            graspnet_node, "GraspNet model initialized and weights loaded", context
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the grasping node.

    Returns:
        LaunchDescription: The launch description containing the grasping node.
    """
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
