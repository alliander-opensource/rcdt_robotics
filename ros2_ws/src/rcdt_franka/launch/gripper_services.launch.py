# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.register import Register


def launch_setup(context: LaunchContext) -> None:
    """Setup the launch description for the Franka gripper services.

    Args:
        context (LaunchContext): The launch context.
    """
    namespace = "franka"

    open_gripper = Node(
        package="rcdt_franka",
        executable="open_gripper.py",
        namespace=namespace,
    )
    close_gripper = Node(
        package="rcdt_franka",
        executable="close_gripper.py",
        namespace=namespace,
    )

    return [
        Register.on_start(open_gripper, context),
        Register.on_start(close_gripper, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Franka gripper services.

    Returns:
        LaunchDescription: The launch description containing the gripper services.
    """
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
