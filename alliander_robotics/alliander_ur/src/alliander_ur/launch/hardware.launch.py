# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from alliander_utilities.config_objects import Arm
from alliander_utilities.launch_argument import LaunchArgument
from alliander_utilities.register import Register
from alliander_utilities.ros_utils import get_file_path
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, Shutdown
from launch_ros.actions import Node

platform_arg = LaunchArgument("platform_config", "")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Franka robot controllers.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    arm_config = Arm.from_str(platform_arg.string_value(context))

    controllers = get_file_path("alliander_ur", ["config"], "controllers.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers,
            {"arm_id": "fr3"},
        ],
        namespace=arm_config.namespace,
        on_exit=Shutdown(),
    )

    return [
        Register.on_log(
            ros2_control_node, "Calibration checked successfully.", context
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Franka robot controllers.

    Returns:
        LaunchDescription: The launch description containing the Franka controllers.
    """
    return LaunchDescription(
        [
            platform_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
