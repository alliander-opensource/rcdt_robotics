# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from alliander_utilities.config_objects import Gripper
from alliander_utilities.launch_argument import LaunchArgument
from alliander_utilities.launch_utils import SKIP
from alliander_utilities.register import Register
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

TIMEOUT = 100

platform_arg = LaunchArgument("platform_config", "")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the controllers.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    gripper_config = Gripper.from_str(platform_arg.string_value(context))
    sim = gripper_config.simulation

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--switch-timeout",
            str(TIMEOUT),
        ],
        namespace=gripper_config.namespace,
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "--switch-timeout",
            str(TIMEOUT),
        ],
        namespace=gripper_config.namespace,
    )

    gripper_controller = Node(
        package="alliander_robotiq",
        executable="gripper_controller.py",
        namespace=gripper_config.namespace,
        parameters=[
            {
                "simulation": gripper_config.simulation,
                "ip": gripper_config.ip_address,
                "port": gripper_config.port,
            }
        ],
    )

    return [
        Register.on_exit(joint_state_broadcaster_spawner, context) if sim else SKIP,
        Register.on_exit(position_controller, context) if sim else SKIP,
        Register.on_log(gripper_controller, "RobotIQ controller initialized.", context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Ewellix lift controllers.

    Returns:
        LaunchDescription: The launch description containing the nodes and actions.
    """
    return LaunchDescription(
        [
            platform_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
