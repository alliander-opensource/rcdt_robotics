# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from alliander_utilities.config_objects import Gripper
from alliander_utilities.launch_argument import LaunchArgument
from alliander_utilities.launch_utils import SKIP, state_publisher_node, static_tf_node
from alliander_utilities.register import Register, RegisteredLaunchDescription
from alliander_utilities.ros_utils import get_file_path
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction

platform_arg = LaunchArgument("platform_config", "")


def launch_setup(context: LaunchContext) -> list:
    """The launch setup.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: The actions to start.
    """
    robotiq_config = Gripper.from_str(platform_arg.string_value(context))

    state_publisher = state_publisher_node(
        namespace=robotiq_config.namespace,
        platform="robotiq",
        xacro="3f-gripper.urdf.xacro",
        xacro_arguments={
            "namespace": robotiq_config.namespace,
            "parent": "" if robotiq_config.parent.link else "world",
            "use_sim": str(robotiq_config.simulation),
        },
    )

    parent = robotiq_config.parent
    static_tf = static_tf_node(
        parent_frame=f"{parent.namespace}/{parent.link}" if parent.link else "map",
        child_frame=f"{robotiq_config.namespace}/{parent.connects_to}",
        position=robotiq_config.position,
        orientation=robotiq_config.orientation,
    )

    hardware = RegisteredLaunchDescription(
        get_file_path("alliander_robotiq", ["launch"], "hardware.launch.py"),
        {"platform_config": robotiq_config.to_str()},
    )

    controllers = RegisteredLaunchDescription(
        get_file_path("alliander_robotiq", ["launch"], "controllers.launch.py"),
        launch_arguments={"platform_config": robotiq_config.to_str()},
    )

    return [
        Register.on_start(state_publisher, context),
        Register.on_start(static_tf, context),
        Register.group(hardware, context) if not robotiq_config.simulation else SKIP,
        Register.group(controllers, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description.

    Returns:
        LaunchDescription: The launch description.
    """
    return LaunchDescription(
        [
            platform_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
