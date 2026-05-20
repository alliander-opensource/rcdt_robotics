# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from alliander_utilities.config_objects import Camera
from alliander_utilities.launch_argument import LaunchArgument
from alliander_utilities.launch_utils import SKIP, state_publisher_node, static_tf_node
from alliander_utilities.register import Register, RegisteredLaunchDescription
from alliander_utilities.ros_utils import get_file_path
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

platform_arg = LaunchArgument("platform_config", "")


def launch_setup(context: LaunchContext) -> list:
    """The launch setup.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: The actions to start.
    """
    namespace = "openVLA"

    vla_node = Node(
        package="alliander_openvla",
        executable="vla_node.py",
        name="vla_node",
        namespace=namespace,
        # parameters=[
        #     {
        #         "enable_visualization": LaunchConfiguration(
        #             "enable_visualization"
        #         ).perform(context),
        #     }
        # ],
        output="screen",
    )

    return [Register.on_start(vla_node, context)]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the OpenVLA Node.

    Returns:
        LaunchDescription: The launch description for the OpenVLA Node.
    """
    enable_viz_arg = DeclareLaunchArgument(
        "enable_visualization",
        default_value="true",
        description="Enable visualization outputs",
    )

    return LaunchDescription(
        [
            enable_viz_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
