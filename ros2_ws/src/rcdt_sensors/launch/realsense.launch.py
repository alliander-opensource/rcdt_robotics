# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import LaunchArgument
from rcdt_utilities.register import Register

use_sim_arg = LaunchArgument("simulation", True, [True, False])
namespace_arg = LaunchArgument("namespace", "realsense")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the realsense camera.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed.
    """
    use_sim = use_sim_arg.bool_value(context)
    namespace = namespace_arg.string_value(context)

    if use_sim:
        convert_32FC1_to_16UC1_node = Node(  # noqa: N806
            package="rcdt_sensors",
            executable="convert_32FC1_to_16UC1.py",
            namespace=namespace,
        )
        combine_camera_topics_node = Node(
            package="rcdt_sensors",
            executable="combine_camera_topics.py",
            namespace=namespace,
        )
        realsense = LaunchDescription(
            [
                Register.on_start(convert_32FC1_to_16UC1_node, context),
                Register.on_start(combine_camera_topics_node, context),
            ]
        )
    else:
        realsense2_camera_node = Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace=namespace,
            parameters=[
                {
                    "align_depth.enable": True,
                    "enable_sync": True,
                    "enable_rgbd": True,
                    "depth_module.depth_profile": "640x480x30",
                    "rgb_camera.color_profile": "640x480x30",
                }
            ],
        )
        realsense = LaunchDescription(
            [Register.on_start(realsense2_camera_node, context)]
        )

    return [realsense]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the realsense camera.

    Returns:
        LaunchDescription: The launch description for the realsense camera.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            namespace_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
