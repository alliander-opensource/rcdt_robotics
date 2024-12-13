# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node

from rcdt_utilities.launch_utils import LaunchArgument, get_file_path

use_sim_arg = LaunchArgument("simulation", True, [True, False])


def launch_setup(context: LaunchContext) -> list:
    use_sim = use_sim_arg.value(context)

    if use_sim:
        convert_32FC1_to_16UC1_node = Node(  # noqa: N806
            package="rcdt_sensors",
            executable="convert_32FC1_to_16UC1_node.py",
        )
        combine_camera_topics_node = Node(
            package="rcdt_sensors",
            executable="combine_camera_topics_node.py",
        )
        realsense = LaunchDescription(
            [
                convert_32FC1_to_16UC1_node,
                combine_camera_topics_node,
            ]
        )
    else:
        realsense = IncludeLaunchDescription(
            get_file_path("realsense2_camera", ["launch"], "rs_launch.py"),
            launch_arguments={
                "align_depth.enable": "true",
                "enable_sync": "true",
                "enable_rgbd": "true",
                "depth_module.depth_profile": "640x480x30",
                "rgb_camera.color_profile": "640x480x30",
            }.items(),
        )

    return [realsense]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
