# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from rcdt_utilities.launch_utils import get_file_path


def generate_launch_description() -> LaunchDescription:
    realsense_launch_description = get_file_path(
        "realsense2_camera", ["launch"], "rs_launch.py"
    )
    realsense_node = IncludeLaunchDescription(
        realsense_launch_description,
        launch_arguments={
            "align_depth.enable": "true",
            "enable_sync": "true",
            "enable_rgbd": "true",
            "depth_module.depth_profile": "640x480x30",
            "rgb_camera.color_profile": "640x480x30",
        }.items(),
    )

    return LaunchDescription(
        [
            realsense_node,
        ]
    )
