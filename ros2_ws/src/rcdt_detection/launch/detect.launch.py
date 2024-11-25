# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

from rcdt_utilities.launch_utils import get_file_path


def generate_launch_description() -> LaunchDescription:
    detection_node = Node(
        package="rcdt_detection",
        executable="object_detection.py",
    )

    realsense_launch_description = get_file_path(
        "realsense2_camera", ["launch"], "rs_launch.py"
    )
    realsense_node = IncludeLaunchDescription(
        realsense_launch_description,
        launch_arguments={
            "align_depth.enable": "true",
            "enable_sync": "true",
            "enable_rgbd": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            detection_node,
            realsense_node,
        ]
    )
