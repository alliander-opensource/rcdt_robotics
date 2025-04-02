# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path

use_sim_arg = LaunchArgument("simulation", True, [True, False])


def launch_setup(context: LaunchContext) -> List:
    use_sim = use_sim_arg.value(context)

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        remappings=[
            ("/image_rect", "/camera/camera/color/image_raw"),
            ("/camera_info", "/camera/camera/color/camera_info"),
        ],
        parameters=[
            {
                "size": 0.16,
            }
        ],
        output="screen",
    )

    calibration = IncludeLaunchDescription(
        get_file_path("easy_handeye2", ["launch"], "calibrate.launch.py"),
        launch_arguments={
            "name": "calibration",
            "calibration_type": "eye_in_hand",
            "robot_base_frame": "fr3_link0",
            "robot_effector_frame": "fr3_hand",
            "tracking_base_frame": "realsense_link",
            "tracking_marker_frame": "tag36h11:0",
        }.items(),
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        apriltag_node,
        calibration,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
