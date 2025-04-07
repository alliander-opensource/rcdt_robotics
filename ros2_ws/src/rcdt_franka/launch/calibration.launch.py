# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import LaunchArgument

use_sim_arg = LaunchArgument("simulation", True, [True, False])


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    calibrate = Node(package="rcdt_franka", executable="calibrate.py")

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        remappings=[
            ("/image_rect", "/camera/camera/color/image_raw"),
            ("/camera_info", "/camera/camera/color/camera_info"),
        ],
        parameters=[
            {
                "size": 0.064,
            }
        ],
        output="screen",
    )

    handeye_server = Node(
        package="easy_handeye2",
        executable="handeye_server",
        name="handeye_server",
        parameters=[
            {
                "name": "calibration",
                "calibration_type": "eye_in_hand",
                "tracking_base_frame": "fr3_link0",
                "tracking_marker_frame": "tag36h11:0",
                "robot_base_frame": "realsense_link",
                "robot_effector_frame": "fr3_hand",
            }
        ],
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        calibrate,
        apriltag_node,
        handeye_server,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
