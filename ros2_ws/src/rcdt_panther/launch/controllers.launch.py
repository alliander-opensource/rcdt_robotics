# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path

controllers_config = get_file_path("rcdt_panther", ["config"], "ros_controller.yaml")

diff_drive_controller = Node(
    package="controller_manager",
    executable="spawner",
    name="diff_drive_controller",
    arguments=[
        "diff_drive_controller",
        "-t",
        "diff_drive_controller/DiffDriveController",
        "-p",
        controllers_config,
    ],
)


def generate_launch_description() -> None:
    return LaunchDescription(
        [
            diff_drive_controller,
        ]
    )
