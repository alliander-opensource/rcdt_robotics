# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node

drive_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["drive_controller"],
    namespace="panther",
)


def generate_launch_description() -> None:
    return LaunchDescription(
        [
            drive_controller_spawner,
        ]
    )
