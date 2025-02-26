# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node

joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_state_broadcaster",
        "--controller-manager",
        "controller_manager",
    ],
    namespace="panther",
)

drive_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "drive_controller",
        "--controller-manager",
        "controller_manager",
    ],
    namespace="panther",
)


def generate_launch_description() -> None:
    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            drive_controller_spawner,
        ]
    )
