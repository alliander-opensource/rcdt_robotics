# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node

franka_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
    namespace="franka",
)

panther_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
    namespace="panther",
)


def generate_launch_description() -> None:
    return LaunchDescription(
        [
            franka_state_broadcaster_spawner,
            panther_state_broadcaster_spawner,
        ]
    )
