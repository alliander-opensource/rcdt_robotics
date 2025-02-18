# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    joy = Node(
        package="joy",
        executable="joy_node",
    )

    joy_to_panther = Node(
        package="rcdt_panther",
        executable="joy_to_panther.py",
    )

    return LaunchDescription(
        [
            joy,
            joy_to_panther,
        ]
    )
