# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import (
    LaunchArgument,
    get_file_path,
    get_moveit_parameters,
)

rviz_frame_arg = LaunchArgument("rviz_frame", "world")
moveit_mode_arg = LaunchArgument("moveit", "off", ["node", "rviz", "servo", "off"])
moveit_package_name_arg = LaunchArgument("moveit_package_name", "")


def launch_setup(context: LaunchContext) -> None:
    rviz_frame = rviz_frame_arg.value(context)
    moveit_mode = moveit_mode_arg.value(context)
    moveit_package_name = moveit_package_name_arg.value(context)

    arguments = []
    if rviz_frame != "":
        arguments.extend(["-f", rviz_frame])
    display_config = get_file_path("rcdt_utilities", ["rviz"], "general.rviz")
    match moveit_mode:
        case "rviz":
            display_config = get_file_path("rcdt_utilities", ["rviz"], "moveit.rviz")
        case "node":
            display_config = get_file_path("rcdt_utilities", ["rviz"], "planning.rviz")
    arguments.extend(["--display-config", display_config])

    parameters = []
    if moveit_mode != "off":
        moveit_config = get_moveit_parameters(
            robot_name="fr3",
            package_name=moveit_package_name,
            mode=moveit_mode,
        )
        parameters.append(moveit_config)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=arguments,
        parameters=parameters,
    )

    rviz_controller = Node(
        package="rcdt_utilities",
        executable="rviz_controller_node",
    )

    return [
        rviz,
        rviz_controller,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            rviz_frame_arg.declaration,
            moveit_mode_arg.declaration,
            moveit_package_name_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
