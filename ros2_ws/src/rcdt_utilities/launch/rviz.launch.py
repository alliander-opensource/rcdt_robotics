# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path
from rcdt_utilities.register import Register

rviz_frame_arg = LaunchArgument("rviz_frame", "world")
rviz_display_config_arg = LaunchArgument("rviz_display_config", "general.rviz")
robot_name_arg = LaunchArgument("robot_name", "")
moveit_package_name_arg = LaunchArgument("moveit_package_name", "")


def launch_setup(context: LaunchContext) -> None:
    """Setup the launch description for RViz.

    Args:
        context (LaunchContext): The launch context.
    """
    rviz_frame = rviz_frame_arg.value(context)
    rviz_display_config = rviz_display_config_arg.value(context)
    robot_name = robot_name_arg.value(context)
    package_name = moveit_package_name_arg.value(context)

    arguments = []
    if rviz_frame:
        arguments.extend(["-f", rviz_frame])
    display_config = get_file_path("rcdt_utilities", ["rviz"], rviz_display_config)
    arguments.extend(["--display-config", display_config])

    parameters = []
    if package_name:
        moveit_config = MoveItConfigsBuilder(robot_name, package_name=package_name)
        moveit_config = moveit_config.to_moveit_configs()
        parameters.append(moveit_config.robot_description)
        parameters.append(moveit_config.robot_description_semantic)
        parameters.append(moveit_config.robot_description_kinematics)
        parameters.append(moveit_config.planning_pipelines)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=arguments,
        parameters=parameters,
    )

    return [
        Register.on_log(rviz, "OpenGl version:", context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for RViz.

    Returns:
        LaunchDescription: The launch description for RViz.
    """
    return LaunchDescription(
        [
            rviz_frame_arg.declaration,
            rviz_display_config_arg.declaration,
            robot_name_arg.declaration,
            moveit_package_name_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
