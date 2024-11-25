# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter

from rcdt_utilities.launch_utils import (
    get_file_path,
    get_robot_description,
    LaunchArgument,
)

use_rviz_arg = LaunchArgument("rviz", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    use_rviz = use_rviz_arg.value(context)

    xacro_path = get_file_path("panther_description", ["urdf"], "panther.urdf.xacro")
    xacro_arguments = {"use_sim": "true"}
    robot_description = get_robot_description(xacro_path, xacro_arguments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    robot = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py")
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world",
        arguments=["--frame-id", "world", "--child-frame-id", "odom"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    controllers = IncludeLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "controllers.launch.py"),
    )

    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py")
    )

    joy = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[
            {"sticky_buttons": True},
        ],
    )

    joy_to_twist_node = Node(
        package="rcdt_utilities",
        executable="joy_to_twist_node.py",
        parameters=[
            {"pub_topic": "/diff_drive_controller/cmd_vel"},
            {"config_pkg": "rcdt_panther"},
        ],
    )

    skip = LaunchDescriptionEntity()
    return [
        SetParameter(name="use_sim_time", value=True),
        robot_state_publisher,
        robot,
        static_transform_publisher,
        joint_state_broadcaster,
        controllers,
        rviz if use_rviz else skip,
        joy,
        joy_to_twist_node,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_rviz_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
