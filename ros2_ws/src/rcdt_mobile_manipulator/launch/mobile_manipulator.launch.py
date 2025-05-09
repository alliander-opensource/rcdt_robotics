# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import (
    SKIP,
    LaunchArgument,
    get_file_path,
    start_actions_in_sequence,
)

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "empty_camera.sdf")
use_rviz_arg = LaunchArgument("rviz", True, [True, False])


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    use_rviz = use_rviz_arg.value(context)
    world = str(world_arg.value(context))

    core = IncludeLaunchDescription(
        get_file_path("rcdt_mobile_manipulator", ["launch"], "core.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "load_gazebo_ui": str(load_gazebo_ui),
            "world": world,
        }.items(),
    )

    franka_controllers = IncludeLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "controllers.launch.py"),
    )

    panther_controllers = IncludeLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "controllers.launch.py"),
    )

    display_config = "mobile_manipulator_general.rviz"
    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={
            "rviz_frame": "panther/odom",
            "robot_name": "fr3",
            "moveit_package_name": "rcdt_franka_moveit_config",
            "rviz_display_config": display_config,
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        get_file_path("rcdt_moveit", ["launch"], "moveit.launch.py"),
        launch_arguments={
            "robot_name": "fr3",
            "moveit_package_name": "rcdt_franka_moveit_config",
            "servo_params_package": "rcdt_franka",
            "namespace": "franka",
        }.items(),
    )

    joy = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[
            {"sticky_buttons": True},
        ],
    )

    joy_topic_manager = Node(
        package="rcdt_mobile_manipulator",
        executable="joy_topic_manager.py",
    )

    joy_to_twist_franka = Node(
        package="rcdt_utilities",
        executable="joy_to_twist.py",
        namespace="franka",
        parameters=[
            {"sub_topic": "/franka/joy"},
            {"pub_topic": "/franka/servo_node/delta_twist_cmds"},
            {"config_pkg": "rcdt_franka"},
            {"pub_frame": "fr3_hand"},
        ],
    )

    joy_to_twist_panther = Node(
        package="rcdt_utilities",
        executable="joy_to_twist.py",
        parameters=[
            {"sub_topic": "/panther/joy"},
            {"pub_topic": "/panther/cmd_vel"},
            {"config_pkg": "rcdt_panther"},
            {"stamped": False},
        ],
    )

    joy_to_gripper = Node(
        package="rcdt_franka",
        executable="joy_to_gripper.py",
        parameters=[{"sub_topic": "/franka/joy"}],
    )

    joystick = LaunchDescription(
        [
            joy,
            joy_topic_manager,
            joy_to_twist_franka,
            joy_to_twist_panther,
            joy_to_gripper,
        ]
    )

    open_gripper = Node(
        package="rcdt_franka",
        executable="open_gripper.py",
    )

    close_gripper = Node(
        package="rcdt_franka",
        executable="close_gripper.py",
    )

    wait_for_franka = Node(
        package="rcdt_utilities",
        executable="wait_for_topic.py",
        parameters=[{"topic": "/franka/joint_states"}, {"msg_type": "JointState"}],
    )

    wait_for_panther = Node(
        package="rcdt_utilities",
        executable="wait_for_topic.py",
        parameters=[{"topic": "/panther/joint_states"}, {"msg_type": "JointState"}],
    )

    launch_description = LaunchDescription(
        [
            franka_controllers,
            panther_controllers,
            open_gripper,
            close_gripper,
            joystick,
            moveit,
            rviz if use_rviz else SKIP,
        ]
    )

    return [
        SetParameter(name="use_sim_time", value=True),
        core,
        start_actions_in_sequence(
            [wait_for_franka, wait_for_panther, launch_description]
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            world_arg.declaration,
            use_rviz_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
