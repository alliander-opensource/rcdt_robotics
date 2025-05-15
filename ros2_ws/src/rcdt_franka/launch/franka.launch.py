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
from rcdt_utilities.register import Register

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
world_arg = LaunchArgument("world", "table_with_1_brick.sdf")
use_realsense_arg = LaunchArgument("realsense", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    use_rviz = use_rviz_arg.value(context)
    world = str(world_arg.value(context))
    use_realsense = use_realsense_arg.value(context)

    namespace = "franka"
    ns = f"/{namespace}" if namespace else ""

    core = IncludeLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "core.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "load_gazebo_ui": str(load_gazebo_ui),
            "realsense": str(use_realsense),
            "world": world,
        }.items(),
    )

    controllers = IncludeLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "controllers.launch.py")
    )

    display_config = "franka_general.rviz"
    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={
            "rviz_frame": f"{ns}/fr3_link0",
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
            "namespace": namespace,
        }.items(),
    )

    realsense = IncludeLaunchDescription(
        get_file_path("rcdt_detection", ["launch"], "realsense.launch.py"),
        launch_arguments={"simulation": str(use_sim)}.items(),
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
        parameters=[
            {"sub_topic": f"{ns}/joy"},
            {"pub_topic": f"{ns}/servo_node/delta_twist_cmds"},
            {"config_pkg": "rcdt_franka"},
            {"pub_frame": "fr3_hand"},
        ],
    )

    joy_to_gripper = Node(
        package="rcdt_franka",
        executable="joy_to_gripper.py",
    )

    joystick = LaunchDescription(
        [
            joy,
            joy_topic_manager,
            joy_to_twist_franka,
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

    manipulate_pose = Node(package="rcdt_utilities", executable="manipulate_pose.py")
    action_executor = Node(package="rcdt_actions", executable="action_executor.py")

    wait_for_franka = Node(
        package="rcdt_utilities",
        executable="wait_for_topic.py",
        parameters=[{"topic": f"{ns}/joint_states"}, {"msg_type": "JointState"}],
    )

    launch_description = LaunchDescription(
        [
            controllers,
            moveit,
            open_gripper,
            close_gripper,
            joystick,
            manipulate_pose,
            action_executor,
            realsense if use_realsense else SKIP,
            Register.on_all_started(rviz) if use_rviz else SKIP,
        ]
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        core,
        start_actions_in_sequence([wait_for_franka, launch_description]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            use_rviz_arg.declaration,
            world_arg.declaration,
            use_realsense_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
