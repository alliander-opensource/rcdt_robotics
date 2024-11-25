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

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
use_realsense_arg = LaunchArgument("realsense", False, [True, False])
moveit_mode_arg = LaunchArgument("moveit", "off", ["node", "rviz", "servo", "off"])


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    use_rviz = use_rviz_arg.value(context)
    use_realsense = use_realsense_arg.value(context)
    moveit_mode = moveit_mode_arg.value(context)

    xacro_path = get_file_path("rcdt_franka", ["urdf"], "franka.urdf.xacro")
    xacro_arguments = {}
    xacro_arguments["robot_ip"] = "172.16.0.2"
    xacro_arguments["gazebo"] = "true" if use_sim else "false"
    xacro_arguments["load_realsense"] = "true" if use_realsense else "false"
    robot_description = get_robot_description(xacro_path, xacro_arguments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    if use_sim:
        robot = IncludeLaunchDescription(
            get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py"),
            launch_arguments={
                "realsense": str(use_realsense),
                "load_gazebo_ui": str(load_gazebo_ui),
            }.items(),
        )
    else:
        robot = IncludeLaunchDescription(
            get_file_path("rcdt_franka", ["launch"], "robot.launch.py")
        )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world",
        arguments=["--frame-id", "world", "--child-frame-id", "base"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    controllers = IncludeLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "controllers.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "arm_controller": "fr3_arm_controller",
            "gripper_controller": "fr3_gripper",
        }.items(),
    )

    rviz_launch_arguments = {
        "moveit": moveit_mode,
        "moveit_package_name": "rcdt_franka_moveit_config",
    }
    if use_realsense:
        display_config = get_file_path("rcdt_utilities", ["rviz"], "realsense.rviz")
        rviz_launch_arguments["rviz_display_config"] = display_config
    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments=rviz_launch_arguments.items(),
    )

    moveit = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "moveit.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "moveit": moveit_mode,
            "moveit_package_name": "rcdt_franka_moveit_config",
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
        executable="joy_topic_manager_node.py",
    )

    joy_to_twist_franka = Node(
        package="rcdt_utilities",
        executable="joy_to_twist_node.py",
        namespace="franka",
        parameters=[
            {"sub_topic": "/franka/joy"},
            {"pub_topic": "/servo_node/delta_twist_cmds"},
            {"config_pkg": "rcdt_franka"},
            {"pub_frame": "fr3_hand"},
        ],
    )

    joy_to_gripper = Node(
        package="rcdt_franka",
        executable="joy_to_gripper_node.py",
    )

    skip = LaunchDescriptionEntity()
    return [
        SetParameter(name="use_sim_time", value=use_sim),
        robot_state_publisher,
        robot,
        static_transform_publisher,
        joint_state_broadcaster,
        controllers,
        rviz if use_rviz else skip,
        moveit if moveit_mode != "off" else skip,
        joy,
        joy_topic_manager if moveit_mode == "servo" else skip,
        joy_to_twist_franka if moveit_mode == "servo" else skip,
        joy_to_gripper,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            use_rviz_arg.declaration,
            use_realsense_arg.declaration,
            moveit_mode_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
